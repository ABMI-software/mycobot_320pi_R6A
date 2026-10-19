#!/usr/bin/env python3
"""SmolVLA fine-tune of local/mycobot_tri_sort on a Kaggle (or Colab) GPU.

Two modes, read from the JSON config written by the notebook's first cell:
  benchmark  short runs over batch sizes and precisions; prints s/step,
             samples/s, data-loading time, peak GPU memory and projections.
  train      fresh run, or resumes automatically from the newest complete
             checkpoint found under the input root (a previous run's output
             attached as input).

Standard library only: lerobot-train is run from the venv as a subprocess.
Paths come from the config so the same file runs on Kaggle, Colab and in the
gazebo_to_lerobot container (CPU) where it was tested.
"""
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

REPO_ID = 'local/mycobot_tri_sort'
CAMERA_RENAME = {
    'observation.images.top': 'observation.images.camera1',
    'observation.images.right': 'observation.images.camera2',
    'observation.images.left': 'observation.images.camera3',
}
# smolvla_base loads in bfloat16 (447 M of 450 M parameters, 97 M of the 100 M trained).
# A T4 has no bfloat16 tensor cores, and fp16 mixed precision cannot unscale bfloat16
# gradients ("_amp_foreach_non_finite_check_and_unscale_cuda" not implemented for
# 'BFloat16', Kaggle 2026-10-12). Casting the policy to float32 right after LeRobot builds
# it keeps float32 master weights; with use_amp the matmuls then run in float16.
#
# Gradient accumulation (SMOLVLA_MICRO_BATCH > 0): lerobot-train still loads and preprocesses
# the whole --batch_size batch, and update_policy is replaced by one that runs forward and
# backward on micro-batches of that size, then clips and steps once. Steps, the LR schedule,
# checkpoints and logs all count whole batches, as without accumulation.
#
# The fp16 loss scaler: LeRobot saves neither it nor restores it, so after a resume its scale
# restarted at 65536 and the first updates could be skipped. The launcher writes its state to
# training_state/grad_scaler.json with every checkpoint (before LeRobot's own files, so the last
# of those, scheduler_state.json, still marks a complete checkpoint) and restores it on resume.
FP32_LAUNCHER = '''import copy
import json
import os
import sys
import time
from collections import Counter
from contextlib import nullcontext

from pathlib import Path

import torch
import lerobot.scripts.lerobot_train as lt

_make_policy = lt.make_policy
_save_checkpoint = lt.save_checkpoint
_load_training_state = lt.load_training_state
ACCELERATOR = None
SCALER_STATE = None
SCALER_FILE = "grad_scaler.json"


FREEZE = os.environ.get("SMOLVLA_FREEZE") or None
MICRO_BATCH = int(os.environ.get("SMOLVLA_MICRO_BATCH") or 0)
ACCUM_CHECK = os.environ.get("SMOLVLA_ACCUM_CHECK") or None


def group(name):
    if "vlm_with_expert.vlm." not in name:
        return "expert and projections"
    if "vision_model" in name:
        return "vision"
    if "connector" in name:
        return "connector"
    return "language"


def make_policy_fp32(*args, **kwargs):
    policy = _make_policy(*args, **kwargs).float()
    if FREEZE == "runC":
        # Language layers and lm_head frozen; gradients still flow through them to the vision encoder.
        for name, p in policy.named_parameters():
            if group(name) == "language":
                p.requires_grad = False
    trained = Counter(str(p.dtype) for p in policy.parameters() if p.requires_grad)
    by_group = Counter()
    for name, p in policy.named_parameters():
        if p.requires_grad:
            by_group[group(name)] += p.numel()
    print(f"[fp32 launcher] policy cast to float32; freeze={FREEZE}; trainable tensors by dtype: {dict(trained)}; "
          f"trainable parameters by group (M): { {k: round(v / 1e6, 1) for k, v in sorted(by_group.items())} }, "
          f"total {sum(by_group.values()) / 1e6:.1f} M", flush=True)
    return policy


def split_batch(batch, size):
    n = len(batch["action"])
    for start in range(0, n, size):
        yield {k: v[start:start + size]
               if (torch.is_tensor(v) and v.dim() > 0 and len(v) == n) or (isinstance(v, list) and len(v) == n)
               else v for k, v in batch.items()}


def update_policy_accumulated(train_metrics, policy, batch, optimizer, grad_clip_norm, accelerator,
                              lr_scheduler=None, lock=None, sample_weighter=None):
    if sample_weighter is not None or accelerator.num_processes > 1:
        raise SystemExit("gradient accumulation: one GPU and no sample weighting only")
    start_time = time.perf_counter()
    policy.train()
    if torch.cuda.is_available():
        torch.cuda.reset_peak_memory_stats()
    chunks = list(split_batch(batch, MICRO_BATCH))
    # SmolVLA's loss is a mean over the batch's unpadded action steps, so each micro-batch is
    # weighted by its share of them: the summed gradient is then the whole batch's.
    if "action_is_pad" in batch:
        valid = [float((~c["action_is_pad"]).sum()) for c in chunks]
    else:
        valid = [float(len(c["action"])) for c in chunks]
    total = sum(valid) or 1.0
    loss_value, outputs = 0.0, {}
    for chunk, v in zip(chunks, valid):
        with accelerator.autocast():
            loss, out = policy.forward(chunk)
        accelerator.backward(loss * (v / total))
        loss_value += loss.item() * v / total
        for k, x in (out or {}).items():
            if isinstance(x, (int, float)):
                outputs[k] = outputs.get(k, 0.0) + x * v / total
    if grad_clip_norm > 0:
        grad_norm = accelerator.clip_grad_norm_(policy.parameters(), grad_clip_norm)
    else:
        grad_norm = torch.nn.utils.clip_grad_norm_(policy.parameters(), float("inf"), error_if_nonfinite=False)
    with lock if lock is not None else nullcontext():
        optimizer.step()
    if getattr(optimizer, "step_was_skipped", False):
        print(f"[fp32 launcher] update skipped by the fp16 loss scaler (inf/nan gradients), "
              f"scale now {accelerator.scaler.get_scale() if accelerator.scaler else None}", flush=True)
    optimizer.zero_grad()
    if lr_scheduler is not None:
        lr_scheduler.step()
    if lt.has_method(accelerator.unwrap_model(policy, keep_fp32_wrapper=True), "update"):
        accelerator.unwrap_model(policy, keep_fp32_wrapper=True).update()
    train_metrics.loss = loss_value
    train_metrics.grad_norm = grad_norm.item()
    train_metrics.lr = optimizer.param_groups[0]["lr"]
    train_metrics.update_s = time.perf_counter() - start_time
    if torch.cuda.is_available():
        train_metrics.gpu_mem_gb = torch.cuda.max_memory_allocated() / (1024**3)
    if outputs:
        train_metrics.update_metrics(outputs)
    return train_metrics, outputs


class FixedDraws:
    """Replays the noise and flow-matching times of the first (whole-batch) pass to the later ones,
    micro-batch by micro-batch, so every pass of the accumulation check sees the same draws."""

    def __init__(self, model):
        self.model, self.noise, self.time = model, None, None
        self.sample_noise, self.sample_time = model.sample_noise, model.sample_time
        model.sample_noise, model.sample_time = self.draw_noise, self.draw_time

    def rewind(self):
        self.at_noise = self.at_time = 0

    def draw_noise(self, shape, device):
        if self.noise is None:
            self.noise = self.sample_noise(shape, device)
        out = self.noise[self.at_noise:self.at_noise + shape[0]]
        self.at_noise += shape[0]
        return out

    def draw_time(self, bsize, device):
        if self.time is None:
            self.time = self.sample_time(bsize, device)
        out = self.time[self.at_time:self.at_time + bsize]
        self.at_time += bsize
        return out


def to_cpu(x):
    if torch.is_tensor(x):
        return x.detach().to("cpu", copy=True)
    if isinstance(x, dict):
        return {k: to_cpu(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return type(x)(to_cpu(v) for v in x)
    return copy.deepcopy(x)


def rel(a, b, names):
    num = sum(float(((a[n] - b[n]).double() ** 2).sum()) for n in names)
    den = sum(float((b[n].double() ** 2).sum()) for n in names)
    return (num / den) ** 0.5 if den > 0 else float("nan")


CHECK_ROWS = []


def update_policy_checked(train_metrics, policy, batch, optimizer, grad_clip_norm, accelerator,
                          lr_scheduler=None, lock=None, sample_weighter=None):
    """Accumulation check (SMOLVLA_ACCUM_CHECK=<report .json>): from the same weights, optimizer,
    scheduler and loss-scaler state, the same batch and the same noise and times, one update on
    the whole batch (twice: the second gives the run-to-run floor), then one accumulated over
    micro-batches of MICRO_BATCH, which training continues from. Compares the gradients the
    optimizer receives (unscaled, after clipping), the pre-clip gradient norm, the loss and the
    weight update, overall and per parameter group."""
    inner = accelerator.unwrap_model(policy, keep_fp32_wrapper=True)
    params = {n: p for n, p in inner.named_parameters() if p.requires_grad}
    w0 = {n: to_cpu(p) for n, p in params.items()}
    # The optimizer state (2 x the trained weights) goes to disk, not RAM: a CPU run of the check
    # would not fit in 12 GB with it.
    opt0 = ACCUM_CHECK + ".optimizer.pt"
    torch.save(optimizer.state_dict(), opt0)
    sched0 = to_cpu(lr_scheduler.state_dict()) if lr_scheduler else None
    scaler0 = copy.deepcopy(accelerator.scaler.state_dict()) if accelerator.scaler else None
    draws = getattr(inner.model, "_fixed_draws", None) or FixedDraws(inner.model)
    inner.model._fixed_draws = draws
    draws.noise = draws.time = None
    grads = {}
    hook = optimizer.optimizer.register_step_pre_hook(
        lambda opt, args, kwargs: grads.update({n: to_cpu(p.grad) for n, p in params.items() if p.grad is not None}))

    def restore():
        with torch.no_grad():
            for n, p in params.items():
                p.copy_(w0[n].to(p.device))
        optimizer.load_state_dict(torch.load(opt0, map_location="cpu", weights_only=False))
        if lr_scheduler:
            lr_scheduler.load_state_dict(copy.deepcopy(sched0))
        if scaler0:
            accelerator.scaler.load_state_dict(copy.deepcopy(scaler0))

    def one(update):
        grads.clear()
        draws.rewind()
        update(train_metrics, policy, batch, optimizer, grad_clip_norm, accelerator=accelerator,
               lr_scheduler=lr_scheduler, lock=lock)
        return {"loss": train_metrics.loss.val, "grad_norm": train_metrics.grad_norm.val, "skipped": not grads,
                "grads": dict(grads), "delta": {n: to_cpu(p) - w0[n] for n, p in params.items()}}

    by_group = {}
    for n in params:
        by_group.setdefault(group(n), []).append(n)

    def compare(x, ref):
        if x["skipped"] or ref["skipped"]:
            return {"skipped": True}
        out = {"loss_rel": abs(x["loss"] - ref["loss"]) / abs(ref["loss"]),
               "grad_norm_rel": abs(x["grad_norm"] - ref["grad_norm"]) / ref["grad_norm"],
               "grads_rel": rel(x["grads"], ref["grads"], list(ref["grads"])),
               "update_rel": rel(x["delta"], ref["delta"], list(params))}
        for g, names in sorted(by_group.items()):
            out[f"grads_rel_{g}"] = rel(x["grads"], ref["grads"], [n for n in names if n in ref["grads"]])
            out[f"update_rel_{g}"] = rel(x["delta"], ref["delta"], names)
        return out

    whole = one(_update_policy)
    restore()
    floor = compare(one(_update_policy), whole)
    restore()
    accumulated = one(update_policy_accumulated)
    hook.remove()
    os.remove(opt0)
    row = {"step": len(CHECK_ROWS) + 1, "batch": len(batch["action"]), "micro_batch": MICRO_BATCH,
           "loss": whole["loss"], "grad_norm": whole["grad_norm"],
           "missing_grads": sorted(set(params) - set(whole["grads"]))[:5] if not whole["skipped"] else [],
           "whole_again_vs_whole": floor, "accumulated_vs_whole": compare(accumulated, whole)}
    CHECK_ROWS.append(row)
    print("[accum check] " + json.dumps(row), flush=True)
    with open(ACCUM_CHECK, "w") as f:
        json.dump(CHECK_ROWS, f, indent=1)
    return train_metrics, {}


def save_checkpoint_with_scaler(checkpoint_dir, *args, **kwargs):
    scaler = ACCELERATOR.scaler if ACCELERATOR is not None else None
    if scaler is not None and scaler.is_enabled():
        state_dir = Path(checkpoint_dir) / "training_state"
        state_dir.mkdir(parents=True, exist_ok=True)
        (state_dir / SCALER_FILE).write_text(json.dumps(scaler.state_dict()))
    return _save_checkpoint(checkpoint_dir, *args, **kwargs)


def load_training_state_with_scaler(checkpoint_dir, *args, **kwargs):
    global SCALER_STATE
    out = _load_training_state(checkpoint_dir, *args, **kwargs)
    saved = Path(checkpoint_dir) / "training_state" / SCALER_FILE
    if saved.is_file():
        SCALER_STATE = json.loads(saved.read_text())
        print(f"[fp32 launcher] fp16 loss scaler state read from the checkpoint: {SCALER_STATE}", flush=True)
    else:
        print(f"[fp32 launcher] no {SCALER_FILE} in the checkpoint: the fp16 loss scaler starts afresh", flush=True)
    return out


def with_scaler(update):
    def wrapped(*args, accelerator, **kwargs):
        global ACCELERATOR, SCALER_STATE
        ACCELERATOR = accelerator
        if SCALER_STATE is not None:
            if accelerator.scaler is not None and accelerator.scaler.is_enabled():
                accelerator.scaler.load_state_dict(SCALER_STATE)
                print(f"[fp32 launcher] fp16 loss scaler restored: scale {accelerator.scaler.get_scale()}", flush=True)
            SCALER_STATE = None
        return update(*args, accelerator=accelerator, **kwargs)
    return wrapped


# DataLoader workers start with "spawn" and re-import this file as __mp_main__.
if __name__ == "__main__":
    lt.make_policy = make_policy_fp32
    _update_policy = lt.update_policy
    if ACCUM_CHECK:
        lt.update_policy = update_policy_checked
        print(f"[fp32 launcher] accumulation check: whole batch against micro-batches of {MICRO_BATCH}", flush=True)
    elif MICRO_BATCH:
        lt.update_policy = update_policy_accumulated
        print(f"[fp32 launcher] gradient accumulation: micro-batches of {MICRO_BATCH}", flush=True)
    lt.update_policy = with_scaler(lt.update_policy)
    lt.save_checkpoint = save_checkpoint_with_scaler
    lt.load_training_state = load_training_state_with_scaler
    sys.exit(lt.main())
'''
# lerobot prints the step through format_big_number ("step:950", then "step:1K"), so from 1000 on
# the count comes from the tqdm prefix on the same line ("1000/20000 ["); after a resume that
# prefix restarts at 1, so below 1000 the step: field is used.
STEP_RE = re.compile(r'step:([\d.]+)([KMB]?)\s.*?loss:([\d.eE+-]+).*?updt_s:([\d.]+).*?data_s:([\d.]+)')
TQDM_RE = re.compile(r'(\d+)/\d+ \[')
MEM_RE = re.compile(r'mem_gb:([\d.]+)')
SUFFIX = {'': 1, 'K': 1_000, 'M': 1_000_000, 'B': 1_000_000_000}
OOM_MARKERS = ('OutOfMemoryError', 'CUDA out of memory')


def parse_step(line, m):
    if not m[2]:
        return int(m[1])
    t = TQDM_RE.search(line[:m.start()])
    return int(t[1]) if t else int(float(m[1]) * SUFFIX[m[2]])


def log(msg):
    print(f'[driver {time.strftime("%H:%M:%S")}] {msg}', flush=True)


def find_dataset(input_root, scratch):
    for root_dir in (Path(input_root), Path(scratch) / 'unzipped'):
        for info in sorted(root_dir.rglob('meta/info.json')):
            root = info.parent.parent
            features = json.loads(info.read_text()).get('features', {})
            if all(k in features for k in CAMERA_RENAME) and (root / 'videos').is_dir():
                return root
        archives = sorted(Path(input_root).rglob('mycobot_tri_sort.zip'))
        if archives and root_dir == Path(input_root):
            log(f'extracting {archives[0]}')
            shutil.unpack_archive(archives[0], Path(scratch) / 'unzipped')
    sys.exit(f'No mycobot_tri_sort dataset (meta/info.json with top/right/left, or mycobot_tri_sort.zip) '
             f'under {input_root}. Attach the uploaded dataset as an input.')


def stage_dataset(cfg):
    src = find_dataset(cfg['input_root'], cfg['scratch'])
    dst = Path(cfg['data_root']) / REPO_ID
    if not (dst / 'meta' / 'info.json').exists():
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copytree(src, dst, ignore=shutil.ignore_patterns('images'))
    info = json.loads((dst / 'meta' / 'info.json').read_text())
    log(f'dataset {src} -> {dst}: {info["total_episodes"]} episodes, {info["total_frames"]} frames, '
        f'fps {info["fps"]}')
    return dst


def drop_incomplete_checkpoints(run_dir):
    """After lerobot-train has exited, delete checkpoints newer than `last`: lerobot moves `last`
    only once a save is complete, so a newer one was cut mid-write and must not be resumed from."""
    last = Path(run_dir) / 'checkpoints' / 'last'
    if not last.is_symlink():
        return
    for d in last.parent.iterdir():
        if d.is_dir() and not d.is_symlink() and d.name.isdigit() and int(d.name) > int(last.resolve().name):
            shutil.rmtree(d, ignore_errors=True)
            log(f'deleted {d.name}: written past `last`, incomplete')


# Every file lerobot writes in training_state; scheduler_state.json comes last. A session stopped
# by the time limit in the middle of a save leaves a checkpoint with only some of them (CPU test,
# 19/10: optimizer state written, parameter groups and scheduler not), which must not be resumed.
# The `last` symlink is not used: Kaggle may not keep symlinks in a session's output.
TRAINING_STATE_FILES = ('training_step.json', 'rng_state.safetensors', 'optimizer_state.safetensors',
                        'optimizer_param_groups.json', 'scheduler_state.json')


def complete_checkpoints(root):
    found = []
    for cfg_file in Path(root).rglob('checkpoints/*/pretrained_model/train_config.json'):
        ckpt = cfg_file.parent.parent
        if ckpt.is_symlink() or not ckpt.name.isdigit():
            continue
        if (all((ckpt / 'training_state' / f).is_file() for f in TRAINING_STATE_FILES)
                and any((ckpt / 'pretrained_model').glob('*.safetensors'))):
            found.append((int(ckpt.name), ckpt))
    return sorted(found)


def prune_checkpoints(run_dir, keep_every=0, kept_root=None):
    """Delete checkpoints older than the one `last` points to.

    lerobot moves `last` only after a save completes, and a checkpoint newer than `last` may be
    one still being written, so both are kept. Safe to call at any moment. With keep_every, the
    weights of every keep_every-th step move to kept_root/<step>/pretrained_model first (no
    optimizer state: 1.8 GB instead of ~3.5), so earlier checkpoints can be scored in Gazebo.
    """
    ckpt_root = Path(run_dir) / 'checkpoints'
    last = ckpt_root / 'last'
    if not last.is_symlink():
        return
    keep = int(last.resolve().name)
    for d in ckpt_root.iterdir():
        if d.is_dir() and not d.is_symlink() and d.name.isdigit() and int(d.name) < keep:
            if keep_every and int(d.name) % keep_every == 0 and (d / 'pretrained_model').is_dir():
                dst = Path(kept_root) / d.name / 'pretrained_model'
                if not dst.exists():
                    dst.parent.mkdir(parents=True, exist_ok=True)
                    shutil.move(str(d / 'pretrained_model'), str(dst))
                    log(f'kept weights of step {d.name} in {dst}')
            shutil.rmtree(d, ignore_errors=True)
            log(f'pruned {d.name}')


class GpuMemSampler(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.peak_mb = None
        self.stop = threading.Event()

    def run(self):
        if not shutil.which('nvidia-smi'):
            return
        while not self.stop.wait(1.0):
            try:
                out = subprocess.run(['nvidia-smi', '--query-gpu=memory.used', '--format=csv,noheader,nounits',
                                      '-i', os.environ.get('CUDA_VISIBLE_DEVICES', '0').split(',')[0]],
                                     capture_output=True, text=True, timeout=10).stdout.strip()
                self.peak_mb = max(self.peak_mb or 0, int(out.splitlines()[0]))
            except (ValueError, IndexError, subprocess.TimeoutExpired):
                pass


def run_lerobot(cfg, args, log_path, deadline=None, prune_dir=None, stop_after_step=None):
    """Stream lerobot-train output; stop it cleanly at the deadline, or once a step past stop_after_step
    is logged (resume check). Returns (rc, steps, oom, peak_mb, stopped)."""
    keep = dict(keep_every=cfg.get('keep_every', 0), kept_root=Path(cfg['working']) / 'kept')
    if cfg.get('fp32_weights', True):
        launcher = Path(cfg['scratch']) / 'lerobot_train_fp32.py'
        launcher.write_text(FP32_LAUNCHER)
        cmd = [str(Path(cfg['venv']) / 'bin' / 'python'), str(launcher)] + args
    else:
        cmd = [str(Path(cfg['venv']) / 'bin' / 'lerobot-train')] + args
    env = dict(os.environ, HF_HOME=cfg['hf_home'], WANDB_MODE='disabled', PYTHONUNBUFFERED='1',
               SMOLVLA_FREEZE=cfg.get('freeze') or '', SMOLVLA_MICRO_BATCH=str(cfg.get('micro_batch') or ''),
               SMOLVLA_ACCUM_CHECK=cfg.get('accum_check') or '')
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log('run: ' + ' '.join(cmd))
    sampler = GpuMemSampler()
    sampler.start()
    steps, oom, stopped = [], False, False
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env,
                            start_new_session=True)
    watchdog = None
    if deadline:
        def stop_at_deadline():
            nonlocal stopped
            while proc.poll() is None:
                if time.time() > deadline:
                    stopped = True
                    log('session time limit reached: stopping training (resume from the last checkpoint)')
                    os.killpg(proc.pid, signal.SIGINT)
                    try:
                        proc.wait(timeout=180)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid, signal.SIGKILL)
                    return
                time.sleep(10)
        watchdog = threading.Thread(target=stop_at_deadline, daemon=True)
        watchdog.start()
    # Pruning at every step line and every "Checkpoint" line keeps at most two checkpoints on disk:
    # the one `last` points to and the one being written.
    with open(log_path, 'a') as f:
        for line in proc.stdout:
            f.write(line)
            if any(m in line for m in OOM_MARKERS):
                oom = True
            m = STEP_RE.search(line)
            if m:
                step = parse_step(line, m)
                mem = MEM_RE.search(line)
                steps.append({'step': step, 'loss': float(m[3]), 'updt_s': float(m[4]), 'data_s': float(m[5]),
                              'mem_gb': float(mem[1]) if mem else None})
                if stop_after_step and step > stop_after_step and not stopped:
                    stopped = True
                    log(f'step {step} logged: stopping training as a session cut would (resume check)')
                    os.killpg(proc.pid, signal.SIGINT)
            if prune_dir and (m or 'Checkpoint policy after step' in line):
                prune_checkpoints(prune_dir, **keep)
            if m or 'Checkpoint' in line or '[fp32 launcher]' in line or '[accum check]' in line or (
                    ('Traceback' in line or 'Error' in line) and 'torchcodec' not in line
                    and 'libnppicc' not in line and 'Could not load this library' not in line):
                print(line.rstrip()[-300:], flush=True)
    rc = proc.wait()
    sampler.stop.set()
    if prune_dir:
        prune_checkpoints(prune_dir, **keep)
    return rc, steps, oom, sampler.peak_mb, stopped


def common_args(cfg, data_dir, batch_size, steps, use_amp, schedule_steps=None):
    schedule_steps = schedule_steps or steps
    return [
        '--policy.path=lerobot/smolvla_base',
        f'--policy.device={cfg["device"]}',
        '--policy.push_to_hub=false',
        f'--policy.use_amp={str(use_amp).lower()}',
        f'--policy.scheduler_decay_steps={schedule_steps}',
        f'--policy.scheduler_warmup_steps={min(1000, max(1, schedule_steps // 20))}',
        f'--dataset.repo_id={REPO_ID}',
        f'--dataset.root={data_dir}',
        '--rename_map=' + json.dumps(CAMERA_RENAME, separators=(',', ':')),
        f'--batch_size={batch_size}',
        f'--steps={steps}',
        f'--num_workers={cfg["num_workers"]}',
        '--wandb.enable=false',
        f'--seed={cfg["seed"]}',
    ] + ([f'--dataset.video_backend={cfg["video_backend"]}'] if cfg.get('video_backend') else []) + (
        ['--policy.freeze_vision_encoder=false', '--policy.train_expert_only=false']
        if cfg.get('freeze') == 'runC' else [])


def benchmark(cfg, data_dir):
    work = Path(cfg['working'])
    rows = []
    for use_amp in cfg['bench_amp']:
        for bs in cfg['bench_batch_sizes']:
            out = Path(cfg['scratch']) / f'bench_bs{bs}_amp{int(use_amp)}'
            shutil.rmtree(out, ignore_errors=True)
            args = common_args(cfg, data_dir, bs, cfg['bench_steps'], use_amp) + [
                '--log_freq=1', '--save_checkpoint=false', f'--output_dir={out}']
            t0 = time.time()
            rc, steps, oom, peak, _ = run_lerobot(cfg, args, work / 'logs' / f'bench_bs{bs}_amp{int(use_amp)}.log')
            shutil.rmtree(out, ignore_errors=True)
            steady = [s for s in steps if s['step'] > cfg['bench_warmup_steps']]
            row = {'batch_size': bs, 'amp': use_amp, 'rc': rc, 'oom': oom, 'peak_gpu_mb': peak,
                   'wall_s': round(time.time() - t0)}
            if steady and rc == 0:
                updt = sum(s['updt_s'] for s in steady) / len(steady)
                data = sum(s['data_s'] for s in steady) / len(steady)
                row.update(s_per_step=round(updt + data, 3), updt_s=round(updt, 3), data_s=round(data, 3),
                           samples_per_s=round(bs / (updt + data), 2),
                           mem_gb=max((s['mem_gb'] for s in steps if s['mem_gb'] is not None), default=None))
            rows.append(row)
            log(f'benchmark {row}')
            if oom or rc != 0:
                break
    (work / 'benchmark.json').write_text(json.dumps(rows, indent=2))
    ok = [r for r in rows if 's_per_step' in r]
    print('\n batch  amp    s/step  data_s  samples/s  mem_gb  peak GPU MB   h for train_steps   steps in 11 h')
    for r in ok:
        hours = r['s_per_step'] * cfg['train_steps'] / 3600
        print(f" {r['batch_size']:5d}  {str(r['amp']):5s}  {r['s_per_step']:6.2f}  {r['data_s']:6.2f}  "
              f"{r['samples_per_s']:9.2f}  {str(r['mem_gb']):>6s}  {str(r['peak_gpu_mb']):>11s}   {hours:17.1f}   "
              f"{int(11 * 3600 / r['s_per_step']):13d}")
    for r in rows:
        if 's_per_step' not in r:
            print(f" {r['batch_size']:5d}  {str(r['amp']):5s}  {'out of memory' if r['oom'] else 'failed rc=' + str(r['rc'])}"
                  f" (log: logs/bench_bs{r['batch_size']}_amp{int(r['amp'])}.log)")
    if ok:
        best = max(ok, key=lambda r: r['samples_per_s'])
        print(f"\nFastest: batch {best['batch_size']}, amp {best['amp']}: {best['samples_per_s']} samples/s. "
              f"train_steps={cfg['train_steps']} at that batch would take "
              f"{best['s_per_step'] * cfg['train_steps'] / 3600:.1f} h of GPU.")
        if best['data_s'] > 0.3 * best['s_per_step']:
            print('Note: data loading is over 30 % of each step; video decoding on the CPU is limiting.')


def train(cfg, data_dir):
    work = Path(cfg['working'])
    run_dir = work / 'run'
    # A Kaggle session's output holds only what it wrote: carry the kept weights of earlier
    # sessions (attached as input) forward, so the last session's output has them all.
    for kept in sorted(Path(cfg['input_root']).rglob('kept/*/pretrained_model')):
        dst = work / 'kept' / kept.parent.name / 'pretrained_model'
        if kept.parent.name.isdigit() and not dst.exists():
            shutil.copytree(kept, dst)
            log(f'carried kept weights of step {kept.parent.name} forward')
    previous = complete_checkpoints(cfg['input_root'])
    if previous and previous[-1][0] >= cfg['train_steps']:
        log(f'TRAINING COMPLETE already: the attached input holds step {previous[-1][0]} ({previous[-1][1]})')
        return
    if previous:
        step, ckpt = previous[-1]
        log(f'resuming from step {step}: {ckpt}')
        # A session cut right after a kept step leaves that checkpoint only in the input,
        # where pruning never sees it.
        dst = work / 'kept' / ckpt.name / 'pretrained_model'
        if cfg.get('keep_every') and step % cfg['keep_every'] == 0 and not dst.exists():
            shutil.copytree(ckpt / 'pretrained_model', dst)
            log(f'kept weights of step {ckpt.name} (resumed from) in {dst}')
        args = [f'--config_path={ckpt / "pretrained_model" / "train_config.json"}', '--resume=true']
    else:
        if run_dir.exists() and complete_checkpoints(run_dir):
            sys.exit(f'{run_dir} already holds checkpoints from this session; attach it as input to resume, '
                     'or delete it to start over.')
        shutil.rmtree(run_dir, ignore_errors=True)
        log('no previous checkpoint attached: fresh run from lerobot/smolvla_base')
        args = common_args(cfg, data_dir, cfg['train_batch_size'], cfg['train_steps'], cfg['train_amp']) + [
            f'--save_freq={cfg["save_freq"]}', f'--log_freq={cfg["log_freq"]}', f'--output_dir={run_dir}']
    deadline = time.time() + cfg['session_hours'] * 3600
    rc, steps, oom, peak, stopped = run_lerobot(cfg, args, work / 'logs' / f'train_{time.strftime("%Y%m%d_%H%M%S")}.log',
                                                deadline=deadline, prune_dir=run_dir,
                                                stop_after_step=cfg.get('stop_after_step'))
    drop_incomplete_checkpoints(run_dir)
    done = complete_checkpoints(run_dir)
    summary = {'rc': rc, 'oom': oom, 'stopped_at_time_limit': stopped, 'peak_gpu_mb': peak,
               'last_logged_step': steps[-1]['step'] if steps else None,
               'last_logged_loss': steps[-1]['loss'] if steps else None,
               'checkpoint': str(done[-1][1]) if done else None,
               'target_steps': cfg['train_steps']}
    (work / 'train_summary.json').write_text(json.dumps(summary, indent=2))
    log(f'summary {summary}')
    if not done:
        sys.exit('No complete checkpoint was written in this session.')
    if done[-1][0] >= cfg['train_steps']:
        log(f'TRAINING COMPLETE. Download {done[-1][1] / "pretrained_model"}')
    else:
        log(f'Stopped at step {done[-1][0]} of {cfg["train_steps"]}. Next session: attach this output as input '
            'and run again in train mode; it resumes from this checkpoint.')


# Accumulation check pass rule, fixed before the first GPU run (19/10/2026): on every step where
# neither pass was skipped by the fp16 loss scaler (at least 3 such steps), the accumulated pass
# against the whole-batch pass: loss, pre-clip gradient norm and gradients (overall and per
# group) within 1 %, weight update (overall and per group) within 5 %. The update is looser
# because Adam's first steps divide each gradient by its own magnitude, so entries near zero
# can flip sign on rounding alone.
CHECK_LIMITS = {'loss_rel': 0.01, 'grad_norm_rel': 0.01, 'grads_rel': 0.01, 'update_rel': 0.05}
CHECK_MIN_STEPS = 3


def accum_check(cfg, data_dir):
    work = Path(cfg['working'])
    out = Path(cfg['scratch']) / 'accum_check_run'
    report = work / 'accum_check.json'
    shutil.rmtree(out, ignore_errors=True)
    report.unlink(missing_ok=True)
    args = common_args(cfg, data_dir, cfg['check_batch'], cfg['check_steps'], cfg['train_amp'],
                       schedule_steps=cfg['train_steps']) + [
        '--log_freq=1', '--save_checkpoint=false', f'--output_dir={out}']
    rc, _, oom, peak, _ = run_lerobot(dict(cfg, micro_batch=cfg['check_micro'], accum_check=str(report)), args,
                                      work / 'logs' / 'accum_check.log')
    shutil.rmtree(out, ignore_errors=True)
    rows = json.loads(report.read_text()) if report.exists() else []
    judged, failures = 0, []
    print(f"\n accumulation check: batch {cfg['check_batch']} against {cfg['check_batch'] // cfg['check_micro']} x "
          f"{cfg['check_micro']}; rc {rc}, oom {oom}, nvidia-smi peak {peak} MB")
    print(' step   loss   grad_norm   floor grads   loss_rel  grad_norm_rel  grads_rel  update_rel   per group grads / update')
    for r in rows:
        a, f = r['accumulated_vs_whole'], r['whole_again_vs_whole']
        if a.get('skipped'):
            print(f" {r['step']:4d}   skipped by the fp16 loss scaler")
            continue
        judged += 1
        groups = ', '.join(f"{k[10:]} {a[k]:.1e}/{a['update_rel_' + k[10:]]:.1e}"
                           for k in sorted(a) if k.startswith('grads_rel_'))
        print(f" {r['step']:4d}  {r['loss']:.4f}  {r['grad_norm']:9.3f}   {f.get('grads_rel', float('nan')):.1e}      "
              f"{a['loss_rel']:.1e}    {a['grad_norm_rel']:.1e}       {a['grads_rel']:.1e}    {a['update_rel']:.1e}     {groups}")
        for k, v in a.items():
            limit = CHECK_LIMITS.get(next((b for b in ('grads_rel', 'update_rel') if k.startswith(b)), k))
            if limit is not None and not v <= limit:
                failures.append(f"step {r['step']}: {k} {v:.2e} > {limit}")
        if r['missing_grads']:
            failures.append(f"step {r['step']}: trainable tensors without a gradient, e.g. {r['missing_grads']}")
    if judged < CHECK_MIN_STEPS:
        failures.append(f'only {judged} steps judged (need {CHECK_MIN_STEPS})')
    verdict = {'pass': not failures and rc == 0, 'judged_steps': judged, 'failures': failures, 'rc': rc, 'oom': oom,
               'peak_gpu_mb': peak, 'limits': CHECK_LIMITS}
    (work / 'accum_check_verdict.json').write_text(json.dumps(verdict, indent=2))
    print('\nACCUMULATION CHECK ' + ('PASS' if verdict['pass'] else 'FAIL') + ''.join('\n  ' + x for x in failures))


def logged_steps(log_path):
    rows = {}
    for line in Path(log_path).read_text().splitlines():
        m = STEP_RE.search(line)
        if m:
            rows[parse_step(line, m)] = float(m[3])
    return rows


def resume_check(cfg, data_dir):
    """The same run twice through train(): once in one go, once stopped right after step
    resume_stop_step (SIGINT, as the session time limit does; the checkpoint before it is complete)
    and resumed from its output attached as input. Compares the final checkpoints (step, scheduler,
    weights, optimizer state) and the per-step losses."""
    import torch
    from safetensors.torch import load_file
    base = Path(cfg['scratch']) / 'resume_check'
    shutil.rmtree(base, ignore_errors=True)
    no_input = base / 'no_input'
    no_input.mkdir(parents=True)
    common = dict(cfg, mode='train', train_steps=cfg['resume_steps'], save_freq=cfg['resume_save_freq'],
                  log_freq=1, keep_every=0)
    t0 = time.time()
    one = dict(common, working=str(base / 'one_go'), input_root=str(no_input), session_hours=24)
    train(one, data_dir)
    one_go_s = time.time() - t0
    # As on Kaggle: both sessions write to the same working directory (the checkpoint's
    # train_config.json names it as output_dir), and the first one's output comes back as input.
    session = base / 'session'
    first = dict(common, working=str(session), input_root=str(no_input), session_hours=24,
                 stop_after_step=cfg['resume_stop_step'])
    try:
        train(first, data_dir)
    except SystemExit as e:
        log(f'first session ended: {e}')
    first['working'] = str(base / 'first_output')
    shutil.move(str(session), first['working'])
    stopped_at = complete_checkpoints(Path(first['working']) / 'run')
    second = dict(common, working=str(session), input_root=first['working'], session_hours=24)
    try:
        train(second, data_dir)
    except SystemExit as e:
        result = {'pass': False, 'failure': f'second session: {e}', 'resumed_from_step': stopped_at[-1][0] if stopped_at else None}
        (Path(cfg['working']) / 'resume_check.json').write_text(json.dumps(result, indent=2))
        print('\nRESUME CHECK ' + json.dumps(result, indent=2))
        return

    def final(work):
        step, ckpt = complete_checkpoints(Path(work) / 'run')[-1]
        state = ckpt / 'training_state'
        scaler = state / 'grad_scaler.json'
        return {'step': step, 'weights': load_file(ckpt / 'pretrained_model' / 'model.safetensors'),
                'optimizer': load_file(state / 'optimizer_state.safetensors'),
                'scheduler': json.loads((state / 'scheduler_state.json').read_text()),
                'scaler': json.loads(scaler.read_text()) if scaler.is_file() else None}

    def skipped(*works):
        return sum(p.read_text().count('update skipped by the fp16 loss scaler')
                   for w in works for p in (Path(w) / 'logs').glob('train_*.log'))
    a, b = final(one['working']), final(second['working'])

    def rel_diff(x, y):
        keys = sorted(set(x) & set(y))
        num = sum(float(((x[k].double() - y[k].double()) ** 2).sum()) for k in keys)
        den = sum(float((x[k].double() ** 2).sum()) for k in keys)
        return (num / den) ** 0.5, len(keys), len(set(x) ^ set(y))
    w_rel, w_n, w_extra = rel_diff(a['weights'], b['weights'])
    o_rel, o_n, o_extra = rel_diff(a['optimizer'], b['optimizer'])
    la, lb = logged_steps(newest_log(one['working'])), {}
    for p in sorted((Path(first['working']) / 'logs').glob('train_*.log')) + sorted((Path(second['working']) / 'logs').glob('train_*.log')):
        lb.update(logged_steps(p))
    resumed_from = stopped_at[-1][0] if stopped_at else None
    # The data order after a resume is exact, but recreating the data loader takes one draw from
    # the restored random state, so the flow-matching noise and times differ from the one-go run
    # (traced on CPU, 19/10): losses are compared as means over the steps after the resume.
    after = [s for s in sorted(la) if resumed_from is not None and s > resumed_from and s in lb]
    xa, xb = [la[s] for s in after], [lb[s] for s in after]

    def mean_sd(x):
        m = sum(x) / len(x)
        return m, (sum((v - m) ** 2 for v in x) / max(1, len(x) - 1)) ** 0.5
    (ma, sa), (mb, sb) = (mean_sd(xa), mean_sd(xb)) if after else ((None, None), (None, None))
    se = ((sa ** 2 + sb ** 2) / len(after)) ** 0.5 if after else None
    missing = [s for s in range(1, cfg['resume_steps'] + 1) if s not in lb]
    no_jump = bool(after) and abs(mb - ma) <= 2 * se
    result = {'one_go_s': round(one_go_s), 'resumed_from_step': resumed_from,
              'final_step': [a['step'], b['step']], 'scheduler_equal': a['scheduler'] == b['scheduler'],
              'scheduler_one_go': a['scheduler'], 'scheduler_resumed': b['scheduler'],
              'fp16_scaler_one_go': a['scaler'], 'fp16_scaler_resumed': b['scaler'],
              'fp16_scaler_restored': 'fp16 loss scaler restored' in ''.join(
                  p.read_text() for p in (Path(second['working']) / 'logs').glob('train_*.log')),
              'updates_skipped': {'one_go': skipped(one['working']), 'first_session': skipped(first['working']),
                                  'second_session': skipped(second['working'])},
              'weights_rel_diff': w_rel, 'weights_tensors': w_n, 'weights_tensors_unmatched': w_extra,
              'optimizer_rel_diff': o_rel, 'optimizer_tensors_unmatched': o_extra,
              'steps_logged_one_go': len(la), 'steps_logged_resumed': len(lb),
              'missing_steps_resumed': missing[:20],
              'loss_mean_after_resume': {'steps': len(after), 'one_go': ma, 'resumed': mb, 'sd_one_go': sa,
                                         'sd_resumed': sb, 'difference': None if not after else mb - ma,
                                         'two_standard_errors': None if not after else 2 * se},
              'pass': (resumed_from is not None and a['step'] == b['step'] == cfg['resume_steps']
                       and a['scheduler'] == b['scheduler'] and not missing and no_jump),
              'losses_around_resume': {s: [la.get(s), lb.get(s)] for s in range((resumed_from or 0) - 3, (resumed_from or 0) + 6)
                                       if s in la or s in lb}}
    (Path(cfg['working']) / 'resume_check.json').write_text(json.dumps(result, indent=2))
    print('\nRESUME CHECK ' + json.dumps(result, indent=2))


def newest_log(work):
    return sorted((Path(work) / 'logs').glob('train_*.log'))[-1]


def main():
    cfg = json.loads(Path(sys.argv[1]).read_text())
    for k in ('working', 'scratch', 'data_root', 'hf_home'):
        Path(cfg[k]).mkdir(parents=True, exist_ok=True)
    data_dir = stage_dataset(cfg)
    if cfg['mode'] == 'benchmark':
        benchmark(cfg, data_dir)
    elif cfg['mode'] == 'train':
        train(cfg, data_dir)
    elif cfg['mode'] == 'accumcheck':
        accum_check(cfg, data_dir)
    elif cfg['mode'] == 'resumecheck':
        resume_check(cfg, data_dir)
    else:
        sys.exit(f'unknown mode {cfg["mode"]!r}: use "benchmark", "train", "accumcheck" or "resumecheck"')


if __name__ == '__main__':
    main()