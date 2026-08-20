# CLAUDE.md — MyCobot 320 Pi R6A

*Loaded at session start. Gives Claude the minimum it needs to be useful on this repo without spelunking.*

---

## Project in one paragraph

A research platform built around a **MyCobot 320 Pi** 6-DoF arm. Today the repo covers (a) direct control via a ROS2/TCP bridge, (b) a Gazebo Harmonic digital twin with synthetic data collection, (c) a vision-based **pose-estimation** pipeline built on NVlabs' DREAM (VGG-19 → belief maps → PnP), and (d) a hand-teleoperation pipeline (Orbbec Astra → Wilor → rosbridge → joints) validated on the physical robot on 22/04/2026. The system runs split across a **PC Tour** (`10.10.0.115`) and a **Raspberry Pi** on the arm (`10.10.0.221` — not `.223` or `.225`, older docs are wrong).

See [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) for the full diagram, [`SESSION_RESUME.md`](SESSION_RESUME.md) for where active work stands, and [`CHANGELOG.md`](CHANGELOG.md) for the version history.

---

## Session workflow — docs and commits are automatic

**Important.** The user does not want to ask Claude to update docs or commit at the end of every session. Claude must do both **proactively** when:

- the user signals end-of-session ("je m'arrête", "I'm stopping", "c'est bon pour aujourd'hui", "to be continued", "document what we did"), OR
- a coherent milestone just completed (test run conclusive, feature demonstrated, scaffold finished, doc sweep done).

The full policy — when to trigger, which files to update, branch discipline, commit-message conventions, and what **not** to auto-commit — lives in [`.claude/rules/auto-commit.md`](.claude/rules/auto-commit.md). Read it at session start; follow it without prompting.

**Hard boundaries:**
- Branch must match the work's domain (see [`.claude/rules/git-branching.md`](.claude/rules/git-branching.md)). Mismatch → stop and ask.
- Pushing is **never** automatic — only on explicit user instruction.
- Skip backup files (`*.bak*`), local reports (`*.xlsx` in `teleop/`), build outputs, training checkpoints, runtime locks.

For explicit mid-session invocation, use [`/finish-session`](.claude/commands/finish-session.md).

---

## POC direction (2026+)

This repo is not just a control-software project — it's the starting substrate for an emerging-technology POC. The near-term ambition is:

1. **Physics-accurate digital twin** → migrate the simulation path from Gazebo/DART to **NVIDIA Isaac Sim + Isaac Lab**, unlocking photorealistic rendering, soft-body gripper physics, and GPU-parallel training envs. See [`.claude/skills/isaac-sim-integration/SKILL.md`](.claude/skills/isaac-sim-integration/).
2. **AI physics** → use Isaac Sim's differentiable physics and learned world models to train policies that transfer to the real robot without hand-tuned dynamics.
3. **Vision-Language-Action models** → fine-tune a VLA (OpenVLA / Octo / π0 class) on episodic teleop data, deploy a VLA inference node behind the same ROS2 topics the teleop dashboard already uses. See [`.claude/agents/vla-integrator.md`](.claude/agents/vla-integrator.md).
4. **Pose estimation at production accuracy** → keypoint detection sim-to-real gap is now closed (`vgg_ultimate_v4_mix_ft_e30`: ~99% synthetic / **91.6% real**, up from ~26%, via a mix-fine-tune on 50K synthetic + real_3cam×5 oversampled). Remaining gap is in **angle reconstruction**, not detection — see "DREAM pose-estimation — validation status" below. See [`.claude/skills/dream-workflow/SKILL.md`](.claude/skills/dream-workflow/).
5. **Robot training + standardized benchmarks** → a reproducible loop of (teleop demos → LeRobot dataset → VLA fine-tune → sim eval → real-robot eval). See [`.claude/skills/lerobot-dataset/SKILL.md`](.claude/skills/lerobot-dataset/).
6. **POC-ready demonstrator** → a single-command launch that shows the full stack (digital twin + VLA policy + real robot + dashboard) running coherently.

**Gazebo is not being deprecated.** It stays on `main` for kinematic work and fast iteration. Isaac Sim lives on its own branch until parity is proven.

---

## DREAM pose-estimation — validation status (2026-07-13)

Live validation tool: `ros2 run mycobot_gateway dream_validation_dashboard` — overlays DREAM's camera-only pose estimate against real encoder angles, with a per-joint MAE/RMS counter (cumulative over the session, not a rolling window) and a robust 2-pass + Kalman-filtered solver. Current checkpoint: `vgg_ultimate_v4_mix_ft_e30`.

**Target (José): 0.5°–0.9° angular error J1-J6. Current measured gap is 10-20× that, even under ideal conditions**, per `training/dream/angle_error_diagnosis.py` (replays cached real detections with an oracle warm-start to isolate keypoint/geometry error from solver artifacts):

| Joint | kp observing it | Median error (oracle warm-start) |
|-------|------------------|-----------------------------------|
| J1 | 5 | ~6.9° |
| J2 | 4 | ~6.7° |
| J3 | 3 | ~13.7° |
| J4 | 2 | ~13.6° |
| J5 | 1 | ~10.6° — structurally weak (see `training/dream/j5_observability_test.py`, ~15-23° ambiguity band) |
| **J6** | **0** | **Structurally unobservable — no keypoint in the 7-point schema depends on J6's own rotation.** Not a bug, not fixable by retraining or solver tuning. |

**What actually moves the needle toward 0.5-0.9°** (solver tuning — reg_weight, robust loss, filtering — was tested and does not): a 2nd camera (mainly helps J1-J5 via triangulation, does *not* fix J6 alone) and/or adding a keypoint downstream of J6 (e.g. gripper/flange) to the DREAM schema. See `training/dream/README.md` § Joint-Angle Observability.

**Session pose anchor**: single-frame `cv2.solvePnP`, no persisted extrinsic-calibration file, re-derived every launch. A pose-diversity requirement (multi-frame pooled fit) was built and then explicitly removed at user request 2026-07-13 — trading back in single-view PnP rotation ambiguity for an anchor that freezes immediately. If the green (encoder/FK) skeleton looks misaligned after the arm has moved far from wherever the anchor froze, that's this tradeoff, not a regression.

### 2026-07-15 — monocular disambiguation exhausted + consistency mode added

Three monocular angle-fixing ideas were tried and their outcomes proven experimentally (scripts left in the session scratchpad; source solver unchanged):

1. **Mirror-pose seeds in the cold-restart** (IPPE twofold planar ambiguity) — *reverted*. On the flat wrong-branch case, all branches reproject at 0.02–0.13 px and the WRONG branch often fits *better* than the true one, so selection-by-reprojection actively prefers it. Reprojection carries no information to pick the branch.
2. **Multi-frame bundle** (one shared fixed camera pose + per-frame q, keypoints only) — *reverted*. Oracle-init reaches 2.8° at 2 px, but from real single-frame seeds it stays wrong (46° MAE) because wrong per-frame branches also admit a consistent shared pose. Confirms the ambiguity is a genuine information limit, not a solver-tuning gap.
3. **Consistency mode** (`use_encoder_seed`, default ON since 2026-07-20) — *kept*. Seeds the solver on the encoder branch every frame + a strong distal prior `_CONSISTENCY_REG_VEC=[10,40,40,40,40,1.5]` (J1→J6; **updated 2026-07-23**: J2 raised to 40 — under the near-top-down camera J2 flips to the wrong monocular branch, ~45°→~2-3° once pinned; J1 firmed to 10). **Deliberately NOT independent** (loudly labelled as such): the encoder resolves the branch the image can't, so this is a camera↔encoder CONSISTENCY/refinement check, not autonomous DREAM pose recovery. Measured on the real robot: **MAE(J1-J5) session ~5.5°, window ~3°** (NOT the synthetic-only 2.8° floor). Independent free-solver mode is untouched. Confirmed 2026-07-23 that **free mode is unusable monocularly** (MAE ~65°, wild branch flips) and that lowering the distal weights does **not** help J3-J6 — their distal keypoints are often undetected, so there's nothing to refine; the real lever is distal detection or a 2nd camera, not solver weights.

**Kalman reset on commanded motion (2026-07-23):** `reset_kalman()` clears the per-joint filters on `SET Angles` / `SET Coords` / `Pose automatique`. The commanded move is known-real, so the outlier gate must not freeze the old value (symptom: filtered curve stuck on the previous angle through a real move). `q_pos` also lowered `radians(3.0)²`→`radians(0.5)²`. CSV acquisitions written while the filter is on go to a `…/kalman/` subfolder (filtered `dream` column) vs the parent folder (raw). See [`docs/DREAM_VALIDATION_DASHBOARD.md`](docs/DREAM_VALIDATION_DASHBOARD.md).

**J3/J4 residual — diagnosed, no fix applied.** J4 sits at 7–9° even under consistency mode. A signed-error study (200 cache poses + a live 5-pose J4 sweep {−30,−15,0,+15,+30}°, all 7/7, link5/6 valid) showed the error changes sign pose-to-pose (|mean|/σ ≈ 0.3), reprojection stays 0.5–2.7 px through 17–57° of J4 error, and DREAM J4 does not correlate with the encoder. So it is **weak observability, not a correctable bias** — J4's reprojection sensitivity (~0.32 px/°) is ~4–5× below J2/J3 (2 distal keypoints, low leverage). No offset was added; adding one would help as many poses as it hurts. Full write-up: [`training/dream/J4_OBSERVABILITY_DIAG.md`](training/dream/J4_OBSERVABILITY_DIAG.md). **Takeaway: in the current monocular setup with the 7-keypoint schema, J4 (like J6) is not reliably observable on all poses; closing it needs a 2nd view or a better-placed keypoint.**

---

## 2nd camera (Astra) — tried and reverted, 2026-07-13

A 2-camera fusion (Arducam + Astra, single shared joint-angle fit) was built and wired end-to-end same day, then **fully removed** because the Astra's live framing/distance never reliably hit the ≥5/7 keypoint detection needed to freeze its session anchor (2-3/7 typical, declining over time). Not a code bug — a physical camera-positioning problem.

Removed: `mycobot_gateway/mycobot_gateway/astra_shm_publisher.py` (deleted), `solve_joint_angles_fixed_pose_multiview`/`solve_joint_angles_two_pass_multiview` (removed from `dream_angle_solver.py`), and all Astra state/subscription/anchor-capture/fusion-branch code in `dream_validation_dashboard.py`. Dashboard is back to Arducam-only, exactly the pre-2026-07-13 behavior.

If revisited later: the approach (single-frame non-persisted anchor per camera, same as the Arducam one, fused via one combined `least_squares` residual across both views) is sound — it just needs the Astra actually mounted close/angled enough to get consistent 5+/7 detections before the fusion logic has anything to fuse.

---

## Multi-camera validation dashboard (Arducam + SVPRO) — 2026-07-24

The validation dashboard is now **multi-camera aware and auto-detecting** (1 or 2 calibrated V4L2 cameras, no code edit — plug the 2nd camera and relaunch). Launch: **`ros2 launch mycobot_gateway dream_multicam.launch.py`** (auto-detects; `cameras:=arducam` forces mono). Astra stays out (no V4L2 node / no PnP intrinsic, see above).

**ROS graph** — one parallel branch per detected camera, plus shared nodes. `mycobot_gateway/vision/camera_registry.py` probes `v4l2-ctl`, identifies each camera and loads/rescales its existing intrinsic (arducam=`cam_3` expo 75, SVPRO=`cam_2` 800×600→640×480, normal expo):

```
camera_publisher_arducam → /camera/image_raw       → dream_inference_arducam → /dream/keypoints       ┐
camera_publisher_svpro   → /camera_svpro/image_raw  → dream_inference_svpro   → /dream_svpro/keypoints  ├→ dream_validation_dashboard
joint_sync (/joint_states)  ·  bridge_tour (↔ Pi TCP 5005)                                              ┘
```

`camera_publisher` param `output_topic`, `dream_inference` param `output_prefix` — one instance per camera. Inspect live with `rqt_graph` / `ros2 topic list` (both `/dream/*` and `/dream_svpro/*` present in fusion). Full node/topic table + diagnostics: [`docs/DREAM_VALIDATION_LAUNCH.md`](docs/DREAM_VALIDATION_LAUNCH.md).

**Fusion = solve-then-fuse** (NOT a shared bundle — that branch-flipped, J1 −43°): each camera solves its own `q` (per-view consistency mode), then per-joint fusion weighted by observability (observing keypoint detected AND reproj ≤ `JOINT_CONFIDENCE_PX_THRESHOLD=15px`). Never worse than the best camera per joint; occlusion on one view is covered by the other; falls back to **MONO via {camera}** if the primary goes blind. Measured fusion MAE(J1-J5) ~1.1-1.9°. Keypoint table + "Détection globale (fusion) N/7" reflect the **union** of cameras. ⚠ Not yet validated across many poses on hardware.

**Curve stability = 3 selectable temporal filters** (radio group, **`aucun` is the default — Kalman is NOT on by default**): `kalman` (constant-velocity 1D), `passe_bas` (EMA `_EMA_ALPHA=0.06`), `moyenne` (moving average `_MA_WINDOW=20`). All filter the DREAM estimate only, never the encoder; switching purges all states (`reset_kalman()`); CSV goes to a `…/<filter>/` subfolder. A filter only removes fast tremor — the residual **slow wander** on weakly-observable joints (J3/J4/J5) is the estimate genuinely drifting and is not filterable; the real lever stays camera placement / distal detection. Exposure (arducam 60 vs 75) was re-tested and **does not change detection** (stays ~4-5/7) — keep 75. Anti-flicker: secondary keypoints are display-held 0.8 s; the pose dot stays green while a view detected ≥4 kp within the last 1 s.

---

## Three Python environments — never mix them

This is the single most common source of breakage. **Always know which env you are in.**

| Env | How | Purpose | Python |
|-----|-----|---------|--------|
| **System ROS2** | `conda deactivate && source /opt/ros/jazzy/setup.bash` | Everything ROS2 (colcon, `ros2 launch`, node code) | 3.12 |
| **conda `hand-teleop`** | `conda activate hand-teleop` | Wilor, Orbbec Astra, `teleop/*.py` | 3.10 |
| **venv_dream** | `source ~/ros_jazzy/venv_dream/bin/activate` | DREAM training/eval | 3.12 |

A fourth will appear when Isaac Sim lands — likely a dedicated container or venv pinned to Isaac Sim's Python. Do not add it to system without a plan.

Before *any* ROS2 command: **`conda deactivate`** first. Conda's 3.13 shadows `rclpy`'s 3.12 and everything falls over silently.

See [`.claude/rules/python-environments.md`](.claude/rules/python-environments.md).

---

## Branch map

| Branch | Role | State |
|--------|------|-------|
| `main` | Stable | — |
| `feature/pose-training` | Active DREAM work (vision) | In progress |
| `feature/teleoperation` | Hand teleop (Astra → Wilor → robot), validated 22/04/2026 | In progress |
| `feature/gazebo` | Gazebo simulation infrastructure | Merged |
| `feature/synthetic-data` | Synthetic dataset collection | Merged |
| `feature/isaac-sim` *(planned)* | Isaac Sim / Isaac Lab digital-twin port | Not yet created |
| `feature/vla` *(planned)* | VLA fine-tune + inference node | Not yet created |

Do not cross-pollinate teleop commits into `feature/pose-training` (or vice versa). See [`.claude/rules/git-branching.md`](.claude/rules/git-branching.md).

---

## Common commands

```bash
# Build everything (run from ~/ros_jazzy, not from inside src/)
conda deactivate
cd ~/ros_jazzy && colcon build --packages-select mycobot_gateway mycobot_description --symlink-install
source install/setup.bash

# Control a live robot (bridge must run on the Pi)
ssh er@10.10.0.221        # Pi — start `python3 gripper_bridge.py` (voir avertissement ci-dessous)
ros2 launch mycobot_gateway simple_gui.launch.py

# Gazebo simulation
ros2 launch mycobot_gateway mycobot_teleop.launch.py target:=sim

# Hand teleoperation (4 terminals — see .claude/commands/launch-teleop.md)
# Real-robot preflight (5 checks)
bash scripts/real_robot_preflight.sh
```

More in [`.claude/commands/`](.claude/commands/).

---

## Scaffolding reference

| Need | Go to |
|------|-------|
| How to launch sim / teleop / preflight / train DREAM / collect synthetic | [`.claude/commands/`](.claude/commands/) |
| Troubleshooting teleop, DREAM workflow, Gazebo setup, real-robot session ritual, Isaac Sim migration, LeRobot dataset format | [`.claude/skills/`](.claude/skills/) |
| ROS2 graph debugging, DREAM training runs, teleop tuning, URDF edits, digital-twin parity, VLA integration | [`.claude/agents/`](.claude/agents/) |
| Env rules, ROS2 conventions, real-robot safety, git discipline, doc discipline | [`.claude/rules/`](.claude/rules/) |

---

## Coding conventions

- **Do not create documentation files** (README, *.md, *SUMMARY*) unless the user explicitly asks. See [`.claude/rules/documentation.md`](.claude/rules/documentation.md) for what *does* get updated, and when.
- **Do not add comments** explaining *what* the code does — names are the contract. Only comment *why* when a hidden constraint or surprising invariant is in play.
- **Do not add fallback/defensive code** for impossible states. Validate only at system boundaries (user input, TCP messages, ROS topics). Trust internal code.
- Python style: 4-space indent, no `from X import *`, prefer `pathlib.Path` over `os.path`, prefer f-strings.
- ROS2 style: one node per process for long-running nodes; launch files describe topology, not CLI args.
- **Never use `--no-verify`** when committing. Hooks exist for a reason.

---

## Commande cartésienne — `send_coords` est écarté (mesuré 20/08/2026)

**Ne pas piloter ce robot en `send_coords`.** Comparaison A/B sur cible et
métrique identiques, 267 mm à parcourir : méthode officielle Elephant Robotics
**247,8 mm d'erreur finale** (9 % du trajet) contre **18,2 mm** (93 %) via
`send_angles` + IK différentielle. Les deux reçoivent `OK` du bridge — la méthode
constructeur **échoue en silence**. Cause : blocage de cardan, la tâche se
déroulant entre RY = −78° et −83°, où RX et RZ sont dégénérés.

Utiliser [`scripts/diff_ik.py`](scripts/diff_ik.py) : `fk_pose` rend une **matrice
de rotation**, `solve_pose` la tient, puis `send_angles`. Trois règles qui en
découlent, toutes mesurées :

1. **Tourner l'orientation cible selon l'azimut** — `Rz(azimut_cible −
   azimut_référence) @ R_référence`. Sur 33° d'écart : résidu IK **0,19 mm** au
   lieu de **20,0 mm** à orientation figée.
2. **Travailler sur la branche IK coude haut** (`J3 < 0`, marge 23–72°). Toutes
   les poses historiques (`observation_clear`, pick du 17/08) sont sur la branche
   coude bas, plaquée contre la butée J2 (**marge 0°**) — d'où les sauts de branche
   de 150° sur J4 en boucle fermée. Ne jamais basculer de branche pince en appui.
3. **Compenser l'affaissement gravitaire** — ~13 mm à vide, ~15 mm chargé,
   reproductible. Compensé, l'erreur verticale passe de 13 mm à ~2 mm.

Allonge réelle **≈ 390 mm** (aucune solution IK au-delà). Répétabilité mesurée :
**0,67 mm** en approche unidirectionnelle par le haut, mais **5,88 mm de biais**
si on mélange les directions d'approche — ne jamais les mélanger entre
l'apprentissage d'un point et sa reprise.

⚠ Le bridge de la Pi doit être **`scripts/gripper_bridge.py`**, pas
`bridge_pi_simple.py` : seul le premier répond à `get_pro_gripper_status`, sans
quoi aucune saisie n'est confirmable. Il est **mono-client et bloquant** — un
`bridge_tour` résiduel (que `real_robot_preflight.sh` laisse tourner) le fige :
la connexion TCP est acceptée mais plus rien ne répond.

## Safety — real robot

- Default IP is `10.10.0.221`. Always `ping` before launching anything that commands motion.
- Run [`scripts/real_robot_preflight.sh`](scripts/real_robot_preflight.sh) before each physical session.
- On `feature/teleoperation`: start every session with the `🐢 Safe start` preset (gains 0.6/0.6/0.6, tfs 0.3). Only go to `⚙️ Nominal` (1.2/1.2/1.6/0.25 — the validated default) once calibration is clean.
- **Le robot a désormais une pince Pro adaptative** (`gripper_id=14`, ~1,6 s entre
  deux ordres). Saisie validée sur balle le 20/08/2026. La pince **cale sur l'objet**
  à un angle qui n'est pas celui commandé (52 mesuré pour une consigne de 20) :
  viser un angle plus bas ne serre **pas** davantage, le seul levier est
  `set_pro_gripper_torque`. Le statut (`get_pro_gripper_status`) est la seule
  confirmation de prise valable — un statut *inconnu* n'est pas une vérification.
  Le drapeau `--no-gripper` de `mycobot_teleop.py` ne s'applique plus qu'aux
  sessions de téléop menées sans la pince montée.

See [`.claude/rules/real-robot-safety.md`](.claude/rules/real-robot-safety.md) and [`.claude/skills/real-robot-session/SKILL.md`](.claude/skills/real-robot-session/).

---

## Where to look for more

- [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) — global architecture
- [`docs/TELEOPERATION.md`](docs/TELEOPERATION.md) — hand-teleop pipeline (on `feature/teleoperation`)
- [`docs/SYNTHETIC_DATA.md`](docs/SYNTHETIC_DATA.md) — Gazebo data collection
- [`training/dream/`](training/dream/) — DREAM training scripts, configs
- [`.claude/`](.claude/) — everything above, expanded
