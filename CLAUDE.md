# CLAUDE.md — MyCobot 320 Pi R6A

*Loaded at session start. Gives Claude the minimum it needs to be useful on this repo without spelunking.*

---

## Project in one paragraph

A research platform built around a **MyCobot 320 Pi** 6-DoF arm. Today the repo covers (a) direct control via a ROS2/TCP bridge, (b) a Gazebo Harmonic digital twin with synthetic data collection, (c) a vision-based **pose-estimation** pipeline built on NVlabs' DREAM (VGG-19 → belief maps → PnP), and (d) a hand-teleoperation pipeline (Orbbec Astra → Wilor → rosbridge → joints) validated on the physical robot on 22/04/2026. The system runs split across a **PC Tour** (`10.10.0.115`) and a **Raspberry Pi** on the arm (`10.10.0.223` — not `.225`, older docs are wrong).

See [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) for the full diagram, [`SESSION_RESUME.md`](SESSION_RESUME.md) for where active work stands, and [`CHANGELOG.md`](CHANGELOG.md) for the version history.

---

## POC direction (2026+)

This repo is not just a control-software project — it's the starting substrate for an emerging-technology POC. The near-term ambition is:

1. **Physics-accurate digital twin** → migrate the simulation path from Gazebo/DART to **NVIDIA Isaac Sim + Isaac Lab**, unlocking photorealistic rendering, soft-body gripper physics, and GPU-parallel training envs. See [`.claude/skills/isaac-sim-integration/SKILL.md`](.claude/skills/isaac-sim-integration/).
2. **AI physics** → use Isaac Sim's differentiable physics and learned world models to train policies that transfer to the real robot without hand-tuned dynamics.
3. **Vision-Language-Action models** → fine-tune a VLA (OpenVLA / Octo / π0 class) on episodic teleop data, deploy a VLA inference node behind the same ROS2 topics the teleop dashboard already uses. See [`.claude/agents/vla-integrator.md`](.claude/agents/vla-integrator.md).
4. **Pose estimation at production accuracy** → close the sim-to-real gap for DREAM (currently 97% synthetic / ~26% real) by retraining on Isaac-Sim-generated photorealistic data. See [`.claude/skills/dream-workflow/SKILL.md`](.claude/skills/dream-workflow/).
5. **Robot training + standardized benchmarks** → a reproducible loop of (teleop demos → LeRobot dataset → VLA fine-tune → sim eval → real-robot eval). See [`.claude/skills/lerobot-dataset/SKILL.md`](.claude/skills/lerobot-dataset/).
6. **POC-ready demonstrator** → a single-command launch that shows the full stack (digital twin + VLA policy + real robot + dashboard) running coherently.

**Gazebo is not being deprecated.** It stays on `main` for kinematic work and fast iteration. Isaac Sim lives on its own branch until parity is proven.

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
ssh er@10.10.0.223        # Pi — start `python3 bridge_pi_simple.py`
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

## Safety — real robot

- Default IP is `10.10.0.223`. Always `ping` before launching anything that commands motion.
- Run [`scripts/real_robot_preflight.sh`](scripts/real_robot_preflight.sh) before each physical session.
- On `feature/teleoperation`: start every session with the `🐢 Safe start` preset (gains 0.6/0.6/0.6, tfs 0.3). Only go to `⚙️ Nominal` (1.2/1.2/1.6/0.25 — the validated default) once calibration is clean.
- **Current physical robot has no gripper.** The `--no-gripper` flag on `mycobot_teleop.py` is mandatory.

See [`.claude/rules/real-robot-safety.md`](.claude/rules/real-robot-safety.md) and [`.claude/skills/real-robot-session/SKILL.md`](.claude/skills/real-robot-session/).

---

## Where to look for more

- [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) — global architecture
- [`docs/TELEOPERATION.md`](docs/TELEOPERATION.md) — hand-teleop pipeline (on `feature/teleoperation`)
- [`docs/SYNTHETIC_DATA.md`](docs/SYNTHETIC_DATA.md) — Gazebo data collection
- [`training/dream/`](training/dream/) — DREAM training scripts, configs
- [`.claude/`](.claude/) — everything above, expanded
