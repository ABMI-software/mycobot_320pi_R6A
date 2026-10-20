# Contributing

These rules exist because this repository mixes three Python environments, a
physical robot that can hurt someone, and measurements that only mean something
if their protocol is respected. Most of them were written after something broke.

## Before anything: the Python environments

Three interpreters coexist on a development machine and **must not be mixed**.

| Environment | Activation | Python | Used for |
|---|---|---|---|
| System ROS 2 | `conda deactivate && source /opt/ros/jazzy/setup.bash` | 3.12 | Anything `ros2`, `colcon`, `rclpy` |
| `hand-teleop` (conda) | `conda activate hand-teleop` | 3.10 | Wilor, Orbbec, everything in `teleop/` |
| `venv_dream` | `source <your_ws>/venv_dream/bin/activate` | 3.12 | DREAM training and evaluation |

**Run `conda deactivate` before any ROS 2 command**, every time, even when you
believe you are already outside conda. Conda's interpreter shadows the one
`rclpy`'s C extensions are compiled against, and the failure is a wall of
unreadable extension errors rather than a clear message.

## Build

Always from the workspace root, never from inside `src/` — colcon writes
`build/`, `install/` and `log/` into its working directory.

```bash
conda deactivate
cd <your_ws>
colcon build --packages-select mycobot_gateway mycobot_description --symlink-install
source install/setup.bash
```

`--symlink-install` means edits to Python nodes take effect without rebuilding.
Scope the build: a full rebuild drags in unrelated pinned packages.

## Branches

| Branch | Scope |
|---|---|
| `main` | Stable, buildable |
| `feature/pose-training` | DREAM work — keypoints, sim-to-real |
| `feature/teleoperation` | Hand teleoperation; carries the real-robot validation history |
| `feature/gazebo`, `feature/synthetic-data` | Largely merged, kept for context |

1. **Commit scope matches branch.** A fix spanning teleoperation and DREAM
   becomes two commits on two branches, never one omnibus.
2. **Never force-push a branch someone may be reviewing.** Rewriting a branch
   that carries a validation history destroys the link between a measurement and
   the code that produced it.
3. **Rebase locally, merge publicly.**
4. **Never `--no-verify`.** If a hook rejects a commit, fix the cause and make a
   new commit — `--amend` after a hook failure amends the *previous* commit,
   which is almost never what you want.

## Commits

Conventional prefixes, scoped by domain:

```
feat(dream): ...     fix(teleop): ...     docs(gazebo): ...
refactor(sorting): ...   test: ...   chore: ...   perf: ...
```

Scopes in use: `teleop` · `dream` · `gazebo` · `sorting` · `bridge`, omitted for
cross-cutting changes.

The body says **why**, with the measured numbers a future reader will need.
Never write "as requested" — nobody cares who asked, they care what changed.

**No attribution trailers.** No `Co-Authored-By`, no assistant mention. A commit
here carries the name of its human author and nothing else.

### What never gets committed

- `*.bak*`, `*.orig`, `*.log`, `__pycache__/`, `build/`, `install/`, `log/`
- Training checkpoints (`*.pth`), runtime locks, `.env`, credentials, keys
- **Extrinsic calibration files** — they go stale, and a stale extrinsic
  committed as fresh is worse than none

`*.xlsx` workbooks are committable **on explicit approval only**. The reason is
not size: a workbook is opaque to `git diff`, so whoever commits it vouches for
its contents.

## Documentation

Three files stay current, and an unrecorded change is an undocumented one:

| File | When |
|---|---|
| `CHANGELOG.md` | Any user-visible change — **same commit**, not a follow-up |
| `SESSION_RESUME.md` | Any session that moved the project's state |
| `README.md` | Any change to a public-facing command or requirement |

Plus the domain document matching the code you touched: `docs/ARCHITECTURE.md`
for topology, `docs/TELEOPERATION.md` for the teleop pipeline,
`mycobot_description/README_GAZEBO.md` for URDF, worlds and meshes, and so on.

Do not create new scratchpad documents — `NOTES.md`, `TODO.md`, `IDEAS.md`,
session-dated files. `INDEX.md` and `DEVELOPMENT_SUMMARY.md` are established
exceptions and are maintained, not multiplied.

Do not backdate CHANGELOG entries, and do not delete old ones to shorten the
file. It is a history; truncating it defeats its purpose.

## Code

- **Do not comment what the code does** — names are the contract. Comment *why*,
  and only when a hidden constraint or a surprising invariant is in play.
- **Do not add defensive code for impossible states.** Validate at system
  boundaries — user input, TCP messages, ROS topics — and trust internal code.
- Python: 4-space indent, no `from X import *`, `pathlib.Path` over `os.path`,
  f-strings.
- ROS 2: one node per process for long-running nodes; launch files describe
  topology, not business logic. Parameters over CLI arguments.
- Topic names stay inside the existing namespaces (`/teleop/*`,
  `/mycobot_controller/*`, `/from_robot`, `/to_robot`). A new top-level
  namespace means updating `docs/ARCHITECTURE.md` in the same commit.

## Working on the physical robot

The arm can injure a hand and destroy a gripper. Before anything that moves a
motor:

1. **The board's address is not fixed.** A successful `ping` proves nothing —
   confirm with a TCP round-trip on port 5005.
2. Run `bash scripts/real_robot_preflight.sh`.
3. **Hold or secure the arm** before any command that releases the servos. It
   goes limp instantly.
4. Teleoperation starts on the **Safe start** preset, never on Nominal. Step up
   only once tracking is validated.
5. `Ctrl+C` does **not** stop the servos — it kills ROS nodes and the arm holds
   its last commanded pose. The emergency stop is the dashboard's release
   button, or unplugging the board.

A pull request that touches the robot, a bridge or the teleoperation path must
describe a physical test pass. One reviewer minimum on every pull request.

## Measurements

Everything in this repository that carries a number was measured, and the
distinction between the following pairs is not pedantry — the project has
already published a wrong conclusion by confusing them:

- A **repeatability** is not an **absolute accuracy**.
- A calibration's **fit residual** is not its **accuracy**; leave-one-out on a
  held-out marker is the honest figure.
- A result from **one run** of the sorting bench is not a result: its outcome is
  not deterministic at identical commanded geometry.

State the protocol with the number, or do not state the number.

## For contributors using an AI assistant

`.claude/` holds machine-readable versions of these same rules, plus commands
and skills specific to this project. **This file is the source of truth**;
`.claude/` restates it for tooling. If the two disagree, this file wins and
`.claude/` needs fixing.
