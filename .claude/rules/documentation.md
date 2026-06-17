# Rule — documentation discipline

Docs get stale fast in this repo. The rule below keeps the three always-current files actually current, without turning every PR into a documentation exercise.

## Three always-current files

| File | What it holds | Update trigger |
|------|---------------|----------------|
| [`CHANGELOG.md`](../../CHANGELOG.md) | Keep-a-Changelog entries, versioned | Any user-visible change (feature, fix, breaking) — same PR |
| [`SESSION_RESUME.md`](../../SESSION_RESUME.md) | "What is the project doing *right now*" | End of each working session that moved state |
| [`README.md`](../../README.md) | How to run the thing from a fresh clone | When a public-facing command or requirement changes |

If you're unsure whether a change is "user-visible" for the CHANGELOG: if a future developer cloning the repo would want to know about it, yes. If it's a refactor only visible in diffs, no.

## Dedicated docs (update when touching their domain)

| File | Update when |
|------|-------------|
| [`docs/ARCHITECTURE.md`](../../docs/ARCHITECTURE.md) | Topology changes — new nodes, new topics, IP changes |
| [`docs/TELEOPERATION.md`](../../docs/TELEOPERATION.md) | Hand-teleop pipeline changes (filters, mapping, bridges) |
| [`docs/TELEOP_DASHBOARD.md`](../../docs/TELEOP_DASHBOARD.md) | Dashboard UI or control surface changes |
| [`docs/TELEOP_TUNING.md`](../../docs/TELEOP_TUNING.md) | Gain/parameter reference updates |
| [`docs/REAL_ROBOT_TEST_PROCEDURE.md`](../../docs/REAL_ROBOT_TEST_PROCEDURE.md) | Real-robot protocol changes (preflight, validated gains) |
| [`docs/SYNTHETIC_DATA.md`](../../docs/SYNTHETIC_DATA.md) | Synthetic data collector or world changes |
| [`training/README.md`](../../training/README.md), [`training/dream/README.md`](../../training/dream/README.md) | Training pipeline changes |

## Package READMEs

- [`mycobot_gateway/README.md`](../../mycobot_gateway/README.md) — update when nodes are added/removed/renamed
- [`mycobot_description/README_GAZEBO.md`](../../mycobot_description/README_GAZEBO.md) — update with any URDF / world / mesh change
- [`datasets/README.md`](../../datasets/README.md) — update with any dataset change (size, format, source)

## Do not

- **Do not create `SUMMARY.md`, `NOTES.md`, `TODO.md`, `IDEAS.md`.** The three always-current files plus the dedicated docs are sufficient. Scratchpad work goes in issues/PR bodies, not in the repo.
- **Do not duplicate content across docs.** Link to the canonical source. Example: real-robot gains live in [`docs/REAL_ROBOT_TEST_PROCEDURE.md`](../../docs/REAL_ROBOT_TEST_PROCEDURE.md) + [`.claude/agents/teleop-tuner.md`](../agents/teleop-tuner.md); other docs should link, not restate.
- **Do not backdate CHANGELOG entries.** Today's work goes under today's date; if the session spans midnight, pick one date and stick with it.
- **Do not remove old CHANGELOG entries** to make the file shorter. It's a history; truncating it defeats the purpose.

## When Claude edits code

If the change is user-visible, update the CHANGELOG in the **same commit**, not a follow-up. Future-you cloning at HEAD will only see what's in the tree.

When the user says "update the docs": it means the three always-current files + whichever dedicated docs in [`docs/`](../../docs/) match the change's domain. It does **not** mean creating new doc files unless the user explicitly asks.
