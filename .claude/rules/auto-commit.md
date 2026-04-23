# Rule — Auto-documentation and auto-commit

The user does not want to ask Claude to update docs or commit at the end of each session. Claude must **do it proactively**, on the correct branch, with the correct scope — without prompting.

This rule governs when, what, and how.

---

## When to auto-commit

### Trigger automatically (no user prompt needed)

1. **End-of-session signals** — any of:
   - User says "I'm stopping", "je m'arrête", "j'ai fini", "c'est bon pour aujourd'hui", "to be continued", "stop working", or similar.
   - User says "document / sauvegarde / enregistre ce qu'on a fait".
   - User signals they're handing off (starting another task that doesn't continue this thread).
2. **Milestone completion** — a coherent chunk of work just finished and the user moved on:
   - A test run completed with conclusive results.
   - A feature was shipped and demonstrated working.
   - A doc sweep finished.
   - A tooling addition (script, rule, scaffold) was completed.
3. **Branch-switch signals** — user is about to check out a different branch or start unrelated work.

### Do NOT auto-commit

- **Mid-debugging** — if changes are WIP and would break `main` on a fresh clone.
- **Unusual file sets** — untracked binaries, `.env`, credentials, anything in a directory you don't recognize as owned by the project.
- **Incoherent diffs** — changes that span teleop + DREAM + sorting in one pass with no unifying thread. Split first.
- **User has explicitly said "don't commit"** earlier in the session.
- **Dirty local tree you didn't introduce** — if the session started with un-staged changes you didn't make, investigate before touching them.

---

## What to update in docs (always, in the same commit)

| File | Update when |
|------|-------------|
| `SESSION_RESUME.md` | Any session with progress. Prepend a dated entry; do not rewrite the file. |
| `CHANGELOG.md` | Any user-visible code change (new files, new flags, new behavior, renamed commands). Follow Keep-a-Changelog + SemVer. |
| `README.md` | Only if public-facing commands or requirements changed (new install step, new env, new branch). |
| `docs/<domain>.md` | Only if you touched that domain's code. See [`documentation.md`](documentation.md) for the full domain map. |

**Always skip** — never create/update `SUMMARY.md`, `NOTES.md`, `TODO.md`, `IDEAS.md`, session-dated files. The three always-current files plus the `docs/` set are enough.

**SESSION_RESUME dated-entry template:**

```markdown
## État actuel (DD mois YYYY — [matin/après-midi/soir])

### Ce qui a été accompli aujourd'hui
...

### Décisions prises
...

### Prochaines actions
1. [ROUGE] ...
2. [JAUNE] ...
3. [VERT] ...

### Commande rapide de reprise
```bash
...
```

Insert this block just after the "Point de départ rapide" section of the file. Do not delete prior dated entries.

---

## Branch discipline (hard requirement)

Before staging, map each changed file to its canonical branch using the table in [`git-branching.md`](git-branching.md):

| File pattern | Canonical branch |
|--------------|------------------|
| `teleop/**`, `docs/TELEOP_*.md`, `scripts/real_robot_preflight.sh`, `mycobot_gateway/**/trajectory_to_robot_bridge.py`, `mycobot_gateway/**/gripper_to_robot_bridge.py` | `feature/teleoperation` |
| `training/dream/**`, `docs/ARCHITECTURE.md` (DREAM-related edits), DREAM checkpoints | `feature/pose-training` |
| `mycobot_description/urdf/**`, `mycobot_description/worlds/**`, `mycobot_description/meshes/**` | `feature/gazebo` (historical) or relevant active branch |
| `mycobot_gateway/**/synthetic_data_*.py`, `mycobot_description/worlds/randomized_*.sdf` | `feature/synthetic-data` (historical) or `feature/pose-training` |
| `mycobot_gateway/**/pick_and_place_*`, `mycobot_gateway/**/sorting*`, `mycobot_description/worlds/pick_and_place_*.sdf` | active branch (currently main after merge) |
| `CLAUDE.md`, `.claude/**`, `.gitignore`, top-level `README.md`, root-level infra | `main` directly |
| `CHANGELOG.md`, `SESSION_RESUME.md`, `DEVELOPMENT_SUMMARY.md`, `INDEX.md` | Whatever branch the commit's code changes belong to |

**Decision tree:**

1. Run `git branch --show-current` and `git status`.
2. For each changed file, determine its canonical branch from the table.
3. If **all files** map to the current branch → proceed.
4. If some files belong to a *different* branch → **STOP**. Report the mismatch to the user in plain language and ask how to proceed (typically: stash, switch branch, commit the right subset there, come back).
5. **Never** cross-pollinate (e.g. teleop changes committed to `feature/pose-training`). This is a silent-but-serious bug source — squash-merges become unbacktrackable.

If the branch check is ambiguous (e.g. a file that genuinely spans domains like `docs/ARCHITECTURE.md`), prefer `main`-direct commits if the edit is small and cross-cutting; otherwise ask.

---

## What to stage

### Always stage

- Tracked files you modified (`M ` in `git status`).
- New code files under canonical project dirs (`mycobot_gateway/`, `mycobot_description/`, `teleop/`, `training/`, `scripts/`, `.claude/`, `docs/`).
- New documentation files under `docs/` or at the repo root, if the user asked for them.

### Never stage

- `*.bak`, `*.bak2`, `*.orig`, `*.log`, `*.pyc`, `__pycache__/`
- `results/`, `build/`, `install/`, `log/` (ROS2 build tree)
- `*.xlsx` reports (user-local analyses — belong elsewhere)
- Training checkpoints (`*.pth`) — too heavy, project already avoids tracking them
- `.env`, `credentials.json`, private keys, SSH keys
- Files ending in `.session`, `.sessionlock`, `.lock`
- Anything the user introduced earlier in the session that they didn't mark as done

If in doubt, **don't stage it** — a missed file gets picked up in the next commit; a wrongly-committed secret does not get un-committed cleanly.

---

## Commit message conventions

Follow the conventional-commit prefix style already in use on this repo:

```
<type>(<scope>): <summary under 70 chars>

Body paragraph explaining the "why" of the change, including
whatever measured numbers, test results, or context a future
reader needs. Wrap at 72 chars.

Additional paragraphs for unrelated-but-connected facts, artefacts
created, or next-step pointers.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
```

Type vocabulary in use: `feat` · `fix` · `docs` · `refactor` · `test` · `chore` · `perf`.

Scope vocabulary: `teleop` · `dream` · `gazebo` · `sorting` · `bridge` · omit for cross-cutting changes.

**Always** include the `Co-Authored-By` trailer when Claude drafted the commit.

**Never** write commit messages that say "as requested" or "per user's instruction". Future readers don't care who requested it; they care what changed and why.

Use a HEREDOC when invoking `git commit -m` so formatting is preserved:

```bash
git commit -m "$(cat <<'EOF'
feat(dream): short summary

Body.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Push behavior

**Do not push automatically.** Even if committing is automatic, pushing is a different category of action:

- Pushes are visible to collaborators and external systems (CI, reviewers).
- Pushes to protected branches can trigger deployments or protected-branch hooks.
- A bad auto-commit is reversible locally; a bad auto-push is visible for everyone.

Push only when the user explicitly says so ("push", "ship", "envoie", "publie", etc.).

When the user says to push, respect:
- No `--force` without an explicit request.
- No `--force` to `main` ever, even with request — warn first.
- No `-o ci.skip` / `--no-verify` / bypass flags.

---

## Interaction with slash commands

The user may also invoke `/finish-session` explicitly at any time. That command runs the same workflow described here but unconditionally. If auto-commit has already run for the current chunk of work, `/finish-session` should be a no-op (report "nothing to commit").

---

## What the user sees

Keep the auto-commit output terse. A good post-commit report:

```
Committed 9877abcd on feature/pose-training (docs: …).
  SESSION_RESUME.md     +34 −0
  CHANGELOG.md          +12 −0
  training/dream/X.py   +120 new
Push when ready.
```

Do NOT dump the full diff, the full commit body, or the file list unless the user asks.

---

## Edge cases

- **Nothing to commit**: don't make an empty commit. Just say "nothing to commit" and move on.
- **Only backup files / untrackable files changed**: report that, don't commit, don't ask.
- **Pre-commit hook fails**: fix the underlying issue (lint, format, type error) and make a **new** commit. Never `--amend` a hook-failed commit.
- **Merge conflict on a file you auto-edit (e.g., SESSION_RESUME)**: stop, present the conflict, do not resolve autonomously.
- **Very large diff (> 500 files or > 10 MB)**: stop and ask — likely a rebase mishap or accidental bulk change.
