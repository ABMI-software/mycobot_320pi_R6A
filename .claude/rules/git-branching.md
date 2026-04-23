# Rule — Git branching

## Branch map

| Branch | Role | Notes |
|--------|------|-------|
| `main` | Stable, buildable | All PRs land here via squash merge |
| `feature/pose-training` | Active DREAM work (VGG keypoints, sim-to-real) | Do not add teleop commits here |
| `feature/teleoperation` | Hand teleop (Astra → Wilor → robot), validated 22/04/2026 | Do not add DREAM training commits here |
| `feature/gazebo` | Gazebo sim infrastructure | Largely merged; kept for historical context |
| `feature/synthetic-data` | Data collection for DREAM | Largely merged; kept for historical context |

## Rules of engagement

1. **Never commit directly to `main`.** Always branch + PR.
2. **Commit scope matches branch.** If a fix crosses teleop + DREAM, split it into two commits on the right branches — don't create an omnibus.
3. **Never force-push to a branch someone else may be reviewing.** `feature/pose-training` has an active reviewer; `feature/teleoperation` has the real-robot validation history — don't rewrite those.
4. **Rebase locally, merge publicly.** Keep your local history clean while working, but once you've pushed, use regular merges to preserve the shared history.
5. **Don't skip hooks** (`--no-verify`). If a pre-commit fails, fix the underlying issue and make a new commit. `--amend` on a hook-rejected commit amends the *previous* commit — which is almost never what you want.

## Commit messages

Follow the conventional prefix style already in use:

```
feat(teleop): ...
fix(teleop): ...
docs(teleop): ...
feat(dream): ...
docs: ...       # cross-cutting
```

Include a `Co-Authored-By:` trailer when Claude helped write the code.

## Before pushing

- `git status` — no stray files (`*.bak*`, `*.xlsx` reports, `__pycache__`)
- Check `.gitignore` covers `results/`, `build/`, `install/`, `log/`
- If you pushed *then* realize you forgot a file — make a new commit, not `--amend` + `--force`

## PRs

- Use `gh pr create` with a body that has `## Summary` and `## Test plan`
- One reviewer per PR minimum; for real-robot-affecting PRs (teleop, bridges) require a physical test pass in the description
