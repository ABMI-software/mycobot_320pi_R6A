---
description: End-of-session workflow — update docs, commit on the right branch. Same logic as the auto-commit rule, but invokable on demand at any milestone.
---

Run the full auto-commit workflow described in [`.claude/rules/auto-commit.md`](../rules/auto-commit.md) right now, regardless of whether the normal trigger conditions have fired.

## Steps

1. `git branch --show-current` and `git status`.
2. Classify every changed file by domain → canonical branch (see [`.claude/rules/git-branching.md`](../rules/git-branching.md)).
3. If any file maps to a branch other than the current one → **STOP**, explain the mismatch, ask how to proceed.
4. Otherwise:
   - Update `SESSION_RESUME.md` with a dated entry summarising what was accomplished this session (append, don't rewrite).
   - Update `CHANGELOG.md` if any user-visible code changed.
   - Update `README.md` and/or relevant `docs/*.md` only if their domain changed.
   - Stage the right files (skip `*.bak*`, `*.xlsx`, `*.log`, `*.pth`, runtime locks — see the auto-commit rule's "Never stage" list).
   - Draft and make a single commit with a conventional message + `Co-Authored-By` trailer.
5. Report: commit hash + branch + 1-line-per-file summary. No giant diff.
6. **Do not push.** Pushing is a separate action that requires an explicit user instruction.

## When to prefer this over letting auto-commit fire

- You want to commit *mid-session* before switching to a different task.
- You want to commit after a specific milestone without waiting for a session-end signal.
- You want an explicit record of when the chunk of work was logically complete (e.g. before a lunch break, before a meeting).

## When NOT to use this

- If auto-commit already ran for the current chunk of work → this will be a no-op.
- If there are unresolved merge conflicts → resolve first, then use this.
- If the diff spans multiple branches' domains → split the work manually first, this command does not split commits.
