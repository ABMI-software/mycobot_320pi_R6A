#!/usr/bin/env bash
# État de reprise du dépôt, en dix lignes au plus : où l'on est, ce qui manque.
# Lancé au démarrage de chaque session Claude Code (hook SessionStart dans
# .claude/settings.json) ; utilisable aussi à la main : bash scripts/etat_session.sh
# Ne contacte pas GitHub (pas de fetch) : l'avance/retard est celle du dernier fetch.
set -u
racine="$(git rev-parse --show-toplevel 2>/dev/null)" || exit 0
cd "$racine"

branche="$(git branch --show-current)"
echo "== État du dépôt ($(date +%d/%m/%Y\ %H:%M))"
if git rev-parse -q --verify origin/main >/dev/null; then
  read -r avance retard < <(git rev-list --left-right --count HEAD...origin/main)
  echo "branche ${branche:-<détachée>} : ${avance} commit(s) d'avance, ${retard} de retard sur origin/main (dernier fetch)"
fi
modifs=$(git status --porcelain --untracked-files=no | wc -l)
[ "$modifs" -gt 0 ] && echo "modifications non commitées : $modifs fichier(s) suivi(s)"
remises=$(git stash list | wc -l)
[ "$remises" -gt 0 ] && echo "remises (stash) : $remises — git stash list"

[ "$(git config core.hooksPath)" = ".githooks" ] \
  || echo "⚠ hooks du dépôt inactifs sur ce clone : bash scripts/installer_hooks.sh"

for f in scripts/tool_offset.json; do
  [ -f "$f" ] || echo "⚠ $f absent : le tri réel refusera tout mouvement (voir CLAUDE.md § Safety)"
done

derniere=$(grep -m1 -oE '^## État actuel \([^)]*\)' SESSION_RESUME.md 2>/dev/null | sed 's/^## //')
[ -n "$derniere" ] && echo "SESSION_RESUME.md : dernière entrée « $derniere »"

if [ -n "${MYCOBOT_PI:-}" ]; then
  if timeout 1 bash -c "echo > /dev/tcp/$MYCOBOT_PI/5005" 2>/dev/null; then
    echo "Pi ${MYCOBOT_PI} : bridge joignable (TCP 5005)"
  else
    echo "Pi ${MYCOBOT_PI} : bridge injoignable (TCP 5005) — l'adresse a peut-être changé"
  fi
fi
exit 0
