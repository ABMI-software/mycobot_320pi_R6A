#!/usr/bin/env python3
"""Garde-fous du dépôt : ce qui ne doit jamais entrer, et ce qu'on ne doit pas oublier.

Un seul script, appelé à trois endroits :

    python3 scripts/verifier_depot.py --indexe          # hook pre-commit : fichiers indexés
    python3 scripts/verifier_depot.py --message FICHIER  # hook commit-msg
    python3 scripts/verifier_depot.py --complet          # CI et audit : tout le dépôt

Bloquant (code de sortie 1) : secret, fichier interdit, gros fichier hors LFS,
NOUVEAU chemin /home/... ou NOUVELLE IP 10.10.0.x dans le code, lien Markdown
cassé, message de commit hors convention ou portant une ligne d'attribution.

Avertissement seulement : code modifié sans CHANGELOG.md, document de docs/
absent de INDEX.md, et (en --complet) la dette existante d'IP et de chemins.

Seules les lignes AJOUTÉES sont jugées pour les IP et les chemins : la dette
existante ne bloque personne, elle ne peut simplement plus grossir. Une ligne
qui doit vraiment garder une valeur littérale porte le marqueur
`verifier: ok` en commentaire, avec la raison à côté.
"""
from __future__ import annotations

import argparse
import fnmatch
import re
import subprocess
import sys
from pathlib import Path

RACINE = Path(__file__).resolve().parents[1]
LIENS_TOLERES = RACINE / '.github' / 'known-broken-links.txt'
MARQUEUR = 'verifier: ok'
TAILLE_MAX_OCTETS = 5 * 1024 * 1024

SECRETS = [
    (re.compile(r'-----BEGIN [A-Z ]*PRIVATE KEY-----'), 'clé privée'),
    (re.compile(r'\bghp_[A-Za-z0-9]{30,}'), 'jeton GitHub'),
    (re.compile(r'\bhf_[A-Za-z0-9]{30,}'), 'jeton HuggingFace'),
    (re.compile(r'\bsk-ant-[A-Za-z0-9_-]{20,}'), 'clé API Anthropic'),
    (re.compile(r'\bAKIA[0-9A-Z]{16}\b'), 'clé AWS'),
    (re.compile(r'(?i)\b(passwd|password|mot de passe|mdp)\b[^\n]{0,20}?[:=][ \t]*[`"\']?'
                r'(?!\s|<|\{|\$|inconnu|oubli|non |demander|[A-Za-z_][\w.]*[(\[])[^\s`"\'<>]{4,}'),
     'mot de passe en clair'),
    # Forme « mot de passe » suivie de la valeur entre accents graves, sans signe égal.
    (re.compile(r'(?i)\b(passwd|password|mot de passe|mdp)\b[^\n`]{0,20}`[^`\s<>{}$]{4,}`'),
     'mot de passe en clair'),
]

# Motifs fnmatch sur le chemin relatif ; jamais committés, même par erreur.
INTERDITS = [
    ('*.bak', 'sauvegarde'), ('*.bak[0-9]', 'sauvegarde'), ('*.orig', 'reste de fusion'),
    ('*.log', 'journal'), ('.env', 'variables locales'), ('*/.env', 'variables locales'),
    ('results/*', 'résultats de session (à archiver hors Git, voir CONTRIBUTING.md)'),
    ('docs/PI_REMOTE_ACCESS.md', 'contient le mot de passe VNC de la Pi'),
    ('*.pth', 'poids de réseau : seuls les checkpoints listés dans .gitignore sont admis'),
    ('*.code-workspace', 'espace de travail personnel'),
]
POIDS_ADMIS = re.compile(r'^training/dream/(output/)?checkpoints_dream/[^/]+/best_network\.pth$')

CODE = ('.py', '.sh', '.yaml', '.yml', '.launch.py')
IP_BANC = re.compile(r'\b10\.10\.0\.\d{1,3}\b')
CHEMIN_PERSO = re.compile(r'/home/[a-z_][a-z0-9_-]*/')
TYPES_COMMIT = 'feat|fix|docs|refactor|test|chore|perf|ci|style|revert'
SUJET = re.compile(rf'^({TYPES_COMMIT})(\([a-z0-9_./-]+\))?!?: \S.*$')
ATTRIBUTION = re.compile(r'(?im)^(co-authored-by|generated with|signed-off-by: .*claude)')


def git(*args: str) -> str:
    return subprocess.run(['git', *args], cwd=RACINE, capture_output=True,
                          text=True, check=True).stdout


class Rapport:
    def __init__(self):
        self.erreurs: list[str] = []
        self.avertissements: list[str] = []

    def erreur(self, msg: str):
        self.erreurs.append(msg)

    def avertir(self, msg: str):
        self.avertissements.append(msg)

    def conclure(self) -> int:
        for m in self.avertissements:
            print(f'  ⚠ {m}')
        for m in self.erreurs:
            print(f'  ✗ {m}')
        if self.erreurs:
            print(f'\n{len(self.erreurs)} problème(s) bloquant(s). Corriger, puis recommencer.'
                  '\nContournement exceptionnel, à justifier dans le message : git commit --no-verify')
            return 1
        print('  ✓ vérifications passées' + (f' ({len(self.avertissements)} avertissement(s))'
                                           if self.avertissements else ''))
        return 0


# --- briques -----------------------------------------------------------------

def lignes_ajoutees_indexees() -> dict[str, list[tuple[int, str]]]:
    """{chemin: [(numéro, texte)]} des lignes ajoutées dans l'index."""
    diff = git('diff', '--cached', '--unified=0', '--no-color', '--diff-filter=AM')
    ajouts: dict[str, list[tuple[int, str]]] = {}
    courant, num = None, 0
    for ligne in diff.splitlines():
        if ligne.startswith('+++ '):
            courant = ligne[6:] if ligne.startswith('+++ b/') else None
        elif ligne.startswith('@@'):
            num = int(re.search(r'\+(\d+)', ligne).group(1))
        elif courant and ligne.startswith('+') and not ligne.startswith('+++'):
            ajouts.setdefault(courant, []).append((num, ligne[1:]))
            num += 1
    return ajouts


def chercher_secrets(chemin: str, lignes, rapport: Rapport):
    for num, texte in lignes:
        if MARQUEUR in texte:
            continue
        for motif, nature in SECRETS:
            if motif.search(texte):
                rapport.erreur(f'{chemin}:{num} : {nature} — ne jamais versionner un secret')


def chercher_valeurs_codees(chemin: str, lignes, rapport: Rapport):
    if not chemin.endswith(CODE) or chemin.startswith('tests/'):
        return
    for num, texte in lignes:
        if MARQUEUR in texte or texte.lstrip().startswith('#'):
            continue
        if IP_BANC.search(texte):
            rapport.erreur(f"{chemin}:{num} : IP du banc codée en dur — lire os.environ['MYCOBOT_PI'] "
                           '(voir .claude/rules/configuration.md)')
        if CHEMIN_PERSO.search(texte):
            rapport.erreur(f'{chemin}:{num} : chemin /home/... codé en dur — chemin relatif à la '
                           'racine du dépôt ou au paquet ROS')


def est_interdit(chemin: str) -> str | None:
    if chemin.endswith('.pth') and POIDS_ADMIS.match(chemin):
        return None
    for motif, raison in INTERDITS:
        if fnmatch.fnmatch(chemin, motif):
            return raison
    return None


def est_pointeur_lfs(chemin: str) -> bool:
    attr = git('check-attr', 'filter', '--', chemin)
    return attr.strip().endswith('lfs')


def liens_casses(fichiers_md, suivis: set[str]) -> list[str]:
    tolerees = set()
    if LIENS_TOLERES.exists():
        tolerees = {l.strip() for l in LIENS_TOLERES.read_text().splitlines()
                    if l.strip() and not l.startswith('#')}
    casses = []
    for f in fichiers_md:
        p = RACINE / f
        if not p.exists() or f.startswith(('training/dream/DREAM/', 'datasets/')):
            continue
        for m in re.finditer(r'\[[^\]]*\]\(([^)\s#]+)', p.read_text(errors='ignore')):
            cible_brute = m.group(1)
            if re.match(r'^[a-z][a-z0-9+.-]*:', cible_brute):
                continue
            try:
                cible = str((p.parent / cible_brute).resolve().relative_to(RACINE))
            except ValueError:
                continue
            # Un fichier doit être versionné : présent sur ce poste seulement, il
            # cassera sur un clone neuf et dans la CI. Un dossier existant suffit.
            if cible in tolerees or cible in suivis or (RACINE / cible).is_dir():
                continue
            casses.append(f'{f} -> {cible_brute}')
    return casses


# --- modes -------------------------------------------------------------------

def verifier_index(rapport: Rapport):
    etat = git('diff', '--cached', '--name-status', '--no-renames').splitlines()
    ajoutes_modifies = [l.split('\t', 1)[1] for l in etat if l[0] in 'AM']
    if not etat:
        rapport.avertir('rien d\'indexé')
        return

    for chemin in ajoutes_modifies:
        raison = est_interdit(chemin)
        if raison:
            rapport.erreur(f'{chemin} : fichier interdit ({raison}) — git restore --staged {chemin}')
        taille = int(git('cat-file', '-s', f':{chemin}').strip() or 0)
        if taille > TAILLE_MAX_OCTETS and not est_pointeur_lfs(chemin):
            rapport.erreur(f'{chemin} : {taille / 1e6:.1f} Mo hors Git LFS (limite '
                           f'{TAILLE_MAX_OCTETS // 1_000_000} Mo)')

    for chemin, lignes in lignes_ajoutees_indexees().items():
        chercher_secrets(chemin, lignes, rapport)
        chercher_valeurs_codees(chemin, lignes, rapport)

    suivis = set(git('ls-files').split())
    md = [c for c in ajoutes_modifies if c.endswith('.md')]
    for lien in liens_casses(md, suivis | set(ajoutes_modifies)):
        rapport.erreur(f'lien cassé : {lien}')

    code = [c for c in ajoutes_modifies if c.endswith('.py')
            and c.startswith(('mycobot_gateway/', 'mycobot_description/', 'scripts/',
                              'training/', 'teleop/'))]
    if code and 'CHANGELOG.md' not in ajoutes_modifies:
        rapport.avertir(f'{len(code)} fichier(s) de code modifié(s) sans CHANGELOG.md dans le même '
                        'commit (règle du dépôt si le changement est visible)')

    index = (RACINE / 'INDEX.md').read_text(errors='ignore') if (RACINE / 'INDEX.md').exists() else ''
    for c in etat:
        statut, chemin = c.split('\t', 1)
        if statut == 'A' and re.match(r'^(docs|training)/.*\.md$', chemin) \
                and Path(chemin).name not in index:
            rapport.avertir(f'{chemin} : nouveau document absent de INDEX.md')


def verifier_message(fichier: Path, rapport: Rapport):
    texte = '\n'.join(l for l in fichier.read_text().splitlines() if not l.startswith('#')).strip()
    if not texte:
        return
    sujet = texte.splitlines()[0]
    if sujet.startswith(('Merge ', 'Revert "', 'fixup! ', 'squash! ')):
        return
    if not SUJET.match(sujet):
        rapport.erreur(f'sujet « {sujet} » : attendu « type(portée): résumé », type parmi '
                       f'{TYPES_COMMIT.replace("|", ", ")} (CONTRIBUTING.md § Commits)')
    if len(sujet) > 72:
        rapport.erreur(f'sujet de {len(sujet)} caractères : 72 au plus')
    if ATTRIBUTION.search(texte):
        rapport.erreur('ligne d\'attribution (Co-Authored-By, « Generated with »…) : un commit '
                       'ne porte que le nom de son auteur humain')


def verifier_complet(rapport: Rapport):
    suivis = git('ls-files').split()
    ensemble = set(suivis)
    for lien in liens_casses([f for f in suivis if f.endswith('.md')], ensemble):
        rapport.erreur(f'lien cassé : {lien}')

    dette_ip, dette_chemin = set(), set()
    for f in suivis:
        p = RACINE / f
        if f.startswith(('datasets/', 'training/dream/DREAM/')) or not p.is_file() \
                or p.stat().st_size > 2_000_000:
            continue
        try:
            texte = p.read_text()
        except (UnicodeDecodeError, OSError):
            continue
        chercher_secrets(f, enumerate(texte.splitlines(), 1), rapport)
        if f.endswith(CODE) and not f.startswith('tests/'):
            if IP_BANC.search(texte):
                dette_ip.add(f)
            if CHEMIN_PERSO.search(texte):
                dette_chemin.add(f)
    if dette_ip:
        rapport.avertir(f'dette : {len(dette_ip)} fichier(s) de code avec une IP du banc codée')
    if dette_chemin:
        rapport.avertir(f'dette : {len(dette_chemin)} fichier(s) de code avec un chemin /home/...')


def main() -> int:
    a = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    mode = a.add_mutually_exclusive_group(required=True)
    mode.add_argument('--indexe', action='store_true', help='fichiers indexés (pre-commit)')
    mode.add_argument('--message', type=Path, help='fichier du message de commit (commit-msg)')
    mode.add_argument('--complet', action='store_true', help='tout le dépôt (CI, audit)')
    args = a.parse_args()

    rapport = Rapport()
    if args.indexe:
        print('verifier_depot : fichiers indexés')
        verifier_index(rapport)
    elif args.message:
        verifier_message(args.message, rapport)
        if not rapport.erreurs:
            return 0
        print('verifier_depot : message de commit')
    else:
        print('verifier_depot : dépôt complet')
        verifier_complet(rapport)
    return rapport.conclure()


if __name__ == '__main__':
    sys.exit(main())
