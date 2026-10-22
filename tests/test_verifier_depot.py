"""Les garde-fous de scripts/verifier_depot.py, sur les cas réels du dépôt."""
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import verifier_depot as v  # noqa: E402


def secrets(texte):
    r = v.Rapport()
    v.chercher_secrets('f.md', [(1, texte)], r)
    return r.erreurs


def valeurs(chemin, texte):
    r = v.Rapport()
    v.chercher_valeurs_codees(chemin, [(1, texte)], r)
    return r.erreurs


def message(texte):
    with tempfile.NamedTemporaryFile('w', suffix='.txt', delete=False) as f:
        f.write(texte)
    r = v.Rapport()
    v.verifier_message(Path(f.name), r)
    return r.erreurs


class Secrets(unittest.TestCase):
    def test_les_deux_formes_du_mot_de_passe_vnc_publie(self):
        # Valeur assemblée à l'exécution : aucune ligne du dépôt ne porte le secret.
        valeur = 'Ele' + 'phant'
        self.assertTrue(secrets(f'> **VNC** (mot de passe `{valeur}`), clavier en panne'))
        self.assertTrue(secrets(f'| VNC | `<ip>:5900` | **Mot de passe VNC : `{valeur}`** |'))

    def test_un_mot_de_passe_absent_ou_redirige_passe(self):
        self.assertFalse(secrets("mot de passe : demander à l'équipe, il n'est pas versionné"))
        self.assertFalse(secrets('⚠️ mot de passe système **inconnu/oublié** — SSH inutilisable'))
        self.assertFalse(secrets('password = os.environ["PI_PASSWORD"]'))

    def test_cles_et_jetons(self):
        self.assertTrue(secrets('-----BEGIN OPENSSH ' + 'PRIVATE KEY-----'))
        self.assertTrue(secrets('token = "ghp_' + 'a' * 36 + '"'))

    def test_le_marqueur_justifie_une_exception(self):
        self.assertFalse(secrets('mot de passe `' + 'Ele' + 'phant`  <!-- verifier: ok : exemple -->'))


class ValeursCodees(unittest.TestCase):
    def test_ip_du_banc_dans_le_code(self):
        self.assertTrue(valeurs('scripts/x.py', "PI = ('10.10.0.224', 5005)"))

    def test_ip_dans_un_commentaire_ou_un_document(self):
        self.assertFalse(valeurs('scripts/x.py', '# la Pi était en 10.10.0.218 le 22/10'))
        self.assertFalse(valeurs('docs/x.md', 'Pi en 10.10.0.218'))

    def test_chemin_personnel(self):
        self.assertTrue(valeurs('mycobot_gateway/n.py',
                                'return "/home/genji/Osama_ws/src/mycobot_R6A/training"'))

    def test_les_tests_et_le_marqueur(self):
        self.assertFalse(valeurs('tests/test_x.py', "IP = '10.10.0.1'"))
        self.assertFalse(valeurs('scripts/x.py', "DEFAUT = '10.10.0.224'  # verifier: ok"))


class FichiersInterdits(unittest.TestCase):
    def test_interdits(self):
        for chemin in ('SESSION_RESUME.md.bak2', 'results/tri/essai.csv',
                       'docs/PI_REMOTE_ACCESS.md', 'training/dream/x/model.pth', '.env'):
            self.assertIsNotNone(v.est_interdit(chemin), chemin)

    def test_checkpoints_admis_et_fichiers_ordinaires(self):
        self.assertIsNone(v.est_interdit(
            'training/dream/checkpoints_dream/vgg_tri_reel_ft_e10/best_network.pth'))
        self.assertIsNone(v.est_interdit('scripts/pick_fsm.py'))


class MessageDeCommit(unittest.TestCase):
    def test_convention(self):
        self.assertFalse(message('docs: remise en ordre de la documentation\n\nCorps.'))
        self.assertFalse(message('fix(dream): tri_yolo demarre hors du poste d origine'))
        self.assertTrue(message('mise à jour'))
        self.assertTrue(message('feat: ' + 'x' * 80))

    def test_aucune_ligne_d_attribution(self):
        self.assertTrue(message('feat: x\n\nCo-Authored-By: Quelqu\'un <a@b.c>'))

    def test_fusions_et_commentaires_git(self):
        self.assertFalse(message("Merge branch 'main' into feature/x"))
        self.assertFalse(message('# ligne de commentaire de git\n'))


if __name__ == '__main__':
    unittest.main()
