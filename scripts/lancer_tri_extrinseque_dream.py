#!/usr/bin/env python3
"""Le tri reel avec les extrinseques trouvees par DREAM, sans aucun marqueur.

    MYCOBOT_PI=10.10.0.223 /usr/bin/python3 scripts/lancer_tri_extrinseque_dream.py --svpro-dream --dream

Exactement `lancer_pick_dashboard_final.py` (memes options, memes fichiers,
rien n'y est modifie), a une difference pres : partout ou une extrinseque
ArUco servait, c'est celle de DREAM (`extrinseque_dream_session.py`, DREAM
seul, calculee une fois sur un tri) :
  - YOLO ramene ses pixels sur la table par `arducam_extrinsic_dream_tri.yaml` ;
  - les detections de cote de `pick_dashboard` par `svpro_extrinsic_dream_tri.yaml` ;
  - le panneau DREAM compare chaque image a ces memes extrinseques DREAM : la
    pince DREAM(t) = inv(T_DREAM calibre) . T_DREAM(t) . pince, contre les
    codeurs. Aucun marqueur nulle part ; l'ArUco n'intervient plus que si on
    clique « verifier l'extrinseque ».

Et deux aides au test, sans toucher au tri : les messages de la boucle (« la
pince tient deja un objet », « la boucle du dashboard tourne ») partent dans le
« Journal du tri » au lieu du seul terminal — sinon « demarrer la boucle »
semble ne rien faire ; et une erreur pendant une saisie arrete la boucle
proprement (la piece comptee en echec) au lieu de garder son verrou, qui
ignorerait ensuite tous les clics.

Mesure avant ce test (tri du 14/10 18:02, 229 images) : DREAM contre ArUco
4,7 mm / 1,14 deg ; sur la table 7,4 mm median, 10,6 mm max, decalage presque
constant d'environ (-5 ; -5) mm.
"""
import sys
import traceback
from pathlib import Path

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE / 'scripts'))
import lancer_pick_dashboard_final as lanceur  # noqa: E402
import dream_tri_panneau  # noqa: E402
import pick_dashboard  # noqa: E402
import tri_fluide  # noqa: E402

EXTRINSEQUES_DREAM = {'arducam_extrinsic_pick': 'arducam_extrinsic_dream_tri',
                      'svpro_extrinsic_servo': 'svpro_extrinsic_dream_tri'}
PANNEAU_DREAM = {'arducam': 'arducam_extrinsic_dream_tri.yaml', 'svpro': 'svpro_extrinsic_dream_tri.yaml'}


def installe_et_rend_visible(module, *k, **kw):
    panneau = installe_panneau(module, *k, **kw)
    lanceur.print = panneau.ecrit
    trie = tri_fluide.trie

    def trie_sans_blocage(ctx, piece, xy, bac_xy, variante='actuelle'):
        try:
            return trie(ctx, piece, xy, bac_xy, variante)
        except Exception as erreur:  # noqa: BLE001 — tout arret doit liberer la boucle
            traceback.print_exc()
            panneau.ecrit(f'{piece} : erreur pendant le tri ({erreur!r}) — boucle arretee')
            return False

    tri_fluide.trie = trie_sans_blocage
    return panneau


installe_panneau = dream_tri_panneau.installe


def main():
    for nom in EXTRINSEQUES_DREAM.values():
        # `Vision` retombe EN SILENCE sur arducam_extrinsic_servo si le fichier manque :
        # le test mesurerait alors une autre extrinseque que celle annoncee.
        if not (pick_dashboard.CALIB / f'{nom}.yaml').exists():
            raise SystemExit(f'{nom}.yaml introuvable : lancer extrinseque_dream_session.py --ecrire')
    init = pick_dashboard.Vision.__init__

    def init_dream(self, stem='arducam_extrinsic_pick'):
        init(self, EXTRINSEQUES_DREAM.get(stem, stem))

    pick_dashboard.Vision.__init__ = init_dream
    dream_tri_panneau.EXTRINSEQUES = PANNEAU_DREAM
    dream_tri_panneau.installe = installe_et_rend_visible
    for avant, apres in EXTRINSEQUES_DREAM.items():
        print(f'extrinseque : {apres}.yaml au lieu de {avant}.yaml (DREAM seul, aucun marqueur)', flush=True)
    lanceur.main()


if __name__ == '__main__':
    main()
