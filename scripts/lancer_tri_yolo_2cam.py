#!/usr/bin/env python3
"""Le tri reel : YOLO par l'arducam, la SVPRO en appui quand l'arducam ne voit pas.

    MYCOBOT_PI=10.10.0.223 /usr/bin/python3 scripts/lancer_tri_yolo_2cam.py --dream

Exactement `lancer_pick_dashboard_final.py` sans `--svpro-dream` (aucun fichier
du dashboard n'est modifie) : « calibrer » calibre l'arducam ET la SVPRO. Seule
difference : si la calibration SVPRO n'a pas pu se faire (il lui faut les 4
marqueurs), la SVPRO prend `svpro_extrinsic_labels.yaml` (4 ArUco, 13/10) plutot
que `svpro_extrinsic_servo.yaml` perime. Controle du 15/10 : servo 39-54 mm,
labels 4-5 mm. servo n'est prise que recente ET propre (voir RMS_SERVO_MAX_PX).

L'arducam donne seule la position des objets (plan fixe HAUTEUR_OBJET, comme
avec --svpro-dream) et des bacs. La SVPRO ne fusionne rien : ni Z triangule
(`hauteur_triangulee` coupee), ni bac (`bacs_arducam_seule`). Elle reste l'appui
de `pick_dashboard` (`points_interessants`) : quand le bras cache la vue de
dessus, elle dit qu'il reste quelque chose, sans viser de saisie.
"""
import sys
from pathlib import Path

import yaml

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE / 'scripts'))
import lancer_pick_dashboard_final as lanceur  # noqa: E402
import pick_dashboard  # noqa: E402
import yolo26_dashboard  # noqa: E402

SERVO, LABELS = 'svpro_extrinsic_servo', 'svpro_extrinsic_labels'


# Seuil de `calibrate_svpro_readonly.py`. Le 15/10 15:28 « calibrer » a ecrit servo sur
# 3 marqueurs (aucun leave-one-out) : 5,7 px, et le 19 — cote robot, ou sont les
# pieces — a 12,3 mm, contre 4-6 mm partout pour labels.
RMS_SERVO_MAX_PX = 1.5


def extrinseque_svpro():
    """servo si « calibrer » vient de la reecrire (arducam ET SVPRO, au demarrage du
    lanceur) ET proprement, sinon labels. Decide a la construction de `Vision`, donc
    APRES la calibration du demarrage."""
    lit = lambda nom: pick_dashboard.CALIB / f'{nom}.yaml'
    rms = yaml.safe_load(lit(SERVO).read_text()).get('rms_reproj_px', float('inf'))
    plus_recente = lit(SERVO).stat().st_mtime > lit(LABELS).stat().st_mtime
    choix = SERVO if plus_recente and rms <= RMS_SERVO_MAX_PX else LABELS
    print(f'extrinseque SVPRO : {choix}.yaml ({SERVO} : {rms:.1f} px, '
          f'{"plus" if plus_recente else "moins"} recente que {LABELS})', flush=True)
    return choix


def main():
    if '--svpro-dream' in sys.argv:
        raise SystemExit('ce lanceur sert a utiliser YOLO sur la SVPRO : sans --svpro-dream')
    for nom in (SERVO, LABELS):
        # `Vision` retombe EN SILENCE sur arducam_extrinsic_servo si le fichier manque.
        if not (pick_dashboard.CALIB / f'{nom}.yaml').exists():
            raise SystemExit(f'{nom}.yaml introuvable')
    init = pick_dashboard.Vision.__init__

    def init_svpro(self, stem='arducam_extrinsic_pick'):
        init(self, extrinseque_svpro() if stem == SERVO else stem)

    pick_dashboard.Vision.__init__ = init_svpro
    juge_bacs = yolo26_dashboard.juge_bacs

    def bacs_arducam_seule(vision, *k, **kw):
        # 15/10 14:55 : bac_bleu cache a l'arducam par le bras, sa seule position venait
        # de la SVPRO, a 715-730 mm de la base -> largage refuse, cube garde en main.
        # Vue rasante : le centre de boite d'un bac ramene sur un plan est a 87 mm
        # (mediane, 14/10) et parfois a des centaines. La SVPRO ne place plus aucun bac.
        if not vision.source.startswith('arducam'):
            return []
        return juge_bacs(vision, *k, **kw)

    yolo26_dashboard.juge_bacs = bacs_arducam_seule
    # Pas de fusion des deux vues : sans Z triangule, juge_objets projette le XY de
    # l'arducam au plan HAUTEUR_OBJET et la hauteur de prise reste celle de pick_fsm.
    yolo26_dashboard.hauteur_triangulee = lambda *k, **kw: None
    lanceur.main()


if __name__ == '__main__':
    main()
