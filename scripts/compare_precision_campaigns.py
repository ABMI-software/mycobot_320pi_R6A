#!/usr/bin/env python3
"""Compare two complete Gazebo campaigns with unchanged motion protocol."""
import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from gazebo_precision_campaign import model_masses

PROTOCOL_KEYS = [
    'target_mm', 'historical_tcp_mm', 'orientation_matrix', 'inclination_deg',
    'poses', 'groups', 'max_joint_velocity_deg_s', 'minimum_move_duration_s',
    'settle_minimum_s', 'settle_maximum_s', 'settle_window_s',
    'settle_velocity_rad_s', 'settle_range_deg', 'encoder_rounding_deg',
    'physics_step_s', 'requested_real_time_factor', 'gripper_close_command_rad',
]


def read_run(folder):
    folder = Path(folder)
    status = json.loads((folder / 'status.json').read_text())
    if not status['complete'] or status['arrivals'] != 56:
        raise ValueError(f'Incomplete run: {folder}')
    return {
        'folder': folder,
        'plan': json.loads((folder / 'plan.json').read_text()),
        'reference': json.loads((folder / 'reference.json').read_text()),
        'comparison': json.loads((folder / 'comparison.json').read_text()),
        'mass': model_masses((folder / 'robot.urdf').read_text()),
    }


def compare(before, after):
    a, b = read_run(before), read_run(after)
    for key in PROTOCOL_KEYS:
        if a['plan'][key] != b['plan'][key]:
            raise ValueError(f'Motion/measurement protocol changed: {key}')
    if a['reference']['sha256'] != b['reference']['sha256']:
        raise ValueError('Real reference workbook differs')
    metrics = [('RP par le haut A', 'A/haut', 'rp_mm'),
               ('RP contrôle F-UNI', 'F-UNI/haut', 'rp_mm')]
    metrics += [('RP G ' + d, 'G/' + d, 'rp_mm') for d in ('devant','derriere','gauche','droite')]
    metrics += [('Étalement G (diagonale des barycentres)', 'G_inter', 'bbox_diagonal_mm'),
                ('Erreur moyenne à la cible B', 'B', 'mean_error_mm')]
    rows = []
    for title, group, metric in metrics:
        real = b['reference']['metrics'][group][metric]
        old = a['comparison']['simulated_rounded'][group][metric]
        new = b['comparison']['simulated_rounded'][group][metric]
        rows.append({'measure': title, 'real_mm': real, 'old_mm': old, 'new_mm': new,
                     'change_mm': new - old})
    output = b['folder']
    with (output / 'comparaison_masses.csv').open('w') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader();writer.writerows(rows)
    lines = ['# Effet de la correction de masse du bras', '',
             f"Ancien essai : bras {a['mass']['arm_kg']:.3f} kg + pince {a['mass']['gripper_kg']:.3f} kg "
             f"= **{a['mass']['arm_and_gripper_kg']:.3f} kg**.",
             f"Nouvel essai : bras {b['mass']['arm_kg']:.3f} kg + pince {b['mass']['gripper_kg']:.3f} kg "
             f"= **{b['mass']['arm_and_gripper_kg']:.3f} kg**.", '',
             'Masses lues dans les modèles URDF effectivement chargés. '
             'Le contrôle direct des liens dans Gazebo est enregistré dans '
             '`gazebo_link_properties.txt` et `gazebo_masses_verified.json` pour le nouvel essai.', '',
             'Les 17 paramètres de mouvement et de mesure comparés sont identiques, ainsi que '
             'le classeur réel de référence. Chaque campagne contient 56 arrivées.', '',
             '| Mesure | Réel (mm) | Ancienne masse (mm) | Masse corrigée (mm) | Variation (mm) |',
             '|---|---:|---:|---:|---:|']
    for row in rows:
        lines.append(f"| {row['measure']} | {row['real_mm']:.6f} | {row['old_mm']:.6f} | "
                     f"{row['new_mm']:.6f} | {row['change_mm']:+.6f} |")
    lines += ['', '## Ce que permet cet essai', '',
              'La masse nominale de 3 kg est celle de la version 320 Pi 2022. '
              'Il ne s’agit pas d’une pesée du montage individuel. Les masses et les inerties '
              'des sept liens du bras ont été multipliées par 3 / 1,58 ; leurs centres de masse '
              'restent ceux du modèle initial. La masse de la pince reste 0,340 kg.', '',
              'Un résultat numérique inférieur à 0,5 mm indique le comportement de ce modèle '
              'pour cet essai. Il ne certifie pas la répétabilité ±0,5 mm du robot physique. '
              'Le point outil est reconstruit par FK ; le montage réel du gripper, la répartition '
              'des masses, la réponse des servos sous charge, le jeu et la souplesse restent à identifier.', '',
              'Les essais A/F-UNI/G décrivent la répétabilité à sens constant. L’étalement entre '
              'directions et l’erreur moyenne à la cible sont des grandeurs distinctes, '
              'qui ne se comparent pas directement à la spécification de répétabilité.', '',
              '## Mesures non arrondies', '']
    for label, item in [('Ancien modèle', a), ('Masse corrigée', b)]:
        raw=item['comparison']['simulated_unrounded']
        lines.append(f"- {label} : erreur B = {raw['B']['mean_error_mm']:.6e} mm ; "
                     f"RP A = {raw['A/haut']['rp_mm']:.6e} mm.")
    lines += ['', 'L’arrondi des angles à 0,01° est appliqué après acquisition ; '
              'les valeurs proches de zéro ne constituent pas une mesure physique indépendante.', '',
              '[Rapport du nouvel essai](rapport.md) · [Trajectoires](trajectoires.png) · '
              '[Données comparées](comparaison_masses.csv)', '']
    (output / 'comparaison_masses.md').write_text('\n'.join(lines))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    chosen = [rows[0], rows[-2], rows[-1]]
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.5), layout='constrained')
    for ax, row in zip(axes, chosen):
        values = [row['real_mm'], row['old_mm'], row['new_mm']]
        labels = ['Réel', f"Gazebo\n{a['mass']['arm_and_gripper_kg']:.2f} kg", f"Gazebo\n{b['mass']['arm_and_gripper_kg']:.2f} kg"]
        ax.bar(labels, values, color=['#245b8e','#cf9446','#349779'])
        for i, value in enumerate(values):
            ax.text(i, value, f'{value:.3f}', ha='center', va='bottom')
        ax.set_title(row['measure']); ax.set_ylabel('mm')
        ax.set_ylim(0, max(values)*1.2 + .01)
    fig.suptitle('Même protocole — masse du bras corrigée, répartition et actionneurs non identifiés')
    fig.savefig(output / 'comparaison_masses.png', dpi=160)
    print(output / 'comparaison_masses.md')


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('before', type=Path);parser.add_argument('after', type=Path)
    args=parser.parse_args();compare(args.before,args.after)
