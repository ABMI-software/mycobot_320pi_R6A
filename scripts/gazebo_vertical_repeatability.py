"""Prepare and report a physical gripper-down, straight-line repeatability trial."""
import csv
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from diff_ik import fk_pose

TARGET_VERTICAL = np.array([250., 31.843, 60.])
TOOL_VERTICAL = np.array([-1., 7.8, 166.])
ROT_VERTICAL = np.diag([1., -1., -1.])


def make_vertical_path():
    """41 continuous IK waypoints with a smooth 40 mm Z-only rise, fingers down."""
    # Limites MESUREES en pratique (deg), identiques a celles de l'URDF :
    # J1 168, J2 135, J3 150, J4 145, J5 165, J6 180. Elles bornent le domaine
    # atteignable, pas la repetabilite — ni le jeu, ni les couples, ni le pas
    # des codeurs n'en decoulent.
    lower = -np.array([168., 135., 150., 145., 165., 180.])
    upper = -lower
    seed = np.array([28., -15., -75., 0., 90., -62.])
    path = []
    for t in np.linspace(0., 1., 41):
        target = TARGET_VERTICAL + [0., 0., 40. * (3*t*t - 2*t*t*t)]
        def residual(q):
            p, r = fk_pose(q)
            return np.r_[p + r @ TOOL_VERTICAL - target,
                         Rotation.from_matrix(ROT_VERTICAL @ r.T).as_rotvec() * 400]
        result = least_squares(residual, seed, bounds=(lower, upper), max_nfev=300,
                               xtol=1e-11, ftol=1e-11, gtol=1e-11)
        if np.linalg.norm(result.fun) > 0.02:
            raise ValueError(f'Vertical IK failed at {target}: {result.fun}')
        if path and np.max(np.abs(result.x - path[-1])) > 5:
            raise ValueError('IK branch jump in vertical path')
        path.append(result.x.tolist())
        seed = result.x
    return path


def prepare_vertical(output, xlsx, mass, arm_mass):
    from gazebo_precision_campaign import ROOT, dump, prepare
    prepare(output, xlsx, mass, arm_mass)
    plan = json.loads((output / 'plan.json').read_text())
    path = make_vertical_path()
    plan.update({
        'protocol': 'vertical_gripper_down',
        'target_mm': TARGET_VERTICAL.tolist(),
        'tcp_mm': TOOL_VERTICAL.tolist(),
        'orientation_matrix': ROT_VERTICAL.tolist(),
        'inclination_deg': 0.,
        'groups': [['A-vertical', 'haut', 10]],
        'poses': {
            'target': {'q_deg': path[0], 'target_mm': TARGET_VERTICAL.tolist()},
            'haut': {'q_deg': path[-1], 'target_mm': (TARGET_VERTICAL + [0,0,40]).tolist()},
        },
        'vertical_waypoints_up_deg': path,
        'vertical_segment_duration_s': 3.,
        'historical_comparison_valid': False,
        'limitations': [
            'Real gripper is vertical according to the user. New simulated trial: target X=250 mm and physically vertical gripper, different from the previously reconstructed target at X=330.476 mm.',
            'The physical pad-centre TCP of the current simulated geometry is used; gripper CAD is still approximate.',
            f'Arm {arm_mass} kg + gripper {mass} kg nominal masses; segment COMs, inertias and servo response are not identified.',
            'No real measurements for this new pose: no direct validation of the historical 0.403 mm RP.',
            'Ten returns to one pose, position-controlled rigid model: not ISO conformity certification.',
        ],
    })
    plan.pop('historical_tcp_mm', None)
    dump(output / 'plan.json', plan)
    config = (ROOT / 'mycobot_description/config/trajectory_gui.config').read_text()
    config = config.replace('Trajectoire pince — bleu', 'Pince verticale — retours par le haut')
    # Side view makes the downward orientation and the vertical line legible.
    config = config.replace('1.2 -1.5 1.0 0 0.4 2.2', '0.8 -1.1 0.65 0 0.32 2.05')
    (output / 'gui.config').write_text(config)
    # Non-colliding reference ball: marks the point approached, no grasp load.
    tree = ET.parse(output / 'world.sdf')
    world = tree.getroot().find('world')
    marker = ET.SubElement(world, 'model', name='cible_repetabilite')
    ET.SubElement(marker, 'static').text = 'true'
    ET.SubElement(marker, 'pose').text = ' '.join(str(v / 1000) for v in TARGET_VERTICAL) + ' 0 0 0'
    link = ET.SubElement(marker, 'link', name='reference')
    visual = ET.SubElement(link, 'visual', name='cible')
    sphere = ET.SubElement(ET.SubElement(visual, 'geometry'), 'sphere')
    ET.SubElement(sphere, 'radius').text = '0.004'
    material = ET.SubElement(visual, 'material')
    ET.SubElement(material, 'ambient').text = '1 0.1 0.1 1'
    ET.SubElement(material, 'diffuse').text = '1 0.1 0.1 1'
    tree.write(output / 'world.sdf', encoding='utf-8', xml_declaration=True)
    print('VERTICAL: 10 retours, cible [250, 31.843, 60] mm, montée 40 mm, pince vers le bas.', flush=True)


def report_vertical(output, samples, plan):
    from gazebo_precision_campaign import JOINTS, dump, metrics, model_masses, check_requested_masses
    expected = sum(g[2] for g in plan['groups'])
    if len(samples) != expected or any(s['series'] != 'A-vertical' for s in samples):
        raise ValueError('Incomplete vertical trial')
    actual = model_masses((output / 'robot.urdf').read_text())
    check_requested_masses(actual, plan['arm_mass_kg'], plan['gripper_mass_kg'])
    rounded = metrics([s['xyz_fk_rounded_mm'] for s in samples], TARGET_VERTICAL)
    raw = metrics([s['xyz_fk_mm'] for s in samples], TARGET_VERTICAL)
    with (output / 'trajectory.csv').open() as f:
        trace = [r for r in csv.DictReader(f) if r['series'] == 'A-vertical/haut']
    xyz = np.array([[float(r[k]) for k in ('x_mm', 'y_mm', 'z_mm')] for r in trace])
    down_errors = []
    for row in trace:
        _, rot = fk_pose([float(row[j]) for j in JOINTS])
        down_errors.append(float(np.degrees(np.arccos(np.clip(-rot[2,2], -1, 1)))))
    lateral = float(np.linalg.norm(xyz[:, :2] - TARGET_VERTICAL[:2], axis=1).max())
    flagged = sum(not s['settled'] or s['arrival_motion']['action_code'] != 0
                  or s['departure_motion']['action_code'] != 0 for s in samples)
    result = {'control_mode': plan.get('control_mode', 'position'), 'protocol': plan['protocol'], 'historical_comparison_valid': False,
              'rounded': rounded, 'unrounded': raw, 'loaded_masses': actual,
              'flagged_arrivals': flagged, 'max_line_deviation_mm': lateral,
              'max_downward_orientation_error_deg': max(down_errors),
              'limitations': plan['limitations']}
    result['measurement_source'] = '/joint_states.position (simulated joint feedback)'
    result['encoder_rounding_deg'] = 0.01
    result['joint_peak_to_peak_deg'] = np.ptp([s['q_read_deg'] for s in samples], axis=0).tolist()
    with (output / 'lectures_encodeurs.csv').open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['arrival', 'sim_s'] + ['q_read_' + j + '_deg' for j in JOINTS]
                        + ['tcp_x_mm', 'tcp_y_mm', 'tcp_z_mm'])
        for sample in samples:
            writer.writerow([sample['trial'], sample['sim_s'], *sample['q_read_deg'], *sample['xyz_fk_mm']])
    dump(output / 'comparison.json', result)
    lines = ['# Répétabilité — pince verticale, arrivée par le haut', '',
             f"{len(samples)} retours, masse totale chargée **{actual['arm_and_gripper_kg']:.3f} kg**.",
             'Cible : (250 ; 31,843 ; 60) mm. Montée et descente de 40 mm, orientation maintenue vers le bas.',
             'La sphère rouge est un repère visuel sans collision, pas un objet saisi.', '',
             'Mesures : six positions articulaires de /joint_states à chaque arrivée stabilisée, '
             'converties en position du point de prise par cinématique directe. Les consignes '
             'ne sont pas utilisées comme observations. Lectures brutes : lectures_encodeurs.csv.',
             'L’arrondi à 0,01° est un post-traitement, pas un modèle de résolution des encodeurs physiques.', '',
             '| Mesure simulée | Valeur |', '|---|---:|',
             f"| RP après arrondi des angles à 0,01° | {rounded['rp_mm']:.6f} mm |",
             f"| Erreur moyenne à la cible après arrondi | {rounded['mean_error_mm']:.6f} mm |",
             f"| RP sans arrondi | {raw['rp_mm']:.6e} mm |",
             f"| Écart maximal à la ligne verticale, pendant les cycles | {lateral:.6f} mm |",
             f"| Écart maximal d'orientation à la verticale | {max(down_errors):.6f}° |",
             f'| Arrivées signalées (stabilisation / action) | {flagged} |', '',
             '**Ce nouvel essai ne se compare pas directement au RP historique de 0,403 mm : '
             'la cible et le point outil diffèrent de la reconstruction précédente.** Une mesure réelle avec '
             'ces mêmes conditions est nécessaire pour valider la simulation.', '',
             ('Modèle dynamique en couple avec correcteur PD : poids, inerties et frottements agissent sur le mouvement. '
              'Gains et couples limites sont des hypothèses non identifiées sur le robot réel. Aucun bruit ajouté. '
              if plan.get('control_mode') == 'effort' else
              'Le modèle rigide en commande de position ne représente pas encore les imperfections des actionneurs réels. ')
             + 'Ce résultat ne certifie pas ±0,5 mm sur le robot réel.', '',
             '## Hypothèses', ''] + ['- ' + s for s in plan['limitations']]
    (output / 'rapport.md').write_text('\n'.join(lines) + '\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(10, 4.5), layout='constrained')
    axes[0].plot(xyz[:,0],xyz[:,2],color='royalblue')
    axes[0].plot(TARGET_VERTICAL[0],TARGET_VERTICAL[2],'ro',label='Cible')
    axes[0].set(xlabel='X (mm)',ylabel='Z (mm)',title='Trajet vertical du point de prise',
                xlim=(TARGET_VERTICAL[0]-25,TARGET_VERTICAL[0]+25),ylim=(50,110))
    axes[0].set_aspect('equal');axes[0].legend();axes[0].grid(True)
    axes[1].plot([float(r['sim_s']) for r in trace],xyz[:,2],color='royalblue')
    axes[1].set(xlabel='Temps simulé (s)',ylabel='Z (mm)',title='Dix descentes vers la même cible')
    axes[1].grid(True)
    fig.suptitle(f"Pince vers le bas — bras + pince {actual['arm_and_gripper_kg']:.3f} kg")
    fig.savefig(output / 'trajectoires.png',dpi=160)
    print('\n'.join(lines[:19]),flush=True)
