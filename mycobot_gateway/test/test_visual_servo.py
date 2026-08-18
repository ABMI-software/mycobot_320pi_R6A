#!/usr/bin/env python3
"""Essais de validation de l'asservissement visuel (§14), partie logicielle.

Le tableau du §14 mélange des essais qui exigent le robot (taux de préhension,
latence réelle du bus) et des essais qui se décident dans la logique de commande.
Ce fichier couvre les seconds, sur un modèle de pince à intégrateur simple — pas
de ROS2, pas de caméra, pas de bras :

    Répétabilité                dispersion de 20 estimations d'un objet fixe
    Convergence statique        ‖e(t)‖ décroît jusqu'au seuil
    Objet déplacé lentement     la pince suit la NOUVELLE position
    Occultation de l'ArduCam    la SVPRO prend le relais sans à-coup
    Perte des deux caméras      arrêt dans le délai imparti
    Latence                     la compensation §8.2 réduit le retard de traînage

Ce qui reste à mesurer sur matériel : erreur de la transformation caméra-base sur
des points physiques (produite par calibrate_camera_base_extrinsic.py, section
validation leave-one-out) et le taux de préhension.

    python -m pytest mycobot_gateway/test/test_visual_servo.py -v
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mycobot_gateway.visual_servo.fusion import (  # noqa: E402
    CameraMeasurement, FusionConfig, MultiCameraFusion)
from mycobot_gateway.visual_servo.object_tracker import (  # noqa: E402
    ObjectTracker, TargetLock, TrackerConfig)
from mycobot_gateway.visual_servo.safety import (  # noqa: E402
    SafetyLimits, SafetyState, SafetySupervisor, Verdict)
from mycobot_gateway.visual_servo.servo_law import (  # noqa: E402
    ConvergenceMonitor, ServoGains, compute_command)
from mycobot_gateway.visual_servo.state_machine import (  # noqa: E402
    Action, Context, MissionConfig, PickStateMachine, State)

DT = 0.1                      # 10 Hz, la cadence visée par le §11.3
GAINS = ServoGains()


def measurement(camera, position, stamp, **kwargs):
    defaults = {'detector_score': 0.9, 'pixel_area': 500.0, 'sharpness': 0.9,
                'reprojection_px': 0.5}
    defaults.update(kwargs)
    return CameraMeasurement(camera=camera, position=np.asarray(position, float),
                             stamp=stamp, **defaults)


def run_servo(target_of_t, ee0, steps, gains=GAINS, velocity_of_t=None):
    """Boucle fermée sur une pince à intégrateur parfait — retourne (‖e‖, positions).

    Le modèle est volontairement idéal : on teste la LOI, pas le robot. Un défaut
    qui apparaît ici apparaîtra a fortiori sur le vrai bras.
    """
    ee = np.asarray(ee0, dtype=np.float64).copy()
    errors, path = [], []
    for k in range(steps):
        target = np.asarray(target_of_t(k * DT), dtype=np.float64)
        velocity = np.zeros(3) if velocity_of_t is None else velocity_of_t(k * DT)
        command = compute_command(ee, target, velocity, DT, gains)
        ee = ee + command.step
        errors.append(float(np.linalg.norm(ee - target)))
        path.append(ee.copy())
    return np.array(errors), np.array(path)


# — Répétabilité ————————————————————————————————————————————————————

def _half_drift(samples: np.ndarray) -> float:
    return float(np.linalg.norm(samples[:10].mean(axis=0) - samples[10:].mean(axis=0)))


def test_repeatabilite_objet_fixe():
    """20 estimations d'un objet immobile : faible dispersion, aucune dérive.

    Le seuil de dérive est calculé, pas choisi : l'écart entre les moyennes de
    deux moitiés de 10 tirages a lui-même un écart-type de σ·√(2/10) par axe. Un
    seuil « rond » sous cette valeur ne testerait que le tirage aléatoire. On
    prend 3 écarts-types, et on vérifie sur un contre-exemple qu'une VRAIE dérive
    le franchit — sans quoi le test passerait quoi qu'il arrive.
    """
    rng = np.random.default_rng(7)
    truth = np.array([0.30, 0.05, 0.02])
    sigma = 0.004
    lock = TargetLock(n_samples=20)
    samples = []
    for _ in range(20):
        noisy = truth + rng.normal(0.0, sigma, 3)
        samples.append(noisy)
        lock.add(noisy)
    samples = np.array(samples)

    assert lock.locked is not None, 'le verrou doit se poser après 20 échantillons'
    assert np.linalg.norm(lock.locked - truth) < 0.003, 'biais du verrou > 3 mm'
    assert lock.dispersion() < 0.010, 'dispersion > 10 mm'

    limit = 3.0 * sigma * np.sqrt(2.0 / 10.0) * np.sqrt(3)
    assert _half_drift(samples) < limit, \
        f'dérive de {_half_drift(samples) * 1000:.1f} mm (> {limit * 1000:.1f} mm)'

    drifting = samples + np.outer(np.arange(20) * 0.002, np.array([1.0, 0.0, 0.0]))
    assert _half_drift(drifting) > limit, \
        'le critère ne détecte même pas une dérive franche — seuil trop lâche'


def test_verrou_rejette_les_aberrantes():
    """Une mesure aberrante isolée ne doit pas déplacer la pose verrouillée."""
    truth = np.array([0.30, 0.05, 0.02])
    lock = TargetLock(n_samples=8)
    for i in range(7):
        lock.add(truth + np.array([0.001 * i, 0.0, 0.0]))
    lock.add(truth + np.array([0.25, 0.25, 0.0]))       # aberrante franche
    assert np.linalg.norm(lock.locked - truth) < 0.01, \
        'la médiane robuste a été tirée par une aberrante'


# — Convergence statique ————————————————————————————————————————————

def test_convergence_statique():
    """‖e(t)‖ décroît régulièrement jusqu'au seuil, sans dépassement."""
    target = np.array([0.30, 0.10, 0.08])
    errors, _ = run_servo(lambda t: target, [0.20, -0.05, 0.20], steps=120)

    assert errors[-1] < GAINS.deadband * 1.5, \
        f'pas convergé : {errors[-1] * 1000:.1f} mm restants'
    # Décroissance monotone à la tolérance numérique près : une loi proportionnelle
    # saturée ne doit jamais faire remonter l'erreur.
    assert np.all(np.diff(errors) <= 1e-9), 'l\'erreur remonte — dépassement'
    # Décroissance STRICTE tant qu'on est loin du but ; une fois dans la bande
    # morte l'erreur stagne, c'est voulu — comparer là n'aurait aucun sens.
    moving = errors[errors > GAINS.deadband * 1.5]
    assert len(moving) > 10 and moving[0] > moving[len(moving) // 2] > moving[-1]


def test_moniteur_detecte_la_divergence():
    """Une erreur qui croît doit être signalée, pas subie."""
    monitor = ConvergenceMonitor(threshold=0.006, required_frames=5)
    for e in np.linspace(0.02, 0.05, 14):
        monitor.push(float(e))
    assert monitor.diverging, 'divergence non détectée'

    monitor.reset()
    for e in list(np.linspace(0.05, 0.008, 9)) + [0.003] * 5:
        monitor.push(float(e))
    assert not monitor.diverging
    assert monitor.converged, 'convergence non reconnue'


# — Objet déplacé —————————————————————————————————————————————————

def test_objet_deplace_pendant_l_approche():
    """La pince doit rallier la NOUVELLE position, pas l'ancienne (§2.1 vs §8)."""
    first = np.array([0.30, 0.10, 0.08])
    second = np.array([0.34, -0.06, 0.08])

    def target(t):
        return first if t < 2.0 else second

    errors, path = run_servo(target, [0.20, 0.0, 0.20], steps=200)
    final = path[-1]
    assert np.linalg.norm(final - second) < 0.005, \
        f'la pince a fini à {final} au lieu de la nouvelle cible {second}'
    assert np.linalg.norm(final - first) > 0.10, \
        'la pince est restée sur l\'ancienne position — boucle ouverte'


def test_anticipation_reduit_le_retard_de_trainage():
    """V_O dans la loi §8.3 doit réduire l'erreur de poursuite d'une cible mobile."""
    speed = np.array([0.03, 0.0, 0.0])           # 30 mm/s vers l'avant

    def target(t):
        return np.array([0.28, 0.0, 0.08]) + speed * t

    with_ff, _ = run_servo(target, [0.28, 0.0, 0.08], steps=150,
                           velocity_of_t=lambda t: speed)
    without_ff, _ = run_servo(
        target, [0.28, 0.0, 0.08], steps=150,
        gains=ServoGains(feedforward=0.0), velocity_of_t=lambda t: speed)

    assert with_ff[-1] < without_ff[-1], 'l\'anticipation n\'aide pas'
    assert with_ff[-1] < 0.002, \
        f'retard résiduel de {with_ff[-1] * 1000:.1f} mm malgré l\'anticipation'


def test_compensation_de_latence():
    """P + V·τ doit viser devant l'objet, du bon côté et de la bonne quantité."""
    tracker = ObjectTracker(TrackerConfig(measurement_std=0.002))
    speed = np.array([0.05, 0.0])
    for k in range(40):
        t = k * DT
        tracker.update(np.array([0.25, 0.0]) + speed * t, t, confidence=1.0)

    assert np.allclose(tracker.velocity, speed, atol=0.01), \
        f'vitesse mal estimée : {tracker.velocity}'
    tau = 0.15
    predicted = tracker.predict_position(tau)
    expected = tracker.position + speed * tau
    assert np.linalg.norm(predicted - expected) < 0.002


# — Occultation ————————————————————————————————————————————————————

def test_occultation_arducam_relais_svpro():
    """Vue supérieure perdue : la latérale reprend, sans saut de cible (§7.2)."""
    fusion = MultiCameraFusion('arducam')
    truth = np.array([0.30, 0.08, 0.02])

    result = fusion.fuse([measurement('arducam', truth, 0.0),
                          measurement('svpro', truth + [0.004, 0.0, 0.0], 0.0)], 0.0)
    assert result.mode == 'FUSION' and set(result.contributing) == {'arducam', 'svpro'}
    before = result.position

    # L'arducam devient aveugle : seule la svpro publie encore.
    result = fusion.fuse([measurement('svpro', truth + [0.004, 0.0, 0.0], 0.1)], 0.1)
    assert result.mode == 'MONO' and result.contributing == ('svpro',)
    jump = float(np.linalg.norm(result.position - before))
    assert jump < 0.005, f'à-coup de {jump * 1000:.1f} mm au basculement'


def test_mesure_perimee_ecartee():
    """Une image trop ancienne ne doit pas peser, même si sa détection est belle."""
    fusion = MultiCameraFusion('arducam', FusionConfig(max_age=0.25))
    result = fusion.fuse([measurement('arducam', [0.3, 0.0, 0.02], 0.0)], now=0.5)
    assert result.mode == 'PERDU', 'une image de 0,5 s a été acceptée'


def test_fusion_ignore_une_vue_incoherente():
    """Une vue qui saute loin reçoit un poids quasi nul (§7.2)."""
    fusion = MultiCameraFusion('arducam')
    truth = np.array([0.30, 0.08, 0.02])
    fusion.fuse([measurement('arducam', truth, 0.0)], 0.0)

    result = fusion.fuse([measurement('arducam', truth, 0.1),
                          measurement('svpro', truth + [0.30, 0.0, 0.0], 0.1)], 0.1)
    assert result.per_camera['svpro'] == 0.0, 'la vue aberrante a gardé du poids'
    assert np.linalg.norm(result.position - truth) < 0.005


def test_gate_kalman_finit_par_accepter_un_deplacement_reel():
    """Un objet vraiment déplacé ne doit pas rester rejeté indéfiniment."""
    tracker = ObjectTracker(TrackerConfig(gate_max_rejects=4))
    for k in range(20):
        tracker.update([0.25, 0.0], k * DT)
    moved = np.array([0.40, 0.10])
    accepted = [tracker.update(moved, (20 + k) * DT) for k in range(6)]
    assert accepted[0] is False, 'le saut a été gobé immédiatement'
    assert any(accepted), 'le filtre est resté bloqué sur l\'ancienne position'
    assert np.linalg.norm(tracker.position - moved) < 0.02


# — Sûreté ————————————————————————————————————————————————————————

def test_perte_des_deux_cameras_arrete_dans_le_delai():
    """Au-delà du délai sans mesure : ARRÊT (§12, §8.3)."""
    limits = SafetyLimits(max_lost_time=0.40)
    supervisor = SafetySupervisor(limits)

    ok = supervisor.check(SafetyState(now=1.0, time_since_measurement=0.2))
    assert ok.verdict is Verdict.OK

    late = supervisor.check(SafetyState(now=1.0, time_since_measurement=0.45))
    assert late.verdict is Verdict.STOP
    assert any('LOST' in r for r in late.reasons)


@pytest.mark.parametrize('state,expected', [
    (SafetyState(now=0.0, image_age=0.5), 'STALE'),
    (SafetyState(now=0.0, position_jump=0.5), 'JUMP'),
    (SafetyState(now=0.0, reprojection_px=20.0), 'REPROJECTION'),
    (SafetyState(now=0.0, target_speed=1.5), 'TARGET_TOO_FAST'),
    (SafetyState(now=0.0, diverging=True), 'DIVERGENCE'),
    (SafetyState(now=0.0, joint_angles_deg=np.array([169.0, 0, 0, 0, 0, 0])),
     'JOINT_LIMIT'),
    (SafetyState(now=0.0, commanded_position=np.array([2.0, 0.0, 0.2])), 'WORKSPACE'),
    (SafetyState(now=0.0, commanded_position=np.array([0.3, 0.0, -0.005])), 'TABLE'),
])
def test_chaque_condition_du_paragraphe_12_arrete(state, expected):
    report = SafetySupervisor().check(state)
    assert report.verdict is Verdict.STOP, f'{expected} n\'a pas arrêté'
    assert any(expected in r for r in report.reasons)


def test_descente_autorise_le_contact_table():
    """La descente doit pouvoir approcher la table, sinon la saisie est impossible."""
    state = SafetyState(now=0.0, commanded_position=np.array([0.3, 0.0, 0.002]),
                        allow_table_contact=True)
    assert SafetySupervisor().check(state).verdict is Verdict.OK


def test_marge_table_validee_de_26_mm_est_obligatoire_hors_descente():
    report = SafetySupervisor().check(SafetyState(
        now=0.0, commanded_position=np.array([0.3, 0.0, 0.025])))
    assert report.verdict is Verdict.STOP
    assert any('TABLE' in reason for reason in report.reasons)

    report = SafetySupervisor().check(SafetyState(
        now=0.0, commanded_position=np.array([0.3, 0.0, 0.026])))
    assert report.verdict is Verdict.OK


def test_fusion_ne_moyenne_pas_deux_cameras_en_desaccord():
    """Un biais inter-caméras doit sélectionner la primaire, pas créer un faux XY."""
    fusion = MultiCameraFusion('arducam', FusionConfig(max_age=1.0))
    primary = CameraMeasurement('arducam', np.array([0.254, 0.011, 0.035]), 0.0,
                                pixel_area=800.0)
    lateral = CameraMeasurement('svpro', np.array([0.273, 0.010, 0.035]), 0.0,
                                pixel_area=800.0)
    result = fusion.fuse([primary, lateral], now=0.1)
    assert result.mode == 'MONO'
    assert result.contributing == ('arducam',)
    assert np.allclose(result.position, primary.position)


def test_robot_qui_ne_suit_pas_est_detecte():
    """Bridge qui répond OK sans que le bras bouge : incohérence commande/mesure."""
    supervisor = SafetySupervisor(SafetyLimits(follow_window=3))
    ee = np.array([0.30, 0.0, 0.20])
    verdict = None
    for k in range(4):
        supervisor.register_command(ee, ee + np.array([0.01, 0.0, 0.0]), now=0.0)
        verdict = supervisor.check(SafetyState(now=0.6 + k * 0.1,
                                                ee_position=ee))  # ne bouge pas
    assert verdict.verdict is Verdict.STOP
    assert any('NOT_FOLLOWING' in r for r in verdict.reasons)


def test_encodeurs_plus_lents_que_commandes_ne_declenchent_pas_faux_arret():
    """Plusieurs envois avant la trame encodeur doivent garder la 1re référence."""
    supervisor = SafetySupervisor(SafetyLimits(follow_window=5))
    start = np.array([0.30, 0.0, 0.20])
    target = start + np.array([0.010, 0.0, 0.0])

    # Deux commandes à 10 Hz, encore une seule mesure encodeur à 5 Hz.
    assert supervisor.register_command(start, target, now=0.0)
    assert supervisor.check(SafetyState(now=0.0, ee_position=start)).verdict is Verdict.OK
    assert not supervisor.register_command(
        start, target + np.array([0.002, 0.0, 0.0]), now=0.1)
    assert supervisor.check(SafetyState(now=0.1, ee_position=start)).verdict is Verdict.OK

    # La trame suivante confirme au moins 60 % du premier incrément : acquitté.
    moved = start + np.array([0.007, 0.0, 0.0])
    assert supervisor.check(SafetyState(now=0.2, ee_position=moved)).verdict is Verdict.OK
    assert not supervisor._pending
    assert supervisor._not_following == 0


def test_proximite_butee_ralentit_avant_d_arreter():
    """Marge intermédiaire : ralentir, pas arrêter — sinon le bras se fige souvent."""
    supervisor = SafetySupervisor()
    report = supervisor.check(SafetyState(
        now=0.0, joint_angles_deg=np.array([165.0, 0, 0, 0, 0, 0])))
    assert report.verdict is Verdict.SLOW
    assert 0.0 < report.speed_scale < 1.0


def test_limite_reelle_j2_arrete_avant_refus_firmware():
    """J2 ±137° est la butée Pi : le domaine pratique s'arrête à ±135°."""
    report = SafetySupervisor().check(SafetyState(
        now=0.0, joint_angles_deg=np.array([0.0, -134.0, 0, 0, 0, 0])))
    assert report.verdict is Verdict.STOP
    assert any('JOINT_LIMIT J2' in reason for reason in report.reasons)


# — Machine à états ————————————————————————————————————————————————

def base_context(now=0.0, **kwargs):
    defaults = {
        'ee_position': np.array([0.25, 0.0, 0.20]),
        'object_position': np.array([0.30, 0.05, 0.02]),
        'measurement_valid': True,
        'confidence': 0.9,
        'time_since_measurement': 0.0,
        'error_norm': 0.20,
    }
    defaults.update(kwargs)
    return Context(now=now, **defaults)


def test_sequence_nominale_complete():
    """SEARCH → … → DONE sur une cible immobile idéale."""
    machine = PickStateMachine(MissionConfig(mobile_target=False))
    locked = np.array([0.30, 0.05, 0.02])

    for k in range(5):
        machine.step(base_context(now=k * DT))
    assert machine.state is State.TRACK

    directive = machine.step(base_context(now=0.6, locked_position=locked))
    assert machine.state is State.APPROACH
    assert directive.action is Action.RELEASE

    machine.step(base_context(now=0.7, locked_position=locked, error_norm=0.010))
    assert machine.state is State.FINE_SERVO

    machine.step(base_context(now=0.8, locked_position=locked, error_norm=0.003,
                              converged=True))
    assert machine.state is State.DESCEND

    directive = machine.step(base_context(
        now=0.9, locked_position=locked, ee_position=np.array([0.30, 0.05, 0.012])))
    assert machine.state is State.GRASP

    directive = machine.step(base_context(now=1.0, locked_position=locked))
    assert directive.action is Action.GRIP

    # Fermeture, puis passe de serrage, puis seulement LIFT.
    directive = machine.step(base_context(now=1.1, locked_position=locked,
                                          gripper_closed=True))
    assert directive.action is Action.GRIP and directive.firm
    machine.step(base_context(now=1.1 + MissionConfig().grasp_firm_settle,
                              locked_position=locked, gripper_closed=True))
    assert machine.state is State.LIFT

    # Levage en deux temps : 5 mm de contrôle, stabilisation, puis transport.
    grasped = np.array([0.30, 0.05, 0.012])
    machine.step(base_context(now=3.0, ee_position=grasped, gripper_closed=True))
    verified = np.array([0.30, 0.05, 0.017])
    machine.step(base_context(now=4.2, ee_position=verified, gripper_closed=True))
    machine.step(base_context(now=4.3, ee_position=np.array([0.30, 0.05, 0.12]),
                              gripper_closed=True))
    assert machine.state is State.PLACE

    place = np.asarray(MissionConfig().place_position)
    directive = machine.step(base_context(now=4.4, ee_position=place,
                                          gripper_closed=True))
    assert directive.action is Action.RELEASE
    machine.step(base_context(now=4.5, ee_position=place, gripper_closed=True))
    assert machine.state is State.DONE


def test_approche_seule_termine_au_pregrasp_sans_descendre_ni_saisir():
    machine = PickStateMachine(MissionConfig(stop_at_pregrasp=True))
    machine.state = State.FINE_SERVO
    directive = machine.step(base_context(
        now=1.0, error_norm=0.003, converged=True,
        locked_position=np.array([0.30, 0.05, 0.02])))

    assert machine.state is State.DONE
    assert directive.action is Action.IDLE
    assert 'descente désactivée' in directive.reason

    # Une frame suivante reste inerte : aucune transition différée vers
    # DESCEND/GRASP n'est possible après l'arrêt au pré-grasp.
    directive = machine.step(base_context(now=1.1, converged=True))
    assert machine.state is State.DONE
    assert directive.action is Action.IDLE


def test_pince_est_ouverte_obligatoirement_avant_approche():
    machine = PickStateMachine(MissionConfig())
    machine.state = State.TRACK
    directive = machine.step(base_context(
        now=1.0, locked_position=np.array([0.30, 0.05, 0.02])))
    assert machine.state is State.APPROACH
    assert directive.action is Action.RELEASE


def test_perte_en_approche_passe_en_reacquire_et_stoppe_le_robot():
    """Cible perdue sans verrou : on ne continue PAS vers la dernière position."""
    machine = PickStateMachine(MissionConfig(mobile_target=True))
    machine.state = State.APPROACH
    machine.entered_at = 0.0

    directive = machine.step(base_context(now=1.0, measurement_valid=False,
                                          object_position=None))
    assert machine.state is State.REACQUIRE
    directive = machine.step(base_context(now=1.1, measurement_valid=False,
                                          object_position=None))
    assert directive.action is Action.IDLE, \
        'le robot bouge encore alors que la cible est perdue'


def test_reacquire_expire_en_safe_stop():
    machine = PickStateMachine(MissionConfig(reacquire_timeout=1.0))
    machine.state = State.REACQUIRE
    machine.entered_at = 0.0
    directive = machine.step(base_context(now=1.5, measurement_valid=False,
                                          object_position=None))
    assert machine.state is State.SAFE_STOP
    assert directive.action is Action.STOP


def test_cible_mobile_ne_verrouille_jamais():
    """Sur cible mobile, coasting interdit : ce serait la boucle ouverte du §2.1."""
    machine = PickStateMachine(MissionConfig(mobile_target=True))
    machine.state = State.APPROACH
    machine.entered_at = 0.0
    machine.step(base_context(now=0.5, measurement_valid=False, object_position=None,
                              locked_position=np.array([0.30, 0.05, 0.02])))
    assert machine.state is State.REACQUIRE, \
        'la machine a continué sur une pose verrouillée alors que la cible peut bouger'


def test_arret_de_surete_prime_sur_tout():
    machine = PickStateMachine()
    machine.state = State.DESCEND
    directive = machine.step(base_context(now=1.0, safety_stop=True,
                                          safety_reasons=('LOST test',)))
    assert machine.state is State.SAFE_STOP
    assert directive.action is Action.STOP


def test_prise_perdue_pendant_la_montee():
    """Pince ouverte pendant le transport = objet tombé : ne pas aller à PLACE.

    Distinct de `test_levage_controle_detecte_la_perte`, qui couvre la perte dès
    les 5 mm de contrôle. Ici la prise a passé la vérification et lâche plus haut.
    """
    machine = PickStateMachine()
    machine.state = State.LIFT
    machine.entered_at = 0.0
    machine._lift_origin_z = 0.012
    machine._lift_verified = True
    directive = machine.step(base_context(
        now=1.0, ee_position=np.array([0.30, 0.05, 0.12]), gripper_closed=False))
    assert machine.state is State.SAFE_STOP
    assert directive.action is Action.STOP


def test_saisie_exige_plusieurs_images_sous_le_seuil():
    """Une seule mesure bonne ne doit pas déclencher la descente (§12)."""
    machine = PickStateMachine(MissionConfig(fine_frames=5))
    machine.state = State.FINE_SERVO
    machine.entered_at = 0.0
    monitor = ConvergenceMonitor(threshold=0.006, required_frames=5)

    monitor.push(0.003)
    machine.step(base_context(now=0.1, error_norm=0.003, converged=monitor.converged))
    assert machine.state is State.FINE_SERVO, 'descente déclenchée sur une seule image'

    for _ in range(4):
        monitor.push(0.003)
    machine.step(base_context(now=0.2, error_norm=0.003, converged=monitor.converged))
    assert machine.state is State.DESCEND


# — Saturations ————————————————————————————————————————————————————

def test_pas_borne_meme_apres_une_periode_perdue():
    """Un dt anormal (détecteur qui a hoqueté) ne doit pas produire un bond."""
    command = compute_command(np.array([0.20, 0.0, 0.20]), np.array([0.40, 0.0, 0.20]),
                              np.zeros(3), dt=0.8, gains=GAINS)
    assert np.linalg.norm(command.step) <= GAINS.max_step + 1e-12
    assert command.saturated


def test_bande_morte_ne_commande_rien():
    command = compute_command(np.array([0.30, 0.0, 0.08]),
                              np.array([0.3005, 0.0, 0.08]), np.zeros(3), DT, GAINS)
    assert command.idle
    assert np.allclose(command.step, 0.0)


# — Leçons de l'essai réel du 18/08 ————————————————————————————————

def test_descente_gele_xy():
    """XY ne doit plus bouger une fois la convergence atteinte.

    L'essai du 18/08 avait un centrage XY correct et une pince perpendiculaire
    au pré-grasp ; continuer d'asservir XY pendant la descente a dégradé ce
    centrage et la pince n'a attrapé que le haut de la balle.
    """
    machine = PickStateMachine(MissionConfig(freeze_xy_on_descend=True))
    machine.state = State.FINE_SERVO
    machine.entered_at = 0.0
    converged_ee = np.array([0.300, 0.050, 0.085])

    machine.step(base_context(now=0.1, ee_position=converged_ee, error_norm=0.003,
                              converged=True))
    assert machine.state is State.DESCEND

    # l'objet « bouge » ensuite de 4 cm : la descente ne doit PAS suivre
    moved = np.array([0.340, 0.010, 0.020])
    directive = machine.step(base_context(
        now=0.2, ee_position=converged_ee, object_position=moved, error_norm=0.003))
    assert directive.action is Action.SERVO
    assert np.allclose(directive.target[:2], converged_ee[:2], atol=1e-9), \
        f'XY a bougé pendant la descente : {directive.target[:2]} au lieu de {converged_ee[:2]}'


def test_descente_par_paliers():
    """Chaque période descend d'un palier borné, pas d'un trait vers la saisie."""
    cfg = MissionConfig(descend_step=0.005, grasp_height=0.012)
    machine = PickStateMachine(cfg)
    machine.state = State.DESCEND
    machine.entered_at = 0.0
    machine._descend_xy = np.array([0.300, 0.050])

    z = 0.085
    for _ in range(3):
        d = machine.step(base_context(now=1.0, ee_position=np.array([0.300, 0.050, z])))
        assert d.action is Action.SERVO
        assert z - d.target[2] <= cfg.descend_step + 1e-9, 'palier trop grand'
        assert d.allow_table_contact, 'la descente doit autoriser l\'approche table'
        z = d.target[2]
    assert z < 0.085, 'la pince n\'est pas descendue'


def test_levage_controle_detecte_la_perte():
    """5 mm de levage suffisent à révéler une prise qui ne tient pas."""
    cfg = MissionConfig(verify_lift_height=0.005, verify_lift_settle=0.0)
    machine = PickStateMachine(cfg)
    machine.state = State.LIFT
    machine.entered_at = 0.0

    start = np.array([0.300, 0.050, 0.012])
    d = machine.step(base_context(now=0.1, ee_position=start, gripper_closed=True))
    assert d.action is Action.SERVO
    assert abs(d.target[2] - (start[2] + cfg.verify_lift_height)) < 1e-9, \
        'le premier levage doit faire exactement la hauteur de contrôle'

    lifted = np.array([0.300, 0.050, 0.017])
    d = machine.step(base_context(now=0.5, ee_position=lifted, gripper_closed=False))
    assert machine.state is State.SAFE_STOP, 'perte de prise non détectée à 5 mm'
    assert d.action is Action.STOP


def test_serrage_apres_contact_confirme():
    """Le premier contact ne suffit pas : une 2e passe serre avant tout levage."""
    cfg = MissionConfig(grasp_firm=True, grasp_firm_settle=1.8)
    machine = PickStateMachine(cfg)
    machine.state = State.GRASP
    machine.entered_at = 0.0

    d = machine.step(base_context(now=0.0))
    assert d.action is Action.GRIP and not d.firm, 'la fermeture doit venir en 1er'

    d = machine.step(base_context(now=0.3, gripper_closed=True))
    assert d.action is Action.GRIP and d.firm, 'pas de serrage après le contact'
    assert machine.state is State.GRASP, 'on ne lève pas avant la fin du serrage'

    d = machine.step(base_context(now=1.0, gripper_closed=True))
    assert d.action is Action.IDLE and machine.state is State.GRASP

    machine.step(base_context(now=2.2, gripper_closed=True))
    assert machine.state is State.LIFT


def test_serrage_ne_declenche_pas_expiration():
    """Le statut repasse par « en mouvement » pendant le serrage : pas un échec."""
    cfg = MissionConfig(grasp_firm=True, grasp_firm_settle=1.8, grasp_timeout=1.0)
    machine = PickStateMachine(cfg)
    machine.state = State.GRASP
    machine.entered_at = 0.0

    machine.step(base_context(now=0.0))
    machine.step(base_context(now=0.3, gripper_closed=True))       # serrage envoyé
    machine.step(base_context(now=1.5, gripper_closed=False))      # « en mouvement »
    assert machine.state is State.GRASP, \
        'le délai a expiré pendant le serrage — il est compté depuis le mauvais instant'

    machine.step(base_context(now=2.2, gripper_closed=True))
    assert machine.state is State.LIFT


def test_serrage_desactivable():
    cfg = MissionConfig(grasp_firm=False)
    machine = PickStateMachine(cfg)
    machine.state = State.GRASP
    machine.entered_at = 0.0

    machine.step(base_context(now=0.0))
    machine.step(base_context(now=0.3, gripper_closed=True))
    assert machine.state is State.LIFT


def test_statut_pince_inconnu_bloque_le_levage():
    """Un statut indisponible n'est PAS une vérification de prise réussie.

    Régression : `is False` laissait passer `None` — le bridge de la Pi qui
    n'implémente pas get_pro_gripper_status rendait le contrôle silencieusement
    inopérant, exactement sur l'étape censée détecter une prise douteuse.
    """
    cfg = MissionConfig(verify_lift_height=0.005, verify_lift_settle=0.0)
    machine = PickStateMachine(cfg)
    machine.state = State.LIFT
    machine.entered_at = 0.0

    start = np.array([0.300, 0.050, 0.012])
    machine.step(base_context(now=0.1, ee_position=start, gripper_closed=True))
    lifted = np.array([0.300, 0.050, 0.017])
    d = machine.step(base_context(now=0.5, ee_position=lifted, gripper_closed=None))

    assert machine.state is State.SAFE_STOP
    assert d.action is Action.STOP
    assert 'INVÉRIFIABLE' in ' '.join(machine.stop_reasons)


def test_levage_controle_puis_transport():
    """Prise confirmée à 5 mm : on continue jusqu'à la hauteur de transport."""
    cfg = MissionConfig(verify_lift_height=0.005, verify_lift_settle=0.0,
                        lift_height=0.12)
    machine = PickStateMachine(cfg)
    machine.state = State.LIFT
    machine.entered_at = 0.0
    start = np.array([0.300, 0.050, 0.012])

    machine.step(base_context(now=0.1, ee_position=start, gripper_closed=True))
    lifted = np.array([0.300, 0.050, 0.017])
    machine.step(base_context(now=0.5, ee_position=lifted, gripper_closed=True))
    d = machine.step(base_context(now=0.6, ee_position=lifted, gripper_closed=True))
    assert d.action is Action.SERVO and d.target[2] > 0.1, \
        'le transport ne reprend pas après vérification'
    assert machine.state is State.LIFT
