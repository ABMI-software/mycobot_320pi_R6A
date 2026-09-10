#!/usr/bin/env python3
"""Machine à états du pick-and-place asservi (§10).

    SEARCH      détection stable de l'objet        -> valide pendant N images
    TRACK       position + vitesse estimées        -> confiance suffisante
    APPROACH    correction continue vers le pré-grasp -> erreur < seuil
    REACQUIRE   priorité à la vue latérale         -> cible de nouveau valide
    FINE_SERVO  petites corrections finales        -> erreur faible N images
    DESCEND     descente lente, cible surveillée   -> hauteur de saisie
    GRASP       fermeture de la pince              -> confirmation
    LIFT        montée + vérification de la prise  -> hauteur de transport
    PLACE       dépôt et relâchement               -> objet déposé
    SAFE_STOP   arrêt sur perte / incohérence      -> réinitialisation

La machine ne parle ni à la caméra ni au robot : elle reçoit un `Context` mesuré
et rend une `Directive`. C'est ce qui la rend testable image par image sans
matériel, et c'est aussi ce qui garantit qu'aucune décision d'état ne se cache
dans le code de communication.

Deux chemins cohabitent, comme le demande le §10 :
  - cible IMMOBILE : TRACK fige une pose verrouillée (§7.1), qui permet de
    traverser une occultation courte ;
  - cible MOBILE : la cible reste rafraîchie pendant APPROACH et FINE_SERVO,
    et le verrou n'est jamais posé (sinon on retombe en boucle ouverte, §2.1).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

import numpy as np


class State(Enum):
    SEARCH = 'SEARCH'
    TRACK = 'TRACK'
    APPROACH = 'APPROACH'
    REACQUIRE = 'REACQUIRE'
    FINE_SERVO = 'FINE_SERVO'
    DESCEND = 'DESCEND'
    GRASP = 'GRASP'
    LIFT = 'LIFT'
    PLACE = 'PLACE'
    SAFE_STOP = 'SAFE_STOP'
    DONE = 'DONE'


class Action(Enum):
    IDLE = 'idle'                # rien à commander, on observe
    SERVO = 'servo'              # asservir vers `target` par petits incréments
    GRIP = 'grip'                # fermer la pince
    RELEASE = 'release'          # ouvrir la pince
    STOP = 'stop'                # arrêt immédiat


@dataclass(frozen=True)
class MissionConfig:
    """Géométrie de la mission. Hauteurs en mètres, repère base."""
    pregrasp_height: float = 0.065       # 50-80 mm au-dessus de l'objet (§7.1)
    grasp_height: float = 0.012          # hauteur des doigts à la saisie
    lift_height: float = 0.12            # hauteur de transport
    place_position: tuple = (0.24, -0.20, 0.10)
    approach_threshold: float = 0.020    # m — sortie d'APPROACH
    fine_threshold: float = 0.006        # m — sortie de FINE_SERVO
    fine_frames: int = 5                 # images consécutives sous le seuil
    search_frames: int = 5               # détections valides pour quitter SEARCH
    descend_speed: float = 0.020         # m/s — descente lente
    reacquire_timeout: float = 1.5       # s en REACQUIRE avant SAFE_STOP
    grasp_timeout: float = 3.0           # s d'attente de confirmation de pince
    mobile_target: bool = False          # True = objet susceptible de bouger
    lateral_camera: str = 'svpro'
    # S'arrêter une fois convergé au pré-grasp, sans descendre ni saisir. Sert
    # aux premiers essais en commande réelle : la boucle fermée complète est
    # exercée (mesure → erreur → incrément → nouvelle mesure) mais la pince ne
    # s'approche jamais de la table ni de l'objet.
    stop_at_pregrasp: bool = False
    # Descente : XY GELÉ, on ne commande plus que Z. Leçon de l'essai du 18/08 —
    # le centrage XY au pré-grasp était bon et la pince perpendiculaire ; en
    # continuant d'asservir XY pendant la descente on a perdu ce centrage, la
    # pince n'a attrapé que le quart supérieur de la balle et l'a laissée glisser
    # au levage. Une fois convergé, la bonne pose est ACQUISE : la corriger avec
    # des mesures que le bras occulte de plus en plus ne peut que la dégrader.
    freeze_xy_on_descend: bool = True
    descend_step: float = 0.005          # m par palier de descente
    # Levage en deux temps : 5 mm d'abord, on vérifie que l'objet est réellement
    # tenu, et seulement alors on monte à la hauteur de transport. Lever
    # directement de 120 mm avec une prise douteuse fait tomber l'objet de haut.
    verify_lift_height: float = 0.005    # m — levage de contrôle
    verify_lift_settle: float = 1.0      # s d'attente avant de juger la prise
    # Mesuré le 20/08 avec la balle en main : un levage commandé à +5 mm a donné
    # −2,6 mm réels — chargé, le bras s'affaisse plus qu'il ne monte. Sans délai
    # l'étape de vérification boucle indéfiniment sur une hauteur inatteignable.
    # Au-delà du délai on ne conclut pas à l'échec : on juge sur le statut pince.
    verify_lift_timeout: float = 4.0     # s
    # Passe de serrage — DÉSACTIVÉE par défaut : mesurée sans effet le 20/08.
    # Sur prise réelle la pince a calé à l'angle 52 (butée de couple atteinte au
    # premier contact) ; commander ensuite 12 au lieu de 20 l'a laissée à 52,
    # inchangée. Viser un angle plus bas ne serre donc PAS davantage — le seul
    # levier réel est `set_pro_gripper_torque`, que gripper_bridge.py expose.
    # L'option reste câblée pour une pince dont l'angle serait effectivement
    # asservi, mais l'activer ici ne coûte que grasp_firm_settle secondes.
    grasp_firm: bool = False
    grasp_firm_settle: float = 1.8       # s — la pince ignore un ordre < 1,6 s


@dataclass
class Context:
    """Tout ce que le nœud a mesuré cette période."""
    now: float
    ee_position: np.ndarray | None = None        # (3,) m — codeurs + FK
    object_position: np.ndarray | None = None    # (3,) m — fusion, compensée
    object_velocity: np.ndarray = field(default_factory=lambda: np.zeros(2))
    measurement_valid: bool = False
    confidence: float = 0.0
    contributing: tuple = ()
    time_since_measurement: float = float('inf')
    primary_visible: bool = False
    lateral_visible: bool = False
    error_norm: float = float('inf')
    converged: bool = False
    locked_position: np.ndarray | None = None
    gripper_closed: bool | None = None
    safety_stop: bool = False
    safety_reasons: tuple = ()


@dataclass
class Directive:
    action: Action
    target: np.ndarray | None = None
    fine: bool = False
    firm: bool = False           # GRIP : passe de serrage, pas la fermeture
    allow_table_contact: bool = False
    use_locked_target: bool = False
    reason: str = ''


class PickStateMachine:
    """Séquence complète. Une transition par appel de `step`."""

    def __init__(self, config: MissionConfig | None = None):
        self.cfg = config or MissionConfig()
        self.state = State.SEARCH
        self.entered_at = 0.0
        self.stop_reasons: tuple = ()
        self._stable_detections = 0
        self._approach_open_sent = False
        self._grasp_sent = False
        self._grasp_at = 0.0
        self._firm_sent = False
        self._firm_at = 0.0
        self._place_sent = False
        self._descend_xy: np.ndarray | None = None   # XY figé au pré-grasp
        self._lift_origin_z: float | None = None
        self._lift_verified = False

    def reset(self, now: float = 0.0):
        self.state = State.SEARCH
        self.entered_at = now
        self.stop_reasons = ()
        self._stable_detections = 0
        self._approach_open_sent = False
        self._grasp_sent = False
        self._grasp_at = 0.0
        self._firm_sent = False
        self._firm_at = 0.0
        self._place_sent = False
        self._descend_xy: np.ndarray | None = None   # XY figé au pré-grasp
        self._lift_origin_z: float | None = None
        self._lift_verified = False

    def _enter(self, state: State, now: float):
        if state is not self.state:
            self.state = state
            self.entered_at = now

    def elapsed(self, now: float) -> float:
        return now - self.entered_at

    def step(self, ctx: Context) -> Directive:
        """Fait avancer la machine d'une image et rend l'ordre à exécuter."""
        # La sûreté prime sur toute logique d'état, sauf une fois l'objet posé :
        # une perte de cible après PLACE n'est pas une anomalie.
        if ctx.safety_stop and self.state not in (State.SAFE_STOP, State.DONE):
            self.stop_reasons = ctx.safety_reasons
            self._enter(State.SAFE_STOP, ctx.now)
            return Directive(Action.STOP, reason=' | '.join(ctx.safety_reasons))

        handler = getattr(self, f'_on_{self.state.name.lower()}')
        return handler(ctx)

    # — recherche —————————————————————————————————————————————————————

    def _on_search(self, ctx: Context) -> Directive:
        if ctx.measurement_valid:
            self._stable_detections += 1
        else:
            self._stable_detections = 0
        if self._stable_detections >= self.cfg.search_frames:
            self._enter(State.TRACK, ctx.now)
            return Directive(Action.IDLE, reason='détection stable acquise')
        return Directive(Action.IDLE,
                         reason=f'recherche {self._stable_detections}/'
                                f'{self.cfg.search_frames}')

    # — poursuite —————————————————————————————————————————————————————

    def _on_track(self, ctx: Context) -> Directive:
        if not ctx.measurement_valid:
            self._enter(State.REACQUIRE, ctx.now)
            return Directive(Action.IDLE, reason='cible perdue en poursuite')
        # Sur cible immobile on attend le verrou (§7.1) ; sur cible mobile on
        # part tout de suite, la cible restera rafraîchie pendant l'approche.
        if not self.cfg.mobile_target and ctx.locked_position is None:
            return Directive(Action.IDLE, reason='verrouillage de la cible immobile')
        self._enter(State.APPROACH, ctx.now)
        # Une approche commence toujours pince ouverte : sinon les doigts
        # poussent la balle avant la fin du centrage et invalident la mesure.
        self._approach_open_sent = True
        return Directive(Action.RELEASE,
                         reason='ouverture obligatoire avant approche')

    # — approche ——————————————————————————————————————————————————————

    def _on_approach(self, ctx: Context) -> Directive:
        if not ctx.measurement_valid and not self._can_coast(ctx):
            self._enter(State.REACQUIRE, ctx.now)
            return Directive(Action.IDLE, reason='cible perdue en approche')
        if ctx.error_norm < self.cfg.approach_threshold:
            self._enter(State.FINE_SERVO, ctx.now)
        return Directive(
            Action.SERVO, target=self._pregrasp(ctx),
            use_locked_target=self._coasting(ctx),
            reason=f'approche e={ctx.error_norm * 1000:.0f}mm')

    def _on_reacquire(self, ctx: Context) -> Directive:
        """Vue supérieure masquée : la latérale reprend la main (§7.2, §10)."""
        if ctx.measurement_valid:
            self._enter(State.APPROACH, ctx.now)
            return Directive(Action.IDLE,
                             reason=f'cible réacquise via {",".join(ctx.contributing)}')
        if self.elapsed(ctx.now) > self.cfg.reacquire_timeout:
            self.stop_reasons = (
                f'REACQUIRE échouée après {self.cfg.reacquire_timeout:.1f}s',)
            self._enter(State.SAFE_STOP, ctx.now)
            return Directive(Action.STOP, reason=self.stop_reasons[0])
        # Le robot ne bouge pas pendant la réacquisition : avancer sans mesure
        # fraîche sur une cible qui peut avoir bougé, c'est la boucle ouverte.
        return Directive(Action.IDLE, reason='réacquisition — robot à l\'arrêt')

    # — asservissement fin ————————————————————————————————————————————

    def _on_fine_servo(self, ctx: Context) -> Directive:
        if not ctx.measurement_valid and not self._can_coast(ctx):
            self._enter(State.REACQUIRE, ctx.now)
            return Directive(Action.IDLE, reason='cible perdue en asservissement fin')
        if ctx.error_norm > self.cfg.approach_threshold * 2.0:
            self._enter(State.APPROACH, ctx.now)
            return Directive(Action.SERVO, target=self._pregrasp(ctx),
                             reason='erreur repartie — retour en approche')
        if ctx.converged:
            if self.cfg.stop_at_pregrasp:
                # Essai d'approche terminé avec succès. IDLE conserve les
                # servos sous tension à la pose atteinte ; STOP déclencherait
                # inutilement un arrêt d'urgence alors qu'il n'y a pas de faute.
                self._enter(State.DONE, ctx.now)
                return Directive(Action.IDLE,
                                 reason='pré-grasp atteint — descente désactivée')
            # On fige ici le XY RÉELLEMENT ATTEINT par la pince, pas celui que
            # la caméra vise : c'est cette pose-là qui vient d'être validée
            # perpendiculaire et centrée. « Conserver exactement cette pose. »
            if self.cfg.freeze_xy_on_descend and ctx.ee_position is not None:
                self._descend_xy = np.asarray(ctx.ee_position, dtype=np.float64)[:2].copy()
            self._enter(State.DESCEND, ctx.now)
            return Directive(Action.IDLE,
                             reason=f'convergence confirmée — XY figé à '
                                    f'({self._descend_xy[0] * 1000:.1f}, '
                                    f'{self._descend_xy[1] * 1000:.1f}) mm'
                             if self._descend_xy is not None else 'convergence confirmée')
        return Directive(Action.SERVO, target=self._pregrasp(ctx), fine=True,
                         reason=f'asservissement fin e={ctx.error_norm * 1000:.1f}mm')

    # — saisie ————————————————————————————————————————————————————————

    def _on_descend(self, ctx: Context) -> Directive:
        """Descente PUREMENT verticale, par paliers, XY figé au pré-grasp.

        Aucune correction latérale ici, volontairement : à mesure que la pince
        descend elle occulte l'objet, donc les mesures se dégradent exactement
        quand on serait tenté de les croire. Le centrage a été acquis au
        pré-grasp ; on n'y touche plus.
        """
        if ctx.ee_position is None:
            return Directive(Action.IDLE, reason='pas de pose effecteur')
        target = self._grasp_target(ctx)
        if ctx.ee_position[2] <= target[2] + self.cfg.fine_threshold:
            self._enter(State.GRASP, ctx.now)
            return Directive(Action.IDLE, reason='hauteur de saisie atteinte')

        # Un palier par période : la boucle réobserve caméra et codeurs entre
        # chaque palier au lieu de filer d'un trait vers la hauteur de saisie.
        step_target = np.asarray(ctx.ee_position, dtype=np.float64).copy()
        if self._descend_xy is not None:
            step_target[:2] = self._descend_xy
        step_target[2] = max(target[2],
                             ctx.ee_position[2] - self.cfg.descend_step)
        return Directive(Action.SERVO, target=step_target, fine=True,
                         allow_table_contact=True,
                         reason=f'descente z={ctx.ee_position[2] * 1000:.1f}'
                                f'→{step_target[2] * 1000:.1f}mm')

    def _on_grasp(self, ctx: Context) -> Directive:
        if not self._grasp_sent:
            self._grasp_sent = True
            self._grasp_at = ctx.now
            return Directive(Action.GRIP, allow_table_contact=True,
                             reason='fermeture de la pince')

        # Pendant le serrage la pince repasse par « en mouvement » : ni la
        # confirmation ni l'expiration n'ont de sens tant qu'il dure.
        if self._firm_sent and ctx.now - self._firm_at < self.cfg.grasp_firm_settle:
            return Directive(Action.IDLE, allow_table_contact=True,
                             reason='serrage en cours')

        if ctx.gripper_closed:
            if self.cfg.grasp_firm and not self._firm_sent:
                self._firm_sent = True
                self._firm_at = ctx.now
                return Directive(Action.GRIP, firm=True, allow_table_contact=True,
                                 reason='serrage sur contact établi')
            self._enter(State.LIFT, ctx.now)
            return Directive(Action.IDLE, reason='prise confirmée')

        # Le délai court depuis le dernier ordre envoyé, pas depuis l'entrée
        # dans l'état : le serrage dure à lui seul plus que le délai de
        # confirmation, et le compter depuis l'entrée le ferait expirer.
        since = ctx.now - (self._firm_at if self._firm_sent else self._grasp_at)
        if since > self.cfg.grasp_timeout:
            self.stop_reasons = (
                'GRASP objet chassé par le serrage' if self._firm_sent
                else 'GRASP sans confirmation de fermeture',)
            self._enter(State.SAFE_STOP, ctx.now)
            return Directive(Action.STOP, reason=self.stop_reasons[0])
        return Directive(Action.IDLE, allow_table_contact=True,
                         reason='attente de confirmation de pince')

    def _on_lift(self, ctx: Context) -> Directive:
        """Levage en deux temps : 5 mm de contrôle, puis la hauteur de transport.

        Une prise qui ne tient que le haut de l'objet le laisse glisser dès
        qu'on le soulève — constaté le 18/08. Cinq millimètres suffisent à le
        révéler, et à cette hauteur l'objet retombe sans dommage. Monter les
        120 mm d'un trait, c'est ne l'apprendre qu'en haut.
        """
        if ctx.ee_position is None:
            return Directive(Action.IDLE, reason='pas de pose effecteur')

        if self._lift_origin_z is None:
            self._lift_origin_z = float(ctx.ee_position[2])

        verify_z = self._lift_origin_z + self.cfg.verify_lift_height
        # La tolérance ne peut pas être `fine_threshold` : à 6 mm elle dépasse le
        # levage de contrôle de 5 mm, et la hauteur cible serait déclarée
        # atteinte avant même d'avoir bougé — le contrôle ne contrôlerait rien.
        verify_tol = min(self.cfg.fine_threshold, self.cfg.verify_lift_height / 2.0)
        if not self._lift_verified:
            if ctx.ee_position[2] < verify_z - verify_tol \
                    and self.elapsed(ctx.now) < self.cfg.verify_lift_timeout:
                target = ctx.ee_position.copy()
                target[2] = verify_z
                return Directive(Action.SERVO, target=target, fine=True,
                                 allow_table_contact=True,
                                 reason=f'levage de contrôle '
                                        f'{self.cfg.verify_lift_height * 1000:.0f}mm')
            # Hauteur de contrôle atteinte : laisser l'objet se stabiliser avant
            # d'interroger la pince — un statut lu trop tôt dit encore « saisi ».
            if self.elapsed(ctx.now) < self.cfg.verify_lift_settle:
                return Directive(Action.IDLE, allow_table_contact=True,
                                 reason='stabilisation avant vérification')
            # `is not True` et non `is False` : un statut INCONNU n'est pas une
            # vérification. Si le bridge de la Pi ne répond pas à
            # get_pro_gripper_status, `gripper_closed` reste None et la version
            # précédente laissait passer — la vérification demandée devenait un
            # no-op silencieux, précisément sur le contrôle censé attraper une
            # prise douteuse. On s'arrête, l'objet est à 5 mm de la table.
            if ctx.gripper_closed is not True:
                self.stop_reasons = (
                    'LIFT objet perdu dès les premiers millimètres — la pince '
                    'ne tenait que le haut de l\'objet' if ctx.gripper_closed is False
                    else 'LIFT statut de pince indisponible — prise INVÉRIFIABLE '
                         '(bridge sans get_pro_gripper_status ?)',)
                self._enter(State.SAFE_STOP, ctx.now)
                return Directive(Action.STOP, reason=self.stop_reasons[0])
            self._lift_verified = True
            return Directive(Action.IDLE, reason='prise confirmée sous charge')

        # La perte se surveille à CHAQUE période, pas seulement une fois arrivé
        # en haut : l'objet peut lâcher n'importe où sur la montée, et continuer
        # à monter une pince vide ne fait que l'éloigner du sol.
        if ctx.gripper_closed is not True:
            self.stop_reasons = ('LIFT prise perdue pendant la montée',)
            self._enter(State.SAFE_STOP, ctx.now)
            return Directive(Action.STOP, reason=self.stop_reasons[0])
        if ctx.ee_position[2] >= self.cfg.lift_height - self.cfg.fine_threshold:
            self._enter(State.PLACE, ctx.now)
            return Directive(Action.IDLE, reason='hauteur de transport atteinte')
        target = ctx.ee_position.copy()
        target[2] = self.cfg.lift_height
        return Directive(Action.SERVO, target=target, fine=True, reason='montée')

    def _on_place(self, ctx: Context) -> Directive:
        target = np.asarray(self.cfg.place_position, dtype=np.float64)
        if ctx.ee_position is None:
            return Directive(Action.IDLE, reason='pas de pose effecteur')
        if float(np.linalg.norm(ctx.ee_position - target)) < self.cfg.approach_threshold:
            if not self._place_sent:
                self._place_sent = True
                return Directive(Action.RELEASE, reason='ouverture de la pince')
            self._enter(State.DONE, ctx.now)
            return Directive(Action.IDLE, reason='objet déposé')
        return Directive(Action.SERVO, target=target, reason='transfert vers le dépôt')

    # — terminaux —————————————————————————————————————————————————————

    def _on_safe_stop(self, ctx: Context) -> Directive:
        return Directive(Action.STOP, reason=' | '.join(self.stop_reasons)
                         or 'arrêt de sûreté')

    def _on_done(self, ctx: Context) -> Directive:
        return Directive(Action.IDLE, reason='mission terminée')

    # — cibles ————————————————————————————————————————————————————————

    def _pregrasp(self, ctx: Context) -> np.ndarray | None:
        source = self._target_source(ctx)
        if source is None:
            return None
        target = np.asarray(source, dtype=np.float64).copy()
        target[2] += self.cfg.pregrasp_height
        return target

    def _grasp_target(self, ctx: Context) -> np.ndarray:
        """Pose de saisie : XY figé au pré-grasp si disponible, Z = hauteur de saisie."""
        if self._descend_xy is not None:
            return np.array([self._descend_xy[0], self._descend_xy[1],
                             self.cfg.grasp_height])
        source = self._target_source(ctx)
        base = np.zeros(3) if source is None else np.asarray(source, dtype=np.float64)
        target = base.copy()
        target[2] = self.cfg.grasp_height
        return target

    def _target_source(self, ctx: Context):
        """Mesure fraîche si disponible, sinon pose verrouillée (objet immobile)."""
        if ctx.measurement_valid and ctx.object_position is not None:
            return ctx.object_position
        return ctx.locked_position if not self.cfg.mobile_target else None

    def _coasting(self, ctx: Context) -> bool:
        return not ctx.measurement_valid and ctx.locked_position is not None

    def _can_coast(self, ctx: Context) -> bool:
        """Traverser une occultation courte n'est légitime que sur cible immobile."""
        return not self.cfg.mobile_target and ctx.locked_position is not None
