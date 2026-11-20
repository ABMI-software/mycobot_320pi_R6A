import { Injectable, computed, inject } from '@angular/core';
import { toSignal } from '@angular/core/rxjs-interop';
import { Observable, filter, map, scan, share, take, throwError, timeout } from 'rxjs';
import { RosClient } from '../ros/ros-client';
import { COORDS, COORD_MODES, CoordMode, JOINTS, ROBOT_TOPICS, SPEED } from './robot.config';

/** Message ROS std_msgs/String : un seul champ texte. */
interface StringMessage {
  data: string;
}

/**
 * Commandes JSON comprises par la Pi (bridge_pi_simple.py, méthode execute_json).
 * Pour une nouvelle commande : ajouter son type ici, puis une méthode publique dans RobotService.
 */
export type RobotCommand =
  | { action: 'send_angles'; angles: number[]; speed: number }
  | { action: 'go_home' }
  | { action: 'go_zero' }
  | { action: 'send_coords'; coords: number[]; speed: number; mode: 0 | 1 }
  | { action: 'get_angles' }
  | { action: 'get_coords' }
  | { action: 'power_on' };

export type FeedbackLevel = 'ok' | 'error' | 'info';

/** Une réponse de la Pi reçue sur /from_robot. */
export interface RobotFeedback {
  id: number;
  text: string;
  level: FeedbackLevel;
  at: Date;
}

/** Nombre de réponses conservées dans le journal. */
const FEEDBACK_LOG_SIZE = 50;

/** Classe une réponse de la Pi : « OK: ... », « ERROR: ... », « 🚨 ARRÊT D'URGENCE »… */
export function feedbackLevel(text: string): FeedbackLevel {
  if (/^OK\b/i.test(text)) {
    return 'ok';
  }
  if (/^(ERROR|ERREUR)\b|❌|🚨/i.test(text)) {
    return 'error';
  }
  return 'info';
}

/** Réponse de la Pi absente, en erreur ou illisible. */
export class RobotReplyError extends Error {
  constructor(message: string) {
    super(message);
    this.name = 'RobotReplyError';
  }
}

/** Préfixe d'une réponse de lecture : « ANGLES: [...] » (get_angles) ou « COORDS: [...] » (get_coords). */
export type ReadingLabel = 'ANGLES' | 'COORDS';

/**
 * Extrait les 6 valeurs d'une réponse de lecture de la Pi, par exemple
 * « ANGLES: [0.0, 8.0, -127.0, 40.0, 0.0, 0.0] ».
 * Lève RobotReplyError si la Pi a répondu par une erreur ou par autre chose que 6 nombres
 * (pymycobot renvoie par exemple -1 quand le bras ne répond pas).
 */
export function parseReading(label: ReadingLabel, text: string): number[] {
  if (feedbackLevel(text) === 'error') {
    throw new RobotReplyError(`La Pi n'a pas pu lire le robot : ${text}`);
  }
  const expected = label === 'ANGLES' ? JOINTS.length : COORDS.length;
  const list = new RegExp(`^${label}:\\s*\\[(.*)\\]\\s*$`, 'i').exec(text.trim())?.[1];
  const values = list === undefined ? [] : list.split(',').map((v) => (v.trim() === '' ? NaN : Number(v)));
  if (values.length !== expected || !values.every(Number.isFinite)) {
    throw new RobotReplyError(`Réponse illisible de la Pi : « ${text} »`);
  }
  return values;
}

/** Commande refusée avant l'envoi (angle hors butée, vitesse invalide…). */
export class InvalidCommandError extends Error {
  constructor(message: string) {
    super(message);
    this.name = 'InvalidCommandError';
  }
}

/**
 * Point d'entrée unique pour commander le robot.
 * Les vues appellent ces méthodes ; seul ce service connaît le format JSON et le topic /to_robot.
 */
@Injectable({ providedIn: 'root' })
export class RobotService {
  private readonly ros = inject(RosClient);
  private nextFeedbackId = 0;

  readonly status = this.ros.status;
  readonly rosbridgeUrl = this.ros.url;

  /** rosbridge est joignable. Ne garantit pas que bridge_tour et la Pi tournent : voir le journal. */
  readonly connected = computed(() => this.status() === 'connected');

  /** Flux des réponses de la Pi, partagé entre le journal et les notifications. */
  readonly feedback$: Observable<RobotFeedback> = this.ros
    .subscribe<StringMessage>(ROBOT_TOPICS.feedback.name, ROBOT_TOPICS.feedback.type)
    .pipe(
      map(({ data }) => ({ id: this.nextFeedbackId++, text: data, level: feedbackLevel(data), at: new Date() })),
      share(),
    );

  /** Dernières réponses de la Pi, la plus récente en premier. */
  readonly feedbackLog = toSignal(
    this.feedback$.pipe(
      scan((log: RobotFeedback[], entry) => [entry, ...log].slice(0, FEEDBACK_LOG_SIZE), []),
    ),
    { initialValue: [] },
  );

  readonly latestFeedback = computed(() => this.feedbackLog()[0] ?? null);

  /** Texte de la dernière réponse, par exemple « OK: angles envoyés [...] ». */
  readonly lastFeedback = computed(() => this.latestFeedback()?.text ?? null);

  /** Envoie les angles de J1 à J6, en degrés, à la vitesse donnée (1 à 100). */
  sendAngles(angles: readonly number[], speed: number): void {
    assertWithinLimits(angles, JOINTS);
    assertValidSpeed(speed);
    this.send({ action: 'send_angles', angles: [...angles], speed });
  }

  /** Ramène le bras à la pose home définie côté Pi ([0, 8, -127, 40, 0, 0], vitesse 30). */
  goHome(): void {
    this.send({ action: 'go_home' });
  }

  /** Amène tous les axes à 0° (pose définie côté Pi, vitesse 30). */
  goZero(): void {
    this.send({ action: 'go_zero' });
  }

  /**
   * Déplace l'outil vers une position cartésienne : x, y, z en mm puis rx, ry, rz en degrés.
   * mode 'linear' : l'outil va en ligne droite ; 'angular' : chaque axe tourne directement vers sa cible.
   */
  sendCoords(coords: readonly number[], speed: number, mode: CoordMode = 'linear'): void {
    assertWithinLimits(coords, COORDS);
    assertValidSpeed(speed);
    this.send({ action: 'send_coords', coords: [...coords], speed, mode: COORD_MODES[mode] });
  }

  /** Remet les moteurs sous tension, par exemple après un relâchement. */
  powerOn(): void {
    this.send({ action: 'power_on' });
  }

  /** Lit la position actuelle des 6 axes, en degrés (action get_angles). */
  readAngles(timeoutMs = 3000): Observable<number[]> {
    return this.read({ action: 'get_angles' }, 'ANGLES', timeoutMs);
  }

  /** Lit la position de l'outil : x, y, z en mm puis rx, ry, rz en degrés (action get_coords). */
  readCoords(timeoutMs = 3000): Observable<number[]> {
    return this.read({ action: 'get_coords' }, 'COORDS', timeoutMs);
  }

  /**
   * Envoie une demande de lecture et attend la réponse « ANGLES: » ou « COORDS: ».
   * Les réponses de la Pi n'ont pas d'identifiant : on retient la première réponse du bon type,
   * ou la première erreur, qui suit la demande. Échoue si rien n'arrive dans le délai.
   */
  private read(command: RobotCommand, label: ReadingLabel, timeoutMs: number): Observable<number[]> {
    return new Observable<number[]>((subscriber) => {
      // Écouter avant d'envoyer, pour ne pas rater une réponse rapide.
      const reply = this.feedback$
        .pipe(
          filter(({ text, level }) => text.toUpperCase().startsWith(`${label}:`) || level === 'error'),
          take(1),
          timeout({
            first: timeoutMs,
            with: () =>
              throwError(
                () => new RobotReplyError("La Pi n'a pas répondu. bridge_tour et la Pi sont-ils lancés ?"),
              ),
          }),
          map(({ text }) => parseReading(label, text)),
        )
        .subscribe(subscriber);
      try {
        this.send(command);
      } catch (e) {
        subscriber.error(e);
      }
      return reply;
    });
  }

  private send(command: RobotCommand): void {
    this.ros.publish<StringMessage>(ROBOT_TOPICS.command.name, ROBOT_TOPICS.command.type, {
      data: JSON.stringify(command),
    });
  }
}

/** Vérifie qu'il y a une valeur par axe et que chacune reste dans les limites de son axe. */
function assertWithinLimits(
  values: readonly number[],
  axes: readonly { name: string; min: number; max: number; unit?: string }[],
): void {
  if (values.length !== axes.length) {
    throw new InvalidCommandError(`${axes.length} valeurs attendues, ${values.length} reçues.`);
  }
  values.forEach((value, i) => {
    const { name, min, max, unit = '°' } = axes[i];
    const format = (v: number) => (unit === '°' ? `${v}°` : `${v} ${unit}`);
    if (!Number.isFinite(value)) {
      throw new InvalidCommandError(`${name} : valeur manquante ou invalide.`);
    }
    if (value < min || value > max) {
      throw new InvalidCommandError(`${name} : ${format(value)} hors limites [${format(min)}, ${format(max)}].`);
    }
  });
}

function assertValidSpeed(speed: number): void {
  if (!Number.isInteger(speed) || speed < SPEED.min || speed > SPEED.max) {
    throw new InvalidCommandError(
      `Vitesse invalide : un entier entre ${SPEED.min} et ${SPEED.max} est attendu.`,
    );
  }
}
