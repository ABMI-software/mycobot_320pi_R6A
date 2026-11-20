import { DatePipe } from '@angular/common';
import { ChangeDetectionStrategy, Component, DestroyRef, computed, inject, signal } from '@angular/core';
import { takeUntilDestroyed } from '@angular/core/rxjs-interop';
import { JOINTS, Joint, SPEED } from '../../core/robot/robot.config';
import { RobotService } from '../../core/robot/robot.service';
import { Icon } from '../../shared/icon/icon';

const zeroAngles = () => JOINTS.map(() => 0);

/** Position d'un angle entre les butées de l'axe, de 0 (min) à 1 (max). */
const ratio = (joint: Joint, angle: number) =>
  (Math.min(Math.max(angle, joint.min), joint.max) - joint.min) / (joint.max - joint.min);

/**
 * Pilotage en angle : un curseur et un champ par axe, puis envoi groupé des 6 angles.
 * Les axes modifiés depuis le dernier envoi sont signalés jusqu'au prochain envoi.
 */
@Component({
  selector: 'app-joint-control',
  imports: [DatePipe, Icon],
  templateUrl: './joint-control.html',
  styleUrl: './joint-control.scss',
  changeDetection: ChangeDetectionStrategy.OnPush,
})
export class JointControl {
  protected readonly robot = inject(RobotService);
  private readonly destroyRef = inject(DestroyRef);
  protected readonly speedLimits = SPEED;

  protected readonly angles = signal<number[]>(zeroAngles());
  /** Angles du dernier envoi réussi : référence pour repérer les modifications. */
  protected readonly sentAngles = signal<number[]>(zeroAngles());
  protected readonly speed = signal<number>(SPEED.default);
  protected readonly error = signal<string | null>(null);
  /** Heure de la dernière lecture de la pose réelle ; null tant que les curseurs ne la reflètent pas. */
  protected readonly poseReadAt = signal<Date | null>(null);
  protected readonly reading = signal(false);

  /** Une ligne par axe, prête pour l'affichage. */
  protected readonly rows = computed(() => {
    const angles = this.angles();
    const sent = this.sentAngles();
    return JOINTS.map((joint, index) => {
      const angle = angles[index];
      return {
        joint,
        index,
        angle,
        changed: angle !== sent[index],
        zero: ratio(joint, 0),
        value: ratio(joint, Number.isFinite(angle) ? angle : 0),
      };
    });
  });

  protected readonly changedCount = computed(() => this.rows().filter((row) => row.changed).length);
  protected readonly speedRatio = computed(() => (this.speed() - SPEED.min) / (SPEED.max - SPEED.min));

  protected setAngle(index: number, value: number): void {
    this.angles.update((angles) => angles.map((angle, i) => (i === index ? value : angle)));
  }

  /** Décale un axe de 1° (10° avec Maj), sans dépasser ses butées. */
  protected nudge(index: number, direction: 1 | -1, event: MouseEvent): void {
    const { min, max } = JOINTS[index];
    const current = this.angles()[index];
    const next = (Number.isFinite(current) ? current : 0) + direction * (event.shiftKey ? 10 : 1);
    this.setAngle(index, Math.min(Math.max(next, min), max));
  }

  /** Revient aux angles du dernier envoi. */
  protected revert(): void {
    this.angles.set([...this.sentAngles()]);
    this.error.set(null);
  }

  /** Met tous les curseurs à 0° (rien n'est envoyé). */
  protected zero(): void {
    this.angles.set(zeroAngles());
  }

  protected send(): void {
    const angles = this.angles();
    this.run(() => {
      this.robot.sendAngles(angles, this.speed());
      this.sentAngles.set([...angles]);
    });
  }

  protected goHome(): void {
    this.run(() => {
      this.robot.goHome();
      // Le bras part vers home : les curseurs ne reflètent plus sa pose.
      this.poseReadAt.set(null);
    });
  }

  /** Place les curseurs sur la position réelle du bras, qui devient la référence des modifications. */
  protected readPose(): void {
    this.error.set(null);
    this.reading.set(true);
    this.robot
      .readAngles()
      .pipe(takeUntilDestroyed(this.destroyRef))
      .subscribe({
        next: (angles) => {
          const rounded = angles.map((angle) => Math.round(angle * 10) / 10);
          this.angles.set(rounded);
          this.sentAngles.set([...rounded]);
          this.poseReadAt.set(new Date());
          this.reading.set(false);
        },
        error: (e: unknown) => {
          this.error.set(e instanceof Error ? e.message : String(e));
          this.reading.set(false);
        },
      });
  }

  /** Exécute une commande et affiche son erreur éventuelle (validation, coupure réseau). */
  private run(command: () => void): void {
    try {
      command();
      this.error.set(null);
    } catch (e) {
      this.error.set(e instanceof Error ? e.message : String(e));
    }
  }
}
