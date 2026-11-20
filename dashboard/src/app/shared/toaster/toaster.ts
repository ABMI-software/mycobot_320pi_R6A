import { DatePipe } from '@angular/common';
import { ChangeDetectionStrategy, Component, inject, signal } from '@angular/core';
import { takeUntilDestroyed } from '@angular/core/rxjs-interop';
import { FeedbackLevel, RobotFeedback, RobotService } from '../../core/robot/robot.service';
import { Icon, IconName } from '../icon/icon';

const TOAST_DURATION_MS = 5000;
const MAX_TOASTS = 4;

/** Notifications éphémères pour chaque réponse de la Pi, quelle que soit la vue ouverte. */
@Component({
  selector: 'app-toaster',
  imports: [DatePipe, Icon],
  templateUrl: './toaster.html',
  styleUrl: './toaster.scss',
  host: { role: 'region', 'aria-label': 'Retours du robot', 'aria-live': 'polite' },
  changeDetection: ChangeDetectionStrategy.OnPush,
})
export class Toaster {
  protected readonly toasts = signal<RobotFeedback[]>([]);
  protected readonly duration = `${TOAST_DURATION_MS}ms`;
  protected readonly icons: Record<FeedbackLevel, IconName> = { ok: 'check', error: 'alert', info: 'info' };

  constructor() {
    inject(RobotService)
      .feedback$.pipe(takeUntilDestroyed())
      .subscribe((feedback) => {
        this.toasts.update((toasts) => [feedback, ...toasts].slice(0, MAX_TOASTS));
        setTimeout(() => this.dismiss(feedback.id), TOAST_DURATION_MS);
      });
  }

  protected dismiss(id: number): void {
    this.toasts.update((toasts) => toasts.filter((toast) => toast.id !== id));
  }
}
