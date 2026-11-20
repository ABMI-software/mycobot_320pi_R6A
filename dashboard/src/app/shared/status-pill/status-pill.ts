import { ChangeDetectionStrategy, Component, computed, input } from '@angular/core';
import { ConnectionStatus } from '../../core/ros/ros-client';

const LABELS: Record<ConnectionStatus, string> = {
  connecting: 'Connexion…',
  connected: 'Connecté',
  disconnected: 'Hors ligne',
};

/** Pastille d'état de la connexion rosbridge, avec un point animé. */
@Component({
  selector: 'app-status-pill',
  template: `<span class="dot" aria-hidden="true"></span>{{ label() }}`,
  styleUrl: './status-pill.scss',
  host: { '[attr.data-status]': 'status()' },
  changeDetection: ChangeDetectionStrategy.OnPush,
})
export class StatusPill {
  readonly status = input.required<ConnectionStatus>();
  protected readonly label = computed(() => LABELS[this.status()]);
}
