import { DatePipe } from '@angular/common';
import { ChangeDetectionStrategy, Component, inject } from '@angular/core';
import { RouterLink } from '@angular/router';
import { RobotService } from '../../core/robot/robot.service';
import { Icon } from '../../shared/icon/icon';
import { StatusPill } from '../../shared/status-pill/status-pill';
import { VIEWS } from '../views';

/** Accueil : état du système en un coup d'œil, puis une carte par vue du registre. */
@Component({
  selector: 'app-home',
  imports: [DatePipe, RouterLink, Icon, StatusPill],
  templateUrl: './home.html',
  styleUrl: './home.scss',
  changeDetection: ChangeDetectionStrategy.OnPush,
})
export class Home {
  protected readonly views = VIEWS;
  protected readonly robot = inject(RobotService);
}
