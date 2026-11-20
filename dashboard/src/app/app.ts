import { ChangeDetectionStrategy, Component, computed, inject, signal } from '@angular/core';
import { toSignal } from '@angular/core/rxjs-interop';
import { NavigationEnd, Router, RouterLink, RouterLinkActive, RouterOutlet } from '@angular/router';
import { filter, map } from 'rxjs';
import { RobotService } from './core/robot/robot.service';
import { Icon } from './shared/icon/icon';
import { StatusPill } from './shared/status-pill/status-pill';
import { Toaster } from './shared/toaster/toaster';
import { VIEWS } from './views/views';

/**
 * Coque du dashboard : barre latérale (menu généré depuis VIEWS), barre du haut
 * (fil d'Ariane, état de connexion), bandeau hors ligne et notifications du robot.
 */
@Component({
  selector: 'app-root',
  imports: [RouterOutlet, RouterLink, RouterLinkActive, Icon, StatusPill, Toaster],
  templateUrl: './app.html',
  styleUrl: './app.scss',
  host: {
    '[class.menu-open]': 'menuOpen()',
    '(document:keydown.escape)': 'closeMenu()',
  },
  changeDetection: ChangeDetectionStrategy.OnPush,
})
export class App {
  private readonly router = inject(Router);

  protected readonly views = VIEWS;
  protected readonly robot = inject(RobotService);

  /** Menu latéral ouvert (écrans étroits uniquement : il est toujours visible ailleurs). */
  protected readonly menuOpen = signal(false);

  private readonly url = toSignal(
    this.router.events.pipe(
      filter((event) => event instanceof NavigationEnd),
      map(() => this.router.url),
    ),
    { initialValue: this.router.url },
  );

  /** Titre de la vue courante, pour le fil d'Ariane. */
  protected readonly currentTitle = computed(() => {
    const path = this.url().split(/[?#]/)[0].replace(/^\//, '');
    return VIEWS.find((view) => view.path === path)?.title ?? 'Accueil';
  });

  protected closeMenu(): void {
    this.menuOpen.set(false);
  }
}
