import { ChangeDetectionStrategy, Component, computed, input } from '@angular/core';

/** Cercle exprimé en chemin SVG, pour décrire chaque icône par une simple liste de chemins. */
const circle = (cx: number, cy: number, r: number) =>
  `M${cx - r} ${cy}a${r} ${r} 0 1 0 ${2 * r} 0a${r} ${r} 0 1 0 ${-2 * r} 0`;

/**
 * Jeu d'icônes du dashboard : grille 24×24, trait de 2 px, extrémités arrondies.
 * Pour une nouvelle icône : ajouter son nom et ses chemins ici, en gardant ce style.
 */
const ICONS = {
  home: ['M3 10.5 12 3l9 7.5', 'M5 9v11a1 1 0 0 0 1 1h4v-6h4v6h4a1 1 0 0 0 1-1V9'],
  arm: ['M4 21h12', 'M10 21v-4', 'M10 17l3-7', 'M13 10l5-3', 'M18 7l2.5 2', 'M18 7l1-3', circle(10, 17, 1.5), circle(13, 10, 1.5), circle(18, 7, 1.5)],
  send: ['M22 2 11 13', 'M22 2l-7 20-4-9-9-4z'],
  refresh: ['M19 8a8 8 0 0 0-14 3', 'M19 3v5h-5', 'M5 16a8 8 0 0 0 14-3', 'M5 21v-5h5'],
  undo: ['M9 14 4 9l5-5', 'M4 9h10.5a5.5 5.5 0 0 1 0 11H11'],
  target: [circle(12, 12, 7), 'M12 2v5', 'M12 17v5', 'M2 12h5', 'M17 12h5'],
  menu: ['M4 6h16', 'M4 12h16', 'M4 18h16'],
  close: ['M18 6 6 18', 'M6 6l12 12'],
  minus: ['M5 12h14'],
  plus: ['M12 5v14', 'M5 12h14'],
  check: ['M20 6 9 17l-5-5'],
  alert: ['M12 3 2.5 20h19z', 'M12 9.5v4', 'M12 17h.01'],
  info: [circle(12, 12, 9), 'M12 16v-4.5', 'M12 8h.01'],
  'arrow-right': ['M5 12h14', 'M13 6l6 6-6 6'],
  'chevron-right': ['M9 6l6 6-6 6'],
  activity: ['M3 12h4l3-8 4 16 3-8h4'],
  gauge: ['M4.2 18a9 9 0 1 1 15.6 0', 'M12 13.5l3.5-4'],
} satisfies Record<string, readonly string[]>;

export type IconName = keyof typeof ICONS;

/** Icône SVG décorative, à la couleur du texte (currentColor) et à la taille de la police. */
@Component({
  selector: 'app-icon',
  template: `
    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"
         stroke-linecap="round" stroke-linejoin="round" aria-hidden="true" focusable="false">
      @for (d of paths(); track $index) {
        <path [attr.d]="d" />
      }
    </svg>
  `,
  styles: `
    :host { display: inline-flex; flex: none; width: 1.15em; height: 1.15em; }
    svg { width: 100%; height: 100%; }
  `,
  changeDetection: ChangeDetectionStrategy.OnPush,
})
export class Icon {
  readonly name = input.required<IconName>();
  protected readonly paths = computed(() => ICONS[this.name()]);
}
