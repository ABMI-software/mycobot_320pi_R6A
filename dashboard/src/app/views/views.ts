import { Type } from '@angular/core';
import { IconName } from '../shared/icon/icon';

export interface DashboardView {
  /** Segment d'URL de la vue : 'joints' → #/joints. Unique. */
  path: string;
  /** Libellé du menu, de la carte d'accueil et du fil d'Ariane. */
  title: string;
  /** Icône du menu et de la carte (voir les noms disponibles dans shared/icon/icon.ts). */
  icon: IconName;
  /** Une phrase affichée sur la carte de l'accueil. */
  description: string;
  /** Chargement à la demande du composant (le code n'est téléchargé qu'à l'ouverture). */
  loadComponent: () => Promise<Type<unknown>>;
}

/**
 * Registre des vues du dashboard. C'est le seul fichier à modifier pour ajouter une vue :
 *   1. ng generate component views/ma-vue
 *   2. ajouter une entrée ci-dessous.
 * Le menu, les routes et les cartes de l'accueil sont générés à partir de cette liste.
 */
export const VIEWS: readonly DashboardView[] = [
  {
    path: 'joints',
    title: 'Pilotage articulaire',
    icon: 'arm',
    description: 'Régler les angles des six axes et les envoyer au robot.',
    loadComponent: () => import('./joint-control/joint-control').then((m) => m.JointControl),
  },
];
