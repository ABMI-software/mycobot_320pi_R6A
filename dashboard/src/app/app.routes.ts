import { Routes } from '@angular/router';
import { VIEWS } from './views/views';

/** Routes générées depuis le registre VIEWS : ne pas ajouter les vues ici, mais dans views.ts. */
export const routes: Routes = [
  { path: '', title: 'Accueil', loadComponent: () => import('./views/home/home').then((m) => m.Home) },
  ...VIEWS.map(({ path, title, loadComponent }) => ({ path, title, loadComponent })),
  { path: '**', redirectTo: '' },
];
