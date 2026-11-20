import { VIEWS } from './views';

// Garde-fou pour les prochaines vues : une entrée mal déclarée fait échouer `npm test`.
describe('Registre des vues', () => {
  it('donne un chemin unique à chaque vue', () => {
    const paths = VIEWS.map((view) => view.path);
    expect(new Set(paths).size).toBe(paths.length);
  });

  it.each(VIEWS)('charge le composant de la vue « $title »', async (view) => {
    expect(await view.loadComponent()).toBeTypeOf('function');
  });
});
