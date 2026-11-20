import { TestBed } from '@angular/core/testing';
import { provideRouter } from '@angular/router';
import { App } from './app';
import { routes } from './app.routes';
import { RosClient } from './core/ros/ros-client';
import { FakeRosClient } from './core/ros/testing/fake-ros-client';
import { VIEWS } from './views/views';

describe('App (coque du dashboard)', () => {
  let ros: FakeRosClient;

  beforeEach(() => {
    ros = new FakeRosClient();
    TestBed.configureTestingModule({
      imports: [App],
      providers: [provideRouter(routes), { provide: RosClient, useValue: ros }],
    });
  });

  it('affiche un lien de menu par vue du registre', async () => {
    const fixture = TestBed.createComponent(App);
    await fixture.whenStable();

    const links = Array.from(
      (fixture.nativeElement as HTMLElement).querySelectorAll('.nav .label'),
      (link) => link.textContent?.trim(),
    );
    expect(links).toEqual(['Accueil', ...VIEWS.map((view) => view.title)]);
  });

  it('affiche chaque réponse de la Pi en notification', async () => {
    const fixture = TestBed.createComponent(App);
    await fixture.whenStable();

    ros.emit('/from_robot', { data: 'OK: retour home' });
    await fixture.whenStable();

    const toast = (fixture.nativeElement as HTMLElement).querySelector('app-toaster .toast');
    expect(toast?.textContent).toContain('OK: retour home');
    expect(toast?.getAttribute('data-level')).toBe('ok');
  });

  it('affiche le bandeau hors ligne quand rosbridge est injoignable', async () => {
    const fixture = TestBed.createComponent(App);
    ros.status.set('disconnected');
    await fixture.whenStable();

    expect((fixture.nativeElement as HTMLElement).querySelector('.offline')?.textContent).toContain(
      'rosbridge injoignable',
    );
  });
});
