import { ComponentFixture, TestBed } from '@angular/core/testing';
import { RosClient } from '../../core/ros/ros-client';
import { FakeRosClient } from '../../core/ros/testing/fake-ros-client';
import { JointControl } from './joint-control';

describe('JointControl', () => {
  let ros: FakeRosClient;
  let fixture: ComponentFixture<JointControl>;
  let page: HTMLElement;

  beforeEach(async () => {
    ros = new FakeRosClient();
    TestBed.configureTestingModule({
      imports: [JointControl],
      providers: [{ provide: RosClient, useValue: ros }],
    });
    fixture = TestBed.createComponent(JointControl);
    page = fixture.nativeElement;
    await fixture.whenStable();
  });

  /** Saisit une valeur dans le champ numérique d'un axe, comme un utilisateur. */
  function typeAngle(joint: string, degrees: number): void {
    const field = page.querySelector<HTMLInputElement>(`input[aria-label="${joint}, valeur en degrés"]`)!;
    field.value = String(degrees);
    field.dispatchEvent(new Event('change'));
  }

  function button(label: string): HTMLButtonElement {
    // includes et non === : le libellé peut être précédé d'une icône.
    const found = Array.from(page.querySelectorAll('button')).find((b) => b.textContent?.includes(label));
    if (!found) throw new Error(`Bouton « ${label} » introuvable`);
    return found;
  }

  /** Commande JSON envoyée en dernier sur /to_robot. */
  function lastCommand(): unknown {
    const last = ros.published.at(-1)!;
    expect(last.topic).toBe('/to_robot');
    return JSON.parse((last.message as { data: string }).data);
  }

  it('envoie au robot les angles saisis quand on clique sur « Envoyer les angles »', async () => {
    typeAngle('J1', 45);
    typeAngle('J3', -30);

    button('Envoyer les angles').click();
    await fixture.whenStable();

    expect(lastCommand()).toEqual({ action: 'send_angles', angles: [45, 0, -30, 0, 0, 0], speed: 20 });
  });

  it("affiche l'erreur et n'envoie rien si un angle dépasse la butée", async () => {
    typeAngle('J2', 500);

    button('Envoyer les angles').click();
    await fixture.whenStable();

    expect(ros.published).toEqual([]);
    expect(page.querySelector('[role="alert"]')?.textContent).toContain('J2');
  });

  it("signale les axes modifiés jusqu'au prochain envoi", async () => {
    typeAngle('J1', 45);
    page.querySelector<HTMLButtonElement>('[aria-label="Augmenter J2"]')!.click();
    await fixture.whenStable();

    expect(page.querySelector('.pending')?.textContent).toContain('2 axes modifiés');

    button('Envoyer les angles').click();
    await fixture.whenStable();

    expect(lastCommand()).toEqual({ action: 'send_angles', angles: [45, 1, 0, 0, 0, 0], speed: 20 });
    expect(page.querySelector('.pending')).toBeNull();
  });

  it('place les curseurs sur la pose réelle lue sur le robot', async () => {
    button('Lire la pose actuelle').click();
    expect(lastCommand()).toEqual({ action: 'get_angles' });

    ros.emit('/from_robot', { data: 'ANGLES: [12.46, 8.0, -127.0, 40.0, 0.0, 0.0]' });
    await fixture.whenStable();

    const value = (joint: string) =>
      page.querySelector<HTMLInputElement>(`input[aria-label="${joint}, valeur en degrés"]`)!.value;
    expect([value('J1'), value('J3')]).toEqual(['12.5', '-127']);
    expect(page.querySelector('.sync')?.textContent).toContain('Curseurs alignés');
    // La pose lue devient la référence : aucun axe n'est « modifié ».
    expect(page.querySelector('.pending')).toBeNull();
  });

  it('désactive les commandes quand rosbridge est déconnecté', async () => {
    ros.status.set('disconnected');
    await fixture.whenStable();

    expect(button('Envoyer les angles').disabled).toBe(true);
    expect(button('Position home').disabled).toBe(true);
    expect(button('Lire la pose actuelle').disabled).toBe(true);
  });
});
