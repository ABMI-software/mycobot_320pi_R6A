import { TestBed } from '@angular/core/testing';
import { RosClient, RosNotConnectedError } from '../ros/ros-client';
import { FakeRosClient } from '../ros/testing/fake-ros-client';
import { firstValueFrom } from 'rxjs';
import { InvalidCommandError, RobotReplyError, RobotService, parseReading } from './robot.service';

describe('RobotService', () => {
  let ros: FakeRosClient;
  let robot: RobotService;

  beforeEach(() => {
    ros = new FakeRosClient();
    TestBed.configureTestingModule({ providers: [{ provide: RosClient, useValue: ros }] });
    robot = TestBed.inject(RobotService);
  });

  it('publie send_angles sur /to_robot au format attendu par la Pi', () => {
    robot.sendAngles([10, 20, -30, 0, 45, 90], 25);

    // Contrat avec le backend : std_msgs/String dont le champ data contient le JSON.
    expect(ros.published).toEqual([
      {
        topic: '/to_robot',
        messageType: 'std_msgs/msg/String',
        message: { data: '{"action":"send_angles","angles":[10,20,-30,0,45,90],"speed":25}' },
      },
    ]);
  });

  it('refuse un angle hors butée sans rien publier', () => {
    expect(() => robot.sendAngles([0, 0, 0, 0, 0, 200], 20)).toThrow(/J6/);
    expect(ros.published).toEqual([]);
  });

  it('refuse un angle manquant (champ vide → NaN)', () => {
    expect(() => robot.sendAngles([0, Number.NaN, 0, 0, 0, 0], 20)).toThrow(/J2/);
    expect(ros.published).toEqual([]);
  });

  it("refuse une liste qui n'a pas 6 angles", () => {
    expect(() => robot.sendAngles([0, 0, 0], 20)).toThrow(InvalidCommandError);
    expect(ros.published).toEqual([]);
  });

  it('refuse une vitesse hors de [1, 100]', () => {
    expect(() => robot.sendAngles([0, 0, 0, 0, 0, 0], 0)).toThrow(/Vitesse/);
    expect(() => robot.sendAngles([0, 0, 0, 0, 0, 0], 101)).toThrow(/Vitesse/);
    expect(ros.published).toEqual([]);
  });

  it('refuse de publier quand rosbridge est déconnecté', () => {
    ros.status.set('disconnected');

    expect(robot.connected()).toBe(false);
    expect(() => robot.sendAngles([0, 0, 0, 0, 0, 0], 20)).toThrow(RosNotConnectedError);
    expect(ros.published).toEqual([]);
  });

  it('expose la dernière réponse de la Pi reçue sur /from_robot', () => {
    expect(robot.lastFeedback()).toBeNull();

    ros.emit('/from_robot', { data: 'OK: angles envoyés [0, 0, 0, 0, 0, 0]' });

    expect(robot.lastFeedback()).toBe('OK: angles envoyés [0, 0, 0, 0, 0, 0]');
  });

  it('tient un journal des réponses, la plus récente en premier, classées par niveau', () => {
    ros.emit('/from_robot', { data: 'OK: retour home' });
    ros.emit('/from_robot', { data: 'ERROR: Robot non connecté' });
    ros.emit('/from_robot', { data: "🚨 ARRÊT D'URGENCE" });

    expect(robot.feedbackLog().map((entry) => entry.level)).toEqual(['error', 'error', 'ok']);
    expect(robot.latestFeedback()?.text).toBe("🚨 ARRÊT D'URGENCE");
  });

  describe('lecture de la pose (get_angles)', () => {
    it('demande get_angles et renvoie les angles de la réponse « ANGLES: [...] »', async () => {
      const angles = firstValueFrom(robot.readAngles());

      expect(JSON.parse((ros.published.at(-1)!.message as { data: string }).data)).toEqual({ action: 'get_angles' });
      ros.emit('/from_robot', { data: 'ANGLES: [0.0, 8.1, -127.0, 40.0, 0.0, -0.5]' });

      await expect(angles).resolves.toEqual([0, 8.1, -127, 40, 0, -0.5]);
    });

    it("ignore les autres réponses et échoue sur une réponse d'erreur", async () => {
      const angles = firstValueFrom(robot.readAngles());

      ros.emit('/from_robot', { data: 'OK: retour home' });
      ros.emit('/from_robot', { data: 'ERROR: Robot non connecté' });

      await expect(angles).rejects.toThrow(/Robot non connecté/);
    });

    it("échoue si la Pi ne répond pas dans le délai", async () => {
      await expect(firstValueFrom(robot.readAngles(10))).rejects.toThrow(RobotReplyError);
    });

    it('refuse une réponse illisible (pymycobot renvoie -1 si le bras ne répond pas)', () => {
      expect(() => parseReading('ANGLES', 'ANGLES: -1')).toThrow(/illisible/);
      expect(() => parseReading('ANGLES', 'ANGLES: [1.0, 2.0]')).toThrow(/illisible/);
    });
  });

  describe('autres commandes', () => {
    /** Dernier JSON publié sur /to_robot. */
    const lastCommand = () => JSON.parse((ros.published.at(-1)!.message as { data: string }).data);

    it('publie go_zero et power_on', () => {
      robot.goZero();
      expect(lastCommand()).toEqual({ action: 'go_zero' });

      robot.powerOn();
      expect(lastCommand()).toEqual({ action: 'power_on' });
    });

    it('publie send_coords au format attendu par la Pi, en mode linéaire par défaut', () => {
      robot.sendCoords([200, 0, 250, -180, 0, -90], 30);
      expect(ros.published.at(-1)!.message).toEqual({
        data: '{"action":"send_coords","coords":[200,0,250,-180,0,-90],"speed":30,"mode":1}',
      });

      robot.sendCoords([200, 0, 250, -180, 0, -90], 30, 'angular');
      expect(lastCommand().mode).toBe(0);
    });

    it('refuse des coordonnées hors limites sans rien publier', () => {
      expect(() => robot.sendCoords([200, 0, 600, -180, 0, -90], 30)).toThrow('Z : 600 mm hors limites');
      expect(() => robot.sendCoords([200, 0, 250], 30)).toThrow(InvalidCommandError);
      expect(ros.published).toEqual([]);
    });

    it('lit les coordonnées en ignorant les autres réponses', async () => {
      const coords = firstValueFrom(robot.readCoords());

      expect(lastCommand()).toEqual({ action: 'get_coords' });
      ros.emit('/from_robot', { data: 'ANGLES: [0.0, 8.0, -127.0, 40.0, 0.0, 0.0]' });
      ros.emit('/from_robot', { data: 'COORDS: [153.2, -64.1, 251.0, -179.9, 0.3, -90.0]' });

      await expect(coords).resolves.toEqual([153.2, -64.1, 251, -179.9, 0.3, -90]);
    });
  });
});
