import { DestroyRef, Injectable, InjectionToken, inject, signal } from '@angular/core';
import { Observable } from 'rxjs';
import { Ros, Topic } from 'roslib';
import { ConnectionStatus, RosClient, RosNotConnectedError } from './ros-client';

/** URL du WebSocket rosbridge, fournie par provideRos() dans app.config.ts. */
export const ROSBRIDGE_URL = new InjectionToken<string>('ROSBRIDGE_URL');

const RECONNECT_DELAY_MS = 3000;

/**
 * Implémentation réelle de RosClient : une seule connexion roslib vers rosbridge,
 * partagée par toute l'application, avec reconnexion automatique.
 */
@Injectable()
export class RoslibRosClient extends RosClient {
  readonly url = inject(ROSBRIDGE_URL);
  private readonly ros = new Ros();
  private readonly publishers = new Map<string, Topic<unknown>>();
  private readonly statusSignal = signal<ConnectionStatus>('connecting');
  private reconnectTimer?: ReturnType<typeof setTimeout>;
  private destroyed = false;

  readonly status = this.statusSignal.asReadonly();

  constructor() {
    super();
    this.ros.on('connection', () => this.statusSignal.set('connected'));
    this.ros.on('close', () => {
      this.statusSignal.set('disconnected');
      this.scheduleReconnect();
    });
    inject(DestroyRef).onDestroy(() => {
      this.destroyed = true;
      clearTimeout(this.reconnectTimer);
      this.ros.close();
    });
    this.connect();
  }

  publish<T>(topic: string, messageType: string, message: T): void {
    // Hors connexion, roslib garderait le message en file d'attente et l'enverrait
    // à la reconnexion. Pour un robot, une commande périmée ne doit jamais partir en différé.
    if (!this.ros.isConnected) {
      throw new RosNotConnectedError();
    }
    let publisher = this.publishers.get(topic);
    if (!publisher) {
      publisher = new Topic({ ros: this.ros, name: topic, messageType });
      this.publishers.set(topic, publisher);
    }
    publisher.publish(message);
  }

  subscribe<T>(topic: string, messageType: string): Observable<T> {
    return new Observable<T>((subscriber) => {
      // queue_length: 1 → rosbridge ne garde que le dernier message si l'interface prend du retard.
      const rosTopic = new Topic<T>({ ros: this.ros, name: topic, messageType, queue_length: 1 });
      const onMessage = (message: T) => subscriber.next(message);
      rosTopic.subscribe(onMessage);
      return () => rosTopic.unsubscribe(onMessage);
    });
  }

  private connect(): void {
    this.ros.connect(this.url).catch(() => this.scheduleReconnect());
  }

  private scheduleReconnect(): void {
    if (this.destroyed) {
      return;
    }
    clearTimeout(this.reconnectTimer);
    this.reconnectTimer = setTimeout(() => this.connect(), RECONNECT_DELAY_MS);
  }
}
