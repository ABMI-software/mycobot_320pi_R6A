import { signal } from '@angular/core';
import { Observable, Subject, filter, map } from 'rxjs';
import { ConnectionStatus, RosClient, RosNotConnectedError } from '../ros-client';

export interface PublishedMessage {
  topic: string;
  messageType: string;
  message: unknown;
}

/**
 * Faux RosClient pour les tests, sans robot ni rosbridge.
 * - `published` : tout ce que l'application a publié ;
 * - `emit()` : simule un message arrivant de ROS ;
 * - `status.set('disconnected')` : simule une coupure.
 *
 * Utilisation : providers: [{ provide: RosClient, useValue: new FakeRosClient() }]
 */
export class FakeRosClient extends RosClient {
  readonly url = 'ws://fake-rosbridge:9090';
  readonly status = signal<ConnectionStatus>('connected');
  readonly published: PublishedMessage[] = [];
  private readonly incoming = new Subject<{ topic: string; message: unknown }>();

  publish<T>(topic: string, messageType: string, message: T): void {
    if (this.status() !== 'connected') {
      throw new RosNotConnectedError();
    }
    this.published.push({ topic, messageType, message });
  }

  subscribe<T>(topic: string): Observable<T> {
    return this.incoming.pipe(
      filter((incoming) => incoming.topic === topic),
      map((incoming) => incoming.message as T),
    );
  }

  emit(topic: string, message: unknown): void {
    this.incoming.next({ topic, message });
  }
}
