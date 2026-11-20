import { Signal } from '@angular/core';
import { Observable } from 'rxjs';

export type ConnectionStatus = 'connecting' | 'connected' | 'disconnected';

/**
 * Contrat d'accès à ROS utilisé par toute l'application.
 *
 * C'est l'équivalent d'une interface .NET injectée : les services et les vues en
 * dépendent, jamais de roslib directement. En production, Angular injecte
 * RoslibRosClient ; dans les tests, on injecte FakeRosClient.
 */
export abstract class RosClient {
  /** Adresse du WebSocket rosbridge, pour l'affichage. */
  abstract readonly url: string;

  /** État de la connexion WebSocket à rosbridge. */
  abstract readonly status: Signal<ConnectionStatus>;

  /** Publie un message sur un topic. Lève RosNotConnectedError si rosbridge est injoignable. */
  abstract publish<T>(topic: string, messageType: string, message: T): void;

  /** Flux des messages d'un topic. Le désabonnement RxJS désabonne aussi côté ROS. */
  abstract subscribe<T>(topic: string, messageType: string): Observable<T>;
}

export class RosNotConnectedError extends Error {
  constructor() {
    super('Pas de connexion à rosbridge : commande non envoyée.');
    this.name = 'RosNotConnectedError';
  }
}
