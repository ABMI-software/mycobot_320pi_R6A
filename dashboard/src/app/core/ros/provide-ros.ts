import { Provider } from '@angular/core';
import { RosClient } from './ros-client';
import { ROSBRIDGE_URL, RoslibRosClient } from './roslib-ros-client';

/** Enregistre la connexion ROS réelle dans l'injection de dépendances. */
export function provideRos(rosbridgeUrl: string): Provider[] {
  return [
    { provide: ROSBRIDGE_URL, useValue: rosbridgeUrl },
    { provide: RosClient, useClass: RoslibRosClient },
  ];
}

/**
 * Adresse de rosbridge :
 * - `?ros=ws://10.10.0.115:9090` dans l'URL de la page, si on veut la forcer ;
 * - sinon la machine qui sert l'interface, port 9090. Servie depuis la tour,
 *   l'interface trouve donc rosbridge toute seule, y compris depuis une tablette.
 */
export function resolveRosbridgeUrl(location: Location = window.location): string {
  return new URLSearchParams(location.search).get('ros') ?? `ws://${location.hostname}:9090`;
}
