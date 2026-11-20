import { ApplicationConfig, provideBrowserGlobalErrorListeners } from '@angular/core';
import { provideRouter, withHashLocation } from '@angular/router';
import { routes } from './app.routes';
import { provideRos, resolveRosbridgeUrl } from './core/ros/provide-ros';

export const appConfig: ApplicationConfig = {
  providers: [
    provideBrowserGlobalErrorListeners(),
    // URLs en #/vue : le build fonctionne avec n'importe quel serveur de fichiers statiques.
    provideRouter(routes, withHashLocation()),
    provideRos(resolveRosbridgeUrl()),
  ],
};
