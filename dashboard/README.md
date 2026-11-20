# MyCobot Dashboard

Interface web centrale pour piloter le MyCobot 320 et regrouper les vues du projet.
Angular 21, connectée à ROS2 par le WebSocket de **rosbridge** (bibliothèque `roslib`).

```
Navigateur (PC, tablette) ──WebSocket :9090──> rosbridge (tour) ──/to_robot──> bridge_tour ──TCP──> Pi ──> bras
                          <────────────────────────────────────/from_robot──── « OK: ... » / « ERROR: ... »
```

## Démarrer sans robot

```bash
npm install          # si erreur « edgesOut » (bug de npm 10.9) : npx npm@11 install
npm run mock:ros     # terminal 1 : faux rosbridge qui imite la Pi
npm start            # terminal 2 : http://localhost:4200
npm test             # tests unitaires (Vitest)
```

Node 20.19+ ou 22.12+ requis.

## Organisation du code

```
src/app/
  core/
    ros/                     Connexion à ROS, sans rien de spécifique au robot
      ros-client.ts            Contrat RosClient (publish, subscribe, status)
      roslib-ros-client.ts     Implémentation réelle : une seule connexion, reconnexion auto
      provide-ros.ts           Enregistrement dans l'injection de dépendances + URL rosbridge
      testing/fake-ros-client.ts   Faux RosClient pour les tests
    robot/                   Ce qui est propre au MyCobot
      robot.config.ts          Topics, butées articulaires, vitesse
      robot.service.ts         Commandes (sendAngles, sendCoords, goHome, goZero, powerOn, readAngles, readCoords) : seul endroit qui écrit du JSON pour la Pi
  views/
    views.ts                 REGISTRE DES VUES : menu, routes et accueil sont générés ici
    home/                    Accueil (une carte par vue)
    joint-control/           Pilotage en angle des 6 axes
  shared/                  Composants réutilisables : icon (SVG), status-pill, toaster (notifications)
  app.*                    Coque : barre latérale, barre du haut, bandeau hors ligne, notifications
tools/mock-rosbridge.mjs     Faux rosbridge pour le développement
```

## Ajouter une vue

1. `npx ng generate component views/ma-vue`
2. Ajouter une entrée dans `src/app/views/views.ts` :

   ```ts
   {
     path: 'ma-vue',
     title: 'Ma vue',
     icon: 'gauge', // nom d'icône de shared/icon/icon.ts
     description: 'Une phrase pour la carte de l’accueil.',
     loadComponent: () => import('./ma-vue/ma-vue').then((m) => m.MaVue),
   },
   ```

Le menu, la route `#/ma-vue` et la carte de l'accueil apparaissent automatiquement.
`npm test` vérifie que chaque vue déclarée se charge.

## Règles communes

- **Une vue n'utilise jamais `roslib` directement.** Elle passe par `RobotService` (commandes robot)
  ou par `RosClient` (autres topics, par exemple `/joint_states` ou une caméra).
- **Une nouvelle commande robot** : ajouter son type dans `RobotCommand`, puis une méthode publique
  dans `RobotService`, avec sa validation et son test dans `robot.service.spec.ts`.
- **Valider avant d'envoyer.** Une commande invalide lève une erreur et n'est jamais publiée.
  La vue affiche le message d'erreur.
- **Pas de commande différée.** Hors connexion, `publish` lève `RosNotConnectedError` au lieu de
  mettre le message en file d'attente, et les boutons de commande sont désactivés.
- **État dans des signals**, composants en `ChangeDetectionStrategy.OnPush`. L'application tourne
  sans Zone.js : seuls les composants dont un signal a changé sont redessinés, même avec des
  topics à haute fréquence.
- **Style** : identité ABMI reprise de `teleop/teleop_dashboard.py` (palette, logo, couleurs
  des axes). Tout le système de design est dans `src/styles.scss` :
  - Couleurs, ombres, rayons, durées : uniquement les variables (`--surface`, `--accent`, `--ok`,
    `--danger`, `--joint-1` à `--joint-6`…), jamais de valeur en dur dans un composant.
  - En-tête de vue : `<header class="page-head">` avec `.eyebrow`, `h1` et `.lead`.
  - Bloc de contenu : `<section class="card">` + `.card-head` / `.card-title` / `.card-body`.
  - Boutons : `.btn` + `.btn-primary` (action principale, verte), `.btn-secondary`,
    `.btn-ghost` ou `.btn-icon`. Étiquettes : `.tag`. Erreurs : `.alert`.
  - Curseurs : stylés globalement ; poser `--range-color`, `--z` et `--v` sur un parent pour la
    couleur et le remplissage de la piste (voir la vue de pilotage).
  - Icônes : `<app-icon name="…" />` (SVG, `shared/icon/icon.ts`), pas d'emoji : leur rendu
    change d'un système à l'autre.
  - Polices Inter et JetBrains Mono embarquées dans le build : l'interface ne charge rien
    depuis Internet.
  - Un axe garde partout la même couleur (`--joint-N`), y compris sur les futures courbes.
- **Retours du robot** : `RobotService.feedback$` (flux) et `feedbackLog` (journal). La coque
  affiche déjà chaque réponse de la Pi en notification : une vue n'a rien à faire pour ça.
- **Tests** : chaque vue a un `.spec.ts` qui injecte `FakeRosClient`.

## Tester avec le vrai backend (sur la tour)

**Étape A : sans robot, vérifier ce que l'interface publie.** Ne pas lancer `bridge_tour`.

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
ros2 topic echo /to_robot std_msgs/msg/String          # autre terminal
```

1. Ouvrir l'interface (voir ci-dessous) : le badge doit indiquer « Connecté à rosbridge ».
2. Pilotage articulaire → régler J1 sur 10 → « Envoyer les angles ». `topic echo` doit afficher
   `data: '{"action":"send_angles","angles":[10,0,0,0,0,0],"speed":20}'`.
3. Simuler une réponse de la Pi ; elle doit s'afficher en bas de l'interface :
   `ros2 topic pub --once /from_robot std_msgs/msg/String "{data: 'OK: test'}"`

**Étape B : avec le robot.** Lancer `scripts/real_robot_preflight.sh`, puis `bridge_tour` comme
d'habitude. Garder l'arrêt d'urgence physique à portée de main et rester à vitesse 20.
Avant tout envoi, cliquer sur **« Lire la pose actuelle »** : au démarrage les curseurs sont à 0°, pas
à la position du bras. Ensuite, ne modifier qu'un axe de quelques degrés. Le bas de l'interface doit afficher
`OK: angles envoyés [...]`.

## Ouvrir l'interface sur la tour ou une tablette

- Développement : `npm run start:lan` sur la tour, puis `http://<IP-tour>:4200` depuis la tablette.
- Version construite : `npm run build`, puis servir le dossier avec n'importe quel serveur statique :
  `python3 -m http.server 8080 -d dist/mycobot-dashboard/browser` → `http://<IP-tour>:8080`.

L'interface se connecte à rosbridge sur **la machine qui la sert**, port 9090.
Pour forcer une autre adresse : `http://localhost:4200/?ros=ws://10.10.0.115:9090`.

## Points ouverts

- **Pas de bouton d'arrêt d'urgence, volontairement.** Côté Pi, l'action `emergency_stop` appelle
  `release_all_servos()` : les moteurs sont relâchés et le bras tombe. Proposition : ajouter côté
  Pi une action `stop` qui appelle `mc.stop()` (arrêt du mouvement, position maintenue), puis
  l'exposer dans la barre du haut.
- **Butées articulaires et limites cartésiennes** (`robot.config.ts`) : valeurs de la documentation
  Elephant Robotics et de pymycobot, absentes du dépôt. À confirmer sur le robot.
- **Actions de la Pi non exposées, volontairement** : `power_off`, `emergency_stop` et la commande
  texte `stop` relâchent les moteurs, le bras tombe. Pince et pro gripper : aucune pince n'est montée.
