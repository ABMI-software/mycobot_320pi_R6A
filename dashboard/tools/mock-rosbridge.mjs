// Faux rosbridge pour développer sans ROS ni robot : `npm run mock:ros`, puis `npm start`.
// Il implémente juste ce qu'il faut du protocole rosbridge (advertise, publish, subscribe)
// et répond sur /from_robot comme le ferait bridge_pi_simple.py sur la Pi.
import { WebSocketServer } from 'ws';

const PORT = 9090;
const server = new WebSocketServer({ port: PORT });

server.on('connection', (socket) => {
  console.log('Interface connectée');
  const subscriptions = new Set();

  socket.on('message', (raw) => {
    const message = JSON.parse(raw.toString());
    switch (message.op) {
      case 'subscribe':
        subscriptions.add(message.topic);
        break;
      case 'unsubscribe':
        subscriptions.delete(message.topic);
        break;
      case 'publish':
        if (message.topic === '/to_robot') {
          console.log('/to_robot   ←', message.msg.data);
          const reply = fakePiReply(message.msg.data);
          console.log('/from_robot →', reply);
          if (subscriptions.has('/from_robot')) {
            socket.send(JSON.stringify({ op: 'publish', topic: '/from_robot', msg: { data: reply } }));
          }
        }
        break;
    }
  });

  socket.on('close', () => console.log('Interface déconnectée'));
});

/** Pose simulée du bras, en degrés : home au démarrage, comme le vrai robot. */
const HOME = [0, 8, -127, 40, 0, 0];
let pose = [...HOME];
/** Position simulée de l'outil (mm puis degrés). Ne suit pas les angles : c'est un faux. */
let coords = [153.2, -64.1, 251.0, -179.9, 0.3, -90.0];

/** Écrit une liste comme Python l'afficherait : [0.0, 8.0, -127.5]. */
const pythonList = (values) => `[${values.map((v) => (Number.isInteger(v) ? v.toFixed(1) : String(v))).join(', ')}]`;

/** Mêmes réponses texte que bridge_pi_simple.py (méthode execute_json). */
function fakePiReply(data) {
  let command;
  try {
    command = JSON.parse(data);
  } catch {
    return 'ERROR: JSON invalide';
  }
  switch (command.action) {
    case 'send_angles':
      pose = [...command.angles];
      return `OK: angles envoyés [${command.angles.join(', ')}]`;
    case 'go_home':
      pose = [...HOME];
      return 'OK: retour home';
    case 'go_zero':
      pose = [0, 0, 0, 0, 0, 0];
      return 'OK: retour zero';
    case 'send_coords':
      coords = [...command.coords];
      return `OK: coords envoyées [${command.coords.slice(0, 3).join(', ')}]`;
    case 'get_angles':
      return `ANGLES: ${pythonList(pose)}`;
    case 'get_coords':
      return `COORDS: ${pythonList(coords)}`;
    case 'power_on':
      return 'OK: moteurs allumés';
    default:
      return `ERROR: action inconnue ${command.action}`;
  }
}

console.log(`Faux rosbridge à l'écoute sur ws://localhost:${PORT} (Ctrl+C pour arrêter)`);
