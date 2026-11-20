/** Topics ROS2 du projet (voir mycobot_gateway/README.md dans le dépôt). */
export const ROBOT_TOPICS = {
  /** Commandes JSON vers la Pi, relayées en TCP par le nœud bridge_tour. */
  command: { name: '/to_robot', type: 'std_msgs/msg/String' },
  /** Réponses texte de la Pi : « OK: ... » ou « ERROR: ... ». */
  feedback: { name: '/from_robot', type: 'std_msgs/msg/String' },
} as const;

export interface Joint {
  name: string;
  /** Libellé lisible de l'articulation. */
  label: string;
  /** Butée basse, en degrés. */
  min: number;
  /** Butée haute, en degrés. */
  max: number;
}

/**
 * Butées articulaires du MyCobot 320, d'après la documentation Elephant Robotics.
 * Le dépôt ne les définit nulle part : à confirmer sur le robot réel.
 */
export const JOINTS: readonly Joint[] = [
  { name: 'J1', label: 'Base', min: -170, max: 170 },
  { name: 'J2', label: 'Épaule', min: -137, max: 137 },
  { name: 'J3', label: 'Coude', min: -151, max: 142 },
  { name: 'J4', label: 'Poignet 1', min: -148, max: 148 },
  { name: 'J5', label: 'Poignet 2', min: -169, max: 169 },
  { name: 'J6', label: 'Poignet 3', min: -180, max: 180 },
];

/** Vitesse pymycobot (1 à 100). Défaut volontairement bas pour les premiers essais. */
export const SPEED = { min: 1, max: 100, default: 20, presets: [10, 20, 50] } as const;

/** Axe du repère cartésien de pymycobot : position de l'outil (mm) ou orientation (°). */
export interface CoordAxis {
  name: string;
  label: string;
  unit: 'mm' | '°';
  min: number;
  max: number;
}

/**
 * Limites cartésiennes du MyCobot 320 (send_coords), d'après pymycobot.
 * Comme les butées articulaires : à confirmer sur le robot réel.
 */
export const COORDS: readonly CoordAxis[] = [
  { name: 'X', label: 'Position X', unit: 'mm', min: -350, max: 350 },
  { name: 'Y', label: 'Position Y', unit: 'mm', min: -350, max: 350 },
  { name: 'Z', label: 'Position Z', unit: 'mm', min: -41, max: 523.9 },
  { name: 'RX', label: 'Rotation X', unit: '°', min: -180, max: 180 },
  { name: 'RY', label: 'Rotation Y', unit: '°', min: -180, max: 180 },
  { name: 'RZ', label: 'Rotation Z', unit: '°', min: -180, max: 180 },
];

/**
 * Trajectoire de send_coords (paramètre mode de pymycobot) :
 * 'linear' : l'outil va en ligne droite ; 'angular' : chaque axe tourne directement vers sa cible.
 */
export const COORD_MODES = { angular: 0, linear: 1 } as const;
export type CoordMode = keyof typeof COORD_MODES;
