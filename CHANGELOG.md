# Changelog

Toutes les modifications notables de ce projet sont documentées dans ce fichier.

Le format est basé sur [Keep a Changelog](https://keepachangelog.com/fr/1.0.0/),
et ce projet adhère au [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [Non publié]

### Corrigé

- **`arducam_extrinsic_dream_v4.yaml` n'était comparable à rien.**
  `self_calibrate_arducam.py:66-67` charge `cam_0` (focale ≈ 527) via
  `convert_to_ndds.py:102` (`arducam → cam_0`), alors que l'arducam de ce banc
  est **`cam_3`** (focale ≈ 495) — c'est ce que déclare
  `arducam_extrinsic_pick.yaml` et ce qu'utilise `pick_dashboard`. **6,4 %
  d'écart de focale, plus les mauvais coefficients de distorsion.** S'y ajoute
  un montage physique différent (cf. `convert_to_ndds.py:87-92`). Le désaccord
  de ~1 m latéral avec `markers` et `handeye` n'est donc pas une énigme
  ouverte : ce fichier n'a jamais mesuré la même caméra. Il sort de la liste des
  questions en suspens. `pick_and_place_live_dashboard.py` n'est **pas**
  concerné, il prend `cam_3` par le registre.

- **`link1` et `link2` sont le même point 3D dans la FK**, pas un artefact de la
  vue zénithale comme supposé : ils se projettent sur le pixel **exact** dans
  les deux caméras (195,5/342,5 et 425,7/299,2). DREAM a donc deux sorties pour
  un seul point physique — indissociables par construction, quelle que soit la
  caméra ou le nombre de vues.

- **Le bras bouge à nouveau en simulation de tri.** Le plugin
  `gz-sim-joint-position-controller-system` avait disparu de
  `mycobot_pro_320_pi_gazebo.urdf` dans cette copie du dépôt — il est présent
  dans `~/ros_jazzy`, où la démo fonctionnait. Sans lui, les topics
  `/model/mycobot_320/joint/<j>/cmd_pos` n'ont **aucun abonné** : mesuré le
  28/08, `ros2 control list_controllers` rend « No controllers are currently
  loaded! » et `/joint_states` reste à zéro pendant tout le tri. La démo allait
  pourtant jusqu'à « Sorting complete » parce qu'elle est en boucle ouverte —
  seule la **téléportation** des cubes (`gz set_pose`) donnait l'illusion d'un
  pick-and-place. Plugin restauré tel quel.

### Ajouté

- **Le biais DREAM n'est corrigeable par aucune transformation 2D (01/09).**
  75 correspondances, 13 poses, FK et extrinsèques validées :

  | modèle ajusté | RMS | sur la planche |
  |---|---|---|
  | aucun (DREAM = FK) | 56,9 px | 122 mm |
  | translation | 24,3 px | 52 mm |
  | similitude (échelle **0,845**, rot +8,1°) | 19,8 px | 42 mm |
  | affine complète (6 param.) | 17,0 px | **36 mm** |

  Le décalage **n'est pas constant** : `dx` suit J1 (+8 px à J1=0° → +53 px à
  J1=60°) tandis que `dy` reste à ≈ −43 px. Un décalage constant ne peut donc
  pas le corriger, et même l'affine complète laisse 36 mm. La dispersion par
  pose vaut déjà 20-24 px : **une fois tout le systématique retiré il reste le
  bruit propre de DREAM, ~40 mm.** Pour une saisie au millimètre, rédhibitoire.

  L'échelle **0,845** dit que DREAM voit le robot 15 % plus petit que la
  réalité — ce qu'on attend d'un réseau affiné caméra plus loin. Converge avec
  le montage différent de `real_3cam` (`convert_to_ndds.py:87-92`).

  ⇒ **Pour utiliser DREAM sur ce banc il faut le réaffiner sur CE montage.**
  D'ici là l'extrinsèque marqueurs reste la référence du pick.

- **La fusion multi-caméras ne corrige pas ce biais — mesuré, pas supposé.**
  Sur une même pose : arducam 7/7 détectés, erreurs 52→98 px ; SVPRO 3/7,
  erreurs 117→207 px (J4, J5, bride non détectés). La SVPRO est 2 à 3× pire.
  La fusion `solve-then-fuse` pondère par la reprojection avec
  `JOINT_CONFIDENCE_PX_THRESHOLD = 15 px` : à 117-207 px la SVPRO reçoit un
  poids **nul** partout et la fusion se replie sur « MONO via arducam ». C'est
  un mécanisme de **robustesse** (jamais pire que la meilleure caméra, comble
  les occlusions), pas un correcteur de biais commun aux deux vues.

- **`fk_vs_dream_series.py --balayage` : écran de visibilité avant de bouger.**
  Une pose dont les keypoints FK tombent hors de la fenêtre réseau
  (x ∈ [80, 560], marge 30 px) ne mesure rien — elle est écartée sans être
  jouée. Sur le balayage du 01/09 : **8 poses sur 21 rejetées**. Répond au
  constat que le bras est parfois coupé dans l'arducam.

- **`scripts/fk_vs_dream_series.py` — le biais DREAM est mesuré : ~52 px
  systématiques (01/09).** Sur des poses de travail (bras au-dessus de la
  planche, pince vers le bas), avec une FK et des extrinsèques déjà validées,
  les keypoints DREAM se décalent **tous dans la même direction** :

  | modèle ajusté (22 correspondances, 4 poses) | RMS résiduel |
  |---|---|
  | aucun (DREAM = FK) | 56,5 px |
  | **translation** `(+31,6, −41,2)` | **22,0 px** |
  | similitude (échelle 0,905, rot +10,7°) | 17,3 px |
  | affine complète | 16,8 px |

  Une translation seule absorbe l'essentiel ; échelle et rotation n'apportent
  que 4,7 px. Décalage moyen **51,2 px**, dispersion 23,3 px. À 1,06 m et
  f ≈ 495 px, 52 px valent **~11 cm** sur la planche — l'ordre de grandeur des
  pick qui ratent de quelques centimètres.

  **Cause la plus probable : le réseau a été affiné sur `real_3cam`, où
  l'arducam était sur un AUTRE montage** (cf. `convert_to_ndds.py:87-92`). Il a
  appris un a priori de point de vue. Ceci explique aussi que la
  self-calibration place la caméra ~1 m à côté : le PnP absorbe un décalage
  uniforme de 52 px en une grande translation.

  Alternative non exclue : un décalage constant en XY monde (indiscernable d'un
  décalage image sous une caméra zénithale). La SVPRO trancherait, mais elle ne
  détecte que 0-3/7.

- **DREAM ne voit que 75 % de l'image.** `image_preprocessing: shrink-and-crop`,
  640×480 → 400×400 : la fenêtre réseau est **x ∈ [80, 560]**, deux bandes
  verticales de 80 px sont jetées. Quand le bras part à gauche (J1 ≈ 88°) la
  bride tombe à x ≈ 60, **hors champ réseau** — la détection s'effondre à 1/7.
  Ce n'est pas « le bras a quitté la planche », c'est le recadrage. La
  bibliothèque inverse correctement le recadrage (`convert_keypoints_to_raw_from_netin`),
  donc ce n'est **pas** la cause des 52 px : vérifié, les 7 keypoints étaient
  dans la fenêtre sur les 4 poses mesurées.

- **Diagnostic robuste au rebranchement de l'arducam.** Caméra retrouvée par son
  nom V4L2 (son `/dev/videoN` change au replug) et exposition **75** réimposée à
  chaque capture (elle est perdue au replug). Un balayage 20→300 confirme que
  l'exposition ne change pas la détection (0 à 4/7, aucun optimum) — on la fige
  pour que les mesures soient comparables, pas pour améliorer quoi que ce soit.

- **`scripts/fk_vs_dream_diagnostic.py` — départage la FK et DREAM.** La
  self-calibration markerless laissait un résidu systématique (`link3` à
  23,6 px) qu'aucune pose de caméra n'expliquait : erreur de FK, ou biais de
  détection ? DREAM ne peut pas trancher — dans son schéma la FK est une
  **entrée** du PnP, pas une sortie, donc s'en servir pour vérifier la FK est
  circulaire. L'outil prend une référence extérieure aux deux : les extrinsèques
  **marqueurs** déjà calculées (`arducam_extrinsic_pick` 20/08,
  `svpro_extrinsic_servo` 24/08), qui ne doivent rien à DREAM. Il projette le
  squelette FK à travers elles sur **les deux caméras à la fois** et le compare
  aux détections DREAM. Le bras ne bouge pas. `--brut --angles …` rejoue hors
  ligne sur les images enregistrées, sans matériel.

  **Verdict mesuré le 31/08 (figure `docs/fk_vs_dream.png`) : la FK et les deux
  extrinsèques sont validées, l'écart est du côté de DREAM.** Le squelette vert
  épouse le bras sur toute sa longueur dans les deux vues — deux caméras, deux
  extrinsèques calculées séparément, deux géométries opposées (zénith / côté) :
  elles ne peuvent pas se tromper de la même manière par hasard. Le keypoint
  `base` sert de juge de paix, sa projection étant **fixe** et indépendante des
  angles : la FK le place juste, DREAM le rate de 57 px (arducam) et 62 px
  (svpro). Écarts médians FK↔DREAM 73 px et 132 px.

  ⚠ Ceci **ne prouve pas** un biais DREAM général : dans cette pose le bras est
  sorti de la planche, au-dessus du clavier — le mode d'échec déjà mesuré le
  31/08 (7/7 → 0-1/7). Les 3/7 et 2/7 « détectés » sont des détections fantômes,
  instables d'une trame à l'autre (6/7 et 3/7 à la capture précédente). La
  mesure du vrai biais demande de refaire ce tableau sur des poses favorables.

- **Conséquence : les 23,6 px de `link3` ne sont pas une erreur de FK.** La
  chaîne cinématique et les intrinsèques `cam_3` / `cam_2` sont saines.

- **`scripts/pick_and_place_live_dashboard.py` — self-calibration markerless au
  lancement.** Même tableau de bord que `pick_dashboard`, mais l'extrinsèque est
  refaite à chaque démarrage avec le robot comme mire (FK des encodeurs ↔
  keypoints DREAM), sans marqueur. Il **importe** `pick_dashboard` et remplace
  `Vision` à l'exécution : `pick_dashboard.py` n'est pas modifié.
  **État : la calibration n'est pas encore exploitable** — voir les Notes.

- **Bibliothèque DREAM restaurée dans `/home/genji/DREAM`** (persistant), avec
  `/tmp/DREAM` en lien symbolique pour les 10 scripts qui codent ce chemin en
  dur. Elle avait disparu : `/tmp` est vidé au redémarrage. Après un reboot,
  refaire `ln -sfn /home/genji/DREAM /tmp/DREAM`.

- **`sim_sorting_grasp` — les quatre objets tries par saisie PHYSIQUE.**
  4/4 le 31/08 sur le banc `sim_grasp.launch.py`, sans aucune téléportation :
  bras au JTC `mycobot_controller`, pince au `gripper_position_controller`,
  chaque prise vérifiée sur la pose Gazebo de l'objet. Les quatre objets
  finissent **à plat au fond** de leur bac (0,0° d'inclinaison, z au millimètre
  du fond), écart au centre −6/+3, −5/+0, −2/+4 et −13/+4 mm pour une ouverture
  utile de 95 mm. Cycle complet en **115 s**.
  `ros2 run mycobot_gateway sim_sorting_grasp`.
- **[`docs/PICK_AND_PLACE_SIMULATION.md`](docs/PICK_AND_PLACE_SIMULATION.md).**
  Le banc de tri en simulation : résultat mesuré, comment le lancer, le graphe
  ROS, la géométrie de la pince en chiffres (point outil, ouverture et
  encombrement selon l'angle), le cycle étape par étape, et les trois
  contraintes non évidentes — bac vert par-dessus l'épaule, plafond de
  hauteur à ~140 mm, doigts qui entrent dans le bac mais ne peuvent pas s'y
  ouvrir.

- **`mycobot_gateway/setup.cfg`.** Il manquait : sans lui `setuptools` installe
  les points d'entrée dans `install/mycobot_gateway/bin`, où `ros2 run` ne
  regarde pas. Tout nœud ajouté au paquet depuis la migration vers Osama_ws
  restait donc introuvable (« No executable found ») alors que la compilation
  réussissait. Les exécutables présents dans `lib/` dataient d'avant.

- **Frottement sur les doigts de la pince simulée** (μ = 1,6 sur `gripper_left1`
  et `gripper_right1`, plus `kp`/`kd`). Ils n'en avaient aucun de déclaré alors
  que les cubes du monde de tri sont à μ = 1,0.
- **Course de la pince simulée portée de 0,7 à 1,10 rad.** Mesuré dans Gazebo :
  les doigts sont écartés de 136 mm au repos et encore de **67 mm** à 0,7 rad —
  trop pour pincer un cube de 40 mm. La course vaut ~98,6 mm/rad ; à 1,10 rad
  ils se referment à **17,9 mm**.

### Corrigé (suite — 31/08)

- **Les deux barres extérieures de la pince ne sont plus soudées à la bride.**
  `gripper_left2` / `gripper_right2` étaient déclarées `fixed`, donc absorbées
  dans `link6` (`link6_fixed_joint_lump__gripper_left2_visual`) : elles
  restaient immobiles pendant que le doigt tournait, d'où deux bras noirs à
  l'horizontale — la pince paraissait cassée sur les côtés alors que la saisie
  fonctionnait. Ce sont en réalité les barres extérieures d'un quadrilatère
  articulé : mesuré sur les meshes, la barre part du pivot (−0,047 ; −0,010)
  dans une direction parallèle **à 0,0° près** à la bielle motrice. C'est donc
  un parallélogramme, et la barre tourne du même angle que son servo. Passées
  en `revolute` et pilotées : le bout de barre reste à 1,0–1,3 mm du doigt sur
  toute la course. Le `gripper_position_controller` attend désormais **6**
  valeurs et non 4 (`teleop/mycobot_teleop.py` mis à jour en conséquence).

- **L'objet est posé au fond du bac, plus lâché au-dessus.** Il tombait de 15 à
  25 mm, rebondissait sur la paroi et restait couché sur le rebord (mesure :
  cube bleu à 44,6° d'inclinaison, les trois autres à 0,0°). La collision des
  doigts avec le bac demande **deux** conditions simultanées — être sous le
  rebord (30 mm) ET plus écarté que la paroi interne (±47,5 mm) — or refermés
  sur l'objet les doigts ne font que ±34 à ±44 mm. Ils peuvent donc descendre
  au fond. Le lâcher se fait en deux temps : on rend la largeur exacte de
  l'objet (force de serrage nulle, il repose déjà), on remonte, puis seulement
  on ouvre en grand. Marge latérale la plus faible : 1,5 mm sur le cube bleu.
  Résultat : les quatre objets à 0,0° d'inclinaison, au fond.

- **Cycle 4 objets ramené à 115 s.** Chaque mouvement attendait une durée
  **fixe** (4 s de trajectoire + 2,5 s de repos, soit ~6,5 s × 8 mouvements par
  objet, l'essentiel du temps passé à ne rien faire). `move_to` dimensionne
  désormais la durée sur le trajet réel et rend la main dès que l'écart passe
  sous 0,35°. L'IK est passée de 150 à 60 itérations et s'arrête au premier
  résultat franc au lieu de balayer les 12 orientations.

### Notes — self-calibration markerless, mesures du 31/08

Trois causes de mauvaise détection ont été isolées, chacune divisant par trois
le nombre de keypoints trouvés. Elles valent pour toute capture DREAM en direct :

- **Le tampon V4L2 doit être vidé avant chaque capture.** Sans ça on lit une
  trame antérieure, prise *pendant* le déplacement, donc floue : détection de
  7/7 à 2/7 pour cette seule raison. Six lectures à jeter suffisent.
- **Le bras doit rester au-dessus de la planche en bois.** Des poses plus
  dépliées le sortent sur le tapis gris et les pieds du trépied : 7/7 → 0-1/7.
  Le fond compte plus que l'étalement des keypoints.
- **La pince doit pointer vers le bas**, comme en travail. Les poses de
  calibration à `J5=90` mettaient l'outil **à l'horizontale** (90° de la
  verticale mesurés sur −X bride, contre 7-15° pour `pick`/`pick_approach`) :
  orientation jamais prise en production, donc hors de la distribution sur
  laquelle DREAM a été affiné. Les poses sont désormais dérivées des points de
  travail enregistrés, en ne faisant tourner que J1 — cette rotation conserve
  exactement l'inclinaison de l'outil.

Une fois ces trois points corrigés : **4/4 keypoints sur six poses sur huit,
29 correspondances**, contre 10 au départ.

**Ce qui bloque encore** : le résidu de reprojection reste à 10,97 px avec
`link3` à **23,6 px sur 6 points**. Avec de bonnes détections partout, aucune
position de caméra unique n'explique les quatre keypoints — c'est un désaccord
**systématique**, donc un problème de MODÈLE et non de capture (FK qui ne colle
pas au robot réel, biais de détection propre à `link3`, ou intrinsèque `cam_3`).
Le garde-fou refuse d'écrire l'extrinsèque tant qu'un keypoint est aberrant,
même quand la médiane passe.

**Fait dur à connaître** : `link1` et `link2` sont le **même point 3D** (vérifié
sur 400 poses aléatoires, écart maximal 0,000 mm). Les compter tous les deux,
c'est compter deux fois la même mesure. Et vus d'une caméra au zénith, `base`
les rejoint sur le même pixel — la perspective ne les sépare que parce que la
caméra n'est pas exactement à l'aplomb de la base.

**Question ouverte** : `arducam_extrinsic_dream_v4.yaml` (en production) place la
caméra à **1,572 m** de la base, alors que `arducam_extrinsic_markers.yaml` et
`arducam_extrinsic_handeye.yaml` disent 1,095 et 1,118 m — ces deux références
indépendantes s'accordant à 2,3 cm près. Non tranché : une self-calibration
markerless ne peut pas se valider elle-même, elle dit que deux extrinsèques sont
en désaccord, pas laquelle a raison.

### Notes

- La saisie **physique** en simulation fonctionne depuis le 31/08 (voir
  `sim_sorting_grasp` ci-dessus). Le servo gauche bloqué venait de bornes
  posées exactement sur 0 dans l'URDF, corrigé le même jour. Trois autres
  causes ont été mesurées puis levées, et elles valent pour le vrai bras :
  - **le point outil est le centre des PATINS, pas le bout des doigts.**
    Viser l'extrémité place la consigne 15 mm trop loin sur l'axe Z de la
    pince — soit exactement la largeur d'un patin. Le cube de 40 mm rattrapait
    l'erreur par sa largeur, le cylindre de 44 mm non : les quatre joints
    atteignaient la consigne **au millième**, preuve de zéro contact.
    `TOOL_OFFSET` vaut `(-0.001, +0.0078, 0.166)` m dans le repère link6.
  - **le poignet ne doit pas tourner entre la saisie et la levée.** Résoudre
    l'IK indépendamment à chaque hauteur laisse φ changer d'un point au
    suivant, et l'objet se dévisse des doigts. `solve_column` impose un φ
    unique à toute une colonne de poses.
  - **le bac vert n'est atteignable que par-dessus l'épaule.** L'outil sort à
    ~22° d'azimut de J1, ce qui demanderait J1 ≈ 187° à l'azimut 164,7° :
    au-delà de la butée. La branche J1 ≈ −35°, J3 > 0, J5 < 0 y va.
- Monter les gains du `mycobot_controller` (100 → 2000-4000) a été essayé pour
  compenser l'affaissement et **retiré** : le suivi empire (J1 raté de 34 deg,
  J5 de 22 deg, contre 3 à 13 avant).

- **Un statut « objet saisi » sur une pince VIDE ne trompe plus la machine.**
  Mesuré le 28/08 sur la figurine imprimée, roulis 0 : la pince rend statut 2
  avec un angle de **22** — deux degrés au-dessus de la pince vide — et la
  figurine est retrouvée **poussée de 18,3 mm**. `_saisie` concluait sur le
  statut seul : elle partait en remontée pour rien, *et* n'inscrivait jamais le
  roulis raté, condamnant la machine à rejouer l'angle qui pousse. Le statut est
  désormais démenti par l'angle quand celui-ci est lisible (un angle illisible,
  lui, ne dément rien). `_saisie` lit l'angle via `Pont.angle_pince()` au lieu
  de reparser la réponse brute.

- **Une lecture parasite de la pince ne fait plus lâcher l'objet.** Mesuré le
  28/08 sur douze lectures pendant une remontée en trois paliers : deux statuts
  « 6 » (la pince ne rend que 0-3) et un angle « 65535 » (le −1 du registre lu
  en 16 bits non signés). Une seule suffisait à conclure « objet lâché » — la
  remontée s'arrêtait et la pince s'ouvrait **en l'air**, jetant le rouleau
  hors de la planche. `porte_objet` relit désormais jusqu'à trois fois, ignore
  tout témoin illisible, et répond « tenu » si les deux témoins le restent :
  croire tenir ce qu'on ne tient pas coûte un cycle, croire avoir lâché ce
  qu'on tient jette l'objet.
- **Un roulis qui POUSSE l'objet n'est plus rejoué** (`ctx.prises_ratees`).
  À 389 mm seules les inclinaisons −30 et −45 résolvent, et à −30 les roulis
  0, 30, 60 et 90 passent tous la géométrie : le roulis 0 referme la pince à
  vide et pousse le rouleau de 6,4 mm, le roulis +30 le saisit (statut 2,
  angle 26) et le tient jusqu'à Z=170. La géométrie ne les sépare pas ; seule
  la prise réelle le fait. Le couple raté passe en dernier — écarté, jamais
  supprimé, sinon un objet devient insaisissable.
- **L'outil se couche avant que le vertical ne bloque**
  (`INCLINAISONS_PAR_PORTEE` : seuil 325 → **320 mm**). Le seuil était « un
  milieu raisonné, jamais vérifié ». Balayage de `colonne_continue` : le
  vertical descend jusqu'à 320 mm et refuse à partir de 324, l'outil couché
  passe partout. Le rouleau blanc à 324 mm tombait exactement du mauvais côté
  et refusait de descendre.

- **Le rouleau se saisit par sa BANDE, plus par son trou**
  (`PRISE_PAR_EPAISSEUR` inclut `scotch`). Il gardait le centroïde de son
  anneau — c'est-à-dire le trou. Mesuré le 27/08 sur le rouleau bleu : la prise
  se fait (statut 2, angle 24, signature d'un rouleau tenu) puis l'objet glisse
  à la remontée, et chaque essai raté le **pousse** — 65 mm de dérive en sept
  tentatives. Six décalages latéraux (16 et 22 mm dans les quatre directions)
  échouent, et le couple monté à 250 aussi : ni la visée ni la force, les
  doigts ne prenaient qu'un quart de rouleau.
- **Couple de serrage par catégorie** — `scotch` à 250, le petit robot reste au
  défaut (pièce imprimée à maillons fins, montée à 250 le 25/08 puis
  redescendue le même jour).
- **La descente ne creuse plus sous la planche** — `Z_PRISE_MIN` passe de −18 à
  **−8 mm**, exactement la consigne nominale. Approfondir ne sert à rien
  (mesuré : à 312 mm, −14/−18/−20 se referment tous sur du vide et les doigts
  touchent la planche) et le bras arrive déjà 6 à 25 mm sous sa consigne. La
  prise nominale ne bouge pas d'un millimètre ; seules les reprises sont bornées.
- **Une tache trop grande n'est plus un carton** (`AIRE_CARTON_MIN/MAX`, 0,4 à
  2,0 fois l'aire attendue). Le 27/08, marqueur 11 absent, une tache de
  230 × 236 mm — **543 cm², plus de quatre fois le grand carton** — a été nommée
  « petit » à 226 mm, c'est-à-dire sur la balle, et le cycle est parti en boucle.
  Le gabarit par côtés ne pouvait pas l'arrêter : porté à 260 mm pour un carton
  vu de biais, il accepte un carré de 236. Le test se fait **après** le
  recentrage sur le cœur sombre — avant, il effaçait aussi le vrai grand carton.
- **La reconstruction par marqueur ne téléporte plus une boîte** : elle ne sert
  qu'à garder en vie une boîte déjà suivie qui vient de perdre son ouverture
  parce qu'on l'a remplie. Le 27/08 l'étiquette « petit » s'est posée **sur le
  bras**, à 210 mm, pendant qu'il portait la balle.
- **Le journal se déverse sur la sortie standard** — il ne vivait que dans la
  fenêtre Qt, et la seule trace après un cycle raté était une capture tronquée.

### Ajouté

- **Une boîte pleine se place par son marqueur** (`Vision._apprend_forme` /
  `_forme_depuis_marqueur`). Une boîte qui se remplit perd son ouverture (mesure
  du 25/08 : 126 → 59 cm²), donc son nom *et* le polygone dont le point de
  largage a besoin. L'écart marqueur → ouverture, appris quand elle était vide,
  la replace — exprimé dans le repère du MARQUEUR, il suit aussi une boîte
  tournée. Mesuré le 27/08 sur le vrai banc, écart gelé 5 s plus tôt :
  reconstruction à **6,5 mm d'écart médian (13 max)** pour le petit et **10,3
  (16,1)** pour le grand, contre 28 à 40 mm de marge intérieure du point de
  largage. Rien n'est persisté.
- **`pose_marqueur` rend l'orientation du marqueur** dans le plan de la planche,
  prise sur son premier côté ramené en base.

### Corrigé

- **Une ouverture là où une boîte est déjà suivie n'est plus prise pour un objet
  à trier.** Une boîte ne cesse pas d'être une boîte parce qu'on y met quelque
  chose. Mesure du 27/08, petit carton à (392, −133) marqueur 11 invisible : son
  ouverture est trouvée 38 fois sur 40 et **volée par le filtre des objets 33
  fois** — nommée « petit » 5 fois sur 40. Après correction : **75/80**, puis
  80/80 une fois le marqueur revenu.
- **La hauteur de largage compense l'affaissement du bras**
  (`z_largage_commande`, `affaissement_largage`). Mesuré le 27/08 : la pointe
  arrive 11,9 mm sous la consigne à 340 mm de portée et 25,3 mm à 470 — la garde
  de 25 mm au-dessus du rebord était donc mangée dès 410 mm et **nulle à 470**
  (pointe à 82,7 mm pour un rebord mesuré à 82,9). Vérifié à un azimut autre que
  celui de la régression : 108,0 / 107,1 / 109,0 mm atteints pour 108 voulus.

- **La pince ne s'ouvre plus si le bras n'est pas arrivé** (`_largage`,
  `ECART_LARGAGE_MAX = 45 mm`). Le 26/08 le largage a été commandé en
  (401, 204) à 450 mm, le bras s'est immobilisé en (373, −52) — 256 mm avant —
  et la pince s'est ouverte : le petit robot est tombé à côté du petit carton.
  `va_vers` rend la main quand le bras ne bouge **plus**, ce qui n'est pas la
  même chose qu'être à la cible. Au-delà de la tolérance, l'objet reste en main
  (`ECHEC_PORTANT`) et le point jamais atteint est mémorisé
  (`Contexte.largages_rates`) pour ne plus être proposé.
- **Une boîte identifiée par son marqueur ne se déplace plus sur une occlusion**
  (`SuiviCarton.marque_vue`). Le bras qui se place au-dessus du carton pour
  déposer le fait sortir de vue plus que `PEREMPTION_CARTON`, et la détection
  revient décalée de 50 à 170 mm ; cette position d'occlusion était adoptée,
  comptée comme un déplacement (« carton grand déplacé » ×4 dans le journal) et
  périmait le point de largage en plein transfert. La péremption ne relocalise
  plus une boîte marquée, et un saut marqué demande **deux images concordantes**
  (~0,2 s) au lieu d'une seule.
- **`marque` signifie « cette position vient du marqueur »**, non « le marqueur
  est visible quelque part » : une position relayée par la SVPRO, corrigée d'un
  décalage appris de 14 à 60 mm, n'a plus cette autorité.

### Modifié

- **Un carton au-delà de `PORTEE_LARGAGE_CONFORT` (400 mm) se vise par son bord
  proche**, plus par son milieu : tous les candidats gardent le même recul des
  parois, donc le bord proche dépose dedans lui aussi. Sur le carton du 26/08 le
  premier point de largage passe de 450 à 431 mm.

### Ajouté

- **Un objet déjà dans un carton n'est plus une cible** (`Fenetre._depose`,
  `MARGE_DEPOSE = 15 mm`) — la balle déposée était redétectée au fond de la
  boîte, **35,4 mm à l'intérieur de l'ouverture**, et le cycle repartait la
  chercher.
- **Le petit robot se saisit par son point le PLUS ÉPAIS**
  (`Vision.point_le_plus_epais`, `PRISE_PAR_EPAISSEUR`) — transformée de
  distance, moyennée sur tout ce qui dépasse 85 % de l'épaisseur maximale. Le
  centroïde de l'enveloppe convexe suivait les membres articulés : mesuré
  **23,5 mm hors du ventre**, sur un ventre de 41 mm de large. C'est ce qui
  faisait refermer la pince sur un maillon, d'où `objet saisi` puis `objet
  lâché pendant la remontée`. Le scotch garde le centroïde — c'est le centre de
  son anneau. Après correction : `objet saisi (angle 39)`, tenu jusqu'au
  largage.
- **Un décalage SVPRO par carton** au lieu d'un seul pour les deux — mesuré, la
  SVPRO tombe à 14 mm de l'arducam sur le grand carton et à 60 mm sur le petit,
  qu'elle voit par la tranche. Une moyenne des deux est fausse pour les deux.

- **L'ouverture du carton se mesure sur son cœur sombre** (`_coeur_sombre`) —
  une paroi de carton à l'ombre est sombre elle aussi, elle se colle à
  l'ouverture et le contour les avale toutes les deux. Le petit carton, **115 ×
  70 mm au mètre ruban**, était ainsi mesuré 105 × 203 mm. Un seuil d'Otsu à
  l'intérieur du seul creux les sépare : **67 × 115,5 mm**, soit la mesure
  réelle. Conséquences en chaîne — le point de largage se choisit sur ce
  polygone, donc il tombait au-dessus de la paroi plutôt que dans la boîte ; et
  les deux cartons, mesurés faux, devenaient indiscernables.
- **Rien à moins de 200 mm de la base n'est un carton** (`RAYON_BASE_MIN`) — le
  bras au repos était détecté comme un creux de 70 × 164 mm à 57 mm de la base
  et prenait le nom de « petit carton ». C'est le carton fantôme au milieu de la
  table, qui ne bougeait pas quand on déplaçait le vrai. Le masque cinématique
  ne suffit pas : il exige les angles, donc le pont vers la Pi, et sans lui il
  ne masque rien.
- **Désignation du grand carton par un clic** (`VueCliquable`,
  `_designe_grand`) — un clic sur un carton dans le flux caméra le déclare
  GRAND, l'autre devient le petit ; gardé dans `scripts/cartons_designes.json`,
  il suit les cartons qui bougent. Filet de sécurité : depuis que l'ouverture
  est mesurée juste, le gabarit sépare les deux cartons de lui-même. Le
  détecteur n'écrit jamais la désignation seul — laissé libre, il y a inscrit
  l'ombre du bras comme « petit carton » à (54, −18).
- **Identification des cartons par marqueur ArUco collé** *(optionnel, second
  moyen d'arriver au même résultat)*
  (`scripts/aruco_service.py`, `Vision.cartons_marques`) — `id 10` = grand
  carton, `id 11` = petit, 45 mm de côté, collés **à plat sur un rabat**. Le
  marqueur donne le nom sans ambiguïté *et* la hauteur du rebord (son plan est
  celui du rebord). `cv2.aruco` fait segfaulter l'OpenCV 4.6 du système où
  tourne le tableau de bord : la détection est déportée dans un processus du
  venv qui reste ouvert et reçoit les images par un tube — **4,5 ms par image**,
  contre ~1 s si on relançait un interpréteur à chaque fois. Service absent, la
  géométrie reprend la main sans bruit. Feuille à imprimer à 100 % :
  `~/marqueurs_cartons.png`.
- **Identité des cartons par continuité** (`CONTINUITE_CARTON = 150 mm`) — un
  carton déjà nommé garde son nom tant qu'il reste près de là où on l'a vu.
  C'est ce qui permet de le déplacer à la main sans qu'il échange son nom avec
  l'autre. L'ordre de décision est désormais : marqueur, puis continuité, puis
  robe et gabarit (ce dernier réduit au rôle d'amorce).
- **Hauteur de largage calée sur le rebord mesuré** (`fsm.z_largage`,
  `GARDE_LARGAGE = 25 mm`) — au lieu d'un rebord supposé à 60 mm. La garde
  réelle était de 17 mm, et négative pour un carton plus haut.
- **Tri par catégorie : trois classes d'objets, deux cartons de destination**
  (`scripts/pick_dashboard.py`, `scripts/pick_fsm.py`) — `scotch` → petit
  carton, `balle` et `robot` → grand carton. Le **cycle** est validé sur le
  robot réel le 24/08 pour les trois classes (prise, transport, largage dans un
  carton). L'**étiquetage grand/petit ne l'est pas** : voir « Connu, non
  résolu » plus bas. Chaque classe est reconnue par une signature qui ne
  dépend pas de l'éclairage : le **scotch est un anneau** (un trou dans le
  contour — à l'exposition 75 son bleu se lit H15 S90 V48, indistinguable du
  bois sombre), le **robot est noir désaturé** (S=44 contre S=170 pour le bois
  même à l'ombre) et long d'au moins 60 mm, la **balle** garde son détecteur
  jaune. Cycle mesuré : 58 à 99 s, saisie confirmée par le statut pince.
- **Deux cartons détectés et suivis séparément** — un `SuiviCarton` par
  destination, chacun rattrapé en 0,12 s quand on le déplace à la main, sans
  que la cible de l'autre bouge.

### Corrigé (25/08, après-midi)

- **Les objets étaient mesurés au plan du rebord des cartons (83 mm)** au lieu
  du plan où ils reposent (`HAUTEUR_OBJET`, 12 mm) — soit **7 % trop gros**.
- **Plafond du gabarit scotch 70 → 90 mm** : le rouleau blanc en fait 72,8 et
  était rejeté avant même d'être classé. Un seul des deux scotchs était détecté.
- **Plafond du gabarit robot 140 → 200 mm** : le petit robot a des membres
  articulés et son encombrement dépend de la pose où on le trouve — 71×109 mm
  ramassé, **79×146 pattes étalées**. À 140 il était rejeté pour 6 mm.
- **Le couple de la pince avait été monté à 250 pour le robot, puis annulé** :
  c'est une pièce imprimée en 3D à maillons fins, que le serrage casserait. Le
  lâchage venait de la visée, pas de la force. La raison est écrite dans le
  code pour qu'on ne le remonte pas.

### Validé sur le robot réel (25/08)

| Objet | Carton | Largage | Prise | Cycle |
|---|---|---|---|---|
| scotch blanc | petit | (358,7 · −156,7) Z=89,8 | confirmée, angle 30 | 64 s |
| scotch bleu | petit | (349,8 · −167,4) Z=90,5 | confirmée, angle 25 | 96 s |
| petit robot | grand | (311,0 · 167,4) Z=91,3 | confirmée, angle 39 | 68 s |

### Mesuré (25/08)

- **Le rebord des cartons est à 82,9 mm, pas 60.** Triangulation des deux vues
  sur l'ouverture, écart des rayons 12 mm. Or 60 mm était le plan sur lequel
  toute la géométrie des cartons se projetait : entre Z=0 et Z=100 le centre
  d'un carton se déplace de **50 mm**.
- **La piste « hauteur des parois par les deux caméras » est fermée.** Elle
  tient sur le carton proche (Z = 82,9 mm, écart 12 mm) et pas sur le lointain :
  la SVPRO en voit la **paroi du fond par la tranche**, pas l'ouverture, et la
  triangulation rend Z = **−27 mm** — sous la table — avec 39 mm d'écart entre
  les rayons. Ni l'appariement des centroïdes ni le recouvrement des masques ne
  la rattrapent.
- **Les dimensions séparent les deux cartons, une fois l'ouverture mesurée
  juste.** Contours gonflés par l'ombre des parois : 160×214 et 144×205 mm, 16 %
  d'écart pour 18 % de bruit — indiscernables. Cœurs sombres : **115 ± 7 cm²
  contre 74 ± 1 cm²**, soit six fois le bruit. La piste « dimensions » était
  bonne, c'est la mesure qui était fausse.
- **Détection des deux cartons, 25 images consécutives, bras dégagé** :
  avant 2/20 et 14/20 avec l'étiquette qui basculait à chaque image ; après
  **25/25 et 25/25**, étiquette stable, tremblement du centre 0,5 et 1,1 mm.
- **Point de largage vérifié dans les deux cartons** : marge aux parois +32 mm
  (petit) et +41 mm (grand), tous deux atteignables, largage à Z = 108 mm.
- **Enveloppe de largage balayée sur tout le plateau** (IK seule, pas de
  mouvement) : X de 200 à 480 mm, Y de −240 à +240 mm par pas de 40 mm.
  **Aucun trou** — toute position en deçà de `PORTEE_CARTON_MAX` (460 mm) admet
  une solution de largage, l'inclinaison de l'outil passant de 0° au centre à
  −60° aux angles. Un carton déplacé n'importe où sur la planche est donc
  atteignable.

### Connu, non résolu

- **Un objet déposé déforme le creux de son carton.** Le scotch blanc lâché
  dans le petit carton l'a fait passer de 74×127 à **83×172 mm** : l'aire ne
  sépare plus les deux boîtes et l'étiquette peut s'inverser sur une détection
  à froid. Contourné — la désignation est gardée dans
  `scripts/cartons_designes.json` et sert d'amorce — mais la mesure elle-même
  reste fausse tant qu'il y a quelque chose dans la boîte.
- **Le suivi du petit carton a sauté à (500 · −27), hors planche**, pendant le
  cycle du robot du 25/08. Sans conséquence — le robot visait le grand — mais
  c'est une fausse détection à traiter.

- **Lequel des deux cartons est le grand — RÉSOLU le 25/08.** La cause n'était
  pas le critère mais la mesure : le contour avalait l'ombre de la paroi et
  gonflait l'ouverture. Cœurs sombres, les deux cartons sont à 115 et 74 cm²,
  six fois le bruit. Restent en filet trois autres moyens d'y arriver — le clic,
  le marqueur ArUco, la continuité. Historique de ce qui avait été essayé sur
  les contours gonflés et n'avait pas tenu :
  Ni l'aire de l'ouverture (10 915 contre 9 981 mm² à une position, 138×202
  contre 62×113 mm à une autre — elle dépend trop de l'angle de vue), ni la robe
  (brun contre noir, mesurée `S174 V87` / 2 % de pixels sombres contre
  `S148 V52` / 59 %) n'ont tenu : le sens a dû être inversé deux fois et le
  24/08 au soir le scotch bleu, dirigé vers `petit`, a atterri dans le grand
  carton. Le transport et le largage sont justes ; c'est l'identité de la boîte
  qui ne l'était pas.
- **Masque du bras déduit de la cinématique** (`Vision.masque_bras`) — la
  silhouette du bras est projetée et retirée de l'image avant toute détection.
  Son ombre était un creux sombre cerclé de brun, donc une ouverture de carton
  parfaite.
- **Hauteur de prise par catégorie ET par régime** (`Z_PRISE_PAR_CLASSE`) —
  balle (−5 / 25 mm), scotch (2 / 11), robot (2 / 10). Chaque essai de saisie
  raté descend ensuite de 4 mm plutôt que de refaire le même geste.
- **Enveloppe de dépose portée de 360 à 460 mm** — l'outil se couche aussi pour
  larguer, comme il le fait déjà pour saisir.

### Corrigé

- **Le carton fantôme au milieu de la table** — trois fois de suite la machine a
  visé une ouverture inexistante (211, 1), (205, −16), (212, 6) et y a lâché
  l'objet. C'était l'ombre du bras : elle le suit image après image, donc elle
  se confirme aussi bien qu'un vrai déplacement, et aucun filtre de forme ou de
  taille ne l'arrête. Un carton ne peut plus être localisé à moins de 200 mm du
  bras, distance mesurée au bras **entier** et non à sa pointe. La mémoire sur
  disque contenait aussi ce fantôme et le ressortait à chaque redémarrage ; elle
  est désormais **une par carton** et n'est écrite que sur une détection propre.
- **La cible sautait d'un objet à l'autre en cours de descente** — le bras
  masquait le scotch visé, le détecteur trouvait l'autre rouleau 311 mm plus
  loin et la machine repartait à zéro. Porte de suivi à 80 mm.
- **Un changement de caméra n'est plus lu comme un déplacement de l'objet** —
  les 10 mm d'écart de repérage entre l'arducam et la SVPRO relançaient un
  `DETECTION` complet (11 s perdues, objet immobile).
- **Le dégagement exigeait la balle spécifiquement** et pouvait boucler sept
  fois à vide pendant que d'autres objets attendaient ; il exige maintenant la
  **vue de dessus** de n'importe quel objet, et ordonne son balayage par
  l'azimut de la cible dès le premier cycle.
- **Le scotch passait pour le robot** quand son trou n'était pas vu : la
  fermeture morphologique le bouchait (28 px pour l'anneau bleu). Fermeture
  supprimée, et le robot exige désormais 60 mm de longueur minimale.

### Mesuré

- **La limite verticale de 355 mm est mécanique.** Elle avait été mesurée en
  exigeant aussi `Z_TRANSFERT` au-dessus du point de prise ; en ne demandant que
  le survol et la prise, elle ne bouge pas d'un millimètre (350 mm passe,
  360 non). Piste fermée, ne pas re-tenter.
- **La SVPRO ne peut pas classifier comme l'arducam** — vue oblique, elle prend
  une paroi de carton pour le robot et ne voit pas les anneaux du tout. Elle
  fournit des positions ; c'est l'arducam qui nomme. Les deux vues affichent
  alors les mêmes étiquettes.

### Ajouté

- **Invariant de dépose : objet en main ⇒ jamais de retour au ramassage**
  (`scripts/pick_fsm.py`) — garde unique et non contournable dans
  `MachineEtats.pas()`. Deux états nouveaux : `RECHERCHE_CARTON` (cherche le
  carton en hauteur sans lâcher, balayage J1, repli sur la dernière position
  connue persistée) et `ECHEC_PORTANT` (garde l'objet et s'arrête, même en
  automatique). Seule une perte de prise — statut pince 1 ou 3 — relance la
  saisie. Le cycle tournait auparavant avec la balle en main indéfiniment :
  `TRANSFERT` échouait → `ECHEC` → `ATTENTE` → `DEGAGEMENT`.
- **Le carton est jugé AVANT la saisie** — `DETECTION` refuse de démarrer un
  cycle si le carton n'est ni vu ni mémorisé à portée. C'est le seul moment où
  le bras est dégagé et où la caméra le voit sans obstacle.
- **Point de largage choisi dans l'ouverture, au plus près du milieu** — viser
  le centre n'est pas obligatoire : mesuré, un centre à 363 mm inatteignable
  (à *toutes* les hauteurs de largage, 100 à 160 mm) pour une ouverture dont le
  bord proche est à 290 mm. Les candidats sont classés par distance au milieu,
  reculés des parois. Mesure : largage à 2–22 mm du milieu, 31–43 mm de marge.
- **Suivi du carton (`SuiviCarton`)** — lissage tant que la détection reste
  proche, hystérésis de 6 images concordantes avant d'admettre un déplacement,
  garde de taille (±45 %), péremption 4 s. Le rectangle affiché est la position
  suivie, pas la détection brute.
- **La SVPRO assiste l'arducam** — l'arducam reste la source du X/Y ; la SVPRO
  ne fournit que la hauteur, par triangulation des deux rayons, remplaçant
  l'hypothèse « centre de balle à 35 mm ». Relais complet quand le bras masque
  la vue de dessus. Refus si les rayons s'écartent de plus de 25 mm. Mesure du
  24/08 : écartement 2,3 mm, hauteur réelle 29,9 mm, correction XY 2,01 mm.
- **Chronomètre de cycle** — durée par état dans le journal, secondes qui
  défilent sur l'étape en cours, ligne `CYCLE COMPLET : XX s — les 3 étapes les
  plus coûteuses` à chaque retrait.
- **Tests** — `tests/test_pick_fsm_depose.py` (17 tests : invariant de dépose,
  point de largage, descente depuis le bras dressé) et
  `tests/test_suivi_carton.py` (8 tests).

### Corrigé

- **Chaque mouvement coûtait 22,7 s d'attente** (`va_vers`) — l'arrivée était
  jugée sur l'atteinte de la consigne à 1,2° près, or l'affaissement laisse un
  écart permanent d'environ 1,9° sur J2 : le test n'était jamais satisfait et
  l'attente allait au bout de sa patience. Mesuré identique à vitesse 25 et 50,
  ce qui prouve que le temps ne venait pas du robot. L'arrivée se détecte
  désormais à l'**immobilité** du bras : **22,7 s → 1,35 s par mouvement**, à
  vitesse inchangée.
- **Détection du carton par la couleur** — carton H14 S171 V60 et planche
  H15 S187 V84 : même teinte, l'ancien seuil prenait la planche entière pour un
  carton (tache de 30 000 px², centroïde à 300 mm de la vraie boîte). Remplacé
  par la recherche d'un **creux sombre entouré de brun** dans le plateau projeté
  depuis les marqueurs.
- **Matrice intrinsèque non rescalée** (`pick_dashboard.Vision`) — la SVPRO est
  calibrée en 800×600 et lue en 640×480 ; avec la matrice brute du `.npz`, les
  marqueurs se reprojetaient à 200 mm de leur position. Passe par
  `camera_registry.load_intrinsics`.
- **`PORTEE_MAX` refusait des balles atteignables** — 335 mm alors que la
  mesure donne 330, 340 et 350 mm résolus (roulis +30, +30, +60) et 360 non.
  Une balle à 343,6 mm était refusée. Porté à **355 mm**. Incliner l'outil
  n'ajoute rien : testé de +10 à +30°, aucune solution.
- **Plantage du largage sur `None @ TOOL`** — le bouton « détecter le carton »
  remet `R_carton` à `None` ; un clic pendant la boucle faisait lever
  `matmul: Input operand 0 does not have enough dimensions`. Le largage repasse
  par la recherche.
- **Approche refusée depuis le bras dressé** — la pointe au repos est à 518 mm,
  la cible d'approche à 110 : le garde-fou anti-plongeon (220 mm par ordre)
  refusait, et le cycle bouclait. Découpage en étapes interpolées dans l'espace
  **articulaire** — descendre par paliers au-dessus de la balle ne marche pas,
  à Z=340 l'outil ne peut pas être tenu vertical. Le nombre d'étapes se règle
  sur le profil réel : l'interpolation n'est pas monotone, la pointe monte
  d'abord à 562 mm avant de plonger.
- **Veto de dernière seconde avant le largage — retiré** — à cet instant le bras
  masque le carton, la détection sautait de 55 à 171 mm et le cycle repartait en
  boucle sans jamais déposer. Une détection de carton à moins de 120 mm sous la
  pointe est désormais ignorée : c'est le bras ou son ombre.
- **Boucle infinie sur échec** — arrêt après 3 échecs d'affilée sans progrès,
  compteur remis à zéro par chaque dépose réussie ; le fil du robot s'arrête dès
  que la machine ne change plus d'état.
- **Extrinsèque SVPRO** — dérivée à 16,5 mm d'erreur moyenne sur les 4
  marqueurs, recalibrée sur 16 coins (RANSAC+LM, leave-one-out) : **1,57 mm**,
  au niveau de l'arducam (1,24 mm).

### Performance

- **Le solveur ne cherche plus le roulis à chaque fois** — un roulis qui échoue
  épuise les 22 amorces (5 s), celui qui marche répond en 40 ms. Le roulis
  retenu est réessayé en premier, rangé **par bande d'allonge de 25 mm** (celui
  appris à 257 mm échoue à 357 mm). La pose du carton est mise en cache tant
  qu'il n'a pas bougé de plus de 12 mm. Mesure : **16,7 s → 0,04 s** au retour
  sur une position connue ; 0,83 s après un déplacement réel de 110 mm.
- **L'affaissement n'est plus redécouvert à chaque cycle** — la correction
  articulaire est retenue sur disque et réinjectée dès la première passe de
  convergence. Elle valait auparavant deux passes, donc deux mouvements
  (mesure : passe 1 à 10,79 mm, passe 2 à 4,15, passe 3 à 0,38).
- **Pause de stabilisation supprimée sur les transits** — la seconde d'attente
  après un mouvement ne sert qu'avant une *mesure* ; se dégager, transférer et
  se retirer n'en mesurent aucune.
- **Attente de pince adaptative** — on attend un statut décidé (1/2/3) au lieu
  de 2,2 s forfaitaires ; remontée en 3 paliers au lieu de 4.

### Ajouté

- **Asservissement visuel en boucle fermée (`mycobot_gateway/visual_servo/`)** —
  machine à états pick-and-place complète, testable image par image sans matériel :
  fusion multi-caméras, suivi de Kalman avec compensation de latence, loi de
  commande saturée, superviseur de sûreté à 9 conditions, IK différentielle sur
  **matrice de rotation** (jamais d'angles d'Euler). Lancement :
  `ros2 launch mycobot_gateway visual_servo.launch.py` — démarre **désarmé**
  (`dry_run:=true`), attend un `start` explicite sur `/visual_servo/command`.
  47 tests unitaires.
- **Calibration extrinsèque caméra→base** (`training/calibration/calibrate_camera_base_extrinsic.py`) —
  16 coins au lieu de 4 centres, RANSAC+LM, pooling multi-images, validation
  **leave-one-out** (le seul chiffre qui mesure un point neuf). Résultats :
  arducam RMS 1,01 px / LOO 3,1–3,5 mm ; svpro RMS 1,36 px / LOO 0,4–3,2 mm.
  Stabilité vérifiée après 2 jours : **0,8 à 1,7 mm** de dérive.
- **Cycle pick-and-place complet validé sur robot réel (20/08/2026)** — balle
  localisée par vision, approche, descente par paliers, saisie vérifiée par
  statut pince, transport, dépôt en bac confirmé par statut **et** par image.

### Corrigé

- **`send_coords` est inutilisable sur cette unité** — comparaison A/B sur cible
  et métrique identiques (267 mm à parcourir) : méthode officielle Elephant
  Robotics **247,8 mm d'erreur** (9 % du trajet) contre **18,2 mm** (93 %) via
  `send_angles` + IK. Les deux reçoivent `OK` du bridge : la méthode constructeur
  **échoue en silence**. Cause mesurée : blocage de cardan, toute la tâche se
  déroulant entre RY = −78° et −83°.
- **Orientation cible tournée selon l'azimut** — garder une orientation de bride
  fixe en visant un azimut différent tord le poignet : résidu IK **20,0 mm** sur
  33° d'écart, contre **0,19 mm** avec `Rz(Δazimut) @ R_référence`. Placement
  final obtenu à 3,9 / 0,6 / **0,1** mm en X/Y/Z.
- **Compensation de l'affaissement gravitaire** — le bras arrive systématiquement
  ~13 mm plus bas que commandé à vide, ~15 mm chargé, de façon reproductible sur
  tous les paliers. Compensé, l'erreur verticale tombe à **~2 mm**.
- **Branche IK** — toutes les poses historiques (`observation_clear`, pick du
  17/08) sont sur la branche *coude bas*, plaquée contre la butée J2 (**marge 0°**),
  ce qui rendait toute boucle fermée impossible et expliquait les sauts de branche
  de 150° sur J4. La branche *coude haut* (J3 < 0) atteint les mêmes poses avec
  **23 à 72° de marge**. Transition validée sur matériel.
- **Vérification de prise** — un statut de pince **inconnu** ne vaut plus
  vérification réussie (`is not True` au lieu de `is False`) : `bridge_pi_simple.py`
  n'implémente pas `get_pro_gripper_status`, ce qui rendait le contrôle
  silencieusement inopérant. Utiliser **`scripts/gripper_bridge.py`** sur la Pi.
- **Levage de contrôle** — délai d'expiration ajouté : chargé, un levage commandé
  à +5 mm donne **−2,6 mm** réels, et l'étape bouclait indéfiniment.
- **Perte d'objet pendant la montée** — surveillée à chaque période au lieu de la
  seule arrivée à hauteur de transport.
- **Passe de serrage désactivée par défaut** — sans effet mesuré : la pince cale à
  l'angle 52 dès le premier contact et commander 12 ne la bouge pas. Le seul
  levier réel est `set_pro_gripper_torque`.

- **Pick-and-place vision-guidé (démonstrateur autonome)** — localise un objet par
  caméra puis l'exécute en cartésien sur le vrai robot, sans dashboard :
  - `scripts/pick_and_place_vision.py` (contrôle : approche top-down → descente →
    serrage → vérif statut → dépose ; `--keep-ori`, `--approach-ori`).
  - `scripts/pick_and_place_vision_live.py` (perception+glue : détecteur `color`/`yolo`,
    `--camera-source {rosbridge,v4l2}`, `--robot-via {rosbridge,socket}`).
  - `mycobot_gateway/vision/multiview_localizer.py` — extrinsèque DREAM live,
    `pixel_ray`, `triangulate` (2 vues), repli plan-table mono.
  - Orientation de prise MyCobot `[-91.3, 10.2, -148.9]` (pas `[180,0,0]`) ;
    `send_coords` plafonné à ±350 mm côté firmware.
- **Extrinsèque ArUco table** — 4 marqueurs 80 mm à positions mesurées,
  `training/calibration/calibrate_arducam_markers.py` (RMS 0.71 px), localisation au cm.
  Fichiers : `workspace_markers.yaml`, `arducam_extrinsic_markers.yaml`,
  `aruco_markers_workspace.pdf`.
- **Bridge** — action `set_color` (LED Atom) dans `scripts/gripper_bridge.py`.
- **Dataset gripper 8-kp (scaffolding)** — `training/dream/convert_to_ndds_gripper.py`
  (`--gripper-absent`), `training/capture_real_3cam.py` (gripper 110 mm au garde-au-sol).

- **Dashboard DREAM — multi-caméras / fusion (auto-détection)** : le dashboard
  prend maintenant 1 ou 2 caméras calibrées de façon flexible, sans édition de
  code. Nouveaux éléments :
  - `mycobot_gateway/vision/camera_registry.py` — sonde `v4l2-ctl`, identifie
    arducam/SVPRO, charge et **rescale** leur intrinsèque existante (`cam_3` /
    `cam_2` 800×600→640×480), fixe l'exposition (arducam 75, SVPRO normale).
  - **Fusion *solve-then-fuse*** — chaque caméra résout d'abord son propre `q`
    (mode cohérence par vue), puis on fusionne **par joint**, pondéré par
    l'observabilité (keypoint observant détecté ET reprojection ≤ 15 px). Le `q`
    fusionné n'est jamais pire que la meilleure caméra sur chaque joint, et lève
    l'occlusion (une vue reprend ce que l'autre perd). Remplace le bundle partagé
    `solve_joint_angles_multiview` (conservé mais inutilisé) qui basculait de
    branche (J1 −43°). Repli **MONO via {caméra}** si la primaire devient aveugle.
  - `launch/dream_multicam.launch.py` — launch unique qui auto-détecte les
    caméras et spawne une branche `camera_publisher + dream_inference` par
    caméra + `joint_sync` + `bridge_tour` + dashboard.
  - Dashboard : param `cameras`, badge « 🔗 FUSION N vues » / « MONO via … »,
    vues empilées **verticalement** (chaque vue secondaire porte le même HUD que
    la primaire). **Tableau keypoint = fusion** (erreur moyenne des caméras
    détectant chaque point, `(fusion)`/`(caméra)`/`non détecté`) + ligne
    **Détection globale (fusion) : N/7 kp** (union des vues).
  - `camera_publisher` (param `output_topic`) et `dream_inference` (param
    `output_prefix`) paramétrés pour lancer une instance par caméra.
  - Astra hors périmètre (pas de nœud V4L2 ni d'intrinsèque PnP).
  Détails : [`docs/DREAM_VALIDATION_DASHBOARD.md`](docs/DREAM_VALIDATION_DASHBOARD.md)
  § Multi-caméras. ⚠ Fusion 2-cam non encore validée sur matériel réel (SVPRO à
  brancher) ; chemin mono validé.
- **Dashboard DREAM — 3 filtres temporels au choix (aucun par défaut)** : groupe
  de boutons radio `aucun` · `kalman` · `passe_bas` (EMA α=0.3) · `moyenne`
  (fenêtre glissante 6). Le Kalman n'est **plus** activé d'office. Changer de
  filtre purge les trois états (`reset_kalman()`). Le sous-dossier CSV suit le
  filtre actif (`kalman/`, `passe_bas/`, `moyenne/`).
- **Dashboard DREAM — anti-clignotement** : keypoints secondaires **tenus 0.8 s**
  après leur dernière détection (`display_keypoints`, affichage seul — le solveur
  garde les détections réelles) ; pastille de pose verte tant qu'une vue a détecté
  ≥4 keypoints dans la dernière seconde (`recently_detecting`), corrige la pastille
  qui jaunissait sans mouvement.
- **Dashboard DREAM — `reset_kalman()`** : sur `SET Angles` / `SET Coords` /
  `Pose automatique`, les filtres de Kalman sont réinitialisés. Le mouvement
  commandé étant connu comme réel, le portail anti-aberration ne le gèle plus
  (avant : la courbe filtrée restait bloquée sur l'ancien angle, ex. J2 −45°
  rejeté). La prochaine mesure DREAM devient la nouvelle base.
- **Dashboard DREAM — CSV filtrés séparés** : quand un filtre temporel est actif,
  les acquisitions vont dans un sous-dossier au nom du filtre (`…/kalman/`,
  `passe_bas/`, `moyenne/` — colonne `dream` = valeur filtrée) ; sans filtre,
  valeur brute dans le dossier parent. Les deux séries restent comparables sans
  mélange.
- [`docs/DREAM_VALIDATION_LAUNCH.md`](docs/DREAM_VALIDATION_LAUNCH.md) — procédure
  de lancement des 5 nœuds du dashboard de validation DREAM, table de diagnostic
  (quel symptôme → quel nœud manquant), et le piège `.venv` qui casse toute
  commande ROS2 (Qt xcb / `KeyError: 16`). Inclut le diagnostic de latence du
  16/07 (cause racine `net.core.rmem_max`, correctifs caméra mesurés puis revertés).
- [`docs/CAMERA_CALIBRATION.md`](docs/CAMERA_CALIBRATION.md) — calibration caméra.

### Modifié

- **Dashboard DREAM — réglage Kalman** : `q_pos` abaissé `radians(3.0)²` →
  `radians(0.5)²` (courbe plus lisse au repos). Labels KPI `MAE`/`RMSE` annotés
  `(J1–J6)` pour rappeler que le compteur couvre les 6 joints.
- **Dashboard DREAM — poids solveur** `_CONSISTENCY_REG_VEC`
  `[1.5, 1.5, 40, 40, 40, 1.5]` → `[10, 40, 40, 40, 40, 1.5]` : J2 épinglé à
  l'encodeur (40) pour corriger la bascule de branche monoculaire sous la caméra
  quasi-zénithale (~45° → ~2-3°) ; J1 légèrement raffermi (10). Détails et
  arbitrages : [`docs/DREAM_VALIDATION_DASHBOARD.md`](docs/DREAM_VALIDATION_DASHBOARD.md)
  § Filtrage temporel / Poids solveur.

### Diagnostiqué (aucun correctif appliqué)

- **Warnings `⚠️ Dropped frame`** de `camera_publisher` : la caméra négocie
  **YUYV (10 fps max)** au lieu de MJPG (30 fps) — `cv2.CAP_PROP_FOURCC` est
  silencieusement ignoré par le backend GStreamer — alors que le timer tire à
  30 Hz. Deux `read()` sur trois tombent donc à vide. Confirmé par
  `v4l2-ctl --list-formats-ext`.
- **Exposition manuelle=75 trop basse** pour l'éclairage actuel (image quasi
  noire, pixels 11-49/255) — valeur calée sur l'éclairage de session4, à re-tuner
  comme le prévoit le docstring de `set_manual_exposure()`.
- **Gripper** : `bridge_tour` et `dream_validation_dashboard` n'ont **aucun**
  support gripper ; seul `bridge_pi_simple.py` expose `gripper_open`/`gripper_close`
  via l'API générique tout-ou-rien `set_gripper_state(flag, speed)`.

---

## [1.16.0] - 2026-07-13

### 🎯 Ancre de pose multi-vue + diagnostic d'observabilité J5

Investigation déclenchée par un squelette vert (encodeurs/FK) visiblement décalé
du bras réel dans le dashboard de validation, alors que le squelette DREAM
suivait correctement le bras.

- **Bug trouvé et corrigé — ancre de session dégénérée** : l'ancre de pose
  caméra (`dream_validation_dashboard.py`) était calculée comme une médiane de
  30 solves `solvePnP` pris pendant la fenêtre de démarrage — mais le bras était
  systématiquement **statique** à ce moment-là (étendue de pose mesurée : 0° sur
  les 6 joints). Une médiane de solves répétés sur une seule pose ne moyenne que
  le bruit de détection, elle ne peut ni voir ni corriger l'ambiguïté de rotation
  qu'une vue monoculaire unique laisse sous-contrainte pour une chaîne de
  keypoints quasi colinéaire. Mesuré : RMS 10.85px à la pose de capture, RMS
  122.85px après un déplacement de 30° sur J2 (erreur croissante vers le bout de
  la chaîne — signature classique de dégénérescence PnP mono-vue).
  **Fix** : l'ancrage accumule maintenant une fenêtre glissante de 30 à 50 frames
  et n'accepte de figer l'ancre qu'une fois une diversité de pose confirmée
  (≥3 poses distinctes, ≥15° de variation sur ≥2 joints) ; toutes les
  correspondances 3D–2D de la fenêtre sont alors poolées pour ajuster UNE seule
  pose caméra. Message d'instruction affiché tant que le bras reste statique.
  **Aucun fichier d'extrinsèque, ni avant ni après** — l'ancre reste dérivée en
  direct à chaque lancement. Validé sur le robot réel : squelette vert
  correctement projeté après le fix.
- **Diagnostic — J5 structurellement faiblement observable** : seul le keypoint
  `link6` dépend de la rotation propre de J5 (un seul point 2D pour un DoF). Un
  test de sensibilité (`training/dream/j5_observability_test.py`, angle balayé
  sur toute sa plage mécanique, autres joints + pose caméra fixes) montre une
  bande d'ambiguïté de **~15° (vue de face) à ~23° (vue de dessus)** — plusieurs
  valeurs de J5 reprojettent à moins de 5px les unes des autres, indiscernables
  au bruit typique du détecteur. **Conclusion : c'est un problème de géométrie,
  pas de réglage du solveur** — augmenter `reg_weight` masquerait le symptôme
  sans le résoudre. Pistes non implémentées : seconde caméra, ou prior
  géométrique plus fort. `reg_weight` et le solveur laissés inchangés.

### Ajouté

- `training/dream/j5_observability_test.py` — test de sensibilité J5 (caméra
  synthétique construite en code, aucune calibration extrinsèque chargée).
- `training/dream/j5_observability_test*.png` — preuve expérimentale (3 angles
  de vue).
- Avertissement structurel J5 permanent dans le panneau KPI du dashboard.
- KPI « Erreur RMS des angles (J1-J4, hors J5/J6) » — même logique que
  l'exclusion déjà en place pour J6, pour que le chiffre de validation globale
  ne mélange pas silencieusement un joint connu géométriquement ambigu.

### Modifié

- `mycobot_gateway/mycobot_gateway/dream_validation_dashboard.py` — ancrage de
  pose multi-vue (voir ci-dessus) ; logs de diagnostic à la capture de l'ancre
  (couverture par keypoint, étendue de pose, RMS de reprojection sur la
  fenêtre).

---

## [1.15.0] - 2026-07-13

### 🖥️ Dashboard PyQt de validation temps réel DREAM vs encodeurs

Premier des 3 dashboards de validation demandés (voir réunion "Validation Modèle
IA — Cercles EndEffector") : superposition en direct des keypoints reconstruits
depuis les encodeurs (FK) et des keypoints détectés par DREAM, sur le flux caméra
réel.

- **Aucune extrinsèque pré-calibrée** : la pose caméra est résolue à chaque frame
  par `solvePnP` (points 3D FK aux angles encodeurs courants ↔ détections 2D
  DREAM), à partir de l'intrinsèque Arducam seule (`training/calibration/cam_3.meta.json`).
  Évite la péremption d'un fichier d'extrinsèque figé (cf. caméra bougée juin→juillet).
- Testé en aveugle (sans matériel) avec des messages ROS2 synthétiques : overlay,
  calcul d'erreur par keypoint et rendu Qt validés (erreur de reprojection ~0 px
  sur vérité terrain connue).
- Dashboards 2 (pilotage + KPI) et 3 (courbes 6 joints encodeur vs IA) : onglets
  scaffoldés en placeholder. Nécessitent un ancrage de pose caméra pour être bien
  posés — l'estimation conjointe angles+pose caméra à partir d'une seule vue
  monoculaire est mathématiquement indéterminée sans lui (vérifié : erreur de
  reprojection quasi nulle atteignable avec >60° d'erreur angulaire).

### Ajouté

- `mycobot_gateway/mycobot_gateway/dream_validation_dashboard.py` — dashboard
  PyQt5, 3 onglets, en-tête avec logo ABMI. `ros2 run mycobot_gateway
  dream_validation_dashboard` (`--ros-args -p sim:=true` pour Gazebo).
- `mycobot_gateway/mycobot_gateway/assets/abmi_engineering_logo.jpeg`
- `training/dream/dream_angle_solver.py` — solveur angles+pose caméra conjoint à
  partir des seuls keypoints 2D (parké : confirmé mal posé sans ancrage, voir
  ci-dessus ; conservé pour référence future).

### Modifié

- `mycobot_gateway/setup.py` — entry point `dream_validation_dashboard`.

---

## [1.14.0] - 2026-07-08

### 🔬 Eye-to-hand : plateforme visual-servoing + cartographie détection vs récupération d'angles

Exploration de l'estimation d'angles par caméra unique (eye-to-hand) sur les 3
caméras réelles. Constat structurant : **détection et récupération d'angles sont
deux problèmes séparés.**

- **Détection par caméra** (real_3cam, `vgg_ultimate_v4_mix_ft_e30`) : svpro 98%,
  astra 95%, arducam 89%. → la caméra **astra est bonne** ; l'échec de l'astra
  fraîche (48%) était un problème de **placement** (hors-domaine), pas la caméra.
- **Récupération d'angles** : la caméra **mono** (arducam/svpro, sans depth) est
  mal conditionnée — angles à 7–24° même amorcés (ambiguïté de profondeur). Le
  **depth (astra)** la lève (self-test j1–j4 <2°). → pour une caméra unique, la
  profondeur est indispensable.
- **Self-calibration marker-free** (`self_calibrate_arducam.py`) : cale
  l'extrinsèque sur les keypoints DREAM. **Limite prouvée** : circulaire — elle
  absorbe le biais du détecteur (le squelette FK suit les détections décalées, pas
  le vrai bras). Un extrinsèque **indépendant** (4 ArUco) reste nécessaire.

### Ajouté

- `training/dream/visual_servoing_platform.py` — plateforme 6 fenêtres (une par
  joint) : angle réel (encodeur) vs estimé (caméra), modes live + rejeu.
- `training/dream/visual_servoing_dashboard.py` — dashboard avancé : vue caméra
  (squelette FK vérité + keypoints IA superposés) + 6 courbes + barre d'état.
- `training/dream/keypoint_accuracy_curve.py` — courbe PCK + erreur/keypoint
  (évaluation modèle sans calibration ; découpage par caméra via `offset:every`).
- `training/dream/check_dream_detection.py` — pré-check détection (zéro calibration).
- `training/dream/self_calibrate_arducam.py` — extrinsèque robot-as-target (marker-free, RANSAC).
- `training/dream/capture_arducam.py` — capture arducam RGB + encodeurs (réglage image 3-cam).
- `training/dream/ik_reach_point.py` — point → IK → angles (+ `--send`), validé réel (1° mécanique).
- `training/calibration/arducam_preview.py` — preview arducam + reprojection marqueurs/squelette robot.

### Modifié

- `training/dream/plot_angle_error_curve.py` — ajout du **mode 2D** (arducam mono,
  reprojection) en plus du mode 3D depth (astra).

---

## [1.13.0] - 2026-07-08

### 🎯 Pose estimation — sim-to-real comblé : réel 91.6% (fine-tune mixte terminé)

Le fine-tune mixte annoncé en 1.12.0 est **terminé et validé**.
`vgg_ultimate_v4_mix_ft_e30`
(`checkpoints_dream/vgg_ultimate_v4_mix_ft_e30/best_network.pth`) fait passer la
détection réelle de **≈27% (v4 synth-only) à 91,6%** sur `real_3cam_val_ndds`
(1500 frames jamais vues, 3 caméras), sans régression synthétique.

- **Détection par keypoint** (3 caméras, 1500 frames) : base/link1/link2 100%,
  link3 97,3%, link4 89,8%, link5 75,7%, link6 78,4% — overall **91,6%**.
- **Erreur médiane** : overall 2,91px (base ~1,6 · link3 7,2 · link4 15,9 ·
  link5 21,9 · link6 27,4). Les distaux restent le point faible relatif mais ont
  le plus progressé pendant le fine-tune (+27–33% de MSE).
- Entraînement : depuis le checkpoint v4 (`--pretrained`), mix 50K synth + réel
  oversamplé ×5, `scale_limit=0.3`, poids kp `[1,1,1,1,1.5,1.5,6.0]`, 30 epochs
  (best 27), val_loss 0,000942, 13,8 h.
- **Biais d'échantillonnage corrigé** : `evaluate_dream.py` en défaut 500 frames
  tombait à 100% sur arducam (step=3,0 en phase avec l'ordre des caméras) ;
  `--max-samples 1500` rétablit les 3 caméras et améliore les distaux.

Plan et méthodo : [`training/dream/FINETUNE_MIX_REAL3CAM_PLAN.md`](../training/dream/FINETUNE_MIX_REAL3CAM_PLAN.md).

### 🔭 Prochaine direction — pose estimation eye-to-hand + visual servoing

Cadre fixé pour la suite : caméra **fixe eye-to-hand** placée devant le bras
→ DREAM estime les keypoints → conversion en angles articulaires → **courbe
d'écart par joint** (angles estimés vs encodeurs réels) comme livrable
d'évaluation, puis **visual servoing** pour le pick-and-place. Maillons manquants
identifiés : calibration extrinsèque `T_base_camera` de la caméra fixe, puis
brique glue keypoints → angles (reprojection-min sur la FK existante
[`training/dream/mycobot_fk.py`](../training/dream/mycobot_fk.py) /
[`training/dream/mycobot_ik.py`](../training/dream/mycobot_ik.py)).

### Ajouté — outillage pose estimation eye-to-hand (astra RGB-D)

Pipeline complet keypoints → angles → courbe d'écart par joint, caméra astra
fixe devant le bras. Validé en simulation ; premier run réel en cours.

- [`training/dream/estimate_angles_from_keypoints.py`](../training/dream/estimate_angles_from_keypoints.py)
  — remonte des keypoints DREAM aux angles j1..j6. Mode 2D (reprojection, `cv2`)
  et **mode 3D** (depth → correspondance 3D). Self-tests : le 3D récupère
  j1–j4 à <2° **sans amorçage** (la profondeur supprime la fragilité mono) ;
  j5 faible, **j6 non observable** (keypoint sur l'axe de j6 — limite structurelle).
- [`training/calibration/oni_grabber_rgbd.cpp`](../training/calibration/oni_grabber_rgbd.cpp)
  — grabber OpenNI Astra : couleur + depth aligné couleur (D2C) + FOV (intrinsèques)
  vers `/dev/shm`. Extension du grabber couleur existant.
- [`training/calibration/calibrate_astra_extrinsic_shm.py`](../training/calibration/calibrate_astra_extrinsic_shm.py)
  — extrinsèque `T_base_camera` par recalage 3D (Kabsch) sur les marqueurs sol,
  sans ChArUco. Sort `astra_extrinsic.yaml` + `cam_astra.npz`.
- [`training/calibration/check_astra_markers.py`](../training/calibration/check_astra_markers.py),
  [`training/calibration/astra_preview.py`](../training/calibration/astra_preview.py)
  — aide au cadrage / preview live couleur+depth.
- [`training/dream/capture_astra_rgbd.py`](../training/dream/capture_astra_rgbd.py)
  — dataset RGB-D + encodeurs (mouvement calqué sur `capture_real_3cam` :
  home d'abord, `speed=25`, `settle=3s`). Réutilise le bridge TCP validé.
- [`training/dream/plot_angle_error_curve.py`](../training/dream/plot_angle_error_curve.py)
  — le livrable : DREAM → depth → angles vs encodeurs → courbe d'écart par joint.
- [`training/calibration/CALIBRATION_ASTRA_EXTRINSIC.md`](../training/calibration/CALIBRATION_ASTRA_EXTRINSIC.md)
  — procédure de calibration extrinsèque.

### Modifié — Documentation

- [`docs/ARCHITECTURE.md`](../docs/ARCHITECTURE.md) — historique modèles DREAM : `vgg_ultimate_v4_mix_ft_e30` finalisé (91,6% réel), date à jour.
- [`SESSION_RESUME.md`](../SESSION_RESUME.md) — entrée datée 8 juillet 2026 (état pose estimation + direction eye-to-hand).

---

## [1.12.0] - 2026-07-06

### 🎯 Pose estimation — record synthétique 99.4% (v4) + fine-tune mixte réel en cours

`vgg_ultimate_v4_e50` (50K synthétique, intrinsèques caméra corrigées) évalué à
**99.4% de détection** (2.61px erreur moyenne), dépassant le précédent record
v2 (97.7%). Voir [`training/dream/VGG_ULTIMATE_V4_50K.md`](../training/dream/VGG_ULTIMATE_V4_50K.md).

Le transfert sim-to-real reste bloqué à ≈27% sur `real_3cam_ndds` malgré ce
gain. Un fine-tune depuis `best_network.pth` sur un mix synthétique 50K + réel
3 caméras ×5 (`train_dream_ultimate_v4_mix.py`, `scale_limit=0.3`, 30 epochs)
est en cours pour combler l'écart — voir
[`training/dream/FINETUNE_MIX_REAL3CAM_PLAN.md`](../training/dream/FINETUNE_MIX_REAL3CAM_PLAN.md).

### Modifié — Documentation

- [`docs/ARCHITECTURE.md`](../docs/ARCHITECTURE.md) — historique modèles DREAM à jour (v4, mix fine-tune)
- [`training/README.md`](../training/README.md), [`training/dream/README.md`](../training/dream/README.md) — tableaux de résultats à jour

---

## [1.11.0] - 2026-04-23 (soir)

### 🎯 Pose estimation — diagnostic complet + option 1 épuisée

Session dédiée à débloquer la **pose estimation DREAM** (bloqué ~26 % détection réel depuis mi-avril). Verdict : le modèle `vgg_mixed_real_synth` résout proximal (link1/2/3 à 100 %) mais **n'a pas appris les distal keypoints** (link4/5/6 <= 36 % détection) sur les images réelles. Plus d'entraînement sur la même data (option 1) raffine le proximal mais **ne déplace pas la détection globale** (47.3 % → 47.3 %). Reste à faire : collecter plus de données réelles diverses (option 2).

### Ajouté — Tooling DREAM

- [`training/dream/evaluate_dream_relaxed.py`](../training/dream/evaluate_dream_relaxed.py) — wrapper de `evaluate_dream.py` qui monkey-patch les seuils de peak detection sans toucher la lib vendored `/tmp/DREAM/`. CLI : `--peak-thresh` (défaut 0.001 vs lib 0.01) et `--next-best-score` (défaut 0.05 vs lib 0.25).
- Backup du checkpoint pré-resume : `training/checkpoints_dream/vgg_mixed_real_synth/best_network.e25.{pth,yaml}`.

### Ajouté — Claude Code project structure

Structure complète pour que les sessions Claude aient le contexte projet dès le démarrage :

- [`CLAUDE.md`](../CLAUDE.md) à la racine — project overview, 3 envs Python, branch map, POC scope (digital twin · AI physics · VLA · pose estimation)
- [`.claude/settings.json`](../.claude/settings.json) — permissions partagées projet-wide
- [`.claude/rules/`](../.claude/rules/) (5) — `python-environments` · `ros2-conventions` · `real-robot-safety` · `git-branching` · `documentation`
- [`.claude/commands/`](../.claude/commands/) (5) — `launch-sim` · `launch-teleop` · `real-robot-preflight` · `train-dream` · `collect-synthetic`
- [`.claude/skills/`](../.claude/skills/) (6) — `teleop-troubleshoot` · `dream-workflow` · `gazebo-setup` · `real-robot-session` · `isaac-sim-integration` · `lerobot-dataset`
- [`.claude/agents/`](../.claude/agents/) (6) — `ros2-debugger` · `dream-trainer` · `teleop-tuner` · `urdf-surgeon` · `digital-twin-engineer` · `vla-integrator`
- [`.claude/hooks/validate-ros2-build.sh`](../.claude/hooks/validate-ros2-build.sh) — inactif par défaut (à câbler dans settings si désiré)

La skill `isaac-sim-integration` contient la roadmap 5-phases pour Isaac Sim (USD conversion → ROS2 bridge → synth data DREAM → Isaac Lab parallel envs → real-robot validation). **Aucune migration démarrée** — uniquement la planification. Gazebo reste sur `main`.

### Mesuré

| Checkpoint | Real det | link6 (det / med px) |
|------------|----------|-----------------------|
| `vgg_weighted_50k_e50` (synth-only) | 12.8 % | 5.6 % / 395 |
| `vgg_mixed_real_synth` e25 | 47.3 % | 1.2 % / 263 |
| **`vgg_mixed_real_synth` e50 (best)** | **47.3 %** | **3.0 % / 62** |

### Validé

- Hypothèse "confidence threshold trop strict" réfutée : relaxer `peak_thresh` de 0.01 à 0.001 débloque 48 % de détection sur link5 mais la médiane d'erreur explose de 2.97 px à 176 px → les peaks low-conf sont du bruit, pas des bonnes prédictions masquées.
- Hypothèse "unlearned" confirmée par visualisation (`/tmp/dream_eval_viz_mixed/montage_eval.png`) : bras entièrement visible + GT bien placée + prédictions distal hors bras.

### Prochaine session (24/04/2026)

1. Choisir (a) nouveau dataset `real_cam0_v2` ou (b) append in-place
2. Adapter `training/capture_real.py` pour biaiser vers poses bras-étendu
3. Capturer 5-10 K nouvelles poses (FK safety obligatoire)
4. Retrain mixte v2 (~30 K frames, 50 épochs)
5. Cible : détection ≥ 70 % tous keypoints, link6 médiane ≤ 10 px avant pick-and-place

---

## [2.2.0] - 2026-04-23

### 🎨 Dashboard ABMI + boutons dynamiques

Refonte complète de la GUI [`teleop/teleop_dashboard.py`](../teleop/teleop_dashboard.py) sur la charte **ABMI** (navy `#1B1A3E` + pink `#E6417A`) avec logo intégré. Trois onglets, KPI cards, caméra opérateur inline, comparaison sim ↔ réel côte à côte et ActionButton dynamiques avec feedback visuel.

### Ajouté

- **3-tab UI** (ttkbootstrap Notebook) — 🏠 Home · 📊 Analytics · 🎛️ Tuning
- **5 KPI cards** (`KpiCard` widget) sur Home : Execution mode, Command rate, SIM avg RMS, REAL avg RMS, Signal health — chacune avec accent coloré contextuel (vert / jaune / rose selon les seuils)
- **Badge de mode auto** en haut à droite : SIM / REAL / BOTH / OFFLINE, détecté par fraîcheur des topics `/joint_states` et `/from_robot` (fenêtre 2 s)
- **Caméra opérateur intégrée** — `mycobot_teleop.py` advertize `/teleop/camera/image` (JPEG compressée à 320 px de large, q=60, ~10 Hz via monkey-patch de `cap.read`) · le dashboard l'affiche dans le panneau gauche de Home. Plus besoin de fenêtre OpenCV séparée.
- **Comparaison SIM ↔ REAL** : bar chart RMS par joint (bleu/rose) sur Home · paires miroir de plots joint angles & tracking error sur Analytics
- **Hand position en cm** (au lieu de m) pour lecture directe
- **ActionButton dynamiques** (5 sur Home, 1 sur Tuning, 3 presets) :
  - Tooltip au survol (Toplevel navy) décrivant l'action + ses effets
  - Feedback visuel : label swap `⟳ …` → `✓` / `✗` pendant ~1,4 s
  - Désactivation in-flight pour absorber les double clicks
  - Toast horodaté dans la status bar Home (`✓ Send robot home · 14:23:05`)
- **Presets de gains** — 🐢 Safe start (0.6/0.6/0.6/0.3) · ⚙️ Nominal (1.2/1.2/1.6/0.25) · ⚡ Reactive (1.6/1.6/2.0/0.15). Le preset actif reste highlighted en SUCCESS solide.
- **Polling passif** de `get_angles` (0.3 s) pour remonter les angles réels quand `/joint_states` n'est pas là
- [`teleop/assets/abmi_logo.png`](../teleop/assets/) — logo chargé automatiquement

### Modifié

- [`docs/TELEOP_DASHBOARD.md`](TELEOP_DASHBOARD.md) — **réécrit** pour l'UI 3-tabs, sections par onglet, tableau topics in/out, troubleshooting mis à jour
- [`docs/TELEOPERATION.md`](TELEOPERATION.md) — section dashboard regénérée + topic `/teleop/camera/image` ajouté dans le listing des publications de Stage 4
- [`README.md`](../README.md) — ligne dashboard dans le tableau outils, commentaire T4 rafraîchi

### Non changé

- Protocole de téléopération · filtres (Kalman + EMA + slew 1°/frame) · mapping main → joints · bridge_tour / `trajectory_to_robot_bridge`
- Gains validés le 22/04/2026 sur robot physique : 1.2 / 1.2 / 1.6 / 0.25 (le preset ⚙️ Nominal)

---

## [2.1.0] - 2026-04-22 (soir)

### ✅ Premier test sur robot physique validé

Session de validation end-to-end sur le **MyCobot 320 Pi physique** (IP `10.10.0.221`). Le pipeline complet Astra → Wilor → rosbridge → JTC topic → trajectory_to_robot_bridge → bridge_tour → Pi → pymycobot est fonctionnel avec une latence main→bras de ~150–250 ms, imperceptible visuellement. Mouvements coordonnés, pas d'oscillation ni saturation sur les gains initiaux 0.6/0.6/0.6.

### Ajouté — Documentation

- `docs/TELEOP_ARCHITECTURE_VIZ.md` — visuel détaillé du pipeline complet (9 étapes, types, conversions d'unités, latences mesurées, exemples chiffrés). Répond au besoin de comprendre exactement comment la détection se traduit en mouvement.
- `docs/REAL_ROBOT_TEST_PROCEDURE.md` étendu :
  - Protocole de calibration sécurisé validé (gains 0.6/0.6/0.6, tfs 0.3, speed 25, montée progressive)
  - Conditions de Ctrl+C immédiat
  - Tableau de résultats du premier test
  - Points à creuser pour les prochaines sessions

### Ajouté — Infrastructure test réel

- `scripts/real_robot_preflight.sh` — check pré-vol 5 étapes avec exit codes distincts.
- `mycobot_gateway/gripper_to_robot_bridge.py` — bridge prêt pour gripper physique (non câblé : robot actuel sans pince).
- Flag `--no-gripper` dans `mycobot_teleop.py`.

### Corrigé

- IP par défaut Pi dans les docs : `10.10.0.225` → `10.10.0.223`.
- `bridge_tour` accepte `pi_ip` et `pi_port` comme paramètres ROS2.

### Validé — Session 22/04/2026 (soir)

| Check | Résultat |
|-------|----------|
| ping Pi + TCP 5005 | ✅ |
| bridge_tour TCP connect + ping/pong | ✅ |
| `get_angles`, `home`, `send_angles [45,0,…]` | ✅ |
| Téléop main complète (Wilor → bras physique) | ✅ |

### Points non bloquants

- `bridge_tour` receive_loop : ne log pas `📥 Reçu de Pi` (non bloquant pour téléop)
- Axe J6 (doorknob) : mapping `j6 = yaw * roll_gain` à valider visuellement sur réel

---

## [2.0.0] - 2026-04-22

Version majeure : **téléopération par la main** opérationnelle en simulation Gazebo, avec dashboard live de tuning + rapport de performance Excel.

### Ajouté — Téléopération

**Pipeline de téléopération** (hand-tracking → MyCobot) :
- `teleop/mycobot_teleop.py` — script principal Wilor + Orbbec Astra → joints via rosbridge. Arguments CLI exhaustifs (gains par axe, inversion, fps, time_from_start, camera backend).
- `teleop/orbbec_capture.py` — wrapper shared-memory pour Astra S via `oni_grabber` binaire OpenNI2. Auto-spawn, watchdog, survie aux Ctrl+C (start_new_session).
- `mycobot_gateway/trajectory_to_robot_bridge.py` — nœud ROS2 qui convertit `JointTrajectory` (rad) → JSON `send_angles` (deg) pour le `bridge_tour` / robot réel, avec rate-limit 15 Hz et deadband 1°.
- `mycobot_gateway/launch/mycobot_teleop.launch.py` — orchestration complète (Gazebo + controllers + rosbridge + bridge_tour + trajectory bridge) avec target `sim`/`real`/`both`.

**Dashboard de tuning** (`teleop/teleop_dashboard.py`) :
- GUI ttkbootstrap theme "darkly" connectée à rosbridge.
- 4 sliders live : x/y/z gain + `time_from_start`, appliqués via `/teleop/gains`.
- Bouton **⟲ Recalibrate hand origin** qui reset l'initial_pose de Wilor via `/teleop/recalibrate`.
- 3 plots matplotlib temps-réel (fenêtre 10 s) : Wilor XYZ / commandé vs actual / tracking error par joint avec ligne 5° cible.
- Tableau stats par joint : RMS, max, jitter avec flags colorés `✓ OK / △ JITTERY / ⚠ UNSTABLE`.
- Indicateur connexion rosbridge + compteurs de messages par topic.

**Performance analyzer** (`teleop/performance_analyzer.py`) :
- Générateur de rapport Excel multi-onglets avec verdict `READY / CAUTIOUS / NOT READY`.
- Mode `--guided` : protocole scripté 7 phases (idle, up/down, left/right, forward/back, combined, gripper, rest) sur 64 s.
- Mode `--duration N` : enregistrement passif libre.
- Onglets : Summary (verdict coloré), Per-joint tracking (+ bar chart), Scenarios, Signal health, raw_hand/raw_cmd/raw_actual.

**Pipeline de filtres porté du R5A / LeRobot** :
- Stage 1 (dans `HandTracker`) : jump clamp 2 m/s + Kalman XYZ (dt=1/30, q=r=5e-3).
- Stage 2 (sur les joints) : EMA α=0.20 + slew rate limiter 1 °/frame (= 30 °/s @ 30 Hz).
- Stage 3 (sur le gripper) : deadband 3° + EMA α=0.25 + slew 4 °/frame.

**Mapping main → joints** :
- Position delta-based (rel=(0,0,0) → tous joints 0°) :
  - Y → J1 base yaw, Z → J2 shoulder, X → J3 elbow
- Orientation (Euler ZYX) :
  - pitch → J4 + J5 (split 50/50, EE pointe comme la paume)
  - yaw → J6 (doorknob twist autour axe optique)
- `BASE_SCALE_DEG_PER_M` = 150 (tuné itérativement depuis 600 → 300 → 200 → 150).

**Gripper en Gazebo** :
- 4 joints revolute commandés explicitement (plus de `<mimic>`) — contournement du manque de support DART pour les contraintes mimic + limitations URDF pour les 4-bar linkages.
- `gripper_position_controller` (`JointGroupPositionController`) pilote les 4 joints simultanément via un `Float64MultiArray` avec signes mirrors `[servo_left, servo_right, tip_left, tip_right] = [-0.7, +0.7, +0.7, -0.7]` fermé.

**Robustesse** :
- Wrapper `try/except` sur `pose_computer.compute_relative_pose` pour éviter que le capture thread daemon meure silencieusement sur une exception Wilor.
- Watchdog `oni_grabber` qui respawn automatiquement si les frames s'arrêtent > 2 s.
- Bridge `/clock` Gazebo → ROS2 ajouté au launch (sans lui, le JTC dérive avec des deltas temporels erronés).
- Auto-resume du tracker au démarrage (sinon `tracking_paused=True` par défaut).

### Ajouté — Documentation téléopération

- `docs/TELEOPERATION.md` — pipeline complet, workflow 5 terminaux, filtres, mapping, limitations, historique des 25 commits.
- `docs/TELEOP_DASHBOARD.md` — manuel utilisateur du dashboard avec ascii art de l'UI, guide de tuning step-by-step, troubleshooting.
- `docs/TELEOP_TUNING.md` — référence exhaustive des paramètres (constantes module, flags CLI, controller.yaml) + troubleshooting complet.

### Ajouté — URDF / simulation

- Bloc `<ros2_control>` dans `mycobot_pro_320_pi_gazebo.urdf` avec 10 joints commandés (6 arm + 4 gripper).
- Plugin `gz_ros2_control/GazeboSimROS2ControlPlugin` lié au `controller.yaml`.
- `mycobot_description/config/controller.yaml` — 3 controllers : `joint_state_broadcaster`, `mycobot_controller` (JTC), `gripper_position_controller` avec PID par joint.

### Corrigé

- Bug `rclpy` dans env conda (Python 3.10 vs ROS2 Jazzy 3.12) → `--use-rosbridge` obligatoire documenté.
- IP Pi exposée en paramètre ROS de `bridge_tour` (`pi_ip`, `pi_port`) au lieu de hardcoded 10.10.0.218.

### Limitations connues

- Cinématique 4-barres du `pro_adaptive_gripper` non reproduisible en Gazebo (DART ne supporte pas les contraintes mimic, URDF pas de closed-loops). Contournement : commande explicite des 4 joints. Impact cosmétique uniquement — sur le robot réel, pymycobot gère le mécanisme mécaniquement.
- Axe J6 (doorknob) : mapping actuel `j6 = yaw`, à valider visuellement via le log RPY ajouté dans T3.

---

## [1.10.0] - 2026-04-23

### Ajouté
- **Pick-and-place multi-objets par couleur** (branche `feature/pick-and-place-sorting`)
  - Monde Gazebo `worlds/pick_and_place_sorting.sdf` :
    table 1.0×0.6 m, 4 objets à trier (cube rouge, cube bleu, cylindre vert, boîte
    jaune) côté +X, 4 bacs de réception colorés à parois côté −X, tous dans
    l'enveloppe d'atteinte ~0.32 m du MyCobot 320.
  - `color_object_detector` : segmentation HSV sur la caméra top-down, rétro-projection
    pinhole vers le repère robot, publie `/sorting/detections` (`color,x,y;…`),
    `/sorting/detector_status`, `/sorting/debug_image`.
  - `sorting_orchestrator` : machine à états qui boucle sur les détections,
    plan IK par objet, séquence approche/grasp/lift/place/retreat. Le « grasp »
    est émulé via le service Gazebo `/world/<world>/set_pose` (téléport du modèle
    sur l'EE pendant le portage, dépose dans le bac à la couleur correspondante).
  - Launch `pick_and_place_sorting.launch.py` : Gazebo + spawn + bridges (joints
    + 4 caméras) + détecteur (T+6 s) + orchestrateur (T+10 s).
- **Visuels caméra réalistes** dans le URDF Gazebo : les 4 caméras embarquent
  désormais un corps gris foncé + objectif noir cylindrique + LED rouge,
  au lieu de cubes 3 cm colorés qui ressemblaient aux objets à trier.

### Corrigé
- **`color_object_detector` : paramètre boolisé par YAML 1.1** —
  les valeurs `'y'` / `'x'` étaient coercées en `True` par rclpy (YAML 1.1
  truthy). Renommé en `image_u_to_world_axis_name` / `image_v_to_world_axis_name`
  avec valeurs `'world_y'` / `'world_x'`.
- **Axe Y de la caméra top-down inversé** : ajout de `flip_u: True` dans le
  launch ; les positions détectées correspondent maintenant aux positions SDF
  (vérifié rouge à −0.12, bleu à +0.12).

---

## [1.9.0] - 2026-04-21

### Documentation
- Réécriture complète de `SESSION_RESUME.md` (nettoyage du contenu fusionné corrompu)
- Mise à jour de tous les fichiers README et docs pour refléter l'état actuel

---

## [1.8.0] - 2026-04-15

### Ajouté
- **Gripper adaptatif** : Intégration du `pro_adaptive_gripper` d'Elephant Robotics dans le URDF Gazebo
  - 7 maillages DAE (gripper_base, left1/2/3, right1/2/3)
  - Joints fixés (pas de support `mimic` dans Gazebo Harmonic)
  - Mesh `link6_2022.dae` pour compatibilité avec les maillages du gripper
- **Vérification physique** : Limites articulaires corrigées selon l'URDF officiel elephantrobotics
  - J2 : ±159.9° → ±134.6°
  - J3, J4 : ±159.9° → ±145.0°
- **Anti-collision** : Rejet par cinématique directe dans le collecteur synthétique
  - Table clearance (z < 2cm)
  - Base column proximity check
  - Elbow height validation
  - Extreme fold-back rejection (|j2+j3| > 3.8 rad)
- **Pipeline d'automatisation** : `scripts/train_pipeline.sh` (merge → NDDS → training)
- **Script de merge** : `training/dream/merge_and_convert.py` pour combiner datasets réels+synthétiques
- **Monitoring** : `scripts/monitor_collection.sh` pour suivre la collecte en temps réel
- **Monde Gazebo v2** : `worlds/randomized_v2.sdf` — 6 lumières, 12 objets clutter, 3 murs
- **Collecteur v2** : `synthetic_data_collector_v2.py` — anti-collision FK, domain randomization avancée
- **Launch v3** : `synthetic_data_v3.launch.py` — collecte avec monde randomized_v2

### Corrigé
- **Stale install** : Suppression du répertoire `install/` orphelin dans `src/mycobot_R6A/`
- **Shebang Python** : `#!/usr/bin/python3` pour éviter conda Python 3.13
- **GZ_SIM_RESOURCE_PATH** : Ajout dans les launch files pour résoudre les meshes

---

## [1.7.0] - 2026-04-16

### Ajouté
- **DREAM fine-tuning expérimental** : `training/dream/finetune_real.py`
  - v1 (σ=4) : 0% détection — bug sigma mismatch avec DREAM natif
  - v2 (σ=2) : 0% détection — belief maps effondrées (MSE sur grille quasi-vide)
- **Dataset mixte** : `/tmp/dream_data/mixed_real_synth/` (18K frames — 10K réel ×5 + 8K synth)
- **Training mixte natif** : DREAM `train_network.py` sur dataset 18K, epoch 1 val=0.000474
- **Documentation ARCHITECTURE.md** : réécriture complète

---

## [1.6.0] - 2026-04-15

### Ajouté
- **DREAM VGG 50K** : training sur 50K frames synthétiques
  - Synthétique : 98.3% détection, 3.15px médiane
  - Réel (sim-to-real) : 13.2% détection, 172px médiane
- **Pick-and-place Gazebo** : `pick_and_place_node.py` + `pick_and_place.launch.py` (Step C)
- **DREAM inference node** : `dream_inference_node.py` — nœud ROS2 temps réel (YAML, venv, API)
- **Analyse adéquation** : conversion px→mm→degrés documentée

---

## [1.5.0] - 2026-04-03

### Ajouté
- **Module DREAM** : `training/dream/` — keypoint-based pose estimation (NVlabs DREAM 1.3.0)
  - `mycobot_fk.py` — Forward Kinematics + projection caméra (7 keypoints, paramètres DH)
  - `convert_to_ndds.py` — conversion datasets → format NDDS
  - `train_dream.py` / `train_dream_augmented.py` — wrappers d'entraînement
  - `evaluate_dream.py` — évaluation par keypoint (filtre sentinel -999.99)
  - `infer_dream.py` — inférence single-image + PnP
  - `visualize_ndds.py` — vérification visuelle des annotations
  - `manip_configs/mycobot320.yaml` — configuration 7 keypoints
- **Résultats training DREAM** :
  - ResNet-H : tué à epoch 10 (BatchNorm instable)
  - VGG-base : 25 époques, val=0.000438, 96.1% détection synth, 3.1px médiane
  - VGG-aug : 25 époques, val=0.000667, 96.6% détection synth, 3.1px médiane
- **Sim-to-real baseline** : ~26% détection réel (vs 97% synth) — domain gap identifié

### Corrigé
- Fix `evaluate_dream.py` : filtre coords < -900 (sentinel DREAM -999.99)

---

## [1.4.0] - 2026-04-02

### Ajouté
- **Capture réelle** : `training/capture_real.py`
  - 2000 poses × 2 caméras Pi = 4000 images réelles
  - FK safety : protection table, câbles, limites articulaires
  - 0 collisions sur 2000 poses
- **Diagnostic signal visuel** :
  - Corrélation pose↔pixel = 0.004 (quasi-nulle)
  - Robot = 15.4% (cam0) / 5.7% (cam3) des pixels
  - Dérive d'éclairage pendant capture (luminosité 140→82→143)
- **Dataset réel** : `datasets/real_dataset/` (4000 images, via Git LFS)

### Corrigé
- Gestion robuste des erreurs lors de la sauvegarde d'images

---

## [1.3.0] - 2026-04-01

### Ajouté
- **Camera server Pi** : `scripts/pi_camera_server.py` — serveur TCP pour 2 caméras Arducam USB
  - TCP:5006, streaming JPEG, nommage cam0/cam3
- **Prévisualisation caméras** : `training/preview_cameras.py`

---

## [1.2.0] - 2026-03-31

### Ajouté
- **Pipeline training v2** : `training/train.py` — multi-view ResNet50
  - Résultat synthétique : **12.97° MAE** (4 caméras)
  - ResNet18 single-view : 22.6° MAE
  - ResNet50 single-view : 16.5° MAE
- **Dataset synthétique** : `datasets/synthetic_dataset/` (5000 poses × 4 vues = 20K images, Git LFS)
- **Vérification dataset** : script montage + histogrammes + stats
- **Domain Randomization** : éclairage variable, materials aléatoires
- **Monde Gazebo v1** : `worlds/randomized.sdf` avec table et fond simple
- **PerImageNormalize** : normalisation par image pour les données réelles
- `training/dataset.py` — MyCobotDataset, MyCobotMultiViewDataset

---

## [1.1.0] - 2026-03-31

### Ajouté
- **Simulation Gazebo Harmonic** : intégration `ros_gz_sim` + bridge
  - URDF Gazebo compatible (`mycobot_pro_320_pi_gazebo.urdf`) avec inertials et plugins
  - 4 caméras simulées (front, right, left, top) à 640×480
  - Joint controllers Gazebo (`gz-sim-joint-position-controller`)
- **Collecteur données synthétiques v1** : `synthetic_data_collector.py`
  - Poses aléatoires → capture image + angles → format labels.csv
  - Collecte 5000 poses (20K images total, 4 caméras)
- **Launch files Gazebo** : `gazebo_sim.launch.py`, `synthetic_data.launch.py`

---

## [1.0.0] - 2026-03-26

### Ajouté
- **Architecture distribuée** Tour ↔ Raspberry Pi via TCP/IP
- **Bridge TCP** : `bridge_tour.py` (ROS2) ↔ `bridge_pi_simple.py` (Pi standalone)
  - Communication JSON bidirectionnelle sur TCP:5005
  - Auto-reconnexion Pi
- **Modes de contrôle** :
  - `simple_gui.py` : Interface graphique Tkinter (angles, coords, gripper, LED)
  - `slider_control.py` : Sliders RViz + Joint State Publisher
  - `teleop_keyboard.py` : Contrôle clavier WASD+ZX
  - `robot_commander.py` : CLI interactif
  - `joint_sync.py` : Synchronisation robot réel → RViz
- **URDF MyCobot 320 Pi** : modèle RViz + config RViz prête à l'emploi
- **Réseau** : PC Tour (10.10.0.115) ↔ Raspberry Pi (10.10.0.225)
- **Documentation** : README, SESSION_RESUME, guides de déploiement

### Validé
- Communication TCP ping/pong
- Contrôle LED (RGB)
- Lecture/écriture angles joints
- Mouvements go_home, go_zero
- Gripper open/close
- Synchronisation RViz temps réel

---

## [0.1.0] - 2026-03-26

### Initial
- Initialisation du dépôt
- Structure packages ROS2 (`mycobot_gateway`, `mycobot_description`)
- Configuration réseau Tour (10.10.0.115) / Pi (10.10.0.225)
