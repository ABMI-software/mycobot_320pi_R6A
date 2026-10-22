# Capture 3 caméras — fiche d'instructions

Capture d'images réelles synchronisées du MyCobot 320 depuis 3 caméras (ArduCam + SVPRO + Astra) pour le pipeline DREAM. Script : [`capture_real_3cam.py`](capture_real_3cam.py), lanceur : [`capture_session.sh`](capture_session.sh).

## Lancement

```bash
# Le plus simple (output horodaté unique, by-id, IP Pi, exposition ArduCam déjà réglés)
bash training/capture_session.sh

# Commande directe (preview + 5 poses)
python3 training/capture_real_3cam.py --preview --num-samples 5 \
  --arducam-index /dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_8mp_SN0001-video-index0 \
  --svpro-index   /dev/v4l/by-id/usb-5MP_USB_Camera_5MP_USB_Camera_01.00.00-video-index0
```

Preview : **ENTER** = démarrer la capture (le robot bouge), **q / ESC** = quitter sans toucher le robot.

## Caméras — toujours les chemins by-id, jamais les index entiers

Une caméra USB expose plusieurs `/dev/videoN` (flux + metadata) et les numéros se réassignent au rebranchement. **Toujours** passer le nœud capture stable `…-video-index0` :

| Caméra | Source | Chemin stable |
|---|---|---|
| ArduCam | V4L2 | `/dev/v4l/by-id/usb-Arducam_…_Arducam_8mp_SN0001-video-index0` |
| SVPRO (5MP) | V4L2 | `/dev/v4l/by-id/usb-5MP_USB_Camera_5MP_USB_Camera_01.00.00-video-index0` |
| Astra | OpenNI / oni_grabber | (pas de /dev/video — mémoire partagée) |

Lister les caméras branchées :
```bash
ls /dev/v4l/by-id/        # repérer les lignes *-video-index0
lsusb | grep -iE "arducam|5mp|orbbec"
```

Un **garde-fou** au démarrage refuse de lancer si deux index pointent sur le même capteur physique (cause des images dupliquées).

## Exposition ArduCam (réglée et calée sur session4)

L'ArduCam surexpose (bras blanc cramé) en auto. Points clés découverts :
- En **auto-exposition**, l'AE annule tout réglage de luminosité → il faut le **manuel** (`auto_exposure=1`).
- **Piloter par l'exposition, PAS par un offset `brightness` négatif.** Un brightness négatif soustrait un offset → **écrase les noirs** (ombres p5→2, contraste dur). Baisser l'`exposure_time_absolute` (avec `brightness=0`) assombrit proportionnellement → ombres douces, rendu « session4 ».
- Figer l'exposition rend le manuel reproductible (un `exposure_time_absolute` résiduel trop bas causait des frames noires).

Réglage gravé (reproduit le *rendu* de `session4` — moyenne≈87, ombres p5≈22, contraste≈61) :
```
auto_exposure=1  +  exposure_time_absolute=75  +  gain=0  +  brightness=0
```
Ajuster sans éditer le code : `--arducam-exposure 85` (plus clair) / `60` (plus sombre).

> ⚠️ La bonne valeur **dépend de la lumière ambiante** (session4 a été pris sous une autre lumière). Re-mesurer/re-régler si la pièce change d'éclairage. Comparer **moyenne + p5 (ombres) + contraste**, pas que la moyenne.

## Focus SVPRO (verrouillé)

Le SVPRO a un **vrai focus variable** (contrairement à l'ArduCam, focus fixe). Mesuré (netteté = variance Laplacien, zone poignet/LED) :
- `focus_absolute=1..90` → net (plateau), **pic sur le poignet/LED à ~90** ✅
- `focus_absolute=128..930` → **1-15** (catastrophiquement flou) ❌
- `focus_absolute=970..1023` → net aussi

Réglage gravé : autofocus OFF + `focus_absolute=90` (default `--svpro-focus 90`, le plus piqué sur la LED). L'autofocus est désactivé car il « pompe » entre les poses (netteté variable d'une image à l'autre). **Ne jamais laisser le focus dans 128-930.** Si jamais 90 paraît à la limite, `64` est un repli net plus loin du « cliff ».

Le script pinne aussi **`sharpness=0` + `contrast=1`** (= rendu session9, défauts caméra). L'accentuation in-camera (`sharpness`/`contrast` plus hauts) rend l'image artificielle (« trop ») — on garde le naturel. La mollesse résiduelle du SVPRO est **inhérente** à l'optique, pas un défaut de réglage.

> **Juger sur les images sauvegardées, pas la preview à l'œil.** La preview induit en erreur ; la vérité c'est la mesure :
> ```bash
> python3 -c "import numpy as np,glob; from PIL import Image; \
>   print(np.mean([np.asarray(Image.open(f)).mean() for f in glob.glob('<dossier>/images/arducam/*.png')]))"
> ```
> Cible ≈ 87.

## Robot (Pi)

- IP par défaut : **`10.10.0.224`** (port 5005). Le réseau est instable, l'IP du Pi change parfois.
- Vérifier que le bridge écoute avant de capturer :
  ```bash
  python3 -c "import socket;s=socket.socket();s.settimeout(2);s.connect(('10.10.0.224',5005));print('bridge OK');s.close()"
  ```
- Le bridge `bridge_pi_simple.py` doit tourner sur le Pi. ⚠️ Bras dégagé, main près de l'arrêt d'urgence avant ENTER.

## Dépannage

| Symptôme | Cause / fix |
|---|---|
| 2 images identiques | Deux index sur le même capteur → utiliser les chemins by-id |
| `svpro failed to open` (video3) | `video3` = nœud metadata ; le flux est `video2` → by-id `-video-index0` |
| ArduCam surexposée | exposition trop haute → baisser `--arducam-exposure` (manuel, brightness=0) |
| ArduCam aux noirs écrasés / dure | un offset `brightness` négatif a été utilisé → piloter par `--arducam-exposure` à la place (brightness reste 0) |
| Image noire | exposition manuelle trop basse → remonter `--arducam-exposure` (le script la fixe explicitement) |
| SVPRO flou | focus dérivé dans 128-750 ou autofocus actif → le script verrouille `focus_absolute=1` (autofocus off). Re-régler via `--svpro-focus` |
| Vue d'une caméra ≠ attendue | les numéros `/dev/videoN` ont swappé (vu : SVPRO video2→video0 du jour au lendemain) → **toujours** les by-id, jamais les numéros bruts en manuel |
| `Collecting 0 samples` | Le dossier `--output` existe déjà (reprise) → utiliser un dossier neuf (capture_session.sh en crée un horodaté) |
| `Robot bridge: timed out` | bridge pas en écoute sur `.221:5005` → lancer `bridge_pi_simple.py` sur le Pi |
| `No route to host` | mauvaise IP / Pi hors réseau → vérifier l'IP réelle (`hostname -I` sur le Pi) |

## Calibration (intrinsèques)

Les `_camera_settings.json` (NDDS) sont écrits par caméra à partir de `calibration/<stem>.npz` : ArduCam=`cam_0`, SVPRO=`cam_2`. L'Astra est **non calibrée** (absente de `CALIBRATION_MAP`) — pour la calibrer :
```bash
python3 training/calibration/calibrate_camera.py --source astra --name astra_rgb
```
puis ajouter `"astra": "astra_rgb"` dans `CALIBRATION_MAP`.
