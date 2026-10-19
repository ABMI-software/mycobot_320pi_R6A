#!/usr/bin/env python3
"""Géométrie ArUco *live* pour la table de pick-and-place.

Ce module est volontairement sans ROS et sans accès matériel. Il transforme une
frame BGR déjà acquise en une pose extrinsèque validée, puis permet
l'intersection d'un rayon image avec un plan ``z = projection_z`` du repère
``base_link``.

Conventions
-----------
``T_cam_world`` transforme un point monde/base vers la caméra ;
``T_world_cam`` est son inverse et donne la pose caméra dans le monde.
Les centres connus des marqueurs sont exprimés en mètres.

La pose est recalculée depuis les quatre marqueurs de table à chaque frame. Une
solution n'est retournée que si elle est physique : quatre tags requis par
défaut, points devant la caméra, caméra au-dessus du plan et RMS de
reprojection sous le seuil demandé.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import cv2
import numpy as np
import yaml


REPO = Path(__file__).resolve().parents[1]
DEFAULT_CALIB_NPZ = REPO / "training/calibration/cam_0.npz"
DEFAULT_META_JSON = REPO / "training/calibration/cam_0.meta.json"
DEFAULT_WORKSPACE_YAML = REPO / "training/calibration/workspace_markers.yaml"
DEFAULT_MARKER_IDS = (19, 23, 25, 26)
DEFAULT_DICTIONARY = "DICT_4X4_1000"


class GeometryError(RuntimeError):
    """Erreur de calibration, de pose ou de projection."""


class CalibrationError(GeometryError):
    """Les fichiers de calibration ne sont pas exploitables."""


class PoseEstimationError(GeometryError):
    """Aucune pose ArUco suffisamment fiable et physique n'a été trouvée."""


class ProjectionError(GeometryError):
    """Le rayon image ne peut pas être projeté sur le plan demandé."""


@dataclass(frozen=True)
class CameraCalibration:
    """Intrinsèques OpenCV et métadonnées de la caméra."""

    K: np.ndarray
    dist: np.ndarray
    meta: Mapping[str, Any]
    resolution: tuple[int, int] | None
    dictionary_name: str = DEFAULT_DICTIONARY


@dataclass(frozen=True)
class Workspace:
    """Centres 3D connus des marqueurs dans le repère monde/base."""

    markers: Mapping[int, np.ndarray]
    frame_id: str = "base_link"
    marker_size_m: float | None = None


@dataclass(frozen=True)
class GeometryBundle:
    camera: CameraCalibration
    workspace: Workspace


@dataclass(frozen=True)
class PoseEstimate:
    """Résultat validé du PnP planaire."""

    T_cam_world: np.ndarray
    T_world_cam: np.ndarray
    rvec: np.ndarray
    tvec: np.ndarray
    rms_reprojection_px: float
    per_marker_reprojection_px: Mapping[int, float]
    marker_ids: tuple[int, ...]
    camera_position_world_m: np.ndarray
    depths_camera_m: np.ndarray


def _first_npz_array(data: Any, names: Sequence[str]) -> np.ndarray | None:
    for name in names:
        if name in data.files:
            return np.asarray(data[name], dtype=np.float64)
    return None


def _validate_intrinsics(K: np.ndarray, dist: np.ndarray) -> None:
    if K.shape != (3, 3) or not np.all(np.isfinite(K)):
        raise CalibrationError(f"matrice K invalide : forme {K.shape}, 3x3 attendue")
    if K[0, 0] <= 0.0 or K[1, 1] <= 0.0 or abs(K[2, 2]) < 1e-12:
        raise CalibrationError("matrice K invalide : focales et K[2,2] doivent être positives")
    if dist.size not in (4, 5, 8, 12, 14) or not np.all(np.isfinite(dist)):
        raise CalibrationError(
            f"coefficients de distorsion invalides : {dist.size} valeurs"
        )


def load_camera_calibration(
    calib_npz: str | Path = DEFAULT_CALIB_NPZ,
    meta_json: str | Path = DEFAULT_META_JSON,
) -> CameraCalibration:
    """Charge ``K``/``dist`` depuis NPZ et les métadonnées JSON associées."""

    calib_path = Path(calib_npz)
    meta_path = Path(meta_json)
    try:
        with np.load(calib_path, allow_pickle=False) as data:
            K = _first_npz_array(data, ("mtx", "K", "camera_matrix"))
            dist = _first_npz_array(data, ("dist", "D", "dist_coeffs"))
    except (OSError, ValueError) as exc:
        raise CalibrationError(f"impossible de charger {calib_path}: {exc}") from exc
    if K is None or dist is None:
        raise CalibrationError(
            f"{calib_path} doit contenir K/mtx et dist/D"
        )
    K = np.asarray(K, dtype=np.float64).reshape(3, 3)
    dist = np.asarray(dist, dtype=np.float64).reshape(-1)
    _validate_intrinsics(K, dist)

    try:
        with meta_path.open("r", encoding="utf-8") as stream:
            meta = json.load(stream)
    except (OSError, ValueError) as exc:
        raise CalibrationError(f"impossible de charger {meta_path}: {exc}") from exc
    if not isinstance(meta, dict):
        raise CalibrationError(f"{meta_path}: objet JSON attendu")

    raw_resolution = meta.get("resolution")
    resolution: tuple[int, int] | None = None
    if raw_resolution is not None:
        if (
            not isinstance(raw_resolution, (list, tuple))
            or len(raw_resolution) != 2
            or any(int(v) <= 0 for v in raw_resolution)
        ):
            raise CalibrationError(f"résolution invalide dans {meta_path}")
        resolution = int(raw_resolution[0]), int(raw_resolution[1])

    dictionary_name = str(meta.get("dictionary", DEFAULT_DICTIONARY))
    _aruco_dictionary(dictionary_name)  # validation immédiate du nom
    return CameraCalibration(
        K=K,
        dist=dist,
        meta=meta,
        resolution=resolution,
        dictionary_name=dictionary_name,
    )


def load_workspace(
    workspace_yaml: str | Path = DEFAULT_WORKSPACE_YAML,
    *,
    required_ids: Sequence[int] = DEFAULT_MARKER_IDS,
) -> Workspace:
    """Charge les centres des ArUco connus dans le repère base."""

    path = Path(workspace_yaml)
    try:
        with path.open("r", encoding="utf-8") as stream:
            spec = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as exc:
        raise CalibrationError(f"impossible de charger {path}: {exc}") from exc
    if not isinstance(spec, dict) or not isinstance(spec.get("markers"), dict):
        raise CalibrationError(f"{path}: clé 'markers' absente ou invalide")

    markers: dict[int, np.ndarray] = {}
    for raw_id, raw_xyz in spec["markers"].items():
        try:
            marker_id = int(raw_id)
            xyz = np.asarray(raw_xyz, dtype=np.float64).reshape(3)
        except (TypeError, ValueError) as exc:
            raise CalibrationError(f"marqueur {raw_id!r} invalide dans {path}") from exc
        if not np.all(np.isfinite(xyz)):
            raise CalibrationError(f"marqueur {marker_id}: coordonnées non finies")
        markers[marker_id] = xyz

    missing = sorted(set(map(int, required_ids)) - set(markers))
    if missing:
        raise CalibrationError(f"marqueurs requis absents de {path}: {missing}")

    marker_size = spec.get("marker_size_m")
    if marker_size is not None:
        marker_size = float(marker_size)
        if not np.isfinite(marker_size) or marker_size <= 0.0:
            raise CalibrationError("marker_size_m doit être strictement positif")
    return Workspace(
        markers=markers,
        frame_id=str(spec.get("frame_id", "base_link")),
        marker_size_m=marker_size,
    )


def load_geometry(
    calib_npz: str | Path = DEFAULT_CALIB_NPZ,
    meta_json: str | Path = DEFAULT_META_JSON,
    workspace_yaml: str | Path = DEFAULT_WORKSPACE_YAML,
) -> GeometryBundle:
    """Charge en une fois intrinsèques, distorsion, meta et workspace."""

    return GeometryBundle(
        camera=load_camera_calibration(calib_npz, meta_json),
        workspace=load_workspace(workspace_yaml),
    )


def _aruco_dictionary(name: str):
    if not hasattr(cv2, "aruco"):
        raise CalibrationError("OpenCV a été compilé sans le module aruco")
    code = getattr(cv2.aruco, name, None)
    if code is None:
        raise CalibrationError(f"dictionnaire ArUco OpenCV inconnu : {name}")
    return cv2.aruco.getPredefinedDictionary(code)


def _detector_parameters():
    # Préférer explicitement l'API legacy lorsqu'elle existe. Certaines builds
    # OpenCV 4.6 exposent une partie des symboles modernes mais leur constructeur
    # Python ``DetectorParameters()`` peut segfault ; OpenCV 5 n'expose plus
    # ``*_create`` et passe donc naturellement par la nouvelle API.
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        params = cv2.aruco.DetectorParameters_create()
    else:
        params = cv2.aruco.DetectorParameters()
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 61
    params.adaptiveThreshWinSizeStep = 4
    params.minMarkerPerimeterRate = 0.01
    params.maxMarkerPerimeterRate = 4.0
    if hasattr(cv2.aruco, "CORNER_REFINE_SUBPIX"):
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return params


def detect_marker_centres(
    frame_bgr: np.ndarray,
    *,
    dictionary_name: str = DEFAULT_DICTIONARY,
    target_ids: Sequence[int] = DEFAULT_MARKER_IDS,
    detect_scale: float = 1.0,
) -> dict[int, tuple[float, float]]:
    """Détecte les centres 2D des ArUco demandés dans une frame BGR.

    Les IDs inconnus sont ignorés. En cas de doublon d'un ID, le quadrilatère
    de plus grande aire est conservé. Les coordonnées retournées restent dans la
    résolution de la frame d'origine, même avec ``detect_scale != 1``.
    """

    frame = np.asarray(frame_bgr)
    if frame.dtype != np.uint8 or frame.ndim not in (2, 3):
        raise ValueError("frame_bgr doit être une image uint8 grise, BGR ou BGRA")
    if frame.ndim == 2:
        gray = frame
    elif frame.shape[2] == 3:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    elif frame.shape[2] == 4:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGRA2GRAY)
    else:
        raise ValueError("frame_bgr doit avoir 1, 3 ou 4 canaux")
    if not np.isfinite(detect_scale) or detect_scale <= 0.0:
        raise ValueError("detect_scale doit être strictement positif")

    scale = float(detect_scale)
    work = gray
    if abs(scale - 1.0) > 1e-9:
        interpolation = cv2.INTER_CUBIC if scale > 1.0 else cv2.INTER_AREA
        work = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=interpolation)

    dictionary = _aruco_dictionary(dictionary_name)
    params = _detector_parameters()
    if hasattr(cv2.aruco, "detectMarkers") and hasattr(
        cv2.aruco, "DetectorParameters_create"
    ):
        corners, ids, _ = cv2.aruco.detectMarkers(
            work, dictionary, parameters=params
        )
    else:
        detector = cv2.aruco.ArucoDetector(dictionary, params)
        corners, ids, _ = detector.detectMarkers(work)
    if ids is None:
        return {}

    wanted = set(map(int, target_ids))
    best: dict[int, tuple[float, tuple[float, float]]] = {}
    for raw_id, raw_corners in zip(ids.reshape(-1), corners):
        marker_id = int(raw_id)
        if marker_id not in wanted:
            continue
        quad = np.asarray(raw_corners, dtype=np.float64).reshape(4, 2) / scale
        area = abs(float(cv2.contourArea(quad.astype(np.float32))))
        centre = tuple(float(v) for v in quad.mean(axis=0))
        if marker_id not in best or area > best[marker_id][0]:
            best[marker_id] = area, centre
    return {marker_id: value[1] for marker_id, value in best.items()}


# Orthographe anglaise utile aux appelants externes.
detect_marker_centers = detect_marker_centres
detect_aruco_centres = detect_marker_centres
detect_aruco_centers = detect_marker_centres


def _workspace_markers(workspace: Workspace | Mapping[int, Sequence[float]]):
    return workspace.markers if isinstance(workspace, Workspace) else workspace


def _camera_arrays(
    camera: CameraCalibration | np.ndarray,
    dist: np.ndarray | Sequence[float] | None,
) -> tuple[np.ndarray, np.ndarray]:
    if isinstance(camera, CameraCalibration):
        if dist is not None:
            raise ValueError("dist ne doit pas être fourni avec CameraCalibration")
        K = np.asarray(camera.K, dtype=np.float64)
        distortion = np.asarray(camera.dist, dtype=np.float64).reshape(-1)
    else:
        K = np.asarray(camera, dtype=np.float64)
        if dist is None:
            raise ValueError("dist est requis quand camera est une matrice K")
        distortion = np.asarray(dist, dtype=np.float64).reshape(-1)
    _validate_intrinsics(K, distortion)
    return K, distortion


def estimate_live_extrinsic(
    marker_centres_px: Mapping[int, Sequence[float]],
    workspace: Workspace | Mapping[int, Sequence[float]],
    camera: CameraCalibration | np.ndarray,
    dist: np.ndarray | Sequence[float] | None = None,
    *,
    required_ids: Sequence[int] | None = DEFAULT_MARKER_IDS,
    min_markers: int = 4,
    max_rms_px: float | None = 3.0,
    camera_z_range_m: tuple[float, float] = (0.05, 3.0),
    min_depth_m: float = 1e-4,
    planarity_tolerance_m: float = 1e-5,
) -> PoseEstimate:
    """Estime et valide ``T_cam_world``/``T_world_cam`` avec PnP IPPE.

    Par défaut les quatre IDs 19/23/25/26 doivent être présents. IPPE peut
    proposer les deux solutions miroir d'un PnP planaire ; seules les solutions
    dont tous les points sont devant la caméra et dont la hauteur caméra est
    plausible sont mises en concurrence.
    """

    K, distortion = _camera_arrays(camera, dist)
    known = _workspace_markers(workspace)
    observed: dict[int, np.ndarray] = {}
    for raw_id, raw_uv in marker_centres_px.items():
        marker_id = int(raw_id)
        if marker_id not in known:
            continue
        uv = np.asarray(raw_uv, dtype=np.float64).reshape(2)
        if np.all(np.isfinite(uv)):
            observed[marker_id] = uv

    if required_ids is not None:
        missing = sorted(set(map(int, required_ids)) - set(observed))
        if missing:
            raise PoseEstimationError(
                f"quatre tags requis par défaut ; IDs absents : {missing}"
            )
    if min_markers < 4:
        raise ValueError("IPPE live requiert au moins quatre centres non collinéaires")
    if len(observed) < min_markers:
        raise PoseEstimationError(
            f"{len(observed)} tags utilisables, {min_markers} requis"
        )

    marker_ids = tuple(sorted(observed))
    object_points = np.asarray(
        [known[marker_id] for marker_id in marker_ids], dtype=np.float64
    ).reshape(-1, 3)
    image_points = np.asarray(
        [observed[marker_id] for marker_id in marker_ids], dtype=np.float64
    ).reshape(-1, 2)
    if not np.all(np.isfinite(object_points)):
        raise PoseEstimationError("coordonnées monde non finies")
    if float(np.ptp(object_points[:, 2])) > planarity_tolerance_m:
        raise PoseEstimationError("SOLVEPNP_IPPE exige des marqueurs coplanaires")
    centred_xy = object_points[:, :2] - object_points[:, :2].mean(axis=0)
    if np.linalg.matrix_rank(centred_xy, tol=1e-9) < 2:
        raise PoseEstimationError("centres monde collinéaires : PnP impossible")

    try:
        result = cv2.solvePnPGeneric(
            object_points,
            image_points,
            K,
            distortion,
            flags=cv2.SOLVEPNP_IPPE,
        )
    except cv2.error as exc:
        raise PoseEstimationError(f"solvePnP IPPE a échoué : {exc}") from exc
    count, rvecs, tvecs = int(result[0]), result[1], result[2]
    if count <= 0:
        raise PoseEstimationError("solvePnP IPPE n'a retourné aucune solution")

    low_z, high_z = map(float, camera_z_range_m)
    if not (np.isfinite(low_z) and np.isfinite(high_z) and 0.0 <= low_z < high_z):
        raise ValueError("camera_z_range_m doit être un intervalle positif croissant")
    if min_depth_m <= 0.0:
        raise ValueError("min_depth_m doit être strictement positif")

    physical_candidates = []
    rejection_notes = []
    plane_z = float(np.mean(object_points[:, 2]))
    for index, (raw_rvec, raw_tvec) in enumerate(zip(rvecs, tvecs)):
        rvec = np.asarray(raw_rvec, dtype=np.float64).reshape(3, 1)
        tvec = np.asarray(raw_tvec, dtype=np.float64).reshape(3, 1)
        # IPPE sert à lever proprement l'ambiguïté planaire. Sur OpenCV 4.6,
        # ses solutions peuvent toutefois garder ~2 px de biais, même avec des
        # points synthétiques parfaits. Un raffinement LM depuis *chaque* candidat
        # conserve la branche IPPE et supprime ce biais numérique.
        if hasattr(cv2, "solvePnPRefineLM"):
            try:
                rvec, tvec = cv2.solvePnPRefineLM(
                    object_points,
                    image_points,
                    K,
                    distortion,
                    rvec.copy(),
                    tvec.copy(),
                )
            except cv2.error:
                pass
        R, _ = cv2.Rodrigues(rvec)
        camera_position = -R.T @ tvec.reshape(3)
        points_camera = (R @ object_points.T + tvec).T
        depths = points_camera[:, 2]
        projected, _ = cv2.projectPoints(
            object_points, rvec, tvec, K, distortion
        )
        residuals = np.linalg.norm(
            projected.reshape(-1, 2) - image_points, axis=1
        )
        rms = float(np.sqrt(np.mean(np.square(residuals))))

        in_front = bool(np.all(depths > min_depth_m))
        camera_z = float(camera_position[2])
        plausible_z = low_z <= camera_z <= high_z and camera_z > plane_z
        if in_front and plausible_z:
            physical_candidates.append(
                (rms, index, rvec, tvec, R, camera_position, depths, residuals)
            )
        else:
            rejection_notes.append(
                f"solution {index}: cam_z={camera_z:.3f} m, "
                f"profondeur_min={float(np.min(depths)):.3f} m"
            )

    if not physical_candidates:
        detail = "; ".join(rejection_notes) or "aucun candidat"
        raise PoseEstimationError(
            "aucune solution IPPE physique (caméra Z plausible et points devant) : "
            + detail
        )

    rms, _, rvec, tvec, R, camera_position, depths, residuals = min(
        physical_candidates, key=lambda candidate: candidate[0]
    )
    if max_rms_px is not None:
        if not np.isfinite(max_rms_px) or max_rms_px <= 0.0:
            raise ValueError("max_rms_px doit être positif ou None")
        if rms > float(max_rms_px):
            raise PoseEstimationError(
                f"RMS reprojection {rms:.3f} px > seuil {float(max_rms_px):.3f} px"
            )

    T_cam_world = np.eye(4, dtype=np.float64)
    T_cam_world[:3, :3] = R
    T_cam_world[:3, 3] = tvec.reshape(3)
    T_world_cam = np.linalg.inv(T_cam_world)
    per_marker = {
        marker_id: float(error)
        for marker_id, error in zip(marker_ids, residuals)
    }
    return PoseEstimate(
        T_cam_world=T_cam_world,
        T_world_cam=T_world_cam,
        rvec=rvec,
        tvec=tvec,
        rms_reprojection_px=rms,
        per_marker_reprojection_px=per_marker,
        marker_ids=marker_ids,
        camera_position_world_m=camera_position,
        depths_camera_m=np.asarray(depths, dtype=np.float64),
    )


# Alias court pour l'intégration avec le pick adaptatif.
estimate_pose = estimate_live_extrinsic


def undistort_pixel(
    pixel_uv: Sequence[float],
    camera: CameraCalibration | np.ndarray,
    dist: np.ndarray | Sequence[float] | None = None,
) -> np.ndarray:
    """Retourne les coordonnées normalisées ``(x, y)`` du pixel corrigé."""

    K, distortion = _camera_arrays(camera, dist)
    pixel = np.asarray(pixel_uv, dtype=np.float64).reshape(1, 1, 2)
    if not np.all(np.isfinite(pixel)):
        raise ProjectionError("pixel non fini")
    normalised = cv2.undistortPoints(pixel, K, distortion)
    return normalised.reshape(2)


def deproject_pixel_to_base(
    pixel_uv: Sequence[float],
    camera: CameraCalibration | np.ndarray,
    T_world_cam: PoseEstimate | np.ndarray,
    projection_z: float,
    dist: np.ndarray | Sequence[float] | None = None,
    *,
    pixel_is_undistorted: bool = False,
    require_in_front: bool = True,
    parallel_epsilon: float = 1e-9,
) -> np.ndarray:
    """Intersecte le rayon d'un pixel avec ``z=projection_z`` dans la base.

    Par défaut ``pixel_uv`` est le pixel brut (distordu) : ``undistortPoints``
    est appliqué avant la projection. Avec ``pixel_is_undistorted=True``, le
    pixel est supposé déjà rectifié mais encore exprimé en coordonnées pixel
    associées à ``K``.
    """

    K, distortion = _camera_arrays(camera, dist)
    pixel = np.asarray(pixel_uv, dtype=np.float64).reshape(2)
    if not np.all(np.isfinite(pixel)) or not np.isfinite(projection_z):
        raise ProjectionError("pixel et projection_z doivent être finis")
    if pixel_is_undistorted:
        homogeneous = np.linalg.inv(K) @ np.array([pixel[0], pixel[1], 1.0])
        ray_camera = homogeneous / homogeneous[2]
    else:
        xy = cv2.undistortPoints(
            pixel.reshape(1, 1, 2), K, distortion
        ).reshape(2)
        ray_camera = np.array([xy[0], xy[1], 1.0], dtype=np.float64)

    transform = (
        T_world_cam.T_world_cam
        if isinstance(T_world_cam, PoseEstimate)
        else np.asarray(T_world_cam, dtype=np.float64)
    )
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ProjectionError("T_world_cam doit être une matrice homogène 4x4 finie")
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8):
        raise ProjectionError("dernière ligne homogène de T_world_cam invalide")

    origin_world = transform[:3, 3]
    direction_world = transform[:3, :3] @ ray_camera
    denominator = float(direction_world[2])
    if abs(denominator) <= parallel_epsilon:
        raise ProjectionError("rayon presque parallèle au plan de projection")
    distance = (float(projection_z) - float(origin_world[2])) / denominator
    if require_in_front and distance <= 0.0:
        raise ProjectionError("intersection située derrière la caméra")
    point_world = origin_world + distance * direction_world
    point_world[2] = float(projection_z)  # évite le bruit flottant du calcul
    return point_world


# Aliases explicites pour les conventions monde/base du projet.
pixel_to_base = deproject_pixel_to_base
deproject_pixel_to_world = deproject_pixel_to_base


__all__ = [
    "CalibrationError",
    "CameraCalibration",
    "DEFAULT_CALIB_NPZ",
    "DEFAULT_DICTIONARY",
    "DEFAULT_MARKER_IDS",
    "DEFAULT_META_JSON",
    "DEFAULT_WORKSPACE_YAML",
    "GeometryBundle",
    "GeometryError",
    "PoseEstimate",
    "PoseEstimationError",
    "ProjectionError",
    "Workspace",
    "deproject_pixel_to_base",
    "deproject_pixel_to_world",
    "detect_aruco_centers",
    "detect_aruco_centres",
    "detect_marker_centers",
    "detect_marker_centres",
    "estimate_live_extrinsic",
    "estimate_pose",
    "load_camera_calibration",
    "load_geometry",
    "load_workspace",
    "pixel_to_base",
    "undistort_pixel",
]
