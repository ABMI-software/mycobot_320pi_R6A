#!/usr/bin/env python3
"""Learn a safe joint-space pick from demonstrations.

The learned input is ``(x, y)`` in the robot base frame (metres).  A pixel is
kept as metadata, but is first projected to the object's centre-height plane
with the calibrated Arducam intrinsics, the fixed eye-to-hand camera-to-base
extrinsic, and ``pixel_to_base``.

The four table ArUco are detected live as the moving geometric safety boundary;
they are not used as object detectors and they do not replace the eye-to-hand
extrinsic.  Therefore the board may slide on the calibrated table plane while
the camera remains rigid with the robot base.  Replans within one pick reuse
the initially validated boundary because the arm may hide the markers.

Teaching is deliberately transactional so the ball can be observed before the
arm hides it::

    # 1. ball still visible (stable multi-frame observation)
    .venv/bin/python scripts/adaptive_pick_by_demo.py --begin-live
    # 2. move the arm by hand to the approach pose, then record encoder angles
    .venv/bin/python scripts/adaptive_pick_by_demo.py --capture-approach --read-current-angles
    # 3. move to the pick pose; this commits the complete demo atomically
    .venv/bin/python scripts/adaptive_pick_by_demo.py --capture-pick --read-current-angles

Angles may instead be supplied with ``--angles A1 ... A6``.  Planning is the
default-safe operation and never connects to the robot::

    .venv/bin/python scripts/adaptive_pick_by_demo.py --plan-live

Real motion additionally requires ``--execute`` and an interactive confirmation.
Only learned joint angles are sent; this module never calls Cartesian IK.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
import math
import os
import queue
import sys
import tempfile
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Iterable, Sequence


REPO = Path(__file__).resolve().parents[1]
DEFAULT_DATA = Path(__file__).with_name("adaptive_pick_demos.json")
# Keep this pair together: the eye-to-hand transform was solved for this
# physical Arducam and cam_0 intrinsics.  It stays fixed while the ArUco board
# may slide on the table plane.
DEFAULT_INTRINSIC = REPO / "training/calibration/cam_0.meta.json"
DEFAULT_EXTRINSIC = REPO / "training/calibration/arducam_extrinsic_handeye.yaml"
DEFAULT_WORKSPACE = REPO / "training/calibration/workspace_markers.yaml"
GRIPPER_ID = 14
STATUS_TEXT = {0: "en mouvement", 1: "rien saisi", 2: "objet saisi", 3: "objet tombe"}
ANGLE_LIMIT_DEG = 180.0
DEFAULT_PROJECTION_Z_M = 0.0335
DEFAULT_MIN_DEMO_SEPARATION_M = 0.030
DEFAULT_MAX_NEAREST_DEMO_DISTANCE_M = 0.100
EPS = 1e-10


class DemoError(ValueError):
    """Invalid/incomplete demonstrations or an unsafe query."""


class ExecutionError(RuntimeError):
    """A real-robot step did not complete safely."""


def validate_command_response(command: dict[str, Any], response: str) -> None:
    """Reject acknowledgements that do not correspond to the requested value."""
    if command.get("action") == "pro_gripper_angle":
        import re
        match = re.search(r"OK:\s*pro gripper angle\s+(-?\d+)", response)
        expected = int(command["angle"])
        if match is None or int(match.group(1)) != expected:
            raise ExecutionError(f"accuse pince incoherent: attendu {expected}, recu {response!r}")


@dataclass(frozen=True)
class Prediction:
    base_xy_m: tuple[float, float]
    pick_approach_deg: tuple[float, ...]
    pick_deg: tuple[float, ...]
    weights: tuple[float, ...]


@dataclass(frozen=True)
class LiveObservation:
    pixel_uv: tuple[float, float]
    workspace_base_xy: tuple[tuple[float, float], ...]
    marker_ids: tuple[int, ...]
    max_marker_spread_px: float


def _finite_pair(value: Sequence[Any], label: str) -> tuple[float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise DemoError(f"{label} doit contenir exactement 2 nombres")
    out = (float(value[0]), float(value[1]))
    if not all(math.isfinite(v) for v in out):
        raise DemoError(f"{label} contient une valeur non finie")
    return out


def validate_angles(value: Sequence[Any], label: str = "angles") -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != 6:
        raise DemoError(f"{label} doit contenir exactement 6 angles")
    angles = tuple(float(v) for v in value)
    if not all(math.isfinite(v) for v in angles):
        raise DemoError(f"{label} contient une valeur non finie")
    if any(abs(v) > ANGLE_LIMIT_DEG for v in angles):
        raise DemoError(f"{label} hors limite de securite +/-{ANGLE_LIMIT_DEG:g} deg")
    return angles


def _cross(o: Sequence[float], a: Sequence[float], b: Sequence[float]) -> float:
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def convex_hull(points: Iterable[Sequence[float]]) -> list[tuple[float, float]]:
    """Andrew monotone-chain hull, counter-clockwise, without repeated endpoint."""
    pts = sorted(set(_finite_pair(tuple(p), "point") for p in points))
    if len(pts) <= 1:
        return pts
    lower: list[tuple[float, float]] = []
    for p in pts:
        while len(lower) >= 2 and _cross(lower[-2], lower[-1], p) <= EPS:
            lower.pop()
        lower.append(p)
    upper: list[tuple[float, float]] = []
    for p in reversed(pts):
        while len(upper) >= 2 and _cross(upper[-2], upper[-1], p) <= EPS:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def point_in_convex_hull(point: Sequence[float], hull: Sequence[Sequence[float]],
                         tolerance: float = 1e-9) -> bool:
    """Return True on or inside a counter-clockwise convex polygon."""
    p = _finite_pair(tuple(point), "requete")
    if len(hull) < 3:
        return False
    scale = max(1.0, *(abs(c) for vertex in hull for c in vertex))
    tol = tolerance * scale * scale
    return all(_cross(hull[i], hull[(i + 1) % len(hull)], p) >= -tol
               for i in range(len(hull)))


def _point_segment_distance(point: Sequence[float], a: Sequence[float],
                            b: Sequence[float]) -> float:
    px, py = point
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    denom = dx * dx + dy * dy
    t = 0.0 if denom <= EPS else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denom))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def workspace_clearance(point: Sequence[float], polygon: Sequence[Sequence[float]]) -> float:
    """Signed distance to a convex workspace boundary (positive inside)."""
    hull = convex_hull(polygon)
    if len(hull) < 3:
        raise DemoError("la zone ArUco requiert au moins 3 marqueurs non collineaires")
    distance = min(_point_segment_distance(point, hull[i], hull[(i + 1) % len(hull)])
                   for i in range(len(hull)))
    return distance if point_in_convex_hull(point, hull) else -distance


def validate_dataset(data: dict[str, Any], require_ready: bool = True) -> list[dict[str, Any]]:
    if not isinstance(data, dict) or data.get("schema_version") != 1:
        raise DemoError("dataset absent ou schema_version != 1")
    if data.get("input_space") != "base_xy_m":
        raise DemoError("input_space doit etre base_xy_m (les pixels bruts sont interdits)")
    calibration = data.get("calibration")
    if not isinstance(calibration, dict):
        raise DemoError("section calibration absente")
    demos = data.get("demonstrations")
    if not isinstance(demos, list):
        raise DemoError("demonstrations doit etre une liste")

    safety = data.get("safety", {})
    min_separation = float(safety.get("min_demo_separation_m", DEFAULT_MIN_DEMO_SEPARATION_M))
    max_nearest = float(safety.get("max_nearest_demo_distance_m",
                                   DEFAULT_MAX_NEAREST_DEMO_DISTANCE_M))
    if not math.isfinite(min_separation) or min_separation <= 0:
        raise DemoError("min_demo_separation_m doit etre > 0")
    if not math.isfinite(max_nearest) or max_nearest <= 0:
        raise DemoError("max_nearest_demo_distance_m doit etre > 0")

    normalized: list[dict[str, Any]] = []
    seen: list[tuple[float, float]] = []
    for index, demo in enumerate(demos):
        if not isinstance(demo, dict):
            raise DemoError(f"demonstration {index}: objet JSON attendu")
        ball = demo.get("ball")
        if not isinstance(ball, dict):
            raise DemoError(f"demonstration {index}: section ball absente")
        xy = _finite_pair(ball.get("base_xy_m"), f"demonstration {index}.ball.base_xy_m")
        nearest_old = min((math.dist(xy, old) for old in seen), default=math.inf)
        if nearest_old < min_separation:
            raise DemoError(f"demonstration {index}: trop proche d'une autre "
                            f"({nearest_old*1000:.1f} mm < {min_separation*1000:.1f} mm)")
        seen.append(xy)
        pixel = _finite_pair(ball.get("pixel_uv"), f"demonstration {index}.ball.pixel_uv")
        approach = validate_angles(demo.get("pick_approach_deg"),
                                   f"demonstration {index}.pick_approach_deg")
        pick = validate_angles(demo.get("pick_deg"), f"demonstration {index}.pick_deg")
        normalized.append({"xy": xy, "pixel": pixel, "approach": approach, "pick": pick})

    if require_ready:
        if len(normalized) < 3:
            raise DemoError("au moins 3 demonstrations sont requises")
        if len(convex_hull(item["xy"] for item in normalized)) < 3:
            raise DemoError("les demonstrations doivent contenir 3 XY non collineaires")
    return normalized


def interpolate(data: dict[str, Any], query_xy: Sequence[float], power: float = 2.0) -> Prediction:
    """IDW interpolation of every joint; refuses all extrapolation."""
    if not math.isfinite(power) or power <= 0:
        raise DemoError("la puissance IDW doit etre > 0")
    demos = validate_dataset(data, require_ready=True)
    query = _finite_pair(tuple(query_xy), "query_xy")
    hull = convex_hull(d["xy"] for d in demos)
    if not point_in_convex_hull(query, hull):
        raise DemoError("cible hors enveloppe convexe des demonstrations: extrapolation refusee")

    distances = [math.dist(query, d["xy"]) for d in demos]
    max_nearest = float(data.get("safety", {}).get(
        "max_nearest_demo_distance_m", DEFAULT_MAX_NEAREST_DEMO_DISTANCE_M))
    nearest = min(distances)
    if nearest > max_nearest:
        raise DemoError(f"cible trop loin des demonstrations "
                        f"({nearest*1000:.1f} mm > {max_nearest*1000:.1f} mm)")
    exact = next((i for i, distance in enumerate(distances) if distance <= 1e-9), None)
    if exact is not None:
        weights = [1.0 if i == exact else 0.0 for i in range(len(demos))]
    else:
        raw = [distance ** (-power) for distance in distances]
        total = sum(raw)
        weights = [weight / total for weight in raw]

    def blend(key: str) -> tuple[float, ...]:
        result = tuple(sum(weights[i] * demos[i][key][joint] for i in range(len(demos)))
                       for joint in range(6))
        return validate_angles(result, f"prediction {key}")

    return Prediction(query, blend("approach"), blend("pick"), tuple(weights))


def validate_live_execution_dataset(data: dict[str, Any]) -> None:
    """Real execution accepts only calibrated demos with a live ArUco boundary."""
    validate_dataset(data, require_ready=True)
    for index, demo in enumerate(data["demonstrations"]):
        localization = demo.get("ball", {}).get("localization", {})
        ids = localization.get("marker_ids", [])
        polygon = localization.get("workspace_base_xy", [])
        spread = float(localization.get("max_marker_spread_px", math.inf))
        if (localization.get("source") != "eye_to_hand+aruco_boundary_live"
                or len(set(map(int, ids))) != 4 or len(polygon) != 4
                or not math.isfinite(spread) or spread > 3.0):
            raise DemoError(f"demo {index} sans extrinsèque eye-to-hand et frontière "
                            "ArUco live valides; execution interdite")


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise DemoError(f"fichier de calibration introuvable: {path}")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _path_record(path: Path) -> dict[str, str]:
    resolved = path.expanduser().resolve()
    try:
        shown = str(resolved.relative_to(REPO))
    except ValueError:
        shown = str(resolved)
    return {"path": shown, "sha256": _sha256(resolved)}


def _resolve_record(record: dict[str, str]) -> Path:
    path = Path(record["path"])
    return path if path.is_absolute() else REPO / path


def new_dataset(intrinsic: Path, extrinsic: Path, workspace: Path,
                projection_z_m: float, workspace_margin_m: float,
                min_demo_separation_m: float = DEFAULT_MIN_DEMO_SEPARATION_M,
                max_nearest_demo_distance_m: float = DEFAULT_MAX_NEAREST_DEMO_DISTANCE_M) -> dict[str, Any]:
    if not math.isfinite(projection_z_m):
        raise DemoError("projection_z doit etre fini")
    if not math.isfinite(workspace_margin_m) or workspace_margin_m < 0:
        raise DemoError("workspace_margin doit etre positif")
    if not math.isfinite(min_demo_separation_m) or min_demo_separation_m <= 0:
        raise DemoError("min_demo_separation doit etre > 0")
    if not math.isfinite(max_nearest_demo_distance_m) or max_nearest_demo_distance_m <= 0:
        raise DemoError("max_nearest_demo_distance doit etre > 0")
    return {
        "schema_version": 1,
        "input_space": "base_xy_m",
        "units": {"position": "m", "joint_angles": "deg", "pixel": "px"},
        "calibration": {
            "camera": "arducam",
            "intrinsic_stem": "cam_0",
            "intrinsic": _path_record(intrinsic),
            "extrinsic": _path_record(extrinsic),
            "workspace_markers": _path_record(workspace),
            "projection_z_m": float(projection_z_m),
            "workspace_margin_m": float(workspace_margin_m),
        },
        "safety": {
            "min_demo_separation_m": float(min_demo_separation_m),
            "max_nearest_demo_distance_m": float(max_nearest_demo_distance_m),
        },
        "demonstrations": [],
    }


def load_dataset(path: Path, require_ready: bool = False) -> dict[str, Any]:
    if not path.is_file():
        raise DemoError(f"dataset introuvable: {path}")
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise DemoError(f"dataset illisible: {exc}") from exc
    validate_dataset(data, require_ready=require_ready)
    return data


def verify_calibration(data: dict[str, Any]) -> None:
    calibration = data["calibration"]
    for key in ("intrinsic", "extrinsic", "workspace_markers"):
        record = calibration.get(key)
        if not isinstance(record, dict) or "path" not in record or "sha256" not in record:
            raise DemoError(f"calibration.{key} invalide")
        path = _resolve_record(record)
        if _sha256(path) != record["sha256"]:
            raise DemoError(f"{path} a change depuis l'apprentissage; recalibrer/reapprendre")


def _load_workspace(data: dict[str, Any]) -> list[tuple[float, float]]:
    try:
        import yaml
    except ImportError as exc:
        raise DemoError("PyYAML est requis pour lire la zone ArUco") from exc
    path = _resolve_record(data["calibration"]["workspace_markers"])
    content = yaml.safe_load(path.read_text(encoding="utf-8"))
    markers = content.get("markers", {}) if isinstance(content, dict) else {}
    return [_finite_pair(value[:2], f"marqueur ArUco {key}") for key, value in markers.items()]


def intrinsic_resolution(path: Path) -> tuple[int, int]:
    meta = json.loads(path.read_text(encoding="utf-8"))
    resolution = meta.get("resolution")
    if not isinstance(resolution, list) or len(resolution) != 2:
        raise DemoError(f"resolution intrinsèque absente: {path}")
    width, height = int(resolution[0]), int(resolution[1])
    if width <= 0 or height <= 0:
        raise DemoError(f"resolution intrinsèque invalide: {resolution}")
    return width, height


def load_intrinsic_parameters(intrinsic_path: Path) -> tuple[Any, Any]:
    meta = json.loads(intrinsic_path.read_text(encoding="utf-8"))
    results = meta["results"]
    try:
        import numpy as np
    except ImportError as exc:
        raise DemoError("numpy est requis pour la projection calibree") from exc
    K = np.array([[float(results["fx"]), 0.0, float(results["cx"])],
                  [0.0, float(results["fy"]), float(results["cy"])],
                  [0.0, 0.0, 1.0]], dtype=np.float64)
    dist = np.asarray(results.get("dist_coeffs", []), dtype=np.float64)
    return K, dist


def load_extrinsic_transform(extrinsic_path: Path) -> Any:
    """Load and sanity-check the fixed camera-to-base transform."""
    try:
        import numpy as np
        import yaml
    except ImportError as exc:
        raise DemoError("numpy et PyYAML sont requis pour l'extrinsèque") from exc
    try:
        content = yaml.safe_load(extrinsic_path.read_text(encoding="utf-8"))
        transform = np.asarray(content["T_world_cam"], dtype=np.float64)
    except (OSError, KeyError, TypeError, ValueError) as exc:
        raise DemoError(f"extrinsèque illisible: {extrinsic_path}: {exc}") from exc
    if (transform.shape != (4, 4) or not np.all(np.isfinite(transform))
            or not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8)):
        raise DemoError("T_world_cam extrinsèque invalide")
    rotation = transform[:3, :3]
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=2e-2)
            or not 0.98 <= float(np.linalg.det(rotation)) <= 1.02):
        raise DemoError("rotation de l'extrinsèque invalide")
    camera_z = float(transform[2, 3])
    if not 0.20 <= camera_z <= 2.0:
        raise DemoError(f"hauteur caméra extrinsèque non plausible: {camera_z:.3f} m")
    return transform


def project_raw_pixel_to_plane(intrinsic_path: Path, pixel_uv: Sequence[float],
                               T_world_cam: Any, projection_z_m: float) -> tuple[float, float, float]:
    """Project one distorted camera pixel onto a horizontal plane in base_link."""
    pixel = _finite_pair(tuple(pixel_uv), "pixel_uv")
    try:
        import numpy as np
    except ImportError as exc:
        raise DemoError("numpy est requis pour la projection calibree") from exc
    K, dist = load_intrinsic_parameters(intrinsic_path)
    T_world_cam = np.asarray(T_world_cam, dtype=np.float64)
    if T_world_cam.shape != (4, 4) or not np.all(np.isfinite(T_world_cam)):
        raise DemoError("T_world_cam invalide")
    try:
        from live_aruco_geometry import ProjectionError, deproject_pixel_to_base
        xyz = deproject_pixel_to_base(pixel, K, T_world_cam, float(projection_z_m), dist)
    except (ProjectionError, ValueError) as exc:
        raise DemoError(f"projection pixel->base impossible: {exc}") from exc
    xyz_tuple = tuple(float(value) for value in xyz)
    if len(xyz_tuple) != 3 or not all(math.isfinite(value) for value in xyz_tuple):
        raise DemoError("projection pixel_to_base non finie")
    return xyz_tuple  # type: ignore[return-value]


def project_pixel_with_extrinsic(
    data: dict[str, Any], pixel_uv: Sequence[float], T_world_cam: Any,
    workspace_polygon: Sequence[Sequence[float]] | None = None,
) -> tuple[float, float, float]:
    """Project an object pixel and gate it with the current ArUco boundary."""
    verify_calibration(data)
    intrinsic_path = _resolve_record(data["calibration"]["intrinsic"])
    projection_z = float(data["calibration"].get("projection_z_m", DEFAULT_PROJECTION_Z_M))
    xyz_tuple = project_raw_pixel_to_plane(
        intrinsic_path, pixel_uv, T_world_cam, projection_z)
    polygon = list(workspace_polygon) if workspace_polygon is not None else _load_workspace(data)
    clearance = workspace_clearance(xyz_tuple[:2], polygon)
    margin = float(data["calibration"]["workspace_margin_m"])
    if clearance < margin:
        raise DemoError(f"cible hors zone ArUco sure: bord={clearance*1000:.1f} mm, "
                        f"marge={margin*1000:.1f} mm")
    return xyz_tuple  # type: ignore[return-value]


def pixel_to_calibrated_base(data: dict[str, Any], pixel_uv: Sequence[float]) -> tuple[float, float, float]:
    """Diagnostic projection with the pinned static extrinsic."""
    extrinsic_path = _resolve_record(data["calibration"]["extrinsic"])
    return project_pixel_with_extrinsic(
        data, pixel_uv, load_extrinsic_transform(extrinsic_path))


def _atomic_json_write(path: Path, value: dict[str, Any], create_only: bool = False) -> None:
    path = path.expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    if create_only and path.exists():
        raise DemoError(f"transaction deja en cours: {path}")
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    temporary_path = Path(temporary)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(value, stream, indent=2, ensure_ascii=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        if create_only and path.exists():
            raise DemoError(f"transaction deja en cours: {path}")
        os.replace(temporary_path, path)
    finally:
        if temporary_path.exists():
            temporary_path.unlink()


def _pending_path(data_path: Path) -> Path:
    return data_path.with_name(data_path.name + ".pending")


def begin_demo(data_path: Path, pixel_uv: Sequence[float], name: str | None,
               calibration_defaults: dict[str, Any], localized_xyz: Sequence[float] | None = None,
               localization: dict[str, Any] | None = None,
               workspace_polygon: Sequence[Sequence[float]] | None = None) -> dict[str, Any]:
    if data_path.exists():
        data = load_dataset(data_path)
    else:
        data = new_dataset(**calibration_defaults)
    if localized_xyz is None:
        xyz = pixel_to_calibrated_base(data, pixel_uv)
        localization = {"source": "static_yaml_diagnostic"}
    else:
        if len(localized_xyz) != 3 or not all(math.isfinite(float(v)) for v in localized_xyz):
            raise DemoError("XYZ live invalide")
        xyz = tuple(float(v) for v in localized_xyz)
        polygon = list(workspace_polygon) if workspace_polygon is not None else _load_workspace(data)
        clearance = workspace_clearance(xyz[:2], polygon)
        if clearance < float(data["calibration"]["workspace_margin_m"]):
            raise DemoError("XYZ live hors zone ArUco sure")
    pending = {
        "schema_version": 1,
        "dataset_path": str(data_path.expanduser().resolve()),
        "calibration": data["calibration"],
        "safety": data.get("safety", {
            "min_demo_separation_m": DEFAULT_MIN_DEMO_SEPARATION_M,
            "max_nearest_demo_distance_m": DEFAULT_MAX_NEAREST_DEMO_DISTANCE_M,
        }),
        "demo": {
            "name": name or f"demo-{len(data['demonstrations']) + 1}",
            "captured_at": datetime.now(timezone.utc).isoformat(),
            "ball": {"pixel_uv": list(_finite_pair(tuple(pixel_uv), "pixel_uv")),
                     "base_xyz_m": list(xyz), "base_xy_m": list(xyz[:2]),
                     "localization": localization or {"source": "unknown"}},
        },
    }
    _atomic_json_write(_pending_path(data_path), pending, create_only=True)
    return pending


def _read_pending(data_path: Path) -> dict[str, Any]:
    path = _pending_path(data_path)
    if not path.is_file():
        raise DemoError(f"aucune demonstration en attente: commence par --begin-demo U V")
    pending = json.loads(path.read_text(encoding="utf-8"))
    if pending.get("dataset_path") != str(data_path.expanduser().resolve()):
        raise DemoError("la transaction pending appartient a un autre dataset")
    return pending


def capture_pending(data_path: Path, stage: str, angles: Sequence[float]) -> bool:
    if stage not in ("pick_approach_deg", "pick_deg"):
        raise DemoError(f"etape pending inconnue: {stage}")
    pending = _read_pending(data_path)
    demo = pending.get("demo", {})
    if stage == "pick_deg" and "pick_approach_deg" not in demo:
        raise DemoError("capture d'abord l'approche avec --capture-approach")
    if stage in demo:
        raise DemoError(f"{stage} est deja capture; transaction non modifiee")
    demo[stage] = list(validate_angles(angles, stage))

    if stage == "pick_approach_deg":
        _atomic_json_write(_pending_path(data_path), pending)
        return False

    if data_path.exists():
        data = load_dataset(data_path)
        if data["calibration"] != pending["calibration"]:
            raise DemoError("la calibration du dataset a change pendant la transaction")
    else:
        data = {"schema_version": 1, "input_space": "base_xy_m",
                "units": {"position": "m", "joint_angles": "deg", "pixel": "px"},
                "calibration": pending["calibration"],
                "safety": pending.get("safety", {
                    "min_demo_separation_m": DEFAULT_MIN_DEMO_SEPARATION_M,
                    "max_nearest_demo_distance_m": DEFAULT_MAX_NEAREST_DEMO_DISTANCE_M,
                }), "demonstrations": []}
    candidate = dict(data)
    candidate["demonstrations"] = list(data["demonstrations"]) + [demo]
    validate_dataset(candidate, require_ready=False)
    verify_calibration(candidate)
    _atomic_json_write(data_path, candidate)
    _pending_path(data_path).unlink()
    return True


def read_robot_angles(host: str, port: int) -> tuple[float, ...]:
    """Read-only encoder capture using the project's existing Bridge."""
    scripts_dir = Path(__file__).resolve().parent
    sys.path.insert(0, str(scripts_dir))
    from pick_and_place_real import Bridge  # type: ignore
    bridge = Bridge(host, port)
    try:
        return validate_angles(bridge.get_angles(), "angles robot")
    finally:
        bridge.sock.close()


def read_fresh_robot_angles_ros2(timeout_s: float) -> tuple[float, ...]:
    """Request and parse a fresh ``get_angles`` response through ROS 2."""
    if timeout_s <= 0:
        raise DemoError("joint-timeout doit etre > 0")
    try:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from std_msgs.msg import String
    except ImportError as exc:
        raise DemoError("rclpy et std_msgs sont requis pour --read-current-angles") from exc
    owns_context = not rclpy.ok()
    if owns_context:
        rclpy.init(args=None)
    node = rclpy.create_node(f"adaptive_pick_capture_{os.getpid()}")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    fresh: list[tuple[float, ...]] = []
    request_sent = False
    try:
        def callback(message: Any) -> None:
            if not request_sent:
                return
            for line in str(message.data).splitlines():
                if not line.strip().upper().startswith("ANGLES:"):
                    continue
                try:
                    values = ast.literal_eval(line.split(":", 1)[1].strip())
                    fresh.append(validate_angles(values, "reponse ANGLES fraiche"))
                except (SyntaxError, ValueError, DemoError):
                    pass

        node.create_subscription(String, "/from_robot", callback, 50)
        publisher = node.create_publisher(String, "/to_robot", 10)
        deadline = time.monotonic() + timeout_s
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        if publisher.get_subscription_count() == 0:
            raise DemoError("aucun abonne /to_robot; bridge_tour tourne-t-il?")
        command = String()
        command.data = json.dumps({"action": "get_angles"})
        request_sent = True
        publisher.publish(command)
        last_publish = time.monotonic()
        while not fresh and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
            if time.monotonic() - last_publish >= 0.75:
                publisher.publish(command)
                last_publish = time.monotonic()
        if not fresh:
            raise DemoError("aucune reponse ANGLES fraiche sur /from_robot")
        return fresh[-1]
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        if owns_context and rclpy.ok():
            rclpy.shutdown()


class Ros2Bridge:
    """Bridge-compatible native ROS 2 client for the launch-owned Pi link."""

    def __init__(self, timeout_s: float = 8.0, require_command_link: bool = True):
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from sensor_msgs.msg import JointState
            from std_msgs.msg import String
        except ImportError as exc:
            raise ExecutionError("rclpy, sensor_msgs et std_msgs sont requis") from exc
        self._rclpy = rclpy
        self._String = String
        self._owns_context = not rclpy.ok()
        if self._owns_context:
            rclpy.init(args=None)
        self.node = rclpy.create_node(f"adaptive_pick_command_{os.getpid()}")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.publisher = self.node.create_publisher(String, "/to_robot", 10)
        self.node.create_subscription(String, "/from_robot", self._on_response, 50)
        self.node.create_subscription(JointState, "/joint_states", self._on_joint_state, 50)
        self._responses: queue.Queue[str] = queue.Queue()
        self._joint_angles_deg: tuple[float, ...] | None = None
        self._joint_sequence = 0
        self._timeout_s = timeout_s
        self._last_grip = 0.0
        self._spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._spin_thread.start()
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            command_ready = (not require_command_link or
                             (self.publisher.get_subscription_count() > 0 and
                              self.node.count_publishers("/from_robot") > 0))
            if command_ready and self.node.count_publishers("/joint_states") > 0:
                break
            time.sleep(0.05)
        else:
            self.close()
            raise ExecutionError("topics ROS 2 requis absents (/joint_states, /to_robot, /from_robot)")

    def _on_response(self, message: Any) -> None:
        for line in str(message.data).splitlines():
            if line.strip():
                self._responses.put(line.strip())

    def _on_joint_state(self, message: Any) -> None:
        positions = message.position
        if len(positions) >= 6:
            try:
                angles = tuple(math.degrees(float(value)) for value in positions[:6])
                if all(math.isfinite(value) for value in angles):
                    self._joint_angles_deg = angles
                    self._joint_sequence += 1
            except (TypeError, ValueError):
                pass

    @staticmethod
    def _expected_prefix(action: str) -> str:
        return {
            "send_angles": "OK: angles",
            "pro_gripper_angle": "OK: pro gripper angle",
            "get_pro_gripper_status": "PRO_GRIPPER_STATUS:",
        }.get(action, "OK:")

    def send(self, obj: dict[str, Any]) -> str:
        action = str(obj.get("action", ""))
        expected = self._expected_prefix(action)
        while True:
            try:
                self._responses.get_nowait()
            except queue.Empty:
                break
        message = self._String()
        message.data = json.dumps(obj)
        self.publisher.publish(message)
        deadline = time.monotonic() + self._timeout_s
        while time.monotonic() < deadline:
            try:
                response = self._responses.get(timeout=min(0.1, max(0.0, deadline - time.monotonic())))
            except queue.Empty:
                continue
            if response.startswith("ERROR:"):
                raise ExecutionError(response)
            if response.startswith(expected):
                validate_command_response(obj, response)
                return response
            # ANGLES from joint_sync and unrelated dashboard replies are ignored.
        raise ExecutionError(f"pas de reponse {expected!r} sur /from_robot")

    def joint_sequence(self) -> int:
        return self._joint_sequence

    def wait_for_angles(self, target_deg: Sequence[float], after_sequence: int,
                        tolerance_deg: float, timeout_s: float) -> tuple[float, ...]:
        target = validate_angles(target_deg, "cible mouvement")
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            current = self._joint_angles_deg
            if current is not None and self._joint_sequence > after_sequence:
                errors = [abs((current[i] - target[i] + 180.0) % 360.0 - 180.0)
                          for i in range(6)]
                if max(errors) <= tolerance_deg:
                    return current
            time.sleep(0.05)
        current_text = ([round(v, 1) for v in self._joint_angles_deg]
                        if self._joint_angles_deg is not None else "aucun /joint_states")
        raise ExecutionError(f"pose non atteinte en {timeout_s:.1f}s "
                             f"(tolerance {tolerance_deg:.1f} deg, courant={current_text})")

    def grip(self, obj: dict[str, Any]) -> str:
        remaining = 1.6 - (time.monotonic() - self._last_grip)
        if remaining > 0:
            time.sleep(remaining)
        response = self.send(obj)
        self._last_grip = time.monotonic()
        return response

    def gripper_status(self) -> int | None:
        import re
        response = self.grip({"action": "get_pro_gripper_status", "gripper_id": GRIPPER_ID})
        match = re.search(r"-?\d+", response.split("PRO_GRIPPER_STATUS:", 1)[-1])
        return int(match.group()) if match else None

    def close(self) -> None:
        self.executor.shutdown(timeout_sec=2.0)
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        if self._owns_context and self._rclpy.ok():
            self._rclpy.shutdown()
        self._spin_thread.join(timeout=2.0)


def solve_live_marker_extrinsic(marker_centers: dict[int, Sequence[float]],
                                marker_world: dict[int, Sequence[float]],
                                K: Any, dist: Any, max_rms_px: float = 3.0) -> tuple[Any, float, tuple[int, ...]]:
    """Solve and gate a fresh ArUco camera pose from all four table markers."""
    required = tuple(sorted(int(marker_id) for marker_id in marker_world))
    if len(required) < 4 or any(marker_id not in marker_centers for marker_id in required):
        seen = sorted(set(marker_centers).intersection(marker_world))
        raise DemoError(f"extrinsèque live requiert les 4 ArUco {list(required)}; vus={seen}")
    try:
        from live_aruco_geometry import PoseEstimationError, estimate_live_extrinsic
    except ImportError as exc:
        raise DemoError("live_aruco_geometry est requis pour l'extrinsèque live") from exc
    try:
        estimate = estimate_live_extrinsic(
            marker_centers, marker_world, K, dist,
            required_ids=required, min_markers=4,
            max_rms_px=max_rms_px, camera_z_range_m=(0.30, 2.0))
    except PoseEstimationError as exc:
        raise DemoError(str(exc)) from exc
    return estimate.T_world_cam, estimate.rms_reprojection_px, estimate.marker_ids


def decode_ros_image_bgr8(message: Any) -> Any:
    """Decode a sensor_msgs/Image without cv_bridge/NumPy ABI coupling.

    Row padding from ``step`` is honored.  The launch normally publishes
    ``bgr8``; the other common 8-bit encodings are handled explicitly.
    """
    try:
        import numpy as np
    except ImportError as exc:
        raise DemoError("numpy est requis pour décoder sensor_msgs/Image") from exc
    height, width, step = int(message.height), int(message.width), int(message.step)
    encoding = str(message.encoding).lower()
    channels = {"mono8": 1, "bgr8": 3, "rgb8": 3, "bgra8": 4, "rgba8": 4}.get(encoding)
    if height <= 0 or width <= 0 or channels is None:
        raise DemoError(f"image ROS invalide/encoding non supporté: {width}x{height} {encoding}")
    packed_width = width * channels
    if step < packed_width:
        raise DemoError(f"step image {step} < ligne utile {packed_width}")
    raw = memoryview(message.data)
    required = height * step
    if raw.nbytes < required:
        raise DemoError(f"buffer image tronqué: {raw.nbytes} < {required}")
    rows = np.frombuffer(raw, dtype=np.uint8, count=required).reshape(height, step)
    frame = rows[:, :packed_width].reshape(height, width, channels)
    if encoding == "mono8":
        frame = np.repeat(frame, 3, axis=2)
    elif encoding == "rgb8":
        frame = frame[:, :, ::-1]
    elif encoding == "bgra8":
        frame = frame[:, :, :3]
    elif encoding == "rgba8":
        frame = frame[:, :, [2, 1, 0]]
    return np.ascontiguousarray(frame)


def observe_ball_pixel(image_topic: str, samples: int, timeout_s: float, sample_spread_px: float,
                       hsv_lo: Sequence[int], hsv_hi: Sequence[int],
                       expected_resolution: tuple[int, int], intrinsic_path: Path,
                       extrinsic_path: Path, workspace_path: Path,
                       require_markers: bool = True) -> LiveObservation:
    """Robust live HSV observation from the launch-owned Arducam image topic."""
    if samples < 3:
        raise DemoError("--samples doit etre >= 3")
    if timeout_s <= 0 or sample_spread_px <= 0:
        raise DemoError("live-timeout et sample-spread-px doivent etre > 0")
    try:
        import cv2
        import numpy as np
        import rclpy
        import yaml
        from rclpy.executors import SingleThreadedExecutor
        from sensor_msgs.msg import Image
    except ImportError as exc:
        raise DemoError("OpenCV, numpy, rclpy et sensor_msgs sont requis pour la capture live") from exc
    try:
        cv_major = int(str(cv2.__version__).split(".", 1)[0])
    except (TypeError, ValueError):
        cv_major = 0
    if cv_major < 5:
        raise DemoError(f"OpenCV {cv2.__version__} ArUco est instable ici; lancer avec "
                        f"{REPO / '.venv/bin/python'}")
    lo = np.asarray([int(value) for value in hsv_lo], dtype=np.uint8)
    hi = np.asarray([int(value) for value in hsv_hi], dtype=np.uint8)
    if lo.shape != (3,) or hi.shape != (3,) or np.any(lo > hi):
        raise DemoError("bornes HSV invalides")

    detections: list[tuple[float, float]] = []
    resolution_errors: list[tuple[int, int]] = []
    workspace_data = yaml.safe_load(workspace_path.read_text(encoding="utf-8"))
    marker_world = {int(key): tuple(float(v) for v in value)
                    for key, value in workspace_data["markers"].items()}
    required_marker_ids = set(marker_world)
    marker_observations: dict[int, list[tuple[float, float]]] = {
        marker_id: [] for marker_id in required_marker_ids}
    K, dist = load_intrinsic_parameters(intrinsic_path)
    from live_aruco_geometry import detect_marker_centres
    owns_context = not rclpy.ok()
    if owns_context:
        rclpy.init(args=None)
    node = rclpy.create_node(f"adaptive_pick_camera_{os.getpid()}")
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def on_image(message: Any) -> None:
        try:
            frame = decode_ros_image_bgr8(message)
            height, width = frame.shape[:2]
            if (width, height) != expected_resolution:
                resolution_errors.append((width, height))
                return
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lo, hi)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            best: tuple[float, tuple[float, float]] | None = None
            for contour in contours:
                area = float(cv2.contourArea(contour))
                perimeter = float(cv2.arcLength(contour, True))
                circularity = 4.0 * math.pi * area / (perimeter * perimeter) if perimeter else 0.0
                moments = cv2.moments(contour)
                if not (200.0 <= area <= 5000.0 and circularity >= 0.20 and moments["m00"] > 0):
                    continue
                u = float(moments["m10"] / moments["m00"])
                v = float(moments["m01"] / moments["m00"])
                if u < 20 or u > width - 20 or v < 20 or v > height - 20:
                    continue
                if best is None or area > best[0]:
                    best = (area, (u, v))
            if best is not None and len(detections) < samples:
                detections.append(best[1])
            if require_markers:
                centers = detect_marker_centres(frame, target_ids=tuple(required_marker_ids))
                for marker_id, center in centers.items():
                    marker_observations[marker_id].append(center)
        except (KeyError, TypeError, ValueError, DemoError):
            return

    try:
        node.create_subscription(Image, image_topic, on_image, 10)
        deadline = time.monotonic() + timeout_s
        def markers_ready() -> bool:
            return (not require_markers or
                    all(len(marker_observations[marker_id]) >= 3
                        for marker_id in required_marker_ids))

        while ((len(detections) < samples or not markers_ready()) and not resolution_errors
               and time.monotonic() < deadline):
            executor.spin_once(timeout_sec=0.1)
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        if owns_context and rclpy.ok():
            rclpy.shutdown()
    if resolution_errors:
        raise DemoError(f"resolution image {resolution_errors[-1]} != intrinsèque "
                        f"{expected_resolution}; projection refusée")
    if len(detections) < samples:
        raise DemoError(f"balle non detectee de facon stable: {len(detections)}/{samples} frames")
    points = np.asarray(detections, dtype=np.float64)
    centre = np.median(points, axis=0)
    spread = float(np.max(np.linalg.norm(points - centre, axis=1)))
    if spread > sample_spread_px:
        raise DemoError(f"detection balle instable: dispersion {spread:.1f}px > {sample_spread_px:.1f}px")
    pixel = (float(centre[0]), float(centre[1]))
    if not require_markers:
        return LiveObservation(pixel, (), (), 0.0)
    weak = {marker_id: len(values) for marker_id, values in marker_observations.items()
            if len(values) < 3}
    if weak:
        raise DemoError(f"ArUco live insuffisants (3 observations/ID requises): {weak}")
    median_centers = {
        marker_id: tuple(np.median(np.asarray(values, dtype=np.float64), axis=0))
        for marker_id, values in marker_observations.items() if values
    }
    marker_spreads = {
        marker_id: float(np.max(np.linalg.norm(
            np.asarray(values, dtype=np.float64) - np.asarray(median_centers[marker_id]), axis=1)))
        for marker_id, values in marker_observations.items()
    }
    max_marker_spread = max(marker_spreads.values(), default=math.inf)
    if max_marker_spread > 3.0:
        raise DemoError(f"ArUco instables: dispersion {max_marker_spread:.1f}px > 3.0px")

    marker_z_values = [float(value[2]) for value in marker_world.values()]
    if max(marker_z_values) - min(marker_z_values) > 0.005:
        raise DemoError("les 4 ArUco doivent definir un meme plan de table")
    table_z = float(np.median(np.asarray(marker_z_values, dtype=np.float64)))
    T_world_cam = load_extrinsic_transform(extrinsic_path)
    projected_markers = [
        project_raw_pixel_to_plane(intrinsic_path, median_centers[marker_id],
                                   T_world_cam, table_z)[:2]
        for marker_id in sorted(required_marker_ids)
    ]
    polygon = convex_hull(projected_markers)
    if len(polygon) != 4:
        raise DemoError("frontière ArUco invalide: quadrilatère convexe requis")
    area = 0.5 * abs(sum(
        polygon[index][0] * polygon[(index + 1) % 4][1]
        - polygon[(index + 1) % 4][0] * polygon[index][1]
        for index in range(4)
    ))
    edges = [math.dist(polygon[index], polygon[(index + 1) % 4]) for index in range(4)]
    if area < 0.020 or min(edges) < 0.10 or max(edges) > 1.0:
        raise DemoError(f"frontière ArUco non plausible: aire={area:.3f}m², "
                        f"arêtes={[round(edge, 3) for edge in edges]}m")
    return LiveObservation(pixel, tuple(polygon), tuple(sorted(required_marker_ids)),
                           max_marker_spread)


def prediction_plan(prediction: Prediction, pixel_uv: Sequence[float], speed: int,
                    open_angle: int, grasp_angle: int) -> dict[str, Any]:
    if not 1 <= speed <= 100:
        raise DemoError("speed doit etre dans [1,100]")
    if not 0 <= open_angle <= 100 or not 0 <= grasp_angle <= 100:
        raise DemoError("angles pince doivent etre dans [0,100]")
    return {
        "mode": "dry-run",
        "input": {"pixel_uv": list(_finite_pair(tuple(pixel_uv), "pixel_uv")),
                  "base_xy_m": list(prediction.base_xy_m)},
        "interpolation": {"method": "inverse_distance_weighting", "weights": list(prediction.weights)},
        "steps": [
            {"action": "gripper", "angle": open_angle},
            {"action": "send_angles", "pose": "pick_approach",
             "angles_deg": list(prediction.pick_approach_deg), "speed": speed},
            {"action": "send_angles", "pose": "pick",
             "angles_deg": list(prediction.pick_deg), "speed": max(10, speed // 2)},
            {"action": "gripper", "angle": grasp_angle},
            {"action": "require_gripper_status", "status": 2},
            {"action": "send_angles", "pose": "lift_to_pick_approach",
             "angles_deg": list(prediction.pick_approach_deg), "speed": speed},
        ],
        "result": "balle tenue a la pose d'approche; aucune depose automatique",
    }


def execute_pick(plan: dict[str, Any], host: str, port: int, settle_s: float,
                 robot_via: str = "ros2", joint_tolerance_deg: float = 3.0,
                 motion_timeout_s: float = 15.0,
                 live_replan: Callable[[], dict[str, Any]] | None = None,
                 adaptive_tolerance_m: float = 0.010,
                 max_replans: int = 3) -> None:
    """Execute only the fixed safe sequence represented by ``prediction_plan``."""
    if not sys.stdin.isatty():
        raise ExecutionError("confirmation interactive requise (stdin doit etre un terminal)")
    phrase = "DEPLACER LA BALLE"
    answer = input(f"Le bras VA BOUGER. Zone degagee, arret d'urgence accessible.\n"
                   f"Tape exactement {phrase!r} pour executer: ").strip()
    if answer != phrase:
        raise ExecutionError("execution annulee: confirmation incorrecte")

    topic_bridge = Ros2Bridge(timeout_s=min(8.0, motion_timeout_s),
                             require_command_link=(robot_via == "ros2"))
    if robot_via == "ros2":
        bridge: Any = topic_bridge
    elif robot_via == "socket":
        scripts_dir = Path(__file__).resolve().parent
        sys.path.insert(0, str(scripts_dir))
        from pick_and_place_real import Bridge  # type: ignore
        try:
            bridge = Bridge(host, port)
        except Exception:
            topic_bridge.close()
            raise
    else:
        raise ExecutionError(f"robot-via inconnu: {robot_via}")
    try:
        def checked_move(step: dict[str, Any]) -> None:
            sequence = topic_bridge.joint_sequence()
            print(bridge.send({"action": "send_angles", "angles": step["angles_deg"],
                               "speed": step["speed"]}))
            reached = topic_bridge.wait_for_angles(step["angles_deg"], sequence,
                                                   joint_tolerance_deg, motion_timeout_s)
            print(f"pose verifiee /joint_states: {[round(value, 1) for value in reached]}")
            if settle_s:
                time.sleep(settle_s)

        def checked_gripper_angle(angle: int) -> None:
            command = {"action": "pro_gripper_angle", "angle": angle,
                       "gripper_id": GRIPPER_ID}
            response = bridge.grip(command)
            validate_command_response(command, response)
            print(response)

        checked_gripper_angle(plan["steps"][0]["angle"])
        checked_move(plan["steps"][1])  # must pass before descent

        active_plan = plan
        if live_replan is not None:
            for attempt in range(max_replans + 1):
                fresh_plan = live_replan()  # any safety rejection aborts while still at approach
                old_xy = active_plan["input"]["base_xy_m"]
                new_xy = fresh_plan["input"]["base_xy_m"]
                shift = math.dist(old_xy, new_xy)
                print(f"replan live {attempt + 1}: deplacement cible={shift*1000:.1f} mm")
                if shift <= adaptive_tolerance_m:
                    active_plan = fresh_plan
                    print("cible stable: descente autorisee")
                    break
                if attempt >= max_replans:
                    raise ExecutionError("cible toujours mobile apres le maximum de replans; "
                                         "bras laisse a l'approche")
                checked_move(fresh_plan["steps"][1])
                active_plan = fresh_plan

        try:
            checked_move(active_plan["steps"][2])
            checked_gripper_angle(active_plan["steps"][3]["angle"])
            status = bridge.gripper_status()
            print(f"statut pince: {status} ({STATUS_TEXT.get(status, 'inconnu')})")
            if status != 2:
                raise ExecutionError("objet non saisi (statut pince != 2)")
            checked_move(active_plan["steps"][5])
            lifted_status = bridge.gripper_status()
            print(f"statut apres levee: {lifted_status} "
                  f"({STATUS_TEXT.get(lifted_status, 'inconnu')})")
            if lifted_status != 2:
                raise ExecutionError("objet perdu pendant la levee (statut pince != 2)")
        except Exception:
            # Once descent has started, every failure attempts a joint-space
            # retreat.  Never open the gripper in this fail-safe path.
            print("echec apres debut de descente: retour de secours vers l'approche")
            try:
                checked_move(active_plan["steps"][5])
            except Exception as retreat_error:
                print(f"ECHEC DU RETRAIT: {retreat_error}", file=sys.stderr)
            raise
        print("balle saisie et soulevee a l'approche; aucune depose/relache automatique")
    finally:
        if bridge is not topic_bridge:
            bridge.sock.close()
        topic_bridge.close()


def _capture_angles(args: argparse.Namespace) -> tuple[float, ...]:
    selected = int(args.angles is not None) + int(args.read_robot) + int(args.read_current_angles)
    if selected > 1:
        raise DemoError("choisis une seule source: --angles, --read-current-angles ou --read-robot")
    if args.angles is not None:
        return validate_angles(args.angles)
    if args.read_current_angles:
        return read_fresh_robot_angles_ros2(args.joint_timeout)
    if args.read_robot:
        return read_robot_angles(args.host, args.port)
    raise DemoError("capture requiert --angles, --read-current-angles ou --read-robot")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--begin-demo", nargs=2, type=float, metavar=("U", "V"))
    action.add_argument("--begin-live", action="store_true",
                        help="detecte la balle sur /camera/image_raw puis ouvre une transaction")
    action.add_argument("--capture-approach", action="store_true")
    action.add_argument("--capture-pick", action="store_true")
    action.add_argument("--plan", nargs=2, type=float, metavar=("U", "V"))
    action.add_argument("--plan-live", action="store_true",
                        help="detecte la balle live puis genere le plan dry-run")
    action.add_argument("--show", action="store_true")
    action.add_argument("--discard-pending", action="store_true")
    parser.add_argument("--data", type=Path, default=DEFAULT_DATA)
    parser.add_argument("--name", help="nom libre de la nouvelle demonstration")
    parser.add_argument("--angles", nargs=6, type=float, metavar=("J1", "J2", "J3", "J4", "J5", "J6"))
    parser.add_argument("--read-current-angles", "--read-joint-states",
                        dest="read_current_angles", action="store_true",
                        help="demande get_angles frais via /to_robot et attend ANGLES sur /from_robot")
    parser.add_argument("--read-robot", action="store_true", help="lit get_angles via Bridge (aucun mouvement)")
    parser.add_argument("--host", default="192.168.223.59")
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--joint-timeout", type=float, default=3.0)
    parser.add_argument("--image-topic", default="/camera/image_raw")
    parser.add_argument("--samples", type=int, default=6)
    parser.add_argument("--live-timeout", type=float, default=8.0)
    parser.add_argument("--sample-spread-px", type=float, default=6.0)
    parser.add_argument("--hsv-lo", nargs=3, type=int, default=[20, 170, 70], metavar=("H", "S", "V"))
    parser.add_argument("--hsv-hi", nargs=3, type=int, default=[35, 255, 255], metavar=("H", "S", "V"))
    parser.add_argument("--intrinsic", type=Path, default=DEFAULT_INTRINSIC,
                        help="intrinseque Arducam cam_0 (utilise seulement au premier demo)")
    parser.add_argument("--extrinsic", type=Path, default=DEFAULT_EXTRINSIC,
                        help="extrinsèque eye-to-hand fixe caméra->base")
    parser.add_argument("--workspace", type=Path, default=DEFAULT_WORKSPACE,
                        help="IDs ArUco et hauteur du plan de table")
    parser.add_argument("--projection-z", type=float, default=DEFAULT_PROJECTION_Z_M,
                        help="plan du centroide de l'objet en base, m (balle tennis: 0.0335)")
    parser.add_argument("--workspace-margin", type=float, default=0.025)
    parser.add_argument("--min-demo-separation", type=float,
                        default=DEFAULT_MIN_DEMO_SEPARATION_M)
    parser.add_argument("--max-nearest-demo-distance", type=float,
                        default=DEFAULT_MAX_NEAREST_DEMO_DISTANCE_M)
    parser.add_argument("--idw-power", type=float, default=2.0)
    parser.add_argument("--speed", type=int, default=20)
    parser.add_argument("--open-angle", type=int, default=100)
    parser.add_argument("--grasp-angle", type=int, default=20)
    parser.add_argument("--settle", type=float, default=3.0)
    parser.add_argument("--joint-tolerance", type=float, default=3.0)
    parser.add_argument("--motion-timeout", type=float, default=15.0)
    parser.add_argument("--adaptive-tolerance", type=float, default=0.010,
                        help="deplacement XY live tolere avant replan, m")
    parser.add_argument("--max-replans", type=int, default=3)
    parser.add_argument("--execute", action="store_true",
                        help="autorise le mouvement reel apres confirmation interactive")
    parser.add_argument("--robot-via", choices=("ros2", "socket"), default="ros2",
                        help="topics ROS 2 natifs par defaut; socket seulement si choisi explicitement")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    data_path = args.data.expanduser().resolve()
    if args.execute and not args.plan_live:
        raise DemoError("execution reelle autorisee uniquement avec --plan-live; --plan U V est diagnostic")
    if args.settle < 0 or args.joint_tolerance <= 0 or args.motion_timeout <= 0:
        raise DemoError("settle doit etre >=0; joint-tolerance et motion-timeout >0")
    if args.adaptive_tolerance <= 0 or args.max_replans < 0:
        raise DemoError("adaptive-tolerance doit etre >0 et max-replans >=0")

    begin_pixel = args.begin_demo
    begin_observation: LiveObservation | None = None
    begin_data: dict[str, Any] | None = None
    if args.begin_live:
        if data_path.exists():
            begin_data = load_dataset(data_path)
            begin_intrinsic = _resolve_record(begin_data["calibration"]["intrinsic"])
            begin_extrinsic = _resolve_record(begin_data["calibration"]["extrinsic"])
            begin_workspace = _resolve_record(begin_data["calibration"]["workspace_markers"])
        else:
            begin_intrinsic = args.intrinsic.expanduser().resolve()
            begin_extrinsic = args.extrinsic.expanduser().resolve()
            begin_workspace = args.workspace.expanduser().resolve()
            begin_data = new_dataset(
                args.intrinsic, args.extrinsic, args.workspace, args.projection_z,
                args.workspace_margin, args.min_demo_separation,
                args.max_nearest_demo_distance)
        begin_observation = observe_ball_pixel(
            args.image_topic, args.samples, args.live_timeout, args.sample_spread_px,
            args.hsv_lo, args.hsv_hi, intrinsic_resolution(begin_intrinsic),
            begin_intrinsic, begin_extrinsic, begin_workspace, require_markers=True)
        begin_pixel = begin_observation.pixel_uv
        print(f"balle live stable: pixel=({begin_pixel[0]:.1f}, {begin_pixel[1]:.1f})")
        print(f"extrinsèque eye-to-hand fixe + frontière ArUco live: "
              f"IDs={list(begin_observation.marker_ids)}, "
              f"dispersion={begin_observation.max_marker_spread_px:.2f}px")
    if begin_pixel is not None:
        live_xyz = None
        live_localization = None
        if begin_observation is not None:
            fixed_transform = load_extrinsic_transform(begin_extrinsic)
            live_xyz = project_pixel_with_extrinsic(
                begin_data, begin_pixel, fixed_transform,
                begin_observation.workspace_base_xy)
            live_localization = {
                "source": "eye_to_hand+aruco_boundary_live",
                "marker_ids": list(begin_observation.marker_ids),
                "max_marker_spread_px": begin_observation.max_marker_spread_px,
                "workspace_base_xy": [list(point) for point in
                                      begin_observation.workspace_base_xy],
            }
        pending = begin_demo(data_path, begin_pixel, args.name, {
            "intrinsic": args.intrinsic, "extrinsic": args.extrinsic,
            "workspace": args.workspace, "projection_z_m": args.projection_z,
            "workspace_margin_m": args.workspace_margin,
            "min_demo_separation_m": args.min_demo_separation,
            "max_nearest_demo_distance_m": args.max_nearest_demo_distance,
        }, localized_xyz=live_xyz, localization=live_localization,
           workspace_polygon=(begin_observation.workspace_base_xy
                              if begin_observation is not None else None))
        ball = pending["demo"]["ball"]
        print(f"transaction ouverte: {_pending_path(data_path)}")
        print(f"pixel={ball['pixel_uv']} -> base XYZ={ball['base_xyz_m']} m")
        print("deplace le bras a l'approche puis utilise --capture-approach")
        return 0
    if args.capture_approach or args.capture_pick:
        angles = _capture_angles(args)
        stage = "pick_approach_deg" if args.capture_approach else "pick_deg"
        committed = capture_pending(data_path, stage, angles)
        print(f"{stage} capture: {[round(value, 2) for value in angles]}")
        print(f"demonstration validee et ajoutee a {data_path}" if committed
              else "approche sauvegardee dans la transaction; place maintenant le bras au pick")
        return 0
    if args.discard_pending:
        pending_path = _pending_path(data_path)
        if not pending_path.exists():
            raise DemoError("aucune transaction pending a supprimer")
        answer = input(f"Supprimer la transaction incomplete {pending_path}? [oui/N] ").strip().lower()
        if answer != "oui":
            raise DemoError("suppression annulee")
        pending_path.unlink()
        print("transaction incomplete supprimee")
        return 0
    if args.show:
        data = load_dataset(data_path)
        demos = validate_dataset(data, require_ready=False)
        print(json.dumps({"data": str(data_path), "demonstrations": len(demos),
                          "ready": len(demos) >= 3 and len(convex_hull(d["xy"] for d in demos)) >= 3,
                          "base_xy_m": [list(d["xy"]) for d in demos]}, indent=2))
        return 0

    data = load_dataset(data_path, require_ready=True)
    plan_pixel = args.plan
    plan_observation: LiveObservation | None = None
    if args.plan_live:
        plan_intrinsic = _resolve_record(data["calibration"]["intrinsic"])
        plan_extrinsic = _resolve_record(data["calibration"]["extrinsic"])
        plan_workspace = _resolve_record(data["calibration"]["workspace_markers"])
        plan_observation = observe_ball_pixel(
            args.image_topic, args.samples, args.live_timeout, args.sample_spread_px,
            args.hsv_lo, args.hsv_hi, intrinsic_resolution(plan_intrinsic),
            plan_intrinsic, plan_extrinsic, plan_workspace, require_markers=True)
        plan_pixel = plan_observation.pixel_uv
        print(f"balle live stable: pixel=({plan_pixel[0]:.1f}, {plan_pixel[1]:.1f})")
        print(f"extrinsèque eye-to-hand fixe + frontière ArUco live: "
              f"IDs={list(plan_observation.marker_ids)}, "
              f"dispersion={plan_observation.max_marker_spread_px:.2f}px")
    xyz = (project_pixel_with_extrinsic(
               data, plan_pixel, load_extrinsic_transform(plan_extrinsic),
               plan_observation.workspace_base_xy)
           if plan_observation is not None else pixel_to_calibrated_base(data, plan_pixel))
    prediction = interpolate(data, xyz[:2], power=args.idw_power)
    plan = prediction_plan(prediction, plan_pixel, args.speed, args.open_angle, args.grasp_angle)
    print(json.dumps(plan, indent=2, ensure_ascii=False))
    if args.execute:
        validate_live_execution_dataset(data)
        replan = None
        if args.plan_live:
            def replan() -> dict[str, Any]:
                fresh_observation = observe_ball_pixel(
                    args.image_topic, args.samples, args.live_timeout,
                    args.sample_spread_px, args.hsv_lo, args.hsv_hi,
                    intrinsic_resolution(plan_intrinsic), plan_intrinsic,
                    plan_extrinsic, plan_workspace, require_markers=False)
                fresh_pixel = fresh_observation.pixel_uv
                # Same pick cycle: the calibrated eye-to-hand transform is fixed;
                # reuse the validated live boundary if the arm hides the tags.
                fresh_xyz = project_pixel_with_extrinsic(
                    data, fresh_pixel, load_extrinsic_transform(plan_extrinsic),
                    plan_observation.workspace_base_xy)
                fresh_prediction = interpolate(data, fresh_xyz[:2], power=args.idw_power)
                return prediction_plan(fresh_prediction, fresh_pixel, args.speed,
                                       args.open_angle, args.grasp_angle)
        execute_pick(plan, args.host, args.port, args.settle, args.robot_via,
                     args.joint_tolerance, args.motion_timeout, replan,
                     args.adaptive_tolerance, args.max_replans)
    else:
        print("DRY-RUN: aucune connexion robot, aucune commande envoyee. Ajouter --execute pour le reel.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (DemoError, ExecutionError) as exc:
        print(f"ERREUR: {exc}", file=sys.stderr)
        raise SystemExit(2)
