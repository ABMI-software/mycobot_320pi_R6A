"""cv2.VideoCapture-compatible adapter for the Orbbec Astra S.

Uses the same shared-memory architecture as the `hand_control` LeRobot project:
the `oni_grabber` OpenNI2 binary runs in a separate process and writes RGB
frames to `/dev/shm/oni_color.rgb`. This class reads those frames. Keeping the
OpenNI2 driver out of the main Python process avoids conflicts with Wilor and
the segfault-on-teardown bug that primesense exhibits.

`open_orbbec()` will spawn `oni_grabber` automatically if no frames are coming
through. Pass `auto_spawn=False` if you'd rather run it yourself.

Adapted from `hand_control/scripts/hand_teleop_local.py` (Bantu / ABMI team).
"""
from __future__ import annotations

import os
import subprocess
import time
from pathlib import Path

import numpy as np


ONI_INFO = Path("/dev/shm/oni_info.txt")
ONI_COLOR = Path("/dev/shm/oni_color.rgb")
ONI_TICK = Path("/dev/shm/oni_tick.txt")

DEFAULT_SAMPLES_BIN = (
    "/home/genji/Downloads/Orbbec_OpenNI_v2.3.0.86-beta6_linux_release/"
    "OpenNI_2.3.0.86_202210111154_4c8f5aa4_beta6_linux_x64/"
    "OpenNI_2.3.0.86_202210111154_4c8f5aa4_beta6_linux/samples/bin"
)


class OrbbecCapture:
    """Reads RGB frames written by `oni_grabber` to /dev/shm."""

    def __init__(self, open_timeout: float = 5.0) -> None:
        self._w: int | None = None
        self._h: int | None = None
        self._last_tick = ""
        self._opened = False

        deadline = time.time() + open_timeout
        while not ONI_INFO.exists():
            if time.time() > deadline:
                raise RuntimeError(
                    f"{ONI_INFO} not found within {open_timeout}s — "
                    "is oni_grabber running? "
                    "Use open_orbbec(auto_spawn=True) to start it automatically."
                )
            time.sleep(0.02)

        text = ONI_INFO.read_text()
        import re
        cw_match = re.search(r"CW=(\d+)", text)
        ch_match = re.search(r"CH=(\d+)", text)
        if not (cw_match and ch_match):
            raise RuntimeError(f"Could not parse CW/CH from {ONI_INFO}: {text!r}")
        self._w = int(cw_match.group(1))
        self._h = int(ch_match.group(1))
        self._opened = True

    def isOpened(self) -> bool:
        return self._opened

    def read(self) -> tuple[bool, np.ndarray | None]:
        if not self._opened:
            return False, None
        # Wait for a new tick (new frame). If the tick doesn't advance within
        # ~1 s the grabber is likely dead — return failure rather than a stale
        # frame, so Wilor doesn't keep re-running on the same pixels.
        got_new = False
        for _ in range(500):  # 500 * 2ms = 1s max
            try:
                t = ONI_TICK.read_text().strip()
            except Exception:
                t = self._last_tick
            if t and t != self._last_tick:
                self._last_tick = t
                got_new = True
                break
            time.sleep(0.002)
        if not got_new:
            return False, None
        try:
            buf = ONI_COLOR.read_bytes()
        except Exception:
            return False, None
        expected = self._h * self._w * 3
        if len(buf) != expected:
            return False, None
        rgb = np.frombuffer(buf, np.uint8).reshape(self._h, self._w, 3)
        return True, rgb[:, :, ::-1].copy()  # RGB → BGR

    def set(self, *_args, **_kwargs) -> bool:
        return False

    def get(self, prop_id: int) -> float:
        import cv2
        if prop_id == cv2.CAP_PROP_FRAME_WIDTH:
            return float(self._w or 0)
        if prop_id == cv2.CAP_PROP_FRAME_HEIGHT:
            return float(self._h or 0)
        return 0.0

    def release(self) -> None:
        self._opened = False


def _find_oni_grabber() -> str | None:
    """Locate the oni_grabber binary shipped with the Orbbec OpenNI samples."""
    env_dir = os.environ.get("OPENNI2_REDIST")
    candidates = [
        env_dir and Path(env_dir) / "oni_grabber",
        Path(DEFAULT_SAMPLES_BIN) / "oni_grabber",
    ]
    for c in candidates:
        if c and Path(c).is_file() and os.access(c, os.X_OK):
            return str(c)
    return None


def _spawn_oni_grabber(no_ir: bool = True) -> subprocess.Popen | None:
    """Start oni_grabber in the background. Returns the process or None."""
    exe = _find_oni_grabber()
    if exe is None:
        return None
    # Clean stale shared-memory artifacts from previous runs
    for f in Path("/dev/shm").glob("oni_*"):
        try:
            f.unlink()
        except Exception:
            pass
    args = [exe]
    if no_ir:
        args.append("--no-ir")
    return subprocess.Popen(
        args,
        cwd=str(Path(exe).parent),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        # Detach from the parent's process group so Ctrl+C in the teleop
        # terminal doesn't also SIGINT oni_grabber. Also makes the grabber
        # survive if the teleop crashes or is killed uncleanly.
        start_new_session=True,
    )


def _oni_grabber_alive() -> bool:
    """True if an oni_grabber process is currently running."""
    try:
        import subprocess
        result = subprocess.run(["pgrep", "-f", "oni_grabber"],
                                capture_output=True, text=True)
        return result.returncode == 0 and result.stdout.strip() != ""
    except Exception:
        return False


def _tick_is_fresh(max_age_s: float = 1.0) -> bool:
    """True if /dev/shm/oni_tick.txt was modified within max_age_s."""
    try:
        return (time.time() - ONI_TICK.stat().st_mtime) < max_age_s
    except FileNotFoundError:
        return False


def open_orbbec(auto_spawn: bool = True, open_timeout: float = 8.0) -> OrbbecCapture:
    """Open the Astra via shared-memory, (re)spawning oni_grabber if needed.

    Detects and recovers from the common "stale /dev/shm from a previous run"
    case: if the shared files exist but oni_grabber is dead (or its tick is
    older than 1 s), we clean up and re-spawn before reading.
    """
    alive = _oni_grabber_alive()
    fresh = _tick_is_fresh() if ONI_INFO.exists() else False

    if alive and not fresh:
        # Stale grabber (writing to a tick that's not advancing); kill it so
        # the respawn doesn't result in two grabbers fighting over the device.
        import subprocess
        subprocess.run(["pkill", "-9", "-f", "oni_grabber"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(0.3)
        alive = False

    needs_spawn = not alive or not ONI_INFO.exists()

    if needs_spawn and auto_spawn:
        print("[orbbec] Spawning oni_grabber")
        proc = _spawn_oni_grabber()
        if proc is None:
            raise RuntimeError(
                f"oni_grabber binary not found. Looked in $OPENNI2_REDIST and "
                f"{DEFAULT_SAMPLES_BIN}. Install the Orbbec OpenNI samples or "
                f"start oni_grabber manually."
            )

    return OrbbecCapture(open_timeout=open_timeout)
