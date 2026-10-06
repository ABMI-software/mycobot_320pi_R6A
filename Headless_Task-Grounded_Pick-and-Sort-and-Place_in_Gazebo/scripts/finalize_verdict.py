#!/usr/bin/env python3
"""The episode's FINAL verdict, written into its verdict file.

  python3 scripts/finalize_verdict.py <episode_dir>

verify_episode.py judges the motion (carried, placed, right bin). Three more
checks run after it -- frames complete (check_frames.py), target colour
visible (check_colour.py), no trajectory re-sends -- and until 2026-10-03 they
failed the episode's exit code WITHOUT changing grasp_meta.verdict.json,
which kept saying PASS. port_bag.py converts on that file, so a white-object
or frame-dropping recording that failed both attempts would have entered the
dataset. This rewrites the file: "verdict" becomes the final verdict,
"verdict_motion" keeps verify_episode.py's, "fail_reasons" lists every
failed check. Exit 0 only on a final PASS.
"""
import json
import sys
from pathlib import Path


def load(p):
    try:
        return json.loads(Path(p).read_text())
    except (OSError, ValueError):
        return None


def main():
    ep = Path(sys.argv[1])
    vpath = ep / "grasp_meta.verdict.json"
    v = load(vpath)
    if v is None:
        print("FAIL: no verdict file (the sequence did not complete)")
        return 1
    motion = v.get("verdict_motion", v["verdict"])
    frames, colour, meta = load(ep / "frames.json"), load(ep / "colour.json"), load(ep / "grasp_meta.json")
    resends = (meta or {}).get("trajectory_resends_total")

    reasons = []
    if motion != "PASS":
        reasons.append(f"MOTION_{motion}")
    if not (frames and frames.get("complete")):
        reasons.append("FRAMES_INCOMPLETE")
    if not (colour and colour.get("ok")):
        reasons.append("COLOUR_NOT_VISIBLE")
    if resends != 0:
        # a re-send changes the demonstration trajectory: retry instead
        reasons.append(f"RESENDS_{resends}")

    v.update(verdict_motion=motion, fail_reasons=reasons,
             frames=frames, colour=colour, trajectory_resends_total=resends,
             verdict="PASS" if not reasons else ("WRONG_BIN" if motion == "WRONG_BIN" else "FAIL"))
    vpath.write_text(json.dumps(v, indent=1))
    print(json.dumps({"verdict": v["verdict"], "verdict_motion": motion, "fail_reasons": reasons}))
    return 0 if v["verdict"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
