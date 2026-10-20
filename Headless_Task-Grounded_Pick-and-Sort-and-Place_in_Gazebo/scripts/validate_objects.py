#!/usr/bin/env python3
"""Part 4.7 — validate config/objects.yaml before anything downstream reads it."""
import sys

import yaml

c = yaml.safe_load(open("config/objects.yaml"))
errs = []
for name, o in c["objects"].items():
    if o["bin"] not in c["bins"]:
        errs.append(f"{name}: bin {o['bin']} not declared")
    if o["shape"] == "box" and (not o.get("size") or any(v <= 0 for v in o["size"])):
        errs.append(f"{name}: invalid or placeholder size")
    if o["shape"] == "cylinder" and not (o.get("radius", 0) > 0 and o.get("length", 0) > 0):
        errs.append(f"{name}: invalid or placeholder cylinder dims")
    half = (o["size"][2] / 2) if o["shape"] == "box" else (o["length"] / 2)
    # grasp_dz is signed here (see config/objects.yaml header): a small
    # negative value targets slightly below centre, matching the reference
    # sim_sorting_grasp.py formula exactly. The invariant that must hold is
    # that the resulting absolute target stays strictly inside the object
    # (0 < half + grasp_dz < 2*half), not that grasp_dz itself is positive.
    target_from_base = half + o["grasp_dz"]
    if not (0.0 < target_from_base < 2 * half):
        errs.append(f"{name}: grasp_dz {o['grasp_dz']} places the target "
                    f"outside the object (half-height {half})")
    if not o["instruction"].strip():
        errs.append(f"{name}: empty instruction")
for bn, b in c["bins"].items():
    if b["inner_radius"] <= 0 or b["rim_z"] == 0:
        errs.append(f"{bn}: placeholder geometry not replaced")
    if not b.get("static", False):
        errs.append(f"{bn}: NOT static — see Part 4.4")
print("\n".join(errs) if errs else "objects.yaml OK")
sys.exit(1 if errs else 0)
