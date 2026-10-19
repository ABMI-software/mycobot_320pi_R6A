#!/usr/bin/env python3
"""Part 6.6 — reject colliding spawn layouts before they ever reach Gazebo."""
import math

# Sampler statistics, read by make_matrix.py to report the acceptance rate:
# a rejection sampler that barely finds layouts yields clustered, correlated
# distractor placements -- a diversity defect no clearance check catches.
STATS = {"layouts": 0, "draws": 0}


def validate_layout(obj_xy, bin_xy, min_sep, bin_keepout):
    """Return (ok, reason). obj_xy/bin_xy are {name: (x, y)}."""
    names = list(obj_xy)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            d = math.dist(obj_xy[a], obj_xy[b])
            if d < min_sep:
                return False, f"{a}/{b} separated by {d:.3f} m < {min_sep}"
    for o, p in obj_xy.items():
        for bn, q in bin_xy.items():
            d = math.dist(p, q)
            if d < bin_keepout:
                return False, f"{o} is {d:.3f} m from {bn} (keepout {bin_keepout})"
    return True, "ok"


def sample_distractors(target, target_xy, bin_xy, safe_rect,
                        min_sep, bin_keepout, others, rng, tries=8000, fits=None):
    """Place the three non-target objects at valid random positions.
    fits(name, (x, y)) -> bool, if given, must accept every object's
    position: make_matrix.py uses it to keep each FOOTPRINT on the board.
    The safe rectangle bounds centres only; a 22 mm-radius cylinder centred
    at its edge overhung the board by 4.5 mm (2026-10-04)."""
    x0, x1, y0, y1 = safe_rect
    STATS["layouts"] += 1
    for _ in range(tries):
        STATS["draws"] += 1
        layout = {target: target_xy}
        for name in others:
            layout[name] = (round(rng.uniform(x0, x1), 4),
                            round(rng.uniform(y0, y1), 4))
        if fits and not all(fits(n, p) for n, p in layout.items()):
            continue
        ok, _ = validate_layout(layout, bin_xy, min_sep, bin_keepout)
        if ok:
            return layout
    raise RuntimeError("could not find a valid layout — relax the rectangle "
                       "or reduce min_sep")
