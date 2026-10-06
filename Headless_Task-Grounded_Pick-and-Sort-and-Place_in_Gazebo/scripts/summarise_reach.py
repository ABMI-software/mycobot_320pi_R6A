#!/usr/bin/env python3
"""Part 6.5 — the usable rectangle, margined, never the raw boundary."""
import csv

ok = [(float(r["x"]), float(r["y"])) for r in csv.DictReader(open("config/reach_map.csv"))
      if r["solved"] == "1" and r["elbow_up"] == "1"]
xs, ys = [p[0] for p in ok], [p[1] for p in ok]
print(f"solved points: {len(ok)}")
print(f"x range: {min(xs):.3f} .. {max(xs):.3f}")
print(f"y range: {min(ys):.3f} .. {max(ys):.3f}")
print(f"SAFE (5 mm margin): x {min(xs)+0.005:.3f} .. {max(xs)-0.005:.3f}, "
      f"y {min(ys)+0.005:.3f} .. {max(ys)-0.005:.3f}")
