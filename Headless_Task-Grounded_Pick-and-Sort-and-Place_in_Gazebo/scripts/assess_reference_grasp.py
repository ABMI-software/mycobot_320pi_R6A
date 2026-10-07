#!/usr/bin/env python3
"""Part 3.4 verdict, read from the CSV, not the screen."""
import csv
import statistics
import sys

rows = list(csv.DictReader(open(sys.argv[1])))
lift = [r for r in rows if r["phase"] in ("lift", "transport")]
if not lift:
    sys.exit("no lift/transport samples — the run did not reach the lift phase")
dz = [float(r["dz"]) for r in lift]
bz = [float(r["bz"]) for r in lift]
grasp_bz = [float(r["bz"]) for r in rows if r["phase"] == "grasp"]
rise = max(bz) - (min(grasp_bz) if grasp_bz else min(bz))
print(f"samples in lift/transport : {len(dz)}")
print(f"dz mean / stdev / range   : {statistics.mean(dz):.4f} / "
      f"{statistics.pstdev(dz):.4f} / {max(dz) - min(dz):.4f} m")
print(f"object rise               : {rise:.4f} m")
held = (max(dz) - min(dz)) < 0.005 and rise > 0.05
print("VERDICT:", "GRASP HELD" if held else "NOT HELD")
sys.exit(0 if held else 1)
