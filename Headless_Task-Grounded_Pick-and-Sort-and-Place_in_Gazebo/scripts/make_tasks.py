#!/usr/bin/env python3
"""Part 5.4 — generate config/tasks.jsonl from config/objects.yaml.

Generated, not typed, so the two files cannot drift apart.
"""
import json

import yaml

c = yaml.safe_load(open("config/objects.yaml"))
order = ["red_cube", "blue_cube", "green_cylinder", "yellow_box"]
with open("config/tasks.jsonl", "w") as f:
    for i, name in enumerate(order):
        f.write(json.dumps({"task_index": i,
                            "task": c["objects"][name]["instruction"]}) + "\n")
print("wrote config/tasks.jsonl")
