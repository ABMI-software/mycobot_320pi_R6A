#!/usr/bin/env python3
"""Bring the rosetta recorder to ACTIVE: configure, then activate.

  python3 scripts/recorder_lifecycle.py [--node /episode_recorder] [--timeout 60]

Replaces `sleep 8` + two `ros2 lifecycle set --no-daemon --spin-time 15`
calls, measured at 71 s per episode (MEASUREMENTS.md section 3). The
recorder's change_state service is reachable ~1 s after the process starts;
the CLI itself spent ~31 s per transition. This waits on the actual
condition (the service exists), calls it directly, checks each result and
the resulting state, and exits non-zero on any failure.
"""
import argparse
import sys
import time

import rclpy
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState


def call(node, client, request, timeout):
    fut = client.call_async(request)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    return fut.result()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--node", default="/episode_recorder")
    ap.add_argument("--timeout", type=float, default=60.0)
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("recorder_lifecycle")
    change = node.create_client(ChangeState, f"{args.node}/change_state")
    get = node.create_client(GetState, f"{args.node}/get_state")
    t0 = time.time()
    if not (change.wait_for_service(timeout_sec=args.timeout) and get.wait_for_service(timeout_sec=10)):
        print(f"FATAL: {args.node} lifecycle services not available after {args.timeout:.0f}s")
        return 2
    print(f"services up after {time.time() - t0:.1f}s")

    for tid, label, want in ((Transition.TRANSITION_CONFIGURE, "configure", "inactive"),
                             (Transition.TRANSITION_ACTIVATE, "activate", "active")):
        t = time.time()
        res = call(node, change, ChangeState.Request(transition=Transition(id=tid)), args.timeout)
        state = call(node, get, GetState.Request(), 10)
        label_now = state.current_state.label if state else "unknown"
        if not (res and res.success and label_now == want):
            print(f"FATAL: {label} failed (success={res.success if res else None}, state={label_now})")
            return 1
        print(f"{label}: ok in {time.time() - t:.1f}s, state={label_now}")
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
