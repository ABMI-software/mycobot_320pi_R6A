#!/usr/bin/env python3
"""Guarded weld commands for the per-object DetachableJoint plugins.

  python3 scripts/weld.py detach red_cube blue_cube ...     # release
  python3 scripts/weld.py attach red_cube                   # weld to link6

Each command waits, bounded, until the plugin's subscription to the topic is
connected (`Publisher.has_connections()`, Gazebo transport's equivalent of
ROS's get_subscription_count() > 0) and only then publishes. The `gz topic`
CLI used before advertised and published in one go: if the plugin's
subscription had not been discovered yet, the message went nowhere without
an error. That is the most likely cause of held-out episode 13's first
attempt (2026-10-03): an object still welded to link6 from spawn pinned the
arm and was dragged off the table. gz-sim 8's DetachableJoint has no option
not to attach on load, so the release cannot be avoided, only guarded.
"""
import sys
import threading
import time

from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node

_node = Node()
_state = {}                    # name -> last state the plugin reported
_changed = threading.Condition()
_subscribed = set()


class WeldError(RuntimeError):
    pass


def _watch(name):
    """The plugin publishes "attached"/"detached" on /htgspp/<name>/attached
    each time its joint state ACTUALLY changes (observed 2026-10-04: detach,
    attach, detach -> three messages, in that order)."""
    if name in _subscribed:
        return

    def cb(msg):
        with _changed:
            _state[name] = msg.data
            _changed.notify_all()
    if not _node.subscribe(StringMsg, f"/htgspp/{name}/attached", cb):
        raise WeldError(f"cannot subscribe to /htgspp/{name}/attached")
    _subscribed.add(name)


def send(action, name, timeout_s=20.0, confirm_s=5.0, attempts=3):
    """Publish once the plugin is listening, then CONFIRM the plugin reported
    the new state; re-publish if it did not, up to `attempts` times.

    Episode 57 (2026-10-03): a message delivered to a connected subscriber is
    not proof the weld took -- the box never rose. Base probe (2026-10-04):
    the plugin subscribes to its detach topic as soon as the model loads but
    creates the joint a moment later, so a release landing in between is
    ignored and the object is THEN welded (last reported state 'attached').
    That is held-out episode 13's stuck weld. Re-publishing on an unconfirmed
    state closes the window; only a persistent refusal is an error."""
    want = "attached" if action == "attach" else "detached"
    _watch(name)
    topic = f"/htgspp/{name}/{action}"
    pub = _node.advertise(topic, Empty)
    end = time.time() + timeout_s
    while not pub.has_connections():
        if time.time() > end:
            raise WeldError(f"WELD_{action.upper()}_UNHEARD: no subscriber on {topic} after {timeout_s:.0f} s")
        time.sleep(0.05)
    for n in range(1, attempts + 1):
        with _changed:
            _state.pop(name, None)
        pub.publish(Empty())
        with _changed:
            if _changed.wait_for(lambda: _state.get(name) == want, timeout=confirm_s):
                return n
    raise WeldError(f"WELD_{action.upper()}_UNCONFIRMED: plugin did not report '{want}' on "
                    f"/htgspp/{name}/attached after {attempts} publishes "
                    f"({confirm_s:.0f} s each; last: {_state.get(name)!r})")


def main():
    if len(sys.argv) < 3 or sys.argv[1] not in ("attach", "detach"):
        sys.exit(__doc__)
    try:
        for name in sys.argv[2:]:
            send(sys.argv[1], name)
            print(f"{sys.argv[1]} {name}: confirmed by the plugin")
    except WeldError as e:
        print(f"FATAL: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
