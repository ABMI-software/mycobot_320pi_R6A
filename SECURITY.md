# Security

## Reporting

Report a vulnerability privately to **jo.bernardo@abmi-groupe.com** — not in a
public issue. Expect an acknowledgement within a week.

## What this repository is

A research platform, not a product. It has no authentication, no authorisation
and no transport security anywhere, by design:

- The **robot bridge listens on plain TCP, port 5005**, and accepts motion
  commands from anyone who can reach it. It is single-client and blocking.
- **rosbridge** exposes the ROS graph over an unauthenticated WebSocket.
- The ROS 2 graph itself runs without SROS 2 enclaves.

Consequently: **run this on an isolated lab network.** Anything that can route
to the robot's board can move the arm. That is a physical-safety property, not
just a network one — see the physical-robot section of
[`CONTRIBUTING.md`](CONTRIBUTING.md).

## Scope

In scope: anything that lets an unintended party command motion, corrupt a
calibration, or exfiltrate data from a machine running this code.

Out of scope: the absence of authentication on the bridge and on rosbridge. It
is documented above, it is deliberate for a lab bench, and reporting it as a
vulnerability tells us nothing new. If you have a design for adding it without
breaking the latency budget, open an issue instead.

## Secrets

No credential, token or key belongs in this repository. Calibration extrinsics
are **not committed** either — they go stale, and a stale extrinsic published as
fresh is worse than none.

If a secret is ever committed, treat it as compromised the moment it is pushed:
rotate it first, then remove it from history.
