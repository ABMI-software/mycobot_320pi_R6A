# Items to refer to the robotics team (Appendix B)

Resolved locally where possible; what remains is listed with its actual
current status, not restated as if still open when it isn't.

1. **The world conflict (A1, Part 2).** RESOLVED locally, not just
   referred: `real_table.sdf` was extended from one object to eight
   (Part 2/`doc/WORLD_SELECTION.md`), and the two originally-conflicting
   premises were reconciled (they described the same file at two different
   points in time, not two different files). **Still worth telling the
   team**: this merge happened on `feature/pick-and-place-osama`,
   uncommitted as of this session — they should know before anyone else
   builds on that branch expecting the old single-object world.

2. **The reference pipeline's grasp mechanism (A4, Part 3).** OPEN,
   genuinely needs the team's hardware. The mechanism is confirmed
   genuine (no weld/attach code, statically and at runtime), but two live
   attempts on this constrained container both failed to complete a grasp
   for a diagnosed timing reason (wall-clock settle logic vs. this
   container's RTF≈0.14), not a mechanism failure. **Ask**: re-run the
   same bounded test (`doc/GRASP_DECISION.md` §3.3) on Role B's faster
   hardware (RTF≈0.277 per the specification's own baseline) before
   treating A8's simulated-attachment floor as final for the whole
   dataset — if it succeeds there, it retires the dataset's largest
   caveat at the cost of nothing but confirming it.

3. **The second camera (Part 6.4).** RESOLVED locally: `real_table.sdf`'s
   robot rig already includes `synth_camera_right` (and left/top), a
   genuinely different, world-fixed vantage point, confirmed via the URDF
   (`world_to_camera_right`, pose 0,1.1,0.4). No world edit was needed;
   used directly as the held-out condition.

4. **Bin static-ness (Part 4.4).** RESOLVED, no ambiguity: all four bins
   confirmed `<static>true</static>` directly from the merged SDF.

5. **Bin reach — a new item, not in the original register.** Two of the
   four bins (`blue_bin`, `green_bin`) and `yellow_bin` were repositioned
   mid-session after offline IK found their originally-chosen positions
   entirely unreachable (`doc/BIN_REPOSITIONING.md`). Fixed locally and
   re-verified, but **the team should know the world file changed twice**
   in this session (once to add the 4 objects, once to fix bin reach) —
   both changes are in the same uncommitted `real_table.sdf`.

6. **Object-detector licensing.** Not investigated further this session —
   carried forward as stated in the specification: the detector in use is
   built on an AGPL-3.0-licensed framework with a paid enterprise
   alternative. Does not block this acquisition (the demonstrator uses
   ground truth, per A5), but belongs in any industrial-limitations
   analysis. Raise with whoever handles licensing.

7. **Long-term storage of the bags (Part 10.8).** Confirmed ≥800 GB free
   on both the host and container filesystems — not a near-term
   constraint. ~5 GB expected for the full 60-episode batch. Policy
   (regenerable from `batch.sh`, excluded from version control) stated in
   `datasets/README.md`; ask the team to confirm this is acceptable if
   there's a retention policy this session doesn't know about.

8. **Scheduling of Role B machine time (A10).** Not scheduled this
   session — this is an organisational decision, not a technical one.
   Five to eight hours of dedicated, uninterrupted machine time, host
   sleep disabled (`RUNNING.md` §2). Confirm when.
