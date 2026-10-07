# What this does not prove (Part 12)

Volunteering the limits is what makes the rest credible.

## The grasp mechanism

Per `doc/GRASP_DECISION.md`: the reference grasp mechanism is confirmed
genuine (no weld/attach code anywhere, statically and at runtime), but two
live attempts on Role A's hardware failed to complete a full grasp, for a
diagnosed timing reason (the reference node's wall-clock settle logic vs.
this container's RTF ≈0.14). A8 (simulated attachment) is therefore the
adopted floor for episodes recorded on this hardware.

**If the weld is in use for a given episode** (`simulated_attachment: true`
in that episode's metadata): the carry is not a grasp. A policy trained on
it learns approach, close near the object, carry, open — not the physics
of grasping. This is labelled in the model's SDF comments, in
`run_pick_and_place.py`'s own attach/detach calls, and in every episode's
metadata — never silently.

**If a physical grasp is adopted** (Role B re-runs Part 3's test and it
succeeds there, per the recommendation in `doc/GRASP_DECISION.md`): it is
still rigid-body contact with tuned friction, not the physical gripper.
The physical MyCobot's Pro adaptive gripper stalls on an object at an
angle that is not the one commanded (per `CLAUDE.md`: 52° measured for a
20° command) — none of that stall/compliance behaviour is modelled here.

## Ground truth and scripted selection

Per A5: the demonstrator (`run_pick_and_place.py`) reads ground-truth
object poses from Gazebo. Correct and deliberate — a VLA is trained on
`instruction + image → action` and never observes how the demonstrator knew
the pose — but it means:
- The dataset contains no perception failures, no partial-occlusion
  recoveries, no cases where the object was not where the camera believed.
- Object **selection** was scripted, not perceived — the demonstrator
  always reaches the correct object because the episode matrix told it
  which one. The trained policy must learn selection from pixels and
  language; the dataset shows only the solved form of that problem, never
  a demonstrator hesitating or correcting a wrong guess.

## Two limitations a learned policy inherits that the classical pipeline does not

Both are first-party measurements from this project, not assumptions:

- **Approach direction.** `CLAUDE.md`'s own measured campaign: mixing
  approach directions costs 5.88 mm of bias vs. 0.67 mm for a consistent
  unidirectional approach — roughly 9× worse. The classical pipeline avoids
  this by construction (every descent in this dataset is top-down, Part
  7.1). A policy emitting actions has no such guarantee and will reproduce
  whatever the demonstrations contained; this dataset is unidirectional by
  construction, but a policy trained on it has not been taught *why* that
  matters, only shown data where it happens to hold.
- **Sim-to-real visual domain gap.** The project's own DREAM pose-estimation
  result (`CLAUDE.md`): ~26% real-image accuracy from synthetic-only
  training, rising to 91.6% only after mixing real data with 5× oversampling.
  Mixing beats scaling; the same is almost certainly true here, and this
  dataset is 100% synthetic.

## Scale

**Sixty episodes is not a training set.** The community figure for
fine-tuning a VLA is 100–500 episodes *per task*; this is four tasks × 15.
It is a smoke test with real task semantics and a real held-out condition,
not a dataset ready to train a deployable policy on.

## Reach-limited bin placement (new to this acquisition)

Two of the four bins were repositioned mid-session after offline IK
confirmed the layout's originally-chosen positions were unreachable at any
height, on any kinematic branch (`doc/BIN_REPOSITIONING.md`). The final
positions are verified reachable and mutually non-colliding, but this is a
reminder that this world's geometry has not been through anywhere near the
number of real-world iterations `pick_and_place_sorting.sdf` has — treat
newly-added positions in this world as unverified until checked, not as
safe by analogy to the older world.

## What it does prove, precisely

That the pipeline produces task-grounded episodes in the project's real
ArUco-calibrated world, with four distinct instructions whose language is
load-bearing (Part 5.1 — with one object per episode, previously, a policy
ignoring the instruction stream scored 100%; with four, it ceilings at
25%), verified per episode including correct-bin placement, split against
a genuinely different camera mount as the held-out condition — and that
everything except the batch run itself and the graphical path is feasible
and was actually exercised on constrained hardware.
