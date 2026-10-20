# What to measure (Part 13)

## Why 0.63 mm is the wrong target (A12)

Established at length earlier in this project's work (see the session
record and `training/calibration/PROTOCOLE_ESSAIS_PRECISION.md` §7,
verified directly, not from memory): 0.63 mm is essai 7's converged
closed-loop *servoing* accuracy after two visual correction passes —
10.79 mm open loop → 1.29 mm after one correction → 0.63 mm after two,
18/20 trials under 1 mm, dispersion flat throughout (a purely systematic,
correctable bias, not a floor). It measures how precisely a system
converges on a commanded metrology point when permitted to look, correct,
and look again.

A vision-language-action policy does none of that at inference. It emits
actions from pixels and language, open loop, with no metrology target and
no correction cycle. Holding it to a servoing figure measures a different
quantity and reports the mismatch as failure. Its legitimate place is as
the measured performance of the current production architecture — quoted
honestly as what it is, in any comparative analysis — not as this
acquisition's target.

## The metric set for this dataset

| Metric | Definition | Reference value |
|---|---|---|
| Task success rate | Correct object, correct bin, per episode | Scripted demonstrator: 100% by construction (Part 9 rejects anything less before it's counted) |
| Correct-bin rate | Of successful grasps, the fraction landing in the right bin | The language-following measure |
| Held-out-condition success | Success rate under `synth_camera_right` (never seen in training) | The generalisation measure |
| Placement margin | Object-centre to bin-centre distance, as a fraction of `inner_radius` (0.0475 m) | Centimetres, not millimetres |
| Distractor disturbance rate | Episodes where a non-target object moved > 2 cm (Part 9.3) | Demonstration cleanliness |
| Episode yield | Clean episodes per attempt | Pipeline reliability; feeds A9's scheduling |

## Why correct-bin rate is the headline

Per Part 5.1: with one object and one destination (the previous
acquisition), a policy that discards the language stream entirely can
still score 100% — the dataset cannot distinguish a model that reads
instructions from one that ignores them. With four objects and four
bins present in every frame, a policy ignoring the language ceilings at
25% (chance across four bins). Correct-bin rate is the only metric in
this set that can actually reveal that failure mode; no millimetre figure
can. Report it **alongside** overall task success rate, never instead of
it — a policy can achieve high grasp success and chance-level bin
selection, and those are different failures with different remedies (the
first is a manipulation problem, the second is a
language-grounding problem).

## Where this leaves 0.63 mm

Cite it, honestly, only when discussing the *current production
architecture's* classical-pipeline accuracy ceiling as a point of
comparison for "what would replacing it with a learned policy cost or
gain" — never as a bar the VLA-trained-on-this-dataset must clear.
