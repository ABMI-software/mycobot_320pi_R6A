# GitHub "About" — proposals

Three variants of the repository description, from plainest to most
value-oriented. GitHub caps this field at **350 characters**; the count for each
is given below. Pick one and paste it into *Settings → About → Description*.

---

## Variant A — factual

> Modular ROS 2 robotics platform for a MyCobot 320 Pi: marker-free arm pose
> estimation from RGB in an eye-to-hand setup, a Gazebo Harmonic digital twin
> generating annotated training data, sim-to-real fine-tuning, and an
> instrumented physical bench characterised against ISO 9283.

**278 characters.** States what the repository contains, nothing more. Safest
for an audience that will read the code.

---

## Variant B — problem-led

> Making adaptive robotics affordable for SMEs. Guides a 6-DoF arm from fixed
> cameras with no marker on the robot: a Gazebo digital twin generates the
> annotated data a real robot cannot, a keypoint model closes the sim-to-real
> gap, and a measured bench arbitrates every claim.

**274 characters.** Leads with the industrial problem; keeps the measurement
discipline visible. Good default for a mixed audience.

---

## Variant C — value-oriented

> Vision-guided modular robotics on open-source foundations: marker-free pose
> estimation trained in simulation and validated on a metrologically
> characterised bench. Built so that a cell can be reconfigured rather than
> re-integrated. ABMI Research & Innovation.

**259 characters.** Foregrounds the reconfiguration promise and names the
department. Best suited to a partner or funding reader.

---

## Suggested topics

Lowercase, hyphenated, consistent with what the repository actually contains:

```
ros2
gazebo
digital-twin
robotics
pose-estimation
sim-to-real
computer-vision
mycobot
robot-calibration
synthetic-data
eye-to-hand
metrology
```

Notes on the selection:

- `eye-to-hand` and `metrology` are narrow but they are the two things that
  distinguish this repository from any other ROS 2 arm project.
- Deliberately **not** included: `machine-learning` and `deep-learning` (too
  broad to help discovery), `openvla` and `lerobot` (present only as pipeline
  proofs on scripted episodes — they would over-promise), and `isaac-sim`
  (planned, no code in the repository).
