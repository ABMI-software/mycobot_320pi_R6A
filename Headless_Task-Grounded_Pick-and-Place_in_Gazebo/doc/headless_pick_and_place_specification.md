# Headless Task-Grounded Pick-and-Place in Gazebo
## Closing the "no real task semantics" gap, on a CPU-only laptop

---

## Part 0 — Why this is the right thing to build now

Your two existing episodes prove the **plumbing**: Gazebo → recording → LeRobot dataset → reload. That was the point, and it worked.

What they don't contain is a **task**. The arm moves through scripted joint positions. Nothing is picked up. The instruction string, if any, describes nothing that happens in the frames.

That matters because a VLA learns the mapping *instruction + image → action*. If the instruction has no relationship to what the pixels show or what the actions accomplish, there is nothing to learn. Fine-tuning on such data would produce a model that has seen the right *shape* of data and none of its *content*.

**What this work adds, and what it costs:**

| | |
|---|---|
| **Adds** | Episodes where a real object is really grasped and really moved, with an instruction that truthfully describes it |
| **Compute cost** | Zero ML. Physics only. |
| **Reversibility** | Total — objects are spawned at runtime, no world file is edited |
| **Hardware ceiling** | None hit. This is the largest useful thing your laptop can do on this project |
| **Transfers to real robot?** | The *method* yes, the *data* partially — see Part 10 |

And a practical point worth stating plainly: **headless is genuinely better here, not a fallback.** A viewport tells you the block "looks grasped." Pose telemetry tells you the block's Z rose 87 mm in lockstep with the end-effector, with a constant offset, for 340 consecutive samples. The second is evidence; the first is an impression.

---

## Part 1 — Step 1: Author the block

### 1.1 Why a standalone SDF rather than editing the world

Three reasons, and the third is the one that matters most for you:

1. **Reversibility** — spawning adds; editing a world file mutates. If the block is wrong, you delete an entity rather than reverting a file.
2. **Parametrization** — the same SDF spawns at any pose. Ten different starting positions is ten CLI invocations, not ten world files.
3. **It's the mechanism already in use** — the robot itself is spawned this way in `mycobot_teleop.launch.py`. You're reusing a proven path, not inventing one.

### 1.2 The file

`models/red_block.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="red_block">
    <link name="link">

      <inertial>
        <mass>0.05</mass>
        <inertia>
          <ixx>1.333e-05</ixx>
          <iyy>1.333e-05</iyy>
          <izz>1.333e-05</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>

      <collision name="collision">
        <geometry>
          <box><size>0.04 0.04 0.04</size></box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1.5</mu>
              <mu2>1.5</mu2>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>1e6</kp>
              <kd>1.0</kd>
            </ode>
          </contact>
        </surface>
      </collision>

      <visual name="visual">
        <geometry>
          <box><size>0.04 0.04 0.04</size></box>
        </geometry>
        <material>
          <ambient>0.8 0.05 0.05 1</ambient>
          <diffuse>0.9 0.05 0.05 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
        </material>
      </visual>

    </link>
  </model>
</sdf>
```

### 1.3 Why each number is what it is

**Mass 0.05 kg (50 g).** Light enough that gravity droop and gripper torque aren't the dominant failure. Recall the measured droop on this arm: ~13 mm unloaded, ~15 mm loaded. A heavy block makes that worse.

**Size 0.04 m (4 cm cube).** A genuine trade-off:

| Size | Camera visibility | Graspability |
|---|---|---|
| 2 cm | Poor — the camera sits ~100 cm up, small objects appear notably reduced | Good |
| **4 cm** | **Adequate** | **Adequate** |
| 6 cm | Good | Risky — gripper span |

Start at 4 cm. If the camera frames are unconvincing, go up; if grasping fails, go down. One variable at a time.

**Inertia 1.333e-05.** Not arbitrary. For a solid box:

```
Ixx = m/12 × (y² + z²)
    = 0.05/12 × (0.04² + 0.04²)
    = 0.05/12 × 0.0032
    = 1.333e-05
```

Symmetric cube, so all three are equal and the products of inertia are zero.

**Why not leave inertia at defaults:** the physics engine needs a plausible mass distribution to integrate rotation. Wrong inertia produces blocks that spin absurdly on contact or refuse to tip — and it looks like a grasping bug when it's a modelling bug.

**Friction μ = 1.5 — the single most important number for grasping.** Default friction in most sim setups is around 1.0 or lower, which is often too slippery for a rigid-body pinch grasp. The gripper closes, the block squirts out. Raising μ is the standard first move.

**Contact stiffness kp = 1e6.** Softer than the default rigid contact, which reduces the jitter you get when two hard bodies interpenetrate slightly each timestep.

### ✅ Definition of done for step 1

The file exists and parses. Check before spawning:

```bash
gz sdf -k models/red_block.sdf
```

No output means valid. Errors print with line numbers.

---

## Part 2 — Step 2: Spawn it, headless

### 2.1 The command

```bash
ros2 run ros_gz_sim create \
  -file models/red_block.sdf \
  -name red_block \
  -x 0.25 -y 0.0 -z 0.05
```

### 2.2 Choosing the pose

**x = 0.25 m.** Inside the documented safety envelope (X 130–350 mm) and comfortably within the ~390 mm reach. Not near full extension, where stiffness is lowest and IK is worst-conditioned.

**y = 0.0.** Straight ahead. Simplest case first — off-axis targets involve more J1 rotation and more chance of the elbow-branch issue.

**z = 0.05.** Half the block height (0.02) plus table height. **Adjust to your table.** If your table surface is at z = 0.78, spawn at 0.80.

Get this wrong in either direction and:
- **Too low** → the block spawns inside the table, physics ejects it violently
- **Too high** → it falls, bounces, and ends up somewhere you didn't choose

### 2.3 Worked example — three positions for variation

```bash
# Centre
ros2 run ros_gz_sim create -file models/red_block.sdf -name red_block -x 0.25 -y 0.00 -z 0.05

# Left
ros2 run ros_gz_sim create -file models/red_block.sdf -name red_block -x 0.24 -y 0.08 -z 0.05

# Near
ros2 run ros_gz_sim create -file models/red_block.sdf -name red_block -x 0.20 -y -0.05 -z 0.05
```

Position variation across episodes matters more than you might expect — it's what stops a model learning "always reach to this one spot."

### 2.4 Removing it

```bash
gz service -s /world/<world_name>/remove \
  --reqtype gz.msgs.Entity \
  --reptype gz.msgs.Boolean \
  --timeout 2000 \
  --req 'name: "red_block", type: MODEL'
```

*(Verify the exact service path against `gz service -l` — world names differ.)*

This is what makes the whole exercise reversible: spawn, test, remove, adjust, respawn.

---

## Part 3 — Step 3: Verify it landed correctly, without a viewport

Two independent checks. Do both — they catch different failures.

### 3.1 Numeric check

```bash
gz model --list
gz model -m red_block -p
```

**What you want to see:**

```
Model: [12] red_block
  Pose [ XYZ (m) ] [ RPY (rad) ]:
    [0.250000 | 0.000000 | 0.050000]
    [0.000000 | 0.000000 | 0.000000]
```

**What each failure looks like:**

| Symptom | Diagnosis |
|---|---|
| Z settles slightly *below* spawn height | Normal — the block settled onto the table |
| Z drops toward 0 or negative | Fell through the table — collision geometry missing or table has no collision |
| Z climbs then oscillates | Spawned inside the table, being ejected |
| Non-zero RPY on a block you spawned level | Landed on an edge and tipped |
| X/Y drifted from spawn values | Bounced. Lower the spawn height. |

**Let it settle before judging.** Wait a few seconds of *simulation* time — which at ~6% real-time factor is over a minute of wall clock. Don't read the pose the instant after spawning.

### 3.2 Visual check — the camera-frame technique

Same trick you used for the colour-order check and the gripper verification.

```python
#!/usr/bin/env python3
"""grab_frame.py — save one camera frame to PNG, headless."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np, cv2, sys

class Grab(Node):
    def __init__(self, out):
        super().__init__('grab_frame')
        self.out = out
        self.done = False
        self.create_subscription(
            CompressedImage, '/synth_camera/image/compressed', self.cb, 10)

    def cb(self, msg):
        if self.done:
            return
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        cv2.imwrite(self.out, img)
        self.get_logger().info(f'saved {self.out}  shape={img.shape}')
        self.done = True

def main():
    out = sys.argv[1] if len(sys.argv) > 1 else 'frame.png'
    rclpy.init()
    n = Grab(out)
    while rclpy.ok() and not n.done:
        rclpy.spin_once(n, timeout_sec=1.0)
    n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```bash
python3 grab_frame.py block_check.png
```

**What to look for in the PNG:**
1. Is the block visible at all?
2. Does it sit *on* the table, not intersecting it?
3. Is it actually red? *(If it looks blue, you have the BGR/RGB swap — `cv2.imdecode` returns BGR, and `cv2.imwrite` expects BGR, so this path is consistent. A blue block means the swap is upstream.)*
4. Is it within the arm's likely reach in frame?

### ✅ Definition of done for step 2–3

Pose is stable at the expected height, and the block is visible and correctly coloured in a saved frame.

---

## Part 4 — Step 4: Add the destination

"Place it on the plate" needs a plate. Same pattern — `models/plate.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="plate">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <cylinder><radius>0.06</radius><length>0.008</length></cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder><radius>0.06</radius><length>0.008</length></cylinder>
        </geometry>
        <material>
          <ambient>0.9 0.9 0.85 1</ambient>
          <diffuse>0.95 0.95 0.9 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
```

**`<static>true</static>` is the key line.** The plate doesn't move, doesn't need inertia, and can't be knocked over by a clumsy approach. One fewer failure mode.

```bash
ros2 run ros_gz_sim create -file models/plate.sdf -name plate \
  -x 0.20 -y -0.15 -z 0.044
```

Place it **away from the block** — same workspace, different location — so "place it on the plate" is a real transport, not a 2 cm nudge.

---

## Part 5 — Step 5: The IK gap

This is the one piece of genuinely new code, and you've identified it correctly.

### 5.1 The problem

`send_moveit_goal.py` takes six joint angles. You need to aim at a Cartesian point — the block's (x, y, z). Nothing currently converts one to the other.

### 5.2 Why IK, not `send_coords`

Worth restating because the temptation is real: the manufacturer's Cartesian call was measured at **247.8 mm final error** versus **18.2 mm** for `send_angles` plus differential IK, on an identical 267 mm target — and it **failed silently**, returning `OK` from the bridge either way. Cause: gimbal lock with RY between −78° and −83°.

So: solve IK yourself, command joints.

### 5.3 The approach — reuse what you already have

You already used `moveit_py`'s `RobotState` for forward kinematics in the RLDS pipeline. The same object does inverse kinematics.

```python
# Sketch — verify method names against your moveit_py version
from moveit.core.robot_state import RobotState
from geometry_msgs.msg import PoseStamped

def ik_for_point(robot_model, x, y, z, approach="top_down"):
    """Solve joint angles for a gripper pose above (x, y, z)."""
    state = RobotState(robot_model)

    target = PoseStamped()
    target.header.frame_id = "base_link"
    target.pose.position.x = x
    target.pose.position.y = y
    target.pose.position.z = z
    # top-down: gripper pointing at the table
    target.pose.orientation.x = 1.0
    target.pose.orientation.y = 0.0
    target.pose.orientation.z = 0.0
    target.pose.orientation.w = 0.0

    ok = state.set_from_ik("arm_group", target.pose, "gripper_link", timeout=1.0)
    if not ok:
        return None
    return state.get_joint_group_positions("arm_group")
```

### 5.4 Three things that will bite you

**Handle IK failure explicitly.** `set_from_ik` returns False when no solution exists. Decide now what happens: abort the episode, try a nearby point, or log and hold. Silent failure is how a policy appears to "freeze."

**Force the elbow-up branch.** The documented finding: all historical poses are elbow-down, pressed against the J2 limit with **0° margin**, causing **150° branch jumps on J4** in closed loop. Check `J3 < 0` in the returned solution; if not, reject and re-solve with a different seed.

```python
    q = state.get_joint_group_positions("arm_group")
    if q[2] >= 0:          # J3 must be negative — elbow-up
        return None        # reject, caller retries with another seed
```

**Approach from one consistent direction.** Repeatability is **0.67 mm** approaching unidirectionally from above, but **5.88 mm of bias** when approach directions are mixed. Always descend from above. Never mix.

### ✅ Definition of done for step 5

Given (0.25, 0.0, 0.10), the function returns six joint angles with J3 < 0, and `send_moveit_goal` moves the arm to hover above the block's spawn position.

---

## Part 6 — Step 6: Script the sequence

### 6.1 The eight phases

```
1. HOME          → known starting pose
2. APPROACH      → above the block, gripper open   (block_z + 0.10)
3. DESCEND       → to grasp height                 (block_z + 0.01)
4. GRASP         → close gripper, wait ≥1.6 s
5. LIFT          → back up                         (block_z + 0.10)
6. TRANSPORT     → above the plate                 (plate_z + 0.10)
7. PLACE         → descend, open gripper, wait ≥1.6 s
8. RETREAT       → back up, return HOME
```

### 6.2 Timing constraints that are not optional

| Constraint | Value | Consequence |
|---|---|---|
| Command cooldown | **200 ms minimum** | Max 5 Hz. Faster and commands are dropped. |
| Gripper interval | **~1.6 s** | Roughly 0.6 Hz. The gripper, not the arm, is your slowest link. |
| Simulation rate | **~6% real-time** | A 10 s episode takes ~2.5 min wall clock. Budget accordingly. |

That last row is why the script must be non-interactive. You start it and walk away.

### 6.3 Structure

```python
STEPS = [
    ("home",      lambda: goto_joints(HOME_Q),                  1.0),
    ("approach",  lambda: goto_xyz(bx, by, bz + 0.10),          2.0),
    ("descend",   lambda: goto_xyz(bx, by, bz + 0.01),          2.0),
    ("grasp",     lambda: gripper(CLOSED),                      2.0),   # > 1.6 s
    ("lift",      lambda: goto_xyz(bx, by, bz + 0.10),          2.0),
    ("transport", lambda: goto_xyz(px, py, pz + 0.10),          3.0),
    ("place",     lambda: goto_xyz(px, py, pz + 0.03),          2.0),
    ("release",   lambda: gripper(OPEN),                        2.0),   # > 1.6 s
    ("retreat",   lambda: goto_xyz(px, py, pz + 0.10),          2.0),
    ("home",      lambda: goto_joints(HOME_Q),                  2.0),
]

for name, action, settle in STEPS:
    log(f"--- {name} ---")
    if action() is False:
        log(f"FAILED at {name}"); break
    time.sleep(settle)
    log_pose_snapshot(name)       # feeds Part 7
```

### 6.4 The grasp-height detail

`bz + 0.01` rather than `bz` exactly. For a 4 cm cube whose *centre* is at bz, the top face is at bz + 0.02. Aiming at the centre would drive the gripper through the block. Aiming 1 cm above centre puts the fingers around the upper half.

**Tune this empirically.** It's the number most likely to need adjusting, and the cheapest to change.

---

## Part 7 — Step 7: Prove the grasp numerically

This is the part that's genuinely better headless, and it's the strongest output of the whole exercise.

### 7.1 The principle

A grasp is real if and only if, during LIFT and TRANSPORT:

1. The block's Z rises **together with** the end-effector's Z, and
2. The **offset between them stays constant**

Point 2 is what distinguishes a grasp from a coincidence. A block sitting on a moving table also rises; a block that is *held* maintains a fixed relationship to the hand.

### 7.2 The logger

```python
#!/usr/bin/env python3
"""track_grasp.py — log block pose and end-effector pose together."""
import subprocess, re, time, csv

def block_pose():
    out = subprocess.run(['gz','model','-m','red_block','-p'],
                         capture_output=True, text=True).stdout
    m = re.search(r'\[\s*([-\d.]+)\s*\|\s*([-\d.]+)\s*\|\s*([-\d.]+)\s*\]', out)
    return tuple(map(float, m.groups())) if m else None

def ee_pose():
    # from /tf, or FK on the latest /joint_states — you already have this
    ...

with open('grasp_log.csv','w',newline='') as f:
    w = csv.writer(f)
    w.writerow(['t','phase','bx','by','bz','ex','ey','ez','dz'])
    t0 = time.time()
    while running():
        b, e = block_pose(), ee_pose()
        if b and e:
            w.writerow([round(time.time()-t0,2), current_phase(),
                        *b, *e, round(e[2]-b[2], 4)])
        time.sleep(0.1)
```

### 7.3 What success looks like

```
t      phase       bz      ez      dz
----------------------------------------
2.1    approach    0.050   0.150   0.100
4.3    descend     0.050   0.060   0.010
6.0    grasp       0.050   0.060   0.010
8.2    lift        0.087   0.097   0.010   ← block rising with the hand
9.1    lift        0.124   0.134   0.010   ← dz CONSTANT
11.0   transport   0.150   0.160   0.010   ← still constant
14.5   place       0.052   0.062   0.010
16.0   release     0.048   0.062   0.014   ← dz opens: released
```

**`dz` constant at 0.010 through lift and transport is the proof.** The block is rigidly attached to the hand for that entire span.

### 7.4 What failure looks like

```
t      phase       bz      ez      dz
----------------------------------------
6.0    grasp       0.050   0.060   0.010
8.2    lift        0.050   0.097   0.047   ← hand rose, block didn't
9.1    lift        0.050   0.134   0.084   ← dz growing: slipped
```

**Diagnosis when `dz` grows during lift:** the grasp didn't hold. In order of likelihood — friction too low (raise μ), grasp height wrong (adjust the +0.01), gripper not closing far enough, or block too heavy.

### 7.5 An automatic pass/fail

```python
def grasp_held(rows, tol=0.005):
    """dz must stay within tol of its value at the end of GRASP."""
    lift = [r for r in rows if r['phase'] in ('lift','transport')]
    if not lift:
        return False
    ref = lift[0]['dz']
    return all(abs(r['dz'] - ref) < tol for r in lift)
```

Now each episode is **self-verifying**. You record twenty and the script tells you which eight are good — no video review.

### 7.6 Visual corroboration

Grab a frame at the peak of LIFT:

```python
if phase == 'lift' and not snapped:
    subprocess.run(['python3','grab_frame.py',f'lift_{episode:03d}.png'])
    snapped = True
```

Belt and braces: the numbers say held, the image shows it held.

---

## Part 8 — Step 8: Record it as a real episode

Now the payoff. Run the same sequence under `rosetta`'s recorder, with a truthful instruction.

```bash
# Terminal 1: Gazebo (headless)
gz sim -s -r worlds/mycobot_world.sdf

# Terminal 2: ROS 2 bringup + bridges
ros2 launch mycobot_gazebo mycobot_teleop.launch.py

# Terminal 3: recorder
ros2 run rosetta episode_recorder_server --contract contracts/mycobot.yaml

# Terminal 4: the episode
python3 scripts/spawn_scene.py --block-x 0.25 --block-y 0.00
python3 scripts/record_episode.py \
    --instruction "pick up the red block and place it on the plate" \
    --episode 003
```

### 8.1 The instruction is now true

Compare:

| | Instruction | Does it describe the frames? |
|---|---|---|
| **Episodes 1–2** | "move the arm" | Loosely. No object, no goal state. |
| **Episode 3+** | "pick up the red block and place it on the plate" | **Yes.** A red block is visible, is grasped, and ends on the plate. |

That difference is the entire point of the exercise.

### 8.2 Instruction phrasing

Keep them **short, imperative, verb-first, and consistent across episodes of the same task.** `"pick up the red block and place it on the plate"` for all of them. Save paraphrase variation for later, deliberately, not by accident.

### 8.3 A realistic first batch

| Episodes | Block position | Purpose |
|---|---|---|
| 1–5 | x=0.25, y=0.00 | Same position — establish it works |
| 6–10 | x varies 0.22–0.28 | Position variation |
| 11–15 | y varies −0.08–0.08 | Lateral variation |
| 16–20 | Camera moved | **Held out for validation** |

That last row matters. The DREAM self-calibration failed because the model memorized two fixed camera mounts rather than learning to localize. A held-out camera configuration is the only test that catches that class of failure — holding out *images* isn't enough, you have to hold out *conditions*.

At ~2.5 min wall clock per episode plus reset, **twenty episodes is roughly a working day.** Entirely feasible on your laptop.

---

## Part 9 — How this maps to the work order

This is what makes it defensible as work rather than tinkering.

### 9.1 Sub-study 5 — "Étude de l'intégration dans l'architecture Robot5A"

The FDT asks explicitly for *"l'utilisation du jumeau numérique Gazebo pour la validation."*

**This is that, literally.** Not a description of how the twin could be used — a working demonstration of it validating a manipulation task end to end, with numeric proof of success.

### 9.2 Sub-study 4 — "Analyse des données d'entraînement"

The FDT asks for *"les modalités de collecte"* and *"les approches d'apprentissage par démonstration"*, and to *"évaluer la possibilité de réutiliser les données produites lors des activités de simulation et de téléopération."*

**Before:** you could report that the existing simulation data (50,000 images, 12,500 poses) is *not* reusable for VLA training, because pose-estimation labels aren't actions-in-a-trajectory.

**After:** you can report that the simulation *infrastructure* produces usable VLA episodes, and show them. That converts a negative finding into a positive one.

### 9.3 Sub-study 3 — "Analyse des besoins matériels"

The FDT asks to *"identifier les configurations compatibles avec les plateformes actuellement utilisées."*

You'd have a measured boundary: **data generation needs no GPU at all; only fine-tuning does.** That's a genuinely useful separation for planning — it means data collection can start before any GPU question is resolved.

### 9.4 Sub-study 6 — "Analyse des limitations actuelles"

Everything hit first-hand: the ~6% real-time factor, cameras at ~10 fps in sim-time rather than 30, the 5 Hz command ceiling, the 1.6 s gripper interval, and whatever grasping difficulties emerge.

First-party measurements outrank citations, and the literature's own main criticism is that the field substitutes benchmark numbers for real-condition measurement.

### 9.5 "Synthèse et recommandation finale"

The FDT asks to *"proposer un plan d'expérimentation pour un premier démonstrateur sur le MyCobot 320 PI."*

**You'd be proposing a plan you have already executed in simulation.** The steps are written, the failure modes are known, the verification is automated. That is a materially stronger proposal than one derived from reading.

### 9.6 The repo's own POC direction

Point 5 of the POC direction calls for *"a reproducible loop of (teleop demos → LeRobot dataset → VLA fine-tune → sim eval → real-robot eval)."*

Your POC 1 built links 1 and 2 with empty content. **This fills them with real content** — and everything downstream depends on those links carrying something meaningful.

---

## Part 10 — What this does not prove

Worth stating plainly, both because it's true and because volunteering limits is what makes the rest credible.

**Simulated grasping is not real grasping.** Rigid-body contact with tuned friction is a crude approximation. The real gripper is a Pro adaptive unit that *stalls on the object* at an angle that isn't the commanded one — 52 measured for a command of 20. None of that is modelled.

**Sim-trained data carries a visual domain gap.** Gazebo renders differently from a real camera. A policy trained only on these episodes would likely fail on real images. The in-house DREAM result is the evidence: 10.9% real detection from synthetic-only training, rising to **91.6% only after mixing in real data with ×5 oversampling**. Mixing beats scaling — and that applies here too.

**Scripted motion is not demonstration quality.** A human teleoperating produces natural corrections, hesitations and recovery. A script produces the same trajectory every time. Useful for pipeline validation; thinner as behavioural data.

**Twenty episodes is not a training set.** The community figure for fine-tuning is 100–500 per task. This is a smoke test with real semantics, not a dataset.

**What it does prove, precisely:** the pipeline can produce task-grounded episodes with truthful instructions and verified success, using only the hardware you have. That was the open question, and it's the one worth closing before anyone invests in real-robot recording sessions.

---

## Appendix — Order of work

| # | Step | Effort | Done when |
|---|---|---|---|
| 1 | Author `red_block.sdf` | 1 h | `gz sdf -k` passes |
| 2 | Spawn and verify pose | 1 h | Pose stable at expected height |
| 3 | Camera-frame check | 30 min | Block visible, correct colour |
| 4 | Author and spawn plate | 30 min | Both objects present |
| 5 | Cartesian IK helper | **1–2 days** | Returns J3<0 solution; arm hovers over block |
| 6 | Script the 8-phase sequence | 1 day | Runs start to finish without intervention |
| 7 | Grasp verification logger | 1 day | CSV shows constant `dz` through lift |
| 8 | Record 20 episodes | 1 day | 20 LeRobot episodes, auto-verified |

**Total: roughly one week.** Step 5 is the only genuinely new code and the most likely to overrun; everything else reuses what already exists.

**Start with steps 1–3.** Two hours of work tells you whether the physics behaves at all, and the block is trivially removable if not.
