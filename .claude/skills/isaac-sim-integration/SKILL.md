---
name: isaac-sim-integration
description: Roadmap and guardrails for bringing NVIDIA Isaac Sim / Isaac Lab into the MyCobot R6A project as a physics-accurate digital twin and AI-physics training substrate. Invoke when the user explores Isaac Sim, asks about migrating from Gazebo/DART, or wants photorealistic synthetic data for DREAM or VLA training.
---

# Isaac Sim integration

**Scope.** This skill captures *why* Isaac Sim is on the roadmap, *what* it unlocks for this project, and the *sequenced path* to bring it in without breaking what Gazebo already does well. It does **not** replace Gazebo today — both will coexist through the POC phase.

## Why Isaac Sim, specifically

| Capability | Gazebo Harmonic (current) | Isaac Sim |
|------------|---------------------------|-----------|
| Physics | DART (rigid-body, deterministic) | PhysX 5 (rigid + soft + fluids), GPU-accelerated |
| Rendering | Basic OGRE-based | Omniverse RTX (path-traced, photorealistic) |
| Parallel envs | 1 sim per process | Thousands of copies in a single process (RL-grade) |
| Sensor models | Cameras + LiDAR basic | Accurate cameras, depth, IMU, force-torque with real noise models |
| ROS2 bridge | Native `ros_gz` | Native `omni.isaac.ros2_bridge` |
| USD asset format | No | Yes — industry-standard for DCC interchange |
| Annotated data | Segmentation via custom collectors | Built-in ground-truth for bbox, segmentation, depth, keypoints, 3D pose |

## The POC value proposition (what we actually get)

1. **Pose estimation accuracy jump** — DREAM is bottlenecked today by the sim-to-real gap (97% synthetic, ~26% real). Isaac Sim's path-traced renderer with real-noise sensor models should close a large fraction of that gap *without* adding more real data.
2. **Training signal for VLA** — VLA models (OpenVLA, π0, RT-X class) eat thousands of hours of annotated manipulation demos. A parallel Isaac Lab env generates that in days, not months.
3. **Digital twin as control reference** — a high-fidelity Isaac Sim twin can run in lockstep with the real robot, letting us catch drift, detect collisions predictively, and verify trajectories before they reach hardware.
4. **Physics AI experiments** — PhysX's differentiable-physics mode and the coming Newton engine open the door to learned dynamics models (world models) trained from sim.

## Sequenced path (do NOT parallelize these)

### Phase 0 — decide on host strategy (before any install)

Isaac Sim 4.x needs:
- NVIDIA driver ≥ 535, CUDA 12.x (current machine has CUDA 12.4 ✓)
- ≥ 16 GB VRAM for comfortable use (we have 20 GB Ada ✓)
- ≥ 32 GB RAM, SSD space for Omniverse Nucleus cache
- Ubuntu 22.04 or 24.04 ✓

Decision: native install vs. containerized (NVIDIA's `isaac-sim:4.5.0` container) — containerized is cleaner for CI and won't conflict with the ROS2 stack. **Recommended: containerized.**

### Phase 1 — asset conversion (USD)

1. Convert [`mycobot_description/urdf/320_pi/mycobot_pro_320_pi_gazebo.urdf`](../../../mycobot_description/urdf/320_pi/) → USD with the Isaac Sim URDF importer
2. Verify joint limits match the official elephantrobotics limits (see [`.claude/agents/urdf-surgeon.md`](../../agents/urdf-surgeon.md))
3. Compare FK between Gazebo and Isaac Sim at a grid of known joint configurations — they MUST match within 0.1° or we have a joint-axis convention mismatch
4. Port meshes: keep the 2022 DAE set, convert to USD geom prims with preserved inertia
5. Port the `pro_adaptive_gripper` — Isaac Sim handles mimic joints natively, so the 4-bar linkage can actually be simulated (Gazebo/DART can't — CHANGELOG 2.0.0)

### Phase 2 — ROS2 bridge

Isaac Sim's `omni.isaac.ros2_bridge` publishes the same topic names we already use in Gazebo:
- `/joint_states` at 150 Hz
- Camera topics on `/cameras/<name>/image_raw`
- Subscribe to `/joint_trajectory` for controlling the arm

Keep `/teleop/*` topics identical so [`teleop_dashboard.py`](../../../teleop/) continues to work unchanged.

### Phase 3 — synthetic data (pose estimation first)

Use Isaac Sim's Replicator:
1. Randomize: lighting (HDR env map), materials (Substance/MDL), clutter positions, camera intrinsics jitter
2. Annotate: 7 FK keypoints (base → link6) auto-projected to 2D — exact same format as [`training/dream/convert_to_ndds.py`](../../../training/dream/)
3. Render at real-camera resolution (640 × 480 matches the Arducam + Astra capture)
4. Target: 20 K photorealistic frames to augment the existing 50 K Gazebo frames
5. Retrain DREAM on the mix — eval target: **> 60% detection on real** (vs. current 26%)

### Phase 4 — parallel sim in Isaac Lab (VLA / RL)

Isaac Lab (the research-oriented layer on top of Isaac Sim, formerly Orbit) provides:
- `ManagerBasedEnv` templates for manipulation
- 1024+ parallel copies of the MyCobot arm in one process
- Curriculum + domain randomization APIs
- ROS2-compatible trajectory export

This is where VLA fine-tuning happens. See [`.claude/agents/vla-integrator.md`](../../agents/vla-integrator.md) for the VLA-side workflow.

### Phase 5 — real-robot validation

- Deploy Isaac-trained DREAM weights → measure real-data detection
- Deploy Isaac-trained VLA policy → run a short pick-and-place evaluation
- Publish the gap numbers to CHANGELOG as a milestone entry

## Non-negotiables

- **Do not replace Gazebo on `main`.** Isaac Sim lives on its own branch (`feature/isaac-sim`) until parity is proven.
- **Keep topic names identical** between Gazebo and Isaac Sim publishers. The teleop / DREAM / dashboard code should run against either without changes.
- **Pin the Isaac Sim version** (e.g. `4.5.0`) in every launch / Dockerfile. Omniverse updates can break extensions silently.
- **USD assets are large.** Use Git LFS, same as our DAE meshes. Never commit raw USD to regular git.

## First concrete actions (when the user greenlights)

1. Check driver: `nvidia-smi` — must show driver ≥ 535 and CUDA 12.x
2. Pull the container: `docker pull nvcr.io/nvidia/isaac-sim:4.5.0` (needs NGC login)
3. Quick-start headless test: `./runheadless.native.sh` from inside the container → verify Omniverse starts without crashing
4. URDF → USD conversion — headless Python script, output to `mycobot_description/usd/`
5. Open `feature/isaac-sim` branch, commit the USD asset + a minimal launch script

**Ask before:** installing CUDA updates, pulling multi-GB containers on metered connections, modifying the NVIDIA driver, adding a `feature/isaac-sim` branch to the repo.

## Docs to produce alongside integration

- `docs/ISAAC_SIM.md` — mirror of `docs/ARCHITECTURE.md` for the Isaac path
- A comparison entry in `CHANGELOG.md` once Phase 3 metrics are in hand (real-data detection delta)

## References

- Isaac Sim docs: `https://docs.omniverse.nvidia.com/isaacsim/latest/`
- Isaac Lab (formerly Orbit): `https://isaac-sim.github.io/IsaacLab/`
- URDF importer guide: `https://docs.omniverse.nvidia.com/isaacsim/latest/ext_omni_isaac_urdf.html`
- DREAM keypoint export: [`training/dream/convert_to_ndds.py`](../../../training/dream/convert_to_ndds.py)
