# ur5e-box-sorter

Vision-guided pick-and-place sorting with a UR5e in Gazebo Harmonic (ROS 2 Jazzy).
An overhead RGB-D camera watches a table of boxes. The robot measures each box,
decides which ones fit its 80 mm gripper opening, picks the graspable ones, and
places them in a container — skipping the oversized ones. No box positions or
sizes are hardcoded: the robot re-measures the scene every run.

![demo](demo.gif)

Actual mission log:

```
see 3 boxes: 2 graspable, 1 too big
  skipping 0.120 m box at (+0.52,+0.10) — wider than gripper
--> picking box_small (0.040 m) at (+0.40,-0.12)
    grab (magic hand on)
    released box_small into containerde
--> picking box_medium (0.060 m) at (+0.62,-0.08)
    grab (magic hand on)
    released box_medium into container
MISSION COMPLETE: 2 boxes sorted into container, 1 correctly rejected
```

## How it works

1. **Perceive** — `/rgbd/points` cloud: auto-detect the depth axis, remove the
   ground plane (median depth), split remaining points into boxes via 1 cm-grid
   connected-components clustering; per box: footprint, height (top-vs-ground
   depth difference), center, all converted to the robot's world frame.
2. **Decide** — graspable if min footprint < 0.08 m.
3. **Reach** — forward kinematics written from the published UR5e DH parameters;
   inverse kinematics via damped least squares with a tool-pointing-down
   constraint. Verified against the simulator to < 1 mm. Waypoints are solved
   with chained seeding so the arm never flips configuration mid-path.
4. **Hold** — "magic hand": while carrying, the box pose is servoed to the wrist
   through Gazebo's `set_pose` service. A deliberate, disclosed sim shortcut —
   a physical gripper is the roadmap's next step.
5. **Place** — released into the container; arm homes and reports.

## Built here vs reused

Built from scratch: the world (`ur_sensor_world.sdf`: boxes, container, RGB-D
rig), the perception node, FK/IK, the mission choreography and magic hand, and
the ops tooling (camera health gate, EGL workaround).
Reused: UR5e model and controllers (`ur_simulation_gz`), `ros_gz` bridge,
ROS 2 / Gazebo themselves.

## Run

Prereqs: Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic,
`ros-jazzy-ur-simulation-gz`, `ros-jazzy-ros-gz`.

    colcon build --packages-select ur_box_sorter && source install/setup.bash
    bash start_robot.sh        # clean start: world + arm + camera gate + bridge
    ros2 run ur_box_sorter sort_boxes

Utilities: `ros2 run ur_box_sorter measure_boxes` (perception only),
`ros2 run ur_box_sorter wave_ur` (arm smoke test), `bash stop_robot.sh`.

## Layout

    ur_sensor_world.sdf    the entire scene: arm world, boxes, container, camera
    start_robot.sh         ordered bring-up with a camera health gate
    stop_robot.sh          full teardown
    src/ur_box_sorter/     sort_boxes (mission), measure_boxes, wave_ur

## Roadmap

Smooth fast hold (orientation lock, low-latency following); a visible custom
two-finger gripper via URDF/xacro with attach-based holding; true
contact-physics grasping; MoveIt collision-aware planning; re-scan between
picks and randomized scenes.

## Field notes

- RTX 5090 (Blackwell) + mesa EGL: the sensor render engine crashes or hangs
  (`egl: failed to create dri2 screen`, `driver (null)`). Fix: route EGL to the
  NVIDIA implementation via `__EGL_VENDOR_LIBRARY_FILENAMES` — baked into
  `start_robot.sh`.
- The camera gate: bring-up refuses to continue unless `/rgbd/points` actually
  streams — GUI rendering proves nothing about sensor rendering.
- `ros_gz_sim create` silently ignores the pose inside an SDF (drops models at
  the origin). Models live in the world file instead, where poses are honored.
- Bridges must start after Gazebo; a bridge that outlives a Gazebo restart is
  dead weight.
