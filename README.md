# ur5e-box-sorter

Vision-guided pick-and-place sorting with a UR5e in Gazebo Harmonic (ROS 2 Jazzy).
An overhead RGB-D camera watches a table of boxes. The robot measures each box,
decides which ones fit its 80 mm gripper opening, picks the graspable ones, and
places them in a container — skipping the oversized ones. No box positions or
sizes are hardcoded: the robot re-measures the scene every run.

## Demo

**v2 (current) — custom adaptive gripper + weld-based hold.** The fingers close
to each box's *measured* width (4 cm box → 0.041 m gap, 6 cm box → 0.061 m gap),
the box is welded rigidly to the hand for a zero-slip carry, and released to
drop into the container under gravity:

![demo v2](demo_v2.gif)

**v1 — first complete mission (magic-hand hold) — ✅ completed milestone,
now retired:**

![demo v1](demo.gif)

v2 mission log:

```
see 3 boxes: 2 graspable, 1 too big
  skipping 0.120 m box at (+0.52,+0.10) — wider than gripper
--> picking box_small (0.040 m) at (+0.40,-0.12)
    grab (magic hand on)
    released box_small into containerde
--> picking box_medium (0.060 m) at (+0.62,-0.08)
    fingers closed to 0.061 m gap
    WELDED box_medium to hand
    released box_medium — dropped into container
MISSION COMPLETE: 2 boxes gripped, welded, and sorted; 1 correctly rejected
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
4. **Grip** — a custom two-finger gripper added to the UR5e through
   robot-description surgery (`ur5e_gripper.urdf.xacro` wraps the shipped UR
   description; no upstream files modified). Prismatic fingers under a
   `JointGroupPositionController` close to the width perception measured —
   an adaptive grip, different for every box.
5. **Hold** — at grasp, a Gazebo detachable joint welds the box to the wrist:
   rigid, zero-slip carry with real dynamics. At release the weld opens, the
   fingers part, and the box falls the last centimeters into the container by
   gravity. (v1 used a disclosed `set_pose` "magic hand" teleport; the weld
   retired it.)
6. **Place** — released into the container; arm homes and reports.

## Built here vs reused

Built from scratch: the world (`ur_sensor_world.sdf`: boxes, container, RGB-D
rig), the perception node, FK/IK, the mission choreography, the gripper
(xacro description, controller config, detachable-joint weld integration), and
the ops tooling (camera health gate, EGL workaround, ordered bring-up).
Reused: UR5e model and controllers (`ur_simulation_gz`), `ros_gz` bridge,
ROS 2 / Gazebo themselves.

## Run

Prereqs: Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic,
`ros-jazzy-ur-simulation-gz`, `ros-jazzy-ros-gz`.

    colcon build --packages-select ur_box_sorter && source install/setup.bash
    bash start_robot.sh        # clean start: world + gripper-arm + camera gate + bridge
    ros2 run ur_box_sorter sort_boxes

Utilities: `ros2 run ur_box_sorter measure_boxes` (perception only),
`ros2 run ur_box_sorter wave_ur` (arm smoke test), `bash stop_robot.sh`.

## Layout

    ur_sensor_world.sdf           the entire scene: boxes, container, camera
    ur5e_gripper.urdf.xacro       UR5e + custom gripper + weld plugins
    ur_gripper_controllers.yaml   arm controllers + gripper position controller
    start_robot.sh                ordered bring-up: EGL fix, gates, controllers
    stop_robot.sh                 full teardown
    src/ur_box_sorter/            sort_boxes (mission), measure_boxes, wave_ur

## Roadmap

- [x] v1 — full see → decide → pick → place mission (magic-hand hold)
- [x] Custom two-finger gripper via URDF/xacro, adaptive close-to-width
- [x] Rigid weld-based hold (detachable joint) — retired the magic hand,
      eliminated carry slip entirely
- [ ] True contact-physics grasping (finger friction does the holding)
- [ ] Smooth blended multi-waypoint trajectories
- [ ] MoveIt collision-aware planning
- [ ] Re-scan between picks; randomized box scenes

## Field notes

- RTX 5090 (Blackwell) + mesa EGL: the sensor render engine crashes or hangs
  (`egl: failed to create dri2 screen`, `driver (null)`). Fix: route EGL to the
  NVIDIA implementation via `__EGL_VENDOR_LIBRARY_FILENAMES` — baked into
  `start_robot.sh`.
- A colon inside an XML comment killed the robot: xacro preserves comments into
  the expanded URDF, and the launch stack YAML-scans `robot_description` — a
  comment containing `: ` raises a YAML ScannerError and the launch dies
  instantly. Keep `: ` out of xacro comments.
- Fixed-joint lumping: Gazebo merges links joined by fixed joints into their
  parent. Our gripper palm dissolved into `wrist_3_link`, so the detachable
  joint's `parent_link` must name the *surviving* link, not the URDF child.
- Negative prismatic axes misbehave: a finger on axis `(0,-1,0)` read back
  `-0.0000` and never tracked its command. Rotate the joint frame 180° and keep
  the axis positive instead.
- The camera gate: bring-up refuses to continue unless `/rgbd/points` actually
  streams — GUI rendering proves nothing about sensor rendering.
- `ros_gz_sim create` silently ignores the pose inside an SDF (drops models at
  the origin). Models live in the world file instead, where poses are honored.
- Bridges must start after Gazebo; a bridge that outlives a Gazebo restart is
  dead weight.
