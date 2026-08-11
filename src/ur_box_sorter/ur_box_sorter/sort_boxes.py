import sys, subprocess, threading, time, re
import numpy as np


# ---------- kinematics: verified <1mm vs sim; generates exact joint goals -----
D1, A2, A3, D4, D5, D6 = 0.1625, -0.425, -0.3922, 0.1333, 0.0997, 0.0996
ALPHA = [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]
DH_A = [0, A2, A3, 0, 0, 0]
DH_D = [D1, 0, 0, D4, D5, D6]

def _dh(t, d, a, al):
    ct, st, ca, sa = np.cos(t), np.sin(t), np.cos(al), np.sin(al)
    return np.array([[ct, -st*ca, st*sa, a*ct], [st, ct*ca, -ct*sa, a*st],
                     [0, sa, ca, d], [0, 0, 0, 1.0]])

def fk(q):
    T = np.array([[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])
    for i in range(6):
        T = T @ _dh(q[i], DH_D[i], DH_A[i], ALPHA[i])
    return T

def ik(target, seed, iters=400):
    q = np.array(seed, float); lam = 0.05
    tgt = np.array(target, float)
    for _ in range(iters):
        T = fk(q); p = T[:3, 3]; zt = T[:3, 2]
        e = np.concatenate([tgt - p, (np.array([0, 0, -1.0]) - zt)[:2]])
        if np.linalg.norm(e[:3]) < 5e-5 and abs(zt[2] + 1) < 1e-3:
            break
        J = np.zeros((5, 6)); h = 1e-6
        for j in range(6):
            dq = q.copy(); dq[j] += h; Td = fk(dq)
            J[:3, j] = (Td[:3, 3] - p) / h
            J[3:, j] = -((Td[:3, 2] - zt) / h)[:2]
        J[3:, :] *= -1
        q = q + np.clip(J.T @ np.linalg.solve(J @ J.T + lam*np.eye(5), e), -0.2, 0.2)
        q = np.arctan2(np.sin(q), np.cos(q))
    T = fk(q)
    return q, float(np.linalg.norm(tgt - T[:3, 3])), float(np.linalg.norm(np.array([0, 0, -1.0]) - T[:3, 2]))

def unwrap(q, ref):
    q = np.array(q, float)
    return q + 2*np.pi*np.round((np.array(ref) - q) / (2*np.pi))

def solve_chained(target, seed):
    q, pe, ae = ik(target, seed)
    if pe < 1e-3 and ae < 5e-3:
        return unwrap(q, seed)
    base = np.array([0.0, -1.2, 1.4, -1.8, -1.57, 0.0])
    rng = np.random.default_rng(2)
    for _ in range(30):
        q, pe, ae = ik(target, base + rng.uniform(-0.9, 0.9, 6))
        if pe < 1e-3 and ae < 5e-3:
            return unwrap(q, seed)
    return None

# ---------- perception --------------------------------------------------------
GRIPPER, MARGIN, CELL, MINPTS = 0.08, 0.02, 0.01, 100
CAM = np.array([0.5, 0.0, 0.6])
BOX_MODELS = ["box_small", "box_medium", "box_large"]

def find_boxes(pts):
    spreads = pts.max(0) - pts.min(0)
    dax = int(np.argmin(spreads)); lat = [i for i in range(3) if i != dax]
    d = pts[:, dax]; ground = np.median(d)
    above = pts[np.abs(d - ground) > MARGIN]
    if above.shape[0] < MINPTS:
        return []
    uv = above[:, lat]; cells = np.floor((uv - uv.min(0)) / CELL).astype(np.int64)
    W = int(cells[:, 1].max()) + 1; keys = cells[:, 0]*W + cells[:, 1]
    occ = set(int(k) for k in np.unique(keys)); lab = {}; n = 0
    for k in occ:
        if k in lab: continue
        n += 1; stack = [k]; lab[k] = n
        while stack:
            ci, cj = divmod(stack.pop(), W)
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    nb = (ci+di)*W + (cj+dj)
                    if (di or dj) and 0 <= cj+dj < W and nb in occ and nb not in lab:
                        lab[nb] = n; stack.append(nb)
    pl = np.array([lab[int(k)] for k in keys]); out = []
    for L in range(1, n+1):
        sel = above[pl == L]
        if sel.shape[0] < MINPTS: continue
        dims = sel.max(0) - sel.min(0)
        w1, w2 = float(dims[lat[0]]), float(dims[lat[1]])
        top = float(np.median(sel[:, dax])); c = sel.mean(0)
        wx, wy, wz = CAM[0] + c[lat[1]], CAM[1] + c[lat[0]], CAM[2] - top
        out.append({"x": wx, "y": wy, "top": wz, "h": abs(ground - top),
                    "w": min(w1, w2), "w1": w1, "w2": w2,
                    "graspable": min(w1, w2) < GRIPPER})
    return out

# ---------- gz introspection & scene tools ------------------------------------
POSE_RE = re.compile(r"\[\s*([-\d.eE]+)\s+([-\d.eE]+)\s+([-\d.eE]+)\s*\]")

def parse_model_pose(text):
    m = POSE_RE.search(text)
    return (float(m.group(1)), float(m.group(2)), float(m.group(3))) if m else None

def gz_model_pose(name):
    r = subprocess.run(["gz", "model", "-m", name, "-p"],
                       capture_output=True, text=True, timeout=5)
    return parse_model_pose(r.stdout)

def live_name(b, exclude=()):
    best, bd = None, 0.08
    for name in BOX_MODELS:
        if name in exclude:
            continue
        p = gz_model_pose(name)
        if p is None:
            continue
        d = ((b["x"]-p[0])**2 + (b["y"]-p[1])**2) ** 0.5
        if d < bd:
            best, bd = name, d
    return best

def set_pose(name, x, y, z):
    subprocess.run(["gz", "service", "-s", "/world/empty/set_pose",
        "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
        "--timeout", "300", "--req",
        f'name: "{name}", position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}'],
        capture_output=True)

def weld(name, on):
    suffix = "small" if "small" in name else "medium"
    topic = f"/gripper/attach_{suffix}" if on else f"/gripper/detach_{suffix}"
    subprocess.run(["gz", "topic", "-t", topic, "-m", "gz.msgs.Empty",
                    "-p", "unused: true"], capture_output=True)

def sample_shuffle(rng, sizes, xr=(0.30, 0.70), yr=(-0.18, 0.18), min_sep=0.16):
    spots = []
    for s in sizes:
        for _ in range(300):
            x = rng.uniform(*xr); y = rng.uniform(*yr)
            if all(((x-a)**2 + (y-b)**2) ** 0.5 > min_sep for a, b, _ in spots):
                spots.append((x, y, s/2 + 0.001)); break
        else:
            return None
    return spots

# ---------- ROS section -------------------------------------------------------
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import PointCloud2, JointState
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float64MultiArray
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.msg import PlanningScene, CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose as GeomPose

GROUP = "ur_manipulator"
JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
HOME = [0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0]

FINGER_GAP_OPEN = 0.080
FINGER_MAX_TRAVEL = 0.028
LIFT = (0.45, 0.00, 0.40)
CONT_XY = (0.00, 0.45)
SLOTS = [-0.05, 0.0, 0.05]

class Sorter(Node):
    def __init__(self):
        super().__init__("sorter")
        self.cloud = None; self.cloud_t = None; self.qnow = None
        self.create_subscription(PointCloud2, "/rgbd/points", self.on_cloud, 5)
        self.create_subscription(JointState, "/joint_states", self.on_js, 20)
        self.mg = ActionClient(self, MoveGroup, "/move_action")
        self.grip_pub = self.create_publisher(
            Float64MultiArray, "/gripper_position_controller/commands", 5)
        self.scene_cli = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.obstacle_ids = []

    def on_cloud(self, msg):
        self.cloud = msg; self.cloud_t = time.monotonic()

    def on_js(self, msg):
        names = list(msg.name)
        self.qnow = [msg.position[names.index(j)] for j in JOINTS]

    # ----- MoveIt goals -----
    def _send(self, req, label):
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        done = threading.Event(); out = {"ok": False}
        def on_result(f):
            code = f.result().result.error_code.val
            out["ok"] = (code == 1)
            if not out["ok"]:
                self.get_logger().error(f"[moveit] {label} failed, error_code={code}")
            done.set()
        def on_accept(f):
            gh = f.result()
            if not gh.accepted:
                self.get_logger().error(f"[moveit] {label} goal rejected"); done.set(); return
            gh.get_result_async().add_done_callback(on_result)
        self.mg.send_goal_async(goal).add_done_callback(on_accept)
        done.wait(timeout=40.0)
        time.sleep(0.2)
        return out["ok"]

    def _base_request(self):
        req = MoveGroup.Goal().request
        req.group_name = GROUP
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.35
        req.max_acceleration_scaling_factor = 0.35
        return req

    def mp_pose(self, x, y, z, label="move"):
        seed = self.qnow if self.qnow is not None else [0.0, -1.2, 1.4, -1.8, -1.57, 0.0]
        qt = solve_chained((x, y, z), seed)
        if qt is None:
            self.get_logger().error(f"[ik] no solution for {label} ({x:+.2f},{y:+.2f},{z:.2f})")
            return False
        lbl = f"{label} ({x:+.2f},{y:+.2f},{z:.2f})"
        for attempt in (1, 2):
            if self.mp_joints([float(v) for v in qt], lbl):
                return True
            self.get_logger().warn(f"[moveit] retrying {lbl} ({attempt}/2)")
        return False

    def mp_joints(self, q, label="joints"):
        req = self._base_request()
        c = Constraints()
        for name, val in zip(JOINTS, q):
            jc = JointConstraint()
            jc.joint_name = name; jc.position = float(val)
            jc.tolerance_above = 0.01; jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints = [c]
        return self._send(req, label)

    # ----- perception / gripper (unchanged) -----
    def fresh_scan(self, timeout=6.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self.cloud_t is not None and self.cloud_t > t0 + 0.3:
                pts = point_cloud2.read_points_numpy(
                    self.cloud, field_names=["x", "y", "z"], skip_nans=True)
                pts = pts[np.isfinite(pts).all(axis=1)]
                if pts.shape[0] > 1000:
                    return find_boxes(pts)
            time.sleep(0.1)
        self.get_logger().warn("no fresh cloud; using last known")
        if self.cloud is None:
            return []
        pts = point_cloud2.read_points_numpy(
            self.cloud, field_names=["x", "y", "z"], skip_nans=True)
        return find_boxes(pts[np.isfinite(pts).all(axis=1)])

    def grip_to_width(self, width):
        travel = float(np.clip((FINGER_GAP_OPEN - width)/2.0 - 0.0005,
                               0.0, FINGER_MAX_TRAVEL))
        self.grip_pub.publish(Float64MultiArray(data=[travel, travel]))
        time.sleep(0.8)
        return travel

    def grip_open(self):
        self.grip_pub.publish(Float64MultiArray(data=[0.0, 0.0]))
        time.sleep(0.5)

    # ----- planning scene (M3: perception -> collision objects) -----
    def _apply_scene(self, cobjs, label):
        ps = PlanningScene(); ps.is_diff = True; ps.robot_state.is_diff = True
        ps.world.collision_objects = cobjs
        req = ApplyPlanningScene.Request(); req.scene = ps
        done = threading.Event(); out = {"ok": False}
        def cb(f):
            try: out["ok"] = f.result().success
            except Exception: out["ok"] = False
            done.set()
        self.scene_cli.call_async(req).add_done_callback(cb)
        done.wait(timeout=5.0)
        if not out["ok"]:
            self.get_logger().warn(f"[scene] {label} not acknowledged")
        return out["ok"]

    @staticmethod
    def _cobj(cid, dims, x, y, z, op):
        co = CollisionObject()
        co.header.frame_id = "base_link"
        co.id = cid; co.operation = op
        sp = SolidPrimitive(type=SolidPrimitive.BOX,
                            dimensions=[float(d) for d in dims])
        pose = GeomPose(); pose.position.x = float(x)
        pose.position.y = float(y); pose.position.z = float(z)
        pose.orientation.w = 1.0
        co.primitives = [sp]; co.primitive_poses = [pose]
        return co

    def publish_static_obstacles(self):
        objs = [self._cobj("camera_rig", (0.10, 0.10, 0.08), 0.5, 0.0, 0.60,
                           CollisionObject.ADD),
                self._cobj("container_floor", (0.18, 0.18, 0.01), 0.0, 0.45, 0.005,
                           CollisionObject.ADD)]
        for i, (dx, dy) in enumerate([(0.085, 0), (-0.085, 0), (0, 0.085), (0, -0.085)]):
            dims = (0.01, 0.18, 0.10) if dx else (0.18, 0.01, 0.10)
            objs.append(self._cobj(f"container_w{i}", dims, dx, 0.45 + dy, 0.06,
                                   CollisionObject.ADD))
        self._apply_scene(objs, "static obstacles")

    def publish_box_obstacles(self, boxes):
        if self.obstacle_ids:      # best-effort clear; a stale id must not veto adds
            rem = [self._cobj(cid, (0.01, 0.01, 0.01), 0, 0, -1,
                              CollisionObject.REMOVE) for cid in self.obstacle_ids]
            self._apply_scene(rem, "clear old obstacles")
        self.obstacle_ids = []
        adds = []
        for i, b in enumerate(boxes):
            cid = f"obs_{i}"
            adds.append(self._cobj(cid, (b["w1"] + 0.01, b["w2"] + 0.01, b["h"]),
                                   b["x"], b["y"], b["top"] - b["h"]/2.0,
                                   CollisionObject.ADD))
            self.obstacle_ids.append(cid)
        if adds and self._apply_scene(adds, f"{len(adds)} box obstacles"):
            self.get_logger().info(f"    [scene] {len(adds)} obstacles live in planner")

    def remove_obstacle(self, idx):
        cid = f"obs_{idx}"
        if cid in self.obstacle_ids:
            self._apply_scene([self._cobj(cid, (0.01,)*3, 0, 0, -1,
                                          CollisionObject.REMOVE)], f"allow {cid}")
            self.obstacle_ids.remove(cid)


def mission(node: Sorter):
    log = node.get_logger()
    log.info("waiting for MoveIt (/move_action)...")
    if not node.mg.wait_for_server(timeout_sec=15.0):
        log.error("NO MOVEIT — did start_robot.sh step [3c] pass?"); return
    log.info("waiting for camera + joint states...")
    t0 = time.time()
    while node.cloud is None or node.qnow is None:
        time.sleep(0.2)
        if time.time() - t0 > 10.0:
            log.error(f"MISSING: camera={node.cloud is None} joints={node.qnow is None}")
            return
    if not node.scene_cli.wait_for_service(timeout_sec=10.0):
        log.error("planning-scene service missing"); return
    node.publish_static_obstacles()
    log.info("all inputs ready — planning via MoveIt, scene has container+camera")
    node.grip_open()
    weld("box_small", False); weld("box_medium", False)

    if "--shuffle" in sys.argv:
        rng = np.random.default_rng()
        sizes = {"box_small": 0.04, "box_medium": 0.06, "box_large": 0.12}
        spots = sample_shuffle(rng, list(sizes.values()))
        if spots:
            for (name, _), (x, y, z) in zip(sizes.items(), spots):
                set_pose(name, x, y, z)
                log.info(f"  shuffled {name} -> ({x:+.2f}, {y:+.2f})")
            time.sleep(1.0)
        else:
            log.warn("shuffle sampling failed; keeping current layout")

    if "--roadblock" in sys.argv:
        set_pose("box_large", 0.51, -0.10, 0.061)
        log.info("  ROADBLOCK: big blue box parked in the approach corridor (0.51,-0.10)")
        time.sleep(1.0)

    placed, rejected_logged, picks = set(), False, 0
    while True:
        if not node.mp_joints(HOME, "home for scan"):
            log.error("cannot reach scan pose — aborting"); return
        time.sleep(0.3)
        boxes = node.fresh_scan()
        node.publish_box_obstacles(boxes)
        grasp = sorted(range(len(boxes)), key=lambda i: boxes[i]["w"])
        grasp = [i for i in grasp if boxes[i]["graspable"]]
        skip = [boxes[i] for i in range(len(boxes)) if not boxes[i]["graspable"]]
        if not rejected_logged:
            log.info(f"see {len(boxes)} boxes: {len(grasp)} graspable, {len(skip)} too big")
            for b in skip:
                log.info(f"  skipping {b['w']:.3f} m box at ({b['x']:+.2f},{b['y']:+.2f}) — wider than gripper")
            rejected_logged = True
        if not grasp:
            log.info("no graspable boxes remain — wrapping up")
            break
        bi = grasp[0]; b = boxes[bi]
        name = live_name(b, exclude=placed)
        if name is None or name == "box_large":
            log.warn(f"  cannot resolve weld rig for box at ({b['x']:+.2f},{b['y']:+.2f}) — skipping")
            break
        log.info(f"--> picking {name} ({b['w']:.3f} m) at ({b['x']:+.2f},{b['y']:+.2f})  [rescanned]")
        node.remove_obstacle(bi)          # the target is a goal, not an obstacle
        ok = (node.mp_pose(b["x"], b["y"], 0.25, label="hover")
              and node.mp_pose(b["x"], b["y"], b["top"] + 0.05, label="descend"))
        if not ok:
            log.error("approach failed — aborting safely"); return
        travel = node.grip_to_width(b["w"])
        log.info(f"    fingers closed to {FINGER_GAP_OPEN - 2*travel:.3f} m gap")
        weld(name, True)
        log.info(f"    WELDED {name} to hand")
        node.publish_box_obstacles([])   # carry rides high above all boxes; clear to kill margin-zero starts
        slot = SLOTS[picks % len(SLOTS)]
        ok = (node.mp_pose(b["x"], b["y"], 0.32, label="raise")
              and node.mp_pose(CONT_XY[0] + slot, CONT_XY[1], 0.35, label="to container")
              and node.mp_pose(CONT_XY[0] + slot, CONT_XY[1], 0.22, label="lower"))
        if not ok:
            log.error("carry failed — releasing here")
            weld(name, False); node.grip_open(); return
        weld(name, False)
        node.grip_open()
        log.info(f"    released {name} — dropped into container")
        placed.add(name); picks += 1
        node.mp_pose(CONT_XY[0] + slot, CONT_XY[1], 0.35, label="ascend")
    node.publish_box_obstacles([])     # leave the planner scene matching reality
    node.mp_joints(HOME, "final home")
    log.info(f"MISSION COMPLETE (MoveIt + collision-aware): {picks} boxes sorted; "
             f"planner avoided every published obstacle")

def main():
    rclpy.init()
    node = Sorter()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    try:
        mission(node)
    finally:
        rclpy.shutdown()
        spin.join(timeout=2.0)
        node.destroy_node()

if __name__ == "__main__":
    main()
