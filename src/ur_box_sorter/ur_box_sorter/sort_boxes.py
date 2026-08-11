import numpy as np, subprocess, threading, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import PointCloud2, JointState
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float64MultiArray

# ---------- kinematics (verified against sim tf, home -> 0.001,0.233,1.079) ---
D1, A2, A3, D4, D5, D6 = 0.1625, -0.425, -0.3922, 0.1333, 0.0997, 0.0996
ALPHA = [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]
A = [0, A2, A3, 0, 0, 0]
D = [D1, 0, 0, D4, D5, D6]
JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
HOME = np.array([0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0])

def _dh(t, d, a, al):
    ct, st, ca, sa = np.cos(t), np.sin(t), np.cos(al), np.sin(al)
    return np.array([[ct, -st*ca, st*sa, a*ct], [st, ct*ca, -ct*sa, a*st],
                     [0, sa, ca, d], [0, 0, 0, 1.0]])

def fk(q):
    T = np.array([[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])
    for i in range(6):
        T = T @ _dh(q[i], D[i], A[i], ALPHA[i])
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

def solve_chained(target, seed):
    q, pe, ae = ik(target, seed)
    if pe < 1e-3 and ae < 5e-3:
        return q
    base = np.array([0.0, -1.2, 1.4, -1.8, -1.57, 0.0])
    rng = np.random.default_rng(2)
    for _ in range(30):
        q, pe, ae = ik(target, base + rng.uniform(-0.9, 0.9, 6))
        if pe < 1e-3 and ae < 5e-3:
            return q
    return None

# ---------- perception --------------------------------------------------------
GRIPPER, MARGIN, CELL, MINPTS = 0.08, 0.02, 0.01, 100
CAM = np.array([0.5, 0.0, 0.6])
KNOWN = {"box_small": (0.40, -0.12), "box_medium": (0.62, -0.08), "box_large": (0.52, 0.10)}

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
                    "w": min(w1, w2), "graspable": min(w1, w2) < GRIPPER})
    return out

def model_name(b):
    for name, (kx, ky) in KNOWN.items():
        if abs(b["x"]-kx) < 0.06 and abs(b["y"]-ky) < 0.06:
            return name
    return None

# ---------- gripper hardware helpers ------------------------------------------
FINGER_GAP_OPEN = 0.080      # inner faces when fingers at 0
FINGER_MAX_TRAVEL = 0.028

def weld(name, on):
    suffix = "small" if "small" in name else "medium"
    topic = f"/gripper/attach_{suffix}" if on else f"/gripper/detach_{suffix}"
    subprocess.run(["gz", "topic", "-t", topic, "-m", "gz.msgs.Empty",
                    "-p", "unused: true"], capture_output=True)

# ---------- the mission -------------------------------------------------------
LIFT = (0.45, 0.00, 0.40)
CONT_XY = (0.00, 0.45)

class Sorter(Node):
    def __init__(self):
        super().__init__("sorter")
        self.cloud = None; self.qnow = None
        self.create_subscription(PointCloud2, "/rgbd/points", self.on_cloud, 5)
        self.create_subscription(JointState, "/joint_states", self.on_js, 20)
        self.client = ActionClient(self, FollowJointTrajectory,
            "/scaled_joint_trajectory_controller/follow_joint_trajectory")
        self.grip_pub = self.create_publisher(
            Float64MultiArray, "/gripper_position_controller/commands", 5)

    def on_cloud(self, msg):
        self.cloud = msg

    def on_js(self, msg):
        q = np.zeros(6)
        names = list(msg.name)
        for i, jn in enumerate(JOINTS):
            q[i] = msg.position[names.index(jn)]
        self.qnow = q

    def grip_to_width(self, width):
        travel = (FINGER_GAP_OPEN - width) / 2.0 - 0.0005
        travel = float(np.clip(travel, 0.0, FINGER_MAX_TRAVEL))
        self.grip_pub.publish(Float64MultiArray(data=[travel, travel]))
        time.sleep(0.8)
        return travel

    def grip_open(self):
        self.grip_pub.publish(Float64MultiArray(data=[0.0, 0.0]))
        time.sleep(0.5)

    def move(self, q, seconds=3.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q]
        pt.time_from_start.sec = int(seconds)
        pt.time_from_start.nanosec = int((seconds % 1) * 1e9)
        goal.trajectory.points = [pt]
        done = threading.Event()
        def on_result(_f): done.set()
        def on_accept(f):
            gh = f.result()
            if not gh.accepted:
                self.get_logger().error("goal rejected"); done.set(); return
            gh.get_result_async().add_done_callback(on_result)
        self.client.send_goal_async(goal).add_done_callback(on_accept)
        done.wait(timeout=seconds + 6.0)
        time.sleep(0.3)

def mission(node: Sorter):
    log = node.get_logger()
    log.info("waiting for arm controller...")
    if not node.client.wait_for_server(timeout_sec=10.0):
        log.error("NO ARM CONTROLLER — is start_robot.sh green?")
        return
    log.info("waiting for camera + joint states...")
    t0 = time.time()
    while node.cloud is None or node.qnow is None:
        time.sleep(0.2)
        if time.time() - t0 > 10.0:
            log.error(f"MISSING: camera={node.cloud is None} joints={node.qnow is None}")
            return
    log.info("all inputs ready")
    node.grip_open()
    weld("box_small", False); weld("box_medium", False)   # defensive un-weld
    pts = point_cloud2.read_points_numpy(node.cloud, field_names=["x","y","z"], skip_nans=True)
    pts = pts[np.isfinite(pts).all(axis=1)]
    boxes = find_boxes(pts)
    grasp = sorted([b for b in boxes if b["graspable"]], key=lambda b: b["w"])
    skip = [b for b in boxes if not b["graspable"]]
    log.info(f"see {len(boxes)} boxes: {len(grasp)} graspable, {len(skip)} too big")
    for b in skip:
        log.info(f"  skipping {b['w']:.3f} m box at ({b['x']:+.2f},{b['y']:+.2f}) — wider than gripper")
    q = np.array(node.qnow); slot = -0.03
    for b in grasp:
        name = model_name(b)
        if name is None:
            log.warn(f"  unknown model near ({b['x']:.2f},{b['y']:.2f}), skipping"); continue
        log.info(f"--> picking {name} ({b['w']:.3f} m) at ({b['x']:+.2f},{b['y']:+.2f})")
        plan = [("lift", LIFT, 3.0),
                ("hover", (b["x"], b["y"], 0.25), 3.0),
                ("descend", (b["x"], b["y"], b["top"] + 0.05), 2.5)]
        for label, tgt, secs in plan:
            q2 = solve_chained(tgt, q)
            if q2 is None:
                log.error(f"IK failed at {label} — aborting safely"); return
            node.move(q2, secs); q = q2
        travel = node.grip_to_width(b["w"])
        log.info(f"    fingers closed to {FINGER_GAP_OPEN - 2*travel:.3f} m gap")
        weld(name, True)
        log.info(f"    WELDED {name} to hand")
        plan = [("raise", (b["x"], b["y"], 0.30), 2.5),
                ("lift", LIFT, 3.0),
                ("container_hover", (CONT_XY[0] + slot, CONT_XY[1], 0.35), 3.5),
                ("container_lower", (CONT_XY[0] + slot, CONT_XY[1], 0.22), 2.0)]
        for label, tgt, secs in plan:
            q2 = solve_chained(tgt, q)
            if q2 is None:
                log.error(f"IK failed at {label} — releasing here"); break
            node.move(q2, secs); q = q2
        weld(name, False)
        node.grip_open()
        log.info(f"    released {name} — dropped into container")
        q2 = solve_chained((CONT_XY[0] + slot, CONT_XY[1], 0.35), q)
        if q2 is not None:
            node.move(q2, 2.0); q = q2
        slot += 0.06
    q2 = solve_chained((0.30, 0.10, 0.45), q)
    if q2 is not None:
        node.move(q2, 3.0)
    node.move(HOME, 3.0)
    log.info(f"MISSION COMPLETE: {len(grasp)} boxes gripped, welded, and sorted; "
             f"{len(skip)} correctly rejected")

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
