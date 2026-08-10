import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

GRIPPER_OPENING = 0.08
GROUND_MARGIN = 0.02
CELL = 0.01
MIN_POINTS = 100
CAM_X, CAM_Y, CAM_Z = 0.5, 0.0, 0.6   # camera pose from the world file


def find_boxes(pts):
    spreads = pts.max(0) - pts.min(0)
    depth_ax = int(np.argmin(spreads))
    lat = [i for i in range(3) if i != depth_ax]
    d = pts[:, depth_ax]
    ground = np.median(d)
    above = pts[np.abs(d - ground) > GROUND_MARGIN]
    if above.shape[0] < MIN_POINTS:
        return [], depth_ax, lat
    uv = above[:, lat]
    mins = uv.min(0)
    cells = np.floor((uv - mins) / CELL).astype(np.int64)
    W = int(cells[:, 1].max()) + 1
    keys = cells[:, 0] * W + cells[:, 1]
    occupied = set(int(k) for k in np.unique(keys))
    label_of = {}
    n_labels = 0
    for k in occupied:
        if k in label_of:
            continue
        n_labels += 1
        stack = [k]
        label_of[k] = n_labels
        while stack:
            cur = stack.pop()
            ci, cj = divmod(cur, W)
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    if di == 0 and dj == 0:
                        continue
                    nb = (ci + di) * W + (cj + dj)
                    if 0 <= cj + dj < W and nb in occupied and nb not in label_of:
                        label_of[nb] = n_labels
                        stack.append(nb)
    point_labels = np.array([label_of[int(k)] for k in keys])
    boxes = []
    for lbl in range(1, n_labels + 1):
        sel = above[point_labels == lbl]
        if sel.shape[0] < MIN_POINTS:
            continue
        dims = sel.max(0) - sel.min(0)
        w1, w2 = float(dims[lat[0]]), float(dims[lat[1]])
        top = float(np.median(sel[:, depth_ax]))
        center = sel.mean(0)
        center[depth_ax] = top
        boxes.append({"center": center, "w1": w1, "w2": w2,
                      "height": abs(ground - top), "n": int(sel.shape[0]),
                      "graspable": min(w1, w2) < GRIPPER_OPENING})
    boxes.sort(key=lambda b: -b["n"])
    return boxes, depth_ax, lat


def cam_to_world(c, depth_ax, lat):
    depth = c[depth_ax]
    a, b = c[lat[0]], c[lat[1]]
    wx = CAM_X + b
    wy = CAM_Y + a
    wz = CAM_Z - depth
    return wx, wy, wz


class BoxesMeasurer(Node):
    def __init__(self):
        super().__init__("boxes_measurer")
        self.sub = self.create_subscription(
            PointCloud2, "/rgbd/points", self.on_cloud, 10)
        self.done = False

    def on_cloud(self, msg):
        if self.done:
            return
        pts = point_cloud2.read_points_numpy(
            msg, field_names=["x", "y", "z"], skip_nans=True)
        pts = pts[np.isfinite(pts).all(axis=1)]
        if pts.shape[0] < 1000:
            return
        boxes, depth_ax, lat = find_boxes(pts)
        if not boxes:
            self.get_logger().warn("no boxes found yet")
            return
        self.done = True
        self.get_logger().info(f"=== found {len(boxes)} boxes ===")
        for i, b in enumerate(boxes):
            wx, wy, wz = cam_to_world(b["center"], depth_ax, lat)
            self.get_logger().info(
                f"box {i+1}: {b['w1']:.3f} x {b['w2']:.3f} x {b['height']:.3f} m | "
                f"world=({wx:+.2f}, {wy:+.2f}, {wz:+.2f}) | "
                f"points={b['n']} | GRASPABLE={b['graspable']}")


def main():
    rclpy.init()
    node = BoxesMeasurer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
