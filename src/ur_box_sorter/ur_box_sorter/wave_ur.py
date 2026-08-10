import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
ACTION = "/scaled_joint_trajectory_controller/follow_joint_trajectory"

POSES = {
    "home":  [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
    "reach": [0.0, -1.0, 0.8, -1.4, -1.57, 0.0],
}


class UrMover(Node):
    def __init__(self):
        super().__init__("ur_mover")
        self.client = ActionClient(self, FollowJointTrajectory, ACTION)

    def move_to(self, positions, seconds=4.0):
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("controller not found")
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start.sec = int(seconds)
        goal.trajectory.points = [pt]
        self.get_logger().info(f"sending goal {positions}")
        send = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if not handle.accepted:
            self.get_logger().error("goal rejected")
            return
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result)
        self.get_logger().info("motion complete")


def main():
    rclpy.init()
    node = UrMover()
    node.move_to(POSES["reach"], 4.0)
    node.move_to(POSES["home"], 4.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
