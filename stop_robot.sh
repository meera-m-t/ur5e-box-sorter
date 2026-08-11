#!/bin/bash
pkill -f 'gz sim'; pkill -f ruby; pkill -f parameter_bridge; pkill -f rviz2; pkill -f ros_gz; pkill -f spawner; pkill -f controller_manager; pkill -f robot_state_publisher; pkill -f rqt; pkill -f joint_state_publisher; pkill -f static_transform_publisher; pkill -f move_group; pkill -f ur_moveit
echo "robot stopped"
