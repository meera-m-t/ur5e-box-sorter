#!/bin/bash
echo "=== [1/5] cleaning old processes ==="
pkill -f 'gz sim'; pkill -f ruby; pkill -f parameter_bridge; pkill -f rviz2; pkill -f ros_gz; pkill -f spawner; pkill -f controller_manager; pkill -f robot_state_publisher; pkill -f rqt; pkill -f joint_state_publisher
sleep 3

echo "=== [2/5] EGL -> NVIDIA driver (bypassing mesa) ==="
unset LIBGL_ALWAYS_SOFTWARE
if [ -f /usr/share/glvnd/egl_vendor.d/10_nvidia.json ]; then
  export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
  echo "  ok: using NVIDIA EGL"
else
  echo "  WARNING: 10_nvidia.json missing:"; ls /usr/share/glvnd/egl_vendor.d/ 2>/dev/null
fi

if ! grep -q rgbd_camera "$HOME/projects/robot_ws/ur_sensor_world.sdf"; then
  echo "  FATAL: world file has NO camera model — tell Claude"; exit 1
fi

echo "=== [3/5] launching gazebo + arm (log: /tmp/t1.log) ==="
setsid ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur5e \
  world_file:=$HOME/projects/robot_ws/ur_sensor_world.sdf > /tmp/t1.log 2>&1 < /dev/null &
echo -n "  waiting for arm controllers"
for i in $(seq 1 60); do
  grep -q "Successfully switched controllers" /tmp/t1.log && break
  echo -n "."; sleep 1
done
echo ""
if ! grep -q "Successfully switched controllers" /tmp/t1.log; then
  echo "  ✗ controllers never came up — paste end of /tmp/t1.log to Claude"; exit 1
fi
echo "  ✓ arm controllers active"

echo "=== [4/5] camera gate ==="
CAMERA_OK=0
for i in $(seq 1 10); do
  if timeout 3 gz topic -e -t /rgbd/points 2>/dev/null | head -c 50 | grep -q .; then CAMERA_OK=1; break; fi
  echo "  ...not yet ($i/10)"
done
if [ "$CAMERA_OK" = "1" ]; then echo "  ✓✓✓ CAMERA ALIVE"; else
  echo "  ✗✗✗ CAMERA DEAD — diagnostics:"
  grep -niE "egl|dri2|advertised|Rendering Thread" /tmp/t1.log | tail -20
  echo "  (gazebo left running; paste lines above to Claude)"; exit 1
fi

echo "=== [5/5] starting camera bridge ==="
setsid ros2 run ros_gz_bridge parameter_bridge \
 /rgbd/image@sensor_msgs/msg/Image[gz.msgs.Image \
 /rgbd/depth_image@sensor_msgs/msg/Image[gz.msgs.Image \
 /rgbd/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked \
 /rgbd/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo > /tmp/bridge.log 2>&1 < /dev/null &
sleep 2
echo ""
echo "=================================================="
echo "  ROBOT READY — this terminal is FREE again."
echo "  Run the demo right here:"
echo "    source install/setup.bash"
echo "    ros2 run panda_control sort_boxes"
echo "  Stop the robot anytime:  bash stop_robot.sh"
echo "=================================================="
