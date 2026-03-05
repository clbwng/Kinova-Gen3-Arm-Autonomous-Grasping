# Kinova-Gen3-Arm-Autonomous-Grasping

This repo now contains a ROS 2 Python package at `src/kinova_gen3_control` with a node that:
- Connects to a Kinova Gen3 using the Kortex API (`kortex_api`)
- Subscribes to `target_pose` (`geometry_msgs/PoseStamped`)
- Sends a cartesian `reach_pose` action to move the arm to that pose

## Build

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select kinova_gen3_control
source install/setup.bash
```

## Run

```bash
ros2 launch kinova_gen3_control kinova_pose_controller.launch.py
```

You can override parameters at launch time:

```bash
ros2 launch kinova_gen3_control kinova_pose_controller.launch.py \
  robot_ip:=192.168.1.10 username:=admin password:=admin
```

## Send a target pose

```bash
ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped "
header:
  frame_id: base_link
pose:
  position: {x: 0.45, y: 0.0, z: 0.35}
  orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}
"
```

Notes:
- Pose orientation quaternion is converted to Euler angles (degrees) for Kortex `theta_x/y/z`.
- Make sure your target pose is safe and reachable before commanding motion.
