# Kinova Gen3 Ball Detection and Grasping

This README documents only [`FINAL_FULL_PIPELINE.py`](FINAL_FULL_PIPELINE.py).

That file is the complete end-to-end script for this project: it handles ball detection, real-time tracking, grasping, drop-off into the box, and returning the arm to its tracking pose. The other files in the repository are groundwork for future implementation and are not required to understand or run the current demo pipeline.

## Whitepaper

[Whitepaper](https://drive.google.com/file/d/1ezGyzaLbHz9VIY2MEQj9MzY5GHLofGP5/view?usp=sharing)

## Demo

[Demo video](https://drive.google.com/file/d/1YOITdIvsg0TfcgGtUhHni0DOYVj4CV4L/view?usp=sharing)

## What `FINAL_FULL_PIPELINE.py` Does

- Connects to the Kinova Gen3 using the Kortex API.
- Opens the robot camera RTSP stream in OpenCV.
- Detects the ball with HSV thresholding, morphology, contour filtering, and circularity checks.
- Estimates the ball position in world coordinates from image coordinates, camera intrinsics, and the robot's measured cartesian pose.
- Tracks the ball in real time with PD control in `x`, `y`, and `z`.
- Lets you press `g` to trigger the full grasp routine:
  - stop tracking
  - descend to the ball with a gripper offset
  - close the gripper
  - lift the ball
  - move to the box
  - descend into the box
  - release the ball
  - return to the tracking pose
  - resume tracking
- Displays the live camera feed and mask window for debugging.

## Before You Run It

You need:

- A Kinova Gen3 reachable on the network.
- ROS 2 with `rclpy` available in your shell.
- Python packages used by the script, including:
  - `opencv-python`
  - `numpy`
  - the Kinova `kortex_api`
- OpenCV support for RTSP streaming.

Important script assumptions:

- The robot connection defaults to `192.168.1.10`.
- The camera stream is currently opened from `rtsp://192.168.1.10/color`.
- The script starts from a fixed tracking pose and uses a fixed box drop pose.

Before using it on hardware, make sure these values are correct for your setup:

- `BOX_POSE`
- `BOX_DESCENT_M`
- camera intrinsics: `fx`, `fy`, `cx`, `cy`
- `camera_height`
- HSV ball thresholds
- workspace safety limits
- grab offset and descent distance

## How to Run

From the repository root:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
python3 FINAL_FULL_PIPELINE.py
```

If your Python packages are installed in a virtual environment, activate it first:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source /path/to/venv/bin/activate
python3 FINAL_FULL_PIPELINE.py
```

The script already checks `VIRTUAL_ENV` and adds that environment's `site-packages` path automatically.

You can also override ROS parameters at launch time:

```bash
python3 FINAL_FULL_PIPELINE.py --ros-args \
  -p robot_ip:=192.168.1.10 \
  -p username:=admin \
  -p password:=admin
```

Useful parameters exposed by the script include:

- robot connection: `robot_ip`, `robot_port`, `username`, `password`
- start pose: `track_x`, `track_y`, `track_z`, `track_theta_x`, `track_theta_y`, `track_theta_z`
- detection: `ball_hsv_lower`, `ball_hsv_upper`, `min_ball_area`, `min_circularity`
- controller tuning: `kp_xy`, `kd_xy`, `kp_z`, `kd_z`
- safety limits: `workspace_min_x`, `workspace_max_x`, `workspace_min_y`, `workspace_max_y`, `workspace_min_z`, `workspace_max_z`

Note: the robot connection IP is configurable as a ROS parameter, but the RTSP camera address is hardcoded in the script. If your camera stream is not at `192.168.1.10`, update the RTSP pipeline string in `FINAL_FULL_PIPELINE.py` before running.

## Runtime Controls

When the OpenCV window is focused:

- `g`: start the grasp sequence
- `o`: open the gripper
- `r`: reset, reopen the gripper, clear faults, return to the start pose, and resume tracking
- `q`: quit

## Behavior Summary

On startup, the script:

1. connects to the robot
2. clears faults and configures limits
3. opens the gripper
4. moves to the tracking start pose
5. opens the camera stream
6. starts live detection and closed-loop tracking

During tracking, if the ball is detected, the arm uses twist commands to keep the ball centered and maintain distance. If detection becomes stale or the ball is lost, the robot stops sending motion commands until a valid detection returns.
