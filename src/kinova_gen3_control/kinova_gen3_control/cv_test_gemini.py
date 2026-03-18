#!/usr/bin/env python3

from collections import deque
import os
import threading
from typing import Deque, Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

try:
    from cv_bridge import CvBridge, CvBridgeError
except ImportError:
    CvBridge = None

    class CvBridgeError(Exception):
        pass


# ── Physical setup constants — MEASURE THESE IN THE LAB ──────────────────────
# Jog the arm to the goal line center using the Kortex Web App,
# read the pose from the logger, and fill in these values.

GOAL_LINE_X = -0.267       # x coordinate of goal line in robot base frame (metres)
GOAL_LINE_Z = 0.018       # z height for end-effector above table (metres)
GOAL_LINE_Y_MIN = -0.20   # rightmost extent of goal (metres)
GOAL_LINE_Y_MAX = 0.20    # leftmost extent of goal (metres)


# Purple HSV thresholds. Tune these in the lab under the actual lighting.
PURPLE_HSV_LOWER = np.array([120, 70, 50], dtype=np.uint8)
PURPLE_HSV_UPPER = np.array([165, 255, 255], dtype=np.uint8)

GAUSSIAN_BLUR_KERNEL = (7, 7)
MORPH_KERNEL_SIZE = 5
MIN_BALL_AREA_PX = 150
TRACK_HISTORY_LEN = 4
MIN_TRACK_POINTS_FOR_PREDICTION = 3
MAX_MISSED_DETECTIONS_LOG_EVERY = 30
PROCESSING_SCALE = 0.5
BALL_DIAMETER_M = 0.0762


# Nominal Kinova vision-module color-camera intrinsics.
# The Gen3 vision module color stream comes from the Omnivision OV5640 color sensor,
# while the depth stream comes from the Intel RealSense D410 module.
# For this node's low-latency RTSP color stream, use a simple pinhole model derived
# from Kinova's documented color-camera diagonal FOV of approximately 65 degrees.
KINOVA_COLOR_DIAGONAL_FOV_DEG = 65.0


# Homogeneous transform from camera frame to robot base frame.
# Measured from the robot using the Kinova pose readout.
CAMERA_TO_BASE_TRANSLATION = np.array([-0.565, -0.047, 0.337], dtype=np.float64)
CAMERA_TO_BASE_RPY_DEG = np.array([0.0, 170.8, 86.7], dtype=np.float64)


# Blocking pose orientation. Keep these aligned with your Kinova controller conventions.
BLOCK_THETA_X_DEG = 180.0
BLOCK_THETA_Y_DEG = 0.0
BLOCK_THETA_Z_DEG = 90.0


def rotation_matrix_from_rpy_deg(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    roll, pitch, yaw = np.radians([roll_deg, pitch_deg, yaw_deg])

    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def homogeneous_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


def quaternion_from_rpy_deg(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    roll, pitch, yaw = np.radians([roll_deg, pitch_deg, yaw_deg]) / 2.0

    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return np.array([qx, qy, qz, qw], dtype=np.float64)


class PurpleBallGoalkeeperNode(Node):
    def __init__(self) -> None:
        super().__init__("purple_ball_goalkeeper")

        self.declare_parameter("camera_source", "rtsp")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("camera_frame_id", "camera_color_optical_frame")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("debug_view", True)
        self.declare_parameter("use_camera_info", True)
        self.declare_parameter("rtsp_ip", "192.168.1.10")
        self.declare_parameter("rtsp_url", "")
        self.declare_parameter("rtsp_transport", "udp")
        self.declare_parameter("rtsp_poll_period_s", 0.01)

        self._bridge = CvBridge() if CvBridge is not None else None
        self._history: Deque[Tuple[float, np.ndarray]] = deque(maxlen=TRACK_HISTORY_LEN)
        self._missed_detection_count = 0
        self._capture: Optional[cv2.VideoCapture] = None
        self._capture_thread: Optional[threading.Thread] = None
        self._capture_stop_event = threading.Event()
        self._latest_frame_lock = threading.Lock()
        self._latest_frame: Optional[np.ndarray] = None

        self._camera_source = str(self.get_parameter("camera_source").value).lower()
        self._camera_frame_id = self.get_parameter("camera_frame_id").value
        self._base_frame_id = self.get_parameter("base_frame_id").value
        self._debug_view = bool(self.get_parameter("debug_view").value)
        self._image_topic = self.get_parameter("image_topic").value
        self._camera_info_topic = self.get_parameter("camera_info_topic").value
        self._use_camera_info = bool(self.get_parameter("use_camera_info").value)
        self._rtsp_ip = str(self.get_parameter("rtsp_ip").value)
        self._rtsp_transport = str(self.get_parameter("rtsp_transport").value).lower()
        self._rtsp_poll_period_s = float(self.get_parameter("rtsp_poll_period_s").value)
        configured_rtsp_url = str(self.get_parameter("rtsp_url").value).strip()
        self._rtsp_url = configured_rtsp_url or f"rtsp://admin:admin@{self._rtsp_ip}:554/color"

        self._camera_matrix: Optional[np.ndarray] = None
        self._camera_resolution: Optional[Tuple[int, int]] = None
        self._blocking_quaternion = quaternion_from_rpy_deg(
            BLOCK_THETA_X_DEG,
            BLOCK_THETA_Y_DEG,
            BLOCK_THETA_Z_DEG,
        )

        camera_to_base_rotation = rotation_matrix_from_rpy_deg(*CAMERA_TO_BASE_RPY_DEG)
        self._t_base_camera = homogeneous_transform(
            camera_to_base_rotation,
            CAMERA_TO_BASE_TRANSLATION,
        )

        self._image_sub = None
        self._camera_info_sub = None
        self._rtsp_timer = None
        self._ball_pose_pub = self.create_publisher(PoseStamped, "ball_pose", 10)
        self._target_pose_pub = self.create_publisher(PoseStamped, "target_pose", 10)

        if self._camera_source == "rtsp":
            self._open_rtsp_capture()
            self._rtsp_timer = self.create_timer(self._rtsp_poll_period_s, self._poll_rtsp_frame)
            self.get_logger().info(
                "Using direct RTSP camera input "
                f"from '{self._rtsp_url}' and publishing 'ball_pose' and 'target_pose'."
            )
        elif self._camera_source == "ros_topic":
            if self._bridge is None:
                raise RuntimeError("cv_bridge is required when camera_source=ros_topic")
            self._image_sub = self.create_subscription(
                Image,
                self._image_topic,
                self._on_image,
                qos_profile_sensor_data,
            )
            if self._use_camera_info:
                self._camera_info_sub = self.create_subscription(
                    CameraInfo,
                    self._camera_info_topic,
                    self._on_camera_info,
                    qos_profile_sensor_data,
                )
            self.get_logger().info(
                "Listening for Kinova ROS vision topics "
                f"image='{self._image_topic}', camera_info='{self._camera_info_topic}' "
                "and publishing 'ball_pose' and 'target_pose'."
            )
        else:
            raise ValueError("camera_source must be either 'rtsp' or 'ros_topic'")

    def _on_image(self, msg: Image) -> None:
        if self._bridge is None:
            self.get_logger().error("cv_bridge is not available; cannot process ROS image messages.")
            return

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"cv_bridge conversion failed: {exc}")
            return
        except Exception as exc:
            self.get_logger().error(f"Unexpected image conversion failure: {exc}")
            return

        try:
            self._process_frame(frame, msg.header.stamp)
        except Exception as exc:
            self.get_logger().error(f"Frame processing failed but node will continue: {exc}")

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if len(msg.k) != 9:
            return

        width = int(msg.width)
        height = int(msg.height)
        resolution = (width, height)
        camera_matrix = np.array(
            [
                [float(msg.k[0]), float(msg.k[1]), float(msg.k[2])],
                [float(msg.k[3]), float(msg.k[4]), float(msg.k[5])],
                [float(msg.k[6]), float(msg.k[7]), float(msg.k[8])],
            ],
            dtype=np.float64,
        )

        if self._camera_resolution == resolution and self._camera_matrix is not None:
            if np.allclose(self._camera_matrix, camera_matrix):
                return

        self._camera_matrix = camera_matrix
        self._camera_resolution = resolution

        self.get_logger().info(
            "Using camera intrinsics from ROS camera_info "
            f"for resolution {width}x{height}: fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}, "
            f"cx={msg.k[2]:.1f}, cy={msg.k[5]:.1f}"
        )

    def _poll_rtsp_frame(self) -> None:
        frame = self._take_latest_rtsp_frame()
        if frame is None:
            return

        try:
            self._process_frame(frame, self.get_clock().now().to_msg())
        except Exception as exc:
            self.get_logger().error(f"RTSP frame processing failed but node will continue: {exc}")

    def _process_frame(self, frame: np.ndarray, stamp) -> None:
        self._update_camera_intrinsics_for_frame(frame)
        debug_frame = frame.copy()
        detection = self._detect_ball(frame)
        if detection is None:
            self._handle_missed_detection()
            if self._debug_view:
                self._annotate_status(debug_frame, "purple ball: not detected", (0, 165, 255))
                self._show_debug(debug_frame)
            return

        centroid_px, radius_px, contour = detection
        ball_position_base = self._pixel_to_base_point_from_ball_size(centroid_px, radius_px)
        if ball_position_base is None:
            self.get_logger().warning("Detected purple blob but could not estimate 3D ball position.")
            if self._debug_view:
                cv2.drawContours(debug_frame, [contour], -1, (0, 255, 255), 2)
                self._annotate_status(debug_frame, "3D estimate failed", (0, 165, 255))
                self._show_debug(debug_frame)
            return

        stamp_s = self._stamp_to_seconds(stamp.sec, stamp.nanosec)
        self._history.append((stamp_s, ball_position_base))
        self._missed_detection_count = 0

        ball_pose_msg = self._build_pose_stamped(
            stamp=stamp,
            frame_id=self._base_frame_id,
            position=ball_position_base,
        )
        self._ball_pose_pub.publish(ball_pose_msg)

        target_position = self._predict_interception_point()
        if target_position is not None:
            target_pose_msg = self._build_pose_stamped(
                stamp=stamp,
                frame_id=self._base_frame_id,
                position=target_position,
                orientation_xyzw=self._blocking_quaternion,
            )
            self._target_pose_pub.publish(target_pose_msg)

        if self._debug_view:
            cv2.drawContours(debug_frame, [contour], -1, (0, 255, 255), 2)
            cv2.circle(debug_frame, centroid_px, max(2, int(radius_px)), (0, 255, 0), 2)
            self._annotate_status(debug_frame, "purple ball: tracking", (0, 255, 0))
            cv2.putText(
                debug_frame,
                f"base x={ball_position_base[0]:.3f} y={ball_position_base[1]:.3f} z={ball_position_base[2]:.3f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )
            if target_position is not None:
                cv2.putText(
                    debug_frame,
                    f"target y={target_position[1]:.3f}",
                    (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 0, 0),
                    2,
                )
            else:
                cv2.putText(
                    debug_frame,
                    "target: waiting for more track history",
                    (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 165, 255),
                    2,
                )
            self._show_debug(debug_frame)

    def _detect_ball(
        self,
        frame: np.ndarray,
    ) -> Optional[Tuple[Tuple[int, int], float, np.ndarray]]:
        height, width = frame.shape[:2]
        scaled_width = max(1, int(width * PROCESSING_SCALE))
        scaled_height = max(1, int(height * PROCESSING_SCALE))
        scaled_frame = cv2.resize(frame, (scaled_width, scaled_height), interpolation=cv2.INTER_LINEAR)

        blurred = cv2.GaussianBlur(scaled_frame, GAUSSIAN_BLUR_KERNEL, 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, PURPLE_HSV_LOWER, PURPLE_HSV_UPPER)
        morph_kernel = np.ones((MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, morph_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, morph_kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) >= MIN_BALL_AREA_PX]
        if not valid_contours:
            return None

        best_contour = max(valid_contours, key=cv2.contourArea)
        (cx, cy), radius = cv2.minEnclosingCircle(best_contour)
        moments = cv2.moments(best_contour)
        if moments["m00"] == 0.0:
            return None

        contour_full_res = (best_contour.astype(np.float32) / PROCESSING_SCALE).astype(np.int32)
        centroid = (
            int((moments["m10"] / moments["m00"]) / PROCESSING_SCALE),
            int((moments["m01"] / moments["m00"]) / PROCESSING_SCALE),
        )
        return centroid, radius / PROCESSING_SCALE, contour_full_res

    def _pixel_to_base_point_from_ball_size(
        self,
        pixel_xy: Tuple[int, int],
        radius_px: float,
    ) -> Optional[np.ndarray]:
        if self._camera_matrix is None or radius_px <= 1e-6:
            return None

        u, v = pixel_xy
        fx = self._camera_matrix[0, 0]
        fy = self._camera_matrix[1, 1]
        cx = self._camera_matrix[0, 2]
        cy = self._camera_matrix[1, 2]

        diameter_px = 2.0 * radius_px
        depth_m = (fx * BALL_DIAMETER_M) / diameter_px

        x_camera = ((float(u) - cx) * depth_m) / fx
        y_camera = ((float(v) - cy) * depth_m) / fy
        z_camera = depth_m

        ball_in_camera_h = np.array([x_camera, y_camera, z_camera, 1.0], dtype=np.float64)
        return (self._t_base_camera @ ball_in_camera_h)[:3]

    def _update_camera_intrinsics_for_frame(self, frame: np.ndarray) -> None:
        height, width = frame.shape[:2]
        resolution = (width, height)
        if self._camera_resolution == resolution and self._camera_matrix is not None:
            return

        diagonal_pixels = float(np.hypot(width, height))
        half_diagonal_fov_rad = np.radians(KINOVA_COLOR_DIAGONAL_FOV_DEG / 2.0)
        focal_pixels = diagonal_pixels / (2.0 * np.tan(half_diagonal_fov_rad))
        cx = (width - 1) / 2.0
        cy = (height - 1) / 2.0

        self._camera_matrix = np.array(
            [
                [focal_pixels, 0.0, cx],
                [0.0, focal_pixels, cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        self._camera_resolution = resolution

        self.get_logger().info(
            "Using nominal Kinova color-camera intrinsics "
            f"for resolution {width}x{height}: fx=fy={focal_pixels:.1f}, cx={cx:.1f}, cy={cy:.1f}"
        )

    def _predict_interception_point(self) -> Optional[np.ndarray]:
        if len(self._history) < MIN_TRACK_POINTS_FOR_PREDICTION:
            return None

        oldest_time, oldest_position = self._history[0]
        newest_time, newest_position = self._history[-1]
        delta_t = newest_time - oldest_time
        if delta_t <= 1e-6:
            return None

        velocity = (newest_position - oldest_position) / delta_t
        vx = velocity[0]
        if abs(vx) < 1e-6:
            return None

        time_to_goal = (GOAL_LINE_X - newest_position[0]) / vx
        if time_to_goal < 0.0:
            return None

        predicted_y = newest_position[1] + velocity[1] * time_to_goal
        constrained_y = float(np.clip(predicted_y, GOAL_LINE_Y_MIN, GOAL_LINE_Y_MAX))

        return np.array([GOAL_LINE_X, constrained_y, GOAL_LINE_Z], dtype=np.float64)

    def _build_pose_stamped(
        self,
        stamp,
        frame_id: str,
        position: np.ndarray,
        orientation_xyzw: Optional[np.ndarray] = None,
    ) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])
        if orientation_xyzw is None:
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0
        else:
            msg.pose.orientation.x = float(orientation_xyzw[0])
            msg.pose.orientation.y = float(orientation_xyzw[1])
            msg.pose.orientation.z = float(orientation_xyzw[2])
            msg.pose.orientation.w = float(orientation_xyzw[3])
        return msg

    def _handle_missed_detection(self) -> None:
        self._missed_detection_count += 1
        if self._missed_detection_count % MAX_MISSED_DETECTIONS_LOG_EVERY == 1:
            self.get_logger().warning("Purple ball not detected in current frame; continuing to track.")

    def _show_debug(self, frame: np.ndarray) -> None:
        try:
            cv2.imshow("Purple Ball Goalkeeper", frame)
            cv2.waitKey(1)
        except Exception as exc:
            self.get_logger().warning(f"OpenCV display failed but node will continue: {exc}")

    def _open_rtsp_capture(self) -> None:
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = f"rtsp_transport;{self._rtsp_transport}"
        self._release_rtsp_capture()
        self._capture = cv2.VideoCapture(self._rtsp_url, cv2.CAP_FFMPEG)
        self._configure_capture()
        if self._capture.isOpened():
            self._start_capture_thread()
            return

        self.get_logger().warning("OpenCV CAP_FFMPEG RTSP open failed; retrying with backend auto-detect.")
        self._capture = cv2.VideoCapture(self._rtsp_url)
        self._configure_capture()
        if not self._capture.isOpened():
            self.get_logger().warning(
                f"Could not connect to RTSP camera at '{self._rtsp_url}'. Will keep retrying."
            )
            return

        self._start_capture_thread()

    def _release_rtsp_capture(self) -> None:
        self._capture_stop_event.set()
        if self._capture_thread is not None:
            self._capture_thread.join(timeout=1.0)
            self._capture_thread = None
        if self._capture is not None:
            self._capture.release()
            self._capture = None
        with self._latest_frame_lock:
            self._latest_frame = None
        self._capture_stop_event.clear()

    def _configure_capture(self) -> None:
        if self._capture is None:
            return
        try:
            self._capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

    def _start_capture_thread(self) -> None:
        if self._capture is None or self._capture_thread is not None:
            return
        self._capture_stop_event.clear()
        self._capture_thread = threading.Thread(target=self._rtsp_capture_loop, daemon=True)
        self._capture_thread.start()

    def _rtsp_capture_loop(self) -> None:
        while not self._capture_stop_event.is_set():
            if self._capture is None or not self._capture.isOpened():
                break

            ret, frame = self._capture.read()
            if not ret or frame is None:
                self.get_logger().warning("RTSP frame grab failed; reconnecting.")
                self._capture_stop_event.set()
                break

            with self._latest_frame_lock:
                self._latest_frame = frame

    def _take_latest_rtsp_frame(self) -> Optional[np.ndarray]:
        if self._capture_stop_event.is_set() or (
            self._capture_thread is not None and not self._capture_thread.is_alive()
        ):
            self._release_rtsp_capture()
            self._open_rtsp_capture()
            return None

        if self._capture is None or not self._capture.isOpened():
            self._open_rtsp_capture()
            return None
        if self._capture_thread is None:
            self._start_capture_thread()
            return None

        with self._latest_frame_lock:
            frame = self._latest_frame
            self._latest_frame = None

        return frame

    @staticmethod
    def _annotate_status(frame: np.ndarray, text: str, color: Tuple[int, int, int]) -> None:
        cv2.putText(
            frame,
            text,
            (10, 85),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2,
        )

    @staticmethod
    def _stamp_to_seconds(sec: int, nanosec: int) -> float:
        return float(sec) + float(nanosec) * 1e-9

    def destroy_node(self):
        self._release_rtsp_capture()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PurpleBallGoalkeeperNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
