#!/usr/bin/env python3

# --- MUST be first: patch collections for old protobuf (Python 3.10+) ---
import collections
import collections.abc
import os
import sys
import time

_COLLECTIONS_COMPAT_ATTRS = (
    "Mapping",
    "MutableMapping",
    "Sequence",
    "MutableSequence",
    "Set",
    "MutableSet",
    "Iterable",
)
for _name in _COLLECTIONS_COMPAT_ATTRS:
    if not hasattr(collections, _name) and hasattr(collections.abc, _name):
        setattr(collections, _name, getattr(collections.abc, _name))
# -----------------------------------------------------------------------


def _add_active_venv_site_packages():
    venv = os.environ.get("VIRTUAL_ENV")
    if not venv:
        return

    py_version = f"python{sys.version_info.major}.{sys.version_info.minor}"
    site_packages = os.path.join(venv, "lib", py_version, "site-packages")
    if os.path.isdir(site_packages) and site_packages not in sys.path:
        sys.path.insert(0, site_packages)


_add_active_venv_site_packages()

import math
import threading
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
from kortex_api.SessionManager import SessionManager
from kortex_api.TCPTransport import TCPTransport
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.ControlConfigClientRpc import ControlConfigClient
from kortex_api.autogen.messages import Base_pb2, ControlConfig_pb2, Session_pb2


@dataclass
class KortexConnectionConfig:
    ip: str
    port: int
    username: str
    password: str
    session_inactivity_timeout_ms: int = 60000
    connection_inactivity_timeout_ms: int = 2000


class KortexSession:
    def __init__(self, config: KortexConnectionConfig):
        self._config = config
        self._transport: Optional[TCPTransport] = None
        self._router: Optional[RouterClient] = None
        self._session_manager: Optional[SessionManager] = None

    def __enter__(self):
        self._transport = TCPTransport()
        self._transport.connect(self._config.ip, self._config.port)
        self._router = RouterClient(self._transport, self._error_callback)

        self._session_manager = SessionManager(self._router)
        session_info = Session_pb2.CreateSessionInfo()
        session_info.username = self._config.username
        session_info.password = self._config.password
        session_info.session_inactivity_timeout = self._config.session_inactivity_timeout_ms
        session_info.connection_inactivity_timeout = self._config.connection_inactivity_timeout_ms

        self._session_manager.CreateSession(session_info)
        return self._router

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self._session_manager is not None:
            router_options = RouterClientSendOptions()
            router_options.timeout_ms = 1000
            self._session_manager.CloseSession(router_options)

        if self._transport is not None:
            self._transport.disconnect()

    @staticmethod
    def _error_callback(exception):
        print(f"Kortex router error: {exception}")


def quaternion_to_euler_deg(x: float, y: float, z: float, w: float):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def control_mode_name(mode: int) -> str:
    try:
        return ControlConfig_pb2.ControlMode.Name(mode)
    except ValueError:
        return f"UNKNOWN_CONTROL_MODE_{mode}"


class KinovaPoseControllerNode(Node):
    def __init__(self):
        super().__init__("kinova_pose_controller")

        self.declare_parameter("robot_ip", "192.168.1.10")
        self.declare_parameter("robot_port", 10000)
        self.declare_parameter("username", "admin")
        self.declare_parameter("password", "admin")
        self.declare_parameter("action_timeout_s", 20.0)

        config = KortexConnectionConfig(
            ip=self.get_parameter("robot_ip").get_parameter_value().string_value,
            port=self.get_parameter("robot_port").get_parameter_value().integer_value,
            username=self.get_parameter("username").get_parameter_value().string_value,
            password=self.get_parameter("password").get_parameter_value().string_value,
        )
        self._action_timeout_s = self.get_parameter("action_timeout_s").get_parameter_value().double_value

        self._session = KortexSession(config)
        self.get_logger().info(f"Connecting to Kinova Gen3 at {config.ip}:{config.port}...")
        router = self._session.__enter__()
        self.get_logger().info("Kortex session established")
        self._base = BaseClient(router)
        self._control_config = ControlConfigClient(router)
        self._hard_limits = None

        self._move_lock = threading.Lock()
        self._pose_sub = self.create_subscription(PoseStamped, "target_pose", self._on_target_pose, 10)
        self._ee_pose_timer = self.create_timer(1.0, self._log_measured_cartesian_pose)

        self._set_cartesian_soft_limits_to_hard_limits()
        self._log_kinematic_constraints()
        self.get_logger().info(
            f"Connected to Kinova Gen3 at {config.ip}:{config.port}. Listening on topic 'target_pose'."
        )

    def run(self):
        """
        Put direct cartesian motion logic here using meters and Euler angles in degrees.
        Return after sending any motions you want this node to execute.
        """
        current_pose = self._base.GetMeasuredCartesianPose()
        self.get_logger().info(
            "Current EE pose before run() "
            f"x={current_pose.x:.3f}, y={current_pose.y:.3f}, z={current_pose.z:.3f}, "
            f"theta_x={current_pose.theta_x:.1f}, theta_y={current_pose.theta_y:.1f}, "
            f"theta_z={current_pose.theta_z:.1f}"
        )

        # XYZ position is in relation to base frame
        # theta x, theta y are in relation to end effector pose
        # theta z is in relation to base frame

        # Example start position
        self.move_to_euler_pose(
            x=0.565,
            y=-0.047,
            z=0.337,
            theta_x_deg=175.6,
            theta_y_deg=-3.8,
            theta_z_deg=86.7,
        )

        # time.sleep(5)

        # example interception position
        self.move_to_euler_pose(
            x=0.216,
            y=0.021,
            z=0.086,
            theta_x_deg=175.6,
            theta_y_deg=-6.5,
            theta_z_deg=98.0,
        )

        # if ok:
        #     self.get_logger().info("run() motion finished successfully")
        # else:
        #     self.get_logger().error("run() motion failed")

    def destroy_node(self):
        self.get_logger().info("Closing Kortex session...")
        self._session.__exit__(None, None, None)
        return super().destroy_node()

    def _on_target_pose(self, msg: PoseStamped):
        with self._move_lock:
            pose = msg.pose
            rx, ry, rz = quaternion_to_euler_deg(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )

            self.get_logger().info(
                "Executing cartesian move to "
                f"x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}, "
                f"theta_x={rx:.1f}, theta_y={ry:.1f}, theta_z={rz:.1f}"
            )

            ok = self._execute_cartesian_action(
                x=pose.position.x,
                y=pose.position.y,
                z=pose.position.z,
                theta_x=rx,
                theta_y=ry,
                theta_z=rz,
            )

            if ok:
                self.get_logger().info("Move finished successfully")
            else:
                self.get_logger().error("Move failed or timed out")

    def _log_measured_cartesian_pose(self):
        try:
            pose = self._base.GetMeasuredCartesianPose()
        except Exception as exc:
            self.get_logger().error(f"Failed to read measured cartesian pose: {exc}")
            return

        self.get_logger().info(
            "EE pose "
            f"x={pose.x:.3f}, y={pose.y:.3f}, z={pose.z:.3f}, "
            f"theta_x={pose.theta_x:.1f}, theta_y={pose.theta_y:.1f}, theta_z={pose.theta_z:.1f}"
        )

    def _log_kinematic_constraints(self):
        try:
            hard_limits = self._control_config.GetKinematicHardLimits()
        except Exception as exc:
            self.get_logger().error(f"Failed to query hard kinematic limits: {exc}")
        else:
            self._hard_limits = hard_limits
            self.get_logger().info(
                "Hard limits "
                f"mode={control_mode_name(hard_limits.control_mode)}, "
                f"twist_linear={hard_limits.twist_linear:.3f} m/s, "
                f"twist_angular={hard_limits.twist_angular:.1f} deg/s, "
                f"joint_speed_limits={[round(v, 3) for v in hard_limits.joint_speed_limits]}, "
                f"joint_acceleration_limits={[round(v, 3) for v in hard_limits.joint_acceleration_limits]}"
            )

        try:
            soft_limits = self._control_config.GetAllKinematicSoftLimits()
        except Exception as exc:
            self.get_logger().error(f"Failed to query soft kinematic limits: {exc}")
            return

        for limits in soft_limits.kinematic_limits_list:
            self.get_logger().info(
                "Soft limits "
                f"mode={control_mode_name(limits.control_mode)}, "
                f"twist_linear={limits.twist_linear:.3f} m/s, "
                f"twist_angular={limits.twist_angular:.1f} deg/s, "
                f"joint_speed_limits={[round(v, 3) for v in limits.joint_speed_limits]}, "
                f"joint_acceleration_limits={[round(v, 3) for v in limits.joint_acceleration_limits]}"
            )

    def _set_cartesian_soft_limits_to_hard_limits(self):
        try:
            hard_limits = self._control_config.GetKinematicHardLimits()
        except Exception as exc:
            self.get_logger().error(f"Failed to query hard limits before setting soft limits: {exc}")
            return

        self._hard_limits = hard_limits

        linear_limit = ControlConfig_pb2.TwistLinearSoftLimit()
        linear_limit.twist_linear_soft_limit = hard_limits.twist_linear

        angular_limit = ControlConfig_pb2.TwistAngularSoftLimit()
        angular_limit.twist_angular_soft_limit = hard_limits.twist_angular

        cartesian_modes = (
            ControlConfig_pb2.CARTESIAN_JOYSTICK,
            ControlConfig_pb2.CARTESIAN_TRAJECTORY,
            ControlConfig_pb2.CARTESIAN_WAYPOINT_TRAJECTORY,
        )

        for mode in cartesian_modes:
            linear_limit.control_mode = mode
            angular_limit.control_mode = mode
            try:
                self._control_config.SetTwistLinearSoftLimit(linear_limit)
                self._control_config.SetTwistAngularSoftLimit(angular_limit)
            except Exception as exc:
                self.get_logger().error(
                    f"Failed to set soft Cartesian limits for mode={control_mode_name(mode)}: {exc}"
                )
                continue

            self.get_logger().info(
                "Set soft Cartesian limits "
                f"mode={control_mode_name(mode)}, "
                f"twist_linear={hard_limits.twist_linear:.3f} m/s, "
                f"twist_angular={hard_limits.twist_angular:.1f} deg/s"
            )

    def move_to_euler_pose(
        self,
        x: float,
        y: float,
        z: float,
        theta_x_deg: float,
        theta_y_deg: float,
        theta_z_deg: float,
    ) -> bool:
        self.get_logger().info(
            "Executing direct euler move to "
            f"x={x:.3f}, y={y:.3f}, z={z:.3f}, "
            f"theta_x={theta_x_deg:.1f}, theta_y={theta_y_deg:.1f}, theta_z={theta_z_deg:.1f}"
        )
        return self._execute_cartesian_action(
            x=x,
            y=y,
            z=z,
            theta_x=theta_x_deg,
            theta_y=theta_y_deg,
            theta_z=theta_z_deg,
        )

    def _execute_cartesian_action(
        self,
        x: float,
        y: float,
        z: float,
        theta_x: float,
        theta_y: float,
        theta_z: float,
    ) -> bool:
        done_event = threading.Event()

        def notification_callback(notification):
            action_event = notification.action_event
            if action_event in (Base_pb2.ACTION_END, Base_pb2.ACTION_ABORT):
                done_event.set()

        notification_handle = self._base.OnNotificationActionTopic(
            notification_callback,
            Base_pb2.NotificationOptions(),
        )

        action = Base_pb2.Action()
        action.name = "ros2_reach_pose"
        action.application_data = ""

        target_pose = action.reach_pose.target_pose
        target_pose.x = x
        target_pose.y = y
        target_pose.z = z
        target_pose.theta_x = theta_x
        target_pose.theta_y = theta_y
        target_pose.theta_z = theta_z

        translation_speed = 0.5
        orientation_speed = 100.0
        if self._hard_limits is not None:
            translation_speed = self._hard_limits.twist_linear
            orientation_speed = self._hard_limits.twist_angular

        action.reach_pose.constraint.speed.translation = translation_speed
        action.reach_pose.constraint.speed.orientation = orientation_speed

        try:
            self._base.ExecuteAction(action)
        except Exception as exc:
            self.get_logger().error(f"ExecuteAction failed: {exc}")
            self._base.Unsubscribe(notification_handle)
            return False

        finished = done_event.wait(self._action_timeout_s)
        self._base.Unsubscribe(notification_handle)
        return finished


def main(args=None):
    rclpy.init(args=args)
    node = KinovaPoseControllerNode()

    try:
        node.run()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
