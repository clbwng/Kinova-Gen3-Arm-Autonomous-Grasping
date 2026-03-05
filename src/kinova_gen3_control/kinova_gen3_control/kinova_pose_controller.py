#!/usr/bin/env python3

# --- MUST be first: patch collections for old protobuf (Python 3.10+) ---
import collections
import collections.abc

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

import math
import threading
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from kortex_api.RouterClient import RouterClient
from kortex_api.SessionManager import SessionManager
from kortex_api.TCPTransport import TCPTransport
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Base_pb2, Session_pb2


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
            router_options = RouterClient.RouterClientSendOptions()
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
        router = self._session.__enter__()
        self._base = BaseClient(router)

        self._move_lock = threading.Lock()
        self._pose_sub = self.create_subscription(PoseStamped, "target_pose", self._on_target_pose, 10)

        self.get_logger().info(
            f"Connected to Kinova Gen3 at {config.ip}:{config.port}. Listening on topic 'target_pose'."
        )

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

        self._base.ExecuteAction(action)
        finished = done_event.wait(self._action_timeout_s)
        self._base.Unsubscribe(notification_handle)
        return finished


def main(args=None):
    rclpy.init(args=args)
    node = KinovaPoseControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()