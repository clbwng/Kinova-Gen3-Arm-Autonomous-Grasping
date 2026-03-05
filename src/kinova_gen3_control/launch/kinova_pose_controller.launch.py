from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')
    robot_port = LaunchConfiguration('robot_port')
    username = LaunchConfiguration('username')
    password = LaunchConfiguration('password')
    action_timeout_s = LaunchConfiguration('action_timeout_s')

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.10'),
        DeclareLaunchArgument('robot_port', default_value='10000'),
        DeclareLaunchArgument('username', default_value='admin'),
        DeclareLaunchArgument('password', default_value='admin'),
        DeclareLaunchArgument('action_timeout_s', default_value='20.0'),
        Node(
            package='kinova_gen3_control',
            executable='kinova_pose_controller',
            name='kinova_pose_controller',
            output='screen',
            parameters=[{
                'robot_ip': robot_ip,
                'robot_port': robot_port,
                'username': username,
                'password': password,
                'action_timeout_s': action_timeout_s,
            }],
        ),
    ])
