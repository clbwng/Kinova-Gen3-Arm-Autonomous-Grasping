from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_source = LaunchConfiguration("camera_source")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    camera_frame_id = LaunchConfiguration("camera_frame_id")
    base_frame_id = LaunchConfiguration("base_frame_id")
    debug_view = LaunchConfiguration("debug_view")
    use_camera_info = LaunchConfiguration("use_camera_info")
    rtsp_ip = LaunchConfiguration("rtsp_ip")
    rtsp_url = LaunchConfiguration("rtsp_url")
    rtsp_transport = LaunchConfiguration("rtsp_transport")
    rtsp_poll_period_s = LaunchConfiguration("rtsp_poll_period_s")

    return LaunchDescription([
        DeclareLaunchArgument("camera_source", default_value="rtsp"),
        DeclareLaunchArgument("image_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/color/camera_info"),
        DeclareLaunchArgument("camera_frame_id", default_value="camera_color_optical_frame"),
        DeclareLaunchArgument("base_frame_id", default_value="base_link"),
        DeclareLaunchArgument("debug_view", default_value="true"),
        DeclareLaunchArgument("use_camera_info", default_value="true"),
        DeclareLaunchArgument("rtsp_ip", default_value="192.168.1.10"),
        DeclareLaunchArgument("rtsp_url", default_value=""),
        DeclareLaunchArgument("rtsp_transport", default_value="udp"),
        DeclareLaunchArgument("rtsp_poll_period_s", default_value="0.03"),
        Node(
            package="kinova_gen3_control",
            executable="cv_test_gemini",
            name="purple_ball_goalkeeper",
            output="screen",
            parameters=[{
                "camera_source": camera_source,
                "image_topic": image_topic,
                "camera_info_topic": camera_info_topic,
                "camera_frame_id": camera_frame_id,
                "base_frame_id": base_frame_id,
                "debug_view": debug_view,
                "use_camera_info": use_camera_info,
                "rtsp_ip": rtsp_ip,
                "rtsp_url": rtsp_url,
                "rtsp_transport": rtsp_transport,
                "rtsp_poll_period_s": rtsp_poll_period_s,
            }],
        ),
    ])
