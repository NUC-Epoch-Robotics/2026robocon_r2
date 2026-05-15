from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("is_red_side", default_value="true"),
        DeclareLaunchArgument("camera_index", default_value="0"),

        Node(
            package="r2_lightboard_vision",
            executable="grab_scene_detector",
            name="grab_scene_detector",
            output="screen",
            parameters=[
                {
                    "camera_index": LaunchConfiguration("camera_index"),
                    "is_red_side": LaunchConfiguration("is_red_side"),
                    "fps": 15.0,
                    "frame_width": 640,
                    "frame_height": 480,
                    # ── tune these on the actual field ──
                    "large_block_min_ratio": 0.25,
                    "left_right_cx_threshold": 0.5,
                    "small_block_min_ratio": 0.03,
                    "stable_frames_required": 5,
                    "history_size": 8,
                }
            ],
        )
    ])
