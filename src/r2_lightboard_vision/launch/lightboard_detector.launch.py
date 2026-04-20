from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="r2_lightboard_vision",
            executable="lightboard_detector",
            name="lightboard_detector",
            output="screen",
            parameters=[
                {
                    "rows": 3,
                    "cols": 4,
                    "fps": 15.0,
                    "frame_width": 640,
                    "frame_height": 480,
                }
            ],
        )
    ])
