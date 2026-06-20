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
                    # 灯板竖放: 长边4(行) × 短边3(列) = 12格, 与 zone2_planner 布局一致
                    "rows": 4,
                    "cols": 3,
                    "fps": 15.0,
                    "frame_width": 640,
                    "frame_height": 480,
                    # 离线标定直接出图 (整机启动时此文件不用, 决策侧用 start_enabled:=False 自控)
                    "start_enabled": True,
                }
            ],
        )
    ])
