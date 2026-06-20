from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    show_debug = LaunchConfiguration("show_debug")
    serial_number = LaunchConfiguration("serial_number")

    return LaunchDescription([
        DeclareLaunchArgument(
            "show_debug",
            default_value="true",
            description="显示调试窗口: 网格+采样框+分类颜色 (需有显示器/X11)"),
        # D435i 序列号: 多相机时指定用哪台, 空=用第一台 (单相机留空)
        # 查序列号: rs-enumerate-devices --compact
        DeclareLaunchArgument(
            "serial_number",
            default_value="",
            description="D435i 序列号, 空=第一台 (D435i 无 V4L 节点, 用 pyrealsense2 取流)"),
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
                    # D435i color 流: 640x480@30 最稳; 1280x720@30 多数支持; 不行就回退 640x480
                    "fps": 30.0,
                    "frame_width": 1280,
                    "frame_height": 720,
                    "serial_number": serial_number,
                    # 离线标定直接出图 (整机启动时此文件不用, 决策侧用 start_enabled:=False 自控)
                    "start_enabled": True,
                    "show_debug": show_debug,
                    # ── ROI / 阈值标定参数 (按现场调) ──
                    "roi_x": 0,
                    "roi_y": 0,
                    "roi_w": 0,   # 0 = 用整帧宽
                    "roi_h": 0,   # 0 = 用整帧高
                    "sample_ratio": 0.55,
                    "min_v_lit": 120,
                    "min_s_color": 45,
                    "max_s_white": 35,
                    "min_color_ratio": 0.22,
                }
            ],
        )
    ])
