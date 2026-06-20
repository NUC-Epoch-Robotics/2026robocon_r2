from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    show_debug = LaunchConfiguration("show_debug")
    camera_index = LaunchConfiguration("camera_index")

    return LaunchDescription([
        DeclareLaunchArgument(
            "show_debug",
            default_value="true",
            description="显示调试窗口: 网格+采样框+分类颜色 (需有显示器/X11)"),
        # D435i 相机 id 硬编码在此: 改下面 default_value 即可
        DeclareLaunchArgument(
            "camera_index",
            default_value="0",
            description="相机设备索引 (D435i 彩色流), 0/2/4 等"),
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
                    "fps": 30.0,
                    "frame_width": 1280,
                    "frame_height": 720,
                    # 相机 id (来自上面的 launch arg)
                    "camera_index": camera_index,
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
