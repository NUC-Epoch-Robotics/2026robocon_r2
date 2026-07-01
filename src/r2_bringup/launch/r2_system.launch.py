import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def _launch_decision_node(context, *args, **kwargs):
    """根据 is_red_side 参数动态选择红蓝区配置."""
    is_red = LaunchConfiguration('is_red_side').perform(context) == 'true'

    # ── 公共参数 ──
    params = {
        'is_red_side': is_red,
        'fine_tune_xy_threshold': 0.01,
        'fine_tune_stable_required': 5,
        'fine_tune_timeout_s': 15.0,
        'fine_tune_speed_x': 0.03,
        'fine_tune_speed_y': 0.03,
        'zone1_route': [5],
        'zone1_point_7_x': -1.42,
        'zone1_point_7_y': 1.7,
        'zone1_point_7_z': -0.622,
        'zone1_point_8_x': -1.42,
        'zone1_point_8_y': 1.9,
        'zone1_point_8_z': -0.622,
        'use_fixed_route': True,
        'grab_qz': 0.707,
        'grab_qw': 0.707,
        'stairs_start_x': -1.9,
        'stairs_start_y': 1.9,
        'mf_exit_x': -0.289,
        'mf_exit_y': 7.5,
    }

    # ── 红蓝区差异参数 ──
    if is_red:
        params.update({
            'fine_tune_target_x': 0.348,
            'fine_tune_target_y': 1.222,
            'zone1_point_5_x': 0.60,
            'zone1_point_5_y': -0.36,
            'zone1_point_5_z': -0.622,
        })
    else:
        params.update({
            'fine_tune_target_x': 0.348,
            'fine_tune_target_y': 0.950,
            'zone1_point_5_x': 0.60,
            'zone1_point_5_y': 0.36,
            'zone1_point_5_z': -0.622,
        })

    return [Node(
        package='r2_decision_py',
        executable='r2_decision_py',
        name='r2_decision_py',
        output='screen',
        parameters=[params],
    )]

def generate_launch_description():
    # template image path
    _vision_share = get_package_share_directory('r2_lightboard_vision')
    _template_path = os.path.join(_vision_share, 'cylinder_template.jpg')

    enable_lightboard = LaunchConfiguration('enable_lightboard')
    lightboard_camera_index = LaunchConfiguration('lightboard_camera_index')
    enable_spearhead = LaunchConfiguration('enable_spearhead')
    spearhead_camera_index = LaunchConfiguration('spearhead_camera_index')
    enable_grab_scene = LaunchConfiguration('enable_grab_scene')
    grab_scene_camera_index = LaunchConfiguration('grab_scene_camera_index')
    is_red_side = LaunchConfiguration('is_red_side')
    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_lightboard', default_value='true',
            description='Enable 3x4 lightboard detector node'),
        DeclareLaunchArgument(
            'lightboard_camera_index', default_value='0',
            description='Camera index for lightboard detector'),
        DeclareLaunchArgument(
            'enable_spearhead', default_value='true',
            description='Enable spearhead detector node'),
        DeclareLaunchArgument(
            'spearhead_camera_index', default_value='1',
            description='Camera index for spearhead detector'),
        DeclareLaunchArgument(
            'enable_grab_scene', default_value='true',
            description='Enable grab-scene detector node'),
        DeclareLaunchArgument(
            'grab_scene_camera_index', default_value='0',
            description='Camera index for grab-scene detector (shares with lightboard)'),
        DeclareLaunchArgument(
            'is_red_side', default_value='true',
            description='Red side or blue side'),

        # ── Decision node (Python async FSM) ──────────────────
        OpaqueFunction(function=_launch_decision_node),

        # ── Lightboard detector (3x4 RGBW) ───────────────────
        Node(
            package='r2_lightboard_vision',
            executable='lightboard_detector',
            name='lightboard_detector',
            output='screen',
            condition=IfCondition(enable_lightboard),
            parameters=[{
                # 灯板竖放: 长边4(行) × 短边3(列) = 12格, 与 zone2_planner 邻接表布局一致
                'rows': 4,
                'cols': 3,
                'camera_index': lightboard_camera_index,
                'fps': 15.0,
                'frame_width': 640,
                'frame_height': 480,
                'start_enabled': False,   # decision node controls via lightboard/enable
            }],
        ),

        # ── Spearhead detector ───────────────────────────────
        Node(
            package='r2_lightboard_vision',
            executable='spearhead_detector',
            name='spearhead_detector',
            output='screen',
            condition=IfCondition(enable_spearhead),
            parameters=[{
                'camera_index': spearhead_camera_index,
                'fps': 30.0,
                'frame_width': 1920,
                'frame_height': 1080,
                'start_enabled': False,
                # libuvc backend (V4L2 broken on Jetson)
                'use_libuvc': True,
                'uvc_vid': 0x0c45,
                'uvc_pid': 0x6368,
                'uvc_lib_path': '',  # auto-detect, or set e.g. '/home/epoch/Desktop/libuvc/build/libuvc.so'
                # gray cylinder ROI (on 1920x1080 frame)
                # search around expected cylinder position
                'cyl_roi_x': 600,
                'cyl_roi_y': 200,
                'cyl_roi_w': 700,
                'cyl_roi_h': 700,
                'cyl_band_width': 110,
                # template matching
                'cyl_template_path': _template_path,
                'cyl_template_threshold': 0.3,
                'cyl_expected_width': 110.0,
            }],
        ),

        # ── Grab-scene detector (scene 1/2/3 confirm) ────────
        Node(
            package='r2_lightboard_vision',
            executable='grab_scene_detector',
            name='grab_scene_detector',
            output='screen',
            condition=IfCondition(enable_grab_scene),
            parameters=[{
                'camera_index': grab_scene_camera_index,
                'is_red_side': is_red_side,
                'fps': 15.0,
                'frame_width': 640,
                'frame_height': 480,
            }],
        ),
    ])
