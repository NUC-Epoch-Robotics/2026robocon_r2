import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

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
    sim_mode = LaunchConfiguration('sim_mode')

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
        DeclareLaunchArgument(
            'sim_mode', default_value='false',
            description='Simulation mode: print decision traces, no hardware required'),

        # ── Decision node ────────────────────────────────────
        Node(
            package='r2_decision',
            executable='r2_decision_node',
            name='r2_decision_node',
            output='screen',
            parameters=[{
                'sim_mode': sim_mode,
                'nav_frame_id': 'odom',
                # ── 矛头/吸盘指令 ──
                'spearhead_extend_cmd': 2,   # 伸吸盘指令 (zhuangtai 字段)
                # ── 里程计微调目标点: Nav2 到矛头点后, 用 odom 对齐到此处再抓 ──
                'fine_tune_target_x': 0.710,
                'fine_tune_target_y': -1.170,
                'fine_tune_target_yaw': 0.0,
                # 微调参数 (bang-bang)
                'fine_tune_xy_threshold': 0.01,
                'fine_tune_yaw_threshold': 0.05,
                'fine_tune_stable_required': 5,
                'fine_tune_timeout_s': 15.0,
                'fine_tune_speed_x': 0.05,
                'fine_tune_speed_y': 0.05,
                'fine_tune_speed_yaw': 0.2,
                # 唯一要抓的矛头坐标 (id=5), 其余矛头不抓
                'spearhead_base_x': 0.451,
                'spearhead_base_y': -0.838,
                'spearhead_base_z': -0.622,
                'zone1_route': [5],
                # 其他 Zone1 导航点 (非矛头)
                'zone1_point_2_x': 1.7,
                'zone1_point_2_y': 1.42,
                'zone1_point_2_z': -0.622,
                'zone1_point_3_x': 1.9,
                'zone1_point_3_y': 1.42,
                'zone1_point_3_z': -0.622,
                # Zone2 路线: false=用灯板动态路线 (对接后看灯板自动规划), true=固定路线
                'use_fixed_zone2_route': True,
                'zone2_fixed_count': 4,
                # 梅花林: 1.2m网格, col0_y=0.289 col1_y=1.41 col2_y=2.531
                # row_x: 3.0→4.2→5.4→6.6
                # 高度: 2,1,2 / 3,2,1 / 2,3,2 / 1,2,1
                # 注: 入口块(col1,2.0,1.41)由 ENTRY_GRAB 入口序列接管, 不在 fixed 点表里
                # 入口抓取: block0(col0,y=0.289,h=2) is_finsh=2  block1(col1,y=1.41,h=1) is_finsh=1
                'entry_approach_x': 1.6,
                'entry_block0_x': 2.0,
                'entry_block0_y': 0.289,
                'entry_block0_is_finsh': 2,
                'entry_block2_x': 2.0,
                'entry_block2_y': 1.41,
                'entry_block2_is_finsh': 1,
                # 入口第一次上台阶点 (1.8,1.41): 抓完倒回→走到这里→上台阶. 二区唯一上台阶点
                'entry_stair1_x': 1.8,
                'entry_stair1_y': 1.41,
                # 入口转向点 x=3.0 (y同块2): 上台阶#1后走到这里顺时针转90°
                'entry_rotate_x': 3.0,
                # 点0: row1col0(4.2,0.289) 站col0row0(3.0,0.289) h=3 抓 h=2 → 高抓低 is_finsh=3, 抓完下台阶
                'zone2_fixed_0_x': 4.2,
                'zone2_fixed_0_y': 0.289,
                'zone2_fixed_0_z': -0.222,
                'zone2_fixed_0_qx': 0.0,
                'zone2_fixed_0_qy': 0.0,
                'zone2_fixed_0_qz': 0.0,
                'zone2_fixed_0_qw': 1.0,
                'zone2_fixed_0_use_rotate': False,
                'zone2_fixed_0_rqx': 0.0,
                'zone2_fixed_0_rqy': 0.0,
                'zone2_fixed_0_rqz': 0.0,
                'zone2_fixed_0_rqw': 1.0,
                'zone2_fixed_0_approach_x': 3.0,
                'zone2_fixed_0_approach_y': 0.289,
                'zone2_fixed_0_block_height': 2,
                'zone2_fixed_0_stand_height': 3,
                'zone2_fixed_0_stair_cmd': 2,
                # 点1: row2col0(5.4,0.289) 无抓取, 下台阶
                'zone2_fixed_1_x': 5.4,
                'zone2_fixed_1_y': 0.289,
                'zone2_fixed_1_z': -0.022,
                'zone2_fixed_1_qx': 0.0,
                'zone2_fixed_1_qy': 0.0,
                'zone2_fixed_1_qz': 0.0,
                'zone2_fixed_1_qw': 1.0,
                'zone2_fixed_1_use_rotate': False,
                'zone2_fixed_1_rqx': 0.0,
                'zone2_fixed_1_rqy': 0.0,
                'zone2_fixed_1_rqz': 0.0,
                'zone2_fixed_1_rqw': 1.0,
                'zone2_fixed_1_approach_x': 0.0,
                'zone2_fixed_1_approach_y': 0.0,
                'zone2_fixed_1_block_height': 0,
                'zone2_fixed_1_stand_height': 0,
                'zone2_fixed_1_stair_cmd': 2,
                # 点2: row3col0(6.6,0.289) 无抓取, 下台阶出梅花林
                'zone2_fixed_2_x': 6.6,
                'zone2_fixed_2_y': 0.289,
                'zone2_fixed_2_z': -0.222,
                'zone2_fixed_2_qx': 0.0,
                'zone2_fixed_2_qy': 0.0,
                'zone2_fixed_2_qz': 0.0,
                'zone2_fixed_2_qw': 1.0,
                'zone2_fixed_2_approach_x': 0.0,
                'zone2_fixed_2_approach_y': 0.0,
                'zone2_fixed_2_block_height': 0,
                'zone2_fixed_2_stand_height': 0,
                'zone2_fixed_2_stair_cmd': 2,
                # 点3: 出口(7.5,0.289) 无抓取, 结束二区固定路线
                'zone2_fixed_3_x': 7.5,
                'zone2_fixed_3_y': 0.289,
                'zone2_fixed_3_z': -0.622,
                'zone2_fixed_3_qx': 0.0,
                'zone2_fixed_3_qy': 0.0,
                'zone2_fixed_3_qz': 0.0,
                'zone2_fixed_3_qw': 1.0,
                'zone2_fixed_3_approach_x': 0.0,
                'zone2_fixed_3_approach_y': 0.0,
                'zone2_fixed_3_block_height': 0,
                'zone2_fixed_3_stand_height': 0,
                'zone2_fixed_3_stair_cmd': 0,
            }],
        ),

        # ── Fake hardware (simulation / testing) ─────────────
        Node(
            package='r2_fake_upper_body',
            executable='fake_upper_body_node',
            name='fake_upper_body_node',
            output='screen',
        ),

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
