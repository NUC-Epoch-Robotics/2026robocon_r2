from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
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
                # 矛头基准点 (1号矛头), 间距200mm, 顺序 2→1→4→5→3→6
                'spearhead_base_x': -0.002,
                'spearhead_base_y': -0.959,
                'spearhead_base_spin': -0.622,
                'spearhead_spacing': 0.2,
                'zone1_route': [2, 1, 4, 5, 3, 6, 7, 8],
                # 其他 Zone1 导航点 (非矛头)
                'zone1_point_2_x': 1.7,
                'zone1_point_2_y': 1.42,
                'zone1_point_2_spin': -0.622,
                'zone1_point_3_x': 1.9,
                'zone1_point_3_y': 1.42,
                'zone1_point_3_spin': -0.622,
                # Zone2 固定路线 (6点, 硬编码台阶)
                'use_fixed_zone2_route': True,
                # 梅花林: 1.2m网格, col0_y=2.531 col1_y=1.41 col2_y=0.289
                # row_x: 3.0→4.2→5.4→6.6
                # 高度: 2,1,2 / 3,2,1 / 2,3,2 / 1,2,1
                # 入口抓取: block0(col0,y=0.289,h=2) is_finsh=2  block1(col1,y=1.41,h=1) is_finsh=1
                'entry_approach_x': 1.6,
                'entry_block0_x': 3.0,
                'entry_block0_y': 0.289,
                'entry_block0_is_finsh': 2,
                'entry_block2_x': 3.0,
                'entry_block2_y': 1.41,
                'entry_block2_is_finsh': 1,
                # 点0: row0col1(3.0,1.41) h=1 入口, 无抓取, 需转向
                'zone2_fixed_0_x': 3.0,
                'zone2_fixed_0_y': 1.41,
                'zone2_fixed_0_yaw': -0.422,
                'zone2_fixed_0_qx': 0.0,
                'zone2_fixed_0_qy': 0.0,
                'zone2_fixed_0_qz': 0.0,
                'zone2_fixed_0_qw': 1.0,
                'zone2_fixed_0_use_rotate': True,
                'zone2_fixed_0_rqx': 0.0,
                'zone2_fixed_0_rqy': 0.0,
                'zone2_fixed_0_rqz': -0.707,
                'zone2_fixed_0_rqw': 0.707,
                'zone2_fixed_0_approach_x': 0.0,
                'zone2_fixed_0_approach_y': 0.0,
                'zone2_fixed_0_block_height': 0,
                'zone2_fixed_0_stand_height': 0,
                # 点1: row0col0(3.0,0.289) h=2 站row0col1(h=1)→抓row0col0(h=2) is_finsh=1
                'zone2_fixed_1_x': 3.0,
                'zone2_fixed_1_y': 0.289,
                'zone2_fixed_1_yaw': -0.222,
                'zone2_fixed_1_qx': 0.0,
                'zone2_fixed_1_qy': 0.0,
                'zone2_fixed_1_qz': -0.707,
                'zone2_fixed_1_qw': 0.707,
                'zone2_fixed_1_use_rotate': True,
                'zone2_fixed_1_rqx': 0.0,
                'zone2_fixed_1_rqy': 0.0,
                'zone2_fixed_1_rqz': 0.0,
                'zone2_fixed_1_rqw': 1.0,
                'zone2_fixed_1_approach_x': 3.0,
                'zone2_fixed_1_approach_y': 1.41,
                'zone2_fixed_1_block_height': 2,
                'zone2_fixed_1_stand_height': 1,
                # 点2: row1col0(4.2,0.289) h=3 站row0col0(h=2)→抓row1col0(h=3) is_finsh=1
                'zone2_fixed_2_x': 4.2,
                'zone2_fixed_2_y': 0.289,
                'zone2_fixed_2_yaw': -0.022,
                'zone2_fixed_2_qx': 0.0,
                'zone2_fixed_2_qy': 0.0,
                'zone2_fixed_2_qz': 0.0,
                'zone2_fixed_2_qw': 1.0,
                'zone2_fixed_2_approach_x': 3.0,
                'zone2_fixed_2_approach_y': 0.289,
                'zone2_fixed_2_block_height': 3,
                'zone2_fixed_2_stand_height': 2,
                # 点3: row2col0(5.4,0.289) h=2 站row1col0(h=3)→抓row2col0(h=2) is_finsh=3
                'zone2_fixed_3_x': 5.4,
                'zone2_fixed_3_y': 0.289,
                'zone2_fixed_3_yaw': -0.222,
                'zone2_fixed_3_qx': 0.0,
                'zone2_fixed_3_qy': 0.0,
                'zone2_fixed_3_qz': 0.0,
                'zone2_fixed_3_qw': 1.0,
                'zone2_fixed_3_approach_x': 4.2,
                'zone2_fixed_3_approach_y': 0.289,
                'zone2_fixed_3_block_height': 2,
                'zone2_fixed_3_stand_height': 3,
                # 点4: row3col0(6.6,0.289) h=1 站row2col0(h=2)→抓row3col0(h=1) is_finsh=3
                'zone2_fixed_4_x': 6.6,
                'zone2_fixed_4_y': 0.289,
                'zone2_fixed_4_yaw': -0.422,
                'zone2_fixed_4_qx': 0.0,
                'zone2_fixed_4_qy': 0.0,
                'zone2_fixed_4_qz': 0.0,
                'zone2_fixed_4_qw': 1.0,
                'zone2_fixed_4_approach_x': 5.4,
                'zone2_fixed_4_approach_y': 0.289,
                'zone2_fixed_4_block_height': 1,
                'zone2_fixed_4_stand_height': 2,
                # 点5: 出口(7.5,0.289) 无抓取, 直接离开
                'zone2_fixed_5_x': 7.5,
                'zone2_fixed_5_y': 0.289,
                'zone2_fixed_5_yaw': -0.622,
                'zone2_fixed_5_qx': 0.0,
                'zone2_fixed_5_qy': 0.0,
                'zone2_fixed_5_qz': 0.0,
                'zone2_fixed_5_qw': 1.0,
                'zone2_fixed_5_approach_x': 0.0,
                'zone2_fixed_5_approach_y': 0.0,
                'zone2_fixed_5_block_height': 0,
                'zone2_fixed_5_stand_height': 0,
            }],
        ),

        # ── Fake hardware (simulation / testing) ─────────────
        Node(
            package='r2_fake_lower_body',
            executable='fake_lower_body_node',
            name='fake_lower_body_node',
            output='screen',
        ),
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
                'rows': 3,
                'cols': 4,
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
                'fps': 20.0,
                'frame_width': 640,
                'frame_height': 480,
                'start_enabled': False,
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
