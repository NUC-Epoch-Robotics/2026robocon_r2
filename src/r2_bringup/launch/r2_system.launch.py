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
                # Zone1 三个测试坐标
                'zone1_point_1_x': -0.352,
                'zone1_point_1_y': -0.959,
                'zone1_point_1_spin': -0.622,
                'zone1_point_2_x': 1.7,
                'zone1_point_2_y': 1.45,
                'zone1_point_2_spin': -0.622,
                'zone1_point_3_x': 1.9,
                'zone1_point_3_y': 1.45,
                'zone1_point_3_spin': -0.622,
                'zone1_route': [1, 2, 3],
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
