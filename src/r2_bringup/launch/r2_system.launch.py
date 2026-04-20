from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    enable_lightboard = LaunchConfiguration('enable_lightboard')
    lightboard_camera_index = LaunchConfiguration('lightboard_camera_index')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_lightboard',
            default_value='true',
            description='Enable 3x4 lightboard detector node'
        ),
        DeclareLaunchArgument(
            'lightboard_camera_index',
            default_value='0',
            description='Camera index for lightboard detector'
        ),

        # 你原来的系统
        Node(
            package='r2_decision',
            executable='r2_decision_node',
            output='screen'
        ),
        Node(
            package='r2_fake_lower_body',
            executable='fake_lower_body_node',
            output='screen'
        ),
        Node(
            package='r2_fake_upper_body',
            executable='fake_upper_body_node',
            output='screen'
        ),
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
            }]
        ),
    ])
