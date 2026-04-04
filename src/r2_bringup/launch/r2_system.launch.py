from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bringup_share = get_package_share_directory('r2_bringup')
    default_map_yaml = os.path.join(bringup_share, 'maps', 'wulin', 'map.yaml')

    map_yaml = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=default_map_yaml,
            description='Full path to map yaml file'
        ),

        # 静态地图服务器（发布 /map）
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': map_yaml
            }]
        ),

        # 让 map_server 自动进入 active（否则 lifecycle node 不会真正工作）
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server']
            }]
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
    ])
