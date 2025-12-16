from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
