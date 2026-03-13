from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wheelbot_sim',
            executable='wheelbot_sim',
            name='wheelbot_sim',
            output='screen',
        ),
    ])
