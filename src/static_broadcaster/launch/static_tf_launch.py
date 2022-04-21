from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='static_broadcaster',
            executable='static_tf2_node',
            output='screen',
        ),
    ])