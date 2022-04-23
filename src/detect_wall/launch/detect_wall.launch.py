from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    detect_wall = Node(
        package='detect_wall',
        executable='detect_wall_node',
        output='screen'
    )

    return LaunchDescription([
        detect_wall
    ])    