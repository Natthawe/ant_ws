from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='opencv_tools',
            executable='streaming_camera_cv',
            output='screen',
        ),
    ])