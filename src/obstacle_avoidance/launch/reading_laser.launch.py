from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    reading_laser = Node(
        package='obstacle_avoidance',
        executable='reading_laser',
        output='screen',
        remappings=[
            ('laser_scan', 'scan')
        ]
    )

    return LaunchDescription([
        reading_laser
    ])