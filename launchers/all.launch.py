import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, EmitEvent, IncludeLaunchDescription, DeclareLaunchArgument
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # control drive node
    drive_control_dir = get_package_share_directory('drive_control')
    launch_drive_control_pkg = os.path.join(drive_control_dir, 'launch')

    return LaunchDescription([
        # drive_control include
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_drive_control_pkg, 'drive_launch.py'))
        ),
    ])
