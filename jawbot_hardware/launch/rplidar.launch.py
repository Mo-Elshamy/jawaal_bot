import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    hardware_share = get_package_share_directory('jawbot_hardware')
    rplidar_params = os.path.join(hardware_share, 'config', 'rplidar_params.yaml')

    return LaunchDescription([
        Node(
            package='rplidar_ros',
            executable='rplidar_composition', 
            name='rplidar_node',
            parameters=[rplidar_params],
            output='screen'
        )
    ])