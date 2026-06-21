import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_gazebo = get_package_share_directory('jawbot_gazebo')
    pkg_description = get_package_share_directory('jawbot_description')

    # Configure Environment paths so Gazebo can read your custom assets
    models_dir_gz = os.path.join(pkg_gazebo, 'models')
    install_dir_desc = os.path.join(pkg_description, '..')
    
    # Format the string explicitly with a colon ":" separator!
    resource_paths = f"{os.path.abspath(install_dir_desc)}:{os.path.abspath(models_dir_gz)}"
    
    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_paths
    )


    robot_description_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'robot_description.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'publish_joints': 'false'
        }.items()
    )

    # Include Gazebo Sim Harmonic simulator engine
    world_sdf_path = os.path.join(pkg_gazebo, 'worlds', 'jawbot_world.sdf')
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_sdf_path}'}.items()
    )

    # Spawner entity node to generate your robot model inside the environment scene
    sdf_model_path = os.path.join(pkg_gazebo, 'models', 'jawbot', 'model.sdf')
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', sdf_model_path, '-name', 'jawbot', '-z', '0.05'],
        output='screen'
    )

    # ROS-Gz Bridge parameter runner loaded via your custom translator configuration
    bridge_config = os.path.join(pkg_gazebo, 'config', 'ros_gz_bridge.yaml')
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
        output='screen'
    )

    return LaunchDescription([
        set_resource_path,
        robot_description_bringup, # <-- Clean inclusion call replaces the raw node block
        gazebo_sim,
        spawn_robot,
        ros_gz_bridge
    ])