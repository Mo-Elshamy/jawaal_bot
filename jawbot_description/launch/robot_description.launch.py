import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # 1. Declare Launch Arguments (Allows top-level bringup to toggle sim time)
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    publish_joints_arg = DeclareLaunchArgument(
        'publish_joints',
        default_value='true',
        description='Publish joint states using joint_state_publisher node if true'
    )

    # 2. Capture Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_joints = LaunchConfiguration('publish_joints')

    # 3. Setup project paths safely
    pkg_project_description = get_package_share_directory('jawbot_description')
    
    xacro_file = PathJoinSubstitution([
        pkg_project_description,
        'urdf', 
        'jawbot.urdf.xacro'
    ])

    # Safe evaluation of Xacro command into a string parameter
    robot_desc = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )
    
    # 4. Define Nodes
    
    # Publishes the 3D poses of the robot links based on URDF and joint states
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc,
        }]  
    )
    
    # Generates uniform/fake joint data for visual testing in RViz
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(publish_joints)
    )
    
    # RViz2 Visualization node with cleanly separated argument flags
    rviz_config_path = os.path.join(pkg_project_description, 'rviz', 'jawbot.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='sim_rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        use_sim_time_arg,
        publish_joints_arg,
        robot_state_publisher,
        joint_state_publisher_node,
        rviz_node
    ])