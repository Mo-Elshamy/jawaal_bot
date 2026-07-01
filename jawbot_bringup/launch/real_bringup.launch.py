import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # --- 1. Define Package Paths ---
    pkg_description = FindPackageShare('jawbot_description')
    pkg_bringup = FindPackageShare('jawbot_bringup')

    # --- 2. Declare Dynamic Arguments (Level 2 Architecture) ---
    serial_port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='The assigned Linux USB port for the Pi Pico'
    )

    # --- 3. Process the URDF / Xacro ---
    # This command explicitly injects your terminal argument into the URDF!
    xacro_file = PathJoinSubstitution([pkg_description, 'urdf', 'jawbot.urdf.xacro'])
    robot_description_content = Command([
        'xacro ', xacro_file, ' serial_port:=', LaunchConfiguration('port')
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # --- 4. Load Controller Parameters ---
    controllers_file = PathJoinSubstitution(
        [pkg_bringup, 'config', 'jawbot_controllers.yaml']
    )

    # --- 5. Define ROS 2 Nodes ---
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # --- 6. Node to fix timestamp issue ---
    node_twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        remappings=[
            ('/cmd_vel_in', '/cmd_vel'),
            ('/cmd_vel_out', '/jawbot_base_controller/cmd_vel')
        ]
    )

    # The core engine that loads custom C++ hardware plugin
    node_controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_file],
        output='both'
    )

    # Spawner for the Joint State Broadcaster (Reads encoder telemetry)
    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    # Spawner for the Diff Drive Controller (Translates Twist commands to wheel velocities)
    spawn_ddc = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['jawbot_base_controller', '--controller-manager', '/controller_manager'],
    )

    # --- 7. Orchestrate Startup Sequence ---
    # Delay spawning controllers until the controller_manager is fully up and running
    delay_jsb_after_manager = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=node_controller_manager,
            on_start=[spawn_jsb],
        )
    )

    delay_ddc_after_manager = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=node_controller_manager,
            on_start=[spawn_ddc],
        )
    )

    return LaunchDescription([
        serial_port_arg,
        node_robot_state_publisher,
        node_controller_manager,
        node_twist_stamper,
        delay_jsb_after_manager,
        delay_ddc_after_manager
    ])