# Copyright 2025
# WebSocket Controller integrated launch file for CRANE V2+

from ament_index_python.packages import get_package_share_directory
from crane_plus_description.robot_description_loader import RobotDescriptionLoader
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Launch arguments
    declare_port_name = DeclareLaunchArgument(
        'port_name',
        default_value='/dev/ttyUSB0',
        description='Set port name for CRANE V2+.'
    )

    declare_rosbridge_port = DeclareLaunchArgument(
        'rosbridge_port',
        default_value='9090',
        description='ROSBridge WebSocket port.'
    )

    declare_enable_websocket_controller = DeclareLaunchArgument(
        'enable_websocket_controller',
        default_value='true',
        description='Enable WebSocket controller.'
    )

    declare_controller_script = DeclareLaunchArgument(
        'controller_script',
        default_value='crane_plus_websocket_controller.py',
        description='WebSocket controller script to launch.'
    )

    # Robot description
    description_loader = RobotDescriptionLoader()
    description_loader.port_name = LaunchConfiguration('port_name')
    description = description_loader.load()

    # MoveIt2 move_group
    move_group = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('crane_plus_moveit_config'),
                '/launch/run_move_group.launch.py']),
            launch_arguments={
                'loaded_description': description
            }.items()
        )

    # Hardware control
    control_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('crane_plus_control'),
                '/launch/crane_plus_control.launch.py']),
            launch_arguments={'loaded_description': description}.items()
        )

    # ROSBridge WebSocket server
    rosbridge_server = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        parameters=[{
            'port': LaunchConfiguration('rosbridge_port'),
            'address': '0.0.0.0'
        }],
        output='screen'
    )

    # WebSocket controller (Python script) with delay
    controller_script_path = os.path.join(
        get_package_share_directory('crane_plus_examples'),
        'src'
    )
    
    # Wait for services to be ready before starting controller
    websocket_controller = ExecuteProcess(
        cmd=[
            'bash', '-c', 
            f'sleep 5 && cd {controller_script_path} && python3 crane_plus_websocket_controller.py'
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_websocket_controller'))
    )

    return LaunchDescription([
        declare_port_name,
        declare_rosbridge_port,
        declare_enable_websocket_controller,
        declare_controller_script,
        
        # Start robot systems
        move_group,
        control_node,
        
        # Start WebSocket infrastructure  
        rosbridge_server,
        
        # Start WebSocket controller (with delay)
        websocket_controller
    ])