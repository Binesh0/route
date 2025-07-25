#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    """Setup function for the launch file."""
    
    # Get package directory
    pkg_dir = get_package_share_directory('amr_fleet_adapter')
    
    # Configuration files
    fleet_config_file = PathJoinSubstitution([
        FindPackageShare('amr_fleet_adapter'),
        'config',
        LaunchConfiguration('fleet_config_file')
    ])
    
    # Resolve paths to absolute paths for configuration files
    route_json_file = os.path.join(pkg_dir, 'config', 'routes.json')
    map_yaml_file_param = LaunchConfiguration('map_yaml_file').perform(context)
    
    # Parameters dictionary
    fleet_adapter_params = {
        'fleet_name': LaunchConfiguration('fleet_name'),
        'robot_names': LaunchConfiguration('robot_names'),
        'nav_graph_file': LaunchConfiguration('nav_graph_file'),
        'robot_traits_file': LaunchConfiguration('robot_traits_file'),
        'route_json_file': route_json_file,
        'map_yaml_file': map_yaml_file_param,
        'perform_deliveries': LaunchConfiguration('perform_deliveries'),
        'perform_cleaning': LaunchConfiguration('perform_cleaning'),
        'accept_patrol_requests': LaunchConfiguration('accept_patrol_requests'),
        'discovery_timeout': LaunchConfiguration('discovery_timeout'),
        'task_capabilities_timeout': LaunchConfiguration('task_capabilities_timeout'),
    }
    
    # AMR Fleet Adapter Node
    amr_fleet_adapter_node = Node(
        package='amr_fleet_adapter',
        executable='amr_fleet_adapter_node',
        name='amr_fleet_adapter',
        output='screen',
        parameters=[fleet_adapter_params],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    # RMF Schedule Visualizer (optional)
    rmf_visualizer_node = Node(
        package='rmf_visualization_schedule',
        executable='rmf_visualizer_node',
        name='rmf_schedule_visualizer',
        output='screen',
        condition=LaunchConfiguration('use_rmf_visualizer')
    )
    
    return [
        amr_fleet_adapter_node,
        rmf_visualizer_node,
    ]


def generate_launch_description():
    """Generate the launch description."""
    
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'fleet_name',
            default_value='turtlebot_fleet',
            description='Name of the robot fleet'
        ),
        DeclareLaunchArgument(
            'robot_names',
            default_value='["turtlebot1"]',
            description='List of robot names in the fleet'
        ),
        DeclareLaunchArgument(
            'fleet_config_file',
            default_value='fleet_config.yaml',
            description='Fleet configuration file name'
        ),
        DeclareLaunchArgument(
            'nav_graph_file',
            default_value='',
            description='Navigation graph file path (optional)'
        ),
        DeclareLaunchArgument(
            'robot_traits_file',
            default_value='',
            description='Robot traits configuration file path (optional)'
        ),
        DeclareLaunchArgument(
            'map_yaml_file',
            default_value='',
            description='Map YAML file path'
        ),
        DeclareLaunchArgument(
            'perform_deliveries',
            default_value='true',
            description='Whether the fleet can perform delivery tasks'
        ),
        DeclareLaunchArgument(
            'perform_cleaning',
            default_value='false',
            description='Whether the fleet can perform cleaning tasks'
        ),
        DeclareLaunchArgument(
            'accept_patrol_requests',
            default_value='true',
            description='Whether the fleet accepts patrol requests'
        ),
        DeclareLaunchArgument(
            'discovery_timeout',
            default_value='60.0',
            description='Robot discovery timeout in seconds'
        ),
        DeclareLaunchArgument(
            'task_capabilities_timeout',
            default_value='30.0',
            description='Task capabilities timeout in seconds'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level for the nodes'
        ),
        DeclareLaunchArgument(
            'use_rmf_visualizer',
            default_value='true',
            description='Whether to launch RMF schedule visualizer'
        ),
    ]
    
    return LaunchDescription(declared_arguments + [
        OpaqueFunction(function=launch_setup)
    ])