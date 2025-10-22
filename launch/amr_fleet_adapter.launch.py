#!/usr/bin/env python3

"""
AMR Fleet Adapter Launch File

Launch file for the Python-based AMR Fleet Adapter with delivery capabilities.
Supports multiple TurtleBots with Nav2 navigation and comprehensive delivery management.
"""

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
    
    # Resolve file paths
    config_file = os.path.join(pkg_dir, 'config', 'fleet_config.yaml')
    route_json_file = os.path.join(pkg_dir, 'config', 'routes.json')
    
    # Get launch configuration values
    fleet_name = LaunchConfiguration('fleet_name').perform(context)
    robot_names = LaunchConfiguration('robot_names').perform(context)
    map_yaml_file = LaunchConfiguration('map_yaml_file').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)
    
    # Resolve map file path
    if map_yaml_file and not os.path.isabs(map_yaml_file):
        map_yaml_file = os.path.join(pkg_dir, map_yaml_file)
    
    # Parameters for the fleet adapter
    fleet_adapter_params = {
        'fleet_name': fleet_name,
        'robot_names': eval(robot_names),  # Convert string representation to list
        'route_json_file': route_json_file,
        'map_yaml_file': map_yaml_file,
        'max_concurrent_deliveries': LaunchConfiguration('max_concurrent_deliveries'),
        'auto_assign_tasks': LaunchConfiguration('auto_assign_tasks'),
        'delivery_timeout_minutes': LaunchConfiguration('delivery_timeout_minutes'),
        'enable_rmf_integration': LaunchConfiguration('enable_rmf_integration'),
        'perform_deliveries': LaunchConfiguration('perform_deliveries'),
        'perform_cleaning': LaunchConfiguration('perform_cleaning'),
        'accept_patrol_requests': LaunchConfiguration('accept_patrol_requests'),
        'discovery_timeout': LaunchConfiguration('discovery_timeout'),
        'task_capabilities_timeout': LaunchConfiguration('task_capabilities_timeout'),
    }
    
    # Main AMR Fleet Adapter Node
    amr_fleet_adapter_node = Node(
        package='amr_fleet_adapter',
        executable='amr_fleet_adapter_node.py',
        name='amr_fleet_adapter',
        output='screen',
        parameters=[fleet_adapter_params],
        arguments=['--ros-args', '--log-level', log_level],
        emulate_tty=True
    )
    
    nodes_to_launch = [amr_fleet_adapter_node]
    
    # Optional: Launch RMF visualizer if requested
    if LaunchConfiguration('use_rmf_visualizer').perform(context).lower() == 'true':
        try:
            rmf_visualizer_node = Node(
                package='rmf_visualization_schedule',
                executable='rmf_visualizer_node',
                name='rmf_schedule_visualizer',
                output='screen'
            )
            nodes_to_launch.append(rmf_visualizer_node)
        except Exception:
            # RMF visualizer not available, continue without it
            pass
    
    # Optional: Launch RViz for visualization
    if LaunchConfiguration('use_rviz').perform(context).lower() == 'true':
        rviz_config_file = os.path.join(pkg_dir, 'config', 'fleet_visualization.rviz')
        if os.path.exists(rviz_config_file):
            rviz_node = Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_file],
                output='screen'
            )
            nodes_to_launch.append(rviz_node)
    
    return nodes_to_launch


def generate_launch_description():
    """Generate the launch description."""
    
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'fleet_name',
            default_value='delivery_fleet',
            description='Name of the robot fleet'
        ),
        DeclareLaunchArgument(
            'robot_names',
            default_value='["turtlebot1", "turtlebot2"]',
            description='List of robot names in the fleet (as string representation of Python list)'
        ),
        DeclareLaunchArgument(
            'map_yaml_file',
            default_value='maps/office_map.yaml',
            description='Path to map YAML file (relative to package or absolute)'
        ),
        DeclareLaunchArgument(
            'max_concurrent_deliveries',
            default_value='10',
            description='Maximum number of concurrent delivery tasks'
        ),
        DeclareLaunchArgument(
            'auto_assign_tasks',
            default_value='true',
            description='Whether to automatically assign tasks to available robots'
        ),
        DeclareLaunchArgument(
            'delivery_timeout_minutes',
            default_value='120',
            description='Delivery task timeout in minutes'
        ),
        DeclareLaunchArgument(
            'enable_rmf_integration',
            default_value='true',
            description='Whether to enable RMF integration'
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
            description='Log level for the nodes (debug, info, warn, error)'
        ),
        DeclareLaunchArgument(
            'use_rmf_visualizer',
            default_value='false',
            description='Whether to launch RMF schedule visualizer'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Whether to launch RViz for visualization'
        ),
    ]
    
    return LaunchDescription(declared_arguments + [
        OpaqueFunction(function=launch_setup)
    ])