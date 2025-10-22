#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate a simple launch description for testing."""
    
    # Get package directory
    pkg_dir = get_package_share_directory('amr_fleet_adapter')
    
    # Configuration files
    config_file = os.path.join(pkg_dir, 'config', 'fleet_config.yaml')
    
    # AMR Fleet Adapter Node
    amr_fleet_adapter_node = Node(
        package='amr_fleet_adapter',
        executable='amr_fleet_adapter_node',
        name='amr_fleet_adapter',
        output='screen',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    return LaunchDescription([
        amr_fleet_adapter_node,
    ])