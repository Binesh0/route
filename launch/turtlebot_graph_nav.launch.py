#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    pkg_turtlebot_graph_nav = FindPackageShare('turtlebot_graph_nav')
    pkg_turtlebot3_gazebo = FindPackageShare('turtlebot3_gazebo')
    pkg_nav2_bringup = FindPackageShare('nav2_bringup')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map_file')
    world = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    use_rviz = LaunchConfiguration('use_rviz')
    use_nav2 = LaunchConfiguration('use_nav2')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value=PathJoinSubstitution([pkg_turtlebot_graph_nav, 'config', 'map_graph.json']),
        description='Full path to map graph JSON file'
    )
    
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([pkg_turtlebot3_gazebo, 'worlds', 'turtlebot3_world.world']),
        description='Full path to world file to load'
    )
    
    declare_x_pose_cmd = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='Initial x coordinate of the robot'
    )
    
    declare_y_pose_cmd = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Initial y coordinate of the robot'
    )
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RVIZ'
    )
    
    declare_use_nav2_cmd = DeclareLaunchArgument(
        'use_nav2',
        default_value='true',
        description='Whether to use Nav2 for navigation'
    )
    
    # TurtleBot3 Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_world.launch.py'])
        ]),
        launch_arguments={
            'world': world,
            'x_pose': x_pose,
            'y_pose': y_pose,
        }.items()
    )
    
    # Nav2 launch (conditional)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_nav2_bringup, 'launch', 'navigation_launch.py'])
        ]),
        condition=IfCondition(use_nav2),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # Graph Generator Node
    graph_generator_node = Node(
        package='turtlebot_graph_nav',
        executable='graph_generator',
        name='graph_generator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_file': map_file,
            'frame_id': 'map'
        }]
    )
    
    # Graph Navigator Node
    graph_navigator_node = Node(
        package='turtlebot_graph_nav',
        executable='graph_navigator',
        name='graph_navigator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'tolerance': 0.3,
            'max_linear_velocity': 0.5,
            'max_angular_velocity': 1.0,
            'use_nav2': use_nav2
        }]
    )
    
    # RVIZ Node
    rviz_config_file = PathJoinSubstitution([
        pkg_turtlebot_graph_nav, 'config', 'turtlebot_graph_nav.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_config_file],
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # Goal Publisher Node (for testing)
    goal_publisher_node = Node(
        package='turtlebot_graph_nav',
        executable='goal_publisher',
        name='goal_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_file_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_x_pose_cmd)
    ld.add_action(declare_y_pose_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_nav2_cmd)
    
    # Add the actions to the launch description
    ld.add_action(gazebo_launch)
    ld.add_action(nav2_launch)
    ld.add_action(graph_generator_node)
    ld.add_action(graph_navigator_node)
    ld.add_action(rviz_node)
    
    return ld