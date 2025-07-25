#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
import math
import json
from typing import List, Optional, Dict
from .graph_generator import GraphGenerator


class GraphNavigator(Node):
    def __init__(self):
        super().__init__('graph_navigator')
        
        # Action client for Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10
        )
        self.goal_sub = self.create_subscription(
            String, 'graph_goal', self.goal_callback, 10
        )
        
        # Parameters
        self.declare_parameter('tolerance', 0.2)
        self.declare_parameter('max_linear_velocity', 0.3)
        self.declare_parameter('max_angular_velocity', 0.5)
        self.declare_parameter('use_nav2', True)
        
        # Graph generator instance
        self.graph_gen = GraphGenerator()
        self.graph = self.graph_gen.graph
        
        # Navigation state
        self.current_pose = None
        self.current_path = []
        self.current_goal_index = 0
        self.is_navigating = False
        
        # Control parameters
        self.tolerance = self.get_parameter('tolerance').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.use_nav2 = self.get_parameter('use_nav2').value
        
        # Timer for navigation control
        self.control_timer = self.create_timer(0.1, self.navigation_control_loop)
        
        self.get_logger().info('Graph Navigator Node initialized')

    def odom_callback(self, msg: Odometry):
        """Update current pose from odometry"""
        self.current_pose = msg.pose.pose

    def goal_callback(self, msg: String):
        """Handle goal requests in format 'start_node,goal_node'"""
        try:
            parts = msg.data.split(',')
            if len(parts) == 2:
                start_node, goal_node = parts[0].strip(), parts[1].strip()
                self.navigate_to_goal(start_node, goal_node)
            else:
                self.get_logger().error('Invalid goal format. Use: start_node,goal_node')
        except Exception as e:
            self.get_logger().error(f'Error processing goal: {e}')

    def find_nearest_node(self, pose) -> str:
        """Find the nearest graph node to current position"""
        if not pose:
            return None
        
        min_distance = float('inf')
        nearest_node = None
        
        current_x = pose.position.x
        current_y = pose.position.y
        
        for node_id, node_data in self.graph.items():
            node_x, node_y = node_data['position']
            distance = math.sqrt((current_x - node_x)**2 + (current_y - node_y)**2)
            
            if distance < min_distance:
                min_distance = distance
                nearest_node = node_id
        
        return nearest_node

    def navigate_to_goal(self, start_node: str, goal_node: str):
        """Navigate from start node to goal node"""
        if start_node not in self.graph or goal_node not in self.graph:
            self.get_logger().error(f'Invalid nodes: {start_node}, {goal_node}')
            return
        
        # Find path using Dijkstra's algorithm
        path = self.graph_gen.dijkstra(start_node, goal_node)
        
        if not path:
            self.get_logger().error(f'No path found from {start_node} to {goal_node}')
            return
        
        self.current_path = path
        self.current_goal_index = 0
        self.is_navigating = True
        
        self.get_logger().info(f'Starting navigation: {" -> ".join(path)}')
        
        # Publish the planned path
        self.graph_gen.publish_path(start_node, goal_node)
        
        if self.use_nav2:
            self.navigate_with_nav2()
        else:
            self.navigate_with_basic_control()

    def navigate_with_nav2(self):
        """Use Nav2 for navigation"""
        if not self.current_path or self.current_goal_index >= len(self.current_path):
            return
        
        # Wait for Nav2 action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available')
            return
        
        # Navigate to next waypoint
        self.navigate_to_next_waypoint()

    def navigate_to_next_waypoint(self):
        """Navigate to the next waypoint in the path"""
        if self.current_goal_index >= len(self.current_path):
            self.get_logger().info('Navigation completed!')
            self.is_navigating = False
            return
        
        current_target = self.current_path[self.current_goal_index]
        target_pos = self.graph[current_target]['position']
        
        # Create goal pose
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = target_pos[0]
        goal_msg.pose.pose.position.y = target_pos[1]
        goal_msg.pose.pose.position.z = 0.0
        
        # Calculate orientation towards next waypoint (if exists)
        if self.current_goal_index < len(self.current_path) - 1:
            next_target = self.current_path[self.current_goal_index + 1]
            next_pos = self.graph[next_target]['position']
            
            yaw = math.atan2(
                next_pos[1] - target_pos[1],
                next_pos[0] - target_pos[0]
            )
            
            goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        else:
            goal_msg.pose.pose.orientation.w = 1.0
        
        # Send goal
        self.get_logger().info(f'Navigating to waypoint: {current_target}')
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        """Handle Nav2 goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2')
            return
        
        self.get_logger().info('Goal accepted by Nav2')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        """Handle Nav2 navigation result"""
        result = future.result().result
        
        if result:
            self.get_logger().info(f'Reached waypoint: {self.current_path[self.current_goal_index]}')
            self.current_goal_index += 1
            
            # Continue to next waypoint
            if self.current_goal_index < len(self.current_path):
                self.navigate_to_next_waypoint()
            else:
                self.get_logger().info('Navigation path completed!')
                self.is_navigating = False
        else:
            self.get_logger().error('Navigation failed')
            self.is_navigating = False

    def navigate_with_basic_control(self):
        """Use basic control for navigation (fallback)"""
        self.get_logger().info('Using basic control for navigation')

    def navigation_control_loop(self):
        """Main navigation control loop for basic control"""
        if not self.is_navigating or self.use_nav2 or not self.current_pose:
            return
        
        if self.current_goal_index >= len(self.current_path):
            self.is_navigating = False
            return
        
        # Get current target
        current_target = self.current_path[self.current_goal_index]
        target_pos = self.graph[current_target]['position']
        
        # Calculate distance and angle to target
        dx = target_pos[0] - self.current_pose.position.x
        dy = target_pos[1] - self.current_pose.position.y
        distance = math.sqrt(dx**2 + dy**2)
        
        # Check if we've reached the current waypoint
        if distance < self.tolerance:
            self.get_logger().info(f'Reached waypoint: {current_target}')
            self.current_goal_index += 1
            
            if self.current_goal_index >= len(self.current_path):
                self.get_logger().info('Navigation completed!')
                self.is_navigating = False
                
                # Stop the robot
                cmd = Twist()
                self.cmd_vel_pub.publish(cmd)
            return
        
        # Calculate control commands
        target_yaw = math.atan2(dy, dx)
        
        # Get current yaw from quaternion
        q = self.current_pose.orientation
        current_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y**2 + q.z**2)
        )
        
        # Angular difference
        yaw_error = target_yaw - current_yaw
        
        # Normalize angle difference
        while yaw_error > math.pi:
            yaw_error -= 2 * math.pi
        while yaw_error < -math.pi:
            yaw_error += 2 * math.pi
        
        # Create control command
        cmd = Twist()
        
        # Angular velocity (proportional control)
        cmd.angular.z = max(min(2.0 * yaw_error, self.max_angular_vel), -self.max_angular_vel)
        
        # Linear velocity (reduce when turning)
        if abs(yaw_error) < 0.3:  # Move forward when roughly aligned
            cmd.linear.x = min(0.5 * distance, self.max_linear_vel)
        else:
            cmd.linear.x = 0.1  # Slow forward motion while turning
        
        self.cmd_vel_pub.publish(cmd)

    def cancel_navigation(self):
        """Cancel current navigation"""
        self.is_navigating = False
        self.current_path = []
        self.current_goal_index = 0
        
        # Stop the robot
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        
        # Cancel Nav2 goal if using Nav2
        if self.use_nav2:
            self.nav_client.cancel_all_goals()
        
        self.get_logger().info('Navigation cancelled')

    def get_available_nodes(self) -> List[str]:
        """Get list of available nodes for navigation"""
        return list(self.graph.keys())


def main(args=None):
    rclpy.init(args=args)
    
    navigator = GraphNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()