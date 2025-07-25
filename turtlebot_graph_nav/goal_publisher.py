#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys


class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        
        # Publisher for navigation goals
        self.goal_pub = self.create_publisher(String, 'graph_goal', 10)
        
        # Timer to publish periodic goals (for demo)
        # self.timer = self.create_timer(10.0, self.publish_demo_goal)
        
        self.available_nodes = [
            'start', 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'center'
        ]
        
        self.demo_goals = [
            ('start', 'center'),
            ('center', 'D'),
            ('D', 'F'),
            ('F', 'start'),
            ('start', 'B'),
            ('B', 'E'),
            ('E', 'A'),
            ('A', 'start')
        ]
        
        self.demo_index = 0
        
        self.get_logger().info('Goal Publisher Node initialized')
        self.get_logger().info(f'Available nodes: {", ".join(self.available_nodes)}')
        self.print_usage()

    def print_usage(self):
        """Print usage instructions"""
        self.get_logger().info('=== Goal Publisher Usage ===')
        self.get_logger().info('Send navigation goals using:')
        self.get_logger().info('ros2 topic pub /graph_goal std_msgs/String "data: start,center"')
        self.get_logger().info('Or use the interactive mode with command line arguments')
        self.get_logger().info(f'Available nodes: {", ".join(self.available_nodes)}')
        self.get_logger().info('Example: ros2 run turtlebot_graph_nav goal_publisher start center')

    def publish_goal(self, start_node: str, goal_node: str):
        """Publish a navigation goal"""
        if start_node not in self.available_nodes:
            self.get_logger().error(f'Invalid start node: {start_node}')
            return False
        
        if goal_node not in self.available_nodes:
            self.get_logger().error(f'Invalid goal node: {goal_node}')
            return False
        
        goal_msg = String()
        goal_msg.data = f'{start_node},{goal_node}'
        
        self.goal_pub.publish(goal_msg)
        self.get_logger().info(f'Published goal: {start_node} -> {goal_node}')
        return True

    def publish_demo_goal(self):
        """Publish demo goals in sequence"""
        if self.demo_index >= len(self.demo_goals):
            self.demo_index = 0
        
        start, goal = self.demo_goals[self.demo_index]
        self.publish_goal(start, goal)
        self.demo_index += 1

    def run_interactive_mode(self):
        """Run interactive mode for manual goal input"""
        self.get_logger().info('=== Interactive Mode ===')
        self.get_logger().info('Enter navigation goals in the format: start_node goal_node')
        self.get_logger().info('Type "quit" to exit, "demo" for demo mode, "nodes" to list available nodes')
        
        try:
            while True:
                user_input = input('\nEnter goal (start goal): ').strip()
                
                if user_input.lower() == 'quit':
                    break
                elif user_input.lower() == 'demo':
                    self.start_demo_mode()
                elif user_input.lower() == 'nodes':
                    print(f'Available nodes: {", ".join(self.available_nodes)}')
                else:
                    parts = user_input.split()
                    if len(parts) == 2:
                        start, goal = parts
                        self.publish_goal(start, goal)
                    else:
                        print('Invalid format. Use: start_node goal_node')
        
        except KeyboardInterrupt:
            pass
        except EOFError:
            pass

    def start_demo_mode(self):
        """Start demo mode with automatic goal publishing"""
        self.get_logger().info('Starting demo mode...')
        self.timer = self.create_timer(8.0, self.publish_demo_goal)

    def process_command_line_args(self, args):
        """Process command line arguments for direct goal publishing"""
        if len(args) >= 3:  # script_name, start, goal
            start_node = args[1]
            goal_node = args[2]
            
            self.get_logger().info(f'Command line goal: {start_node} -> {goal_node}')
            
            # Wait a moment for the system to initialize
            import time
            time.sleep(2)
            
            return self.publish_goal(start_node, goal_node)
        
        return False


def main(args=None):
    rclpy.init(args=args)
    
    goal_publisher = GoalPublisher()
    
    # Check if command line arguments were provided
    import sys
    if len(sys.argv) >= 3:
        # Direct goal publishing mode
        success = goal_publisher.process_command_line_args(sys.argv)
        if success:
            # Keep the node alive for a moment to ensure message is sent
            import time
            time.sleep(1)
        goal_publisher.destroy_node()
        rclpy.shutdown()
        return
    
    try:
        # Check if running in interactive terminal
        if sys.stdin.isatty():
            # Run interactive mode in a separate thread
            import threading
            interactive_thread = threading.Thread(target=goal_publisher.run_interactive_mode)
            interactive_thread.daemon = True
            interactive_thread.start()
        
        rclpy.spin(goal_publisher)
    
    except KeyboardInterrupt:
        pass
    finally:
        goal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()