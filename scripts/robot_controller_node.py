#!/usr/bin/env python3

"""
Robot Controller Node

Standalone script for running a single robot controller.
Useful for testing individual robot operations.
"""

import sys
import signal
import argparse

import rclpy
from amr_fleet_adapter.robot_controller import TurtleBotController


def signal_handler(signum, frame):
    """Handle shutdown signals gracefully."""
    print("\nShutting down Robot Controller...")
    rclpy.shutdown()


def main(args=None):
    """Main entry point for the robot controller node."""
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='TurtleBot Robot Controller')
    parser.add_argument('robot_id', help='Robot ID (e.g., turtlebot1)')
    parser.add_argument('--node-name', help='Custom node name')
    
    if args is None:
        args = sys.argv[1:]
    
    parsed_args = parser.parse_args(args)
    robot_id = parsed_args.robot_id
    node_name = parsed_args.node_name
    
    # Set up signal handling
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        print(f"Starting Robot Controller for {robot_id}...")
        
        # Create the robot controller
        controller = TurtleBotController(robot_id, node_name)
        
        print(f"Robot Controller for {robot_id} started successfully!")
        print("Robot Status:")
        print(f"  - Robot ID: {robot_id}")
        print(f"  - Max Payload: {controller.capabilities['max_payload']}kg")
        print(f"  - Max Speed: {controller.capabilities['max_speed']}m/s")
        print(f"  - Supported Cargo: {', '.join(controller.capabilities['supported_cargo_types'])}")
        print("\nWaiting for navigation and delivery commands...")
        print("Press Ctrl+C to shutdown.")
        
        # Spin the node
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        print("\nReceived shutdown signal.")
    except Exception as e:
        print(f"Error in Robot Controller: {e}")
        sys.exit(1)
    finally:
        try:
            rclpy.shutdown()
        except:
            pass
        
        print(f"Robot Controller for {robot_id} shutdown complete.")


if __name__ == '__main__':
    main()