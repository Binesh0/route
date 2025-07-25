#!/usr/bin/env python3

"""
AMR Fleet Adapter Node

Main executable script for the AMR Fleet Adapter with delivery capabilities.
This script initializes and runs the complete fleet management system.
"""

import sys
import signal
from typing import Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor

from amr_fleet_adapter import AMRFleetAdapter


def signal_handler(signum, frame):
    """Handle shutdown signals gracefully."""
    print("\nShutting down AMR Fleet Adapter...")
    rclpy.shutdown()


def main(args=None):
    """Main entry point for the AMR Fleet Adapter node."""
    
    # Set up signal handling
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Initialize ROS2
    rclpy.init(args=args)
    
    fleet_adapter: Optional[AMRFleetAdapter] = None
    executor: Optional[MultiThreadedExecutor] = None
    
    try:
        print("Starting AMR Fleet Adapter...")
        
        # Create the fleet adapter
        fleet_adapter = AMRFleetAdapter()
        
        # Create multi-threaded executor for better performance
        executor = MultiThreadedExecutor(num_threads=4)
        
        # Add all nodes to executor
        executor.add_node(fleet_adapter)
        
        if fleet_adapter.delivery_manager:
            executor.add_node(fleet_adapter.delivery_manager)
        
        if fleet_adapter.route_manager:
            executor.add_node(fleet_adapter.route_manager)
        
        # Add robot controllers
        for robot_id, controller in fleet_adapter.robot_controllers.items():
            executor.add_node(controller)
            print(f"Added robot controller: {robot_id}")
        
        print("AMR Fleet Adapter started successfully!")
        print("Fleet Status:")
        print(f"  - Fleet Name: {fleet_adapter.fleet_name}")
        print(f"  - Total Robots: {len(fleet_adapter.robot_names)}")
        print(f"  - Robot Names: {', '.join(fleet_adapter.robot_names)}")
        print(f"  - Max Concurrent Deliveries: {fleet_adapter.max_concurrent_deliveries}")
        print("\nWaiting for delivery requests...")
        print("Press Ctrl+C to shutdown.")
        
        # Spin the executor
        executor.spin()
        
    except KeyboardInterrupt:
        print("\nReceived shutdown signal.")
    except Exception as e:
        print(f"Error in AMR Fleet Adapter: {e}")
        sys.exit(1)
    finally:
        # Cleanup
        print("Cleaning up...")
        
        if fleet_adapter:
            try:
                # Stop all robots
                fleet_adapter.emergency_stop_all_robots()
                
                # Get final statistics
                stats = fleet_adapter.get_delivery_statistics()
                print("\nFinal Statistics:")
                print(f"  - Total Tasks: {stats['delivery_statistics']['total_tasks']}")
                print(f"  - Completed: {stats['delivery_statistics']['completed_tasks']}")
                print(f"  - Success Rate: {stats['delivery_statistics']['success_rate']:.1f}%")
                
            except Exception as e:
                print(f"Error during cleanup: {e}")
        
        if executor:
            executor.shutdown()
        
        try:
            rclpy.shutdown()
        except:
            pass
        
        print("AMR Fleet Adapter shutdown complete.")


if __name__ == '__main__':
    main()