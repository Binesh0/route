#!/usr/bin/env python3

"""
Delivery Manager Node

Standalone script for running just the delivery manager component.
Useful for testing delivery operations independently.
"""

import sys
import signal

import rclpy
from amr_fleet_adapter.delivery_manager import DeliveryManager


def signal_handler(signum, frame):
    """Handle shutdown signals gracefully."""
    print("\nShutting down Delivery Manager...")
    rclpy.shutdown()


def main(args=None):
    """Main entry point for the delivery manager node."""
    
    # Set up signal handling
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Initialize ROS2
    rclpy.init(args=args)
    
    try:
        print("Starting Delivery Manager...")
        
        # Create the delivery manager
        delivery_manager = DeliveryManager()
        
        print("Delivery Manager started successfully!")
        print("Waiting for delivery requests...")
        print("Press Ctrl+C to shutdown.")
        
        # Spin the node
        rclpy.spin(delivery_manager)
        
    except KeyboardInterrupt:
        print("\nReceived shutdown signal.")
    except Exception as e:
        print(f"Error in Delivery Manager: {e}")
        sys.exit(1)
    finally:
        try:
            rclpy.shutdown()
        except:
            pass
        
        print("Delivery Manager shutdown complete.")


if __name__ == '__main__':
    main()