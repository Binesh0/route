#!/usr/bin/env python3

"""
Delivery Client Example

Example script demonstrating how to send delivery requests
to the AMR Fleet Adapter and monitor delivery status.
"""

import json
import time
import sys
from typing import Dict, Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DeliveryClient(Node):
    """Example client for sending delivery requests."""
    
    def __init__(self):
        super().__init__('delivery_client_example')
        
        # Publishers
        self.delivery_request_pub = self.create_publisher(
            String, 'delivery_requests', 10)
        self.fleet_command_pub = self.create_publisher(
            String, 'fleet_commands', 10)
        
        # Subscribers
        self.fleet_state_sub = self.create_subscription(
            String, 'fleet_state', self._fleet_state_callback, 10)
        self.delivery_metrics_sub = self.create_subscription(
            String, 'delivery_metrics', self._delivery_metrics_callback, 10)
        
        # Wait for publishers to be ready
        time.sleep(1.0)
        
        self.get_logger().info("Delivery Client initialized")
    
    def send_delivery_request(
        self,
        order_id: str,
        pickup_location: str,
        dropoff_location: str,
        cargo_info: Dict[str, Any],
        priority: str = "normal",
        customer_info: Dict[str, Any] = None,
        special_requirements: Dict[str, Any] = None
    ):
        """Send a delivery request to the fleet adapter."""
        
        request_data = {
            'order_id': order_id,
            'pickup_location': pickup_location,
            'dropoff_location': dropoff_location,
            'cargo_info': cargo_info,
            'priority': priority,
            'customer_info': customer_info or {},
            'special_requirements': special_requirements or {}
        }
        
        msg = String()
        msg.data = json.dumps(request_data)
        
        self.delivery_request_pub.publish(msg)
        
        self.get_logger().info(f"Sent delivery request for order {order_id}")
        self.get_logger().info(f"  Pickup: {pickup_location}")
        self.get_logger().info(f"  Dropoff: {dropoff_location}")
        self.get_logger().info(f"  Cargo: {cargo_info['type']} ({cargo_info['weight']}kg)")
        self.get_logger().info(f"  Priority: {priority}")
    
    def send_fleet_command(self, command: str, **kwargs):
        """Send a command to the fleet."""
        
        command_data = {
            'command': command,
            **kwargs
        }
        
        msg = String()
        msg.data = json.dumps(command_data)
        
        self.fleet_command_pub.publish(msg)
        
        self.get_logger().info(f"Sent fleet command: {command}")
    
    def _fleet_state_callback(self, msg: String):
        """Handle fleet state updates."""
        try:
            state_data = json.loads(msg.data)
            
            self.get_logger().info("Fleet State Update:")
            self.get_logger().info(f"  Active Robots: {state_data.get('active_robots', 0)}")
            self.get_logger().info(f"  Idle Robots: {state_data.get('idle_robots', 0)}")
            self.get_logger().info(f"  Active Deliveries: {state_data.get('active_deliveries', 0)}")
            self.get_logger().info(f"  Completed Deliveries: {state_data.get('completed_deliveries', 0)}")
            
        except Exception as e:
            self.get_logger().error(f"Error processing fleet state: {e}")
    
    def _delivery_metrics_callback(self, msg: String):
        """Handle delivery metrics updates."""
        try:
            metrics_data = json.loads(msg.data)
            
            delivery_stats = metrics_data.get('delivery_statistics', {})
            
            self.get_logger().info("Delivery Metrics Update:")
            self.get_logger().info(f"  Success Rate: {delivery_stats.get('success_rate', 0):.1f}%")
            self.get_logger().info(f"  Average Delivery Time: {delivery_stats.get('average_delivery_time', 0):.1f} min")
            self.get_logger().info(f"  Total Tasks: {delivery_stats.get('total_tasks', 0)}")
            
        except Exception as e:
            self.get_logger().error(f"Error processing delivery metrics: {e}")


def main():
    """Main function with example usage."""
    
    rclpy.init()
    
    try:
        # Create client
        client = DeliveryClient()
        
        print("=== AMR Fleet Adapter - Delivery Client Example ===")
        print("This script demonstrates how to send delivery requests.")
        print("Make sure the AMR Fleet Adapter is running before using this client.")
        print()
        
        # Wait a moment for connections
        time.sleep(2.0)
        
        # Get fleet status
        print("Requesting fleet status...")
        client.send_fleet_command('get_status')
        
        # Wait for response
        rclpy.spin_once(client, timeout_sec=2.0)
        
        # Example 1: Simple package delivery
        print("\n--- Example 1: Package Delivery ---")
        client.send_delivery_request(
            order_id="PKG001",
            pickup_location="home",
            dropoff_location="station_a",
            cargo_info={
                'type': 'package',
                'weight': 2.5,
                'dimensions': {'length': 0.3, 'width': 0.2, 'height': 0.15},
                'fragile': False
            },
            priority="normal",
            customer_info={
                'name': 'John Doe',
                'phone': '+1-555-0123',
                'email': 'john.doe@example.com'
            }
        )
        
        # Wait and spin
        time.sleep(3.0)
        rclpy.spin_once(client, timeout_sec=1.0)
        
        # Example 2: Urgent medical delivery
        print("\n--- Example 2: Urgent Medical Delivery ---")
        client.send_delivery_request(
            order_id="MED001",
            pickup_location="station_b",
            dropoff_location="station_c",
            cargo_info={
                'type': 'medicine',
                'weight': 0.8,
                'dimensions': {'length': 0.15, 'width': 0.1, 'height': 0.05},
                'temperature_sensitive': True,
                'special_instructions': 'Keep refrigerated'
            },
            priority="urgent",
            customer_info={
                'name': 'Hospital Reception',
                'phone': '+1-555-0199',
                'emergency_contact': '+1-555-0911'
            },
            special_requirements={
                'signature': True,
                'photo_proof': True,
                'max_attempts': 1
            }
        )
        
        # Wait and spin
        time.sleep(3.0)
        rclpy.spin_once(client, timeout_sec=1.0)
        
        # Example 3: Fragile equipment delivery
        print("\n--- Example 3: Fragile Equipment Delivery ---")
        client.send_delivery_request(
            order_id="EQP001",
            pickup_location="station_a",
            dropoff_location="charging_dock",
            cargo_info={
                'type': 'equipment',
                'weight': 4.2,
                'dimensions': {'length': 0.4, 'width': 0.3, 'height': 0.25},
                'fragile': True,
                'estimated_value': 2500.0,
                'special_instructions': 'Handle with extreme care - precision instrument'
            },
            priority="high",
            customer_info={
                'name': 'Lab Manager',
                'phone': '+1-555-0156',
                'department': 'Research Lab'
            },
            special_requirements={
                'signature': True,
                'max_attempts': 2
            }
        )
        
        # Monitor for a while
        print("\n--- Monitoring Fleet Activity ---")
        print("Monitoring fleet for 30 seconds... (Press Ctrl+C to stop)")
        
        try:
            end_time = time.time() + 30.0
            while time.time() < end_time:
                rclpy.spin_once(client, timeout_sec=1.0)
                time.sleep(1.0)
        except KeyboardInterrupt:
            print("\nMonitoring stopped by user.")
        
        # Final fleet status
        print("\n--- Final Fleet Status ---")
        client.send_fleet_command('get_status')
        rclpy.spin_once(client, timeout_sec=2.0)
        
    except KeyboardInterrupt:
        print("\nClient stopped by user.")
    except Exception as e:
        print(f"Error in delivery client: {e}")
        sys.exit(1)
    finally:
        rclpy.shutdown()
        print("Delivery client shutdown complete.")


if __name__ == '__main__':
    main()