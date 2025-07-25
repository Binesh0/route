"""
Main AMR Fleet Adapter Module

Integrates RMF fleet management with delivery operations,
coordinating multiple TurtleBot robots for efficient delivery service.
"""

import json
import time
from typing import Dict, List, Optional, Any
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from rmf_fleet_msgs.msg import FleetState, RobotState as RMFRobotState
from rmf_task_msgs.msg import TaskSummary, TaskProfile

from .delivery_manager import (
    DeliveryManager, DeliveryTask, DeliveryStatus, 
    DeliveryPriority, CargoType, CargoInfo, DeliveryLocation
)
from .robot_controller import TurtleBotController, RobotStatus, RobotState
from .route_manager import RouteManager
from .utils import DeliveryUtils


class AMRFleetAdapter(Node):
    """
    Main AMR Fleet Adapter that coordinates delivery operations.
    
    Integrates RMF fleet management with delivery task management,
    robot control, and route planning for efficient multi-robot
    delivery operations.
    """
    
    def __init__(self, node_name: str = "amr_fleet_adapter"):
        super().__init__(node_name)
        
        self.get_logger().info("Initializing AMR Fleet Adapter...")
        
        # Configuration
        self.fleet_name = "delivery_fleet"
        self.robot_names = ["turtlebot1", "turtlebot2"]
        self.max_concurrent_deliveries = 10
        
        # Component managers
        self.delivery_manager: Optional[DeliveryManager] = None
        self.route_manager: Optional[RouteManager] = None
        self.robot_controllers: Dict[str, TurtleBotController] = {}
        
        # Fleet state tracking
        self.fleet_state = {
            'total_robots': 0,
            'active_robots': 0,
            'idle_robots': 0,
            'active_deliveries': 0,
            'completed_deliveries': 0,
            'failed_deliveries': 0,
            'fleet_efficiency': 0.0
        }
        
        # Load configuration and initialize components
        self._load_configuration()
        self._initialize_components()
        self._setup_ros_interfaces()
        
        self.get_logger().info("AMR Fleet Adapter initialized successfully")
    
    def _load_configuration(self):
        """Load configuration from ROS parameters."""
        # Declare parameters with defaults
        self.declare_parameter('fleet_name', 'delivery_fleet')
        self.declare_parameter('robot_names', ['turtlebot1', 'turtlebot2'])
        self.declare_parameter('max_concurrent_deliveries', 10)
        self.declare_parameter('route_json_file', '')
        self.declare_parameter('map_yaml_file', '')
        self.declare_parameter('enable_rmf_integration', True)
        self.declare_parameter('delivery_timeout_minutes', 120)
        self.declare_parameter('auto_assign_tasks', True)
        
        # Get parameter values
        self.fleet_name = self.get_parameter('fleet_name').value
        self.robot_names = self.get_parameter('robot_names').value
        self.max_concurrent_deliveries = self.get_parameter('max_concurrent_deliveries').value
        self.route_json_file = self.get_parameter('route_json_file').value
        self.map_yaml_file = self.get_parameter('map_yaml_file').value
        self.enable_rmf = self.get_parameter('enable_rmf_integration').value
        self.delivery_timeout = self.get_parameter('delivery_timeout_minutes').value
        self.auto_assign = self.get_parameter('auto_assign_tasks').value
        
        self.get_logger().info(f"Fleet configuration loaded: {self.fleet_name} with {len(self.robot_names)} robots")
    
    def _initialize_components(self):
        """Initialize all fleet adapter components."""
        try:
            # Initialize delivery manager
            self.delivery_manager = DeliveryManager("delivery_manager")
            
            # Initialize route manager
            self.route_manager = RouteManager(self)
            if self.route_json_file:
                self.route_manager.load_routes_from_json(self.route_json_file)
            if self.map_yaml_file:
                self.route_manager.load_map_info(self.map_yaml_file)
            
            # Initialize robot controllers
            for robot_name in self.robot_names:
                controller = TurtleBotController(robot_name)
                self.robot_controllers[robot_name] = controller
                
                # Register robot with delivery manager
                capabilities = controller.get_capabilities()
                self.delivery_manager.register_robot(robot_name, capabilities)
                
                # Setup callbacks
                self._setup_robot_callbacks(robot_name, controller)
            
            # Setup delivery manager callbacks
            self._setup_delivery_callbacks()
            
            self.fleet_state['total_robots'] = len(self.robot_names)
            
            self.get_logger().info("All components initialized successfully")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize components: {e}")
            raise
    
    def _setup_ros_interfaces(self):
        """Setup ROS2 publishers, subscribers, and services."""
        # Publishers
        self.fleet_state_pub = self.create_publisher(
            String, 'fleet_state', 10)
        self.delivery_metrics_pub = self.create_publisher(
            String, 'delivery_metrics', 10)
        
        # Subscribers
        self.delivery_request_sub = self.create_subscription(
            String, 'delivery_requests', self._delivery_request_callback, 10)
        self.fleet_command_sub = self.create_subscription(
            String, 'fleet_commands', self._fleet_command_callback, 10)
        
        # Timers
        self.state_publish_timer = self.create_timer(5.0, self._publish_fleet_state)
        self.metrics_timer = self.create_timer(30.0, self._publish_delivery_metrics)
        self.health_check_timer = self.create_timer(10.0, self._fleet_health_check)
    
    def _setup_robot_callbacks(self, robot_name: str, controller: TurtleBotController):
        """Setup callbacks for robot events."""
        controller.add_callback('on_navigation_complete', 
                              lambda data: self._on_robot_navigation_complete(robot_name, data))
        controller.add_callback('on_navigation_failed',
                              lambda data: self._on_robot_navigation_failed(robot_name, data))
        controller.add_callback('on_cargo_loaded',
                              lambda data: self._on_robot_cargo_loaded(robot_name, data))
        controller.add_callback('on_cargo_unloaded',
                              lambda data: self._on_robot_cargo_unloaded(robot_name, data))
        controller.add_callback('on_error',
                              lambda data: self._on_robot_error(robot_name, data))
    
    def _setup_delivery_callbacks(self):
        """Setup callbacks for delivery manager events."""
        self.delivery_manager.add_callback('on_task_created', self._on_delivery_task_created)
        self.delivery_manager.add_callback('on_task_assigned', self._on_delivery_task_assigned)
        self.delivery_manager.add_callback('on_task_completed', self._on_delivery_task_completed)
        self.delivery_manager.add_callback('on_task_failed', self._on_delivery_task_failed)
    
    # Public API methods
    def create_delivery_request(
        self,
        order_id: str,
        pickup_location: str,
        dropoff_location: str,
        cargo_info: Dict[str, Any],
        priority: str = "normal",
        customer_info: Optional[Dict[str, Any]] = None,
        special_requirements: Optional[Dict[str, Any]] = None
    ) -> Optional[str]:
        """
        Create a new delivery request.
        
        Args:
            order_id: Unique order identifier
            pickup_location: Pickup waypoint name
            dropoff_location: Dropoff waypoint name
            cargo_info: Cargo details (type, weight, dimensions, etc.)
            priority: Delivery priority (low, normal, high, urgent, emergency)
            customer_info: Customer contact information
            special_requirements: Special delivery requirements
            
        Returns:
            task_id: Unique task identifier if successful, None otherwise
        """
        try:
            # Get waypoint poses from route manager
            pickup_waypoint = self.route_manager.get_waypoint(pickup_location)
            dropoff_waypoint = self.route_manager.get_waypoint(dropoff_location)
            
            if not pickup_waypoint or not dropoff_waypoint:
                self.get_logger().error(f"Invalid waypoints: {pickup_location} or {dropoff_location}")
                return None
            
            # Create delivery locations
            pickup_loc = DeliveryLocation(
                location_id=f"pickup_{order_id}",
                name=pickup_location,
                waypoint_name=pickup_location,
                pose=self.route_manager.waypoint_to_pose(pickup_waypoint)
            )
            
            dropoff_loc = DeliveryLocation(
                location_id=f"dropoff_{order_id}",
                name=dropoff_location,
                waypoint_name=dropoff_location,
                pose=self.route_manager.waypoint_to_pose(dropoff_waypoint)
            )
            
            # Create cargo info
            cargo = CargoInfo(
                cargo_id=f"cargo_{order_id}",
                cargo_type=CargoType(cargo_info.get('type', 'package')),
                weight=cargo_info.get('weight', 1.0),
                dimensions=cargo_info.get('dimensions', {'length': 0.3, 'width': 0.3, 'height': 0.3}),
                fragile=cargo_info.get('fragile', False),
                temperature_sensitive=cargo_info.get('temperature_sensitive', False),
                special_instructions=cargo_info.get('special_instructions', ''),
                estimated_value=cargo_info.get('value', 0.0)
            )
            
            # Set priority
            priority_map = {
                'low': DeliveryPriority.LOW,
                'normal': DeliveryPriority.NORMAL,
                'high': DeliveryPriority.HIGH,
                'urgent': DeliveryPriority.URGENT,
                'emergency': DeliveryPriority.EMERGENCY
            }
            delivery_priority = priority_map.get(priority.lower(), DeliveryPriority.NORMAL)
            
            # Create delivery task
            task_id = self.delivery_manager.create_delivery_task(
                order_id=order_id,
                pickup_location=pickup_loc,
                dropoff_location=dropoff_loc,
                cargo=cargo,
                priority=delivery_priority,
                customer_info=customer_info,
                special_requirements=special_requirements
            )
            
            self.get_logger().info(f"Created delivery request {task_id} for order {order_id}")
            return task_id
            
        except Exception as e:
            self.get_logger().error(f"Failed to create delivery request: {e}")
            return None
    
    def get_delivery_status(self, task_id: str) -> Optional[Dict[str, Any]]:
        """Get status of a delivery task."""
        task = self.delivery_manager.get_task_status(task_id)
        if not task:
            return None
        
        return {
            'task_id': task.task_id,
            'order_id': task.order_id,
            'status': task.status.value,
            'progress': task.progress,
            'assigned_robot': task.assigned_robot,
            'pickup_location': task.pickup_location.name,
            'dropoff_location': task.dropoff_location.name,
            'created_time': task.created_time.isoformat(),
            'estimated_duration': task.estimated_duration,
            'tracking_updates': task.tracking_updates
        }
    
    def cancel_delivery(self, task_id: str, reason: str = "") -> bool:
        """Cancel a delivery task."""
        return self.delivery_manager.cancel_task(task_id, reason)
    
    def get_fleet_status(self) -> Dict[str, Any]:
        """Get current fleet status."""
        # Update fleet state
        active_robots = 0
        idle_robots = 0
        
        for robot_name, controller in self.robot_controllers.items():
            state = controller.get_state()
            if state.status in [RobotStatus.NAVIGATING, RobotStatus.LOADING, RobotStatus.UNLOADING]:
                active_robots += 1
            elif state.status == RobotStatus.IDLE:
                idle_robots += 1
        
        self.fleet_state.update({
            'active_robots': active_robots,
            'idle_robots': idle_robots,
            'active_deliveries': len(self.delivery_manager.get_active_tasks()),
            'timestamp': datetime.now().isoformat()
        })
        
        return self.fleet_state.copy()
    
    def get_robot_status(self, robot_name: str) -> Optional[Dict[str, Any]]:
        """Get status of a specific robot."""
        if robot_name not in self.robot_controllers:
            return None
        
        controller = self.robot_controllers[robot_name]
        state = controller.get_state()
        
        return {
            'robot_id': state.robot_id,
            'status': state.status.value,
            'navigation_status': state.navigation_status.value,
            'position': {
                'x': state.position.x,
                'y': state.position.y,
                'z': state.position.z
            },
            'battery_level': state.battery_level,
            'is_charging': state.is_charging,
            'cargo_loaded': state.cargo_loaded,
            'cargo_weight': state.cargo_weight,
            'current_task_id': state.current_task_id,
            'distance_to_goal': state.distance_to_goal,
            'estimated_time_to_goal': state.estimated_time_to_goal,
            'total_deliveries_completed': state.total_deliveries_completed,
            'error_message': state.error_message,
            'last_update': state.last_update.isoformat()
        }
    
    def emergency_stop_all_robots(self):
        """Emergency stop all robots in the fleet."""
        self.get_logger().warning("Emergency stop activated for all robots")
        
        for robot_name, controller in self.robot_controllers.items():
            controller.emergency_stop()
    
    def resume_all_robots(self):
        """Resume all robots from emergency stop."""
        self.get_logger().info("Resuming all robots from emergency stop")
        
        for robot_name, controller in self.robot_controllers.items():
            controller.resume_from_emergency_stop()
    
    def get_delivery_statistics(self) -> Dict[str, Any]:
        """Get comprehensive delivery statistics."""
        delivery_stats = self.delivery_manager.get_delivery_statistics()
        
        # Add fleet-specific statistics
        robot_stats = {}
        for robot_name, controller in self.robot_controllers.items():
            state = controller.get_state()
            robot_stats[robot_name] = {
                'total_deliveries': state.total_deliveries_completed,
                'total_distance': state.total_distance_traveled,
                'current_status': state.status.value,
                'battery_level': state.battery_level
            }
        
        return {
            'delivery_statistics': delivery_stats,
            'robot_statistics': robot_stats,
            'fleet_summary': self.get_fleet_status(),
            'generated_at': datetime.now().isoformat()
        }
    
    # Private callback methods
    def _on_delivery_task_created(self, task: DeliveryTask):
        """Handle delivery task creation."""
        self.get_logger().info(f"Delivery task created: {task.task_id}")
        
        # Auto-assign if enabled
        if self.auto_assign:
            # Find best robot for the task
            best_robot = self._find_best_robot_for_task(task)
            if best_robot:
                self.delivery_manager.assign_task_to_robot(task.task_id, best_robot)
    
    def _on_delivery_task_assigned(self, task: DeliveryTask):
        """Handle delivery task assignment."""
        self.get_logger().info(f"Delivery task {task.task_id} assigned to robot {task.assigned_robot}")
        
        # Start the delivery task on the robot
        if task.assigned_robot in self.robot_controllers:
            controller = self.robot_controllers[task.assigned_robot]
            controller.start_delivery_task(task)
    
    def _on_delivery_task_completed(self, task: DeliveryTask):
        """Handle delivery task completion."""
        self.get_logger().info(f"Delivery task completed: {task.task_id}")
        self.fleet_state['completed_deliveries'] += 1
    
    def _on_delivery_task_failed(self, task: DeliveryTask):
        """Handle delivery task failure."""
        self.get_logger().warning(f"Delivery task failed: {task.task_id}")
        self.fleet_state['failed_deliveries'] += 1
    
    def _on_robot_navigation_complete(self, robot_name: str, data: Dict[str, Any]):
        """Handle robot navigation completion."""
        self.get_logger().debug(f"Robot {robot_name} navigation completed")
    
    def _on_robot_navigation_failed(self, robot_name: str, data: Dict[str, Any]):
        """Handle robot navigation failure."""
        self.get_logger().warning(f"Robot {robot_name} navigation failed: {data}")
    
    def _on_robot_cargo_loaded(self, robot_name: str, data: Dict[str, Any]):
        """Handle robot cargo loading."""
        self.get_logger().info(f"Robot {robot_name} loaded cargo: {data['weight']}kg")
    
    def _on_robot_cargo_unloaded(self, robot_name: str, data: Dict[str, Any]):
        """Handle robot cargo unloading."""
        self.get_logger().info(f"Robot {robot_name} unloaded cargo: {data['weight']}kg")
    
    def _on_robot_error(self, robot_name: str, data: Dict[str, Any]):
        """Handle robot errors."""
        self.get_logger().error(f"Robot {robot_name} error: {data['message']}")
    
    # Private utility methods
    def _find_best_robot_for_task(self, task: DeliveryTask) -> Optional[str]:
        """Find the best available robot for a delivery task."""
        available_robots = []
        
        for robot_name, controller in self.robot_controllers.items():
            state = controller.get_state()
            capabilities = controller.get_capabilities()
            
            # Check if robot is available
            if state.status != RobotStatus.IDLE:
                continue
            
            # Check payload capacity
            if task.cargo.weight > capabilities.get('max_payload', 0):
                continue
            
            # Check cargo type support
            supported_types = capabilities.get('supported_cargo_types', [])
            if task.cargo.cargo_type.value not in supported_types:
                continue
            
            # Calculate distance to pickup location
            current_pose = controller.get_current_pose()
            if current_pose:
                pickup_pose = task.pickup_location.pose
                distance = DeliveryUtils.calculate_distance(current_pose, pickup_pose)
                available_robots.append((robot_name, distance))
        
        # Return closest available robot
        if available_robots:
            available_robots.sort(key=lambda x: x[1])  # Sort by distance
            return available_robots[0][0]
        
        return None
    
    def _fleet_health_check(self):
        """Perform fleet health checks."""
        # Check robot connectivity
        for robot_name, controller in self.robot_controllers.items():
            state = controller.get_state()
            
            # Check for stale data
            time_since_update = (datetime.now() - state.last_update).total_seconds()
            if time_since_update > 60.0:
                self.get_logger().warning(f"Robot {robot_name} data is stale ({time_since_update}s)")
            
            # Check battery levels
            if state.battery_level < 15.0:
                self.get_logger().warning(f"Robot {robot_name} low battery: {state.battery_level}%")
    
    def _publish_fleet_state(self):
        """Publish current fleet state."""
        fleet_status = self.get_fleet_status()
        
        state_msg = String()
        state_msg.data = json.dumps(fleet_status)
        self.fleet_state_pub.publish(state_msg)
    
    def _publish_delivery_metrics(self):
        """Publish delivery performance metrics."""
        metrics = self.get_delivery_statistics()
        
        metrics_msg = String()
        metrics_msg.data = json.dumps(metrics)
        self.delivery_metrics_pub.publish(metrics_msg)
    
    # ROS2 callback methods
    def _delivery_request_callback(self, msg: String):
        """Handle incoming delivery requests."""
        try:
            request_data = json.loads(msg.data)
            
            task_id = self.create_delivery_request(
                order_id=request_data['order_id'],
                pickup_location=request_data['pickup_location'],
                dropoff_location=request_data['dropoff_location'],
                cargo_info=request_data['cargo_info'],
                priority=request_data.get('priority', 'normal'),
                customer_info=request_data.get('customer_info'),
                special_requirements=request_data.get('special_requirements')
            )
            
            if task_id:
                self.get_logger().info(f"Processed delivery request: {task_id}")
            else:
                self.get_logger().error("Failed to process delivery request")
                
        except Exception as e:
            self.get_logger().error(f"Error processing delivery request: {e}")
    
    def _fleet_command_callback(self, msg: String):
        """Handle fleet commands."""
        try:
            command_data = json.loads(msg.data)
            command = command_data.get('command')
            
            if command == 'emergency_stop':
                self.emergency_stop_all_robots()
            elif command == 'resume':
                self.resume_all_robots()
            elif command == 'get_status':
                # Publish status immediately
                self._publish_fleet_state()
            elif command == 'cancel_task':
                task_id = command_data.get('task_id')
                reason = command_data.get('reason', 'Manual cancellation')
                if task_id:
                    self.cancel_delivery(task_id, reason)
            
        except Exception as e:
            self.get_logger().error(f"Error processing fleet command: {e}")


def main(args=None):
    """Main entry point for the AMR fleet adapter."""
    rclpy.init(args=args)
    
    try:
        # Create fleet adapter
        fleet_adapter = AMRFleetAdapter()
        
        # Use multi-threaded executor for better performance
        executor = MultiThreadedExecutor()
        executor.add_node(fleet_adapter)
        executor.add_node(fleet_adapter.delivery_manager)
        
        # Add robot controllers
        for controller in fleet_adapter.robot_controllers.values():
            executor.add_node(controller)
        
        # Spin
        executor.spin()
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in AMR Fleet Adapter: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()