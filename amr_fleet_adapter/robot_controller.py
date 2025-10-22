"""
Robot Controller Module

Handles individual robot control and navigation for delivery operations.
Integrates with Nav2 for navigation and manages robot state for RMF.
"""

import json
import math
import time
from enum import Enum
from dataclasses import dataclass
from typing import Dict, List, Optional, Callable, Any, Tuple
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration

from std_msgs.msg import String, Header
from geometry_msgs.msg import (
    PoseStamped, Twist, Point, Quaternion, 
    PoseWithCovarianceStamped, TransformStamped
)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from nav2_msgs.action import NavigateToPose
from nav2_simple_commander import BasicNavigator
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_pose

from .delivery_manager import DeliveryStatus, DeliveryTask


class RobotStatus(Enum):
    """Robot operational status."""
    IDLE = "idle"
    NAVIGATING = "navigating"
    LOADING = "loading"
    UNLOADING = "unloading"
    CHARGING = "charging"
    ERROR = "error"
    EMERGENCY_STOP = "emergency_stop"
    MAINTENANCE = "maintenance"


class NavigationStatus(Enum):
    """Navigation task status."""
    IDLE = "idle"
    PLANNING = "planning"
    NAVIGATING = "navigating"
    ARRIVED = "arrived"
    FAILED = "failed"
    CANCELLED = "cancelled"


@dataclass
class RobotState:
    """Complete robot state information."""
    robot_id: str
    position: Point
    orientation: Quaternion
    linear_velocity: float
    angular_velocity: float
    battery_level: float
    is_charging: bool
    status: RobotStatus
    navigation_status: NavigationStatus
    current_task_id: Optional[str]
    cargo_loaded: bool
    cargo_weight: float
    last_update: datetime
    
    # Navigation specific
    current_goal: Optional[PoseStamped] = None
    distance_to_goal: float = 0.0
    estimated_time_to_goal: float = 0.0
    
    # Error tracking
    error_message: str = ""
    error_count: int = 0
    
    # Performance metrics
    total_distance_traveled: float = 0.0
    total_deliveries_completed: int = 0
    average_delivery_time: float = 0.0


class TurtleBotController(Node):
    """
    Controls individual TurtleBot robot for delivery operations.
    
    Integrates with Nav2 for navigation and manages robot state
    for RMF fleet coordination.
    """
    
    def __init__(self, robot_id: str, node_name: Optional[str] = None):
        if node_name is None:
            node_name = f"robot_controller_{robot_id}"
        
        super().__init__(node_name)
        
        self.robot_id = robot_id
        self.get_logger().info(f"Initializing TurtleBot Controller for {robot_id}")
        
        # Robot state
        self.state = RobotState(
            robot_id=robot_id,
            position=Point(),
            orientation=Quaternion(w=1.0),
            linear_velocity=0.0,
            angular_velocity=0.0,
            battery_level=100.0,
            is_charging=False,
            status=RobotStatus.IDLE,
            navigation_status=NavigationStatus.IDLE,
            current_task_id=None,
            cargo_loaded=False,
            cargo_weight=0.0,
            last_update=datetime.now()
        )
        
        # Robot capabilities
        self.capabilities = {
            'max_payload': 5.0,  # kg
            'max_speed': 0.5,    # m/s
            'supported_cargo_types': [
                'package', 'document', 'food', 'equipment'
            ],
            'has_camera': True,
            'has_gripper': False,
            'charging_time': 60.0  # minutes
        }
        
        # Navigation
        self.navigator = BasicNavigator()
        self.current_goal = None
        self.navigation_timeout = 300.0  # seconds
        
        # Task management
        self.current_delivery_task: Optional[DeliveryTask] = None
        self.task_callbacks: Dict[str, List[Callable]] = {
            'on_navigation_complete': [],
            'on_navigation_failed': [],
            'on_cargo_loaded': [],
            'on_cargo_unloaded': [],
            'on_error': []
        }
        
        # Configuration
        self.position_tolerance = 0.5  # meters
        self.orientation_tolerance = 0.2  # radians
        self.low_battery_threshold = 20.0  # percent
        self.critical_battery_threshold = 10.0  # percent
        
        # Setup ROS2 components
        self._setup_tf()
        self._setup_publishers_subscribers()
        self._setup_action_clients()
        self._setup_timers()
        
        # Initialize navigator
        self.navigator.waitUntilNav2Active()
        
        self.get_logger().info(f"TurtleBot Controller {robot_id} initialized successfully")
    
    def _setup_tf(self):
        """Set up TF2 components."""
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
    def _setup_publishers_subscribers(self):
        """Set up ROS2 publishers and subscribers."""
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist, f'{self.robot_id}/cmd_vel', 10)
        self.robot_state_pub = self.create_publisher(
            String, f'{self.robot_id}/robot_state', 10)
        self.delivery_status_pub = self.create_publisher(
            String, f'{self.robot_id}/delivery_status', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, f'{self.robot_id}/odom', self._odometry_callback, 10)
        self.battery_sub = self.create_subscription(
            BatteryState, f'{self.robot_id}/battery_state', self._battery_callback, 10)
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, f'{self.robot_id}/amcl_pose', 
            self._amcl_pose_callback, 10)
        
        # Command subscribers
        self.delivery_command_sub = self.create_subscription(
            String, f'{self.robot_id}/delivery_command', 
            self._delivery_command_callback, 10)
        self.navigation_command_sub = self.create_subscription(
            String, f'{self.robot_id}/navigation_command',
            self._navigation_command_callback, 10)
    
    def _setup_action_clients(self):
        """Set up action clients."""
        self.navigate_action_client = ActionClient(
            self, NavigateToPose, f'{self.robot_id}/navigate_to_pose')
    
    def _setup_timers(self):
        """Set up periodic timers."""
        # State update timer
        self.state_timer = self.create_timer(1.0, self._update_state)
        
        # Navigation monitoring timer
        self.nav_monitor_timer = self.create_timer(2.0, self._monitor_navigation)
        
        # Health check timer
        self.health_timer = self.create_timer(10.0, self._health_check)
    
    # Public API methods
    def navigate_to_pose(
        self, 
        goal_pose: PoseStamped, 
        timeout: Optional[float] = None
    ) -> bool:
        """
        Navigate to a specific pose.
        
        Args:
            goal_pose: Target pose to navigate to
            timeout: Navigation timeout in seconds
            
        Returns:
            bool: True if navigation command accepted
        """
        if self.state.status == RobotStatus.ERROR:
            self.get_logger().error("Cannot navigate: robot in error state")
            return False
        
        if self.state.status == RobotStatus.EMERGENCY_STOP:
            self.get_logger().error("Cannot navigate: emergency stop active")
            return False
        
        # Set navigation state
        self.state.navigation_status = NavigationStatus.PLANNING
        self.state.status = RobotStatus.NAVIGATING
        self.current_goal = goal_pose
        
        # Use Nav2 simple commander
        self.navigator.goToPose(goal_pose)
        
        # Update state
        self.state.current_goal = goal_pose
        self._calculate_goal_distance()
        
        self.get_logger().info(
            f"Robot {self.robot_id} navigating to [{goal_pose.pose.position.x:.2f}, "
            f"{goal_pose.pose.position.y:.2f}]")
        
        return True
    
    def navigate_to_waypoint(self, waypoint_name: str, waypoint_pose: PoseStamped) -> bool:
        """Navigate to a named waypoint."""
        self.get_logger().info(f"Robot {self.robot_id} navigating to waypoint: {waypoint_name}")
        return self.navigate_to_pose(waypoint_pose)
    
    def stop_navigation(self):
        """Stop current navigation."""
        self.navigator.cancelTask()
        
        # Send stop command
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        
        # Update state
        self.state.navigation_status = NavigationStatus.CANCELLED
        self.state.status = RobotStatus.IDLE
        self.current_goal = None
        
        self.get_logger().info(f"Robot {self.robot_id} navigation stopped")
    
    def emergency_stop(self):
        """Activate emergency stop."""
        self.stop_navigation()
        
        # Send zero velocity
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        
        # Update state
        self.state.status = RobotStatus.EMERGENCY_STOP
        self.state.error_message = "Emergency stop activated"
        
        self.get_logger().warning(f"Robot {self.robot_id} emergency stop activated")
        self._publish_robot_state()
    
    def resume_from_emergency_stop(self):
        """Resume from emergency stop."""
        if self.state.status == RobotStatus.EMERGENCY_STOP:
            self.state.status = RobotStatus.IDLE
            self.state.error_message = ""
            self.get_logger().info(f"Robot {self.robot_id} resumed from emergency stop")
            self._publish_robot_state()
    
    def load_cargo(self, cargo_weight: float = 0.0) -> bool:
        """Simulate cargo loading."""
        if self.state.cargo_loaded:
            self.get_logger().warning(f"Robot {self.robot_id} already has cargo loaded")
            return False
        
        if cargo_weight > self.capabilities['max_payload']:
            self.get_logger().error(
                f"Cargo weight {cargo_weight}kg exceeds max payload "
                f"{self.capabilities['max_payload']}kg")
            return False
        
        # Simulate loading time
        self.state.status = RobotStatus.LOADING
        self._publish_robot_state()
        
        # In real implementation, this would involve hardware control
        self.get_logger().info(f"Robot {self.robot_id} loading cargo ({cargo_weight}kg)")
        
        # Update state after loading
        self.state.cargo_loaded = True
        self.state.cargo_weight = cargo_weight
        self.state.status = RobotStatus.IDLE
        
        # Notify callbacks
        self._notify_callbacks('on_cargo_loaded', {'weight': cargo_weight})
        
        self.get_logger().info(f"Robot {self.robot_id} cargo loaded successfully")
        self._publish_robot_state()
        
        return True
    
    def unload_cargo(self) -> bool:
        """Simulate cargo unloading."""
        if not self.state.cargo_loaded:
            self.get_logger().warning(f"Robot {self.robot_id} has no cargo to unload")
            return False
        
        # Simulate unloading time
        self.state.status = RobotStatus.UNLOADING
        self._publish_robot_state()
        
        # In real implementation, this would involve hardware control
        self.get_logger().info(f"Robot {self.robot_id} unloading cargo")
        
        # Update state after unloading
        cargo_weight = self.state.cargo_weight
        self.state.cargo_loaded = False
        self.state.cargo_weight = 0.0
        self.state.status = RobotStatus.IDLE
        
        # Notify callbacks
        self._notify_callbacks('on_cargo_unloaded', {'weight': cargo_weight})
        
        self.get_logger().info(f"Robot {self.robot_id} cargo unloaded successfully")
        self._publish_robot_state()
        
        return True
    
    def start_delivery_task(self, task: DeliveryTask):
        """Start executing a delivery task."""
        if self.current_delivery_task:
            self.get_logger().warning(
                f"Robot {self.robot_id} already has active delivery task")
            return False
        
        self.current_delivery_task = task
        self.state.current_task_id = task.task_id
        
        self.get_logger().info(
            f"Robot {self.robot_id} starting delivery task {task.task_id}")
        
        # Start by navigating to pickup location
        self._execute_delivery_step("navigate_to_pickup")
        
        return True
    
    def get_current_pose(self) -> Optional[PoseStamped]:
        """Get current robot pose."""
        try:
            # Get transform from map to robot base
            transform = self.tf_buffer.lookup_transform(
                'map', f'{self.robot_id}/base_link', rclpy.time.Time())
            
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            
            return pose
            
        except Exception as e:
            self.get_logger().debug(f"Could not get current pose: {e}")
            return None
    
    def get_state(self) -> RobotState:
        """Get current robot state."""
        return self.state
    
    def get_capabilities(self) -> Dict[str, Any]:
        """Get robot capabilities."""
        return self.capabilities.copy()
    
    def add_callback(self, event: str, callback: Callable):
        """Add callback for robot events."""
        if event in self.task_callbacks:
            self.task_callbacks[event].append(callback)
    
    def is_at_position(self, target_pose: PoseStamped, tolerance: float = None) -> bool:
        """Check if robot is at target position."""
        if tolerance is None:
            tolerance = self.position_tolerance
        
        current_pose = self.get_current_pose()
        if not current_pose:
            return False
        
        # Calculate distance
        dx = current_pose.pose.position.x - target_pose.pose.position.x
        dy = current_pose.pose.position.y - target_pose.pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        return distance <= tolerance
    
    # Private methods
    def _execute_delivery_step(self, step: str):
        """Execute a step in the delivery process."""
        if not self.current_delivery_task:
            return
        
        task = self.current_delivery_task
        
        if step == "navigate_to_pickup":
            self._publish_delivery_status(DeliveryStatus.NAVIGATING_TO_PICKUP)
            success = self.navigate_to_pose(task.pickup_location.pose)
            if not success:
                self._handle_delivery_error("Failed to start navigation to pickup")
                
        elif step == "at_pickup":
            self._publish_delivery_status(DeliveryStatus.AT_PICKUP)
            # Simulate arrival processing time
            self.create_timer(2.0, lambda: self._execute_delivery_step("load_cargo"))
            
        elif step == "load_cargo":
            self._publish_delivery_status(DeliveryStatus.LOADING)
            success = self.load_cargo(task.cargo.weight)
            if success:
                self._execute_delivery_step("navigate_to_dropoff")
            else:
                self._handle_delivery_error("Failed to load cargo")
                
        elif step == "navigate_to_dropoff":
            self._publish_delivery_status(DeliveryStatus.NAVIGATING_TO_DROPOFF)
            success = self.navigate_to_pose(task.dropoff_location.pose)
            if not success:
                self._handle_delivery_error("Failed to start navigation to dropoff")
                
        elif step == "at_dropoff":
            self._publish_delivery_status(DeliveryStatus.AT_DROPOFF)
            # Simulate arrival processing time
            self.create_timer(2.0, lambda: self._execute_delivery_step("unload_cargo"))
            
        elif step == "unload_cargo":
            self._publish_delivery_status(DeliveryStatus.UNLOADING)
            success = self.unload_cargo()
            if success:
                self._execute_delivery_step("complete_delivery")
            else:
                self._handle_delivery_error("Failed to unload cargo")
                
        elif step == "complete_delivery":
            self._publish_delivery_status(DeliveryStatus.COMPLETED)
            self._complete_delivery_task()
    
    def _complete_delivery_task(self):
        """Complete the current delivery task."""
        if self.current_delivery_task:
            task = self.current_delivery_task
            
            # Update performance metrics
            self.state.total_deliveries_completed += 1
            
            # Reset task state
            self.current_delivery_task = None
            self.state.current_task_id = None
            self.state.status = RobotStatus.IDLE
            
            self.get_logger().info(
                f"Robot {self.robot_id} completed delivery task {task.task_id}")
            
            self._publish_robot_state()
    
    def _handle_delivery_error(self, error_message: str):
        """Handle delivery task error."""
        self.get_logger().error(
            f"Robot {self.robot_id} delivery error: {error_message}")
        
        self.state.error_message = error_message
        self.state.error_count += 1
        
        if self.current_delivery_task:
            self._publish_delivery_status(DeliveryStatus.FAILED)
            
        self._notify_callbacks('on_error', {'message': error_message})
        self._publish_robot_state()
    
    def _monitor_navigation(self):
        """Monitor navigation progress."""
        if self.state.navigation_status != NavigationStatus.NAVIGATING:
            return
        
        # Check navigation status
        if self.navigator.isTaskComplete():
            result = self.navigator.getResult()
            
            if result == NavigationStatus.SUCCEEDED:
                self.state.navigation_status = NavigationStatus.ARRIVED
                self.state.status = RobotStatus.IDLE
                self.current_goal = None
                
                self.get_logger().info(f"Robot {self.robot_id} navigation completed")
                self._notify_callbacks('on_navigation_complete', {})
                
                # Handle delivery step completion
                if self.current_delivery_task:
                    current_pose = self.get_current_pose()
                    pickup_pose = self.current_delivery_task.pickup_location.pose
                    dropoff_pose = self.current_delivery_task.dropoff_location.pose
                    
                    if current_pose and self.is_at_position(pickup_pose):
                        self._execute_delivery_step("at_pickup")
                    elif current_pose and self.is_at_position(dropoff_pose):
                        self._execute_delivery_step("at_dropoff")
                        
            else:
                self.state.navigation_status = NavigationStatus.FAILED
                self.state.status = RobotStatus.IDLE
                self.current_goal = None
                
                error_msg = f"Navigation failed with result: {result}"
                self.get_logger().error(f"Robot {self.robot_id} {error_msg}")
                self._notify_callbacks('on_navigation_failed', {'result': result})
                
                if self.current_delivery_task:
                    self._handle_delivery_error(error_msg)
    
    def _calculate_goal_distance(self):
        """Calculate distance to current goal."""
        if not self.current_goal:
            self.state.distance_to_goal = 0.0
            return
        
        current_pose = self.get_current_pose()
        if not current_pose:
            return
        
        dx = current_pose.pose.position.x - self.current_goal.pose.position.x
        dy = current_pose.pose.position.y - self.current_goal.pose.position.y
        self.state.distance_to_goal = math.sqrt(dx*dx + dy*dy)
        
        # Estimate time to goal (simple calculation)
        if self.state.linear_velocity > 0:
            self.state.estimated_time_to_goal = (
                self.state.distance_to_goal / self.state.linear_velocity)
        else:
            self.state.estimated_time_to_goal = 0.0
    
    def _health_check(self):
        """Perform robot health checks."""
        # Check battery level
        if self.state.battery_level <= self.critical_battery_threshold:
            if self.state.status != RobotStatus.CHARGING:
                self.get_logger().warning(
                    f"Robot {self.robot_id} critical battery level: "
                    f"{self.state.battery_level}%")
                # In real implementation, would initiate emergency charging
                
        elif self.state.battery_level <= self.low_battery_threshold:
            self.get_logger().info(
                f"Robot {self.robot_id} low battery level: {self.state.battery_level}%")
        
        # Check for stale data
        time_since_update = (datetime.now() - self.state.last_update).total_seconds()
        if time_since_update > 30.0:
            self.get_logger().warning(
                f"Robot {self.robot_id} state data is stale ({time_since_update}s)")
    
    def _update_state(self):
        """Update and publish robot state."""
        self.state.last_update = datetime.now()
        
        # Update navigation distance if navigating
        if self.state.navigation_status == NavigationStatus.NAVIGATING:
            self._calculate_goal_distance()
        
        self._publish_robot_state()
    
    def _publish_robot_state(self):
        """Publish current robot state."""
        state_msg = String()
        state_data = {
            'robot_id': self.state.robot_id,
            'position': {
                'x': self.state.position.x,
                'y': self.state.position.y,
                'z': self.state.position.z
            },
            'orientation': {
                'x': self.state.orientation.x,
                'y': self.state.orientation.y,
                'z': self.state.orientation.z,
                'w': self.state.orientation.w
            },
            'linear_velocity': self.state.linear_velocity,
            'angular_velocity': self.state.angular_velocity,
            'battery_level': self.state.battery_level,
            'is_charging': self.state.is_charging,
            'status': self.state.status.value,
            'navigation_status': self.state.navigation_status.value,
            'current_task_id': self.state.current_task_id,
            'cargo_loaded': self.state.cargo_loaded,
            'cargo_weight': self.state.cargo_weight,
            'distance_to_goal': self.state.distance_to_goal,
            'estimated_time_to_goal': self.state.estimated_time_to_goal,
            'error_message': self.state.error_message,
            'timestamp': self.state.last_update.isoformat()
        }
        
        state_msg.data = json.dumps(state_data)
        self.robot_state_pub.publish(state_msg)
    
    def _publish_delivery_status(self, status: DeliveryStatus):
        """Publish delivery status update."""
        if not self.current_delivery_task:
            return
        
        status_msg = String()
        status_data = {
            'robot_id': self.robot_id,
            'task_id': self.current_delivery_task.task_id,
            'status': status.value,
            'timestamp': datetime.now().isoformat()
        }
        
        status_msg.data = json.dumps(status_data)
        self.delivery_status_pub.publish(status_msg)
    
    def _notify_callbacks(self, event: str, data: Dict[str, Any]):
        """Notify registered callbacks."""
        for callback in self.task_callbacks.get(event, []):
            try:
                callback(data)
            except Exception as e:
                self.get_logger().error(f"Callback error: {e}")
    
    # ROS2 callback methods
    def _odometry_callback(self, msg: Odometry):
        """Handle odometry updates."""
        self.state.position = msg.pose.pose.position
        self.state.orientation = msg.pose.pose.orientation
        self.state.linear_velocity = msg.twist.twist.linear.x
        self.state.angular_velocity = msg.twist.twist.angular.z
        
        # Update total distance traveled
        if hasattr(self, '_last_position'):
            dx = self.state.position.x - self._last_position.x
            dy = self.state.position.y - self._last_position.y
            distance = math.sqrt(dx*dx + dy*dy)
            self.state.total_distance_traveled += distance
        
        self._last_position = Point(
            x=self.state.position.x,
            y=self.state.position.y,
            z=self.state.position.z
        )
    
    def _battery_callback(self, msg: BatteryState):
        """Handle battery state updates."""
        self.state.battery_level = msg.percentage * 100.0
        self.state.is_charging = (msg.power_supply_status == 
                                BatteryState.POWER_SUPPLY_STATUS_CHARGING)
    
    def _amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        """Handle AMCL pose updates (more accurate than odometry)."""
        self.state.position = msg.pose.pose.position
        self.state.orientation = msg.pose.pose.orientation
    
    def _delivery_command_callback(self, msg: String):
        """Handle delivery commands."""
        try:
            command_data = json.loads(msg.data)
            command = command_data.get('command')
            
            if command == 'load_cargo':
                weight = command_data.get('weight', 0.0)
                self.load_cargo(weight)
            elif command == 'unload_cargo':
                self.unload_cargo()
            elif command == 'emergency_stop':
                self.emergency_stop()
            elif command == 'resume':
                self.resume_from_emergency_stop()
                
        except Exception as e:
            self.get_logger().error(f"Error processing delivery command: {e}")
    
    def _navigation_command_callback(self, msg: String):
        """Handle navigation commands."""
        try:
            command_data = json.loads(msg.data)
            command = command_data.get('command')
            
            if command == 'navigate_to_pose':
                pose_data = command_data.get('pose')
                if pose_data:
                    goal_pose = PoseStamped()
                    goal_pose.header.frame_id = pose_data.get('frame_id', 'map')
                    goal_pose.header.stamp = self.get_clock().now().to_msg()
                    goal_pose.pose.position.x = pose_data['position']['x']
                    goal_pose.pose.position.y = pose_data['position']['y']
                    goal_pose.pose.orientation.w = pose_data.get('orientation', {}).get('w', 1.0)
                    
                    self.navigate_to_pose(goal_pose)
                    
            elif command == 'stop_navigation':
                self.stop_navigation()
                
        except Exception as e:
            self.get_logger().error(f"Error processing navigation command: {e}")