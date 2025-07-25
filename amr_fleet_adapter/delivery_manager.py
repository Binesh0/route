"""
Delivery Manager Module

Handles all delivery-related operations including:
- Delivery task creation and management
- Pickup and dropoff coordination
- Cargo tracking and validation
- Delivery status monitoring
- Route optimization for deliveries
"""

import json
import time
import uuid
from enum import Enum
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Callable, Any
from datetime import datetime, timedelta

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose
from rmf_task_msgs.msg import TaskProfile, TaskSummary, TaskType
from rmf_fleet_msgs.msg import FleetState, RobotState as RMFRobotState


class DeliveryStatus(Enum):
    """Enumeration of delivery task statuses."""
    PENDING = "pending"
    ASSIGNED = "assigned"
    NAVIGATING_TO_PICKUP = "navigating_to_pickup"
    AT_PICKUP = "at_pickup"
    LOADING = "loading"
    LOADED = "loaded"
    NAVIGATING_TO_DROPOFF = "navigating_to_dropoff"
    AT_DROPOFF = "at_dropoff"
    UNLOADING = "unloading"
    COMPLETED = "completed"
    CANCELLED = "cancelled"
    FAILED = "failed"


class DeliveryPriority(Enum):
    """Delivery task priority levels."""
    LOW = 1
    NORMAL = 2
    HIGH = 3
    URGENT = 4
    EMERGENCY = 5


class CargoType(Enum):
    """Types of cargo that can be delivered."""
    PACKAGE = "package"
    DOCUMENT = "document"
    FOOD = "food"
    MEDICINE = "medicine"
    EQUIPMENT = "equipment"
    FRAGILE = "fragile"
    HAZARDOUS = "hazardous"
    BULK = "bulk"


@dataclass
class CargoInfo:
    """Information about cargo being delivered."""
    cargo_id: str
    cargo_type: CargoType
    weight: float  # kg
    dimensions: Dict[str, float]  # length, width, height in meters
    fragile: bool = False
    temperature_sensitive: bool = False
    special_instructions: str = ""
    estimated_value: float = 0.0


@dataclass
class DeliveryLocation:
    """Delivery pickup or dropoff location."""
    location_id: str
    name: str
    waypoint_name: str
    pose: PoseStamped
    contact_info: Dict[str, str] = field(default_factory=dict)
    access_code: str = ""
    special_instructions: str = ""
    operating_hours: Dict[str, str] = field(default_factory=dict)


@dataclass
class DeliveryTask:
    """Complete delivery task definition."""
    task_id: str
    order_id: str
    priority: DeliveryPriority
    cargo: CargoInfo
    pickup_location: DeliveryLocation
    dropoff_location: DeliveryLocation
    
    # Task metadata
    created_time: datetime
    requested_pickup_time: Optional[datetime] = None
    requested_delivery_time: Optional[datetime] = None
    estimated_duration: float = 0.0  # minutes
    
    # Status tracking
    status: DeliveryStatus = DeliveryStatus.PENDING
    assigned_robot: Optional[str] = None
    progress: float = 0.0  # percentage
    
    # Timing
    assigned_time: Optional[datetime] = None
    pickup_time: Optional[datetime] = None
    delivery_time: Optional[datetime] = None
    
    # Customer information
    customer_info: Dict[str, Any] = field(default_factory=dict)
    
    # Special requirements
    requires_signature: bool = False
    requires_photo_proof: bool = False
    max_delivery_attempts: int = 3
    delivery_attempts: int = 0
    
    # Notes and tracking
    notes: List[str] = field(default_factory=list)
    tracking_updates: List[Dict[str, Any]] = field(default_factory=list)


class DeliveryManager(Node):
    """
    Manages all delivery operations for the AMR fleet.
    
    Handles task assignment, progress tracking, cargo management,
    and coordination with robot controllers.
    """
    
    def __init__(self, node_name: str = "delivery_manager"):
        super().__init__(node_name)
        
        self.get_logger().info("Initializing Delivery Manager...")
        
        # Task storage
        self.delivery_tasks: Dict[str, DeliveryTask] = {}
        self.active_tasks: Dict[str, str] = {}  # robot_id -> task_id
        self.task_queue: List[str] = []  # task_ids in priority order
        
        # Robot availability tracking
        self.available_robots: List[str] = []
        self.robot_capabilities: Dict[str, Dict[str, Any]] = {}
        
        # Configuration
        self.max_concurrent_deliveries = 10
        self.task_timeout_minutes = 120
        self.retry_failed_tasks = True
        
        # Callbacks
        self.task_callbacks: Dict[str, List[Callable]] = {
            'on_task_created': [],
            'on_task_assigned': [],
            'on_task_completed': [],
            'on_task_failed': []
        }
        
        # ROS2 setup
        self._setup_publishers_subscribers()
        self._setup_timers()
        
        self.get_logger().info("Delivery Manager initialized successfully")
    
    def _setup_publishers_subscribers(self):
        """Set up ROS2 publishers and subscribers."""
        # Publishers
        self.task_status_pub = self.create_publisher(
            String, 'delivery_task_status', 10)
        self.delivery_updates_pub = self.create_publisher(
            String, 'delivery_updates', 10)
        
        # Subscribers
        self.robot_state_sub = self.create_subscription(
            String, 'robot_states', self._robot_state_callback, 10)
        self.task_request_sub = self.create_subscription(
            String, 'delivery_requests', self._delivery_request_callback, 10)
    
    def _setup_timers(self):
        """Set up periodic timers."""
        # Task monitoring timer
        self.task_monitor_timer = self.create_timer(
            5.0, self._monitor_tasks)
        
        # Task assignment timer
        self.assignment_timer = self.create_timer(
            2.0, self._process_task_queue)
        
        # Cleanup timer
        self.cleanup_timer = self.create_timer(
            60.0, self._cleanup_completed_tasks)
    
    def create_delivery_task(
        self,
        order_id: str,
        pickup_location: DeliveryLocation,
        dropoff_location: DeliveryLocation,
        cargo: CargoInfo,
        priority: DeliveryPriority = DeliveryPriority.NORMAL,
        customer_info: Optional[Dict[str, Any]] = None,
        special_requirements: Optional[Dict[str, Any]] = None
    ) -> str:
        """
        Create a new delivery task.
        
        Args:
            order_id: Unique order identifier
            pickup_location: Pickup location details
            dropoff_location: Delivery location details
            cargo: Cargo information
            priority: Task priority level
            customer_info: Customer contact and preference information
            special_requirements: Special delivery requirements
            
        Returns:
            task_id: Unique task identifier
        """
        task_id = str(uuid.uuid4())
        
        # Create delivery task
        task = DeliveryTask(
            task_id=task_id,
            order_id=order_id,
            priority=priority,
            cargo=cargo,
            pickup_location=pickup_location,
            dropoff_location=dropoff_location,
            created_time=datetime.now(),
            customer_info=customer_info or {},
        )
        
        # Apply special requirements
        if special_requirements:
            task.requires_signature = special_requirements.get('signature', False)
            task.requires_photo_proof = special_requirements.get('photo_proof', False)
            task.max_delivery_attempts = special_requirements.get('max_attempts', 3)
            task.requested_pickup_time = special_requirements.get('pickup_time')
            task.requested_delivery_time = special_requirements.get('delivery_time')
        
        # Estimate duration
        task.estimated_duration = self._estimate_delivery_duration(task)
        
        # Store task
        self.delivery_tasks[task_id] = task
        
        # Add to queue
        self._add_to_queue(task_id)
        
        # Log creation
        self.get_logger().info(
            f"Created delivery task {task_id} for order {order_id}")
        self._add_tracking_update(task_id, "Task created", {
            'pickup': pickup_location.name,
            'dropoff': dropoff_location.name,
            'priority': priority.value
        })
        
        # Notify callbacks
        self._notify_callbacks('on_task_created', task)
        
        # Publish status
        self._publish_task_status(task_id)
        
        return task_id
    
    def assign_task_to_robot(self, task_id: str, robot_id: str) -> bool:
        """
        Assign a delivery task to a specific robot.
        
        Args:
            task_id: Task to assign
            robot_id: Robot to assign task to
            
        Returns:
            bool: True if assignment successful
        """
        if task_id not in self.delivery_tasks:
            self.get_logger().error(f"Task {task_id} not found")
            return False
        
        if robot_id not in self.available_robots:
            self.get_logger().error(f"Robot {robot_id} not available")
            return False
        
        if robot_id in self.active_tasks:
            self.get_logger().error(f"Robot {robot_id} already has active task")
            return False
        
        task = self.delivery_tasks[task_id]
        
        # Check robot capabilities
        if not self._check_robot_capability(robot_id, task):
            self.get_logger().error(
                f"Robot {robot_id} not capable of handling task {task_id}")
            return False
        
        # Assign task
        task.status = DeliveryStatus.ASSIGNED
        task.assigned_robot = robot_id
        task.assigned_time = datetime.now()
        task.progress = 10.0
        
        # Update tracking
        self.active_tasks[robot_id] = task_id
        if robot_id in self.available_robots:
            self.available_robots.remove(robot_id)
        
        # Remove from queue
        if task_id in self.task_queue:
            self.task_queue.remove(task_id)
        
        # Log assignment
        self.get_logger().info(
            f"Assigned task {task_id} to robot {robot_id}")
        self._add_tracking_update(task_id, "Task assigned", {
            'robot': robot_id,
            'estimated_duration': task.estimated_duration
        })
        
        # Notify callbacks
        self._notify_callbacks('on_task_assigned', task)
        
        # Publish updates
        self._publish_task_status(task_id)
        self._publish_delivery_update(task_id, "assigned")
        
        return True
    
    def update_task_status(
        self,
        task_id: str,
        status: DeliveryStatus,
        progress: Optional[float] = None,
        notes: Optional[str] = None
    ):
        """Update the status of a delivery task."""
        if task_id not in self.delivery_tasks:
            self.get_logger().error(f"Task {task_id} not found")
            return
        
        task = self.delivery_tasks[task_id]
        old_status = task.status
        
        # Update status
        task.status = status
        if progress is not None:
            task.progress = progress
        
        # Add notes
        if notes:
            task.notes.append(f"{datetime.now().isoformat()}: {notes}")
        
        # Handle status-specific updates
        if status == DeliveryStatus.AT_PICKUP:
            task.progress = 25.0
        elif status == DeliveryStatus.LOADED:
            task.pickup_time = datetime.now()
            task.progress = 50.0
        elif status == DeliveryStatus.AT_DROPOFF:
            task.progress = 75.0
        elif status == DeliveryStatus.COMPLETED:
            task.delivery_time = datetime.now()
            task.progress = 100.0
            self._complete_task(task_id)
        elif status in [DeliveryStatus.FAILED, DeliveryStatus.CANCELLED]:
            self._handle_failed_task(task_id, status)
        
        # Log status change
        self.get_logger().info(
            f"Task {task_id} status: {old_status.value} -> {status.value}")
        self._add_tracking_update(task_id, f"Status changed to {status.value}", {
            'progress': task.progress,
            'notes': notes
        })
        
        # Publish updates
        self._publish_task_status(task_id)
        self._publish_delivery_update(task_id, status.value)
    
    def get_task_status(self, task_id: str) -> Optional[DeliveryTask]:
        """Get current status of a delivery task."""
        return self.delivery_tasks.get(task_id)
    
    def get_active_tasks(self) -> Dict[str, DeliveryTask]:
        """Get all currently active tasks."""
        return {
            task_id: task for task_id, task in self.delivery_tasks.items()
            if task.status not in [DeliveryStatus.COMPLETED, 
                                 DeliveryStatus.CANCELLED, 
                                 DeliveryStatus.FAILED]
        }
    
    def get_robot_current_task(self, robot_id: str) -> Optional[str]:
        """Get current task assigned to a robot."""
        return self.active_tasks.get(robot_id)
    
    def cancel_task(self, task_id: str, reason: str = "") -> bool:
        """Cancel a delivery task."""
        if task_id not in self.delivery_tasks:
            return False
        
        task = self.delivery_tasks[task_id]
        
        if task.status in [DeliveryStatus.COMPLETED, DeliveryStatus.CANCELLED]:
            return False
        
        # Update status
        task.status = DeliveryStatus.CANCELLED
        if reason:
            task.notes.append(f"{datetime.now().isoformat()}: Cancelled - {reason}")
        
        # Free up robot
        if task.assigned_robot and task.assigned_robot in self.active_tasks:
            del self.active_tasks[task.assigned_robot]
            if task.assigned_robot not in self.available_robots:
                self.available_robots.append(task.assigned_robot)
        
        # Log cancellation
        self.get_logger().info(f"Cancelled task {task_id}: {reason}")
        self._add_tracking_update(task_id, "Task cancelled", {'reason': reason})
        
        # Publish updates
        self._publish_task_status(task_id)
        self._publish_delivery_update(task_id, "cancelled")
        
        return True
    
    def add_callback(self, event: str, callback: Callable):
        """Add callback for delivery events."""
        if event in self.task_callbacks:
            self.task_callbacks[event].append(callback)
    
    def register_robot(self, robot_id: str, capabilities: Dict[str, Any]):
        """Register a robot with the delivery manager."""
        self.robot_capabilities[robot_id] = capabilities
        if robot_id not in self.available_robots:
            self.available_robots.append(robot_id)
        
        self.get_logger().info(f"Registered robot {robot_id} for deliveries")
    
    def unregister_robot(self, robot_id: str):
        """Unregister a robot from delivery service."""
        # Cancel any active tasks
        if robot_id in self.active_tasks:
            task_id = self.active_tasks[robot_id]
            self.cancel_task(task_id, f"Robot {robot_id} unregistered")
        
        # Remove from tracking
        if robot_id in self.available_robots:
            self.available_robots.remove(robot_id)
        if robot_id in self.robot_capabilities:
            del self.robot_capabilities[robot_id]
        
        self.get_logger().info(f"Unregistered robot {robot_id}")
    
    def get_delivery_statistics(self) -> Dict[str, Any]:
        """Get delivery performance statistics."""
        completed_tasks = [
            task for task in self.delivery_tasks.values()
            if task.status == DeliveryStatus.COMPLETED
        ]
        
        failed_tasks = [
            task for task in self.delivery_tasks.values()
            if task.status == DeliveryStatus.FAILED
        ]
        
        stats = {
            'total_tasks': len(self.delivery_tasks),
            'completed_tasks': len(completed_tasks),
            'failed_tasks': len(failed_tasks),
            'active_tasks': len(self.get_active_tasks()),
            'pending_tasks': len(self.task_queue),
            'success_rate': len(completed_tasks) / max(1, len(self.delivery_tasks)) * 100,
            'average_delivery_time': 0.0,
            'active_robots': len([r for r in self.active_tasks.keys()]),
            'available_robots': len(self.available_robots)
        }
        
        # Calculate average delivery time
        if completed_tasks:
            total_time = sum([
                (task.delivery_time - task.created_time).total_seconds() / 60
                for task in completed_tasks
                if task.delivery_time and task.created_time
            ])
            stats['average_delivery_time'] = total_time / len(completed_tasks)
        
        return stats
    
    # Private methods
    def _add_to_queue(self, task_id: str):
        """Add task to priority queue."""
        task = self.delivery_tasks[task_id]
        
        # Insert based on priority
        inserted = False
        for i, existing_task_id in enumerate(self.task_queue):
            existing_task = self.delivery_tasks[existing_task_id]
            if task.priority.value > existing_task.priority.value:
                self.task_queue.insert(i, task_id)
                inserted = True
                break
        
        if not inserted:
            self.task_queue.append(task_id)
    
    def _estimate_delivery_duration(self, task: DeliveryTask) -> float:
        """Estimate delivery duration in minutes."""
        # Simple estimation based on distance and cargo type
        base_time = 30.0  # minutes
        
        # Add time based on cargo type
        cargo_time_multipliers = {
            CargoType.PACKAGE: 1.0,
            CargoType.DOCUMENT: 0.8,
            CargoType.FOOD: 1.2,
            CargoType.MEDICINE: 1.5,
            CargoType.EQUIPMENT: 1.8,
            CargoType.FRAGILE: 1.4,
            CargoType.HAZARDOUS: 2.0,
            CargoType.BULK: 2.5
        }
        
        multiplier = cargo_time_multipliers.get(task.cargo.cargo_type, 1.0)
        
        # Add time for special requirements
        if task.requires_signature:
            base_time += 5.0
        if task.requires_photo_proof:
            base_time += 3.0
        
        return base_time * multiplier
    
    def _check_robot_capability(self, robot_id: str, task: DeliveryTask) -> bool:
        """Check if robot can handle the delivery task."""
        if robot_id not in self.robot_capabilities:
            return False
        
        capabilities = self.robot_capabilities[robot_id]
        
        # Check payload capacity
        if task.cargo.weight > capabilities.get('max_payload', 0):
            return False
        
        # Check cargo type compatibility
        supported_cargo = capabilities.get('supported_cargo_types', [])
        if task.cargo.cargo_type.value not in supported_cargo:
            return False
        
        return True
    
    def _process_task_queue(self):
        """Process pending tasks and assign to available robots."""
        if not self.task_queue or not self.available_robots:
            return
        
        # Process highest priority tasks first
        for task_id in self.task_queue.copy():
            if not self.available_robots:
                break
            
            task = self.delivery_tasks[task_id]
            
            # Find suitable robot
            suitable_robot = None
            for robot_id in self.available_robots:
                if self._check_robot_capability(robot_id, task):
                    suitable_robot = robot_id
                    break
            
            if suitable_robot:
                self.assign_task_to_robot(task_id, suitable_robot)
    
    def _monitor_tasks(self):
        """Monitor active tasks for timeouts and issues."""
        current_time = datetime.now()
        
        for task_id, task in self.delivery_tasks.items():
            if task.status in [DeliveryStatus.COMPLETED, 
                             DeliveryStatus.CANCELLED, 
                             DeliveryStatus.FAILED]:
                continue
            
            # Check for timeouts
            if task.assigned_time:
                elapsed_minutes = (current_time - task.assigned_time).total_seconds() / 60
                if elapsed_minutes > self.task_timeout_minutes:
                    self.get_logger().warning(f"Task {task_id} timed out")
                    self.update_task_status(task_id, DeliveryStatus.FAILED, 
                                          notes="Task timeout")
    
    def _complete_task(self, task_id: str):
        """Handle task completion."""
        task = self.delivery_tasks[task_id]
        
        # Free up robot
        if task.assigned_robot and task.assigned_robot in self.active_tasks:
            del self.active_tasks[task.assigned_robot]
            if task.assigned_robot not in self.available_robots:
                self.available_robots.append(task.assigned_robot)
        
        # Calculate actual duration
        if task.assigned_time and task.delivery_time:
            actual_duration = (task.delivery_time - task.assigned_time).total_seconds() / 60
            task.notes.append(f"Actual duration: {actual_duration:.1f} minutes")
        
        # Log completion
        self.get_logger().info(f"Completed delivery task {task_id}")
        self._add_tracking_update(task_id, "Task completed", {
            'delivery_time': task.delivery_time.isoformat() if task.delivery_time else None
        })
        
        # Notify callbacks
        self._notify_callbacks('on_task_completed', task)
    
    def _handle_failed_task(self, task_id: str, status: DeliveryStatus):
        """Handle failed or cancelled tasks."""
        task = self.delivery_tasks[task_id]
        
        # Free up robot
        if task.assigned_robot and task.assigned_robot in self.active_tasks:
            del self.active_tasks[task.assigned_robot]
            if task.assigned_robot not in self.available_robots:
                self.available_robots.append(task.assigned_robot)
        
        # Handle retry logic
        if (status == DeliveryStatus.FAILED and 
            self.retry_failed_tasks and 
            task.delivery_attempts < task.max_delivery_attempts):
            
            task.delivery_attempts += 1
            task.status = DeliveryStatus.PENDING
            task.assigned_robot = None
            task.assigned_time = None
            
            self._add_to_queue(task_id)
            
            self.get_logger().info(
                f"Retrying failed task {task_id} (attempt {task.delivery_attempts})")
        else:
            # Notify callbacks
            self._notify_callbacks('on_task_failed', task)
    
    def _cleanup_completed_tasks(self):
        """Clean up old completed tasks."""
        cutoff_time = datetime.now() - timedelta(days=7)
        
        tasks_to_remove = []
        for task_id, task in self.delivery_tasks.items():
            if (task.status in [DeliveryStatus.COMPLETED, 
                              DeliveryStatus.CANCELLED, 
                              DeliveryStatus.FAILED] and
                task.created_time < cutoff_time):
                tasks_to_remove.append(task_id)
        
        for task_id in tasks_to_remove:
            del self.delivery_tasks[task_id]
        
        if tasks_to_remove:
            self.get_logger().info(f"Cleaned up {len(tasks_to_remove)} old tasks")
    
    def _add_tracking_update(self, task_id: str, message: str, data: Dict[str, Any]):
        """Add tracking update to task."""
        if task_id in self.delivery_tasks:
            update = {
                'timestamp': datetime.now().isoformat(),
                'message': message,
                'data': data
            }
            self.delivery_tasks[task_id].tracking_updates.append(update)
    
    def _notify_callbacks(self, event: str, task: DeliveryTask):
        """Notify registered callbacks."""
        for callback in self.task_callbacks.get(event, []):
            try:
                callback(task)
            except Exception as e:
                self.get_logger().error(f"Callback error: {e}")
    
    def _publish_task_status(self, task_id: str):
        """Publish task status update."""
        if task_id in self.delivery_tasks:
            task = self.delivery_tasks[task_id]
            status_msg = String()
            status_msg.data = json.dumps({
                'task_id': task_id,
                'status': task.status.value,
                'progress': task.progress,
                'assigned_robot': task.assigned_robot
            })
            self.task_status_pub.publish(status_msg)
    
    def _publish_delivery_update(self, task_id: str, event: str):
        """Publish delivery update."""
        update_msg = String()
        update_msg.data = json.dumps({
            'task_id': task_id,
            'event': event,
            'timestamp': datetime.now().isoformat()
        })
        self.delivery_updates_pub.publish(update_msg)
    
    def _robot_state_callback(self, msg):
        """Handle robot state updates."""
        # Implementation depends on robot state message format
        pass
    
    def _delivery_request_callback(self, msg):
        """Handle incoming delivery requests."""
        try:
            request_data = json.loads(msg.data)
            # Process delivery request
            # Implementation depends on request format
        except Exception as e:
            self.get_logger().error(f"Error processing delivery request: {e}")