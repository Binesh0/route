"""
Utility Functions for AMR Fleet Adapter

Common utility functions for delivery operations,
navigation calculations, and data processing.
"""

import math
import json
from typing import Dict, List, Optional, Any, Tuple
from datetime import datetime, timedelta

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Path
from std_msgs.msg import Header


class DeliveryUtils:
    """Utility functions for delivery operations."""
    
    @staticmethod
    def calculate_distance(pose1: PoseStamped, pose2: PoseStamped) -> float:
        """Calculate Euclidean distance between two poses."""
        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y
        dz = pose1.pose.position.z - pose2.pose.position.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    @staticmethod
    def calculate_point_distance(p1: Point, p2: Point) -> float:
        """Calculate Euclidean distance between two points."""
        dx = p1.x - p2.x
        dy = p1.y - p2.y
        dz = p1.z - p2.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    @staticmethod
    def estimate_travel_time(
        distance: float, 
        average_speed: float = 0.5
    ) -> float:
        """
        Estimate travel time based on distance and speed.
        
        Args:
            distance: Distance in meters
            average_speed: Average speed in m/s
            
        Returns:
            Travel time in seconds
        """
        if average_speed <= 0:
            return 0.0
        return distance / average_speed
    
    @staticmethod
    def estimate_delivery_duration(
        pickup_pose: PoseStamped,
        dropoff_pose: PoseStamped,
        robot_pose: PoseStamped,
        cargo_type: str = "package",
        loading_time: float = 30.0,
        unloading_time: float = 30.0,
        average_speed: float = 0.5
    ) -> float:
        """
        Estimate total delivery duration.
        
        Args:
            pickup_pose: Pickup location
            dropoff_pose: Dropoff location  
            robot_pose: Current robot position
            cargo_type: Type of cargo being delivered
            loading_time: Time to load cargo (seconds)
            unloading_time: Time to unload cargo (seconds)
            average_speed: Average robot speed (m/s)
            
        Returns:
            Estimated duration in seconds
        """
        # Calculate distances
        to_pickup_dist = DeliveryUtils.calculate_distance(robot_pose, pickup_pose)
        pickup_to_dropoff_dist = DeliveryUtils.calculate_distance(pickup_pose, dropoff_pose)
        
        # Calculate travel times
        to_pickup_time = DeliveryUtils.estimate_travel_time(to_pickup_dist, average_speed)
        delivery_time = DeliveryUtils.estimate_travel_time(pickup_to_dropoff_dist, average_speed)
        
        # Add cargo-specific multipliers
        cargo_multipliers = {
            'package': 1.0,
            'document': 0.8,
            'food': 1.2,
            'medicine': 1.3,
            'equipment': 1.5,
            'fragile': 1.4,
            'hazardous': 1.8,
            'bulk': 2.0
        }
        
        multiplier = cargo_multipliers.get(cargo_type, 1.0)
        
        total_time = (to_pickup_time + loading_time + delivery_time + unloading_time) * multiplier
        return total_time
    
    @staticmethod
    def validate_cargo_constraints(
        cargo_weight: float,
        cargo_dimensions: Dict[str, float],
        robot_capacity: float,
        robot_dimensions: Dict[str, float]
    ) -> Tuple[bool, str]:
        """
        Validate if cargo fits robot constraints.
        
        Args:
            cargo_weight: Weight of cargo in kg
            cargo_dimensions: Cargo dimensions (length, width, height)
            robot_capacity: Robot payload capacity in kg
            robot_dimensions: Robot cargo space dimensions
            
        Returns:
            (is_valid, error_message)
        """
        # Check weight constraint
        if cargo_weight > robot_capacity:
            return False, f"Cargo weight {cargo_weight}kg exceeds capacity {robot_capacity}kg"
        
        # Check dimension constraints
        for dimension in ['length', 'width', 'height']:
            cargo_size = cargo_dimensions.get(dimension, 0)
            robot_size = robot_dimensions.get(dimension, 0)
            
            if cargo_size > robot_size:
                return False, f"Cargo {dimension} {cargo_size}m exceeds limit {robot_size}m"
        
        return True, ""
    
    @staticmethod
    def format_tracking_update(
        timestamp: datetime,
        status: str,
        location: str = "",
        details: Dict[str, Any] = None
    ) -> Dict[str, Any]:
        """Format a tracking update entry."""
        update = {
            'timestamp': timestamp.isoformat(),
            'status': status,
            'location': location
        }
        
        if details:
            update['details'] = details
        
        return update
    
    @staticmethod
    def calculate_delivery_efficiency(
        planned_duration: float,
        actual_duration: float
    ) -> float:
        """
        Calculate delivery efficiency percentage.
        
        Args:
            planned_duration: Planned delivery time
            actual_duration: Actual delivery time
            
        Returns:
            Efficiency percentage (100% = on time, >100% = late, <100% = early)
        """
        if planned_duration <= 0:
            return 0.0
        
        efficiency = (actual_duration / planned_duration) * 100.0
        return min(efficiency, 200.0)  # Cap at 200% for very late deliveries


class NavigationUtils:
    """Utility functions for navigation operations."""
    
    @staticmethod
    def quaternion_to_yaw(quat: Quaternion) -> float:
        """Convert quaternion to yaw angle."""
        # Extract yaw from quaternion
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    @staticmethod
    def yaw_to_quaternion(yaw: float) -> Quaternion:
        """Convert yaw angle to quaternion."""
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(yaw / 2.0)
        quat.w = math.cos(yaw / 2.0)
        return quat
    
    @staticmethod
    def create_pose_stamped(
        x: float,
        y: float,
        yaw: float = 0.0,
        frame_id: str = "map"
    ) -> PoseStamped:
        """Create a PoseStamped message."""
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = Header().stamp  # Will be set by caller
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        pose.pose.orientation = NavigationUtils.yaw_to_quaternion(yaw)
        
        return pose
    
    @staticmethod
    def calculate_angular_difference(angle1: float, angle2: float) -> float:
        """Calculate the angular difference between two angles."""
        diff = angle2 - angle1
        
        # Normalize to [-pi, pi]
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        
        return diff
    
    @staticmethod
    def is_pose_reached(
        current_pose: PoseStamped,
        target_pose: PoseStamped,
        position_tolerance: float = 0.5,
        orientation_tolerance: float = 0.2
    ) -> bool:
        """
        Check if robot has reached target pose within tolerance.
        
        Args:
            current_pose: Current robot pose
            target_pose: Target pose
            position_tolerance: Position tolerance in meters
            orientation_tolerance: Orientation tolerance in radians
            
        Returns:
            True if pose is reached within tolerance
        """
        # Check position
        position_error = DeliveryUtils.calculate_distance(current_pose, target_pose)
        if position_error > position_tolerance:
            return False
        
        # Check orientation
        current_yaw = NavigationUtils.quaternion_to_yaw(current_pose.pose.orientation)
        target_yaw = NavigationUtils.quaternion_to_yaw(target_pose.pose.orientation)
        
        orientation_error = abs(NavigationUtils.calculate_angular_difference(current_yaw, target_yaw))
        if orientation_error > orientation_tolerance:
            return False
        
        return True
    
    @staticmethod
    def create_path_from_waypoints(
        waypoints: List[PoseStamped],
        frame_id: str = "map"
    ) -> Path:
        """Create a Path message from list of waypoints."""
        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp = Header().stamp  # Will be set by caller
        
        path.poses = waypoints
        
        return path
    
    @staticmethod
    def interpolate_path(
        start_pose: PoseStamped,
        end_pose: PoseStamped,
        num_points: int = 10
    ) -> List[PoseStamped]:
        """
        Interpolate a straight-line path between two poses.
        
        Args:
            start_pose: Starting pose
            end_pose: Ending pose
            num_points: Number of intermediate points
            
        Returns:
            List of interpolated poses
        """
        if num_points < 2:
            return [start_pose, end_pose]
        
        poses = []
        
        for i in range(num_points):
            t = i / (num_points - 1)  # Parameter from 0 to 1
            
            # Interpolate position
            x = start_pose.pose.position.x + t * (end_pose.pose.position.x - start_pose.pose.position.x)
            y = start_pose.pose.position.y + t * (end_pose.pose.position.y - start_pose.pose.position.y)
            z = start_pose.pose.position.z + t * (end_pose.pose.position.z - start_pose.pose.position.z)
            
            # Interpolate orientation (SLERP would be better, but this is simpler)
            start_yaw = NavigationUtils.quaternion_to_yaw(start_pose.pose.orientation)
            end_yaw = NavigationUtils.quaternion_to_yaw(end_pose.pose.orientation)
            
            # Handle angle wrapping
            angle_diff = NavigationUtils.calculate_angular_difference(start_yaw, end_yaw)
            yaw = start_yaw + t * angle_diff
            
            # Create interpolated pose
            pose = NavigationUtils.create_pose_stamped(x, y, yaw, start_pose.header.frame_id)
            poses.append(pose)
        
        return poses


class DataUtils:
    """Utility functions for data processing and validation."""
    
    @staticmethod
    def validate_json_structure(
        data: Dict[str, Any],
        required_fields: List[str]
    ) -> Tuple[bool, str]:
        """
        Validate JSON data structure.
        
        Args:
            data: JSON data to validate
            required_fields: List of required field names
            
        Returns:
            (is_valid, error_message)
        """
        for field in required_fields:
            if field not in data:
                return False, f"Missing required field: {field}"
        
        return True, ""
    
    @staticmethod
    def safe_float_convert(
        value: Any,
        default: float = 0.0,
        min_val: Optional[float] = None,
        max_val: Optional[float] = None
    ) -> float:
        """Safely convert value to float with bounds checking."""
        try:
            result = float(value)
            
            if min_val is not None and result < min_val:
                result = min_val
            if max_val is not None and result > max_val:
                result = max_val
            
            return result
        except (ValueError, TypeError):
            return default
    
    @staticmethod
    def safe_int_convert(
        value: Any,
        default: int = 0,
        min_val: Optional[int] = None,
        max_val: Optional[int] = None
    ) -> int:
        """Safely convert value to int with bounds checking."""
        try:
            result = int(value)
            
            if min_val is not None and result < min_val:
                result = min_val
            if max_val is not None and result > max_val:
                result = max_val
            
            return result
        except (ValueError, TypeError):
            return default
    
    @staticmethod
    def format_duration(seconds: float) -> str:
        """Format duration in seconds to human-readable string."""
        if seconds < 60:
            return f"{seconds:.1f}s"
        elif seconds < 3600:
            minutes = seconds / 60
            return f"{minutes:.1f}m"
        else:
            hours = seconds / 3600
            return f"{hours:.1f}h"
    
    @staticmethod
    def format_distance(meters: float) -> str:
        """Format distance in meters to human-readable string."""
        if meters < 1000:
            return f"{meters:.1f}m"
        else:
            kilometers = meters / 1000
            return f"{kilometers:.1f}km"
    
    @staticmethod
    def create_error_response(
        error_code: str,
        error_message: str,
        details: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """Create standardized error response."""
        response = {
            'success': False,
            'error_code': error_code,
            'error_message': error_message,
            'timestamp': datetime.now().isoformat()
        }
        
        if details:
            response['details'] = details
        
        return response
    
    @staticmethod
    def create_success_response(
        data: Optional[Dict[str, Any]] = None,
        message: str = "Operation completed successfully"
    ) -> Dict[str, Any]:
        """Create standardized success response."""
        response = {
            'success': True,
            'message': message,
            'timestamp': datetime.now().isoformat()
        }
        
        if data:
            response['data'] = data
        
        return response


class ConfigUtils:
    """Utility functions for configuration management."""
    
    @staticmethod
    def load_json_config(file_path: str) -> Optional[Dict[str, Any]]:
        """Load configuration from JSON file."""
        try:
            with open(file_path, 'r') as f:
                return json.load(f)
        except Exception:
            return None
    
    @staticmethod
    def save_json_config(data: Dict[str, Any], file_path: str) -> bool:
        """Save configuration to JSON file."""
        try:
            with open(file_path, 'w') as f:
                json.dump(data, f, indent=2)
            return True
        except Exception:
            return False
    
    @staticmethod
    def merge_configs(
        base_config: Dict[str, Any],
        override_config: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Merge two configuration dictionaries."""
        merged = base_config.copy()
        
        for key, value in override_config.items():
            if (key in merged and 
                isinstance(merged[key], dict) and 
                isinstance(value, dict)):
                merged[key] = ConfigUtils.merge_configs(merged[key], value)
            else:
                merged[key] = value
        
        return merged
    
    @staticmethod
    def validate_config_schema(
        config: Dict[str, Any],
        schema: Dict[str, Any]
    ) -> Tuple[bool, List[str]]:
        """
        Validate configuration against schema.
        
        Args:
            config: Configuration to validate
            schema: Schema definition
            
        Returns:
            (is_valid, list_of_errors)
        """
        errors = []
        
        # Check required fields
        required_fields = schema.get('required', [])
        for field in required_fields:
            if field not in config:
                errors.append(f"Missing required field: {field}")
        
        # Check field types
        field_types = schema.get('types', {})
        for field, expected_type in field_types.items():
            if field in config:
                if not isinstance(config[field], expected_type):
                    errors.append(f"Field '{field}' should be of type {expected_type.__name__}")
        
        # Check value ranges
        ranges = schema.get('ranges', {})
        for field, (min_val, max_val) in ranges.items():
            if field in config:
                value = config[field]
                if isinstance(value, (int, float)):
                    if value < min_val or value > max_val:
                        errors.append(f"Field '{field}' value {value} outside range [{min_val}, {max_val}]")
        
        return len(errors) == 0, errors