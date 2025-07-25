"""
Route Manager Module

Handles route and map management for the AMR fleet adapter.
Loads JSON routes and PGM maps for navigation planning.
"""

import json
import yaml
import math
from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import OccupancyGrid
import cv2
import numpy as np


@dataclass
class Waypoint:
    """Waypoint definition."""
    name: str
    x: float
    y: float
    theta: float = 0.0
    map_name: str = "map"
    connections: List[str] = None
    
    def __post_init__(self):
        if self.connections is None:
            self.connections = []


@dataclass
class Route:
    """Route definition."""
    route_id: str
    name: str
    description: str = ""
    waypoints: List[Waypoint] = None
    start_waypoint: str = ""
    end_waypoint: str = ""
    estimated_duration: float = 0.0
    
    def __post_init__(self):
        if self.waypoints is None:
            self.waypoints = []


class RouteManager(Node):
    """
    Manages routes and maps for the AMR fleet.
    
    Loads waypoints and routes from JSON files,
    handles map loading from PGM/YAML files.
    """
    
    def __init__(self, parent_node: Node, node_name: str = "route_manager"):
        super().__init__(node_name)
        
        self.parent_node = parent_node
        self.get_logger().info("Initializing Route Manager...")
        
        # Storage
        self.waypoints: Dict[str, Waypoint] = {}
        self.routes: Dict[str, Route] = {}
        
        # Map information
        self.map_info = {
            'resolution': 0.05,
            'origin': [0.0, 0.0, 0.0],
            'width': 0,
            'height': 0,
            'map_name': '',
            'loaded': False
        }
        self.occupancy_grid: Optional[OccupancyGrid] = None
        
        # Publishers
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 1)
        
        # Timer for map publishing
        self.map_timer = self.create_timer(5.0, self._publish_map)
        
        self.get_logger().info("Route Manager initialized")
    
    def load_routes_from_json(self, json_file_path: str) -> bool:
        """Load routes from JSON file."""
        try:
            self.get_logger().info(f"Loading routes from: {json_file_path}")
            
            with open(json_file_path, 'r') as f:
                data = json.load(f)
            
            # Load waypoints
            if 'waypoints' in data:
                for wp_data in data['waypoints']:
                    waypoint = Waypoint(
                        name=wp_data['name'],
                        x=wp_data['x'],
                        y=wp_data['y'],
                        theta=wp_data.get('theta', 0.0),
                        map_name=wp_data.get('map_name', 'map'),
                        connections=wp_data.get('connections', [])
                    )
                    self.waypoints[waypoint.name] = waypoint
                    
                self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints")
            
            # Load routes
            if 'routes' in data:
                for route_data in data['routes']:
                    route = Route(
                        route_id=route_data['route_id'],
                        name=route_data['name'],
                        description=route_data.get('description', ''),
                        start_waypoint=route_data.get('start_waypoint', ''),
                        end_waypoint=route_data.get('end_waypoint', ''),
                        estimated_duration=route_data.get('estimated_duration', 0.0)
                    )
                    
                    # Load route waypoints if specified
                    if 'waypoints' in route_data:
                        for wp_data in route_data['waypoints']:
                            waypoint = Waypoint(
                                name=wp_data['name'],
                                x=wp_data['x'],
                                y=wp_data['y'],
                                theta=wp_data.get('theta', 0.0),
                                map_name=wp_data.get('map_name', 'map'),
                                connections=wp_data.get('connections', [])
                            )
                            route.waypoints.append(waypoint)
                    
                    self.routes[route.route_id] = route
                    
                self.get_logger().info(f"Loaded {len(self.routes)} routes")
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to load routes: {e}")
            return False
    
    def load_map_info(self, map_yaml_file: str) -> bool:
        """Load map information from YAML file."""
        try:
            self.get_logger().info(f"Loading map info from: {map_yaml_file}")
            
            with open(map_yaml_file, 'r') as f:
                map_config = yaml.safe_load(f)
            
            # Update map info
            self.map_info.update({
                'resolution': map_config.get('resolution', 0.05),
                'origin': map_config.get('origin', [0.0, 0.0, 0.0]),
                'map_name': map_config.get('image', ''),
                'yaml_file': map_yaml_file
            })
            
            # Load PGM file
            pgm_path = map_config.get('image', '')
            if not pgm_path.startswith('/'):
                # Relative path - combine with YAML directory
                import os
                yaml_dir = os.path.dirname(map_yaml_file)
                pgm_path = os.path.join(yaml_dir, pgm_path)
            
            success = self._load_pgm_map(pgm_path)
            if success:
                self.map_info['loaded'] = True
                self.get_logger().info("Map loaded successfully")
            
            return success
            
        except Exception as e:
            self.get_logger().error(f"Failed to load map info: {e}")
            return False
    
    def _load_pgm_map(self, pgm_file_path: str) -> bool:
        """Load PGM map file."""
        try:
            # Load image using OpenCV
            map_image = cv2.imread(pgm_file_path, cv2.IMREAD_GRAYSCALE)
            
            if map_image is None:
                self.get_logger().error(f"Could not load PGM file: {pgm_file_path}")
                return False
            
            # Create occupancy grid
            self.occupancy_grid = OccupancyGrid()
            self.occupancy_grid.header.frame_id = "map"
            self.occupancy_grid.header.stamp = self.get_clock().now().to_msg()
            
            # Set map metadata
            self.occupancy_grid.info.resolution = self.map_info['resolution']
            self.occupancy_grid.info.width = map_image.shape[1]
            self.occupancy_grid.info.height = map_image.shape[0]
            
            # Set origin
            self.occupancy_grid.info.origin.position.x = self.map_info['origin'][0]
            self.occupancy_grid.info.origin.position.y = self.map_info['origin'][1]
            self.occupancy_grid.info.origin.position.z = self.map_info['origin'][2]
            self.occupancy_grid.info.origin.orientation.w = 1.0
            
            # Convert image to occupancy data
            occupancy_data = []
            for y in range(map_image.shape[0]):
                for x in range(map_image.shape[1]):
                    # Flip Y axis and convert pixel values
                    pixel = map_image[map_image.shape[0] - 1 - y, x]
                    
                    if pixel > 250:  # Free space (white)
                        occupancy_data.append(0)
                    elif pixel < 5:  # Occupied space (black)
                        occupancy_data.append(100)
                    else:  # Unknown space (gray)
                        occupancy_data.append(-1)
            
            self.occupancy_grid.data = occupancy_data
            
            # Update map info
            self.map_info.update({
                'width': map_image.shape[1],
                'height': map_image.shape[0]
            })
            
            self.get_logger().info(
                f"Loaded PGM map: {map_image.shape[1]}x{map_image.shape[0]}, "
                f"resolution: {self.map_info['resolution']}")
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to load PGM map: {e}")
            return False
    
    def get_waypoint(self, waypoint_name: str) -> Optional[Waypoint]:
        """Get waypoint by name."""
        return self.waypoints.get(waypoint_name)
    
    def get_route(self, route_id: str) -> Optional[Route]:
        """Get route by ID."""
        return self.routes.get(route_id)
    
    def get_all_waypoints(self) -> Dict[str, Waypoint]:
        """Get all waypoints."""
        return self.waypoints.copy()
    
    def get_all_routes(self) -> Dict[str, Route]:
        """Get all routes."""
        return self.routes.copy()
    
    def waypoint_to_pose(self, waypoint: Waypoint) -> PoseStamped:
        """Convert waypoint to PoseStamped."""
        pose = PoseStamped()
        pose.header.frame_id = waypoint.map_name
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = waypoint.x
        pose.pose.position.y = waypoint.y
        pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        cos_half = math.cos(waypoint.theta / 2.0)
        sin_half = math.sin(waypoint.theta / 2.0)
        
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = sin_half
        pose.pose.orientation.w = cos_half
        
        return pose
    
    def find_nearest_waypoint(
        self, 
        pose: PoseStamped, 
        max_distance: float = 2.0
    ) -> Optional[Waypoint]:
        """Find nearest waypoint to given pose."""
        nearest_waypoint = None
        min_distance = max_distance
        
        for waypoint in self.waypoints.values():
            distance = self._calculate_distance(
                pose.pose.position, 
                Point(x=waypoint.x, y=waypoint.y, z=0.0)
            )
            
            if distance < min_distance:
                min_distance = distance
                nearest_waypoint = waypoint
        
        return nearest_waypoint
    
    def find_route(self, start_waypoint: str, end_waypoint: str) -> Optional[Route]:
        """Find route between two waypoints."""
        for route in self.routes.values():
            if (route.start_waypoint == start_waypoint and 
                route.end_waypoint == end_waypoint):
                return route
        return None
    
    def map_to_world(self, map_x: float, map_y: float) -> Point:
        """Convert map coordinates to world coordinates."""
        world_point = Point()
        
        if self.map_info['loaded']:
            world_point.x = (self.map_info['origin'][0] + 
                           map_x * self.map_info['resolution'])
            world_point.y = (self.map_info['origin'][1] + 
                           map_y * self.map_info['resolution'])
            world_point.z = 0.0
        else:
            world_point.x = map_x
            world_point.y = map_y
            world_point.z = 0.0
        
        return world_point
    
    def world_to_map(self, world_point: Point) -> Tuple[float, float]:
        """Convert world coordinates to map coordinates."""
        if self.map_info['loaded']:
            map_x = ((world_point.x - self.map_info['origin'][0]) / 
                    self.map_info['resolution'])
            map_y = ((world_point.y - self.map_info['origin'][1]) / 
                    self.map_info['resolution'])
            return map_x, map_y
        else:
            return world_point.x, world_point.y
    
    def validate_routes(self) -> bool:
        """Validate route connectivity."""
        valid = True
        
        # Check waypoint connections
        for waypoint_name, waypoint in self.waypoints.items():
            for connection in waypoint.connections:
                if connection not in self.waypoints:
                    self.get_logger().error(
                        f"Waypoint '{waypoint_name}' has invalid connection to '{connection}'")
                    valid = False
        
        # Check route waypoints
        for route_id, route in self.routes.items():
            if route.start_waypoint and route.start_waypoint not in self.waypoints:
                self.get_logger().error(
                    f"Route '{route_id}' has invalid start waypoint: '{route.start_waypoint}'")
                valid = False
            
            if route.end_waypoint and route.end_waypoint not in self.waypoints:
                self.get_logger().error(
                    f"Route '{route_id}' has invalid end waypoint: '{route.end_waypoint}'")
                valid = False
        
        if valid:
            self.get_logger().info("Route validation passed")
        else:
            self.get_logger().warning("Route validation failed")
        
        return valid
    
    def export_routes_to_json(self) -> str:
        """Export routes to JSON string."""
        data = {
            'waypoints': [],
            'routes': []
        }
        
        # Export waypoints
        for waypoint in self.waypoints.values():
            data['waypoints'].append({
                'name': waypoint.name,
                'x': waypoint.x,
                'y': waypoint.y,
                'theta': waypoint.theta,
                'map_name': waypoint.map_name,
                'connections': waypoint.connections
            })
        
        # Export routes
        for route in self.routes.values():
            route_data = {
                'route_id': route.route_id,
                'name': route.name,
                'description': route.description,
                'start_waypoint': route.start_waypoint,
                'end_waypoint': route.end_waypoint,
                'estimated_duration': route.estimated_duration
            }
            
            if route.waypoints:
                route_data['waypoints'] = []
                for waypoint in route.waypoints:
                    route_data['waypoints'].append({
                        'name': waypoint.name,
                        'x': waypoint.x,
                        'y': waypoint.y,
                        'theta': waypoint.theta,
                        'map_name': waypoint.map_name,
                        'connections': waypoint.connections
                    })
            
            data['routes'].append(route_data)
        
        return json.dumps(data, indent=2)
    
    def get_map_info(self) -> Dict[str, Any]:
        """Get map information."""
        return self.map_info.copy()
    
    def _calculate_distance(self, p1: Point, p2: Point) -> float:
        """Calculate Euclidean distance between two points."""
        dx = p1.x - p2.x
        dy = p1.y - p2.y
        dz = p1.z - p2.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def _publish_map(self):
        """Publish map for visualization."""
        if (self.occupancy_grid and 
            self.map_info['loaded'] and 
            self.map_pub.get_subscription_count() > 0):
            
            self.occupancy_grid.header.stamp = self.get_clock().now().to_msg()
            self.map_pub.publish(self.occupancy_grid)