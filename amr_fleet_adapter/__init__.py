"""
AMR Fleet Adapter with Delivery Capabilities

A comprehensive Python-based AMR Fleet Adapter for ROS2 Humble using RMF
(Robot Middleware Framework) with TurtleBot integration, Nav2 navigation,
and advanced delivery management capabilities.

Author: Your Name
License: Apache-2.0
"""

__version__ = "0.1.0"
__author__ = "Your Name"
__email__ = "your.email@example.com"

from .amr_fleet_adapter import AMRFleetAdapter
from .delivery_manager import DeliveryManager, DeliveryTask, DeliveryStatus
from .robot_controller import TurtleBotController, RobotState
from .route_manager import RouteManager, Waypoint, Route
from .utils import DeliveryUtils, NavigationUtils

__all__ = [
    "AMRFleetAdapter",
    "DeliveryManager", 
    "DeliveryTask",
    "DeliveryStatus",
    "TurtleBotController",
    "RobotState", 
    "RouteManager",
    "Waypoint",
    "Route",
    "DeliveryUtils",
    "NavigationUtils"
]