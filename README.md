# AMR Fleet Adapter with Delivery Capabilities

A comprehensive Python-based AMR (Autonomous Mobile Robot) Fleet Adapter for ROS2 Humble using RMF (Robot Middleware Framework) with TurtleBot integration, Nav2 navigation, and advanced delivery management capabilities.

## Features

### 🚚 **Advanced Delivery Management**
- **Complete Delivery Lifecycle**: From order creation to delivery completion
- **Multiple Cargo Types**: Packages, documents, food, medicine, equipment, fragile items
- **Priority-based Scheduling**: Emergency, urgent, high, normal, and low priority deliveries
- **Customer Management**: Contact information, special requirements, delivery preferences
- **Proof of Delivery**: Signature and photo verification support
- **Real-time Tracking**: Live delivery status updates and progress monitoring

### 🤖 **Fleet Coordination**
- **RMF Integration**: Full integration with Robot Middleware Framework for multi-robot coordination
- **TurtleBot Support**: Optimized for TurtleBot robots with Nav2 navigation stack
- **Intelligent Task Assignment**: Automatic robot selection based on proximity and capabilities
- **Load Balancing**: Distribute deliveries across available robots efficiently
- **Emergency Management**: Fleet-wide emergency stop and resume capabilities

### 🗺️ **Navigation & Mapping**
- **JSON Route Management**: Flexible route definition using JSON format
- **PGM Map Support**: Load and use PGM maps for navigation
- **Waypoint Navigation**: Predefined waypoints with automatic path planning
- **Dynamic Route Optimization**: Adjust routes based on real-time conditions
- **Obstacle Avoidance**: Integrated Nav2 navigation with collision avoidance

### 📊 **Monitoring & Analytics**
- **Real-time State Monitoring**: Robot position, battery, and navigation status tracking
- **Delivery Analytics**: Success rates, average delivery times, efficiency metrics
- **Fleet Performance**: Robot utilization, distance traveled, task completion rates
- **Live Dashboard**: Real-time fleet status and delivery progress visualization

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    AMR Fleet Adapter (Main)                    │
│  - Fleet coordination and task management                      │
│  - RMF integration and robot discovery                        │
│  - Auto-assignment and load balancing                         │
└─────────────────┬───────────────────┬───────────────────────────┘
                  │                   │
        ┌─────────▼─────────┐  ┌──────▼──────┐
        │ Delivery Manager  │  │Route Manager│
        │ - Task lifecycle  │  │- Map loading│
        │ - Priority queue  │  │- Waypoints  │
        │ - Customer mgmt   │  │- Navigation │
        └─────────┬─────────┘  └──────┬──────┘
                  │                   │
                  │    ┌──────────────▼──────────────┐
                  │    │        TurtleBot Robot      │
                  │    │     Controllers (N bots)    │
                  │    │ - Navigation & cargo mgmt   │
                  │    │ - State monitoring          │
                  └────┤ - Safety & performance      │
                       └─────────────────────────────┘
                                     │
                       ┌─────────────▼─────────────┐
                       │        Nav2 Stack         │
                       │   Navigation & Planning   │
                       └───────────────────────────┘
```

The AMR Fleet Adapter consists of four main Python modules working together:

### 1. AMRFleetAdapter (Main Coordinator)
- **Fleet Management**: Central coordination of all robots and delivery operations
- **RMF Integration**: Interfaces with Robot Middleware Framework for task scheduling
- **Auto-assignment**: Intelligent task distribution based on robot capabilities and proximity
- **Configuration Management**: Loads and manages fleet parameters from YAML files

### 2. DeliveryManager (Delivery Operations)
- **Task Lifecycle**: Manages complete delivery process from creation to completion
- **Priority Queue**: Handles urgent, high, normal, and low priority deliveries
- **Customer Management**: Tracks customer information and special requirements
- **Status Tracking**: Real-time delivery progress monitoring and updates
- **Analytics**: Performance metrics and delivery statistics

### 3. TurtleBotController (Robot Control)
- **Navigation Control**: Nav2 integration for autonomous navigation
- **Cargo Handling**: Simulated loading/unloading operations with weight tracking
- **State Management**: Real-time robot status, battery, and position monitoring
- **Safety Features**: Emergency stop, collision avoidance, error recovery
- **Performance Tracking**: Distance traveled, deliveries completed, efficiency metrics

### 4. RouteManager (Navigation & Mapping)
- **Map Management**: Loads PGM maps and converts to ROS2 OccupancyGrid
- **Route Planning**: JSON-based waypoint and route management
- **Coordinate Conversion**: World-to-map coordinate transformations
- **Path Validation**: Ensures route connectivity and accessibility

## Dependencies

### Required ROS2 Packages
- `rmf_fleet_adapter_python` - Python RMF fleet adapter integration
- `rmf_fleet_adapter` - Core RMF fleet adapter functionality
- `rmf_traffic` - RMF traffic management
- `rmf_traffic_ros2` - RMF traffic ROS2 integration
- `rmf_task_msgs` - RMF task message definitions
- `rmf_fleet_msgs` - RMF fleet message definitions
- `nav2_msgs` - Nav2 navigation messages
- `nav2_simple_commander` - Simplified Nav2 Python API
- `rclpy` - ROS2 Python client library
- `std_msgs` - Standard ROS2 messages
- `geometry_msgs` - Geometry-related messages
- `nav_msgs` - Navigation messages
- `tf2_ros` - Transform library for ROS2
- `tf2_geometry_msgs` - TF2 geometry message support

### Python Dependencies
- `python3-yaml` - YAML configuration file parsing
- `python3-numpy` - Numerical computations
- `python3-opencv` - Image processing for PGM map loading
- `dataclasses` - Python dataclass support (Python 3.7+)
- `typing` - Type hints support
- `datetime` - Date and time handling
- `json` - JSON data processing
- `math` - Mathematical functions

## Installation

1. **Clone the repository** into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository_url> amr_fleet_adapter
   ```

2. **Install dependencies**:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the package**:
   ```bash
   colcon build --packages-select amr_fleet_adapter
   source install/setup.bash
   ```

## Configuration

### Fleet Configuration

Edit `config/fleet_config.yaml` to configure your fleet:

```yaml
amr_fleet_adapter:
  ros__parameters:
    fleet_name: "turtlebot_fleet"
    robot_names: ["turtlebot1", "turtlebot2"]
    route_json_file: "config/routes.json"
    map_yaml_file: "maps/office_map.yaml"
    perform_deliveries: true
    accept_patrol_requests: true
```

### Robot Traits

Configure robot characteristics in `config/robot_traits.yaml`:

```yaml
linear:
  nominal_velocity: 0.5      # m/s
  nominal_acceleration: 0.75  # m/s^2

angular:
  nominal_velocity: 1.0      # rad/s
  nominal_acceleration: 2.0   # rad/s^2

footprint:
  radius: 0.3               # meters

vicinity: 1.0                # meters
```

### Route Definition

Define waypoints and routes in `config/routes.json`:

```json
{
  "waypoints": [
    {
      "name": "home",
      "x": 0.0,
      "y": 0.0,
      "theta": 0.0,
      "map_name": "map",
      "connections": ["station_a", "station_b"]
    }
  ],
  "routes": [
    {
      "route_id": "home_to_station_a",
      "name": "Home to Station A",
      "start_waypoint": "home",
      "end_waypoint": "station_a",
      "estimated_duration": 12.0
    }
  ]
}
```

## Usage

### 🚀 Quick Start

1. **Launch the Fleet Adapter**
   ```bash
   # Basic launch with default configuration
   ros2 launch amr_fleet_adapter amr_fleet_adapter.launch.py
   
   # Launch with custom fleet settings
   ros2 launch amr_fleet_adapter amr_fleet_adapter.launch.py \
       fleet_name:=delivery_fleet \
       robot_names:='["turtlebot1", "turtlebot2"]' \
       max_concurrent_deliveries:=5 \
       auto_assign_tasks:=true
   ```

2. **Send a Delivery Request**
   ```bash
   # Run the example client
   ros2 run amr_fleet_adapter delivery_client_example.py
   ```

3. **Monitor Fleet Status**
   ```bash
   # Watch fleet state updates
   ros2 topic echo /fleet_state
   
   # Monitor delivery metrics
   ros2 topic echo /delivery_metrics
   ```

### 📦 Delivery Examples

#### Example 1: Simple Package Delivery
```bash
# Send via ROS2 topic
ros2 topic pub /delivery_requests std_msgs/String "data: '{
  \"order_id\": \"PKG001\",
  \"pickup_location\": \"home\",
  \"dropoff_location\": \"station_a\",
  \"cargo_info\": {
    \"type\": \"package\",
    \"weight\": 2.5,
    \"dimensions\": {\"length\": 0.3, \"width\": 0.2, \"height\": 0.15}
  },
  \"priority\": \"normal\",
  \"customer_info\": {
    \"name\": \"John Doe\",
    \"phone\": \"+1-555-0123\"
  }
}'"
```

#### Example 2: Urgent Medical Delivery
```bash
# High priority medical delivery with special requirements
ros2 topic pub /delivery_requests std_msgs/String "data: '{
  \"order_id\": \"MED001\",
  \"pickup_location\": \"pharmacy\",
  \"dropoff_location\": \"hospital\",
  \"cargo_info\": {
    \"type\": \"medicine\",
    \"weight\": 0.8,
    \"temperature_sensitive\": true,
    \"special_instructions\": \"Keep refrigerated\"
  },
  \"priority\": \"urgent\",
  \"special_requirements\": {
    \"signature\": true,
    \"photo_proof\": true,
    \"max_attempts\": 1
  }
}'"
```

#### Example 3: Fleet Management Commands
```bash
# Emergency stop all robots
ros2 topic pub /fleet_commands std_msgs/String "data: '{\"command\": \"emergency_stop\"}'"

# Resume fleet operations
ros2 topic pub /fleet_commands std_msgs/String "data: '{\"command\": \"resume\"}'"

# Get current fleet status
ros2 topic pub /fleet_commands std_msgs/String "data: '{\"command\": \"get_status\"}'"

# Cancel a specific delivery
ros2 topic pub /fleet_commands std_msgs/String "data: '{
  \"command\": \"cancel_task\",
  \"task_id\": \"task-uuid-here\",
  \"reason\": \"Customer request\"
}'"
```

### With TurtleBot Simulation

1. **Launch TurtleBot simulation**:
   ```bash
   # Terminal 1: Launch TurtleBot simulation
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   
   # Terminal 2: Launch navigation
   ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true
   ```

2. **Launch AMR Fleet Adapter**:
   ```bash
   # Terminal 3: Launch fleet adapter
   ros2 launch amr_fleet_adapter amr_fleet_adapter.launch.py \
       robot_names:='["turtlebot3"]'
   ```

## API Reference

### Core Classes

#### `AMRFleetAdapter`
Main fleet adapter class that coordinates the entire fleet.

**Key Methods:**
- `initialize()`: Initialize the fleet adapter
- `start()`: Start fleet operations
- `stop()`: Stop fleet operations

#### `TurtleBotRobot`
Individual robot control and navigation interface.

**Key Methods:**
- `navigate_to_pose(pose, callback)`: Navigate to specific pose
- `navigate_to_waypoint(index, callback)`: Navigate to waypoint
- `stop_navigation()`: Stop current navigation
- `emergency_stop()`: Emergency stop
- `get_current_state()`: Get robot state

#### `RouteManager`
JSON route parsing and map management.

**Key Methods:**
- `load_routes_from_json(file_path)`: Load routes from JSON file
- `load_map_info(yaml_file)`: Load map information
- `create_rmf_graph()`: Create RMF traffic graph
- `find_nearest_waypoint(pose)`: Find nearest waypoint

### ROS2 Topics

#### Subscribed Topics
- `/{robot_name}/odom` (nav_msgs/Odometry): Robot odometry
- `/{robot_name}/amcl_pose` (geometry_msgs/PoseWithCovarianceStamped): AMCL pose

#### Published Topics
- `/{robot_name}/cmd_vel` (geometry_msgs/Twist): Velocity commands
- `/{robot_name}/initialpose` (geometry_msgs/PoseWithCovarianceStamped): Initial pose
- `/map` (nav_msgs/OccupancyGrid): Map for visualization

#### Action Clients
- `/{robot_name}/navigate_to_pose` (nav2_msgs/NavigateToPose): Navigation action

## JSON Route Format

### Waypoint Structure
```json
{
  "name": "waypoint_name",
  "x": 1.0,
  "y": 2.0,
  "theta": 0.0,
  "map_name": "map",
  "connections": ["other_waypoint1", "other_waypoint2"]
}
```

### Route Structure
```json
{
  "route_id": "unique_route_id",
  "name": "Human Readable Name",
  "description": "Route description",
  "start_waypoint": "start_waypoint_name",
  "end_waypoint": "end_waypoint_name",
  "waypoints": [...],
  "estimated_duration": 30.0
}
```

## Monitoring and Debugging

### View Robot Status
```bash
ros2 topic echo /rmf_fleet_adapter/robot_state
```

### Check Navigation Status
```bash
ros2 topic echo /{robot_name}/navigate_to_pose/_action/status
```

### Monitor Fleet Tasks
```bash
ros2 topic echo /rmf_fleet_adapter/task_summaries
```

## Troubleshooting

### Common Issues

1. **Robot not discovered**
   - Check that robot names match the actual robot namespaces
   - Verify Nav2 action server is running: `ros2 action list`

2. **Navigation failures**
   - Check map loading: `ros2 topic echo /map`
   - Verify AMCL localization: `ros2 topic echo /{robot_name}/amcl_pose`

3. **RMF integration issues**
   - Check RMF traffic manager: `ros2 node list | grep rmf`
   - Verify graph loading in logs

### Debug Mode

Launch with debug logging:
```bash
ros2 launch amr_fleet_adapter amr_fleet_adapter.launch.py log_level:=debug
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

This project is licensed under the Apache License 2.0 - see the LICENSE file for details.

## Support

For issues and questions:
- Create an issue on GitHub
- Check the ROS2 and RMF documentation
- Review the troubleshooting section above