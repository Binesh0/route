# AMR Fleet Adapter

A comprehensive AMR (Autonomous Mobile Robot) Fleet Adapter for ROS2 Humble using RMF (Robot Middleware Framework) with TurtleBot integration and Nav2 navigation.

## Features

- **RMF Integration**: Full integration with Robot Middleware Framework for multi-robot coordination
- **TurtleBot Support**: Optimized for TurtleBot robots with Nav2 navigation stack
- **JSON Route Management**: Flexible route definition using JSON format
- **PGM Map Support**: Load and use PGM maps for navigation
- **Fleet Management**: Multi-robot fleet coordination and task assignment
- **Real-time State Monitoring**: Robot position, battery, and navigation status tracking
- **Task Execution**: Support for delivery, patrol, and custom tasks

## Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  AMR Fleet      │    │  Route Manager  │    │  TurtleBot      │
│  Adapter        │◄──►│                 │◄──►│  Robot          │
│                 │    │                 │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         ▲                       ▲                       ▲
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│     RMF         │    │   JSON Routes   │    │     Nav2        │
│   Traffic       │    │   PGM Maps      │    │   Navigation    │
│   Manager       │    │                 │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## Dependencies

### Required ROS2 Packages
- `rmf_fleet_adapter`
- `rmf_traffic`
- `rmf_traffic_ros2`
- `rmf_utils`
- `nav2_msgs`
- `nav2_util`
- `geometry_msgs`
- `nav_msgs`
- `tf2_ros`

### System Dependencies
- `nlohmann-json-dev`
- `yaml-cpp`
- `opencv2`

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

### Basic Launch

Launch the AMR fleet adapter with default configuration:

```bash
ros2 launch amr_fleet_adapter amr_fleet_adapter.launch.py
```

### Custom Configuration

Launch with custom parameters:

```bash
ros2 launch amr_fleet_adapter amr_fleet_adapter.launch.py \
    fleet_name:=my_fleet \
    robot_names:='["robot1", "robot2", "robot3"]' \
    map_yaml_file:=/path/to/your/map.yaml
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