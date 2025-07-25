# TurtleBot Graph Navigation

A ROS2 package for TurtleBot navigation using graph-based path planning with JSON map structures.

## Overview

This package provides graph-based navigation for TurtleBot3 simulation in Gazebo. It allows you to:

- Define navigation graphs using JSON map structures
- Visualize nodes and edges in RViz
- Plan optimal paths using Dijkstra's algorithm
- Navigate between waypoints using Nav2 or basic control
- Send navigation goals via ROS2 topics

## Features

- **Graph Generation**: Load and visualize navigation graphs from JSON files
- **Path Planning**: Dijkstra's algorithm for shortest path calculation
- **Dual Navigation Modes**: Nav2 integration or basic velocity control
- **Visualization**: Real-time graph and path visualization in RViz
- **Interactive Goals**: Command-line and topic-based goal publishing
- **TurtleBot3 Integration**: Full Gazebo simulation support

## Package Structure

```
turtlebot_graph_nav/
├── turtlebot_graph_nav/
│   ├── __init__.py
│   ├── graph_generator.py      # Graph loading and visualization
│   ├── graph_navigator.py      # Navigation control
│   └── goal_publisher.py       # Goal publishing utilities
├── config/
│   └── map_graph.json          # Sample navigation graph
├── launch/
│   └── turtlebot_graph_nav.launch.py  # Main launch file
├── package.xml
├── setup.py
└── README.md
```

## Dependencies

- ROS2 Humble (or later)
- TurtleBot3 packages
- Nav2 navigation stack
- Gazebo simulation
- RViz2

```bash
sudo apt install ros-humble-turtlebot3* ros-humble-nav2-*
```

## Installation

1. Create a ROS2 workspace (if you don't have one):
```bash
mkdir -p ~/turtlebot_ws/src
cd ~/turtlebot_ws/src
```

2. Clone or copy this package to your workspace:
```bash
# Copy the package files to ~/turtlebot_ws/src/turtlebot_graph_nav/
```

3. Build the package:
```bash
cd ~/turtlebot_ws
colcon build --packages-select turtlebot_graph_nav
source install/setup.bash
```

## JSON Map Structure

The navigation graph is defined in JSON format with nodes and edges:

```json
{
  "nodes": [
    {
      "id": "start",
      "x": 0.0,
      "y": 0.0,
      "type": "waypoint",
      "description": "Starting position"
    }
  ],
  "edges": [
    {
      "from": "start",
      "to": "A",
      "weight": 2.0,
      "bidirectional": true,
      "type": "corridor"
    }
  ]
}
```

### Node Properties
- `id`: Unique identifier
- `x`, `y`: Position coordinates (meters)
- `type`: Node type ("waypoint", "intersection", etc.)
- `description`: Optional description

### Edge Properties
- `from`, `to`: Node IDs
- `weight`: Edge cost (default: Euclidean distance)
- `bidirectional`: Two-way navigation (default: true)
- `type`: Edge type ("corridor", "shortcut", etc.)

## Usage

### 1. Launch the Full System

```bash
# Set TurtleBot3 model
export TURTLEBOT3_MODEL=burger

# Launch everything (Gazebo + Nav2 + Graph Navigation)
ros2 launch turtlebot_graph_nav turtlebot_graph_nav.launch.py
```

### 2. Send Navigation Goals

#### Via Topic:
```bash
ros2 topic pub /graph_goal std_msgs/String "data: 'start,center'"
```

#### Via Goal Publisher:
```bash
# Interactive mode
ros2 run turtlebot_graph_nav goal_publisher

# Direct command
ros2 run turtlebot_graph_nav goal_publisher start center
```

### 3. Visualize in RViz

The launch file automatically opens RViz with:
- Graph nodes (green spheres for waypoints, red for intersections)
- Graph edges (blue lines)
- Node labels
- Planned paths
- TurtleBot pose and sensor data

## Launch Parameters

```bash
ros2 launch turtlebot_graph_nav turtlebot_graph_nav.launch.py \
    map_file:=/path/to/your/map.json \
    use_nav2:=true \
    use_rviz:=true \
    x_pose:=0.0 \
    y_pose:=0.0
```

### Available Parameters:
- `map_file`: Path to JSON map file
- `use_nav2`: Use Nav2 for navigation (default: true)
- `use_rviz`: Launch RViz visualization (default: true)
- `x_pose`, `y_pose`: Initial robot position
- `world`: Gazebo world file
- `use_sim_time`: Use simulation time (default: true)

## Nodes and Topics

### Nodes:
- `graph_generator`: Loads graph and publishes visualization
- `graph_navigator`: Handles navigation between waypoints
- `goal_publisher`: Utility for sending navigation goals

### Topics:
- `/graph_markers` (MarkerArray): Graph visualization
- `/planned_path` (Path): Computed path
- `/graph_goal` (String): Navigation goal input
- `/cmd_vel` (Twist): Robot velocity commands
- `/odom` (Odometry): Robot odometry

## Navigation Modes

### 1. Nav2 Mode (Default)
Uses the Nav2 navigation stack for obstacle avoidance and path following:
```bash
ros2 launch turtlebot_graph_nav turtlebot_graph_nav.launch.py use_nav2:=true
```

### 2. Basic Control Mode
Direct velocity control between waypoints:
```bash
ros2 launch turtlebot_graph_nav turtlebot_graph_nav.launch.py use_nav2:=false
```

## Customization

### Creating Your Own Map

1. Edit `config/map_graph.json` or create a new JSON file
2. Define your nodes with real-world coordinates
3. Connect nodes with edges
4. Launch with your custom map:
```bash
ros2 launch turtlebot_graph_nav turtlebot_graph_nav.launch.py \
    map_file:=/path/to/your/custom_map.json
```

### Adding New Node Types

Modify the `graph_generator.py` to handle custom node types:
```python
# In publish_graph_markers method
if node_data['type'] == 'your_custom_type':
    marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # Yellow
```

## Troubleshooting

### Common Issues:

1. **TurtleBot3 model not set**:
```bash
export TURTLEBOT3_MODEL=burger
```

2. **Nav2 not starting**:
```bash
# Check if navigation launch file exists
ros2 pkg list | grep nav2
```

3. **Graph not visible in RViz**:
- Check if `/graph_markers` topic is publishing
- Verify RViz is subscribed to the correct topics

4. **Navigation not working**:
- Ensure map coordinates match Gazebo world
- Check if robot odometry is publishing
- Verify goal format: "start_node,goal_node"

### Debug Commands:

```bash
# Check running nodes
ros2 node list

# Monitor topics
ros2 topic list
ros2 topic echo /graph_goal

# Check transforms
ros2 run tf2_tools view_frames
```

## Examples

### Example Goals:
```bash
# Navigate from start to center
ros2 topic pub /graph_goal std_msgs/String "data: 'start,center'"

# Go around the perimeter
ros2 topic pub /graph_goal std_msgs/String "data: 'start,A'"
ros2 topic pub /graph_goal std_msgs/String "data: 'A,B'"
ros2 topic pub /graph_goal std_msgs/String "data: 'B,C'"
```

### Interactive Session:
```bash
ros2 run turtlebot_graph_nav goal_publisher
# Enter: start center
# Enter: center D
# Enter: D F
# Enter: quit
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Add your improvements
4. Test with TurtleBot3 simulation
5. Submit a pull request

## License

Apache License 2.0

## Author

Created for TurtleBot3 graph-based navigation demonstrations and research.