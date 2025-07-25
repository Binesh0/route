#ifndef AMR_FLEET_ADAPTER__ROUTEMANAGER_HPP
#define AMR_FLEET_ADAPTER__ROUTEMANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rmf_traffic/agv/Graph.hpp>

#include <nlohmann/json.hpp>
#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include <array>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace amr_fleet_adapter
{

struct RouteWaypoint
{
  std::string name;
  double x;
  double y;
  double theta;
  std::string map_name;
  std::vector<std::string> connections;
};

struct Route
{
  std::string route_id;
  std::string name;
  std::string description;
  std::vector<RouteWaypoint> waypoints;
  std::string start_waypoint;
  std::string end_waypoint;
  double estimated_duration; // seconds
};

struct MapInfo
{
  std::string map_name;
  std::string pgm_file_path;
  std::string yaml_file_path;
  double resolution;
  std::array<double, 2> origin;
  nav_msgs::msg::OccupancyGrid occupancy_grid;
};

class RouteManager
{
public:
  explicit RouteManager(rclcpp::Node::SharedPtr node);
  ~RouteManager();

  /// Initialize the route manager
  bool initialize();

  /// Load routes from JSON file
  bool load_routes_from_json(const std::string& json_file_path);

  /// Load routes from JSON string
  bool load_routes_from_json_string(const std::string& json_string);

  /// Load map information from YAML file
  bool load_map_info(const std::string& map_yaml_file);

  /// Convert routes to RMF traffic graph
  rmf_traffic::agv::Graph create_rmf_graph() const;

  /// Get route by ID
  std::shared_ptr<Route> get_route(const std::string& route_id) const;

  /// Get all available routes
  std::vector<std::shared_ptr<Route>> get_all_routes() const;

  /// Find route between two waypoints
  std::shared_ptr<Route> find_route(
    const std::string& start_waypoint,
    const std::string& end_waypoint) const;

  /// Get waypoint by name
  std::shared_ptr<RouteWaypoint> get_waypoint(const std::string& waypoint_name) const;

  /// Convert waypoint to geometry_msgs::PoseStamped
  geometry_msgs::msg::PoseStamped waypoint_to_pose(
    const RouteWaypoint& waypoint) const;

  /// Find nearest waypoint to given position
  std::shared_ptr<RouteWaypoint> find_nearest_waypoint(
    const geometry_msgs::msg::PoseStamped& pose,
    double max_distance = 2.0) const;

  /// Get map information
  const MapInfo& get_map_info() const;

  /// Convert map coordinates to world coordinates
  geometry_msgs::msg::Point map_to_world(double map_x, double map_y) const;

  /// Convert world coordinates to map coordinates
  std::pair<double, double> world_to_map(const geometry_msgs::msg::Point& world_point) const;

  /// Validate route connectivity
  bool validate_routes() const;

  /// Export routes to JSON
  std::string export_routes_to_json() const;

private:
  // ROS2 node
  rclcpp::Node::SharedPtr node_;
  
  // Route storage
  std::unordered_map<std::string, std::shared_ptr<Route>> routes_;
  std::unordered_map<std::string, std::shared_ptr<RouteWaypoint>> waypoints_;
  
  // Map information
  MapInfo map_info_;
  bool map_loaded_;
  
  // Publishers for visualization
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr map_publish_timer_;
  
  // Helper methods
  bool parse_json_routes(const nlohmann::json& json_data);
  bool load_pgm_map(const std::string& pgm_file_path);
  RouteWaypoint parse_waypoint(const nlohmann::json& waypoint_json);
  Route parse_route(const nlohmann::json& route_json);
  double calculate_route_duration(const Route& route) const;
  void publish_map();
  
  // Validation methods
  bool validate_waypoint_connections() const;
  bool validate_route_waypoints(const Route& route) const;
};

// JSON serialization support
void to_json(nlohmann::json& j, const RouteWaypoint& waypoint);
void from_json(const nlohmann::json& j, RouteWaypoint& waypoint);
void to_json(nlohmann::json& j, const Route& route);
void from_json(const nlohmann::json& j, Route& route);

} // namespace amr_fleet_adapter

#endif // AMR_FLEET_ADAPTER__ROUTEMANAGER_HPP