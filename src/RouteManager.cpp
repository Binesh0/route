#include "amr_fleet_adapter/RouteManager.hpp"
#include <rmf_traffic/agv/Graph.hpp>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

namespace amr_fleet_adapter
{

RouteManager::RouteManager(rclcpp::Node::SharedPtr node)
: node_(node)
, map_loaded_(false)
{
  RCLCPP_INFO(node_->get_logger(), "Creating RouteManager");
}

RouteManager::~RouteManager() = default;

bool RouteManager::initialize()
{
  RCLCPP_INFO(node_->get_logger(), "Initializing RouteManager");
  
  // Create map publisher for visualization
  map_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 1);
  
  // Create timer to publish map periodically
  map_publish_timer_ = node_->create_wall_timer(
    std::chrono::seconds(5),
    [this]() { publish_map(); });
  
  return true;
}

bool RouteManager::load_routes_from_json(const std::string& json_file_path)
{
  RCLCPP_INFO(node_->get_logger(), "Loading routes from JSON file: %s", json_file_path.c_str());
  
  try
  {
    std::ifstream file(json_file_path);
    if (!file.is_open())
    {
      RCLCPP_ERROR(node_->get_logger(), "Cannot open JSON file: %s", json_file_path.c_str());
      return false;
    }
    
    nlohmann::json json_data;
    file >> json_data;
    file.close();
    
    return parse_json_routes(json_data);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(node_->get_logger(), "Exception loading routes from JSON: %s", e.what());
    return false;
  }
}

bool RouteManager::load_routes_from_json_string(const std::string& json_string)
{
  RCLCPP_INFO(node_->get_logger(), "Loading routes from JSON string");
  
  try
  {
    nlohmann::json json_data = nlohmann::json::parse(json_string);
    return parse_json_routes(json_data);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(node_->get_logger(), "Exception parsing JSON string: %s", e.what());
    return false;
  }
}

bool RouteManager::load_map_info(const std::string& map_yaml_file)
{
  RCLCPP_INFO(node_->get_logger(), "Loading map info from: %s", map_yaml_file.c_str());
  
  try
  {
    YAML::Node config = YAML::LoadFile(map_yaml_file);
    
    map_info_.yaml_file_path = map_yaml_file;
    map_info_.resolution = config["resolution"].as<double>();
    map_info_.map_name = config["image"].as<std::string>();
    
    // Handle relative path for PGM file
    std::string pgm_path = config["image"].as<std::string>();
    if (pgm_path[0] != '/')
    {
      // Relative path - combine with YAML file directory
      std::string yaml_dir = map_yaml_file.substr(0, map_yaml_file.find_last_of("/\\"));
      map_info_.pgm_file_path = yaml_dir + "/" + pgm_path;
    }
    else
    {
      map_info_.pgm_file_path = pgm_path;
    }
    
    // Get origin
    auto origin = config["origin"].as<std::vector<double>>();
    if (origin.size() >= 2)
    {
      map_info_.origin[0] = origin[0];
      map_info_.origin[1] = origin[1];
    }
    
    // Load the PGM map
    if (!load_pgm_map(map_info_.pgm_file_path))
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to load PGM map: %s", map_info_.pgm_file_path.c_str());
      return false;
    }
    
    map_loaded_ = true;
    RCLCPP_INFO(node_->get_logger(), "Map loaded successfully: %s", map_info_.map_name.c_str());
    return true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(node_->get_logger(), "Exception loading map info: %s", e.what());
    return false;
  }
}

rmf_traffic::agv::Graph RouteManager::create_rmf_graph() const
{
  RCLCPP_INFO(node_->get_logger(), "Creating RMF traffic graph from routes");
  
  rmf_traffic::agv::Graph graph;
  
  // Create waypoints first
  std::unordered_map<std::string, std::size_t> waypoint_indices;
  
  for (const auto& [waypoint_name, waypoint] : waypoints_)
  {
    std::size_t index = graph.add_waypoint(waypoint->map_name, {waypoint->x, waypoint->y});
    waypoint_indices[waypoint_name] = index;
    
    RCLCPP_DEBUG(node_->get_logger(), "Added waypoint '%s' at index %zu [%.2f, %.2f]",
                 waypoint_name.c_str(), index, waypoint->x, waypoint->y);
  }
  
  // Create lanes between connected waypoints
  for (const auto& [waypoint_name, waypoint] : waypoints_)
  {
    std::size_t from_index = waypoint_indices[waypoint_name];
    
    for (const auto& connection : waypoint->connections)
    {
      auto it = waypoint_indices.find(connection);
      if (it != waypoint_indices.end())
      {
        std::size_t to_index = it->second;
        
        // Add bidirectional lane
        graph.add_lane(from_index, to_index);
        
        RCLCPP_DEBUG(node_->get_logger(), "Added lane from '%s' (%zu) to '%s' (%zu)",
                     waypoint_name.c_str(), from_index, connection.c_str(), to_index);
      }
      else
      {
        RCLCPP_WARN(node_->get_logger(), "Connection '%s' not found for waypoint '%s'",
                    connection.c_str(), waypoint_name.c_str());
      }
    }
  }
  
  RCLCPP_INFO(node_->get_logger(), "Created RMF graph with %zu waypoints and %zu lanes",
              graph.num_waypoints(), graph.num_lanes());
  
  return graph;
}

std::shared_ptr<Route> RouteManager::get_route(const std::string& route_id) const
{
  auto it = routes_.find(route_id);
  return (it != routes_.end()) ? it->second : nullptr;
}

std::vector<std::shared_ptr<Route>> RouteManager::get_all_routes() const
{
  std::vector<std::shared_ptr<Route>> all_routes;
  all_routes.reserve(routes_.size());
  
  for (const auto& [id, route] : routes_)
  {
    all_routes.push_back(route);
  }
  
  return all_routes;
}

std::shared_ptr<Route> RouteManager::find_route(
  const std::string& start_waypoint,
  const std::string& end_waypoint) const
{
  for (const auto& [id, route] : routes_)
  {
    if (route->start_waypoint == start_waypoint && route->end_waypoint == end_waypoint)
    {
      return route;
    }
  }
  return nullptr;
}

std::shared_ptr<RouteWaypoint> RouteManager::get_waypoint(const std::string& waypoint_name) const
{
  auto it = waypoints_.find(waypoint_name);
  return (it != waypoints_.end()) ? it->second : nullptr;
}

geometry_msgs::msg::PoseStamped RouteManager::waypoint_to_pose(const RouteWaypoint& waypoint) const
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = node_->get_clock()->now();
  
  pose.pose.position.x = waypoint.x;
  pose.pose.position.y = waypoint.y;
  pose.pose.position.z = 0.0;
  
  // Convert theta to quaternion
  tf2::Quaternion q;
  q.setRPY(0, 0, waypoint.theta);
  pose.pose.orientation = tf2::toMsg(q);
  
  return pose;
}

std::shared_ptr<RouteWaypoint> RouteManager::find_nearest_waypoint(
  const geometry_msgs::msg::PoseStamped& pose,
  double max_distance) const
{
  std::shared_ptr<RouteWaypoint> nearest_waypoint = nullptr;
  double min_distance = max_distance;
  
  for (const auto& [name, waypoint] : waypoints_)
  {
    double dx = pose.pose.position.x - waypoint->x;
    double dy = pose.pose.position.y - waypoint->y;
    double distance = std::sqrt(dx*dx + dy*dy);
    
    if (distance < min_distance)
    {
      min_distance = distance;
      nearest_waypoint = waypoint;
    }
  }
  
  return nearest_waypoint;
}

const MapInfo& RouteManager::get_map_info() const
{
  return map_info_;
}

geometry_msgs::msg::Point RouteManager::map_to_world(double map_x, double map_y) const
{
  geometry_msgs::msg::Point world_point;
  
  if (map_loaded_)
  {
    world_point.x = map_info_.origin[0] + map_x * map_info_.resolution;
    world_point.y = map_info_.origin[1] + map_y * map_info_.resolution;
    world_point.z = 0.0;
  }
  else
  {
    world_point.x = map_x;
    world_point.y = map_y;
    world_point.z = 0.0;
  }
  
  return world_point;
}

std::pair<double, double> RouteManager::world_to_map(const geometry_msgs::msg::Point& world_point) const
{
  if (map_loaded_)
  {
    double map_x = (world_point.x - map_info_.origin[0]) / map_info_.resolution;
    double map_y = (world_point.y - map_info_.origin[1]) / map_info_.resolution;
    return {map_x, map_y};
  }
  else
  {
    return {world_point.x, world_point.y};
  }
}

bool RouteManager::validate_routes() const
{
  RCLCPP_INFO(node_->get_logger(), "Validating routes...");
  
  bool valid = true;
  
  // Validate waypoint connections
  if (!validate_waypoint_connections())
  {
    valid = false;
  }
  
  // Validate individual routes
  for (const auto& [id, route] : routes_)
  {
    if (!validate_route_waypoints(*route))
    {
      valid = false;
    }
  }
  
  RCLCPP_INFO(node_->get_logger(), "Route validation %s", valid ? "passed" : "failed");
  return valid;
}

std::string RouteManager::export_routes_to_json() const
{
  nlohmann::json json_data;
  
  // Export routes
  json_data["routes"] = nlohmann::json::array();
  for (const auto& [id, route] : routes_)
  {
    json_data["routes"].push_back(*route);
  }
  
  // Export waypoints
  json_data["waypoints"] = nlohmann::json::array();
  for (const auto& [name, waypoint] : waypoints_)
  {
    json_data["waypoints"].push_back(*waypoint);
  }
  
  return json_data.dump(2);
}

bool RouteManager::parse_json_routes(const nlohmann::json& json_data)
{
  try
  {
    // Clear existing data
    routes_.clear();
    waypoints_.clear();
    
    // Parse waypoints first
    if (json_data.contains("waypoints"))
    {
      for (const auto& wp_json : json_data["waypoints"])
      {
        RouteWaypoint waypoint = parse_waypoint(wp_json);
        waypoints_[waypoint.name] = std::make_shared<RouteWaypoint>(waypoint);
        
        RCLCPP_DEBUG(node_->get_logger(), "Parsed waypoint: %s [%.2f, %.2f]",
                     waypoint.name.c_str(), waypoint.x, waypoint.y);
      }
    }
    
    // Parse routes
    if (json_data.contains("routes"))
    {
      for (const auto& route_json : json_data["routes"])
      {
        Route route = parse_route(route_json);
        routes_[route.route_id] = std::make_shared<Route>(route);
        
        RCLCPP_DEBUG(node_->get_logger(), "Parsed route: %s (%s -> %s)",
                     route.name.c_str(), route.start_waypoint.c_str(), route.end_waypoint.c_str());
      }
    }
    
    RCLCPP_INFO(node_->get_logger(), "Loaded %zu waypoints and %zu routes",
                waypoints_.size(), routes_.size());
    
    return validate_routes();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(node_->get_logger(), "Exception parsing JSON routes: %s", e.what());
    return false;
  }
}

bool RouteManager::load_pgm_map(const std::string& pgm_file_path)
{
  try
  {
    // Load image using OpenCV
    cv::Mat map_image = cv::imread(pgm_file_path, cv::IMREAD_GRAYSCALE);
    
    if (map_image.empty())
    {
      RCLCPP_ERROR(node_->get_logger(), "Could not load PGM file: %s", pgm_file_path.c_str());
      return false;
    }
    
    // Convert to OccupancyGrid
    map_info_.occupancy_grid.header.frame_id = "map";
    map_info_.occupancy_grid.header.stamp = node_->get_clock()->now();
    
    map_info_.occupancy_grid.info.resolution = map_info_.resolution;
    map_info_.occupancy_grid.info.width = map_image.cols;
    map_info_.occupancy_grid.info.height = map_image.rows;
    
    map_info_.occupancy_grid.info.origin.position.x = map_info_.origin[0];
    map_info_.occupancy_grid.info.origin.position.y = map_info_.origin[1];
    map_info_.occupancy_grid.info.origin.position.z = 0.0;
    map_info_.occupancy_grid.info.origin.orientation.w = 1.0;
    
    // Convert image data to occupancy grid
    map_info_.occupancy_grid.data.resize(map_image.rows * map_image.cols);
    
    for (int y = 0; y < map_image.rows; ++y)
    {
      for (int x = 0; x < map_image.cols; ++x)
      {
        int index = (map_image.rows - 1 - y) * map_image.cols + x; // Flip Y axis
        uint8_t pixel = map_image.at<uint8_t>(y, x);
        
        // Convert pixel value to occupancy probability
        if (pixel > 250)
        {
          map_info_.occupancy_grid.data[index] = 0;    // Free space
        }
        else if (pixel < 5)
        {
          map_info_.occupancy_grid.data[index] = 100;  // Occupied
        }
        else
        {
          map_info_.occupancy_grid.data[index] = -1;   // Unknown
        }
      }
    }
    
    RCLCPP_INFO(node_->get_logger(), "Loaded PGM map: %dx%d, resolution: %.3f",
                map_image.cols, map_image.rows, map_info_.resolution);
    
    return true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(node_->get_logger(), "Exception loading PGM map: %s", e.what());
    return false;
  }
}

RouteWaypoint RouteManager::parse_waypoint(const nlohmann::json& waypoint_json)
{
  RouteWaypoint waypoint;
  
  waypoint.name = waypoint_json["name"].get<std::string>();
  waypoint.x = waypoint_json["x"].get<double>();
  waypoint.y = waypoint_json["y"].get<double>();
  waypoint.theta = waypoint_json.value("theta", 0.0);
  waypoint.map_name = waypoint_json.value("map_name", "map");
  
  if (waypoint_json.contains("connections"))
  {
    waypoint.connections = waypoint_json["connections"].get<std::vector<std::string>>();
  }
  
  return waypoint;
}

Route RouteManager::parse_route(const nlohmann::json& route_json)
{
  Route route;
  
  route.route_id = route_json["route_id"].get<std::string>();
  route.name = route_json["name"].get<std::string>();
  route.description = route_json.value("description", "");
  route.start_waypoint = route_json["start_waypoint"].get<std::string>();
  route.end_waypoint = route_json["end_waypoint"].get<std::string>();
  
  if (route_json.contains("waypoints"))
  {
    for (const auto& wp_json : route_json["waypoints"])
    {
      route.waypoints.push_back(parse_waypoint(wp_json));
    }
  }
  
  route.estimated_duration = calculate_route_duration(route);
  
  return route;
}

double RouteManager::calculate_route_duration(const Route& route) const
{
  // Simple estimation based on distance and average speed
  double total_distance = 0.0;
  const double average_speed = 0.5; // m/s
  
  if (route.waypoints.size() < 2)
  {
    return 30.0; // Default 30 seconds
  }
  
  for (size_t i = 0; i < route.waypoints.size() - 1; ++i)
  {
    const auto& wp1 = route.waypoints[i];
    const auto& wp2 = route.waypoints[i + 1];
    
    double dx = wp2.x - wp1.x;
    double dy = wp2.y - wp1.y;
    total_distance += std::sqrt(dx*dx + dy*dy);
  }
  
  return total_distance / average_speed;
}

void RouteManager::publish_map()
{
  if (map_loaded_ && map_pub_->get_subscription_count() > 0)
  {
    map_info_.occupancy_grid.header.stamp = node_->get_clock()->now();
    map_pub_->publish(map_info_.occupancy_grid);
  }
}

bool RouteManager::validate_waypoint_connections() const
{
  bool valid = true;
  
  for (const auto& [name, waypoint] : waypoints_)
  {
    for (const auto& connection : waypoint->connections)
    {
      if (waypoints_.find(connection) == waypoints_.end())
      {
        RCLCPP_ERROR(node_->get_logger(), 
                     "Waypoint '%s' has invalid connection to '%s'",
                     name.c_str(), connection.c_str());
        valid = false;
      }
    }
  }
  
  return valid;
}

bool RouteManager::validate_route_waypoints(const Route& route) const
{
  bool valid = true;
  
  // Check if start and end waypoints exist
  if (waypoints_.find(route.start_waypoint) == waypoints_.end())
  {
    RCLCPP_ERROR(node_->get_logger(), 
                 "Route '%s' has invalid start waypoint: '%s'",
                 route.name.c_str(), route.start_waypoint.c_str());
    valid = false;
  }
  
  if (waypoints_.find(route.end_waypoint) == waypoints_.end())
  {
    RCLCPP_ERROR(node_->get_logger(), 
                 "Route '%s' has invalid end waypoint: '%s'",
                 route.name.c_str(), route.end_waypoint.c_str());
    valid = false;
  }
  
  return valid;
}

// JSON serialization implementations
void to_json(nlohmann::json& j, const RouteWaypoint& waypoint)
{
  j = nlohmann::json{
    {"name", waypoint.name},
    {"x", waypoint.x},
    {"y", waypoint.y},
    {"theta", waypoint.theta},
    {"map_name", waypoint.map_name},
    {"connections", waypoint.connections}
  };
}

void from_json(const nlohmann::json& j, RouteWaypoint& waypoint)
{
  j.at("name").get_to(waypoint.name);
  j.at("x").get_to(waypoint.x);
  j.at("y").get_to(waypoint.y);
  waypoint.theta = j.value("theta", 0.0);
  waypoint.map_name = j.value("map_name", "map");
  
  if (j.contains("connections"))
  {
    j.at("connections").get_to(waypoint.connections);
  }
}

void to_json(nlohmann::json& j, const Route& route)
{
  j = nlohmann::json{
    {"route_id", route.route_id},
    {"name", route.name},
    {"description", route.description},
    {"waypoints", route.waypoints},
    {"start_waypoint", route.start_waypoint},
    {"end_waypoint", route.end_waypoint},
    {"estimated_duration", route.estimated_duration}
  };
}

void from_json(const nlohmann::json& j, Route& route)
{
  j.at("route_id").get_to(route.route_id);
  j.at("name").get_to(route.name);
  route.description = j.value("description", "");
  j.at("start_waypoint").get_to(route.start_waypoint);
  j.at("end_waypoint").get_to(route.end_waypoint);
  
  if (j.contains("waypoints"))
  {
    j.at("waypoints").get_to(route.waypoints);
  }
  
  route.estimated_duration = j.value("estimated_duration", 30.0);
}

} // namespace amr_fleet_adapter