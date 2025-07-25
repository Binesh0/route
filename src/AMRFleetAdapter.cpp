#include "amr_fleet_adapter/AMRFleetAdapter.hpp"
#include <rmf_fleet_adapter/StandardNames.hpp>
#include <rmf_fleet_adapter/agv/parse_graph.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>
#include <rmf_utils/optional.hpp>

#include <fstream>
#include <yaml-cpp/yaml.h>

namespace amr_fleet_adapter
{

AMRFleetAdapter::AMRFleetAdapter(const rclcpp::NodeOptions& options)
: Node("amr_fleet_adapter", options)
{
  RCLCPP_INFO(get_logger(), "Creating AMR Fleet Adapter node");
  
  // Load configuration from parameters
  load_configuration();
  
  // Create route manager
  route_manager_ = std::make_shared<RouteManager>(shared_from_this());
}

AMRFleetAdapter::~AMRFleetAdapter()
{
  stop();
}

bool AMRFleetAdapter::initialize()
{
  RCLCPP_INFO(get_logger(), "Initializing AMR Fleet Adapter...");
  
  try
  {
    // Initialize route manager
    if (!route_manager_->initialize())
    {
      RCLCPP_ERROR(get_logger(), "Failed to initialize route manager");
      return false;
    }
    
    // Load navigation graph
    if (!load_nav_graph())
    {
      RCLCPP_ERROR(get_logger(), "Failed to load navigation graph");
      return false;
    }
    
    // Load vehicle traits
    if (!load_vehicle_traits())
    {
      RCLCPP_ERROR(get_logger(), "Failed to load vehicle traits");
      return false;
    }
    
    // Create RMF adapter
    adapter_ = rmf_fleet_adapter::agv::Adapter::make(
      rmf_fleet_adapter::get_client_id(*this, "fleet_name"),
      nav_graph_,
      vehicle_traits_,
      shared_from_this());
    
    if (!adapter_)
    {
      RCLCPP_ERROR(get_logger(), "Failed to create RMF adapter");
      return false;
    }
    
    // Setup fleet
    setup_fleet();
    
    RCLCPP_INFO(get_logger(), "AMR Fleet Adapter initialized successfully");
    return true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "Exception during initialization: %s", e.what());
    return false;
  }
}

void AMRFleetAdapter::start()
{
  RCLCPP_INFO(get_logger(), "Starting AMR Fleet Adapter...");
  
  // Start adapter
  adapter_->start();
  
  // Discover robots
  discover_robots();
  
  // Start periodic update timer
  update_timer_ = create_wall_timer(
    std::chrono::milliseconds(1000),
    [this]() { periodic_update(); });
  
  RCLCPP_INFO(get_logger(), "AMR Fleet Adapter started");
}

void AMRFleetAdapter::stop()
{
  RCLCPP_INFO(get_logger(), "Stopping AMR Fleet Adapter...");
  
  // Stop timer
  if (update_timer_)
  {
    update_timer_->cancel();
    update_timer_.reset();
  }
  
  // Stop all robots
  for (auto& [name, robot] : robots_)
  {
    robot->stop_navigation();
  }
  
  // Stop adapter
  if (adapter_)
  {
    adapter_->stop();
  }
  
  RCLCPP_INFO(get_logger(), "AMR Fleet Adapter stopped");
}

void AMRFleetAdapter::load_configuration()
{
  // Declare parameters with default values
  declare_parameter("fleet_name", "turtlebot_fleet");
  declare_parameter("nav_graph_file", "");
  declare_parameter("robot_traits_file", "");
  declare_parameter("robot_names", std::vector<std::string>{"turtlebot1"});
  declare_parameter("discovery_timeout", 60.0);
  declare_parameter("task_capabilities_timeout", 30.0);
  declare_parameter("perform_deliveries", true);
  declare_parameter("perform_cleaning", false);
  declare_parameter("accept_patrol_requests", true);
  declare_parameter("route_json_file", "");
  declare_parameter("map_yaml_file", "");
  
  // Get parameters
  config_.fleet_name = get_parameter("fleet_name").as_string();
  config_.nav_graph_file = get_parameter("nav_graph_file").as_string();
  config_.robot_traits_file = get_parameter("robot_traits_file").as_string();
  config_.robot_names = get_parameter("robot_names").as_string_array();
  config_.discovery_timeout = get_parameter("discovery_timeout").as_double();
  config_.task_capabilities_timeout = get_parameter("task_capabilities_timeout").as_double();
  config_.perform_deliveries = get_parameter("perform_deliveries").as_bool();
  config_.perform_cleaning = get_parameter("perform_cleaning").as_bool();
  config_.accept_patrol_requests = get_parameter("accept_patrol_requests").as_bool();
  
  // Load route and map files
  auto route_json_file = get_parameter("route_json_file").as_string();
  auto map_yaml_file = get_parameter("map_yaml_file").as_string();
  
  if (!route_json_file.empty())
  {
    route_manager_->load_routes_from_json(route_json_file);
  }
  
  if (!map_yaml_file.empty())
  {
    route_manager_->load_map_info(map_yaml_file);
  }
  
  RCLCPP_INFO(get_logger(), "Configuration loaded - Fleet: %s, Robots: %zu",
              config_.fleet_name.c_str(), config_.robot_names.size());
}

bool AMRFleetAdapter::load_nav_graph()
{
  if (!config_.nav_graph_file.empty())
  {
    // Load from file
    try
    {
      nav_graph_ = rmf_fleet_adapter::agv::parse_graph(config_.nav_graph_file, vehicle_traits_);
      RCLCPP_INFO(get_logger(), "Loaded navigation graph from file: %s", 
                  config_.nav_graph_file.c_str());
      return true;
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Failed to load nav graph from file: %s", e.what());
    }
  }
  
  // Create from route manager
  try
  {
    nav_graph_ = route_manager_->create_rmf_graph();
    RCLCPP_INFO(get_logger(), "Created navigation graph from route manager");
    return true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "Failed to create nav graph from routes: %s", e.what());
    return false;
  }
}

bool AMRFleetAdapter::load_vehicle_traits()
{
  if (!config_.robot_traits_file.empty())
  {
    // Load from file
    try
    {
      YAML::Node config = YAML::LoadFile(config_.robot_traits_file);
      
      auto linear = config["linear"];
      auto angular = config["angular"];
      auto footprint = config["footprint"];
      auto vicinity = config["vicinity"];
      
      rmf_traffic::agv::VehicleTraits::Limits linear_limits(
        linear["nominal_velocity"].as<double>(),
        linear["nominal_acceleration"].as<double>());
      
      rmf_traffic::agv::VehicleTraits::Limits angular_limits(
        angular["nominal_velocity"].as<double>(),
        angular["nominal_acceleration"].as<double>());
      
      rmf_traffic::Profile profile{
        rmf_traffic::geometry::make_final_convex<
          rmf_traffic::geometry::Circle>(footprint["radius"].as<double>())};
      
      vehicle_traits_ = rmf_traffic::agv::VehicleTraits{
        linear_limits, angular_limits, profile, vicinity.as<double>()};
      
      RCLCPP_INFO(get_logger(), "Loaded vehicle traits from file: %s",
                  config_.robot_traits_file.c_str());
      return true;
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Failed to load vehicle traits: %s", e.what());
    }
  }
  
  // Use default traits for TurtleBot
  rmf_traffic::agv::VehicleTraits::Limits linear_limits(0.5, 0.75);
  rmf_traffic::agv::VehicleTraits::Limits angular_limits(1.0, 2.0);
  rmf_traffic::Profile profile{
    rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(0.3)};
  
  vehicle_traits_ = rmf_traffic::agv::VehicleTraits{
    linear_limits, angular_limits, profile, 1.0};
  
  RCLCPP_INFO(get_logger(), "Using default TurtleBot vehicle traits");
  return true;
}

void AMRFleetAdapter::setup_fleet()
{
  auto fleet_handle = adapter_->add_fleet(
    config_.fleet_name,
    vehicle_traits_,
    nav_graph_);
  
  if (!fleet_handle)
  {
    throw std::runtime_error("Failed to create fleet handle");
  }
  
  fleet_handle_ = fleet_handle;
  
  // Set task capabilities
  if (config_.perform_deliveries)
  {
    fleet_handle_->consider_delivery_requests(
      [this](const rmf_fleet_adapter::agv::FleetUpdateHandle::TaskRequestPtr request)
      {
        handle_task_request(request);
      });
  }
  
  if (config_.accept_patrol_requests)
  {
    fleet_handle_->consider_patrol_requests(
      [this](const rmf_fleet_adapter::agv::FleetUpdateHandle::TaskRequestPtr request)
      {
        handle_task_request(request);
      });
  }
  
  RCLCPP_INFO(get_logger(), "Fleet '%s' setup complete", config_.fleet_name.c_str());
}

void AMRFleetAdapter::discover_robots()
{
  RCLCPP_INFO(get_logger(), "Discovering robots...");
  
  for (const auto& robot_name : config_.robot_names)
  {
    add_robot(robot_name);
  }
  
  RCLCPP_INFO(get_logger(), "Robot discovery complete. Found %zu robots", robots_.size());
}

void AMRFleetAdapter::add_robot(const std::string& robot_name)
{
  RCLCPP_INFO(get_logger(), "Adding robot: %s", robot_name.c_str());
  
  try
  {
    // Create robot instance
    auto robot = std::make_shared<TurtleBotRobot>(robot_name, shared_from_this(), nav_graph_);
    
    if (!robot->initialize())
    {
      RCLCPP_ERROR(get_logger(), "Failed to initialize robot: %s", robot_name.c_str());
      return;
    }
    
    // Create RMF robot handle
    auto robot_handle = fleet_handle_->add_robot(
      robot,
      robot_name,
      rmf_traffic::Profile{vehicle_traits_.profile()},
      [this, robot_name](rmf_fleet_adapter::agv::RobotUpdateHandlePtr handle)
      {
        RCLCPP_INFO(get_logger(), "Robot handle created for: %s", robot_name.c_str());
        robot_handles_[robot_name] = handle;
        robots_[robot_name]->set_rmf_handle(handle);
      },
      [this, robot_name](rmf_fleet_adapter::agv::RobotUpdateHandlePtr)
      {
        RCLCPP_ERROR(get_logger(), "Failed to create robot handle for: %s", robot_name.c_str());
      });
    
    robots_[robot_name] = robot;
    
    RCLCPP_INFO(get_logger(), "Robot '%s' added successfully", robot_name.c_str());
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "Exception adding robot '%s': %s", robot_name.c_str(), e.what());
  }
}

void AMRFleetAdapter::periodic_update()
{
  // Update all robot states
  for (auto& [name, robot] : robots_)
  {
    robot->update_rmf_state();
  }
}

void AMRFleetAdapter::handle_task_request(
  const rmf_fleet_adapter::agv::FleetUpdateHandle::TaskRequestPtr request)
{
  RCLCPP_INFO(get_logger(), "Received task request: %s", request->description().c_str());
  
  // For now, accept all requests - more sophisticated logic can be added
  request->accept();
}

void AMRFleetAdapter::handle_robot_state_update(
  const std::string& robot_name,
  const rmf_fleet_adapter::agv::RobotUpdateHandle::Unstable::State& state)
{
  // Handle robot state updates if needed
  RCLCPP_DEBUG(get_logger(), "Robot state update for: %s", robot_name.c_str());
}

} // namespace amr_fleet_adapter