#ifndef AMR_FLEET_ADAPTER__AMRFLEETADAPTER_HPP
#define AMR_FLEET_ADAPTER__AMRFLEETADAPTER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rmf_fleet_adapter/agv/Adapter.hpp>
#include <rmf_fleet_adapter/agv/FleetUpdateHandle.hpp>
#include <rmf_fleet_adapter/agv/RobotUpdateHandle.hpp>
#include <rmf_traffic/agv/Graph.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>
#include <rmf_traffic_ros2/StandardNames.hpp>

#include "TurtleBotRobot.hpp"
#include "RouteManager.hpp"

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

namespace amr_fleet_adapter
{

struct FleetConfiguration
{
  std::string fleet_name;
  std::string nav_graph_file;
  std::string robot_traits_file;
  std::vector<std::string> robot_names;
  double discovery_timeout = 60.0;
  double task_capabilities_timeout = 30.0;
  bool perform_deliveries = true;
  bool perform_cleaning = false;
  bool accept_patrol_requests = true;
};

class AMRFleetAdapter : public rclcpp::Node
{
public:
  explicit AMRFleetAdapter(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~AMRFleetAdapter();

  /// Initialize the fleet adapter with configuration
  bool initialize();

  /// Start the fleet adapter
  void start();

  /// Stop the fleet adapter
  void stop();

private:
  // Configuration
  FleetConfiguration config_;
  
  // RMF Components
  rmf_fleet_adapter::agv::AdapterPtr adapter_;
  rmf_fleet_adapter::agv::FleetUpdateHandlePtr fleet_handle_;
  
  // Traffic graph and vehicle traits
  rmf_traffic::agv::Graph nav_graph_;
  rmf_traffic::agv::VehicleTraits vehicle_traits_;
  
  // Robot management
  std::unordered_map<std::string, std::shared_ptr<TurtleBotRobot>> robots_;
  std::unordered_map<std::string, rmf_fleet_adapter::agv::RobotUpdateHandlePtr> robot_handles_;
  
  // Route management
  std::shared_ptr<RouteManager> route_manager_;
  
  // Timer for periodic updates
  rclcpp::TimerBase::SharedPtr update_timer_;
  
  // Methods
  void load_configuration();
  bool load_nav_graph();
  bool load_vehicle_traits();
  void setup_fleet();
  void discover_robots();
  void add_robot(const std::string& robot_name);
  void periodic_update();
  
  // RMF Callbacks
  void handle_task_request(
    const rmf_fleet_adapter::agv::FleetUpdateHandle::TaskRequestPtr request);
    
  void handle_robot_state_update(
    const std::string& robot_name,
    const rmf_fleet_adapter::agv::RobotUpdateHandle::Unstable::State& state);
};

} // namespace amr_fleet_adapter

#endif // AMR_FLEET_ADAPTER__AMRFLEETADAPTER_HPP