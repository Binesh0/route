#include <rclcpp/rclcpp.hpp>
#include "amr_fleet_adapter/AMRFleetAdapter.hpp"
#include <signal.h>

static std::shared_ptr<amr_fleet_adapter::AMRFleetAdapter> g_adapter_node = nullptr;

void signal_handler(int signum)
{
  if (g_adapter_node)
  {
    RCLCPP_INFO(g_adapter_node->get_logger(), "Shutting down AMR Fleet Adapter...");
    g_adapter_node->stop();
    rclcpp::shutdown();
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  // Set up signal handling
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);
  
  try
  {
    // Create the AMR Fleet Adapter node
    auto adapter_options = rclcpp::NodeOptions();
    g_adapter_node = std::make_shared<amr_fleet_adapter::AMRFleetAdapter>(adapter_options);
    
    RCLCPP_INFO(g_adapter_node->get_logger(), "Starting AMR Fleet Adapter...");
    
    // Initialize the adapter
    if (!g_adapter_node->initialize())
    {
      RCLCPP_ERROR(g_adapter_node->get_logger(), "Failed to initialize AMR Fleet Adapter");
      return 1;
    }
    
    // Start the adapter
    g_adapter_node->start();
    
    RCLCPP_INFO(g_adapter_node->get_logger(), "AMR Fleet Adapter started successfully");
    
    // Spin the node
    rclcpp::spin(g_adapter_node);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("amr_fleet_adapter"), 
                 "Exception in AMR Fleet Adapter: %s", e.what());
    return 1;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("amr_fleet_adapter"), "AMR Fleet Adapter shutdown complete");
  return 0;
}