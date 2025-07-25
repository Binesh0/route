#ifndef AMR_FLEET_ADAPTER__TURTLEBOTROBOT_HPP
#define AMR_FLEET_ADAPTER__TURTLEBOTROBOT_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rmf_fleet_adapter/agv/RobotUpdateHandle.hpp>
#include <rmf_traffic/agv/Graph.hpp>
#include <rmf_traffic/Time.hpp>

#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <chrono>
#include <mutex>

namespace amr_fleet_adapter
{

struct RobotState
{
  geometry_msgs::msg::PoseStamped pose;
  geometry_msgs::msg::Twist velocity;
  double battery_level = 100.0;
  bool is_charging = false;
  bool is_emergency_stopped = false;
  std::chrono::steady_clock::time_point timestamp;
};

struct NavigationGoal
{
  geometry_msgs::msg::PoseStamped target_pose;
  std::size_t graph_index;
  std::string waypoint_name;
  std::function<void(bool)> completion_callback;
};

class TurtleBotRobot
{
public:
  using NavigateAction = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateAction>;

  explicit TurtleBotRobot(
    const std::string& robot_name,
    rclcpp::Node::SharedPtr node,
    const rmf_traffic::agv::Graph& nav_graph);

  ~TurtleBotRobot();

  /// Initialize the robot connections and subscribers
  bool initialize();

  /// Get current robot state
  const RobotState& get_current_state() const;

  /// Navigate to a specific pose
  void navigate_to_pose(
    const geometry_msgs::msg::PoseStamped& goal_pose,
    std::function<void(bool)> completion_callback = nullptr);

  /// Navigate to a graph waypoint
  void navigate_to_waypoint(
    std::size_t waypoint_index,
    std::function<void(bool)> completion_callback = nullptr);

  /// Stop current navigation
  void stop_navigation();

  /// Check if robot is currently navigating
  bool is_navigating() const;

  /// Get robot name
  const std::string& get_name() const;

  /// Set RMF robot update handle
  void set_rmf_handle(rmf_fleet_adapter::agv::RobotUpdateHandlePtr handle);

  /// Update RMF with current robot state
  void update_rmf_state();

  /// Emergency stop
  void emergency_stop();

  /// Resume from emergency stop
  void resume();

private:
  // Robot identification
  std::string robot_name_;
  
  // ROS2 node
  rclcpp::Node::SharedPtr node_;
  
  // Navigation graph
  const rmf_traffic::agv::Graph& nav_graph_;
  
  // Current robot state
  RobotState current_state_;
  mutable std::mutex state_mutex_;
  
  // RMF handle
  rmf_fleet_adapter::agv::RobotUpdateHandlePtr rmf_handle_;
  
  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Navigation
  rclcpp_action::Client<NavigateAction>::SharedPtr nav_action_client_;
  std::shared_ptr<GoalHandleNavigate> current_goal_handle_;
  NavigationGoal current_navigation_goal_;
  bool is_navigating_;
  
  // Publishers and Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  
  // Timers
  rclcpp::TimerBase::SharedPtr state_update_timer_;
  
  // Callbacks
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void navigation_goal_response_callback(const GoalHandleNavigate::SharedPtr& goal_handle);
  void navigation_feedback_callback(
    GoalHandleNavigate::SharedPtr goal_handle,
    const std::shared_ptr<const NavigateAction::Feedback> feedback);
  void navigation_result_callback(const GoalHandleNavigate::WrappedResult& result);
  void state_update_callback();
  
  // Helper methods
  geometry_msgs::msg::PoseStamped get_current_pose();
  bool update_robot_position();
  std::size_t find_nearest_waypoint(const geometry_msgs::msg::PoseStamped& pose);
  double calculate_distance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2);
};

} // namespace amr_fleet_adapter

#endif // AMR_FLEET_ADAPTER__TURTLEBOTROBOT_HPP