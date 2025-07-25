#include "amr_fleet_adapter/TurtleBotRobot.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rmf_traffic/Time.hpp>

namespace amr_fleet_adapter
{

TurtleBotRobot::TurtleBotRobot(
  const std::string& robot_name,
  rclcpp::Node::SharedPtr node,
  const rmf_traffic::agv::Graph& nav_graph)
: robot_name_(robot_name)
, node_(node)
, nav_graph_(nav_graph)
, is_navigating_(false)
{
  RCLCPP_INFO(node_->get_logger(), "Creating TurtleBot robot: %s", robot_name_.c_str());
  
  // Initialize TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  // Initialize current state
  current_state_.timestamp = std::chrono::steady_clock::now();
  current_state_.pose.header.frame_id = "map";
  current_state_.battery_level = 100.0;
}

TurtleBotRobot::~TurtleBotRobot()
{
  stop_navigation();
}

bool TurtleBotRobot::initialize()
{
  RCLCPP_INFO(node_->get_logger(), "Initializing robot: %s", robot_name_.c_str());
  
  try
  {
    // Create navigation action client
    std::string nav_action_name = "/" + robot_name_ + "/navigate_to_pose";
    nav_action_client_ = rclcpp_action::create_client<NavigateAction>(
      node_, nav_action_name);
    
    // Create publishers
    std::string cmd_vel_topic = "/" + robot_name_ + "/cmd_vel";
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_topic, 10);
    
    std::string initial_pose_topic = "/" + robot_name_ + "/initialpose";
    initial_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initial_pose_topic, 1);
    
    // Create subscribers
    std::string odom_topic = "/" + robot_name_ + "/odom";
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        odometry_callback(msg);
      });
    
    std::string amcl_pose_topic = "/" + robot_name_ + "/amcl_pose";
    amcl_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      amcl_pose_topic, 10,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        amcl_pose_callback(msg);
      });
    
    // Create state update timer
    state_update_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { state_update_callback(); });
    
    // Wait for navigation action server
    RCLCPP_INFO(node_->get_logger(), "Waiting for navigation action server: %s", 
                nav_action_name.c_str());
    
    if (!nav_action_client_->wait_for_action_server(std::chrono::seconds(10)))
    {
      RCLCPP_ERROR(node_->get_logger(), 
                   "Navigation action server not available for robot: %s", robot_name_.c_str());
      return false;
    }
    
    RCLCPP_INFO(node_->get_logger(), "Robot '%s' initialized successfully", robot_name_.c_str());
    return true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(node_->get_logger(), "Exception initializing robot '%s': %s", 
                 robot_name_.c_str(), e.what());
    return false;
  }
}

const RobotState& TurtleBotRobot::get_current_state() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return current_state_;
}

void TurtleBotRobot::navigate_to_pose(
  const geometry_msgs::msg::PoseStamped& goal_pose,
  std::function<void(bool)> completion_callback)
{
  RCLCPP_INFO(node_->get_logger(), "Robot '%s' navigating to pose [%.2f, %.2f]",
              robot_name_.c_str(), goal_pose.pose.position.x, goal_pose.pose.position.y);
  
  if (is_navigating_)
  {
    RCLCPP_WARN(node_->get_logger(), "Robot '%s' is already navigating, canceling current goal",
                robot_name_.c_str());
    stop_navigation();
  }
  
  // Prepare navigation goal
  auto nav_goal = NavigateAction::Goal();
  nav_goal.pose = goal_pose;
  nav_goal.pose.header.stamp = node_->get_clock()->now();
  
  // Set current navigation goal
  current_navigation_goal_.target_pose = goal_pose;
  current_navigation_goal_.completion_callback = completion_callback;
  
  // Send goal
  auto send_goal_options = rclcpp_action::Client<NavigateAction>::SendGoalOptions();
  send_goal_options.goal_response_callback = 
    [this](std::shared_ptr<GoalHandleNavigate> goal_handle) {
      navigation_goal_response_callback(goal_handle);
    };
  
  send_goal_options.feedback_callback =
    [this](GoalHandleNavigate::SharedPtr goal_handle,
            const std::shared_ptr<const NavigateAction::Feedback> feedback) {
      navigation_feedback_callback(goal_handle, feedback);
    };
  
  send_goal_options.result_callback =
    [this](const GoalHandleNavigate::WrappedResult& result) {
      navigation_result_callback(result);
    };
  
  nav_action_client_->async_send_goal(nav_goal, send_goal_options);
  is_navigating_ = true;
}

void TurtleBotRobot::navigate_to_waypoint(
  std::size_t waypoint_index,
  std::function<void(bool)> completion_callback)
{
  if (waypoint_index >= nav_graph_.num_waypoints())
  {
    RCLCPP_ERROR(node_->get_logger(), "Invalid waypoint index: %zu", waypoint_index);
    if (completion_callback)
      completion_callback(false);
    return;
  }
  
  const auto& waypoint = nav_graph_.get_waypoint(waypoint_index);
  
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.header.stamp = node_->get_clock()->now();
  
  // Convert waypoint location to pose
  goal_pose.pose.position.x = waypoint.get_location()[0];
  goal_pose.pose.position.y = waypoint.get_location()[1];
  goal_pose.pose.position.z = 0.0;
  
  // Set orientation (assume 0 if not specified)
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  goal_pose.pose.orientation = tf2::toMsg(q);
  
  current_navigation_goal_.graph_index = waypoint_index;
  current_navigation_goal_.waypoint_name = waypoint.get_name();
  
  navigate_to_pose(goal_pose, completion_callback);
}

void TurtleBotRobot::stop_navigation()
{
  RCLCPP_INFO(node_->get_logger(), "Stopping navigation for robot: %s", robot_name_.c_str());
  
  if (current_goal_handle_)
  {
    nav_action_client_->async_cancel_goal(current_goal_handle_);
    current_goal_handle_.reset();
  }
  
  // Send zero velocity command
  auto stop_cmd = geometry_msgs::msg::Twist();
  cmd_vel_pub_->publish(stop_cmd);
  
  is_navigating_ = false;
}

bool TurtleBotRobot::is_navigating() const
{
  return is_navigating_;
}

const std::string& TurtleBotRobot::get_name() const
{
  return robot_name_;
}

void TurtleBotRobot::set_rmf_handle(rmf_fleet_adapter::agv::RobotUpdateHandlePtr handle)
{
  rmf_handle_ = handle;
  RCLCPP_INFO(node_->get_logger(), "RMF handle set for robot: %s", robot_name_.c_str());
}

void TurtleBotRobot::update_rmf_state()
{
  if (!rmf_handle_)
    return;
  
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  // Update position
  std::array<double, 3> position = {
    current_state_.pose.pose.position.x,
    current_state_.pose.pose.position.y,
    0.0  // Assuming 2D navigation
  };
  
  // Convert quaternion to yaw
  tf2::Quaternion q;
  tf2::fromMsg(current_state_.pose.pose.orientation, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  
  // Update RMF with current state
  rmf_handle_->update_position(position, yaw);
  
  // Update battery level
  rmf_handle_->update_battery_soc(current_state_.battery_level / 100.0);
}

void TurtleBotRobot::emergency_stop()
{
  RCLCPP_WARN(node_->get_logger(), "Emergency stop for robot: %s", robot_name_.c_str());
  
  stop_navigation();
  
  std::lock_guard<std::mutex> lock(state_mutex_);
  current_state_.is_emergency_stopped = true;
}

void TurtleBotRobot::resume()
{
  RCLCPP_INFO(node_->get_logger(), "Resuming robot: %s", robot_name_.c_str());
  
  std::lock_guard<std::mutex> lock(state_mutex_);
  current_state_.is_emergency_stopped = false;
}

void TurtleBotRobot::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  // Update velocity
  current_state_.velocity = msg->twist.twist;
  current_state_.timestamp = std::chrono::steady_clock::now();
  
  // If we don't have AMCL pose, use odometry for position
  if (current_state_.pose.header.stamp.sec == 0)
  {
    current_state_.pose.header = msg->header;
    current_state_.pose.pose = msg->pose.pose;
  }
}

void TurtleBotRobot::amcl_pose_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  // Update pose from AMCL (more accurate than odometry)
  current_state_.pose.header = msg->header;
  current_state_.pose.pose = msg->pose.pose;
  current_state_.timestamp = std::chrono::steady_clock::now();
}

void TurtleBotRobot::navigation_goal_response_callback(
  const GoalHandleNavigate::SharedPtr& goal_handle)
{
  if (!goal_handle)
  {
    RCLCPP_ERROR(node_->get_logger(), "Navigation goal was rejected for robot: %s", 
                 robot_name_.c_str());
    is_navigating_ = false;
    if (current_navigation_goal_.completion_callback)
      current_navigation_goal_.completion_callback(false);
    return;
  }
  
  RCLCPP_INFO(node_->get_logger(), "Navigation goal accepted for robot: %s", robot_name_.c_str());
  current_goal_handle_ = goal_handle;
}

void TurtleBotRobot::navigation_feedback_callback(
  GoalHandleNavigate::SharedPtr goal_handle,
  const std::shared_ptr<const NavigateAction::Feedback> feedback)
{
  // Update navigation progress if needed
  RCLCPP_DEBUG(node_->get_logger(), "Navigation feedback for robot '%s': distance remaining: %.2f",
               robot_name_.c_str(), feedback->distance_remaining);
}

void TurtleBotRobot::navigation_result_callback(const GoalHandleNavigate::WrappedResult& result)
{
  current_goal_handle_.reset();
  is_navigating_ = false;
  
  bool success = false;
  
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(), "Navigation succeeded for robot: %s", robot_name_.c_str());
      success = true;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(node_->get_logger(), "Navigation aborted for robot: %s", robot_name_.c_str());
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(node_->get_logger(), "Navigation canceled for robot: %s", robot_name_.c_str());
      break;
    default:
      RCLCPP_ERROR(node_->get_logger(), "Navigation failed for robot: %s", robot_name_.c_str());
      break;
  }
  
  if (current_navigation_goal_.completion_callback)
  {
    current_navigation_goal_.completion_callback(success);
  }
}

void TurtleBotRobot::state_update_callback()
{
  // Update robot position using TF if available
  update_robot_position();
  
  // Simulate battery drain (in real implementation, this would come from robot)
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (current_state_.battery_level > 0.0 && !current_state_.is_charging)
  {
    current_state_.battery_level -= 0.001; // Very slow drain for simulation
    current_state_.battery_level = std::max(0.0, current_state_.battery_level);
  }
}

geometry_msgs::msg::PoseStamped TurtleBotRobot::get_current_pose()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return current_state_.pose;
}

bool TurtleBotRobot::update_robot_position()
{
  try
  {
    // Try to get robot position from TF
    std::string robot_frame = robot_name_ + "/base_link";
    auto transform = tf_buffer_->lookupTransform("map", robot_frame, tf2::TimePointZero);
    
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_state_.pose.header.stamp = transform.header.stamp;
    current_state_.pose.header.frame_id = "map";
    current_state_.pose.pose.position.x = transform.transform.translation.x;
    current_state_.pose.pose.position.y = transform.transform.translation.y;
    current_state_.pose.pose.position.z = transform.transform.translation.z;
    current_state_.pose.pose.orientation = transform.transform.rotation;
    
    return true;
  }
  catch (const tf2::TransformException& ex)
  {
    // TF not available, keep using last known position
    return false;
  }
}

std::size_t TurtleBotRobot::find_nearest_waypoint(const geometry_msgs::msg::PoseStamped& pose)
{
  double min_distance = std::numeric_limits<double>::max();
  std::size_t nearest_index = 0;
  
  for (std::size_t i = 0; i < nav_graph_.num_waypoints(); ++i)
  {
    const auto& waypoint = nav_graph_.get_waypoint(i);
    const auto& wp_location = waypoint.get_location();
    
    double distance = calculate_distance(
      pose.pose.position,
      {wp_location[0], wp_location[1], 0.0});
    
    if (distance < min_distance)
    {
      min_distance = distance;
      nearest_index = i;
    }
  }
  
  return nearest_index;
}

double TurtleBotRobot::calculate_distance(
  const geometry_msgs::msg::Point& p1,
  const geometry_msgs::msg::Point& p2)
{
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  double dz = p1.z - p2.z;
  return std::sqrt(dx*dx + dy*dy + dz*dz);
}

} // namespace amr_fleet_adapter