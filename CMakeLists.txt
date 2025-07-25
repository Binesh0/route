cmake_minimum_required(VERSION 3.8)
project(amr_fleet_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rmf_fleet_adapter REQUIRED)
find_package(rmf_traffic REQUIRED)
find_package(rmf_traffic_ros2 REQUIRED)
find_package(rmf_utils REQUIRED)
find_package(rmf_robot_sim_common REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(include)

# AMR Fleet Adapter Library
add_library(amr_fleet_adapter_lib SHARED
  src/AMRFleetAdapter.cpp
  src/TurtleBotRobot.cpp
  src/RouteManager.cpp
)

target_include_directories(amr_fleet_adapter_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(amr_fleet_adapter_lib PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(amr_fleet_adapter_lib
  rclcpp
  rclcpp_components
  rmf_fleet_adapter
  rmf_traffic
  rmf_traffic_ros2
  rmf_utils
  rmf_robot_sim_common
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  nav2_util
  nav2_costmap_2d
)

target_link_libraries(amr_fleet_adapter_lib 
  nlohmann_json::nlohmann_json
  yaml-cpp
  ${OpenCV_LIBS}
)

# Main executable
add_executable(amr_fleet_adapter_node
  src/main.cpp
)

target_link_libraries(amr_fleet_adapter_node amr_fleet_adapter_lib)

ament_target_dependencies(amr_fleet_adapter_node
  rclcpp
)

# Install targets
install(TARGETS amr_fleet_adapter_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS amr_fleet_adapter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  maps
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(amr_fleet_adapter_lib)
ament_export_targets(export_${PROJECT_NAME})

ament_package()