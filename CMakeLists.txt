cmake_minimum_required(VERSION 3.8)
project(amr_fleet_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rmf_fleet_adapter REQUIRED)
find_package(rmf_fleet_adapter_python REQUIRED)
find_package(rmf_traffic REQUIRED)
find_package(rmf_traffic_ros2 REQUIRED)
find_package(rmf_utils REQUIRED)
find_package(rmf_task_msgs REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_simple_commander REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)

# Python module and scripts install
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/amr_fleet_adapter_node.py
  scripts/delivery_manager_node.py
  scripts/robot_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  config
  launch
  maps
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python modules
install(DIRECTORY ${PROJECT_NAME}/
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_copyright REQUIRED)
  find_package(ament_flake8 REQUIRED)
  find_package(ament_pep257 REQUIRED)
  
  ament_lint_auto_find_test_dependencies()
  
  # Python tests
  find_package(python3-pytest REQUIRED)
  ament_add_pytest_test(${PROJECT_NAME}_tests test/)
endif()

ament_package()