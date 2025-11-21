cmake_minimum_required(VERSION 3.10)
project(laser_scan_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Subscriber node
add_executable(scan_subscriber_node src/scan_subscriber_node.cpp)
ament_target_dependencies(scan_subscriber_node
  rclcpp
  sensor_msgs
  std_msgs
)

# Publisher node
add_executable(scan_publisher_node src/scan_publisher_node.cpp)
ament_target_dependencies(scan_publisher_node
  rclcpp
  sensor_msgs
  std_msgs
)

# Install executables
install(TARGETS
  scan_subscriber_node
  scan_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()