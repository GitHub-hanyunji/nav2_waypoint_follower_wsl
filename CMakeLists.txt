cmake_minimum_required(VERSION 3.8)
project(nav2_waypoint_follower_wsl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(waypoint_publisher src/waypoint_publisher.cpp)
add_executable(singlePoint_publisher src/singlePoint_publisher.cpp)

ament_target_dependencies(waypoint_publisher
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  tf2
  tf2_geometry_msgs
)


ament_target_dependencies(singlePoint_publisher
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS
  waypoint_publisher
  singlePoint_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
