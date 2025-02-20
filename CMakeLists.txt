cmake_minimum_required(VERSION 3.5)
project(nav2_task)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(nav_client_node src/nav_client_node.cpp)
ament_target_dependencies(nav_client_node
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_executable(multi_client_node src/multipoint_nav_client.cpp)
ament_target_dependencies(multi_client_node
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS
  nav_client_node
  multi_client_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()