cmake_minimum_required(VERSION 3.8)
project(cube_pointer_controller)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp_action        REQUIRED)
find_package(control_msgs         REQUIRED)


add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node
  rclcpp
  rclcpp_action
  control_msgs
  moveit_ros_planning_interface
  tf2_ros
  std_srvs
  geometry_msgs
  )


# Install the controller node executable
install(TARGETS controller_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files and configuration
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})


ament_package()
