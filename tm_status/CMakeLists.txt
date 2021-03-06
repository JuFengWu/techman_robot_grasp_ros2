cmake_minimum_required(VERSION 3.5)

project(tm_status)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


add_executable(send_joint_command src/send_joint_command.cpp  src/model.cpp  src/command_source.cpp)
ament_target_dependencies(send_joint_command
  "rclcpp"
  "sensor_msgs")

  add_executable(get_trajectory src/get_trajectory.cpp src/moveit_bridge.cpp)
  ament_target_dependencies(get_trajectory
    "rclcpp"
    "sensor_msgs"
    "trajectory_msgs"
    "geometry_msgs") 
  
install(TARGETS
  send_joint_command
  get_trajectory
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
