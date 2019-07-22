cmake_minimum_required(VERSION 3.5)
project(joint_trajectory_action_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# add_compile_options(-Wall -Wextra -Wpedantic -Werror=ignored-qualifiers)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)

include_directories(include)

add_executable(follow_joint_trajectory_node
  src/main_follow_joint_trajectory_action.cpp
  src/follow_joint_trajectory_action.cpp
)
ament_target_dependencies(follow_joint_trajectory_node
  rclcpp
  rclcpp_action
  control_msgs
)

install(TARGETS
  follow_joint_trajectory_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()