cmake_minimum_required(VERSION 3.5)
project(kuka_rsi_hw_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rcutils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tinyxml_vendor REQUIRED)

add_library(kuka_rsi_hardware SHARED src/kuka_rsi_hardware.cpp)
target_include_directories(kuka_rsi_hardware PRIVATE include ${TinyXML_INCLUDE_DIRS})
target_link_libraries(kuka_rsi_hardware ${TinyXML_LIBRARIES})
ament_target_dependencies(
  kuka_rsi_hardware
  hardware_interface
  rcutils
)

add_executable(kuka_rsi_hardware_node src/kuka_rsi_hardware_node.cpp)
target_include_directories(kuka_rsi_hardware_node PRIVATE include &{TinyXML_INCLUDE_DIRS})
target_link_libraries(kuka_rsi_hardware_node kuka_rsi_hardware ${TinyXML_LIBRARIES})
ament_target_dependencies(
  kuka_rsi_hardware_node
  controller_interface
  controller_manager
  rcutils
  rclcpp
  TinyXML
)


# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(kuka_rsi_hardware PRIVATE "KUKA_RSI_HARDWARE_BUILDING_DLL")

install(DIRECTORY include/
  DESTINATION include)

install(TARGETS kuka_rsi_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS kuka_rsi_hardware_node
  DESTINATION lib/${PROJECT_NAME})


ament_export_libraries(kuka_rsi_hardware)
ament_export_include_directories(include)
ament_package()