cmake_minimum_required(VERSION 3.5)
project(kuka_rsi_hardware)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(angles REQUIRED)
find_package(tinyxml_vendor REQUIRED)
find_package(TinyXML REQUIRED)

add_library(kuka_rsi_hardware SHARED src/kuka_rsi_hardware.cpp)
target_include_directories(kuka_rsi_hardware
                           PRIVATE include ${TinyXML_INCLUDE_DIRS})
target_link_libraries(kuka_rsi_hardware ${TinyXML_LIBRARY})
ament_target_dependencies(kuka_rsi_hardware
                          hardware_interface
                          pluginlib
                          rclcpp
                          angles)

# Causes the visibility macros to use dllexport rather than dllimport, which is
# appropriate when building the dll but not consuming it.
target_compile_definitions(kuka_rsi_hardware PRIVATE
                           "KUKA_RSI_HARDWARE_BUILDING_DLL")

install(DIRECTORY include/ DESTINATION include)

install(TARGETS kuka_rsi_hardware
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(DIRECTORY config DESTINATION share/kuka_rsi_hardware)

pluginlib_export_plugin_description_file(hardware_interface kuka_rsi_hardware.xml)

ament_export_libraries(kuka_rsi_hardware)
ament_export_include_directories(include)
ament_export_dependencies(hardware_interface pluginlib rclcpp)
ament_package()
