#
# Copyright 2026 Chaoheng Meng
# SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required(VERSION 3.8)
project(geometric_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_library(reference_trajectory src/reference_trajectory.cpp)
target_include_directories(reference_trajectory PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(reference_trajectory PUBLIC cxx_std_17)

add_library(controller_algorithms
  src/controller_factory.cpp
  src/full_wrench_controllers.cpp
  src/main_controller_math.cpp)
target_include_directories(controller_algorithms PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(controller_algorithms PUBLIC cxx_std_17)
target_link_libraries(controller_algorithms PUBLIC Eigen3::Eigen)

add_executable(trajectory_offboard_node src/geometric_controller_node.cpp)
target_compile_features(trajectory_offboard_node PUBLIC cxx_std_17)
target_link_libraries(trajectory_offboard_node reference_trajectory controller_algorithms Eigen3::Eigen)
ament_target_dependencies(trajectory_offboard_node
  rclcpp
  px4_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  rcl_interfaces)

add_executable(trajectory_control_panel src/trajectory_control_panel.cpp)
target_compile_features(trajectory_control_panel PUBLIC cxx_std_17)
target_link_libraries(
  trajectory_control_panel reference_trajectory controller_algorithms Eigen3::Eigen Qt5::Widgets)
ament_target_dependencies(trajectory_control_panel
  rclcpp
  std_msgs
  rcl_interfaces)

install(TARGETS reference_trajectory controller_algorithms
  EXPORT export_reference_trajectory
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

install(TARGETS
  trajectory_offboard_node
  trajectory_control_panel
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING
  PATTERN "*.launch.py"
  PATTERN "__pycache__" EXCLUDE)

install(DIRECTORY config rviz
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_targets(export_reference_trajectory HAS_LIBRARY_TARGET)

ament_package()
