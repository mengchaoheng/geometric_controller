cmake_minimum_required(VERSION 3.8)
project(geometric_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_library(reference_trajectory src/reference_trajectory.cpp)
target_include_directories(reference_trajectory PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(reference_trajectory PUBLIC cxx_std_17)

add_executable(trajectory_offboard_node src/geometric_controller_node.cpp)
target_link_libraries(trajectory_offboard_node reference_trajectory)
ament_target_dependencies(trajectory_offboard_node
  rclcpp
  px4_msgs
  geometry_msgs
  nav_msgs
  rcl_interfaces)

add_executable(trajectory_control_panel src/trajectory_control_panel.cpp)
target_compile_features(trajectory_control_panel PUBLIC cxx_std_17)
target_link_libraries(trajectory_control_panel reference_trajectory Qt5::Widgets)
ament_target_dependencies(trajectory_control_panel
  rclcpp
  rcl_interfaces)

install(TARGETS
  reference_trajectory
  trajectory_offboard_node
  trajectory_control_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
