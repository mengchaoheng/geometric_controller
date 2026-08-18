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

# qpDUNES is the online default. Four additional, commonly used baselines are
# retained for diagnosis: HPIPM OCP, qpOASES, OSQP, and DAQP. A reproducible local
# prefix is created by scripts/install_ommpc_solvers.sh.
set(OMMPC_LOCAL_DEPS "${CMAKE_CURRENT_SOURCE_DIR}/.ommpc_deps")
if(EXISTS "${OMMPC_LOCAL_DEPS}")
  list(PREPEND CMAKE_PREFIX_PATH "${OMMPC_LOCAL_DEPS}/opt/ros/jazzy")
  list(PREPEND CMAKE_INCLUDE_PATH "${OMMPC_LOCAL_DEPS}/qpdunes/include")
endif()
set(OMMPC_QPDUNES_INCLUDE "${OMMPC_LOCAL_DEPS}/qpdunes/include/qpdunes")
set(OMMPC_QPDUNES_LIBRARY "${OMMPC_LOCAL_DEPS}/qpdunes/lib/libqpdunes.a")
set(OMMPC_QPOASES_INCLUDE "${OMMPC_LOCAL_DEPS}/qpoases/include")
set(OMMPC_QPOASES_LIBRARY "${OMMPC_LOCAL_DEPS}/qpoases/lib/libqpOASES.a")
set(OMMPC_DAQP_INCLUDE "${OMMPC_LOCAL_DEPS}/daqp/include")
set(OMMPC_DAQP_C_INCLUDE "${OMMPC_LOCAL_DEPS}/daqp/include/daqp")
set(OMMPC_DAQP_WRAPPER_INCLUDE "${OMMPC_LOCAL_DEPS}/src/daqp/interfaces/daqp-eigen")
set(OMMPC_DAQP_LIBRARY "${OMMPC_LOCAL_DEPS}/daqp/lib/libdaqpstat.a")
set(OMMPC_OSQP_INCLUDE "${OMMPC_LOCAL_DEPS}/opt/ros/jazzy/include")
set(OMMPC_OSQP_LIBRARY "${OMMPC_LOCAL_DEPS}/opt/ros/jazzy/lib/libosqp.a")
set(OMMPC_HPIPM_INCLUDE "${OMMPC_LOCAL_DEPS}/hpipm/include")
set(OMMPC_BLASFEO_INCLUDE "${OMMPC_LOCAL_DEPS}/blasfeo/include")
set(OMMPC_HPIPM_LIBRARY "${OMMPC_LOCAL_DEPS}/hpipm/lib/libhpipm.a")
set(OMMPC_BLASFEO_LIBRARY "${OMMPC_LOCAL_DEPS}/blasfeo/lib/libblasfeo.a")
if(NOT EXISTS "${OMMPC_QPDUNES_INCLUDE}/qpDUNES.h" OR
  NOT EXISTS "${OMMPC_QPDUNES_LIBRARY}" OR
  NOT EXISTS "${OMMPC_QPOASES_LIBRARY}" OR
  NOT EXISTS "${OMMPC_DAQP_LIBRARY}" OR
  NOT EXISTS "${OMMPC_DAQP_WRAPPER_INCLUDE}/daqp.cpp" OR
  NOT EXISTS "${OMMPC_OSQP_LIBRARY}" OR
  NOT EXISTS "${OMMPC_HPIPM_LIBRARY}" OR
  NOT EXISTS "${OMMPC_BLASFEO_LIBRARY}")
  message(FATAL_ERROR "retained OMMPC baseline libraries are missing; run scripts/install_ommpc_solvers.sh")
endif()

add_library(reference_trajectory src/reference_trajectory.cpp)
target_include_directories(reference_trajectory PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(reference_trajectory PUBLIC cxx_std_17)

add_library(lu_ommpc_core
  src/lu_ommpc_core.cpp
  src/lu_ommpc_baseline_solvers.cpp
  src/lu_ommpc_structured_solvers.cpp
  src/lu_ommpc_solvers.cpp
  src/lu_ommpc_qp_dataset.cpp)
target_include_directories(lu_ommpc_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(lu_ommpc_core PUBLIC cxx_std_17)
target_include_directories(lu_ommpc_core SYSTEM PRIVATE
  "${OMMPC_QPDUNES_INCLUDE}"
  "${OMMPC_QPOASES_INCLUDE}"
  "${OMMPC_DAQP_INCLUDE}"
  "${OMMPC_DAQP_C_INCLUDE}"
  "${OMMPC_DAQP_WRAPPER_INCLUDE}"
  "${OMMPC_OSQP_INCLUDE}"
  "${OMMPC_HPIPM_INCLUDE}"
  "${OMMPC_BLASFEO_INCLUDE}")
target_link_libraries(lu_ommpc_core PUBLIC
  Eigen3::Eigen
  "${OMMPC_QPDUNES_LIBRARY}"
  "${OMMPC_QPOASES_LIBRARY}"
  "${OMMPC_DAQP_LIBRARY}"
  "${OMMPC_OSQP_LIBRARY}"
  "${OMMPC_HPIPM_LIBRARY}"
  "${OMMPC_BLASFEO_LIBRARY}"
  blas lapack m)
target_sources(lu_ommpc_core PRIVATE
  "${OMMPC_LOCAL_DEPS}/src/daqp/interfaces/daqp-eigen/daqp.cpp")
set_source_files_properties(
  "${OMMPC_LOCAL_DEPS}/src/daqp/interfaces/daqp-eigen/daqp.cpp"
  PROPERTIES COMPILE_OPTIONS "-w")

add_library(controller_algorithms
  src/controller_factory.cpp
  src/full_wrench_controllers.cpp
  src/lu_ommpc_controller.cpp
  src/main_controller_math.cpp)
target_include_directories(controller_algorithms PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(controller_algorithms PUBLIC cxx_std_17)
target_link_libraries(controller_algorithms PUBLIC Eigen3::Eigen lu_ommpc_core)

add_executable(lu_ommpc_benchmark src/lu_ommpc_benchmark.cpp)
target_compile_features(lu_ommpc_benchmark PUBLIC cxx_std_17)
target_link_libraries(lu_ommpc_benchmark lu_ommpc_core Eigen3::Eigen)

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

install(TARGETS reference_trajectory controller_algorithms lu_ommpc_core
  EXPORT export_reference_trajectory
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

install(TARGETS
  trajectory_offboard_node
  trajectory_control_panel
  lu_ommpc_benchmark
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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_lu_ommpc_core test/test_lu_ommpc_core.cpp)
  target_link_libraries(
    test_lu_ommpc_core controller_algorithms lu_ommpc_core Eigen3::Eigen)
endif()

ament_package()
