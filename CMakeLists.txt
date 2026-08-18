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

# Prefer system/ROS installations, with a reproducible no-sudo local prefix
# created by scripts/install_ommpc_solvers.sh as a fallback.
set(OMMPC_LOCAL_DEPS "${CMAKE_CURRENT_SOURCE_DIR}/.ommpc_deps")
if(EXISTS "${OMMPC_LOCAL_DEPS}")
  list(PREPEND CMAKE_PREFIX_PATH
    "${OMMPC_LOCAL_DEPS}/opt/ros/jazzy"
    "${OMMPC_LOCAL_DEPS}/qpoases"
    "${OMMPC_LOCAL_DEPS}/daqp"
    "${OMMPC_LOCAL_DEPS}/piqp"
    "${OMMPC_LOCAL_DEPS}/qpswift")
  list(PREPEND CMAKE_INCLUDE_PATH "${OMMPC_LOCAL_DEPS}/usr/include")
endif()

set(OMMPC_CVXPYGEN_DIR "${OMMPC_LOCAL_DEPS}/cvxpygen_ommpc/c")
if(NOT EXISTS "${OMMPC_CVXPYGEN_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "CVXPYgen OMMPC code missing; run scripts/install_ommpc_solvers.sh")
endif()
add_subdirectory(
  "${OMMPC_CVXPYGEN_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/cvxpygen_ommpc" EXCLUDE_FROM_ALL)
add_library(lu_ommpc_cpg SHARED ${lu__cpg_src})
target_include_directories(lu_ommpc_cpg PRIVATE ${lu__cpg_include})
target_compile_options(lu_ommpc_cpg PRIVATE -O3 -w)
target_link_libraries(lu_ommpc_cpg PRIVATE m)
target_link_options(lu_ommpc_cpg PRIVATE
  "-Wl,-Bsymbolic"
  "-Wl,--version-script=${CMAKE_CURRENT_SOURCE_DIR}/patches/cvxpygen-symbols.map")
add_library(lu_ommpc_cvxpygen_adapter OBJECT src/lu_ommpc_cvxpygen_solver.cpp)
target_include_directories(lu_ommpc_cvxpygen_adapter PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include" ${lu__cpg_include})
target_link_libraries(lu_ommpc_cvxpygen_adapter PRIVATE Eigen3::Eigen)
target_compile_features(lu_ommpc_cvxpygen_adapter PRIVATE cxx_std_17)
find_package(osqp REQUIRED CONFIG)
find_package(qpOASES REQUIRED CONFIG)
find_package(proxsuite REQUIRED CONFIG)
find_package(piqp REQUIRED CONFIG)
find_package(qpSWIFT REQUIRED CONFIG)

add_library(reference_trajectory src/reference_trajectory.cpp)
target_include_directories(reference_trajectory PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(reference_trajectory PUBLIC cxx_std_17)

add_library(lu_ommpc_core
  src/lu_ommpc_core.cpp
  src/lu_ommpc_external_solvers.cpp
  src/lu_ommpc_structured_solvers.cpp
  $<TARGET_OBJECTS:lu_ommpc_cvxpygen_adapter>
  src/lu_ommpc_solvers.cpp
  src/lu_ommpc_qp_dataset.cpp)
target_include_directories(lu_ommpc_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(lu_ommpc_core PUBLIC cxx_std_17)
target_include_directories(lu_ommpc_core SYSTEM PRIVATE
  ${qpOASES_INCLUDE_DIR}
  "${OMMPC_LOCAL_DEPS}/daqp/include"
  "${OMMPC_LOCAL_DEPS}/daqp/include/daqp"
  "${OMMPC_LOCAL_DEPS}/hpipm/include"
  "${OMMPC_LOCAL_DEPS}/blasfeo/include"
  "${OMMPC_LOCAL_DEPS}/ooqp/include/ooqp")
target_include_directories(lu_ommpc_core SYSTEM PRIVATE
  "${OMMPC_LOCAL_DEPS}/qpdunes/include/qpdunes"
  "${OMMPC_LOCAL_DEPS}/src/tinympc-next/src/tinympc")
target_link_libraries(lu_ommpc_core PUBLIC
  Eigen3::Eigen osqp::osqpstatic ${qpOASES_LIBRARY} proxsuite::proxsuite
  piqp::piqp_header_only qpSWIFT::qpSWIFT-static
  "${OMMPC_LOCAL_DEPS}/daqp/lib/libdaqpstat.a"
  "${OMMPC_LOCAL_DEPS}/hpipm/lib/libhpipm.a"
  "${OMMPC_LOCAL_DEPS}/blasfeo/lib/libblasfeo.a"
  "${OMMPC_LOCAL_DEPS}/ooqp/lib/libooqpgendense.a"
  "${OMMPC_LOCAL_DEPS}/ooqp/lib/libooqpgondzio.a"
  "${OMMPC_LOCAL_DEPS}/ooqp/lib/libooqpdense.a"
  "${OMMPC_LOCAL_DEPS}/ooqp/lib/libooqpbase.a"
  "${OMMPC_LOCAL_DEPS}/qpdunes/lib/libqpdunes.a"
  "${OMMPC_LOCAL_DEPS}/build/tinympc-next/src/tinympc/libtinympcstatic.a"
  lu_ommpc_cpg
  blas lapack gfortran m)
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

install(TARGETS reference_trajectory controller_algorithms lu_ommpc_core lu_ommpc_cpg
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
