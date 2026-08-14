cmake_minimum_required(VERSION 3.8)
project(scan_mppi_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scan_planner_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenMP QUIET)

add_library(scan_mppi_core
  src/mppi_core.cpp
  src/voxel_map.cpp)
target_include_directories(scan_mppi_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(scan_mppi_core PUBLIC Eigen3::Eigen)
if(OpenMP_CXX_FOUND)
  target_link_libraries(scan_mppi_core PUBLIC OpenMP::OpenMP_CXX)
  target_compile_definitions(scan_mppi_core PUBLIC SCAN_MPPI_HAS_OPENMP=1)
endif()

add_executable(scan_mppi_controller_node src/scan_mppi_controller_node.cpp)
target_include_directories(scan_mppi_controller_node PRIVATE include)
ament_target_dependencies(scan_mppi_controller_node
  bspline_opt geometry_msgs nav_msgs rclcpp scan_planner_msgs sensor_msgs
  std_msgs tf2 tf2_geometry_msgs visualization_msgs)
target_link_libraries(scan_mppi_controller_node scan_mppi_core Eigen3::Eigen)

install(TARGETS scan_mppi_core scan_mppi_controller_node
  EXPORT export_scan_mppi_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_scan_mppi_core HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_gtest(test_mppi_core test/test_mppi_core.cpp)
  target_link_libraries(test_mppi_core scan_mppi_core Eigen3::Eigen)
  target_include_directories(test_mppi_core PRIVATE include)
  add_launch_test(test/test_controller_launch.py TIMEOUT 30)
endif()

ament_package()
