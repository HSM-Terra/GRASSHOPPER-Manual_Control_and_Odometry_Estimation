cmake_minimum_required(VERSION 3.5)
project(odometry_estimator)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vesc_msgs REQUIRED)
find_package(robot_assembly REQUIRED)
find_package(ros2_laser_scan_matcher REQUIRED)
find_package(robot_localization REQUIRED)

# Vehicle models lib
add_library(vehicle_models src/vehicle_models/vehicle_models.cpp)
target_include_directories(
  vehicle_models
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/vehicle_models>
         $<INSTALL_INTERFACE:include>)

# Node
add_executable(odometry_estimator src/node/odometry_estimation_node.cpp)
ament_target_dependencies(odometry_estimator rclcpp nav_msgs std_msgs sensor_msgs vesc_msgs tf2_geometry_msgs)
add_executable(radian_to_rpm src/node/radian_to_rpm.cpp)
ament_target_dependencies(radian_to_rpm rclcpp nav_msgs std_msgs sensor_msgs vesc_msgs tf2_geometry_msgs)

target_link_libraries(odometry_estimator vehicle_models)
target_include_directories(
  odometry_estimator
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/node>
         $<INSTALL_INTERFACE:include>)

install(TARGETS odometry_estimator DESTINATION lib/${PROJECT_NAME})
install(TARGETS radian_to_rpm DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_package()
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)