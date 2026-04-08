cmake_minimum_required(VERSION 3.5)
project(modelo_omnidireccional)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robmovil_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# --- omni_odometry_node (Ejercicio 1) ---
add_executable(omni_odometry_node
  src/omni_odometry.cpp
  src/omni_odometry_node.cpp
)
ament_target_dependencies(omni_odometry_node
  rclcpp std_msgs geometry_msgs nav_msgs
  robmovil_msgs tf2 tf2_ros tf2_geometry_msgs
)

# --- trajectory_pilot_node (Ejercicio 2) ---
add_executable(trajectory_pilot_node
  src/trajectory_generator.cpp
  src/trajectory_pilot.cpp
  src/trajectory_pilot_node.cpp
)
ament_target_dependencies(trajectory_pilot_node
  rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs
)

# --- laser_detector_node (Ejercicio 3.1) ---
add_executable(laser_detector_node
  src/laser_detector.cpp
  src/laser_detector_node.cpp
)
ament_target_dependencies(laser_detector_node
  rclcpp sensor_msgs robmovil_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs
)

# --- ekf_localizer (Ejercicio 3.4) ---
add_executable(ekf_localizer
src/ekf_localizer.cpp
src/ekf_node.cpp
)
ament_target_dependencies(ekf_localizer
  rclcpp nav_msgs geometry_msgs robmovil_msgs tf2 tf2_ros
)
target_include_directories(ekf_localizer PRIVATE ${EIGEN3_INCLUDE_DIRS})

install(TARGETS
  omni_odometry_node
  trajectory_pilot_node
  laser_detector_node
  ekf_localizer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
