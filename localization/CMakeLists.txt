cmake_minimum_required(VERSION 3.0.2)
project(lam_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  realsense2_camera
  rplidar_ros
  rtabmap_odom
  robot_localization
  my_imu_package
)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES lam_pkg
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs tf2_sensor_msgs realsense2_camera rplidar_ros rtabmap_odom robot_localization my_imu_package
  # DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## If you have nodes, add executables here:
# add_executable(${PROJECT_NAME}_node src/your_node.cpp)
# target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

## Install launch and config files
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
