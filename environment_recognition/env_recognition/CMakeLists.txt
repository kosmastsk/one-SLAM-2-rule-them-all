cmake_minimum_required(VERSION 2.8.3)
project(env_recognition)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES env_recognition
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(recognition_lidar 
  src/recognition_lidar_node.cpp
  src/recognition_lidar.cpp)
target_link_libraries(recognition_lidar 
  ${catkin_LIBRARIES})
add_dependencies(recognition_lidar 
  ${catkin_EXPORTED_TARGETS})

add_executable(recognition_rgb 
  src/recognition_rgb_node.cpp
  src/recognition_rgb.cpp)
target_link_libraries(recognition_rgb 
  ${catkin_LIBRARIES})
add_dependencies(recognition_rgb 
  ${catkin_EXPORTED_TARGETS})

add_executable(recognition_depth 
  src/recognition_depth_node.cpp
  src/recognition_depth.cpp)
target_link_libraries(recognition_depth 
  ${catkin_LIBRARIES})
add_dependencies(recognition_depth 
  ${catkin_EXPORTED_TARGETS})