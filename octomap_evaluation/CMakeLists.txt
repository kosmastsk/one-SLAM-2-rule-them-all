cmake_minimum_required(VERSION 2.8.3)
project(octomap_evaluation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Enable the following for using OpenMP in Metric's calculation
option(WITH_OPENMP "Include OpenMP support" OFF) # When it turns ON, do not forget to uncomment line 125 @ calculate_metric.cpp

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  octomap_ros
  pcl_ros
)

find_package(PCL REQUIRED)

find_package(libpointmatcher 1.1.0 REQUIRED)
message(STATUS "Using libpointmatcher version ${libpointmatcher_VERSION}")

find_package(OCTOMAP REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

if (WITH_OPENMP)
  find_package(OpenMP REQUIRED)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES octomap_evaluation
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${libpointmatcher_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(octomap_to_pc
  src/octomap_to_pc_node.cpp
  src/octomap_to_pc.cpp)
target_link_libraries(octomap_to_pc ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(octomap_to_pc ${catkin_EXPORTED_TARGETS})

add_executable(icp
  src/icp_node.cpp
  src/icp.cpp)
target_link_libraries(icp ${catkin_LIBRARIES} ${libpointmatcher_LIBRARIES})
add_dependencies(icp ${catkin_EXPORTED_TARGETS})

add_executable(calculate_metric
  src/calculate_metric_node.cpp
  src/calculate_metric.cpp)
target_link_libraries(calculate_metric ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(calculate_metric ${catkin_EXPORTED_TARGETS})

add_executable(octomap_evaluation
  src/octomap_evaluation.cpp)
target_link_libraries(octomap_evaluation ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(octomap_evaluation ${catkin_EXPORTED_TARGETS})
