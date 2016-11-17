cmake_minimum_required(VERSION 2.8.3)

project(feature_tracker)

if(NOT CMAKE_BUILD_TYPE MATCHES Release AND NOT CMAKE_BUILD_TYPE MATCHES RelWithDebInfo )
  message("WARNING! Manually setting the build type to RelWithDebInfo to speed up execution")
  SET(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  dynamic_reconfigure
  roscpp
  omip_common
  omip_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
  image_transport
  cv_bridge
  pcl_ros
  pcl_conversions
  message_generation
  rosbag
  lgsm)

find_package(OpenCV REQUIRED) 

find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS signals thread)
find_package(Eigen 2 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/FeatureTrackerDynReconf.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp omip_common omip_msgs std_msgs geometry_msgs sensor_msgs image_transport cv_bridge pcl_ros message_runtime rosbag dynamic_reconfigure pcl_conversions
  DEPENDS opencv pcl lgsm)

#Special set of commands (optimized) for 64bit architecture
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  add_definitions("-msse3")
endif()

include_directories(include ${Boost_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# List of source files for the library. Including headers to see them in qtcreator
FILE(GLOB source_lib_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} 
    "include/feature_tracker/FeatureTracker.h"
    "include/feature_tracker/PointFeatureTracker.h"
    "src/PointFeatureTracker.cpp" )
	
# List of source files for the application/node. Including headers to see them in qtcreator
FILE(GLOB source_node_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} 
    "include/feature_tracker/FeatureTrackerNode.h"
    "src/FeatureTrackerNode.cpp" )


# Generate a library with the name of the project
add_library(${PROJECT_NAME} ${source_lib_files})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
# Generate an executable with the name of the project + "_node" that links to the library
add_executable(${PROJECT_NAME}_node ${source_node_files})
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
