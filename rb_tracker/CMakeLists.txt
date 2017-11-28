cmake_minimum_required(VERSION 2.8.3)

project(rb_tracker)

if(NOT CMAKE_BUILD_TYPE MATCHES Release AND NOT CMAKE_BUILD_TYPE MATCHES RelWithDebInfo )
  message("WARNING! Manually setting the build type to RelWithDebInfo because if you compile in Debug an segmentation fault from PCL will appear (Lanczos...)")
  SET(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS 
  lgsm
  dynamic_reconfigure
  roscpp
  omip_common
  omip_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
  image_transport
  cv_bridge  
  message_generation
  rosbag
  pcl_ros
  pcl_conversions
)

find_package(bfl REQUIRED)
find_package(Boost REQUIRED COMPONENTS signals thread)

generate_dynamic_reconfigure_options(
  cfg/RBTrackerDynReconf.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp omip_common omip_msgs std_msgs geometry_msgs sensor_msgs image_transport cv_bridge pcl_ros message_runtime rosbag dynamic_reconfigure
  DEPENDS message_runtime)

#Special set of commands (optimized) for 64bit architecture
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  add_definitions("-msse3")
endif()

add_definitions("-std=c++11")

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${bfl_INCLUDE_DIRS})

# Generates a list of all .cpp files in /src and subdirectories
FILE(GLOB source_lib_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} 
        "include/rb_tracker/RBFilter.h"
        "include/rb_tracker/StaticEnvironmentFilter.h"
        "include/rb_tracker/MultiRBTracker.h"
	"src/RBFilter.cpp" 
        "src/StaticEnvironmentFilter.cpp"
        "src/MultiRBTracker.cpp"
)
	
FILE(GLOB source_node_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
        "include/rb_tracker/MultiRBTrackerNode.h"
        "src/MultiRBTrackerNode.cpp" )

# Generate a library with the name of the project
add_library(${PROJECT_NAME} ${source_lib_files})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${bfl_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${bfl_LIBRARIES} )

# Generate an executable with the name of the project + "_node" that links to the library
add_executable(${PROJECT_NAME}_node ${source_node_files})
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node  ${PROJECT_NAME} ${bfl_LIBRARIES} )
