cmake_minimum_required(VERSION 2.8.3)

project(joint_tracker)

if(NOT CMAKE_BUILD_TYPE MATCHES Release AND NOT CMAKE_BUILD_TYPE MATCHES RelWithDebInfo )
  message("WARNING! Manually setting the build type to RelWithDebInfo to speed up execution")
  SET(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS 
  lgsm
  roscpp
  omip_common
  omip_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
  image_transport
  cv_bridge
  pcl_ros
  message_generation
  pcl_conversions
  rosbag
)

find_package(bfl REQUIRED)
find_package(Boost REQUIRED COMPONENTS signals thread)

add_service_files(FILES publish_urdf.srv)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp omip_common omip_msgs std_msgs geometry_msgs sensor_msgs image_transport cv_bridge pcl_ros message_runtime rosbag rb_tracker
  DEPENDS)

 #Special set of commands (optimized) for 64bit architecture
 if(CMAKE_SIZEOF_VOID_P EQUAL 8)
   add_definitions("-msse3")
 endif()

add_definitions("-std=c++11")

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${bfl_INCLUDE_DIRS} )

# List of source files for the library
FILE(GLOB source_lib_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} 
        "include/joint_tracker/JointCombinedFilter.h"
        "include/joint_tracker/MultiJointTracker.h"
        "include/joint_tracker/JointFilter.h"
        "include/joint_tracker/PrismaticJointFilter.h"
        "include/joint_tracker/RevoluteJointFilter.h"
        "include/joint_tracker/RigidJointFilter.h"
        "include/joint_tracker/DisconnectedJointFilter.h"
        "include/joint_tracker/pdf/NonLinearPrismaticMeasurementPdf.h"
        "include/joint_tracker/pdf/NonLinearRevoluteMeasurementPdf.h"
        "src/JointCombinedFilter.cpp"
        "src/JointFilter.cpp"
        "src/DisconnectedJointFilter.cpp"
        "src/RigidJointFilter.cpp"
        "src/PrismaticJointFilter.cpp"
        "src/RevoluteJointFilter.cpp"
        "src/MultiJointTracker.cpp"
        "src/pdf/NonLinearPrismaticMeasurementPdf.cpp"
        "src/pdf/NonLinearRevoluteMeasurementPdf.cpp")

# List of source files for the application/node
FILE(GLOB source_node_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} 
    "include/joint_tracker/MultiJointTrackerNode.h"
    "src/MultiJointTrackerNode.cpp" )

# Generate a library with the name of the project
add_library(${PROJECT_NAME} ${source_lib_files})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${bfl_LIBRARIES} )

# Generate an executable with the name of the project + "_node" that links to the library
add_executable(${PROJECT_NAME}_node ${source_node_files})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${bfl_LIBRARIES})
