cmake_minimum_required(VERSION 2.8.3)
project(shape_tracker)

if(NOT CMAKE_BUILD_TYPE MATCHES Release AND NOT CMAKE_BUILD_TYPE MATCHES RelWithDebInfo )
  message("WARNING! Manually setting the build type to RelWithDebInfo to speed up execution")
  SET(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
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
  pcl_ros
  message_generation
  pcl_conversions
  shape_reconstruction
)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(Boost REQUIRED COMPONENTS signals thread filesystem)
find_package(bfl REQUIRED)

# Comment/Uncomment this line to compile the LIBPOINTMATCHER ICP variant
set(USE_LIBPOINTMATCHER_ICP 1)
if(USE_LIBPOINTMATCHER_ICP)
add_definitions( -DUSE_LIBPOINTMATCHER_ICP)
find_package(libpointmatcher REQUIRED)
include_directories("${libpointmatcher_INCLUDE_DIRS}")
message(STATUS "Using libpointmatcher version ${libpointmatcher_VERSION}")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp omip_common omip_msgs std_msgs geometry_msgs sensor_msgs image_transport cv_bridge pcl_ros rb_tracker shape_reconstruction
  DEPENDS opencv pcl
)

###########
## Build ##
###########

# activate profiling built
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")

 #Special set of commands (optimized) for 64bit architecture
 if(CMAKE_SIZEOF_VOID_P EQUAL 8)
   add_definitions("-msse3")
 endif()

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${bfl_INCLUDE_DIRS}
)



if(USE_LIBPOINTMATCHER_ICP)
# List of source files for the library. Including headers to see them in qtcreator
FILE(GLOB source_lib_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
                "include/shape_tracker/ShapeTracker.h"
                "include/shape_tracker/point_cloud_conversions.h"
                "src/ShapeTracker.cpp"
                "src/point_cloud_conversions.cpp"
)
else()
# List of source files for the library. Including headers to see them in qtcreator
FILE(GLOB source_lib_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
                "include/shape_tracker/ShapeTracker.h"
                "src/ShapeTracker.cpp"
)
endif()

# List of source files for the application/node. Including headers to see them in qtcreator
FILE(GLOB source_node_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
                "include/shape_tracker/ShapeTrackerNode.h"
                "src/ShapeTrackerNode.cpp"
)

# Generate a library with the name of the project
add_library(${PROJECT_NAME} ${source_lib_files})

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${bfl_LIBRARIES}
)

if(USE_LIBPOINTMATCHER_ICP)
target_link_libraries(${PROJECT_NAME} ${libpointmatcher_LIBRARIES})
endif()

# Generate an executable with the name of the project + "_node" that links to the library
add_executable(${PROJECT_NAME}_node ${source_node_files} )
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})
