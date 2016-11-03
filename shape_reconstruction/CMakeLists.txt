cmake_minimum_required(VERSION 2.8.3)
project(shape_reconstruction)

if(NOT CMAKE_BUILD_TYPE MATCHES Release AND NOT CMAKE_BUILD_TYPE MATCHES RelWithDebInfo )
  message("WARNING! Manually setting the build type to RelWithDebInfo because if you compile in Debug an segmentation fault from PCL will appear (Lanczos...)")
  SET(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
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
  message_generation
  pcl_conversions
  lgsm
  cmake_modules)

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

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS signals thread filesystem)
find_package(Eigen REQUIRED)
find_package(bfl REQUIRED)

add_service_files(FILES generate_meshes.srv)
generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp omip_common omip_msgs std_msgs geometry_msgs sensor_msgs image_transport cv_bridge pcl_ros rb_tracker
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
  include/vcglib
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${bfl_INCLUDE_DIRS}
)

# List of source files for the library. Including headers to see them in qtcreator
FILE(GLOB source_lib_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
		"include/shape_reconstruction/RangeImagePlanar.h"
                "include/shape_reconstruction/RangeImagePlanar.hpp"
		"include/shape_reconstruction/ShapeReconstruction.h"
                "include/shape_reconstruction/SRUtils.h"
                "include/shape_reconstruction/Passthrough.h"
                "include/shape_reconstruction/Passthrough.hpp"
                "src/ShapeReconstruction.cpp"
                "src/SRUtils.cpp"
)

# List of source files for the application/node. Including headers to see them in qtcreator
FILE(GLOB source_node_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    "include/shape_reconstruction/ShapeReconstructionNode.h"
    "src/ShapeReconstructionNode.cpp"
)

# List of source files for the application/node. Including headers to see them in qtcreator
FILE(GLOB source_smoother_node_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    "include/shape_reconstruction/SurfaceSmootherNode.h"
    "src/SurfaceSmootherNode.cpp"
)

# List of source files for the application/node. Including headers to see them in qtcreator
FILE(GLOB source_srp_node_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    "include/shape_reconstruction/ShapeRecPlotter.h"
    "src/ShapeRecPlotter.cpp"
)

# Generate a library with the name of the project
add_library(${PROJECT_NAME} ${source_lib_files})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${bfl_LIBRARIES}
)

# Generate an executable with the name of the project + "_node" that links to the library
add_executable(${PROJECT_NAME}_node ${source_node_files})
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_smoother_node ${source_smoother_node_files})
add_dependencies(${PROJECT_NAME}_smoother_node ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_smoother_node ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_srp_node ${source_srp_node_files})
add_dependencies(${PROJECT_NAME}_srp_node ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_srp_node ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_statistics src/statistics.cpp)
add_dependencies(${PROJECT_NAME}_statistics ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_statistics ${PROJECT_NAME})
