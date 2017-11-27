cmake_minimum_required(VERSION 2.8.3)

project(omip_common)

if(NOT CMAKE_BUILD_TYPE MATCHES Release)
  message("WARNING! Manually setting the build type to Release to speed up execution")
  SET(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS 
  lgsm  # place private eigen3.2 include in front
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  omip_msgs
  image_transport
  cv_bridge
  pcl_ros
  message_generation
)


find_package(bfl REQUIRED)
find_package(Boost REQUIRED COMPONENTS signals thread)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs omip_msgs image_transport cv_bridge pcl_ros message_runtime
)

#Special set of commands (optimized) for 64bit architecture
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  add_definitions("-msse3")
endif()

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${bfl_INCLUDE_DIRS})

#Added header files to see them in qtcreator. It doesn't harm the compilation
set(source_files
        include/omip_common/Feature.h
        include/omip_common/FeaturesDataBase.h
        include/omip_common/OMIPUtils.h
        include/omip_common/OMIPTypeDefs.h
        include/omip_common/RecursiveEstimatorFilterInterface.h
        include/omip_common/RecursiveEstimatorNodeInterface.h
        src/Feature.cpp
        src/FeaturesDataBase.cpp
        src/OMIPUtils.cpp)

add_library(${PROJECT_NAME} ${source_files})
target_link_libraries(${PROJECT_NAME} ${bfl_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(omip_common ${catkin_EXPORTED_TARGETS})
