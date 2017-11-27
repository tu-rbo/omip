cmake_minimum_required(VERSION 2.8.3)
project(lgsm)

## Find catkin macros and libraries
find_package(catkin)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS EIGEN3
)
