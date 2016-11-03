cmake_minimum_required(VERSION 2.8.3)
project(lgsm)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)
