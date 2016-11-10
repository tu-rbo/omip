# - Config file for the libpointmatcher package
# It defines the following variables
#  libpointmatcher_INCLUDE_DIRS - include directories for pointmatcher
#  libpointmatcher_LIBRARIES    - libraries to link against
 
# Compute paths
get_filename_component(POINTMATCHER_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
#set(libpointmatcher_INCLUDE_DIRS "/usr/include;/usr/include/eigen3;/opt/ros/indigo/include;/tmp/binarydeb/ros-indigo-libpointmatcher-1.2.3/contrib/yaml-cpp-pm/include;/tmp/binarydeb/ros-indigo-libpointmatcher-1.2.3")

#set(libpointmatcher_LIBRARIES "/tmp/binarydeb/ros-indigo-libpointmatcher-1.2.3/obj-x86_64-linux-gnu/libpointmatcher.so;/usr/lib/x86_64-linux-gnu/libboost_thread.so;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so;/usr/lib/x86_64-linux-gnu/libboost_system.so;/usr/lib/x86_64-linux-gnu/libboost_program_options.so;/usr/lib/x86_64-linux-gnu/libboost_date_time.so;/usr/lib/x86_64-linux-gnu/libboost_chrono.so;/usr/lib/x86_64-linux-gnu/libpthread.so;/opt/ros/indigo/lib/libnabo.a;gomp;/tmp/binarydeb/ros-indigo-libpointmatcher-1.2.3/obj-x86_64-linux-gnu/contrib/yaml-cpp-pm/libyaml-cpp-pm.a;rt")

set(libpointmatcher_INCLUDE_DIRS "/opt/ros/indigo/include;/usr/include;/usr/include/eigen3;/opt/ros/indigo/include")

set(libpointmatcher_LIBRARIES "/opt/ros/indigo/lib/libpointmatcher.so;/usr/lib/x86_64-linux-gnu/libboost_thread.so;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so;/usr/lib/x86_64-linux-gnu/libboost_system.so;/usr/lib/x86_64-linux-gnu/libboost_program_options.so;/usr/lib/x86_64-linux-gnu/libboost_date_time.so;/usr/lib/x86_64-linux-gnu/libboost_chrono.so;/usr/lib/x86_64-linux-gnu/libpthread.so;/opt/ros/indigo/lib/libnabo.a;gomp;rt")


# This causes catkin simple to link against these libraries
set(libpointmatcher_FOUND_CATKIN_PROJECT true)
