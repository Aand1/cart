cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
rosbuild_add_boost_directories('/usr/lib','/usr/include')
rosbuild_add_executable(bin/cart_state_estimator src/cart_state_estimator.cpp)
rosbuild_add_library(polygon_point_cloud_filter src/polygon_point_cloud_filter.cpp)
rosbuild_add_executable(bin/scan_to_cloud_filter_chain src/scan_to_cloud_filter_chain.cpp)
rosbuild_link_boost(bin/scan_to_cloud_filter_chain signals)
