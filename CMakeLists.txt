cmake_minimum_required (VERSION 2.4.6)

include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set (ROS_BUILD_TYPE RelWithDebInfo)
#set (ROS_BUILD_TYPE Debug)
rosbuild_init ()
rosbuild_gensrv ()
rosbuild_genmsg ()

set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# Enable dynamic_reconfigure
rosbuild_find_ros_package (dynamic_reconfigure)
include (${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg ()

rosbuild_add_executable (dp_ptu47 src/dp_ptu47.cpp)
rosbuild_add_executable (dp_ptu47_trigger src/dp_ptu47_trigger.cpp)

# Once the PTU unit is calibrated with respect to /base_link, this can create that TF frame
rosbuild_add_executable (ptu_transform_publisher_calibration src/ptu_transform_publisher_calibration.cpp)
