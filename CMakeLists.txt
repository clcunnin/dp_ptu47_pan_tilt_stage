cmake_minimum_required(VERSION 2.8.3)

project(dp_ptu47_pan_tilt_stage)

find_package(catkin REQUIRED
  COMPONENTS
    cv_bridge
    dynamic_reconfigure
    message_generation
    opencv2
    #pcl
    roscpp
    sensor_msgs
    tf
    visualization_msgs
)


add_message_files(
  DIRECTORY
    msg
  FILES
    PanTilt.msg
    PanTiltStamped.msg
)

add_service_files(
  DIRECTORY
    srv
  FILES
    GetLimits.srv
    Reset.srv
    SendCommand.srv
    SendTrajectory.srv
)

generate_messages(
  DEPENDENCIES
    sensor_msgs
    visualization_msgs
)


generate_dynamic_reconfigure_options(cfg/DpPtu47.cfg)


catkin_package(
  CATKIN_DEPENDS
    cv_bridge
    dynamic_reconfigure
    message_runtime
    opencv2
    #pcl
    roscpp
    sensor_msgs
    tf
    visualization_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_dp_ptu47 src/dp_ptu47.cpp)
target_link_libraries(${PROJECT_NAME}_dp_ptu47 ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_dp_ptu47 PROPERTIES OUTPUT_NAME dp_ptu47 PREFIX "")
add_dependencies(${PROJECT_NAME}_dp_ptu47 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_dp_ptu47_trigger src/dp_ptu47_trigger.cpp)
target_link_libraries(${PROJECT_NAME}_dp_ptu47_trigger ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_dp_ptu47_trigger PROPERTIES OUTPUT_NAME dp_ptu47_trigger PREFIX "")
add_dependencies(${PROJECT_NAME}_dp_ptu47_trigger ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Once the PTU unit is calibrated with respect to /base_link, this can create that TF frame
add_executable(${PROJECT_NAME}_ptu_transform_publisher_calibration src/ptu_transform_publisher_calibration.cpp)
target_link_libraries(${PROJECT_NAME}_ptu_transform_publisher_calibration ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_ptu_transform_publisher_calibration PROPERTIES OUTPUT_NAME ptu_transform_publisher_calibration PREFIX "")
add_dependencies(${PROJECT_NAME}_ptu_transform_publisher_calibration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


install(
  TARGETS
    ${PROJECT_NAME}_dp_ptu47
    ${PROJECT_NAME}_dp_ptu47_trigger
    ${PROJECT_NAME}_ptu_transform_publisher_calibration
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
