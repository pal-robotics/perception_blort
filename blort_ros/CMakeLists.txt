cmake_minimum_required(VERSION 2.8.3)
project(blort_ros)

find_package(catkin REQUIRED COMPONENTS roscpp rospy cv_bridge image_transport geometry_msgs std_msgs sensor_msgs tf dynamic_reconfigure blort message_generation)
find_package(OpenCV REQUIRED)
find_package(roslaunch REQUIRED)
find_package(Boost REQUIRED COMPONENTS signals)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(FILES
  NameTypeValue.msg
  ObjectPoseList.msg
  ObjectPose.msg
  TrackerConfidences.msg
)

## Generate services in the 'srv' folder
add_service_files(FILES
  EstimatePose.srv
  RecoveryCall.srv
  SetCameraInfo.srv
  TrackerCommand.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/detector.cfg # Is this needed?
  cfg/Detector.cfg
  cfg/tracker.cfg # Is this needed?
  cfg/Tracker.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS blort
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(learnsifts
  src/nodes/learnsifts.cpp
)
target_link_libraries(learnsifts
  siftgpu
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(gltracker_node
  src/nodes/tracker_node.cpp
  src/trackerinterface.hpp
  src/gltracker.cpp
)
add_dependencies(gltracker_node ${PROJECT_NAME}_gencfg)
target_link_libraries(gltracker_node
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(gldetector_node
  src/nodes/detector_node.cpp
  src/gldetector.cpp
)
add_dependencies(gldetector_node ${PROJECT_NAME}_gencfg)
target_link_libraries(gldetector_node
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME}
  src/pose_util.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(pose2Tf
  src/nodes/pose2Tf.cpp
) 
target_link_libraries(pose2Tf
  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
)

add_executable(pose2Tf_repeat
  src/nodes/pose2Tf_repeat.cpp
)
target_link_libraries(pose2Tf_repeat
  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
)

if(roslaunch_VERSION GREATER 1.9.45)
  roslaunch_add_file_check(
    learnsifts.launch
    singleshot_disparity.launch
    singleshot_histogram.launch
    singleshot_template.launch
    tracking_disparity_and_histogram.launch
    tracking_disparity.launch
    tracking_histogram.launch
    tracking_histogram_with_kinect.launch
    tracking_histogram_with_left_stereo.launch
    tracking.launch
    view_control.launch
  )
endif(roslaunch_VERSION GREATER 1.9.45)

#############
## Install ##
#############

install(TARGETS
  learnsifts gltracker_node gldetector_node ${PROJECT_NAME} pose2Tf pose2Tf_repeat
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

