cmake_minimum_required(VERSION 2.8.3)
project(blort_ros)

find_package(catkin REQUIRED COMPONENTS
  blort
  roscpp
  geometry_msgs
  tf
  sensor_msgs
  dynamic_reconfigure
  image_transport
  cv_bridge
  message_generation
)

find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  NameTypeValue.msg
  ObjectPoseList.msg
  ObjectPose.msg
  TrackerConfidences.msg
  TrackerResults.msg
)

add_service_files(
  FILES
  EstimatePose.srv
  RecoveryCall.srv
  SetCameraInfo.srv
  TrackerCommand.srv
)

generate_messages(
 DEPENDENCIES geometry_msgs sensor_msgs
)

generate_dynamic_reconfigure_options(
 cfg/Tracker.cfg
 cfg/Detector.cfg
)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
)

add_executable(learnsifts src/nodes/learnsifts.cpp)
target_link_libraries(learnsifts siftgpu ${catkin_LIBRARIES})
target_link_libraries(learnsifts module_tomgine module_glwindow module_tracker module_threadobject module_recognizer3d)

add_executable(gltracker_node src/nodes/tracker_node.cpp
                              src/gltracker.cpp)
target_link_libraries(gltracker_node module_tomgine
                                   module_tracker
                                   module_glwindow
                                   ${catkin_LIBRARIES}
                                   GLU GL glut X11 rt)

add_executable(gldetector_node src/nodes/detector_node.cpp
                               src/gldetector.cpp)
target_link_libraries(gldetector_node ${catkin_LIBRARIES}
                                   module_tracker
                                   module_tomgine
                                   siftgpu
                                   module_recognizer3d)

add_library(${PROJECT_NAME} src/pose_util.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS}})

add_executable(pose2Tf src/nodes/pose2Tf.cpp)
target_link_libraries(pose2Tf ${PROJECT_NAME}
                              ${catkin_LIBRARIES})

add_executable(pose2Tf_repeat src/nodes/pose2Tf_repeat.cpp)
target_link_libraries(pose2Tf_repeat ${PROJECT_NAME}
                                     ${catkin_LIBRARIES})

#############
## Install ##
#############

#install(PROGRAMS
#  scripts/my_python_script
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(TARGETS
        learnsifts
        gltracker_node
        gldetector_node
        ${PROJECT_NAME}
        pose2Tf
        pose2Tf_repeat
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch config Resources Tracker)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE
#)

#install(FILES
#  # myfile1
#  # myfile2
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
