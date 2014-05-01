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
target_link_libraries(learnsifts siftgpu ${OpenCV_INCLUDE_DIRS} ${catkin_LIBRARIES})
target_link_libraries(learnsifts module_tomgine module_glwindow module_tracker module_threadobject module_recognizer3d)

add_executable(gltracker_node src/nodes/tracker_node.cpp
                              src/trackerinterface.hpp
                              src/gltracker.cpp)
target_link_libraries(gltracker_node ${OpenCV_INCLUDE_DIRS}
                                   module_tomgine 
                                   module_tracker
                                   module_glwindow
                                   ${catkin_LIBRARIES}
                                  GLU GL glut X11 rt)

add_executable(gldetector_node src/nodes/detector_node.cpp
                               src/gldetector.cpp)
target_link_libraries(gldetector_node ${OpenCV_INCLUDE_DIRS}
                                   ${catkin_LIBRARIES}
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
