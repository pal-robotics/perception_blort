cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()
#add dynamic reconfigure api

rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

rosbuild_add_executable(learnsifts src/nodes/learnsifts.cpp)
target_link_libraries(learnsifts siftgpu ${OpenCV_INCLUDE_DIRS})
target_link_libraries(learnsifts module_tomgine module_glwindow module_tracker module_threadobject module_recognizer3d)

rosbuild_add_executable(gltracker_node src/nodes/tracker_node.cpp
                                       src/trackerinterface.hpp
                                       src/gltracker.cpp)
target_link_libraries(gltracker_node ${OpenCV_INCLUDE_DIRS}
                                   module_tomgine 
                                   module_tracker
                                   module_glwindow
                                  GLU GL glut X11 rt)

rosbuild_add_executable(gldetector_node src/nodes/detector_node.cpp
                                      src/gldetector.cpp)
target_link_libraries(gldetector_node ${OpenCV_INCLUDE_DIRS}
                                   module_tracker
                                   module_tomgine 
                                   siftgpu 
                                   module_recognizer3d)


rosbuild_add_library(${PROJECT_NAME} src/pose_util.cpp)

rosbuild_add_executable(pose2Tf src/nodes/pose2Tf.cpp) 
target_link_libraries(pose2Tf ${PROJECT_NAME})

rosbuild_add_executable(pose2Tf_repeat src/nodes/pose2Tf_repeat.cpp)
target_link_libraries(pose2Tf_repeat ${PROJECT_NAME})


#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_boost_directories()
rosbuild_link_boost(gltracker_node signals)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
