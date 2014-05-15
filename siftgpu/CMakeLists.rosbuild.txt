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

INCLUDE(FindPkgConfig)

pkg_check_modules(GLEW REQUIRED glew)

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(${PROJECT_NAME} src/CLTexImage.cpp
                                     src/CuTexImage.cpp
                                     src/FrameBufferObject.cpp
                                     src/GlobalUtil.cpp
                                     src/GLTexImage.cpp
                                     src/ProgramCG.cpp
                                     src/ProgramCL.cpp
                                     src/ProgramGLSL.cpp
                                     src/ProgramGPU.cpp
                                     src/PyramidCL.cpp
                                     src/PyramidCU.cpp
                                     src/PyramidGL.cpp
                                     src/ShaderMan.cpp
                                     src/SiftGPU.cpp
                                     src/SiftMatch.cpp
                                     src/SiftMatchCU.cpp
                                     src/SiftPyramid.cpp)

target_link_libraries(${PROJECT_NAME} ${GLEW_LIBRARIES} IL)



