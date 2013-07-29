cmake_minimum_required(VERSION 2.8.3)
project(siftgpu)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

find_package(GLEW REQUIRED)
find_package(DevIL REQUIRED)

###################################
## catkin specific configuration ##
###################################

## Catkin package definition
## INCLUDE_DIRS: include directory
## LIBRARIES: libraries that are provided
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${GLEW_INCLUDE_DIRS}
  ${IL_INCLUDE_DIR}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
  src/CLTexImage.cpp
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
  src/SiftPyramid.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${IL_LIBRARIES}
)

#############
## Install ##
#############

# Install header files
install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install libraries
install(FILES ${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

