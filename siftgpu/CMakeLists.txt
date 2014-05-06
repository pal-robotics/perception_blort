cmake_minimum_required(VERSION 2.8.3)
project(siftgpu)

find_package(catkin REQUIRED COMPONENTS
#	libglew
#	libglew-dev
#	libdevil-dev
)

find_package(PkgConfig)
pkg_check_modules(GLEW REQUIRED glew)

include_directories(include)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS libglew
)


add_library(${PROJECT_NAME} src/CLTexImage.cpp
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

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGEdd_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
