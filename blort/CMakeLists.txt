cmake_minimum_required(VERSION 2.8.3)
project(blort)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf
  sensor_msgs
  siftgpu
)

find_package(OpenCV REQUIRED)

set(SIFTGPU_INC /usr/local/include/SiftGPU CACHE PATH "Path to SiftGPU header files")

include_directories(${catkin_INCLUDE_DIRS} ${SIFTGPU_INC} 
                    "${PROJECT_SOURCE_DIR}/src" "include/blort" include)

catkin_package(
  INCLUDE_DIRS include "${PROJECT_SOURCE_DIR}/src" "include/blort" ${catkin_INCLUDE_DIRS}
)

add_definitions(-DLINUX)

add_library(module_glwindow
    src/GLWindow/GLXWindowGetEvent.cpp 
    src/GLWindow/GLXWindow.cpp 
    src/GLWindow/glxhidingwindow.cpp
)
target_link_libraries(module_glwindow GLU GL glut X11 rt)

add_library(module_tomgine
    src/TomGine/tgVector3.cpp 
    src/TomGine/tgTimer.cpp 
    src/TomGine/tgTexture.cpp 
    src/TomGine/tgShapeCreator.cpp 
    src/TomGine/tgShader.cpp 
    src/TomGine/tgRenderModel.cpp 
    src/TomGine/tgQuaternion.cpp 
    src/TomGine/tgPose.cpp 
    src/TomGine/tgPlot2D.cpp 
    src/TomGine/tgModelLoader.cpp 
    src/TomGine/tgModel.cpp 
    src/TomGine/tgMatrix3.cpp 
    src/TomGine/tgMaterial.cpp 
    src/TomGine/tgLighting.cpp 
    src/TomGine/tgLabel.cpp 
    src/TomGine/tgImageProcessor.cpp 
    src/TomGine/tgGUI.cpp 
    src/TomGine/tgFrustum.cpp 
    src/TomGine/tgFont.cpp 
    src/TomGine/tgErrorMetric.cpp 
    src/TomGine/tgError.cpp 
    src/TomGine/tgEngine.cpp 
    src/TomGine/tgCollission.cpp 
    src/TomGine/tgCamera.cpp 
    src/TomGine/ply.c 
)
set_target_properties(module_tomgine PROPERTIES COMPILE_FLAGS -DLINUX)
set_target_properties(module_tomgine PROPERTIES COMPILE_FLAGS -DGL_GLEXT_PROTOTYPES)
target_link_libraries(module_tomgine GLU GL ${OpenCV_LIBS})

add_library(module_threadobject
    src/ThreadObject/TomGineThread.cpp 
    src/ThreadObject/Thread.cpp 
    src/ThreadObject/RecognizerThread.cpp 
    src/ThreadObject/MutexClass.cpp 
    src/ThreadObject/EventClass.cpp 
    src/ThreadObject/CameraThread.cpp 
)
target_link_libraries(module_threadobject module_tomgine module_recognizer3d)

add_library(module_tracker 
    src/Tracker/TrackerThread.cpp 
    src/Tracker/TrackerModel.cpp 
    src/Tracker/Tracker.cpp 
    src/Tracker/Timer.cpp 
    src/Tracker/TextureTracker.cpp 
    src/Tracker/Texture.cpp 
    src/Tracker/ShaderUtils.cpp 
    src/Tracker/Shader.cpp 
    src/Tracker/Resources.cpp 
    src/Tracker/Predictor.cpp 
    src/Tracker/ply.c 
    src/Tracker/Particle.cpp 
    src/Tracker/Noise.cpp 
    src/Tracker/myPredictor.cpp 
    src/Tracker/ModelLoader.cpp 
    src/Tracker/ModelEntry.cpp 
    src/Tracker/ImageProcessor.cpp 
    src/Tracker/EdgeTracker.cpp 
    src/Tracker/Distribution.cpp 
    src/Tracker/CDataFile.cpp 
    src/Tracker/CameraThread.cpp 
    src/blort/pal_util.cpp
)
set_target_properties(module_tracker PROPERTIES COMPILE_FLAGS -DGL_GLEXT_PROTOTYPES)
target_link_libraries(module_tracker module_tomgine module_threadobject module_glwindow)

add_library(module_recognizer3d
    src/Recognizer3D/Vector2.cc 
    src/Recognizer3D/SPolygon.cc 
    src/Recognizer3D/SDraw.cc 
    src/Recognizer3D/Recognizer3D.cpp 
    src/Recognizer3D/PoseCv.cc 
    src/Recognizer3D/ODetect3D.cc 
    src/Recognizer3D/Object3D.cc 
    src/Recognizer3D/ModelObject3D.cc 
    src/Recognizer3D/Math.cc 
    src/Recognizer3D/KeypointDescriptor.cc 
    src/Recognizer3D/Keypoint.cc 
    src/Recognizer3D/Geometry.cc 
    src/Recognizer3D/Except.cc 
    src/Recognizer3D/DetectSIFT.cc 
    src/Recognizer3D/DetectGPUSIFT.cc 
    src/Recognizer3D/Definitions.cc 
    src/Recognizer3D/ConfigFile.cc 
    src/Recognizer3D/Config.cc 
    src/Recognizer3D/CodebookEntry.cc 
)
target_link_libraries(module_recognizer3d ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS 
		module_glwindow
		module_tomgine
		module_threadobject
		module_tracker 
		module_recognizer3d
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
