cmake_minimum_required(VERSION 2.8.3)
project(blort)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs geometry_msgs tf siftgpu)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_library(RT REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES module_glwindow module_tomgine module_threadobject module_tracker module_recognizer3d
  CATKIN_DEPENDS siftgpu
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_PATH}
  ${GLUT_INCLUDE_DIR}
)

add_library(module_glwindow
  src/GLWindow/GLXWindowGetEvent.cpp 
  src/GLWindow/GLXWindow.cpp 
  src/GLWindow/glxhidingwindow.cpp
)
set_target_properties(module_glwindow PROPERTIES COMPILE_FLAGS "-DLINUX")
target_link_libraries(module_glwindow
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${RT_LIBRARY}
)

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
set_target_properties(module_tomgine PROPERTIES COMPILE_FLAGS "-DLINUX -DGL_GLEXT_PROTOTYPES")
target_link_libraries(module_tomgine
  module_glwindow
  ${OpenCV_LIBRARIES}
)

add_library(module_threadobject
    src/ThreadObject/TomGineThread.cpp 
    src/ThreadObject/Thread.cpp 
    src/ThreadObject/RecognizerThread.cpp 
    src/ThreadObject/MutexClass.cpp 
    src/ThreadObject/EventClass.cpp 
    src/ThreadObject/CameraThread.cpp 
)
target_link_libraries(module_threadobject
  module_tomgine
  module_recognizer3d
)

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
set_target_properties(module_tracker PROPERTIES COMPILE_FLAGS "-DGL_GLEXT_PROTOTYPES")
target_link_libraries(module_tracker
  module_tomgine
  module_threadobject
  module_glwindow
)

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
    src/blort/pal_util.cpp
)
target_link_libraries(module_recognizer3d
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS
  module_glwindow module_tomgine module_threadobject module_tracker module_recognizer3d
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

