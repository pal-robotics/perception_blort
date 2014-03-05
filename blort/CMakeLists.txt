cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
# set(ROS_BUILD_TYPE Release)
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

set(SIFTGPU_INC /usr/local/include/SiftGPU CACHE PATH "Path to SiftGPU header files")
set(SIFTGPU_LIB /usr/local/lib CACHE PATH "Path to SiftGPU library")
include_directories(${SIFTGPU_INC})
include_directories("${PROJECT_SOURCE_DIR}/src")


rosbuild_add_library(module_glwindow
    src/GLWindow/GLXWindowGetEvent.cpp 
    src/GLWindow/GLXWindow.cpp 
    src/GLWindow/glxhidingwindow.cpp
)
rosbuild_add_compile_flags(module_glwindow -DLINUX)

target_link_libraries(module_glwindow GLU GL glut X11 rt)

rosbuild_add_library(module_tomgine
    src/TomGine/tgMathlib.cpp
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
rosbuild_add_compile_flags(module_tomgine -DLINUX)
rosbuild_add_compile_flags(module_tomgine -DGL_GLEXT_PROTOTYPES)
target_link_libraries(module_tomgine GLU GL)

rosbuild_add_library(module_threadobject
    src/ThreadObject/TomGineThread.cpp 
    src/ThreadObject/Thread.cpp 
    src/ThreadObject/RecognizerThread.cpp 
    src/ThreadObject/MutexClass.cpp 
    src/ThreadObject/EventClass.cpp 
    src/ThreadObject/CameraThread.cpp 
)
target_link_libraries(module_threadobject module_tomgine module_recognizer3d)

rosbuild_add_library(module_tracker 
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
rosbuild_add_compile_flags(module_tracker -DGL_GLEXT_PROTOTYPES)
target_link_libraries(module_tracker module_tomgine module_threadobject module_glwindow)

rosbuild_add_library(module_recognizer3d
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

