SET(PROJECT_NAME ORBSLAM_DWO)
PROJECT(${PROJECT_NAME})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.6)
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#rosbuild_init()
SET(TRACE TRUE)
SET(MONO_SLAM FALSE)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")
# use sse3 instruction set
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse3")
add_definitions(-DCV_SSE2=1)
MESSAGE ("sse3 ON")

# Set definitions
IF(TRACE)
  ADD_DEFINITIONS(-DSLAM_TRACE)
ENDIF()
IF(MONO_SLAM)
  ADD_DEFINITIONS(-DMONO)
ENDIF()
IF(CMAKE_BUILD_TYPE MATCHES Debug)
  ADD_DEFINITIONS(-DSLAM_DEBUG_OUTPUT)
ENDIF()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Cholmod REQUIRED)
FIND_PACKAGE(SuiteSparse REQUIRED)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.45.0 COMPONENTS system thread REQUIRED)
SET(vikit_common_INCLUDE_DIRS $ENV{HOME}/SVO/rpg_vikit/vikit_common/include)
SET(vikit_common_LIBRARIES $ENV{HOME}/SVO/rpg_vikit/vikit_common/lib/libvikit_common.so)

include_directories(
${PROJECT_SOURCE_DIR}
${EIGEN3_INCLUDE_DIR}
${CHOLMOD_INCLUDE_DIR}
${CSPARSE_INCLUDE_DIR}
${Boost_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}/include
$ENV{HOME}/ScaViSLAM/svslocal/include
${vikit_common_INCLUDE_DIRS}
)

LIST(APPEND SOURCEFILES
src/config.cpp
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FramePublisher.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/FeatureGrid.cpp
src/Map.cc
#src/MapPublisher.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/timegrabber.cpp
src/stereoSFM.cpp
src/MotionModel.cpp
src/utils.cpp
viso2/filter.cpp
viso2/matcher.cpp
viso2/matrix.cpp
#viso2/MEstimator.cpp
viso2/p_match.h
#viso2/timer.h
#viso2/TrackerData.cpp
viso2/triangle.cpp
viso2/viso.cpp
viso2/viso_stereo.cpp
g2o_types/global.h
g2o_types/maths_utils.cpp
g2o_types/eigen_utils.h
g2o_types/eigen_utils.cpp
g2o_types/anchored_points.cpp
g2o_types/IMU_constraint.cpp
)

LIST(APPEND LINK_LIBS
${OpenCV_LIBS}
${EIGEN3_LIBS}
cholmod
csparse
cxsparse
${Boost_LIBRARIES}
#$ENV{HOME}/ScaViSLAM/svslocal/lib/libviso2.so
$ENV{HOME}/ScaViSLAM/svslocal/lib/libg2o_core.so
$ENV{HOME}/ScaViSLAM/svslocal/lib/libg2o_solver_cholmod.so
$ENV{HOME}/ScaViSLAM/svslocal/lib/libg2o_solver_dense.so
$ENV{HOME}/ScaViSLAM/svslocal/lib/libg2o_stuff.so
$ENV{HOME}/ScaViSLAM/svslocal/lib/libg2o_solver_csparse.so
$ENV{HOME}/ScaViSLAM/svslocal/lib/libg2o_csparse_extension.so
$ENV{HOME}/ScaViSLAM/svslocal/lib/libg2o_types_sba.so
$ENV{HOME}/ScaViSLAM/svslocal/lib/libg2o_types_sim3.so
$ENV{HOME}/ScaViSLAM/svslocal/lib/libg2o_types_slam3d.so

${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
#${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_core.so
#${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_solver_cholmod.so
#${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_solver_dense.so
#${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_stuff.so
#${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_sba.so
#${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_sim3.so
#${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_slam3d.so
${vikit_common_LIBRARIES} #for logging and timing
)

ADD_LIBRARY(${PROJECT_NAME}  SHARED ${SOURCEFILES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${LINK_LIBS})


add_executable(test_orbslam src/main.cc)
TARGET_LINK_LIBRARIES(test_orbslam ${PROJECT_NAME})
