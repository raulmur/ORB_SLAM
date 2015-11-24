SET(TRACE TRUE)
SET(MONO_SLAM TRUE)
SET(USE_ROS TRUE)

SET(PROJECT_NAME orbslam_dwo)
PROJECT(${PROJECT_NAME})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.6)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
    "${PROJECT_SOURCE_DIR}/cmake_modules/")

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
ELSE()
  ADD_DEFINITIONS(-DSLAM_OUTPUT_VISO2)
ENDIF()
IF(USE_ROS)
  add_definitions(-DSLAM_USE_ROS)
ENDIF()

IF(CMAKE_BUILD_TYPE MATCHES Debug)
  ADD_DEFINITIONS(-DSLAM_DEBUG_OUTPUT)
ENDIF()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(Eigen3 REQUIRED)
find_package(Cholmod REQUIRED)
FIND_PACKAGE(SuiteSparse REQUIRED)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.45.0 COMPONENTS system thread REQUIRED)
SET(vikit_common_INCLUDE_DIRS $ENV{HOME}/SVO/rpg_vikit/vikit_common/include)
SET(vikit_common_LIBRARIES $ENV{HOME}/SVO/rpg_vikit/vikit_common/lib/libvikit_common.so)


SET(g2o_INCLUDE_DIR $ENV{HOME}/ScaViSLAM/svslocal/include) # g2o, sophus, maybe opencv

IF(USE_ROS)
  FIND_PACKAGE(OpenCV REQUIRED) # do not put findOpencv.cmake in cmake_module_path which causes an error
  FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp roslib cmake_modules tf cv_bridge)
  SET(g2o_LIBRARY_DIR $ENV{HOME}/ScaViSLAM/External/g2o/lib) # do not put local opencv and g2o libs together as it confuses catkin
  catkin_package(
      DEPENDS Eigen OpenCV Cholmod SuiteSparse Boost
      CATKIN_DEPENDS roscpp roslib tf cv_bridge
      INCLUDE_DIRS include
      LIBRARIES ${PROJECT_NAME}
  )
ELSE()
  set(OpenCV_LIBS opencv_core opencv_features2d opencv_flann opencv_gpu
               opencv_nonfree opencv_highgui opencv_imgproc opencv_calib3d opencv_video)
  SET(g2o_LIBRARY_DIR $ENV{HOME}/ScaViSLAM/svslocal/lib)
  link_directories(${g2o_LIBRARY_DIR})
  SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)
ENDIF()

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${CHOLMOD_INCLUDE_DIR}
${CSPARSE_INCLUDE_DIR}
${Boost_INCLUDE_DIRS}
${g2o_INCLUDE_DIR}
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
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
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
viso2/ptsetreg.cpp
viso2/five-point.cpp
g2o_types/global.h
g2o_types/maths_utils.cpp
g2o_types/eigen_utils.h
g2o_types/eigen_utils.cpp
g2o_types/anchored_points.cpp
g2o_types/IMU_constraint.cpp
g2o_types/timegrabber.cpp
)

IF(USE_ROS)
  LIST(APPEND SOURCEFILES src/MapPublisher.cc)
ENDIF()

LIST(APPEND LINK_LIBS
${OpenCV_LIBS}
cholmod
csparse
cxsparse
${Boost_LIBRARIES}
${g2o_LIBRARY_DIR}/libg2o_core.so
${g2o_LIBRARY_DIR}/libg2o_solver_cholmod.so
${g2o_LIBRARY_DIR}/libg2o_solver_dense.so
${g2o_LIBRARY_DIR}/libg2o_stuff.so
${g2o_LIBRARY_DIR}/libg2o_solver_csparse.so
${g2o_LIBRARY_DIR}/libg2o_csparse_extension.so
${g2o_LIBRARY_DIR}/libg2o_types_sba.so
${g2o_LIBRARY_DIR}/libg2o_types_sim3.so
${g2o_LIBRARY_DIR}/libg2o_types_slam3d.so
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${vikit_common_LIBRARIES} #for logging and timing
)

IF(USE_ROS)
  INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
  LIST(APPEND LINK_LIBS ${catkin_LIBRARIES})
ENDIF()

ADD_LIBRARY(${PROJECT_NAME}  SHARED ${SOURCEFILES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${LINK_LIBS})

add_executable(test_orbslam src/main.cc)
TARGET_LINK_LIBRARIES(test_orbslam ${PROJECT_NAME})
