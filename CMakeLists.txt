SET(TRACE FALSE) # record timing of operations
SET(MONO_SLAM TRUE) # if false stereo SLAM with or without IMU, if true, monocular SLAM
SET(USE_ROS TRUE) # use ROS for visualization or not
SET(USE_SYSTEM_OPENCV true) # Set False if you want to build this package using independent OpenCV instead of libopencv-dev

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
  ADD_DEFINITIONS(-DSLAM_OUTPUT_VISO2) #output the libviso2 stereo odometry result
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

IF(USE_SYSTEM_OPENCV)
  FIND_PACKAGE(OpenCV REQUIRED) # do not put findOpencv.cmake in cmake_module_path which causes an error
ELSE()
  SET(OPENCV_INCLUDE_DIR $ENV{HOME}/OpenCV/local_install/include)
  SET(OPENCV_LIBRARY_DIR $ENV{HOME}/OpenCV/local_install/lib)
  set(OpenCV_LIBS opencv_core opencv_features2d opencv_flann opencv_gpu
               opencv_nonfree opencv_highgui opencv_imgproc opencv_calib3d opencv_video)
  link_directories(${OPENCV_LIBRARY_DIR})
ENDIF()

SET(G2O_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/local_install/include) # g2o
SET(G2O_LIBRARY_DIR ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/local_install/lib)

SET(SOPHUS_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/Thirdparty/Sophus) # sophus

SET(VIKIT_COMMON_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/Thirdparty/vikit_common/include)
SET(VIKIT_COMMON_LIBRARIES ${PROJECT_SOURCE_DIR}/Thirdparty/vikit_common/lib/libvikit_common.so)

IF(USE_ROS)
  FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp roslib cmake_modules tf cv_bridge)
  catkin_package(
      DEPENDS Eigen OpenCV Cholmod SuiteSparse Boost
      CATKIN_DEPENDS roscpp roslib tf cv_bridge
      INCLUDE_DIRS include
      LIBRARIES ${PROJECT_NAME}
  )
ELSE()
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
${G2O_INCLUDE_DIR}
${SOPHUS_INCLUDE_DIR}
${OPENCV_INCLUDE_DIR}
${VIKIT_COMMON_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}/Thirdparty/libviso2/include
${PROJECT_SOURCE_DIR}/Thirdparty/g2o_EXTERNAL # ceres autodiff
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

g2o_types/anchored_points.cpp
g2o_types/IMU_constraint.cpp
g2o_types/scale_solver.cpp
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
${G2O_LIBRARY_DIR}/libg2o_core.so
${G2O_LIBRARY_DIR}/libg2o_solver_cholmod.so
${G2O_LIBRARY_DIR}/libg2o_solver_dense.so
${G2O_LIBRARY_DIR}/libg2o_stuff.so
${G2O_LIBRARY_DIR}/libg2o_solver_csparse.so
${G2O_LIBRARY_DIR}/libg2o_csparse_extension.so
${G2O_LIBRARY_DIR}/libg2o_types_sba.so
${G2O_LIBRARY_DIR}/libg2o_types_sim3.so
${G2O_LIBRARY_DIR}/libg2o_types_slam3d.so

${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/libviso2/lib/libviso2.so
${VIKIT_COMMON_LIBRARIES}
)

IF(USE_ROS)
  INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
  LIST(APPEND LINK_LIBS ${catkin_LIBRARIES})
ENDIF()

ADD_LIBRARY(${PROJECT_NAME}  SHARED ${SOURCEFILES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${LINK_LIBS})

add_executable(test_orbslam src/main.cc)
TARGET_LINK_LIBRARIES(test_orbslam ${PROJECT_NAME})
