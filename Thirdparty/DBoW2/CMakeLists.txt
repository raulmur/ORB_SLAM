cmake_minimum_required(VERSION 2.8)
project(DBoW2)

SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
SET(CMAKE_VERBOSE_MAKEFILE OFF)
SET(USE_ROS true) # Set False if you want to build this package using independent OpenCV instead of ROS OpenCV
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

set(HDRS_DBOW2
  DBoW2/BowVector.h
  DBoW2/FORB.h 
  DBoW2/FClass.h       
  DBoW2/FeatureVector.h
  DBoW2/ScoringObject.h   
  DBoW2/TemplatedVocabulary.h)
set(SRCS_DBOW2
  DBoW2/BowVector.cpp
  DBoW2/FORB.cpp      
  DBoW2/FeatureVector.cpp
  DBoW2/ScoringObject.cpp)

set(HDRS_DUTILS
  DUtils/Random.h
  DUtils/Timestamp.h)
set(SRCS_DUTILS
  DUtils/Random.cpp
  DUtils/Timestamp.cpp)
IF(USE_ROS)
  find_package(OpenCV REQUIRED)
ELSE()
  set(OpenCV_INCLUDE_DIRS $ENV{HOME}/ScaViSLAM/svslocal/include)
  set(OpenCV_LIBS opencv_core opencv_features2d opencv_flann opencv_gpu
               opencv_nonfree opencv_highgui opencv_imgproc opencv_calib3d)
  link_directories($ENV{HOME}/ScaViSLAM/svslocal/lib)
ENDIF()

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${OpenCV_INCLUDE_DIRS})
add_library(DBoW2 SHARED ${SRCS_DBOW2} ${SRCS_DUTILS})
target_link_libraries(DBoW2 ${OpenCV_LIBS})

