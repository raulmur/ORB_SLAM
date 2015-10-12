# this program postprocess and evaluate the output of the program orbslam_dwo
SET(PROJECT_NAME DEVKIT)
PROJECT(${PROJECT_NAME})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.6)

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

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${EIGEN3_INCLUDE_DIR}
$ENV{HOME}/ScaViSLAM/svslocal/include # for sophus
)

LIST(APPEND LINK_LIBS
${EIGEN3_LIBS}
${OpenCV_LIBS}
)

add_executable(evaluate src/evaluate_odometry.cpp src/matrix.cpp src/postproc.cpp src/utils.cpp)
TARGET_LINK_LIBRARIES(evaluate ${LINK_LIBS})
