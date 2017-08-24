SET(HAVE_TOON FALSE) #Georg Klein's M-estimator for incremental motion
SET(HAVE_OPENCV FALSE) #five-point algorithm to estimate incremental motion
SET(USE_SYSTEM_OPENCV TRUE) # Set False if you want to build this package using independent OpenCV instead of libopencv-dev, will be ignored if HAVE_OPENCV is false
SET(HAVE_FIVEPOINT_NGHIAHO FALSE) #use Nghia Ho's five point implementation
# project
SET(PROJECT_NAME viso2)
project (${PROJECT_NAME})

cmake_minimum_required (VERSION 2.8.3)
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()
IF(HAVE_TOON)
add_definitions(-DUSE_TOON)
ENDIF()
IF(HAVE_OPENCV)
add_definitions(-DUSE_OPENCV)
ENDIF()
IF(HAVE_OPENCV AND HAVE_FIVEPOINT_NGHIAHO)
add_definitions(-DUSE_NGHIAHO)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
SET(CMAKE_VERBOSE_MAKEFILE OFF)

# Set build flags, set ARM_ARCHITECTURE environment variable on Odroid
SET(CMAKE_CXX_FLAGS "-Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
IF(DEFINED ENV{ARM_ARCHITECTURE})
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -march=armv7-a")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")  
ENDIF()
IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")
MESSAGE("CMAKE_CXX_FLAGS:" ${CMAKE_CXX_FLAGS})

# use sse3 instruction set
SET(CMAKE_CXX_FLAGS "-msse3")

include_directories(include/viso2)

IF(HAVE_TOON)
  SET(TOON_INCLUDE_DIR $ENV{HOME}/TooN)
  include_directories(${TOON_INCLUDE_DIR})
ENDIF()

# add opencv libraries for 5 point algorithm
IF(HAVE_OPENCV)
  IF(USE_SYSTEM_OPENCV)
    FIND_PACKAGE(OpenCV REQUIRED)
  ELSE() 
    SET(OPENCV_INCLUDE_DIR $ENV{HOME}/OpenCV/local_install/include)
    SET(OPENCV_LIBRARY_DIR $ENV{HOME}/OpenCV/local_install/lib)
    set(OpenCV_LIBS opencv_core opencv_features2d opencv_flann opencv_gpu
               opencv_highgui opencv_imgproc opencv_calib3d opencv_video)
    link_directories(${OPENCV_LIBRARY_DIR})
  ENDIF()

  LIST(APPEND LINK_LIBS ${OpenCV_LIBS})
  include_directories(${OpenCV_INCLUDE_DIRS})
ENDIF()

IF(HAVE_OPENCV AND HAVE_FIVEPOINT_NGHIAHO)
include_directories(5point_NghiaHo)
ENDIF()

# sources of library
# FILE(GLOB LIBVISO2_SRC_FILES "viso2/*.cpp")
LIST(APPEND SOURCEFILES
src/filter.cpp
src/matcher.cpp
src/matrix.cpp
include/viso2/p_match.h

#include/viso2/timer.h
#src/reconstruction.cpp

src/triangle.cpp
src/viso.cpp
src/viso_mono.cpp
src/viso_stereo.cpp
)

IF(HAVE_TOON)
LIST(APPEND SOURCEFILES
include/viso2/MEstimator.h
src/TrackerData.cpp)
ENDIF()

IF(HAVE_OPENCV)
LIST(APPEND SOURCEFILES
src/five-point.cpp
src/ptsetreg.cpp)
ENDIF()

IF(HAVE_OPENCV AND HAVE_FIVEPOINT_NGHIAHO)
LIST(APPEND SOURCEFILES
5point_NghiaHo/5point.cpp
5point_NghiaHo/Mblock.h
5point_NghiaHo/Polynomial.h
5point_NghiaHo/Rpoly.cpp)
ENDIF()

# Create library
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCEFILES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${LINK_LIBS})

IF(HAVE_OPENCV)
ADD_EXECUTABLE(demo test/demo.cpp)
TARGET_LINK_LIBRARIES(demo ${PROJECT_NAME})
ENDIF()

# install headers and .so lib
INSTALL(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_PREFIX}/include/viso2 FILES_MATCHING PATTERN "*.h" )
INSTALL(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
