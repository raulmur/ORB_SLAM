SET(PROJECT_NAME vio_g2o)
PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)

SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
SET(CMAKE_VERBOSE_MAKEFILE OFF)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

SET(USE_ROS False) # Set False if you want to build this package without Catkin
SET(USE_ORBSLAM_G2O false) # set true if you want to use the g2o created by Raul for ORBSLAM(2) or the g2o on github?
ADD_DEFINITIONS(-DMONO) #enable compiling monocular related g2o edges and vertices

# Set build flags. Set IS_ARM on odroid board as environment variable
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

# Add plain cmake packages 
FIND_PACKAGE(Eigen REQUIRED)
FIND_PACKAGE(vio_common REQUIRED)
FIND_PACKAGE(Sophus REQUIRED) # the sophus (commit number b474f0) repo of Steven Lovegrove's github account is preferred

IF(USE_ORBSLAM_G2O)
  ADD_DEFINITIONS(-DUSE_SLIM_G2O)
#  SET(ORBSLAM_G2O_PATH) #provide this variable as an CMAKE argument
  SET(G2O_INCLUDE_DIR ${ORBSLAM_G2O_PATH}/g2o)
  SET(G2O_LIBS ${ORBSLAM_G2O_PATH}/g2o/lib/libg2o.so)
ELSE()
  find_package(Cholmod REQUIRED)
  FIND_PACKAGE(SuiteSparse REQUIRED)

  SET(G2O_INCLUDE_DIR ${G2O_INSTALL_PREFIX}/include)
  SET(G2O_LIBRARY_DIR ${G2O_INSTALL_PREFIX}/lib)
  LINK_DIRECTORIES(${G2O_LIBRARY_DIR})
  LIST(APPEND G2O_LIBRARIES
  cholmod
  csparse
  cxsparse
  g2o_core
  g2o_solver_cholmod
  g2o_solver_dense
  g2o_stuff
  g2o_solver_csparse
  g2o_csparse_extension
  g2o_types_sba
  g2o_types_sim3
  g2o_types_slam3d
  )
ENDIF()

# Include dirs
INCLUDE_DIRECTORIES(
  include
  include/g2o_EXTERNAL
  ${Eigen_INCLUDE_DIRS}
  ${vio_common_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIR}
)

IF(USE_ROS)
  FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp cmake_modules)
  LIST(APPEND INCLUDE_DIRECTORIES ${catkin_INCLUDE_DIRS})
  catkin_package(
    DEPENDS Eigen
    CATKIN_DEPENDS roscpp
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
  )
ELSE()
  SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
ENDIF()

# Set Sourcefiles
LIST(APPEND SOURCEFILES 
src/anchored_points.cpp
src/IMU_constraint.cpp
src/scale_solver.cpp
include/vio_g2o/transformations.h
)

LIST(APPEND LINK_LIBS
${G2O_LIBRARIES}
${vio_common_LIBRARIES}
)

# Create library
ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCEFILES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME}
  ${LINK_LIBS})

IF(USE_ROS)
  TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${catkin_LIBRARIES})
ENDIF()

# Tests
IF(FALSE)
set(OpenCV_INCLUDE_DIRS /usr/local/include)
set(OpenCV_LIBS opencv_core opencv_highgui opencv_imgproc opencv_features2d opencv_flann)
message ("opencv include dir " ${OpenCV_INCLUDE_DIRS})
include_directories(BEFORE ${OpenCV_INCLUDE_DIRS})
message("opencv include dirs are " ${OpenCV_INCLUDE_DIRS})
message("opencv libs are " ${OpenCV_LIBS})
ADD_EXECUTABLE(test_IMU test/test_IMU.cpp) # this depends on cv::FileStorage
TARGET_LINK_LIBRARIES(test_IMU ${PROJECT_NAME} ${OpenCV_LIBS})
ENDIF()

################################################################################
# Create the vio_g2oConfig.cmake file for other cmake projects.
IF(NOT USE_ROS)
  SET(vio_g2o_LIBRARIES ${PROJECT_SOURCE_DIR}/lib/libvio_g2o.so )
  SET(vio_g2o_LIBRARY_DIR ${PROJECT_SOURCE_DIR}/lib )
  SET(vio_g2o_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")
  CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/vio_g2oConfig.cmake.in
      ${CMAKE_CURRENT_BINARY_DIR}/vio_g2oConfig.cmake @ONLY IMMEDIATE )
  export( PACKAGE vio_g2o )

  INSTALL(DIRECTORY include/vio DESTINATION ${CMAKE_INSTALL_PREFIX}/include FILES_MATCHING PATTERN "*.h" )
  INSTALL(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
ENDIF()
