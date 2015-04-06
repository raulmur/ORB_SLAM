cmake_minimum_required(VERSION 2.8.3)
project(orb_slam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  sensor_msgs
  image_transport
  cv_bridge
  g2o
)

#LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
#find_package(Cholmod REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package()

message(*************************)
message(${catkin_INCLUDE_DIRS})
message(*************************)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CHOLMOD_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}
)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

add_subdirectory(Thirdparty/DBoW2)

add_executable(${PROJECT_NAME}
  src/main.cc
  src/Tracking.cc
  src/LocalMapping.cc
  src/LoopClosing.cc
  src/ORBextractor.cc
  src/ORBmatcher.cc
  src/FramePublisher.cc
  src/Converter.cc
  src/MapPoint.cc
  src/KeyFrame.cc
  src/Map.cc
  src/MapPublisher.cc
  src/Optimizer.cc
  src/PnPsolver.cc
  src/Frame.cc
  src/KeyFrameDatabase.cc
  src/Sim3Solver.cc
  src/Initializer.cc
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBS}
#  cholmod
  DBoW2
#  g2o_core
#  g2o_solver_cholmod
#  g2o_solver_dense
#  g2o_stuff
#  g2o_types_sba
#  g2o_types_sim3
#  g2o_types_slam3d
)
