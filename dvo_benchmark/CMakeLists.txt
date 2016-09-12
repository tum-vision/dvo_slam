cmake_minimum_required(VERSION 2.8.3)

project(dvo_benchmark)

LIST(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules")

find_package(catkin REQUIRED COMPONENTS roscpp dvo_core dvo_ros dvo_slam)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(G2O REQUIRED)
find_package(OpenCV 2.4.8 REQUIRED)

LIST(APPEND CMAKE_CXX_FLAGS "-std=c++11")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dvo_benchmark
)

include_directories(
  "include"
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIR}
)


add_executable(benchmark
     src/benchmark_slam.cpp
)

#add_executable(experiment
#     src/experiment.cpp
#)

target_link_libraries(benchmark
#  vtkCommon
  ${catkin_LIBRARIES}
)

# create output directory - just convenience for our users
#file(MAKE_DIRECTORY "${PROJECT_SOURCE_DIR}/output")
#file(MAKE_DIRECTORY "${PROJECT_SOURCE_DIR}/output/video")

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
