cmake_minimum_required(VERSION 2.8.3)

project(dvo_core)

find_package(catkin REQUIRED COMPONENTS cv_bridge)
find_package(cmake_modules REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(Eigen3 REQUIRED)
find_package(sophus REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
#find_package(OpenCV 3 REQUIRED)
find_package(TBB REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dvo_core dvo_visualization
  DEPENDS OpenCV PCL
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -msse3 -msse4 -ftree-vectorize")

include_directories(
  "include"
  ${sophus_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(dvo_core 
    src/core/interpolation.cpp
    src/core/intrinsic_matrix.cpp
    src/core/least_squares.cpp
    src/core/math_sse.cpp
    src/core/rgbd_image.cpp
    src/core/rgbd_image_sse.cpp
    src/core/point_selection.cpp
    src/core/surface_pyramid.cpp
    src/core/weight_calculation.cpp
    
    src/util/histogram.cpp
    
    src/visualization/camera_trajectory_visualizer.cpp
    
    src/dense_tracking.cpp
    src/dense_tracking_impl.cpp
    src/dense_tracking_config.cpp
)

target_link_libraries(dvo_core 
    tbb
    boost_thread 
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_library(dvo_visualization
    src/visualization/async_point_cloud_builder.cpp
    src/visualization/point_cloud_aggregator.cpp
#    src/visualization/pcl_camera_trajetory_visualizer.cpp
)

target_link_libraries(dvo_visualization
  dvo_core
)
