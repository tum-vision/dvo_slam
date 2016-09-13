cmake_minimum_required(VERSION 2.8.3)

project(dvo_ros)

find_package(catkin REQUIRED COMPONENTS roscpp tf tf_conversions eigen_conversions dynamic_reconfigure dvo_core sensor_msgs image_transport message_filters cv_bridge visualization_msgs interactive_markers pcl_ros)
find_package(cmake_modules REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(Eigen3 REQUIRED)
#find_package(sophus REQUIRED)
#find_package(Boost REQUIRED thread)
#find_package(OpenCV 3 REQUIRED)
#find_package(TBB REQUIRED)
 
# dynamic reconfigure
generate_dynamic_reconfigure_options(cfg/CameraDenseTracker.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dvo_ros
  DEPENDS PCL
)

include_directories(
  "include"
  ${dvo_ros_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

message(STATUS "hello my dear ${catkin_LIBRARIES}")

add_library(${PROJECT_NAME} 
    src/camera_base.cpp
    src/camera_dense_tracking.cpp
    
    src/visualization/ros_camera_trajectory_visualizer.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_library(camera_tracker_nodelet
    src/camera_tracker_nodelet.cpp
)

target_link_libraries(camera_tracker_nodelet
    ${PROJECT_NAME}
)

add_dependencies(camera_tracker_nodelet ${PROJECT_NAME}_gencfg)

add_executable(camera_tracker
     src/camera_tracker_node.cpp
)

target_link_libraries(camera_tracker
    ${PROJECT_NAME}
)

add_dependencies(camera_tracker ${PROJECT_NAME}_gencfg)
