cmake_minimum_required(VERSION 2.8.3)

project(dvo_slam)

LIST(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules")

find_package(catkin REQUIRED COMPONENTS roscpp dynamic_reconfigure dvo_core dvo_ros sensor_msgs image_transport message_filters cv_bridge visualization_msgs interactive_markers message_generation geometry_msgs)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(G2O REQUIRED)
#find_package(sophus REQUIRED)
#find_package(Boost REQUIRED thread)
find_package(OpenCV 2.4.8 REQUIRED)
find_package(TBB REQUIRED)

LIST(APPEND CMAKE_CXX_FLAGS "-std=c++11")

if(G2O_FOUND)
  message(STATUS "Found G2O")
else()
  message(ERROR "G2O missing")
endif()

# dynamic reconfigure
generate_dynamic_reconfigure_options(cfg/GraphVisualizer.cfg cfg/KeyframeSlam.cfg)

# messages
add_message_files(
  FILES
  PoseStampedArray.msg
)
generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dvo_slam
  DEPENDS OpenCV
  CATKIN_DEPENDS message_runtime
)

include_directories(
  "include"
  ${dvo_slam_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} 
  src/keyframe_tracker.cpp
  src/keyframe_graph.cpp
  src/keyframe_constraint_search.cpp
  src/camera_keyframe_tracking.cpp
  
  src/config.cpp
  
  src/tracking_result_evaluation.cpp
  src/local_map.cpp
  src/local_tracker.cpp
  
  src/constraints/constraint_proposal.cpp
  src/constraints/constraint_proposal_voter.cpp
  src/constraints/constraint_proposal_validator.cpp
  
  src/serialization/map_serializer_interface.cpp
  src/serialization/map_serializer.cpp
  
  src/visualization/graph_visualizer.cpp
  
  src/types.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${G2O_CORE_LIBRARY}
  ${G2O_SOLVER_DENSE}
  ${G2O_SOLVER_EIGEN}
  ${G2O_TYPES_SLAM3D}
  ${catkin_LIBRARIES}
)

#add_executable(camera_keyframe_tracker
#  src/camera_keyframe_tracker_node.cpp
#)

#target_link_libraries(camera_keyframe_tracker
#  ${PROJECT_NAME}
#)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
