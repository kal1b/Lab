cmake_minimum_required(VERSION 3.0.2)
project(parallel_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  pcl_ros
  dynamic_reconfigure
  tf2_ros
  nav_msgs
)


catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs pcl_ros tf2_ros nav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Узел Kalman Filter
add_executable(kalman_filter_node src/nodes/kalman_filter_node.cpp)
add_dependencies(kalman_filter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kalman_filter_node ${catkin_LIBRARIES})

# Узел DSM
add_executable(dsm_node src/nodes/dsm_node.cpp)
add_dependencies(dsm_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dsm_node ${catkin_LIBRARIES})

# Узел VFH+
add_executable(vfh_node src/nodes/vfh_node.cpp)
add_dependencies(vfh_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vfh_node ${catkin_LIBRARIES})

