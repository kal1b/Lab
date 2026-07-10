cmake_minimum_required(VERSION 3.0.2)
project(parallel_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  pcl_ros
  dynamic_reconfigure
  tf2_ros
  nav_msgs
  mavros_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs pcl_ros tf2_ros nav_msgs mavros_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# State estimator
add_executable(kalman_filter_node src/nodes/kalman_filter_node.cpp)
add_dependencies(kalman_filter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kalman_filter_node ${catkin_LIBRARIES})

# Dynamic System Modulation command limiter
add_executable(dsm_node src/nodes/dsm_node.cpp)
add_dependencies(dsm_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dsm_node ${catkin_LIBRARIES})

# Local VFH planner
add_executable(vfh_node src/nodes/vfh_node.cpp)
add_dependencies(vfh_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vfh_node ${catkin_LIBRARIES})

# PX4 Offboard bridge
add_executable(offboard_control_node src/nodes/offboard_control_node.cpp)
add_dependencies(offboard_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(offboard_control_node ${catkin_LIBRARIES})
