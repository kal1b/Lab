#include <ros/ros.h>
#include "parallel_planner/parallel_planner_node.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "parallel_planner_node");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Запуск полноценного узла
  parallel_planner::ParallelPlannerNode node(nh, nh_private);

  ros::AsyncSpinner spinner(2);  // если у тебя используются callback-и с mutex
  spinner.start();

  ros::waitForShutdown();
  return 0;
}

