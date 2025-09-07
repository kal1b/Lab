#include "parallel_planner/parallel_planner_node.h"

namespace parallel_planner {

ParallelPlannerNode::ParallelPlannerNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
  : nh_(nh), nh_private_(nh_private)
{
  ROS_INFO("ParallelPlannerNode constructed and running.");
  // инициализация подписок, паблишеров, параметров и т.п.
}

}

