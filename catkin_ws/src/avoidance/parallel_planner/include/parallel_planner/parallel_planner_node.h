#ifndef PARALLEL_PLANNER_NODE_H
#define PARALLEL_PLANNER_NODE_H

#include <ros/ros.h>

namespace parallel_planner {

class ParallelPlannerNode {
 public:
  ParallelPlannerNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

 private:
  void Initialize();  // вся инициализация: подписки, паблишеры, таймеры
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
};

}  // namespace parallel_planner

#endif  // PARALLEL_PLANNER_NODE_H

