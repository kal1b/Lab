#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <algorithm>
#include <cmath>

class DSMNode {
public:
  DSMNode()
      : nh_(), pnh_("~"), nav_cmd_received_(false), state_received_(false),
        last_output_valid_(false) {
    pnh_.param("max_linear_acc", max_linear_acc_, 1.0);
    pnh_.param("max_angular_acc", max_angular_acc_, 0.5);
    pnh_.param("max_linear_speed", max_linear_speed_, 2.0);
    pnh_.param("max_yaw_rate", max_yaw_rate_, 0.8);

    nav_cmd_sub_ = nh_.subscribe("navigation_cmd", 10, &DSMNode::navCmdCallback, this);
    state_sub_ = nh_.subscribe("state_estimate", 10, &DSMNode::stateCallback, this);
    control_pub_ = nh_.advertise<geometry_msgs::Twist>("control_cmd", 10);

    timer_ = nh_.createTimer(ros::Duration(0.02), &DSMNode::timerCallback, this);

    ROS_INFO("dsm_node: navigation_cmd -> control_cmd with acceleration limiting");
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber nav_cmd_sub_;
  ros::Subscriber state_sub_;
  ros::Publisher control_pub_;
  ros::Timer timer_;

  geometry_msgs::Twist latest_nav_cmd_;
  nav_msgs::Odometry latest_state_;
  geometry_msgs::Twist last_output_;
  bool nav_cmd_received_;
  bool state_received_;
  bool last_output_valid_;
  ros::Time last_time_;

  double max_linear_acc_;
  double max_angular_acc_;
  double max_linear_speed_;
  double max_yaw_rate_;

  static double clamp(double value, double min_value, double max_value) {
    return std::max(min_value, std::min(value, max_value));
  }

  static double limitRate(double desired, double previous, double max_delta) {
    return previous + clamp(desired - previous, -max_delta, max_delta);
  }

  static void limitVectorNorm(geometry_msgs::Vector3 &v, double max_norm) {
    const double norm = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (norm > max_norm && norm > 1e-6) {
      const double scale = max_norm / norm;
      v.x *= scale;
      v.y *= scale;
      v.z *= scale;
    }
  }

  void navCmdCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    latest_nav_cmd_ = *msg;
    nav_cmd_received_ = true;
  }

  void stateCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    latest_state_ = *msg;
    state_received_ = true;
  }

  void timerCallback(const ros::TimerEvent &) {
    if (!nav_cmd_received_ || !state_received_) {
      return;
    }

    const ros::Time now = ros::Time::now();
    double dt = 0.02;
    if (!last_time_.isZero()) {
      dt = (now - last_time_).toSec();
      if (dt <= 0.0 || dt > 0.5) {
        dt = 0.02;
      }
    }
    last_time_ = now;

    geometry_msgs::Twist previous;
    if (last_output_valid_) {
      previous = last_output_;
    } else {
      previous.linear = latest_state_.twist.twist.linear;
      previous.angular = latest_state_.twist.twist.angular;
      last_output_valid_ = true;
    }

    geometry_msgs::Twist cmd;
    const double max_dv = max_linear_acc_ * dt;
    const double max_dw = max_angular_acc_ * dt;

    cmd.linear.x = limitRate(latest_nav_cmd_.linear.x, previous.linear.x, max_dv);
    cmd.linear.y = limitRate(latest_nav_cmd_.linear.y, previous.linear.y, max_dv);
    cmd.linear.z = limitRate(latest_nav_cmd_.linear.z, previous.linear.z, max_dv);
    limitVectorNorm(cmd.linear, max_linear_speed_);

    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = limitRate(latest_nav_cmd_.angular.z, previous.angular.z, max_dw);
    cmd.angular.z = clamp(cmd.angular.z, -max_yaw_rate_, max_yaw_rate_);

    control_pub_.publish(cmd);
    last_output_ = cmd;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "dsm_node");
  DSMNode node;
  ros::spin();
  return 0;
}
