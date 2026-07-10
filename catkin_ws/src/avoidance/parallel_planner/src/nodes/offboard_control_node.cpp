#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>

class OffboardControlNode {
public:
  OffboardControlNode()
      : nh_(), pnh_("~"), connected_(false), armed_(false), offboard_(false),
        odom_received_(false), control_received_(false) {
    pnh_.param("target_altitude", target_altitude_, 2.0);
    pnh_.param("takeoff_speed", takeoff_speed_, 0.6);
    pnh_.param("command_timeout", command_timeout_, 0.5);
    pnh_.param("offboard_warmup_cycles", offboard_warmup_cycles_, 120);

    state_sub_ = nh_.subscribe("mavros/state", 10, &OffboardControlNode::stateCallback, this);
    odom_sub_ = nh_.subscribe("mavros/local_position/odom", 10, &OffboardControlNode::odomCallback, this);
    control_sub_ = nh_.subscribe("control_cmd", 10, &OffboardControlNode::controlCallback, this);

    setpoint_pub_ = nh_.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 20);

    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    timer_ = nh_.createTimer(ros::Duration(0.05), &OffboardControlNode::timerCallback, this);

    ROS_INFO("offboard_control_node: control_cmd -> mavros/setpoint_velocity/cmd_vel_unstamped");
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber state_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber control_sub_;
  ros::Publisher setpoint_pub_;
  ros::ServiceClient arming_client_;
  ros::ServiceClient set_mode_client_;
  ros::Timer timer_;

  bool connected_;
  bool armed_;
  bool offboard_;
  bool odom_received_;
  bool control_received_;

  double current_altitude_ = 0.0;
  double target_altitude_;
  double takeoff_speed_;
  double command_timeout_;
  int offboard_warmup_cycles_;
  int published_cycles_ = 0;

  geometry_msgs::Twist latest_control_cmd_;
  ros::Time last_control_stamp_;
  ros::Time last_request_stamp_;

  void stateCallback(const mavros_msgs::State::ConstPtr &msg) {
    connected_ = msg->connected;
    armed_ = msg->armed;
    offboard_ = (msg->mode == "OFFBOARD");
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    current_altitude_ = msg->pose.pose.position.z;
    odom_received_ = true;
  }

  void controlCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    latest_control_cmd_ = *msg;
    last_control_stamp_ = ros::Time::now();
    control_received_ = true;
  }

  geometry_msgs::Twist buildSafeCommand() const {
    geometry_msgs::Twist cmd;

    if (!odom_received_ || current_altitude_ < target_altitude_ - 0.15) {
      cmd.linear.x = 0.0;
      cmd.linear.y = 0.0;
      cmd.linear.z = takeoff_speed_;
      cmd.angular.z = 0.0;
      return cmd;
    }

    const bool command_fresh = control_received_ &&
                               ((ros::Time::now() - last_control_stamp_).toSec() <= command_timeout_);
    if (command_fresh) {
      cmd = latest_control_cmd_;
      // Keep altitude after takeoff. The local planner currently controls only
      // horizontal motion, so vertical velocity is zeroed in cruise mode.
      cmd.linear.z = 0.0;
      return cmd;
    }

    // Failsafe hover command when the planner is not ready yet.
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.z = 0.0;
    return cmd;
  }

  void requestOffboardAndArm() {
    if (!connected_) {
      return;
    }

    const ros::Time now = ros::Time::now();
    if (!last_request_stamp_.isZero() && (now - last_request_stamp_).toSec() < 2.0) {
      return;
    }

    if (!offboard_) {
      mavros_msgs::SetMode set_mode;
      set_mode.request.custom_mode = "OFFBOARD";
      if (set_mode_client_.call(set_mode) && set_mode.response.mode_sent) {
        ROS_INFO("OFFBOARD mode requested successfully");
      } else {
        ROS_WARN("Failed to request OFFBOARD mode");
      }
      last_request_stamp_ = now;
      return;
    }

    if (!armed_) {
      mavros_msgs::CommandBool arm_cmd;
      arm_cmd.request.value = true;
      if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Vehicle arm requested successfully");
      } else {
        ROS_WARN("Failed to arm vehicle");
      }
      last_request_stamp_ = now;
    }
  }

  void timerCallback(const ros::TimerEvent &) {
    const geometry_msgs::Twist cmd = buildSafeCommand();
    setpoint_pub_.publish(cmd);
    ++published_cycles_;

    // PX4 requires a stream of setpoints before entering OFFBOARD. We wait for
    // several seconds of setpoint publication before requesting mode/arm.
    if (published_cycles_ >= offboard_warmup_cycles_) {
      requestOffboardAndArm();
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "offboard_control_node");
  OffboardControlNode node;
  ros::spin();
  return 0;
}
