#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <limits>
#include <vector>

class VFHNode {
public:
  VFHNode() : nh_(), pnh_("~"), current_yaw_(0.0), state_received_(false) {
    pnh_.param("histogram_bins", num_sectors_, 72);
    pnh_.param("safety_distance", safety_dist_, 4.0);
    pnh_.param("max_forward_speed", forward_speed_, 1.0);
    pnh_.param("max_yaw_rate", max_yaw_rate_, 0.5);
    pnh_.param("min_valid_range", min_valid_range_, 0.25);
    pnh_.param("max_valid_range", max_valid_range_, 12.0);
    pnh_.param("front_sector_width_deg", front_sector_width_deg_, 25.0);

    point_sub_ = nh_.subscribe("point_cloud", 1, &VFHNode::pointCloudCallback, this);
    state_sub_ = nh_.subscribe("state_estimate", 10, &VFHNode::stateCallback, this);
    nav_pub_ = nh_.advertise<geometry_msgs::Twist>("navigation_cmd", 10);

    ROS_INFO("vfh_node: point_cloud + state_estimate -> navigation_cmd");
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber point_sub_;
  ros::Subscriber state_sub_;
  ros::Publisher nav_pub_;

  int num_sectors_;
  double safety_dist_;
  double forward_speed_;
  double max_yaw_rate_;
  double min_valid_range_;
  double max_valid_range_;
  double front_sector_width_deg_;
  double current_yaw_;
  bool state_received_;

  void stateCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    tf2::Quaternion q(msg->pose.pose.orientation.x,
                      msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z,
                      msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll = 0.0;
    double pitch = 0.0;
    m.getRPY(roll, pitch, current_yaw_);
    state_received_ = true;
  }

  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    if (num_sectors_ <= 0) {
      return;
    }

    const int n = num_sectors_;
    std::vector<double> nearest(n, std::numeric_limits<double>::infinity());
    std::vector<bool> occupied(n, false);

    for (sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x"), it_y(*msg, "y"), it_z(*msg, "z");
         it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      const float x = *it_x;
      const float y = *it_y;
      const float z = *it_z;

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }

      const double dist = std::sqrt(x * x + y * y + z * z);
      if (dist < min_valid_range_ || dist > max_valid_range_) {
        continue;
      }

      double angle = std::atan2(y, x);
      if (angle < 0.0) {
        angle += 2.0 * M_PI;
      }

      int sector = static_cast<int>(std::floor(angle / (2.0 * M_PI / n)));
      sector = std::max(0, std::min(sector, n - 1));

      nearest[sector] = std::min(nearest[sector], dist);
      if (dist < safety_dist_) {
        occupied[sector] = true;
      }
    }

    const int desired_sector = 0;  // Forward direction in the body frame.
    int best_sector = desired_sector;

    const int inflation = std::max(1, static_cast<int>(std::ceil(front_sector_width_deg_ / (360.0 / n))));
    std::vector<bool> inflated = occupied;
    for (int i = 0; i < n; ++i) {
      if (!occupied[i]) {
        continue;
      }
      for (int k = -inflation; k <= inflation; ++k) {
        const int idx = (i + k + n) % n;
        inflated[idx] = true;
      }
    }

    if (inflated[desired_sector]) {
      best_sector = -1;
      for (int offset = 1; offset < n / 2; ++offset) {
        const int left = (desired_sector + offset) % n;
        const int right = (desired_sector - offset + n) % n;

        if (!inflated[left]) {
          best_sector = left;
          break;
        }
        if (!inflated[right]) {
          best_sector = right;
          break;
        }
      }
    }

    geometry_msgs::Twist cmd;
    if (best_sector < 0) {
      cmd.linear.x = 0.0;
      cmd.linear.y = 0.0;
      cmd.linear.z = 0.0;
      cmd.angular.z = 0.0;
      nav_pub_.publish(cmd);
      return;
    }

    double sector_angle = best_sector * (2.0 * M_PI / n);
    if (sector_angle > M_PI) {
      sector_angle -= 2.0 * M_PI;
    }

    const double angle_mag = std::fabs(sector_angle);
    const double turn_gain = std::min(1.0, angle_mag / (M_PI / 2.0));

    cmd.linear.x = forward_speed_ * std::max(0.0, std::cos(angle_mag));
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.z = (sector_angle >= 0.0 ? 1.0 : -1.0) * max_yaw_rate_ * turn_gain;

    if (angle_mag < 0.08) {
      cmd.angular.z = 0.0;
    }

    nav_pub_.publish(cmd);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "vfh_node");
  VFHNode node;
  ros::spin();
  return 0;
}
