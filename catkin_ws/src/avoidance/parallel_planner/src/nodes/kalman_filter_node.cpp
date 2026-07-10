#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>

class KalmanFilterNode {
public:
  KalmanFilterNode()
      : nh_(), pnh_("~"), initialized_(false) {
    pnh_.param("process_noise", process_noise_, 0.05);
    pnh_.param("measurement_noise", measurement_noise_, 0.20);

    x_.setZero();
    P_.setIdentity();
    P_ *= 1.0;

    H_.setZero();
    H_.block<3, 3>(0, 0).setIdentity();

    Q_.setIdentity();
    Q_ *= process_noise_;

    R_.setIdentity();
    R_ *= measurement_noise_;

    odom_sub_ = nh_.subscribe("mavros/local_position/odom", 20,
                             &KalmanFilterNode::odomCallback, this);
    imu_sub_ = nh_.subscribe("mavros/imu/data", 20,
                            &KalmanFilterNode::imuCallback, this);

    state_pub_ = nh_.advertise<nav_msgs::Odometry>("state_estimate", 20);

    ROS_INFO("kalman_filter_node: publishing relative topic state_estimate as nav_msgs/Odometry");
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber imu_sub_;
  ros::Publisher state_pub_;

  bool initialized_;
  ros::Time last_stamp_;
  geometry_msgs::Quaternion last_orientation_;
  Eigen::Vector3d last_acc_world_{0.0, 0.0, 0.0};

  // State: [x, y, z, vx, vy, vz]^T
  Eigen::Matrix<double, 6, 1> x_;
  Eigen::Matrix<double, 6, 6> P_;
  Eigen::Matrix<double, 3, 6> H_;
  Eigen::Matrix<double, 6, 6> Q_;
  Eigen::Matrix<double, 3, 3> R_;

  double process_noise_;
  double measurement_noise_;

  void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    // IMU is kept for future process-model refinement. The first stabilization
    // step relies on MAVROS local odometry because it already contains PX4 EKF2
    // position, orientation and velocity in one synchronized message.
    last_acc_world_.x() = msg->linear_acceleration.x;
    last_acc_world_.y() = msg->linear_acceleration.y;
    last_acc_world_.z() = msg->linear_acceleration.z;
  }

  void predict(double dt) {
    if (dt <= 0.0 || dt > 1.0) {
      dt = 0.02;
    }

    Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
    F(0, 3) = dt;
    F(1, 4) = dt;
    F(2, 5) = dt;

    x_ = F * x_;
    P_ = F * P_ * F.transpose() + Q_;
  }

  void correctPosition(const Eigen::Vector3d &z) {
    Eigen::Vector3d y = z - H_ * x_;
    Eigen::Matrix3d S = H_ * P_ * H_.transpose() + R_;
    Eigen::Matrix<double, 6, 3> K = P_ * H_.transpose() * S.inverse();

    x_ = x_ + K * y;
    P_ = (Eigen::Matrix<double, 6, 6>::Identity() - K * H_) * P_;
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;

    Eigen::Vector3d measured_position;
    measured_position << msg->pose.pose.position.x,
                         msg->pose.pose.position.y,
                         msg->pose.pose.position.z;

    if (!initialized_) {
      x_.setZero();
      x_(0) = measured_position.x();
      x_(1) = measured_position.y();
      x_(2) = measured_position.z();
      x_(3) = msg->twist.twist.linear.x;
      x_(4) = msg->twist.twist.linear.y;
      x_(5) = msg->twist.twist.linear.z;
      last_stamp_ = stamp;
      last_orientation_ = msg->pose.pose.orientation;
      initialized_ = true;
    } else {
      const double dt = (stamp - last_stamp_).toSec();
      predict(dt);
      correctPosition(measured_position);

      // PX4/MAVROS already estimates velocity; blend it in directly to avoid
      // the old unstable IMU-only velocity integration.
      x_(3) = msg->twist.twist.linear.x;
      x_(4) = msg->twist.twist.linear.y;
      x_(5) = msg->twist.twist.linear.z;
      last_stamp_ = stamp;
      last_orientation_ = msg->pose.pose.orientation;
    }

    nav_msgs::Odometry estimate;
    estimate.header.stamp = stamp;
    estimate.header.frame_id = msg->header.frame_id.empty() ? "map" : msg->header.frame_id;
    estimate.child_frame_id = msg->child_frame_id.empty() ? "base_link" : msg->child_frame_id;

    estimate.pose.pose.position.x = x_(0);
    estimate.pose.pose.position.y = x_(1);
    estimate.pose.pose.position.z = x_(2);
    estimate.pose.pose.orientation = last_orientation_;

    estimate.twist.twist.linear.x = x_(3);
    estimate.twist.twist.linear.y = x_(4);
    estimate.twist.twist.linear.z = x_(5);
    estimate.twist.twist.angular = msg->twist.twist.angular;

    state_pub_.publish(estimate);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "kalman_filter_node");
  KalmanFilterNode node;
  ros::spin();
  return 0;
}
