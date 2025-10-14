#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Dense>
#include <string>

// Глобальные объекты и параметры фильтра Калмана
ros::Publisher state_pub;
Eigen::VectorXd x;         // Оценка состояния (скорости)
Eigen::MatrixXd P;         // Ковариация оценки
Eigen::MatrixXd A;         // Модель перехода (тождественная)
Eigen::MatrixXd Q;         // Ковариация процесса
Eigen::MatrixXd H;         // Матрица измерения (тождественная)
Eigen::MatrixXd R;         // Ковариация измерений

bool is_initialized = false;
ros::Time last_time;

double initial_covariance = 1.0;
double process_noise = 0.1;
double measurement_noise = 0.5;
double min_dt = 0.01;
std::string frame_id = "base_link";

void initKalmanFilter()
{
  int n = 3; // число параметров (скорости по x, y, z)
  x = Eigen::VectorXd::Zero(n);
  P = Eigen::MatrixXd::Identity(n, n) * initial_covariance;

  A = Eigen::MatrixXd::Identity(n, n);
  Q = Eigen::MatrixXd::Identity(n, n) * process_noise;

  H = Eigen::MatrixXd::Identity(n, n);
  R = Eigen::MatrixXd::Identity(n, n) * measurement_noise;

  is_initialized = true;
}

void predict(double dt)
{
  // Для простоты используем модель постоянной скорости.
  // В реальной реализации нужно учитывать динамику (например, интегрировать ускорение).
  // Если требуется, можно добавить влияние dt в матрицу A.
  x = A * x;
  P = A * P * A.transpose() + Q;
}

void update(const Eigen::VectorXd &z)
{
  Eigen::MatrixXd S = H * P * H.transpose() + R;
  Eigen::MatrixXd K = P * H.transpose() * S.inverse();
  x = x + K * (z - H * x);
  int n = x.size();
  P = (Eigen::MatrixXd::Identity(n, n) - K * H) * P;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  // Инициализация фильтра при первом вызове
  if (!is_initialized)
  {
    initKalmanFilter();
    last_time = msg->header.stamp;
    return;
  }

  ros::Time current_time = msg->header.stamp;
  double dt = (current_time - last_time).toSec();
  if (dt <= 0) dt = min_dt; // защита от нулевого интервала
  last_time = current_time;

  // Предсказание
  predict(dt);

  // Для демонстрации: используем интегрированное ускорение как приближение изменения скорости.
  // В реальности интегрирование требует более аккуратного подхода.
  Eigen::VectorXd z(3);
  z(0) = msg->linear_acceleration.x * dt;
  z(1) = msg->linear_acceleration.y * dt;
  z(2) = msg->linear_acceleration.z * dt;

  // Обновление оценки состояния
  update(z);

  // Подготовка и публикация сообщения состояния
  geometry_msgs::TwistStamped state_msg;
  state_msg.header.stamp = current_time;
  state_msg.header.frame_id = frame_id;
  state_msg.twist.linear.x = x(0);
  state_msg.twist.linear.y = x(1);
  state_msg.twist.linear.z = x(2);
  state_pub.publish(state_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kalman_filter_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.param("initial_covariance", initial_covariance, 1.0);
  pnh.param("process_noise", process_noise, 0.1);
  pnh.param("measurement_noise", measurement_noise, 0.5);
  pnh.param("min_dt", min_dt, 0.01);
  pnh.param<std::string>("frame_id", frame_id, "base_link");

  state_pub = nh.advertise<geometry_msgs::TwistStamped>("state_estimate", 10);
  ros::Subscriber imu_sub = nh.subscribe("mavros/imu/data", 10, imuCallback);

  // Задаём постоянную частоту публикации (например, 50 Гц)
  ros::Rate loop_rate(50);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

