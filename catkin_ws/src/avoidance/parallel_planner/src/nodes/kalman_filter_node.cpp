#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Dense>

// Глобальный паблишер для состояния
ros::Publisher state_pub;

// Параметры фильтра Калмана (упрощённый пример)
// Предположим, что состояние состоит из 3-х компонент скорости: [vx, vy, vz]
Eigen::VectorXd x;         // Оценка состояния (скорости)
Eigen::MatrixXd P;         // Ковариация оценки

// Матрицы фильтра (для простоты используем идентичные модели)
Eigen::MatrixXd A;         // Модель перехода (здесь - тождественная)
Eigen::MatrixXd Q;         // Ковариация процесса
Eigen::MatrixXd H;         // Матрица измерения (тождественная)
Eigen::MatrixXd R;         // Ковариация измерений

bool is_initialized = false;
ros::Time last_time;

void initKalmanFilter()
{
  int n = 3; // число параметров (скорости по x, y, z)
  x = Eigen::VectorXd::Zero(n);
  P = Eigen::MatrixXd::Identity(n, n) * 1.0;

  A = Eigen::MatrixXd::Identity(n, n);
  Q = Eigen::MatrixXd::Identity(n, n) * 0.1;

  H = Eigen::MatrixXd::Identity(n, n);
  R = Eigen::MatrixXd::Identity(n, n) * 0.5;

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
  if (dt <= 0) dt = 0.01; // защита от нулевого интервала
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
  state_msg.header.frame_id = "base_link";  // можно изменить на нужный frame
  state_msg.twist.linear.x = x(0);
  state_msg.twist.linear.y = x(1);
  state_msg.twist.linear.z = x(2);
  state_pub.publish(state_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kalman_filter_node");
  ros::NodeHandle nh;

  // Изменённый выходной топик: /uav/state_estimate
  state_pub = nh.advertise<geometry_msgs::TwistStamped>("/uav/state_estimate", 10);
  ros::Subscriber imu_sub = nh.subscribe("imu/data", 10, imuCallback);

  // Задаём постоянную частоту публикации (например, 50 Гц)
  ros::Rate loop_rate(50);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

