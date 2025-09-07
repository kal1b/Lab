#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <vector>

class VFHNode {
private:
  ros::NodeHandle nh_;
  ros::Subscriber point_sub_;
  ros::Subscriber state_sub_;
  ros::Publisher nav_pub_;
  int num_sectors_;         // число секторов гистограммы (разрешение направления)
  double safety_dist_;      // расстояние безопасности до препятствий (м)
  double forward_speed_;    // базовая поступательная скорость (м/с) – максимальная скорость вперёд
  double max_yaw_rate_;     // максимальная скорость поворота (рад/с)
  double current_yaw_;      // текущий курс (yaw) БПЛА

public:
  VFHNode() {
    // Загрузка параметров алгоритма VFH+ из Parameter Server
    // При отсутствии на Parameter Server будут использованы указанные значения по умолчанию.
    nh_.param("histogram_bins",    num_sectors_,   72);   // число секторов (360/5°=72)
    nh_.param("safety_distance",   safety_dist_,   5.0);  // радиус безопасности, м
    nh_.param("max_forward_speed", forward_speed_, 1.0);  // максимальная поступательная скорость, м/с
    nh_.param("max_yaw_rate",      max_yaw_rate_,  0.5);  // максимальная угловая скорость, рад/с
    current_yaw_ = 0.0;

    // Подписка на облако точек лидара и на оценку состояния БПЛА
    point_sub_ = nh_.subscribe("/point_cloud", 1, &VFHNode::pointCloudCallback, this);
    state_sub_ = nh_.subscribe("/uav/state_estimate", 10, &VFHNode::stateCallback, this);
    // Публикация команд навигации (линейная и угловая скорость движения дрона)
    nav_pub_ = nh_.advertise<geometry_msgs::Twist>("/uav/navigation_cmd", 10);

    ROS_INFO("VFHNode initialized with %d histogram bins, safety_distance=%.2f, max_forward_speed=%.2f, max_yaw_rate=%.2f",
             num_sectors_, safety_dist_, forward_speed_, max_yaw_rate_);
  }

  void stateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // Обновляем текущий угол рыскания (yaw) дрона на основе ориентировки из одометри/оценки состояния
    tf2::Quaternion q(msg->pose.orientation.x,
                      msg->pose.orientation.y,
                      msg->pose.orientation.z,
                      msg->pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_yaw_);
    // current_yaw_ теперь хранит актуальный угол yaw БПЛА (непосредственно не используется в данном простом алгоритме)
  }

  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // Инициализация массива занятости секторов (все сектора свободны = false)
    int n = num_sectors_;
    std::vector<bool> occupied(n, false);

    // Проход по точкам облака лидара и определение занятых секторов
    for (sensor_msgs::PointCloud2ConstIterator<float> it(*msg, "x"); it != it.end(); ++it) {
      float x = it[0];
      float y = it[1];
      float z = it[2];
      float dist = std::sqrt(x*x + y*y + z*z);
      if (dist < safety_dist_) {
        // Вычисляем горизонтальный угол точки относительно направления вперед (ось X дрона)
        float angle = std::atan2(y, x);
        if (angle < 0) angle += 2 * M_PI;
        int sector = static_cast<int>(std::floor(angle / (2 * M_PI / n)));
        if (sector < 0)   sector = 0;
        if (sector >= n)  sector = n - 1;
        occupied[sector] = true;
      }
    }

    // Определение желаемого сектора движения (0 соответствует направлению вперед)
    int desired_sector = 0;
    if (occupied[desired_sector]) {
      // Если сектор прямо по курсу занят, ищем ближайший свободный сектор по бокам
      int free_sector = -1;
      for (int offset = 1; offset < n/2; ++offset) {
        int left  = (desired_sector - offset + n) % n;
        int right = (desired_sector + offset) % n;
        if (!occupied[left]) {
          free_sector = left;
          break;
        }
        if (!occupied[right]) {
          free_sector = right;
          break;
        }
      }
      if (free_sector == -1) {
        // Нет свободного направления в пределах полусферы — останавливаемся (препятствия вокруг)
        geometry_msgs::Twist stop_cmd;
        stop_cmd.linear.x  = 0.0;
        stop_cmd.linear.y  = 0.0;
        stop_cmd.linear.z  = 0.0;
        stop_cmd.angular.z = 0.0;
        nav_pub_.publish(stop_cmd);
        return;
      } else {
        desired_sector = free_sector;
      }
    }

    // Вычисление относительного угла (рад) выбранного сектора относительно текущего курса (yaw=0 вперед)
    double sector_angle = desired_sector * (2 * M_PI / n);
    if (sector_angle > M_PI) {
      sector_angle -= 2 * M_PI;  // нормализуем угол в диапазон [-π, π]
    }
    double relative_angle = sector_angle;

    // Формируем команду поворота: задаём угловую скорость по оси Z пропорционально направлению
    double yaw_rate_cmd = 0.0;
    if (std::fabs(relative_angle) > 0.1) {
      // Если требуемый поворот больше порога 0.1 рад (~5.7°), крутимся с максимальной скоростью в нужную сторону
      yaw_rate_cmd = (relative_angle > 0 ? max_yaw_rate_ : -max_yaw_rate_);
    }

    // Вычисляем поступательную скорость вперед с учётом поворота:
    // при большом угле поворота уменьшаем скорость (до 0 при повороте > 90°)
    double angle_mag = std::fabs(relative_angle);
    double forward_cmd = forward_speed_;
    if (angle_mag > M_PI / 2) {
      forward_cmd = 0.0;
    } else {
      forward_cmd = forward_speed_ * std::cos(angle_mag);
    }

    // Публикуем команду навигации (линейная и угловая скорость)
    geometry_msgs::Twist cmd;
    cmd.linear.x  = forward_cmd;
    cmd.linear.y  = 0.0;
    cmd.linear.z  = 0.0;
    cmd.angular.z = yaw_rate_cmd;
    nav_pub_.publish(cmd);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "vfh_node");
  VFHNode node;
  ros::spin();
  return 0;
}

