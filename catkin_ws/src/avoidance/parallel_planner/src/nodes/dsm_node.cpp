#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

// Параметры ограничений (можно задать через параметрический сервер)
double max_linear_acc = 1.0;   // максимальное линейное ускорение (м/с^2)
double max_angular_acc = 0.5;  // максимальное угловое ускорение (рад/с^2)

// Глобальные переменные для хранения последней полученной команды и состояния
geometry_msgs::Twist latest_nav_cmd;
bool nav_cmd_received = false;

geometry_msgs::TwistStamped latest_state;
bool state_received = false;

ros::Publisher control_pub;

// Функция для ограничения значения между min_val и max_val
double clamp(double value, double min_val, double max_val)
{
    if (value < min_val)
        return min_val;
    if (value > max_val)
        return max_val;
    return value;
}

// Callback для входящего топика навигационных команд (/uav/navigation_cmd)
void navCmdCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    latest_nav_cmd = *msg;
    nav_cmd_received = true;
}

// Callback для оценки состояния (/uav/state_estimate)
void stateCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    latest_state = *msg;
    state_received = true;
}

// Функция модуляции команды с учётом текущего состояния и ограничений
void modulateAndPublishCommand()
{
    if (!nav_cmd_received || !state_received)
        return;

    geometry_msgs::Twist modulated_cmd;
    // Предположим, что цикл работает на 50 Гц (dt ≈ 0.02 сек)
    double dt = 0.02;

    // Ограничение для линейной компоненты по оси X
    double curr_vx = latest_state.twist.linear.x;
    double desired_vx = latest_nav_cmd.linear.x;
    double delta_vx = desired_vx - curr_vx;
    double max_delta_vx = max_linear_acc * dt;
    delta_vx = clamp(delta_vx, -max_delta_vx, max_delta_vx);
    modulated_cmd.linear.x = curr_vx + delta_vx;

    // Ограничение для линейной компоненты по оси Y
    double curr_vy = latest_state.twist.linear.y;
    double desired_vy = latest_nav_cmd.linear.y;
    double delta_vy = desired_vy - curr_vy;
    double max_delta_vy = max_linear_acc * dt;
    delta_vy = clamp(delta_vy, -max_delta_vy, max_delta_vy);
    modulated_cmd.linear.y = curr_vy + delta_vy;

    // Ограничение для линейной компоненты по оси Z
    double curr_vz = latest_state.twist.linear.z;
    double desired_vz = latest_nav_cmd.linear.z;
    double delta_vz = desired_vz - curr_vz;
    double max_delta_vz = max_linear_acc * dt;
    delta_vz = clamp(delta_vz, -max_delta_vz, max_delta_vz);
    modulated_cmd.linear.z = curr_vz + delta_vz;

    // Ограничение для угловой компоненты по оси Z (yaw)
    double curr_vyaw = latest_state.twist.angular.z;
    double desired_vyaw = latest_nav_cmd.angular.z;
    double delta_vyaw = desired_vyaw - curr_vyaw;
    double max_delta_vyaw = max_angular_acc * dt;
    delta_vyaw = clamp(delta_vyaw, -max_delta_vyaw, max_delta_vyaw);
    modulated_cmd.angular.z = curr_vyaw + delta_vyaw;

    // Остальные угловые компоненты можно оставить без ограничения или добавить аналогичные проверки
    modulated_cmd.angular.x = latest_nav_cmd.angular.x;
    modulated_cmd.angular.y = latest_nav_cmd.angular.y;

    // Публикуем модулированную команду
    control_pub.publish(modulated_cmd);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dsm_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // Чтение параметров ограничений из параметрического сервера
    pnh.param("max_linear_acc", max_linear_acc, 1.0);
    pnh.param("max_angular_acc", max_angular_acc, 0.5);

    // Подписка на входные топики:
    // 1. Навигационные команды от планировщика (/uav/navigation_cmd)
    ros::Subscriber nav_cmd_sub = nh.subscribe("/uav/navigation_cmd", 10, navCmdCallback);
    // 2. Оценка состояния от фильтра Калмана (/uav/state_estimate)
    ros::Subscriber state_sub = nh.subscribe("/uav/state_estimate", 10, stateCallback);

    // Публикация модулированной команды управления на топик /uav/control
    control_pub = nh.advertise<geometry_msgs::Twist>("/uav/control", 10);

    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        ros::spinOnce();
        modulateAndPublishCommand();
        loop_rate.sleep();
    }

    return 0;
}

