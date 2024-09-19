#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1; // std::bind fonksiyonu için

class kalman_filter : public rclcpp::Node
{
private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

    nav_msgs::msg::Odometry kalman_odom_message;

    double mean, variance, imu_angular_z, last_angular_z, motion, measurment_variance, motion_variance;
    bool is_first_odom;

    void odom_sub_callback(const nav_msgs::msg::Odometry &msg);
    void imu_sub_callback(const sensor_msgs::msg::Imu &msg);

public:
    kalman_filter(const std::string &node_name);
    void statePrediction();
    void measurmentUpdate();
};

kalman_filter::kalman_filter(const std::string &node_name) : Node(node_name)
{
    this->mean = 0;
    this->imu_angular_z = 0.0;
    this->variance = 1000.0;
    this->is_first_odom = true;
    this->last_angular_z = 0.0;
    this->motion = 0.0;
    this->measurment_variance = 0.5;
    this->motion_variance = 4.0;

    this->odom_sub = create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&kalman_filter::odom_sub_callback, this, _1));
    this->imu_sub = create_subscription<sensor_msgs::msg::Imu>("/imu", 10, std::bind(&kalman_filter::imu_sub_callback, this, _1));
    this->odom_pub = create_publisher<nav_msgs::msg::Odometry>("robot/odom_kalman", 10);
}

void kalman_filter::odom_sub_callback(const nav_msgs::msg::Odometry &odom_data)
{
    this->kalman_odom_message = odom_data;
    if (this->is_first_odom)
    {
        this->mean = odom_data.twist.twist.angular.z;
        this->last_angular_z = this->mean;
        this->is_first_odom = false;
        return;
    }
    this->motion = odom_data.twist.twist.angular.z;
    this->statePrediction();
    this->measurmentUpdate();

    kalman_odom_message.twist.twist.angular.z = mean;
    odom_pub->publish(kalman_odom_message);
}
void kalman_filter::imu_sub_callback(const sensor_msgs::msg::Imu &imu_data)
{
    this->imu_angular_z = imu_data.angular_velocity.z;
}
void kalman_filter::statePrediction()
{
    mean = mean + motion;
    variance = variance + motion_variance;
}
void kalman_filter::measurmentUpdate()
{
    // kalman filtresi formulunden geldi
    mean = (measurment_variance * mean + variance * imu_angular_z) / (variance + measurment_variance);
    variance = (variance * measurment_variance) / (variance / measurment_variance);
}
#endif
