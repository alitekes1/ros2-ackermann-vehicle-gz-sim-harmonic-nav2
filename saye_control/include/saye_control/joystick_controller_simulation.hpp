#ifndef JOYSTICK_CONTROLLER__HPP
#define JOYSTICK_CONTROLLER__HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
using namespace std::placeholders;

class Joystick_Controller : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
    geometry_msgs::msg::Twist cmd_vel_msg;

public:
    Joystick_Controller(/* args */);
    ~Joystick_Controller();
    void subscriber_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void timer_callback();
};

Joystick_Controller::Joystick_Controller() : Node("joystick_controller_node")
{
    this->publisher = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    this->subscriber = create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&Joystick_Controller::subscriber_callback, this, _1));
    this->timer = create_wall_timer(std::chrono::seconds(1), std::bind(&Joystick_Controller::timer_callback, this));
}

Joystick_Controller::~Joystick_Controller()
{
}
void Joystick_Controller::subscriber_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    cmd_vel_msg.linear.x = msg->axes[1];
    cmd_vel_msg.angular.z = msg->axes[0];
    RCLCPP_INFO(this->get_logger(), "Subscriber is worked.Message: Joy\n left X : %f, Left Y:%f", cmd_vel_msg.linear.x, cmd_vel_msg.angular.z);
}
void Joystick_Controller::timer_callback()
{
    publisher->publish(cmd_vel_msg);
    RCLCPP_INFO(this->get_logger(), "Publisher is worked.linear x: %f, angular z:%f", cmd_vel_msg.linear.x, cmd_vel_msg.angular.z);
}
#endif