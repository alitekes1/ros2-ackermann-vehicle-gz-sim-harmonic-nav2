#include "my_robot_localization/kalman_filter.hpp"
#include "my_robot_localization/joystick_controller.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // auto node = std::make_shared<kalman_filter>("kalman_node");
    auto node = std::make_shared<Joystick_Controller>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}