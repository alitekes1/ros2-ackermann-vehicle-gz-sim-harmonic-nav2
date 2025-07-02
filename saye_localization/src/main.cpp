#include "saye_localization/kalman_filter.hpp"
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<kalman_filter>("kalman_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}