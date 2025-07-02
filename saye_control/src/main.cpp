#include "saye_control/control.hpp"
#include "saye_control/joystick_controller_simulation.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Control>();
    auto joystick_controller = std::make_shared<Joystick_Controller>();
    rclcpp::executors::SingleThreadedExecutor executor;
    // executor.add_node(node);
    executor.add_node(joystick_controller);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}