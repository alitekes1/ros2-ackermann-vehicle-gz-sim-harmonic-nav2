#include "saye_control/control.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto control = std::make_shared<Control>();
    while (!control->client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    
    auto request = std::make_shared<saye_msgs::srv::ShareMap::Request>();
    request->topic_name = "my_topic";
    auto respond = control->client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(control->get_node_base_interface(), respond) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Is Completed: %d", respond.get()->is_completed); // burada merged occupancy grid i de alacağız.
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
    }
    rclcpp::shutdown();
    return 0;
}