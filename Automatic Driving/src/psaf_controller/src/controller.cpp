//
// Created by psaf on 30.10.24.
//
#include "rclcpp/rclcpp.hpp"
#include "psaf_controller/controller_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<ControllerNode> node = std::make_shared<ControllerNode>();
    double update_frequency = node->declare_parameter("update_frequency", 1000.0);
    rclcpp::WallRate rate(update_frequency);

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        node->update();
        rate.sleep();
    }

    rclcpp::shutdown();
}
