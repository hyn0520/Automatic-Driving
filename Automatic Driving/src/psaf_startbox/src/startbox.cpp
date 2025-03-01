#include "rclcpp/rclcpp.hpp"
#include "psaf_startbox/startbox_node.hpp"


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<StartBoxNode> node = std::make_shared<StartBoxNode>();
    double update_frequency = node->declare_parameter("update_frequency", 30.0);
    rclcpp::WallRate rate(update_frequency);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
}
