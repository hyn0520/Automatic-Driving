//
// Created by psaf on 30.10.24.
//
#include "rclcpp/rclcpp.hpp"
#include "psaf_lane_detection_inside/lane_detection_inside_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<LaneDetectionInsideNode> node = std::make_shared<LaneDetectionInsideNode>();
    double update_frequency = node->declare_parameter("update_frequency", 125.0);
    rclcpp::WallRate rate(update_frequency);

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        rate.sleep();
        }
    rclcpp::shutdown();
}
