#ifndef POSITION_NODE_HPP
#define POSITION_NODE_HPP

#include <ignition/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class PositionNode : public rclcpp::Node {
public:
    PositionNode();

private:
    void ignCallback(const ignition::msgs::Pose_V &ign_msg);

    ignition::transport::Node ign_node_;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ros_publisher_;
};

#endif // POSITION_NODE_HPP