#include "car_world_reset_node/position_node.hpp"

PositionNode::PositionNode() : Node("position_node") {
    // 初始化ROS2 Publisher
    ros_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("vehicle_pose", 10);
    
    // 初始化Ignition Subscriber
    ign_node_.Subscribe("/world/car_world/dynamic_pose/info", &PositionNode::ignCallback, this);
    
    RCLCPP_INFO(this->get_logger(), "Position Transformation is finished");
}

void PositionNode::ignCallback(const ignition::msgs::Pose_V &ign_msg) {
    for (const auto &pose : ign_msg.pose()) {
        if (pose.name() == "vehicle_blue") {
            // 转换Ignition消息到ROS2消息
            auto ros_msg = geometry_msgs::msg::PoseStamped();
            ros_msg.header.stamp = this->now();
            ros_msg.header.frame_id = "map";
            
            // 位置
            ros_msg.pose.position.x = pose.position().x();
            ros_msg.pose.position.y = pose.position().y();
            ros_msg.pose.position.z = pose.position().z();
            
            // 姿态（需处理四元数坐标系差异）
            ros_msg.pose.orientation.x = pose.orientation().x();
            ros_msg.pose.orientation.y = pose.orientation().y();
            ros_msg.pose.orientation.z = pose.orientation().z();
            ros_msg.pose.orientation.w = pose.orientation().w();
            
            // 发布ROS2消息
            ros_publisher_->publish(ros_msg);
        }
    }
}