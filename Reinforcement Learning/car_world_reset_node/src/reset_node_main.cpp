#include "car_world_reset_node/reset_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<car_world_reset_node::ResetNode>();
  // auto position_node = std::make_shared<PositionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
  // rclcpp::executors::MultiThreadedExecutor executor;
  // executor.add_node(reset_node);
  // executor.add_node(position_node);

  // // 运行所有节点
  // executor.spin();

  // rclcpp::shutdown();
  // return 0;
}
