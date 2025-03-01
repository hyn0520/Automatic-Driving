#include "car_world_reset_node/reset_node.hpp"
#include <ignition/transport/Node.hh>
#include <chrono>
#include <thread>

namespace car_world_reset_node
{

//////////////////////////////////////////////////
ResetNode::ResetNode()
: Node("reset_node")
{
  // Create a ROS2 service /reset_env
  using Trigger = std_srvs::srv::Trigger;
  srv_ = this->create_service<Trigger>(
    "/reset_env",
    std::bind(&ResetNode::onResetEnv, this,
              std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "ResetNode is ready. Service: /reset_env");
}

//////////////////////////////////////////////////
void ResetNode::onResetEnv(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received /reset_env call");

  bool ok = this->callIgnSetPose();
  if (ok)
  {
    response->success = true;
    response->message = "Ign set_pose succeeded";
  }
  else
  {
    response->success = false;
    response->message = "Ign set_pose failed";
  }
  RCLCPP_INFO(this->get_logger(), "Done /reset_env -> success=%d", (int)ok);
}

//////////////////////////////////////////////////
bool ResetNode::callIgnSetPose()
{
  // 1) 准备Pose请求
  ignition::msgs::Pose req;
  req.set_name("vehicle_blue");
  req.mutable_position()->set_x(1.0); //0
  req.mutable_position()->set_y(4.0); //0.22
  req.mutable_position()->set_z(0.0);  //0.0
  req.mutable_orientation()->set_x(0.0);
  req.mutable_orientation()->set_y(0.0);
  req.mutable_orientation()->set_z(20.0);
  req.mutable_orientation()->set_w(-0.5); // 无旋转//1.0

  // 2) 服务名
  const std::string serviceName = "/world/car_world/set_pose";

  // 3) 调用同步 Request
  ignition::msgs::Boolean replyMsg;
  bool resultFlag = false;
  const unsigned int timeoutMs = 2000;  // 2秒可调

  bool success = ign_node_.Request(serviceName, req, timeoutMs, replyMsg, resultFlag);

  if (!success)
  {
    // 请求本身没发出去
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to place request on ign service [%s].", serviceName.c_str());
    return false;
  }

  if (!resultFlag)
  {
    // 服务端处理失败
    RCLCPP_ERROR(this->get_logger(),
                 "Ign service [%s] call returned 'result=false'.", serviceName.c_str());
    return false;
  }

  // 服务端成功返回
  bool final = replyMsg.data();
  RCLCPP_INFO(this->get_logger(), "Ign service responded: %s", final ? "true" : "false");
  return final;
}


}  // namespace car_world_reset_node
