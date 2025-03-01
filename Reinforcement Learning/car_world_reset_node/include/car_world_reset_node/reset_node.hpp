#ifndef CAR_WORLD_RESET_NODE__RESET_NODE_HPP_
#define CAR_WORLD_RESET_NODE__RESET_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <ignition/transport/Node.hh>
#include <ignition/msgs/pose.pb.h>
#include <ignition/msgs/boolean.pb.h>

#include <string>
#include <mutex>

namespace car_world_reset_node
{

/// \brief A ROS 2 node that provides a /reset_env service.
///        Upon request, it sends an ign-transport request to /world/car_world/set_pose 
///        with ignition::msgs::Pose, and expects ignition::msgs::Boolean as response.
class ResetNode : public rclcpp::Node
{
public:
  /// \brief Constructor
  ResetNode();

private:
  /// \brief Callback for the /reset_env service
  /// \param[in] request  Standard Trigger request (no fields)
  /// \param[out] response success/fail
  void onResetEnv(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /// \brief Actually call ign service /world/car_world/set_pose
  /// \return true if success, false otherwise
  bool callIgnSetPose();

private:
  /// \brief The ROS 2 service server handle
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;

  /// \brief The ignition transport node
  ignition::transport::Node ign_node_;

  /// \brief Protect the response received from ign
  std::mutex result_mutex_;
  bool ign_result_{false};
};

}  // namespace car_world_reset_node

#endif  // CAR_WORLD_RESET_NODE__RESET_NODE_HPP_
