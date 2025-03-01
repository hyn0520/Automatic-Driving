//
// Created by psaf on 27.11.19.
//
#include "translator_node.hpp"
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

/*!
 *
 * \brief Angle controller node for controlling the steering
 *
 * \param argc An integer argument count of the command line arguments
 * \param argv An argument vector of the command line arguments
 * \return Status of the main program
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TranslatorNode>("TranslatorNode");

  rclcpp::WallRate loopRate(20);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    loopRate.sleep();
  }
  // rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
