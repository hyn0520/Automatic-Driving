#include "translator_node.hpp"

#include <rcl/node_options.h>
#include <string>

using namespace std::chrono_literals;

TranslatorNode::TranslatorNode(std::string name)
: Node(name)
{
  const rclcpp::QoS QOS{rclcpp::KeepLast(1)};
  rclcpp::SensorDataQoS SQOS{rclcpp::KeepLast(10)};

  cameraPublisher = this->create_publisher<sensor_msgs::msg::Image>(
    "/color/image_raw", SQOS);

  speedPublisher = this->create_publisher<std_msgs::msg::Float64>(
    "/set_motor_level_msg", QOS);

  twistPublisher = this->create_publisher<std_msgs::msg::Float64>(
    "/set_steering_level_msg", QOS);

  frontDistancePublisher = this->create_publisher<sensor_msgs::msg::Range>(
    "/us_range_f", QOS);

  backDistancePublisher = this->create_publisher<sensor_msgs::msg::Range>(
    "/us_range_b", QOS);

  leftDistancePublisher = this->create_publisher<sensor_msgs::msg::Range>(
    "/us_range_l", QOS);
  
  rightDistancePublisher = this->create_publisher<sensor_msgs::msg::Range>(
    "/us_range_r", QOS);

  angleSubscriber = this->create_subscription<std_msgs::msg::Int16>(
    "/set_steering_angle", 10,
    std::bind(&TranslatorNode::currentAngleSubscriberCallback,
    this, std::placeholders::_1));

  timer = this->create_wall_timer(200ms, std::bind(&TranslatorNode::timerCallback, this));

  imageSubscriber = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera", QOS,
    std::bind(&TranslatorNode::imageCallback,
    this, std::placeholders::_1));

  speedSubscriber = this->create_subscription<std_msgs::msg::Int16>(
    "/set_motor_level_forward", QOS,
    std::bind(&TranslatorNode::currentSpeedSubscriberCallback,
    this, std::placeholders::_1));

  frontDistanceSubscriber = this->create_subscription<std_msgs::msg::Float64>(
    "/us_front", QOS,
    std::bind(&TranslatorNode::frontDistanceCallback,
    this, std::placeholders::_1));

  backDistanceSubscriber = this->create_subscription<std_msgs::msg::Float64>(
    "/us_back", QOS,
    std::bind(&TranslatorNode::backDistanceCallback,
    this, std::placeholders::_1));

  leftDistanceSubscriber = this->create_subscription<std_msgs::msg::Float64>(
    "/us_left", QOS,
    std::bind(&TranslatorNode::leftDistanceCallback,
    this, std::placeholders::_1));

  rightDistanceSubscriber = this->create_subscription<std_msgs::msg::Float64>(
    "/us_right", QOS,
    std::bind(&TranslatorNode::rightDistanceCallback,
    this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "translator node started");
}


// callback funkction for array of current points delivered by the camera
void TranslatorNode::timerCallback()
{
  
}

// callback funkction for array of current points delivered by the camera
void TranslatorNode::currentAngleSubscriberCallback(const std_msgs::msg::Int16::SharedPtr p)
{
  std_msgs::msg::Float64 steer_message;
  steer_message.data = p->data;

  twistPublisher->publish(steer_message);
}

// callback fucntion for array of goal points calculated by cv node
void TranslatorNode::currentSpeedSubscriberCallback(const std_msgs::msg::Int16::SharedPtr p)
{
  std_msgs::msg::Float64 message;

  message.data = p->data;

  speedPublisher->publish(message);
}

// callback for the front distance
void TranslatorNode::frontDistanceCallback(const std_msgs::msg::Float64::SharedPtr p)
{
  sensor_msgs::msg::Range message;

  message.range = p-> data;

  frontDistancePublisher->publish(message);
}

// callback for the back distance
void TranslatorNode::backDistanceCallback(const std_msgs::msg::Float64::SharedPtr p)
{
  sensor_msgs::msg::Range message;

  message.range = p-> data;

  backDistancePublisher->publish(message);
}

// callback for the left distance
void TranslatorNode::leftDistanceCallback(const std_msgs::msg::Float64::SharedPtr p)
{
  sensor_msgs::msg::Range message;

  message.range = p-> data;

  leftDistancePublisher->publish(message);
}

// callback for the right distance
void TranslatorNode::rightDistanceCallback(const std_msgs::msg::Float64::SharedPtr p)
{
  sensor_msgs::msg::Range message;

  message.range = p-> data;

  rightDistancePublisher->publish(message);
}

// callback for image
void TranslatorNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr p)
{
  sensor_msgs::msg::Image message;

  message.data = p->data;
  message.header = p->header;
  message.encoding = p->encoding;
  message.height = p->height;
  message.width = p->width;
  message.step = p->step;
  message.is_bigendian = p->is_bigendian;

  cameraPublisher->publish(message);
}


