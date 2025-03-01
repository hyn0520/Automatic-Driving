#ifndef TRANSLATOR_NODE_
#define TRANSLATOR_NODE_


#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int16.hpp"
#include "sensor_msgs/msg/range.hpp"
#include <cmath>


class TranslatorNode : public rclcpp::Node
{
public:
  TranslatorNode()
  : rclcpp::Node("translator_node") {}
  explicit TranslatorNode(std::string name);

private:
  /**
   * Publisher for motor level in case of too big derivate
   * Topic: /cmd_vel
   */
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speedPublisher;

  /**
   * Publisher for distance of front ultrasonic sensor
   * Topic: /cmd_vel_2
   */
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr frontDistancePublisher;

   /**
   * Publisher for distance of front ultrasonic sensor
   * Topic: /cmd_vel_2
   */
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr backDistancePublisher;

   /**
   * Publisher for distance of front ultrasonic sensor
   * Topic: /cmd_vel_2
   */
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr leftDistancePublisher;

   /**
   * Publisher for distance of front ultrasonic sensor
   * Topic: /cmd_vel_2
   */
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr rightDistancePublisher;

  /**
   * Publisher for motor level in case of too big derivate
   * Topic: /cmd_vel
   */
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr twistPublisher;

  /**
   * Publisher for motor level in case of too big derivate
   * Topic: /cmd_vel_2
   */
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr twistPublisher2;

  /**
   * Publisher for iamge
   * Topic: /camera/color/image_raw
   */
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cameraPublisher;

  /**
   * Subscriber for state of statemachine
   * Topic: /statemachine/state
   */
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr angleSubscriber;

  /**
   * Subscriber for right distance of car to lane (currently unused)
   * Topic: /lance_detection/right_distance
   */
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr speedSubscriber;

  /**
   * Subscriber for the measured distance of the front ultrasonic sensor
   * Topic: /lance_detection/right_distance
   */
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr backDistanceSubscriber;

  /**
   * Subscriber for the measured distance of the front ultrasonic sensor
   * Topic: /lance_detection/right_distance
   */
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr leftDistanceSubscriber;

  /**
   * Subscriber for the measured distance of the front ultrasonic sensor
   * Topic: /lance_detection/right_distance
   */
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rightDistanceSubscriber;

  /**
   * Subscriber for the measured distance of the front ultrasonic sensor
   * Topic: /lance_detection/right_distance
   */
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr frontDistanceSubscriber;

  /**
   * Subscriber for iamge
   * Topic: /camera
   */
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubscriber;

  /**
   * a timer
   */
  rclcpp::TimerBase::SharedPtr timer;

  /**
   * the current angle
   */
  double angle = 0.0;

  /**
   * the current speed
   */
  double speed = 0.0;

public:
  /**
   * \brief timer callback
   */
  void timerCallback();

  /**
   * \brief callback funkction for current respective angle
   *        of the car to the road delivered by the camera
   */
  void currentAngleSubscriberCallback(const std_msgs::msg::Int16::SharedPtr p);

  /**
   * \brief callback funkction for current respective angle
   *        of the car to the road delivered by the camera
   */
  void currentSpeedSubscriberCallback(const std_msgs::msg::Int16::SharedPtr p);

  /**
   * \brief callback funkction for current front disctance
   */
  void frontDistanceCallback(const std_msgs::msg::Float64::SharedPtr p);

  /**
   * \brief callback funkction for current front disctance
   */
  void backDistanceCallback(const std_msgs::msg::Float64::SharedPtr p);

  /**
   * \brief callback funkction for current front disctance
   */
  void leftDistanceCallback(const std_msgs::msg::Float64::SharedPtr p);

  /**
   * \brief callback funkction for current front disctance
   */
  void rightDistanceCallback(const std_msgs::msg::Float64::SharedPtr p);

  /**
   * \brief callback funkction for iamge
   */
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr p);
};

#endif