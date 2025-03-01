//
// Created by psaf on 18.12.24.
//

#ifndef WS_TEMPLATE_STARTBOX_NODE_HPP
#define WS_TEMPLATE_STARTBOX_NODE_HPP


#ifndef WS_TEMPLATE_A_HPP
#define WS_TEMPLATE_A_HPP

#include "rclcpp/rclcpp.hpp"
#include "psaf_configuration/configuration.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "opencv2//opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include <zbar.h>
#include "std_msgs/msg/int16.hpp"

using namespace cv;
using namespace std;


class StartBoxNode : public rclcpp::Node
{
public:
    StartBoxNode();

    std::string last_read_qr_{"INIT"};
    // At least one QR-code has been read
    bool detected_at_least_once_{false};
    // The door is presumed to be open
    bool is_open_{false};
    // The current state of the state machine
    int current_state_{0};
    int status_msg = -1;
    // The amount of images where no QR-code was detected
    int no_qr_msg_counter_{0};


protected:
    void readQR(cv::Mat & img);

private:
    Mat current_image_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr state_publisher_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr status_subscriber_;


    void imageCallback(sensor_msgs::msg::Image::SharedPtr msg);
    void publishStatusInfo();
    void updateState(std_msgs::msg::Int16::SharedPtr state);
    void statusCallback(std_msgs::msg::Int16::SharedPtr msg);
};

#endif //WS_TEMPLATE_A_HPP

#endif //WS_TEMPLATE_STARTBOX_NODE_HPP