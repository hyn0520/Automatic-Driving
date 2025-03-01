//
// Created by psaf on 01.11.24.
//

#ifndef WS_TEMPLATE_A_HPP
#define WS_TEMPLATE_A_HPP

#include "rclcpp/rclcpp.hpp"
#include "psaf_configuration/configuration.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/int16.hpp"
#include "opencv2//opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "psaf_interfaces/msg/point.hpp"
#include "psaf_interfaces/msg/lane_marking.hpp"
#include <opencv2/ximgproc.hpp>

using namespace cv;
using namespace cv::ximgproc;
using namespace std;



class LaneDetectionInsideNode : public rclcpp::Node
{
public:
    LaneDetectionInsideNode();
    double homography_matrix[9]={
            -3.038122819655312, -8.672877526884868, 2377.9484334015165, 0.29213993514510084,
            -16.02172757573472, 2881.3325514309886, 0.0004933373163602723, -0.02681487260493437,
            1.0
    };


protected:

    Mat output_img;

    std::vector<std::vector<cv::Point>> lane_marking_;

    float measured_distance = 10.0;
    bool hinder = false;
    int hinder_count = 0;

    void update();
    void processImageNormal(const Mat& img, int padding);
    void processImageReverse(const Mat& img, int padding);
    std::pair<std::vector<cv::Point>, std::vector<cv::Rect>> sliding_window_sampling_right_line(
            const cv::Mat& image, cv::Point base_point, int window_width = 120, int window_height = 20);

    std::pair<std::vector<cv::Point>, std::vector<cv::Rect>> sliding_window_sampling_center_line(
            const cv::Mat& image, cv::Point base_point, int window_width = 120, int window_height = 15);

private:
    cv::Mat current_image_;
    int current_state_ = -1;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr distance_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr state_subscriber_;
    rclcpp::Publisher<psaf_interfaces::msg::LaneMarking>::SharedPtr lane_markings_publisher_;

    void imageCallback(sensor_msgs::msg::Image::SharedPtr msg);
    void publishLaneMarkings(const std::vector<std::vector<cv::Point>>& lane_marking_);
    void distanceCallback(sensor_msgs::msg::Range::SharedPtr msg);
    void stateCallback(std_msgs::msg::Int16::SharedPtr msg);
};

#endif //WS_TEMPLATE_A_HPP