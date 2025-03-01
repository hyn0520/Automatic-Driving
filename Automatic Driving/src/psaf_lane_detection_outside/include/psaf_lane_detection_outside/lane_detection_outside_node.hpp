//
// Created by psaf on 01.11.24.
//

#ifndef WS_TEMPLATE_A_HPP
#define WS_TEMPLATE_A_HPP

#include "rclcpp/rclcpp.hpp"
#include "psaf_configuration/configuration.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "opencv2//opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "psaf_interfaces/msg/point.hpp"
#include "psaf_interfaces/msg/lane_marking.hpp"
#include <opencv2/ximgproc.hpp>

using namespace cv;
using namespace cv::ximgproc;
using namespace std;



class LaneDetectionOutsideNode : public rclcpp::Node
{
public:
    LaneDetectionOutsideNode();
    double homography_matrix[9]={
            -3.038122819655312, -8.672877526884868, 2377.9484334015165, 0.29213993514510084,
            -16.02172757573472, 2881.3325514309886, 0.0004933373163602723, -0.02681487260493437,
            1.0
    };

//    new H matrix
//    double homography_matrix[9]={
//            -1.6516428499606437, -4.100057233423164, 1300.5237337736692, -0.19308774762766134,
//            -7.608093593980515, 1680.0475377360606, -0.00039937867393503266, -0.012594507220985117,
//            1.0
//    };

protected:

    Mat output_img;

    std::vector<std::vector<cv::Point>> lane_marking_;


    void update();
    void processImageOutercircle(const Mat& img, int padding);
    std::pair<std::vector<cv::Point>, std::vector<cv::Rect>> sliding_window_sampling_right_line(
            const cv::Mat& image, cv::Point base_point, int window_width = 120, int window_height = 40);

    std::pair<std::vector<cv::Point>, std::vector<cv::Rect>> sliding_window_sampling_center_line(
            const cv::Mat& image, cv::Point base_point, int window_width = 120, int window_height = 15);

private:
    cv::Mat current_image_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Publisher<psaf_interfaces::msg::LaneMarking>::SharedPtr lane_markings_publisher_;

    void imageCallback(sensor_msgs::msg::Image::SharedPtr msg);
    void publishLaneMarkings(const std::vector<std::vector<cv::Point>>& lane_marking_);
    //...... weitere Funktionen!
};




#endif //WS_TEMPLATE_A_HPP
