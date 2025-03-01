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

protected:

    Mat output_img;

    std::vector<std::vector<cv::Point>> lane_marking_;
//    psaf_interfaces::msg::LaneMarking LaneMarking_msg;

    void update();
    void drawSamplingPoints(Mat& img, const vector<Point>& sampledPoints, const Scalar& color);
    vector<Point> samplingPoints(const Mat& skeleton);
    int findSkeletonWithMaxAvgX(const vector<Mat>& skeletons);
    vector<Mat> clusterSkeletons(const vector<Mat>& skeletons, int radius);
    vector<Mat> generateSkeletons(const vector<vector<Point>>& contours, Size binaryShape);
    void processImageOutercircle(const Mat& img, int padding);

private:
    cv::Mat current_image_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Publisher<psaf_interfaces::msg::LaneMarking>::SharedPtr lane_markings_publisher_;

    void imageCallback(sensor_msgs::msg::Image::SharedPtr msg);
    void publishLaneMarkings(const std::vector<std::vector<cv::Point>>& lane_marking_);
    //...... weitere Funktionen!
};




#endif //WS_TEMPLATE_A_HPP
