//
// Created by Administrator on 2024/11/21.
//

/* LaneDetectionNode.hpp */
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
#include "std_msgs/msg/int32.hpp"
#include <opencv2/ximgproc.hpp>

using namespace cv;
using namespace cv::ximgproc;
using namespace std;

struct BasePointInfo{
    int lBase = 0;
    int rBase = 0;
    int cBase = 0;
    bool searchL = false;
    bool searchR = false;
    bool searchC = false;
    
    };
class LaneDetectionOutsideNode : public rclcpp::Node
{
public:
    LaneDetectionOutsideNode();
    void processImage(const Mat& img);
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr num_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
protected:
    cv::Mat homography;
    int sl_width;
    int sl_height;
    
    
    

    std::vector<cv::Point> right_lane;
    std::vector<cv::Point> center_lane;
    std::vector<cv::Point> left_lane;
    std::vector<std::vector<cv::Point>> lane_marking_;
    Mat output_img;

    cv::Point startpoint = cv::Point(0,0);
    cv::Point second_startpoint = cv::Point(0,0) ;

    void extractLaneMarkings(const cv::Mat& img, cv::Mat& debugImage);
    std::pair<cv::Point, bool> getMaxContour(const cv::Mat& img);
    std::vector<cv::Point> getBasePoints(const cv::Mat& img, cv::Mat& debugImage);
    BasePointInfo getBasePointsInformation(const std::vector<cv::Point>& basePoints, const cv::Size& imageSize);


private:
    cv::Mat current_image_;
    
    
    rclcpp::Publisher<psaf_interfaces::msg::LaneMarking>::SharedPtr lane_markings_publisher_;
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void numCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void publishLaneMarkings(const std::vector<std::vector<cv::Point>>& lane_marking_);

    int img_count;

    //cv::KalmanFilter kalman;



};

#endif /* LANE_DETECTION_NODE_HPP */
