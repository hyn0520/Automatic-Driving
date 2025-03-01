//
// Created by psaf on 01.11.24.
//

#ifndef WS_TEMPLATE_TRAJECTORY_PLAN_NODE_HPP
#define WS_TEMPLATE_TRAJECTORY_PLAN_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "psaf_configuration/configuration.hpp"
#include "psaf_interfaces/msg/trajectory.hpp"
#include "psaf_interfaces/msg/lane_marking.hpp"
#include "opencv2//opencv.hpp"
#include "std_msgs/msg/int16.hpp"

using namespace cv;
using namespace std;

class TrajectoryPlanNode : public rclcpp::Node
{
public:
    TrajectoryPlanNode();

protected:
    vector<Point2f> prev_transformed_trajectory;
    vector<Point2f> transformed_trajectory;
    psaf_interfaces::msg::Trajectory trajectory_msg;

    std::vector<cv::Point> calculate_trajectory(const std::vector<cv::Point>& right_lane,
                                                const std::vector<cv::Point>& center_lane,
                                                const std::vector<cv::Point>& left_lane);

    std::vector<cv::Point2f> transform_to_car_coordinate_system(const std::vector<cv::Point>& trajectory);
    double quadraticFunc(double y, double a, double b, double c);
    double linearFunc(double y, double a, double b);
    int findClosestPointIndex(const vector<Point2f>& trajectory, float target_x);
    void plotAndSaveTrajectory(const vector<Point2f>& transformed_trajectory);
    bool if_initialized = true;

    int current_state_ = -1;

private:
    rclcpp::Subscription<psaf_interfaces::msg::LaneMarking>::SharedPtr lane_markings_subscriber_;
    rclcpp::Publisher< psaf_interfaces::msg::Trajectory>::SharedPtr trajectory_publisher_;

    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr err_publisher_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr state_subscriber_;

    void lane_markingCallback(psaf_interfaces::msg::LaneMarking::SharedPtr LaneMarking);
    void publishTrajectory(const vector<cv::Point2f>& transformed_trajectory);
    void stateCallback(std_msgs::msg::Int16::SharedPtr msg);
};


#endif //WS_TEMPLATE_TRAJECTORY_PLAN_NODE_HPP
