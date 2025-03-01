//
// Created by psaf on 01.11.24.
//

#ifndef WS_TEMPLATE_CONTROLLER_NODE_HPP
#define WS_TEMPLATE_CONTROLLER_NODE_HPP


#include "rclcpp/rclcpp.hpp"
#include "psaf_configuration/configuration.hpp"
#include "psaf_interfaces/msg/trajectory.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/range.hpp"
#include <cstdint>
#include <cmath>
#include <algorithm>
#include "opencv2//opencv.hpp"

using namespace std;

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode();
    void update();

//    static constexpr double v_max = 700.0;//1000.0 max outside(10,1s)  inside 700 stable       outside 800 stable
//    static constexpr double v_min = 450.0;//850.0  max outside(10,1s)    inside 450 stable       outside 500 stable
    double v_max = 900.0;//1000.0 max outside(10,1s)  inside 800 stable       outside 800 stable   sign detection 400
    double v_min = 850.0;//850.0  max outside(10,1s)    inside 550 stable       outside 600(700) stable  sign detection 400
    static constexpr double v_default = 0.0;
    static constexpr double L = 258;


protected:

    int current_state_ = -1;

    double current_steering_= 0.0;
    double prev_steering_original = 0.0 ;
    double prev_steering_ = 0.0;
    double prev_forward_speed_ = 0.0;
    double current_forward_speed_ = 0.0;

    std_msgs::msg::Int16 steering_msg;
    std_msgs::msg::Int16 forward_speed_msg;

    vector<vector<double>> global_trajectory;
    int sign_id = 7;
    int pre_sign_id = 7;
    int sign_id_counter;
    bool in_zone = false;

    float measured_distance = 10.0;
    bool hinder = false;
    int hinder_count = 0;
    double delta = 0.0;
    int init = 0;

    double calculate_path_curvature(const vector<vector<double>>& trajectory, int window);
    double adaptive_lookahead(int16_t v, double curvature);
    double calculate_speed_based_on_delta(double delta);
    pair<double, double> pure_pursuit_target(const vector<vector<double>>& global_trajectory);


private:
    rclcpp::Subscription<psaf_interfaces::msg::Trajectory>::SharedPtr trajectory_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sign_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr distance_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr steering_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr forward_speed_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr backward_speed_publisher_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr state_subscriber_;
    
    void trajectoryCallback(psaf_interfaces::msg::Trajectory::SharedPtr trajectory_msg);
    void signCallback(std_msgs::msg::Int32::SharedPtr sign_msg);
    void distanceCallback(sensor_msgs::msg::Range::SharedPtr msg);
    void publishsignal();
    void stateCallback(std_msgs::msg::Int16::SharedPtr msg);
};

#endif //WS_TEMPLATE_CONTROLLER_NODE_HPP
