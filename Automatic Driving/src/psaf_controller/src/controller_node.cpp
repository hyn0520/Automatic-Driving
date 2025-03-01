//
// Created by psaf on 01.11.24.
//
#include "psaf_controller/controller_node.hpp"

/**
 * @brief Constructs a new ControllerNode object and initializes all subscribers and publishers.
 */
ControllerNode::ControllerNode() : Node(CONTROLLER_NODE) {
    trajectory_subscriber_ = this->create_subscription<psaf_interfaces::msg::Trajectory>(
            TRAJECTORY_TOPIC, 10, std::bind(&ControllerNode::trajectoryCallback, this, std::placeholders::_1));

    sign_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
            "road_sign_class", 10, std::bind(&ControllerNode::signCallback, this, std::placeholders::_1));

    steering_publisher_ = this->create_publisher<std_msgs::msg::Int16>(
            SET_STEERING_TOPIC, 10);

    forward_speed_publisher_ = this->create_publisher<std_msgs::msg::Int16>(
            SET_SPEED_FORWARD_TOPIC, 10);

    backward_speed_publisher_ = this->create_publisher<std_msgs::msg::Int16>(
            SET_SPEED_BACKWARD_TOPIC, 10);

    state_subscriber_ = this->create_subscription<std_msgs::msg::Int16>(
            "StateInfo", 10, std::bind(&ControllerNode::stateCallback, this, std::placeholders::_1));
}

/**
 * @brief Handles state updates and sets velocity limits accordingly.
 * 
 * @param msg The received state message.
 */
void ControllerNode::stateCallback(std_msgs::msg::Int16::SharedPtr msg){
    current_state_ = msg->data;
    switch (current_state_)
    {
        case -1: // Stop in Startbox
            v_max = 0.0;
            v_min = 0.0;
            break;
            
        case  1: // outer circle
            v_max = 1000.0;
            v_min = 850.0;
            break;

        case 2: // inner circle
            v_max = 750.0;
            v_min = 600.0;
            break;

        case 3: // Sign detection
            v_max = 400.0;
            v_min = 400.0;
            break;

        default:
            break;
    }
}

/**
 * @brief Processes incoming trajectory messages and updates the global trajectory.
 * 
 * @param trajectory_msg The received trajectory message.
 */
void ControllerNode::trajectoryCallback(psaf_interfaces::msg::Trajectory::SharedPtr trajectory_msg) {
    RCLCPP_INFO(this->get_logger(), "Received trajectory message");
    global_trajectory.clear();
    for (const auto& point : trajectory_msg->trajectory) {
        global_trajectory.push_back({point.x, point.y});
    }
}

/**
 * @brief Processes incoming road sign messages and updates the sign ID.
 * 
 * @param sign_msg The received road sign message
 */
void ControllerNode::signCallback(std_msgs::msg::Int32::SharedPtr sign_msg) {
    RCLCPP_INFO(this->get_logger(), "Received sign message");
    sign_id = sign_msg->data;
    RCLCPP_INFO(this->get_logger(), "Sign========================================================================================================================: %d", sign_id);
}

/**
 * @brief Updates the control signals based on the current trajectory and state
 */
void ControllerNode:: update(){
    if (global_trajectory.empty() ) {
        RCLCPP_WARN(this->get_logger(), "Global trajectory is empty. Skipping update.");

        current_forward_speed_ = prev_forward_speed_;
        current_steering_ = prev_steering_;

        publishsignal();
    }else{
        auto start = std::chrono::steady_clock::now();
        auto result = pure_pursuit_target(global_trajectory);
        double target_heading = result.first;
        
        double target_heading_new = target_heading;
        RCLCPP_INFO(this->get_logger(), "-------------------------------------------------------target_heading: %f", target_heading *  180 / M_PI);

        double derivative = (current_steering_ - prev_steering_) / 0.001; // Calculate the derivative

        double kp = 2.0046; // Proportional gain (from existing P controller)
        double kd = 0.0;    // Derivative gain (adjust this to fine-tune performance
        
        current_steering_ = ((target_heading_new * 180 / M_PI) * 1.6 + 2.244 + kd * derivative) * 10;
        
        prev_steering_original = target_heading;
        prev_steering_ = current_steering_ ;
        prev_forward_speed_ = current_forward_speed_ = result.second;        
        
        publishsignal();
        auto end = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    }

}

/**
 * @brief Publishes steering and speed signals to the topics
 */
void ControllerNode::publishsignal(){

    steering_msg.data = std::clamp(static_cast<int16_t>(current_steering_ ), static_cast<int16_t>(-300), static_cast<int16_t>(300));//inside: 300

    RCLCPP_INFO(this->get_logger(), "steering: %d", steering_msg.data);

    if (current_state_==3){
        if (sign_id_counter < 30 || sign_id != pre_sign_id){

            forward_speed_msg.data = static_cast<int16_t>(current_forward_speed_);
            if (sign_id == pre_sign_id){
                sign_id_counter += 1;
            }
            else{
                sign_id_counter = 0 ;
            }
        }
        else{
            switch (sign_id) {
                case 0: // Label: Richtungsweiser (Sharp curve) 
                    if(in_zone){
                        forward_speed_msg.data = 250;
                    }
                    else{
                        forward_speed_msg.data = static_cast<int16_t>(current_forward_speed_);
                    }
                    RCLCPP_INFO(this->get_logger(), "Forward speed: %d", forward_speed_msg.data);
                    break;

                case 1: // Label: fußgaenger (pedestrian crossing)
                    forward_speed_msg.data = 150;
                    RCLCPP_INFO(this->get_logger(), "Forward speed: %d", forward_speed_msg.data);
                    break;

                case 2: // Label: stop
                    v_max = 400.0;
                    v_min = 400.0;
                    forward_speed_msg.data = 0;
                    RCLCPP_INFO(this->get_logger(), "Forward speed: %d", forward_speed_msg.data);
                    break;

                case 3: // Label: tempo 30
                    v_max = 250.0;
                    v_min = 250.0;
                    RCLCPP_INFO(this->get_logger(), "Forward speed: %d", forward_speed_msg.data);
                    break;

                case 4: // Label: tempo 30 zone beginn
                    v_max = 400.0;
                    v_min = 400.0;
                    in_zone = true;
                    forward_speed_msg.data = 250;
                    RCLCPP_INFO(this->get_logger(), "Signal case 4: Custom logic here.");
                    break;

                case 5: // Label: tempo 30 zone ende
                    in_zone = false;
                    forward_speed_msg.data = static_cast<int16_t>(current_forward_speed_);
                    RCLCPP_INFO(this->get_logger(), "Forward speed: %d", forward_speed_msg.data);
                    break;

                case 6: // Label: vorfahrt achten
                    forward_speed_msg.data = 150;
                    RCLCPP_INFO(this->get_logger(), "Forward speed: %d", forward_speed_msg.data);
                    break;

                case 7: // no sign detected
                    if(in_zone){
                        forward_speed_msg.data = 250;
                    }
                    else{
                        forward_speed_msg.data = static_cast<int16_t>(current_forward_speed_);
                    }
                    RCLCPP_INFO(this->get_logger(), "Forward speed: %d", forward_speed_msg.data);
                    break;

                default: // Dafault case normal speed
                    forward_speed_msg.data = static_cast<int16_t>(current_forward_speed_);
                    RCLCPP_INFO(this->get_logger(), "Forward speed: %d", forward_speed_msg.data);
                    break;
                }
            }
        pre_sign_id = sign_id ;
    }
    else{
        forward_speed_msg.data = static_cast<int16_t>(current_forward_speed_);
    }
    forward_speed_publisher_->publish(forward_speed_msg);

    RCLCPP_INFO(this->get_logger(), "target_heading: %d speed: %d", steering_msg.data, forward_speed_msg.data);

    steering_publisher_->publish(steering_msg);
}

/**
 * @brief Calculates the curvature of a given trajectory.
 * 
 * @param trajectory The trajectory points
 * @param window The window size for curvature estimation
 * @return The average curvature of the trajectory
 */
double ControllerNode::calculate_path_curvature(const vector<vector<double>>& trajectory, int window) {

    //1. Collect x, y coordinates (from the first window+1 point of the trajectory)
    vector<double> x, y;
    x.reserve(window + 1);
    y.reserve(window + 1);
    for (int i = 0; i <= window; ++i) {
        x.push_back(trajectory[i][0]);
        y.push_back(trajectory[i][1]);
    }

    // 2. Use central difference to calculate first and second order derivatives
    //    Only calculate i = 1..window-1 to ensure access to i-1, i+1
    double curvature_sum = 0.0;
    int valid_count = 0;

    double dx, dy, d2x, d2y, denom_sq, curvature_val

    for (int i = 1; i < window; ++i) {
        // (a) Central difference calculation of the first-order derivatives dx, dy
        dx = (x[i + 1] - x[i - 1]) * 0.5;
        dy = (y[i + 1] - y[i - 1]) * 0.5;

        // (b) Second-order derivatives d2x, d2y
        d2x = x[i + 1] - 2.0 * x[i] + x[i - 1];
        d2y = y[i + 1] - 2.0 * y[i] + y[i - 1];

        // (c) Check if the denominator is too small: (dx^2 + dy^2)^(3/2)
        //     If dx, dy are very small (the points almost coincide or are very close), the denominator is close to 0
        denom_sq = dx * dx + dy * dy; // Here is (dx^2 + dy^2)
        if (denom_sq < 1e-12) {
            // If the denominator is too small, skip the curvature calculation of this point.
            continue;
        }

        // (d) Calculating curvature kappa = |dx * d2y - dy * d2x| / ( (dx^2 + dy^2)^(3/2) )
        curvature_val = fabs(dx * d2y - dy * d2x) / pow(denom_sq, 1.5);

        curvature_sum += curvature_val;
        valid_count++;
    }    
    // 3. Calculates the mean curvature (or returns the sum if desired)
    //    If all points are skipped, valid_count = 0 and returns 0
    return (valid_count > 0) ? (curvature_sum / valid_count) : 0.0;
}

/**
 * @brief Computes an adaptive lookahead distance based on speed and curvature.
 * 
 * @param v The current speed
 * @param curvature The calculated curvature
 * @return The computed lookahead distance
 */
double ControllerNode::adaptive_lookahead(int16_t v, double curvature) {
    // Adaptive preview distance based on speed and curvature
    const double min_distance = 800;//inside 650     outside 800(1000)      sign detection 700
    const double max_distance = 1000;//inside 1100   outside 1000     sign detection 1000

    double v_factor = 0.5 * v;
    double k_factor = exp(-2.0 * abs(curvature));

    double ld = (min_distance + v_factor) * k_factor;
    RCLCPP_INFO(this->get_logger(), "-----------------------------------distance: %f", max(min(ld, max_distance), min_distance));
    return max(min(ld, max_distance), min_distance);
}

/**
 * @brief Determines the vehicle's speed based on steering angle (delta).
 * 
 * @param delta The computed steering angle
 * @return The adjusted speed value
 */
double ControllerNode::calculate_speed_based_on_delta(double delta) {
    // Calculate the speed of adaptation based on curvature
    if (abs(delta) < M_PI / 36) {  // Approaching the straight
        return v_max;
    } else {  // Slow down on curves
        return v_min;
    }
}

/**
 * @brief Pure pursuit algorithm to find the target point and required steering angle
 * 
 * @param global_trajectory The global trajectory points
 * @return A pair containing the steering angle and target speed
 */
pair<double, double> ControllerNode::pure_pursuit_target(const vector<vector<double>>& global_trajectory) {

    double curvature = calculate_path_curvature(global_trajectory, 10);
    RCLCPP_INFO(this->get_logger(), "-----------------------------------curvature: %f", curvature);

    double lookahead = adaptive_lookahead(current_forward_speed_, curvature);
    double speed, target_x, target_y, alpha, ld;

    for (size_t i = 0; i < global_trajectory.size(); ++i) {
        double dist = sqrt(pow(global_trajectory[i][0], 2) + pow(global_trajectory[i][1], 2));

        if (dist >= lookahead) {
            target_x = global_trajectory[i][0];
            target_y = global_trajectory[i][1];

            alpha = atan2(target_y, target_x);  // Deflection angle α

            RCLCPP_INFO(this->get_logger(), "alpha: %f", alpha);

            ld = dist;  // Preview distance

            // Calculate the steering angle δ
            delta = atan((2 * L * sin(alpha)) / ld);

            // Limit the rotation angle to +/- 45 degrees
            delta = max(min(delta, M_PI / 6), -M_PI / 6);  // Limit radians to ±π/4 (i.e. ±45 degrees)

            speed = calculate_speed_based_on_delta(delta);

            RCLCPP_INFO(this->get_logger(), "delta: %f", delta);
            return {delta, speed};
        }
    }
    // If no suitable preview point is found, the last point is returned.
    vector<double> target_point = global_trajectory.back();
    target_x = target_point[0];
    target_y = target_point[1];

    alpha = atan2(target_y, target_x);  // Deflection angle α

    RCLCPP_INFO(this->get_logger(), "alpha: %f", alpha);

    ld = sqrt(pow(target_x, 2) + pow(target_y, 2));

    // Calculate the steering angle δ
    delta = atan((2 * L * sin(alpha)) / ld);

    // Limit the rotation angle to +/- 45 degrees
    delta = max(min(delta, M_PI / 6), -M_PI / 6);  // Limit radians to ±π/4 (i.e. ±45 degrees)

    speed = calculate_speed_based_on_delta(delta);

    RCLCPP_INFO(this->get_logger(), "delta: %f", delta);
    return {delta, speed};
}