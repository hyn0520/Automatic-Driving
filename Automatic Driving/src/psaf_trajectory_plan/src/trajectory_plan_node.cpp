//
// Created by psaf on 01.11.24.
//

#include "psaf_trajectory_plan/trajectory_plan_node.hpp"
#include <atomic> 
std::atomic<int> file_counter(0);

/**
 * @brief Constructs the TrajectoryPlanNode object and initializes subscribers and publishers.
 */
TrajectoryPlanNode::TrajectoryPlanNode() : Node(TRAJECTORY_PLAN_NODE) {

    // Main Subscriber for the Lane Markings from Node Lane_Detection
    lane_markings_subscriber_ = this->create_subscription<psaf_interfaces::msg::LaneMarking>(
            LANE_MARKINGS_TOPIC, 10, std::bind(&TrajectoryPlanNode::lane_markingCallback, this, std::placeholders::_1)); 

    // Main Publisher for the calculated Trajectory 
    trajectory_publisher_ = this->create_publisher<psaf_interfaces::msg::Trajectory>(
            TRAJECTORY_TOPIC, 10);

    // Error-Publisher
    err_publisher_ = this->create_publisher<std_msgs::msg::Int16>(
            "error", 10);

    // State-Subscriber for current state (inner lane, outer lane, ...)
    state_subscriber_ = this->create_subscription<std_msgs::msg::Int16>(
            "StateInfo", 10, std::bind(&TrajectoryPlanNode::stateCallback, this, std::placeholders::_1));
}

/**
 * @brief Callback function for state updates.
 * @param msg Shared pointer to the state message.
 */
void TrajectoryPlanNode::stateCallback(std_msgs::msg::Int16::SharedPtr msg){
    current_state_ = msg->data;
}


/**
 * @brief Callback function for processing lane marking messages and calculating trajectories.
 * @param LaneMarking Shared pointer to lane marking message.
 */
void TrajectoryPlanNode::lane_markingCallback(psaf_interfaces::msg::LaneMarking::SharedPtr LaneMarking){

    std::vector<cv::Point> right_lane;
    std::vector<cv::Point> center_lane;
    std::vector<cv::Point> left_lane;
    std::vector<cv::Point> trajectory;
    const float threshold = 400.0f;
    const float target_point = 1200.0f;

    auto start = std::chrono::steady_clock::now();

    // save the received lane Markings in name-equivalent vectors
    RCLCPP_INFO(this->get_logger(), "Trajectory start callback");
    for (const auto& point : LaneMarking->right_lane) {
        right_lane.emplace_back(static_cast<int>(point.x), static_cast<int>(point.y));
    }

    for (const auto& point : LaneMarking->center_lane) {
        center_lane.emplace_back(static_cast<int>(point.x), static_cast<int>(point.y));
    }

    for (const auto& point : LaneMarking->left_lane) {
        left_lane.emplace_back(static_cast<int>(point.x), static_cast<int>(point.y));
    }

    // calls calculate_trajectory and returns the calculated trajectory to variable trajectory
    trajectory = calculate_trajectory(right_lane, center_lane, left_lane);

    // tranform the trajectory to car-coordinates 
    transformed_trajectory = transform_to_car_coordinate_system(trajectory);

    // saves the current trajectory for later use at first call 
    if(if_initialized){
        prev_transformed_trajectory = transformed_trajectory;
        if_initialized = false;
    }

    // Find the point closest to target_point for the current and previous trajectory 
    int index_transformed = findClosestPointIndex(transformed_trajectory, target_point);
    int index_prev_transformed = findClosestPointIndex(prev_transformed_trajectory, target_point);

    RCLCPP_INFO(this->get_logger(), "index_transformed: %d", index_transformed);
    RCLCPP_INFO(this->get_logger(), "index_prev_transformed: %d", index_prev_transformed);

    // When closest Points for both trajectorys are found, check for unreasonable differences between them
    // If the difference is to big, discard the new trajectory
    if (index_transformed != -1 && index_prev_transformed != -1) {
        
        float y_transformed = transformed_trajectory[index_transformed].y;
        float y_prev_transformed = prev_transformed_trajectory[index_prev_transformed].y;

        // Print closest point
        RCLCPP_INFO(this->get_logger(), "Closest point in transformed_trajectory: (%f, %f)",
                    transformed_trajectory[index_transformed].x, y_transformed);
        RCLCPP_INFO(this->get_logger(), "Closest point in prev_transformed_trajectory: (%f, %f)",
                    prev_transformed_trajectory[index_prev_transformed].x, y_prev_transformed);

        // Discards the new trajectory if the difference is greater then the threshold
        if (abs(y_transformed - y_prev_transformed) > threshold){    
            RCLCPP_INFO(this->get_logger(), "Difference in y > 400. Assigning prev_transformed_trajectory to transformed_trajectory.");
            transformed_trajectory = prev_transformed_trajectory;
        }
    }

    // save the current trajectory for the next call
    prev_transformed_trajectory = transformed_trajectory;

    // publish the trajectory 
    publishTrajectory(transformed_trajectory);
    transformed_trajectory.clear();

    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    RCLCPP_INFO(this->get_logger(), "=====================================================Trajectory Processing time: %ld ms", duration);
}

/**
 * @brief Publishes the calculated trajectory.
 * @param transformed_trajectory The trajectory points in the transformed coordinate system.
 */
void TrajectoryPlanNode::publishTrajectory(const vector<cv::Point2f>& transformed_trajectory){
    // Traverse in reverse order (transformed_trajectory)
    for (auto it = transformed_trajectory.rbegin(); it != transformed_trajectory.rend(); ++it) {
        psaf_interfaces::msg::FloatPoint point_msg;
        point_msg.x = static_cast<float>(it->x);
        point_msg.y = static_cast<float>(it->y);
        trajectory_msg.trajectory.push_back(point_msg);
    }

    // Post a message
    trajectory_publisher_->publish(trajectory_msg);
    trajectory_msg.trajectory.clear();
}

/**
 * @brief Calculates the trajectory and uses a quadratic approximation to smooth it
 * @param right_lane Vector with the right lane in picture-coordinates (640 x 480)
 * @param center_lane Vector with the center lane in picture-coordinates (640 x 480)
 * @param left_lane Vector with the left lane in picture-coordinates (640 x 480)
 * @return Vector which containes the calculated trajectory after the quadratic approximation
 */
vector<Point> TrajectoryPlanNode::calculate_trajectory(const vector<Point>& right_lane, const vector<Point>& center_lane, const vector<Point>& left_lane) {
    
    int y1, y2, x1, x2, xi, yi;
    double dx, dy, length, nx, ny, mid_x, mid_y, left_x, left_y, right_x, right_y, x, y;

    // If the right line and center line are both empty, an empty track list is returned.
    if (right_lane.empty() && center_lane.empty()) {
        return {};
    }

    vector<Point> trajectory; // Used to store the final trajectory points

    // Calculate the left endpoint of the mid-vertical line based on right_lane
    if (!right_lane.empty()) {
        for (size_t i = 0; i < right_lane.size() - 1; ++i) {
            // Get two adjacent points
            y1 = right_lane[i].y;
            x1 = right_lane[i].x;
            y2 = right_lane[i + 1].y;
            x2 = right_lane[i + 1].x;

            // Calculate the direction vector of the vertical line
            dx = x2 - x1;
            dy = y2 - y1;
            length = sqrt(dx * dx + dy * dy);
            if (length == 0) {
                continue; // If length is 0, skip this point pair
            }

            // Calculate the unit vector in the vertical direction
            nx = -dy / length;
            ny = dx / length;

            // Calculate the left endpoint of the vertical line (move 90 pixels to the left)
            mid_x = (x1 + x2) / 2.0;
            mid_y = (y1 + y2) / 2.0; // find midpoint

            if(current_state_==2){
                left_x = mid_x - nx * 90;   //outside 90
                left_y = mid_y - ny * 90;   //outside 90
            }
            else{
                left_x = mid_x - nx * 90;   //inside 90 
                left_y = mid_y - ny * 90;   //inside 90 
            }

            // Add the left endpoint to the trajectory
            trajectory.emplace_back(cvRound(left_x), cvRound(left_y));
        }
    }

    // Calculate the right endpoint of the center vertical line based on center_lane
    if (!center_lane.empty()) {
        for (size_t i = 0; i < center_lane.size() - 1; ++i) {

            // Get two adjacent points
            y1 = center_lane[i].y;
            x1 = center_lane[i].x;
            y2 = center_lane[i + 1].y;
            x2 = center_lane[i + 1].x;

            // Calculate the direction vector of the vertical line
            dx = x2 - x1;
            dy = y2 - y1;
            length = sqrt(dx * dx + dy * dy);
            if (length == 0) {
                continue; // If length is 0, skip this point pair
            }

            // Calculate the unit vector in the vertical direction
            nx = -dy / length;
            ny = dx / length;

            // Calculate the right endpoint of the vertical line (moved 90 pixels to the right)
            mid_x = (x1 + x2) / 2.0;
            mid_y = (y1 + y2) / 2.0; // find midpoint

            if(current_state_==2){
                right_x = mid_x + nx * 90;  //outside 90 
                right_y = mid_y + ny * 90;  //outside 90
            }
            else{
                right_x = mid_x + nx * 90;   //inside 90
                right_y = mid_y + ny * 90;   //inside 90 
            }

            // Add the right endpoint to the trajectory
            trajectory.emplace_back(cvRound(right_x), cvRound(right_y));
        }
    }

    // Sort trajectories from largest to smallest on the y-axis
    sort(trajectory.begin(), trajectory.end(), [](const Point& a, const Point& b) {
        return a.y > b.y;
    });

    vector<Point> final_trajectory;

    if (trajectory.size() > 2) { // Fit only when there are enough points
        // Extract x and y coordinates
        vector<double> y_points, x_points;
        for (const auto& point : trajectory) {
            y_points.push_back(point.y);
            x_points.push_back(point.x);
        }

        // Fit trajectories using least squares method
        try {
            // Construct matrix A and vector B
            Mat A(y_points.size(), 3, CV_64F);
            Mat B(x_points.size(), 1, CV_64F);
            double y;
            for (size_t i = 0; i < y_points.size(); ++i) {
                y = y_points[i];
                A.at<double>(i, 0) = y * y;
                A.at<double>(i, 1) = y;
                A.at<double>(i, 2) = 1.0;
                B.at<double>(i, 0) = x_points[i];
            }

            // Solve the least squares problem
            Mat params;
            solve(A, B, params, DECOMP_QR);

            double a = params.at<double>(0, 0);
            double b = params.at<double>(1, 0);
            double c = params.at<double>(2, 0);

            // Generate smoothed trajectories
            double y_min = *min_element(y_points.begin(), y_points.end());
            double y_max = *max_element(y_points.begin(), y_points.end());
            int num_points = static_cast<int>(trajectory.size()) * 15;
            vector<double> y_smooth(num_points);
            double y_step = (y_max - y_min) / (num_points - 1);
            for (int i = 0; i < num_points; ++i) {
                y_smooth[i] = y_min + i * y_step;
            }

            vector<Point> smooth_trajectory;

            for (size_t i = 0; i < y_smooth.size(); i += 5) {
                y = y_smooth[i];
                x = quadraticFunc(y, a, b, c);
                xi = cvRound(x);
                yi = cvRound(y);
                if (xi >= 0 && xi < 660 && yi >= 0 && yi < 500) {
                    smooth_trajectory.emplace_back(xi, yi);
                }
            }

            final_trajectory = smooth_trajectory;

        }
        catch (const Exception& e) {
            cerr << "Error in least squares fitting: " << e.what() << endl;
            final_trajectory = trajectory; // If fitting fails, use the original trajectory
        }
    }
    else {
        final_trajectory = trajectory; // If there are not enough points, use the original trajectory
    }
    return final_trajectory;
}

/**
 * @brief Convert the trajectory from the original image coordinate system to the new world coordinate system
 * 
 * @param trajectory 
 * @return vector<Point2f> 
 */
vector<Point2f> TrajectoryPlanNode::transform_to_car_coordinate_system(const vector<Point>& trajectory) {

    int y_original, x_original;
    float new_x, new_y;
    vector<Point2f> transformed_trajectory;

    for (const auto& point : trajectory) {
        // Remove the original image padding and restore to the original image coordinates of 640x480
        y_original = point.y - 10;
        x_original = point.x - 10;

        // Transform coordinates into the new world coordinate system
        new_x = static_cast<float>(480 - y_original) * (1200.0f / 480.0f) + 480;
        new_y = static_cast<float>(x_original - 320) * (1600.0f / 640.0f);

        transformed_trajectory.emplace_back(static_cast<float>(new_x), static_cast<float>(new_y));
    }

    return transformed_trajectory;
}

/**
 * @brief Define a quadratic function for fitting
 * 
 * @param y 
 * @param a 
 * @param b 
 * @param c 
 * @return double 
 */
double TrajectoryPlanNode::quadraticFunc(double y, double a, double b, double c) {
    return a * y * y + b * y + c;
}

/**
 * @brief Plots and saves the trajectory as an image.
 * @param trajectory The trajectory points to be plotted.
 * @param filename The name of the output image file.
 */
void TrajectoryPlanNode::plotAndSaveTrajectory(const vector<Point2f>& transformed_trajectory) {
    // Create a blank image, the size can be adjusted as needed
    int image_width = 1600;  // Corresponds to the x-axis range -800 to 800
    int image_height = 1680; // Corresponds to the y-axis range 0 to 1680
    Mat image = Mat::zeros(image_height, image_width, CV_8UC3);

    // Set background to white
    image.setTo(Scalar(255, 255, 255));

    // Convert coordinates to image pixel coordinates
    auto worldToImage = [image_width, image_height](float x, float y) -> Point {
        int img_x = static_cast<int>((x + 800));
        int img_y = static_cast<int>(1680 - y); // The y-axis is positive downward
                return Point(img_x, img_y);
    };

    // Draw a rectangular box with (125, 258), (-125, 258), (-125, 0), (125, 0) as vertices
    vector<Point2f> rectangle_points = {
            Point2f(125, 258),
            Point2f(-125, 258),
            Point2f(-125, 0),
            Point2f(125, 0),
            Point2f(125, 258) // Closed rectangle
    };

    // Convert rectangle vertices to image coordinates
    vector<Point> rectangle_image_points;
    for (const auto& pt : rectangle_points) {
        rectangle_image_points.push_back(worldToImage(pt.x, pt.y));
    }

    // Draw a blue rectangle
    polylines(image, rectangle_image_points, false, Scalar(255, 0, 0), 2);

    // draw trajectory
    vector<Point> trajectory_image_points;
    for (const auto& pt : transformed_trajectory) {
        trajectory_image_points.push_back(worldToImage(pt.y, pt.x)); // Note the order of the coordinates here
    }

    // Draw red track points and connecting lines, and label the x and y coordinates of each point
    if (!trajectory_image_points.empty()) {
        for (size_t i = 0; i < trajectory_image_points.size(); ++i) {
            circle(image, trajectory_image_points[i], 3, Scalar(0, 0, 255), -1); // red dot
            if (i > 0) {
                line(image, trajectory_image_points[i - 1], trajectory_image_points[i], Scalar(0, 0, 255), 2); // red line
            }
            // Label coordinates
            std::string coordinate_text = "(" + std::to_string(static_cast<int>(transformed_trajectory[i].x)) +
                                          ", " + std::to_string(static_cast<int>(transformed_trajectory[i].y)) + ")";
            putText(image, coordinate_text, trajectory_image_points[i] + Point(5, -5), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 255), 1);
        }
    }

    // Draw the axes
    line(image, worldToImage(-800, 0), worldToImage(800, 0), Scalar(0, 0, 0), 1); // x axis
    line(image, worldToImage(0, 0), worldToImage(0, 1680), Scalar(0, 0, 0), 1); // y axis

    // Add axis labels
    putText(image, "X Axis (mm)", Point(50, image_height - 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 0), 2);
    putText(image, "Y Axis (mm)", Point(image_width - 200, image_height - 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 0), 2);

    // Add a title
    putText(image, "Transformed Trajectory in World Coordinate System", Point(50, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 0), 2);

    // Displaying images
    imshow("Transformed Trajectory", image);
    waitKey(0);
}

/**
 * @brief Finds the index of the closest point in a trajectory to a given x-coordinate.
 * @param trajectory The list of trajectory points.
 * @param target_x The target x-coordinate.
 * @return Index of the closest point or -1 if not found.
 */
int TrajectoryPlanNode::findClosestPointIndex(const vector<Point2f>& trajectory, float target_x) {
    int closest_index = -1;
    float min_distance = numeric_limits<float>::max();
    float distance;

    for (size_t i = 0; i < trajectory.size(); ++i) {
        distance = abs(trajectory[i].x - target_x);
        if (distance < min_distance) {
            min_distance = distance;
            closest_index = i;
        }
    }
    return closest_index;
}