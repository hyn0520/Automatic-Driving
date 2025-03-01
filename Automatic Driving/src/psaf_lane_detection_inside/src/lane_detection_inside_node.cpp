//
// Created by psaf on 30.10.24.
//
#include "psaf_lane_detection_inside/lane_detection_inside_node.hpp"
#include <atomic> // thread-safe counter
std::atomic<int> file_counter(0);

/**
 * @brief Constructs a new LaneDetectionInsideNode object.
 * Initializes subscribers for image, distance, and state information, and a publisher for lane markings.
 */
LaneDetectionInsideNode::LaneDetectionInsideNode() : Node(LANE_DETECTION_INSIDE_NODE) {

    image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            CAM_TOPIC_RGB, 10, std::bind(&LaneDetectionInsideNode::imageCallback, this, std::placeholders::_1));

    distance_subscriber_ = this->create_subscription<sensor_msgs::msg::Range>(
            "/uc_bridge/us_front_center", 10, std::bind(&LaneDetectionInsideNode::distanceCallback, this, std::placeholders::_1));

    state_subscriber_ = this->create_subscription<std_msgs::msg::Int16>(
            "StateInfo", 10, std::bind(&LaneDetectionInsideNode::stateCallback, this, std::placeholders::_1));

    lane_markings_publisher_ = this->create_publisher<psaf_interfaces::msg::LaneMarking>(LANE_MARKINGS_TOPIC, 10);

    lane_marking_.resize(3);
}

/**
 * @brief Callback function for state updates.
 * Updates the current state based on the received message.
 * 
 * @param msg Shared pointer to the state message.
 */
void LaneDetectionInsideNode::stateCallback(std_msgs::msg::Int16::SharedPtr msg){
    current_state_ = msg->data;
}

/**
 * @brief Callback function for distance sensor updates.
 * Checks if the measured distance is below a threshold and sets a flag accordingly.
 * 
 * @param msg Shared pointer to the distance sensor message.
 */
void LaneDetectionInsideNode::distanceCallback(sensor_msgs::msg::Range::SharedPtr msg){
    measured_distance = msg->range;
    if(measured_distance < 1){
        hinder = true;
    }
}

/**
 * @brief Callback function for processing incoming image messages.
 * Converts the ROS image message to OpenCV format, processes it if the state is valid, 
 * and publishes detected lane markings.
 * 
 * @param msg Shared pointer to the received image message.
 */
void LaneDetectionInsideNode::imageCallback(sensor_msgs::msg::Image::SharedPtr msg){
//    RCLCPP_INFO(this->get_logger(), "image with resolution: %d * %d", msg->width, msg->height);

    // transform sensor message to opencv form
    cv::Mat resized_image, warped_image;
    cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
    current_image_ = cv_ptr->image;

    auto start = std::chrono::steady_clock::now();

    RCLCPP_INFO(this->get_logger(), "=====================================================current_state_: %d", current_state_);

    if (current_state_ > 0){
        RCLCPP_WARN(this->get_logger(), "current_state_: %d.", current_state_);
        processImageNormal(current_image_, 10);

        publishLaneMarkings(lane_marking_);
        lane_marking_.at(0).clear();
        lane_marking_.at(1).clear();
        lane_marking_.at(2).clear();
    }

    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

//    RCLCPP_INFO(this->get_logger(), "=====================================================Image Processing time: %ld ms", duration);
}

/**
 * @brief Publishes detected lane markings.
 * Converts lane markings into a message format and publishes it.
 * 
 * @param lane_marking_ The detected lane markings as a vector of point lists.
 */
void LaneDetectionInsideNode::publishLaneMarkings(const std::vector<std::vector<cv::Point>>& lane_marking_){
    psaf_interfaces::msg::LaneMarking LaneMarking_msg;
    for(size_t i = 0; i < lane_marking_.size(); ++i)
    {
        const auto& lane = lane_marking_[i];
        for (const auto& point : lane)
        {
            psaf_interfaces::msg::Point point_msg;
            point_msg.x = static_cast<int>(point.x);
            point_msg.y = static_cast<int>(point.y);

            if (i == 0) {
                LaneMarking_msg.left_lane.push_back(point_msg);
            } else if (i == 1) {
                LaneMarking_msg.center_lane.push_back(point_msg);

            } else if (i == 2) {
                LaneMarking_msg.right_lane.push_back(point_msg);
            }
        }
    }

    lane_markings_publisher_->publish(LaneMarking_msg);
    LaneMarking_msg.left_lane.clear();
    LaneMarking_msg.center_lane.clear();
    LaneMarking_msg.right_lane.clear();
}

/**
 * @brief Processes the input image to detect lane markings.
 * Converts the image to grayscale, applies filtering, edge detection, and extracts lane contours.
 * 
 * @param img The input image in OpenCV format.
 * @param padding Padding applied to the image during processing.
 */
void LaneDetectionInsideNode::processImageNormal(const Mat& img, int padding) {

    RCLCPP_INFO(this->get_logger(), "in");
    auto start = std::chrono::steady_clock::now();

    Mat bgrImg;
    if (img.channels() == 1) {
        cvtColor(img, bgrImg, COLOR_GRAY2BGR);
    } else {
        bgrImg = img;
    }

    Mat gray;
    cvtColor(bgrImg, gray, COLOR_BGR2GRAY);

    Mat blurred;
    GaussianBlur(gray, blurred, Size(9, 9), 0);

    int blockSize = 61;
    int constSubtrahend = -70;

    Mat small_blurred;
    resize(blurred, small_blurred, Size(), 0.25, 0.25);

    Mat small_binary;
    adaptiveThreshold(small_blurred, small_binary, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, blockSize, constSubtrahend);

    Mat binary;
    resize(small_binary, binary, blurred.size());

    Mat homography(3, 3, CV_64F, homography_matrix);
    Mat binary_eagle;
    warpPerspective(binary, binary_eagle, homography, Size(640, 480));

    int original_height = binary.rows;
    int original_width = binary.cols;

    Mat binary_eagle_padded;
    copyMakeBorder(binary_eagle, binary_eagle_padded, padding, padding, padding, padding, BORDER_CONSTANT, Scalar(0));

    Mat blurred_bird;
    GaussianBlur(binary_eagle_padded, blurred_bird, Size(5, 5), 0);
    Mat edges;
    Canny(blurred_bird, edges, 10, 150);

    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    Mat edges_dilated;
    dilate(edges, edges_dilated, kernel, Point(-1, -1), 1);

    kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    Mat edges_closed;
    morphologyEx(edges_dilated, edges_closed, MORPH_CLOSE, kernel);

    vector<vector<Point>> contours;
    findContours(edges_closed, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    cvtColor(binary_eagle_padded, output_img, COLOR_GRAY2BGR);

    vector<Point> fixed_rectangle_1 = { Point(250, 490), Point(250, 300), Point(580, 300), Point(580, 490) };
    Mat fixed_rect_mask_1 = Mat::zeros(binary_eagle_padded.size(), CV_8UC1);
    fillPoly(fixed_rect_mask_1, vector<vector<Point>>{fixed_rectangle_1}, Scalar(255));

    unordered_map<int, int> rightmost_x_by_row;
    vector<vector<Point>> filtered_contours;
    vector<pair<vector<Point>, double>> intersecting_contour_candidate;
    vector<Point> intersecting_contour;

    double area, perimeter, width, height, center_Point, aspect_ratio;
    bool has_intersection_1;
    RotatedRect rect;
    Mat mask, intersection_1;

    for (const auto& contour : contours) {
        area = contourArea(contour);
        perimeter = arcLength(contour, true);
        rect = minAreaRect(contour);
        width = rect.size.width;
        height = rect.size.height;
        center_Point = rect.center.x;

        mask = Mat::zeros(binary_eagle_padded.size(), CV_8UC1);
        drawContours(mask, vector<vector<Point>>{contour}, -1, Scalar(255), FILLED);

        bitwise_and(fixed_rect_mask_1, mask, intersection_1);
        has_intersection_1 = countNonZero(intersection_1) > 0;

        if (has_intersection_1 && area > 1500 && max(height, width) > 150) {
            intersecting_contour_candidate.push_back(make_pair(contour, center_Point));
        }

        if (area > 500 && perimeter / area > 0.001) {
            aspect_ratio = max(height, width) / min(height, width);

            if (min(width, height) > 5 && aspect_ratio > 3) {
                filtered_contours.push_back(contour);
            }
        }
    }

    int x, y; 
    if (!intersecting_contour_candidate.empty()) {

        // Find the contour with the smallest x-coordinate of the center point
        auto min_element_it = min_element(
                intersecting_contour_candidate.begin(),
                intersecting_contour_candidate.end(),
                [](const pair<vector<Point>, double>& a, const pair<vector<Point>, double>& b) {
                    return a.second < b.second;
                }
        );
        intersecting_contour = min_element_it->first;

        
        for (const auto& point : intersecting_contour) {
            x = point.x;
            y = point.y;
            if (rightmost_x_by_row.find(y) == rightmost_x_by_row.end()) {
                rightmost_x_by_row[y] = x;
            } else {
                rightmost_x_by_row[y] = min(rightmost_x_by_row[y], x);
            }
        }

        auto topmost_point = *min_element(intersecting_contour.begin(), intersecting_contour.end(),
                                          [](const Point& a, const Point& b) { return a.y < b.y; });
        int topmost_x = topmost_point.x;
        int topmost_y = topmost_point.y;

        for (int y = 0; y <= topmost_y; ++y) {
            if (rightmost_x_by_row.find(y) == rightmost_x_by_row.end()) {
                rightmost_x_by_row[y] = topmost_x;
            } else {
                rightmost_x_by_row[y] = min(rightmost_x_by_row[y], topmost_x);
            }
        }
    } else {
        intersecting_contour.clear();
    }

    if (!intersecting_contour.empty() &&
        find(filtered_contours.begin(), filtered_contours.end(), intersecting_contour) == filtered_contours.end()) {
        filtered_contours.push_back(intersecting_contour);
    }

    vector<vector<Point>> final_contours;
    vector<Point> y_max_points;
    Mat binary_output_right = Mat::zeros(output_img.size(), CV_8UC1);
    Mat binary_output_center = Mat::zeros(output_img.size(), CV_8UC1);

    bool keep_contour;
    for (const auto& contour : filtered_contours) {
        keep_contour = true;
        if (!intersecting_contour.empty()) {
            for (const auto& point : contour) {
                x = point.x;
                y = point.y;
                if (rightmost_x_by_row.find(y) != rightmost_x_by_row.end() && x > rightmost_x_by_row[y]) {
                    keep_contour = false;
                    break;
                }
            }
        }

        if (keep_contour && contour != intersecting_contour) {
            drawContours(binary_output_center, vector<vector<Point>>{contour}, -1, Scalar(255), FILLED);
            final_contours.push_back(contour);

            auto max_point = *max_element(contour.begin(), contour.end(),
                                          [](const Point& a, const Point& b) { return a.y < b.y; });
            y_max_points.push_back(max_point);
        }
    }

    if (!intersecting_contour.empty()) {
        drawContours(binary_output_right, vector<vector<Point>>{intersecting_contour}, -1, Scalar(255), FILLED);
    }

    vector<cv::Point> center_lane;
    vector<cv::Point> right_lane;
    vector<cv::Point> left_lane;
    std::pair<std::vector<cv::Point>, std::vector<cv::Rect>> center_result;
    std::pair<std::vector<cv::Point>, std::vector<cv::Rect>> right_result;
    std::pair<std::vector<cv::Point>, std::vector<cv::Rect>> left_result;

    if (!intersecting_contour.empty()) {
        auto max_y_point = *max_element(intersecting_contour.begin(), intersecting_contour.end(),
                                        [](const Point& a, const Point& b) { return a.y < b.y; });
        Point right_lane_basepoint = max_y_point;

        right_result = sliding_window_sampling_right_line(binary_output_right, right_lane_basepoint);
        right_lane = right_result.first;

        if (!y_max_points.empty()) {
            Point target_point(280, 470);
            auto center_lane_basepoint = *min_element(y_max_points.begin(), y_max_points.end(),
                                                      [&target_point](const Point& a, const Point& b) {
                                                          return norm(a - target_point) < norm(b - target_point);
                                                      });

            center_result = sliding_window_sampling_center_line(binary_output_center, center_lane_basepoint);
            center_lane = center_result.first;
        }

        else {
            center_lane.clear();
        }
        left_lane.clear();
    }
    else {
        right_lane.clear();

        if (!y_max_points.empty()) {
            Point target_point(280, 470);
            auto center_lane_basepoint = *min_element(y_max_points.begin(), y_max_points.end(),
                                                      [&target_point](const Point& a, const Point& b) {
                                                          return norm(a - target_point) < norm(b - target_point);
                                                      });

            center_result = sliding_window_sampling_center_line(binary_output_center, center_lane_basepoint);
            center_lane = center_result.first;
        }
        else {
            center_lane.clear();
        }
        left_lane.clear();
    }

    lane_marking_[0] = left_lane;
    lane_marking_[1] = center_lane;
    lane_marking_[2] = right_lane;

}

/**
 * @brief Processes the input image to detect lane markings in reverse mode.
 * Converts the image to grayscale, applies filtering, edge detection, and extracts lane contours.
 * Applies a homography transformation to get a bird's-eye view and detects lane boundaries.
 * 
 * @param img The input image in OpenCV format.
 * @param padding Padding applied to the image during processing.
 */
void LaneDetectionInsideNode::processImageReverse(const Mat& img, int padding) {
    auto start = std::chrono::steady_clock::now();

    Mat bgrImg;
    if (img.channels() == 1) {
        cvtColor(img, bgrImg, COLOR_GRAY2BGR);
    } else {
        bgrImg = img;
    }

    Mat gray;
    cvtColor(bgrImg, gray, COLOR_BGR2GRAY);

    Mat blurred;
    GaussianBlur(gray, blurred, Size(9, 9), 0);

    int blockSize = 61;
    int constSubtrahend = -70;

    Mat small_blurred;
    resize(blurred, small_blurred, Size(), 0.25, 0.25);

    Mat small_binary;
    adaptiveThreshold(small_blurred, small_binary, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, blockSize, constSubtrahend);

    Mat binary;
    resize(small_binary, binary, blurred.size());

    Mat homography(3, 3, CV_64F, homography_matrix);
    Mat binary_eagle;
    warpPerspective(binary, binary_eagle, homography, Size(640, 480));

    int original_height = binary.rows;
    int original_width = binary.cols;

    Mat binary_eagle_padded;
    copyMakeBorder(binary_eagle, binary_eagle_padded, padding, padding, padding, padding, BORDER_CONSTANT, Scalar(0));

    Mat blurred_bird;
    GaussianBlur(binary_eagle_padded, blurred_bird, Size(5, 5), 0);
    Mat edges;
    Canny(blurred_bird, edges, 10, 150);

    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    Mat edges_dilated;
    dilate(edges, edges_dilated, kernel, Point(-1, -1), 1);

    kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    Mat edges_closed;
    morphologyEx(edges_dilated, edges_closed, MORPH_CLOSE, kernel);

    vector<vector<Point>> contours;
    findContours(edges_closed, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    cvtColor(binary_eagle_padded, output_img, COLOR_GRAY2BGR);

    vector<Point> fixed_rectangle_1 = {Point(410,490),Point(410,300),Point(80,300),Point(80,490)};
    Mat fixed_rect_mask_1 = Mat::zeros(binary_eagle_padded.size(), CV_8UC1);
    fillPoly(fixed_rect_mask_1, vector<vector<Point>>{fixed_rectangle_1}, Scalar(255));

    unordered_map<int, int> leftmost_x_by_row;
    vector<vector<Point>> filtered_contours;
    vector<pair<vector<Point>, double>> intersecting_contour_candidate;
    vector<Point> intersecting_contour;

    double area, perimeter, width, height, center_Point, aspect_ratio;
    bool has_intersection_1;
    RotatedRect rect;
    Mat mask, intersection_1;


    for (const auto& contour : contours) {
        area = contourArea(contour);
        perimeter = arcLength(contour, true);
        rect = minAreaRect(contour);
        width = rect.size.width;
        height = rect.size.height;
        center_Point = rect.center.x;

        mask = Mat::zeros(binary_eagle_padded.size(), CV_8UC1);
        drawContours(mask, vector<vector<Point>>{contour}, -1, Scalar(255), FILLED);

        bitwise_and(fixed_rect_mask_1, mask, intersection_1);
        has_intersection_1 = countNonZero(intersection_1) > 0;

        if (has_intersection_1 && area > 1500 && max(height, width) > 150) {
            // intersecting_contour_candidate.push_back(make_pair(contour, max(width, height)));
            intersecting_contour_candidate.push_back(make_pair(contour, center_Point));
        }

        if (area > 500 && perimeter / area > 0.001) {
            aspect_ratio = max(height, width) / min(height, width);

            if (min(width, height) > 5 && aspect_ratio > 3) {
                filtered_contours.push_back(contour);
            }
        }
    }

    int x, y;
    if (!intersecting_contour_candidate.empty()) {

        // Find the contour with the smallest x-coordinate of the center point
        auto max_element_it = max_element(
                intersecting_contour_candidate.begin(),
                intersecting_contour_candidate.end(),
                [](const pair<vector<Point>, double>& a, const pair<vector<Point>, double>& b) {
                    return a.second < b.second;
                }
        );
        intersecting_contour= max_element_it->first;

        for (const auto& point : intersecting_contour) {
            x = point.x;
            y = point.y;
            if (leftmost_x_by_row.find(y) == leftmost_x_by_row.end()) {
                leftmost_x_by_row[y] = x;
            } else {
                leftmost_x_by_row[y] = min(leftmost_x_by_row[y], x);
            }
        }

        auto topmost_point = *min_element(intersecting_contour.begin(), intersecting_contour.end(),
                                          [](const Point& a, const Point& b) { return a.y < b.y; });
        int topmost_x = topmost_point.x;
        int topmost_y = topmost_point.y;

        for (int y = 0; y <= topmost_y; ++y) {
            if (leftmost_x_by_row.find(y) == leftmost_x_by_row.end()) {
                leftmost_x_by_row[y] = topmost_x;
            } else {
                leftmost_x_by_row[y] = min(leftmost_x_by_row[y], topmost_x);
            }
        }
    } else {
        intersecting_contour.clear();
    }

    if (!intersecting_contour.empty() &&
        find(filtered_contours.begin(), filtered_contours.end(), intersecting_contour) == filtered_contours.end()) {
        filtered_contours.push_back(intersecting_contour);
    }

    vector<vector<Point>> final_contours;
    vector<Point> y_max_points;
    Mat binary_output_right = Mat::zeros(output_img.size(), CV_8UC1);
    Mat binary_output_center = Mat::zeros(output_img.size(), CV_8UC1);

    bool keep_contour;
    for (const auto& contour : filtered_contours) {
        keep_contour = true;
        if (!intersecting_contour.empty()) {
            for (const auto& point : contour) {
                x = point.x;
                y = point.y;
                if (leftmost_x_by_row.find(y) != leftmost_x_by_row.end() && x < leftmost_x_by_row[y]) {
                    keep_contour = false;
                    break;
                }
            }
        }

        if (keep_contour && contour != intersecting_contour) {
            drawContours(binary_output_right, vector<vector<Point>>{contour}, -1, Scalar(255), FILLED);
            final_contours.push_back(contour);

            auto max_point = *max_element(contour.begin(), contour.end(),
                                          [](const Point& a, const Point& b) { return a.y < b.y; });
            y_max_points.push_back(max_point);
        }
    }

    if (!intersecting_contour.empty()) {
        drawContours(binary_output_center, vector<vector<Point>>{intersecting_contour}, -1, Scalar(255), FILLED);
    }

    vector<cv::Point> center_lane;
    vector<cv::Point> right_lane;
    vector<cv::Point> left_lane;
    std::pair<std::vector<cv::Point>, std::vector<cv::Rect>> center_result;
    std::pair<std::vector<cv::Point>, std::vector<cv::Rect>> right_result;
    std::pair<std::vector<cv::Point>, std::vector<cv::Rect>> left_result;

    if (!intersecting_contour.empty()) {
        auto max_y_point = *max_element(intersecting_contour.begin(), intersecting_contour.end(),
                                        [](const Point& a, const Point& b) { return a.y < b.y; });
        Point center_lane_basepoint = max_y_point;

        center_result = sliding_window_sampling_right_line(binary_output_center, center_lane_basepoint);
        center_lane = center_result.first;

        if (!y_max_points.empty()) {
            Point target_point(380,470);
            auto right_lane_basepoint = *min_element(y_max_points.begin(), y_max_points.end(),
                                                     [&target_point](const Point& a, const Point& b) {
                                                         return norm(a - target_point) < norm(b - target_point);
                                                     });

            right_result = sliding_window_sampling_center_line(binary_output_right, right_lane_basepoint);
            right_lane = right_result.first;
        }

        else {
            right_lane.clear();
        }
        left_lane.clear();
    }
    else {
        center_lane.clear();

        if (!y_max_points.empty()) {
            Point target_point(380,470);
            auto right_lane_basepoint = *min_element(y_max_points.begin(), y_max_points.end(),
                                                     [&target_point](const Point& a, const Point& b) {
                                                         return norm(a - target_point) < norm(b - target_point);
                                                     });

            right_result = sliding_window_sampling_center_line(binary_output_right, right_lane_basepoint);
            right_lane = right_result.first;
        }
        else {
            right_lane.clear();
        }

        left_lane.clear();
    }

    lane_marking_[0] = left_lane;
    lane_marking_[1] = center_lane;
    lane_marking_[2] = right_lane;
}

/**
 * @brief Performs sliding window sampling to detect the right lane markings.
 * 
 * This function uses a sliding window approach to track and extract right lane markings
 * from a given image. It starts from a base point and iteratively moves upward while 
 * searching for lane pixels within a defined window.
 * 
 * @param image The input image in OpenCV format, typically a binary or edge-detected image.
 * @param base_point The initial point from which lane detection starts.
 * @param window_width The width of the sliding window used to search for lane pixels.
 * @param window_height The height of the sliding window used to search for lane pixels.
 * @return A pair consisting of a vector of detected lane points and a vector of bounding rectangles representing the sampled windows.
 */
std::pair<std::vector<cv::Point>, std::vector<cv::Rect>> LaneDetectionInsideNode::sliding_window_sampling_right_line(const cv::Mat& image, cv::Point base_point, int window_width, int window_height) {
    std::vector<cv::Point> right_lane;
    cv::Point current_center = base_point;
    int img_height = image.rows;
    int img_width = image.cols;
    std::vector<cv::Rect> rectangles;

    int cx, cy, x_start, x_end, y_start, y_end, middle_row;
    cv::Mat row_pixels;
    bool found;
    while (true) {
        right_lane.push_back(current_center);
        cx = current_center.x;
        cy = current_center.y;

        x_start = std::max(0, cx - window_width / 2);
        x_end = std::min(img_width, cx + window_width / 2);
        y_start = std::max(0, cy - window_height / 2);
        y_end = std::min(img_height, y_start - window_height);

        if (y_end < 0
            || y_start > image.rows
            || x_start < 0
            || x_end > image.cols
            || current_center.x < 0
            || current_center.x > image.cols
            || current_center.y < 0
            || current_center.y > image.rows) {
            break;
        }

        middle_row = (y_start + y_end) / 2;
        row_pixels = image.row(middle_row).colRange(x_start, x_end);
        found = false;
        for (int i = 0; i < row_pixels.cols; ++i) {
            if (row_pixels.at<uchar>(0, i) > 0) {
                current_center = cv::Point(x_start + i, middle_row);
                found = true;
                break;
            }
        }
        if (!found) {
            break;
        }

        x_start = std::max(0, current_center.x - window_width / 2);
        y_start = std::max(0, current_center.y - window_height / 2);

        if (y_end < 0
            || y_start > image.rows
            || x_start < 0
            || x_end > image.cols
            || current_center.x < 0
            || current_center.x > image.cols
            || current_center.y < 0
            || current_center.y > image.rows) {
            break;
        }
        rectangles.push_back(cv::Rect(x_start, y_start, window_width, window_height));
    }
    return std::make_pair(right_lane, rectangles);
}

/**
 * @brief Performs sliding window sampling to detect the center lane markings.
 * 
 * This function follows a similar approach as right lane detection but specifically 
 * focuses on tracking the center lane. If lane pixels are not found, it attempts 
 * to estimate the next search position using prior detected points.
 * 
 * @param image The input image in OpenCV format, typically a binary or edge-detected image.
 * @param base_point The initial point from which center lane detection starts.
 * @param window_width The width of the sliding window used to search for lane pixels.
 * @param window_height The height of the sliding window used to search for lane pixels.
 * @return A pair consisting of a vector of detected center lane points and a vector of bounding rectangles representing the sampled windows.
 */
std::pair<std::vector<cv::Point>, std::vector<cv::Rect>> LaneDetectionInsideNode::sliding_window_sampling_center_line( const cv::Mat& image, cv::Point base_point, int window_width, int window_height) {
    std::vector<cv::Point> center_line_points;
    cv::Point current_center = base_point;
    int img_height = image.rows;
    int img_width = image.cols;
    std::vector<cv::Rect> rectangles;
    double distance;

    int cx, cy, x_start, x_end, y_start, y_end, middle_row, retry;
    double dx, dy, magnitude;
    cv::Mat row_pixels;
    bool found;

    while (true) {
        center_line_points.push_back(current_center);
        cx = current_center.x;
        cy = current_center.y;

        x_start = std::max(0, cx - window_width / 2);
        x_end = std::min(img_width, cx + window_width / 2);
        y_start = std::max(0, cy - window_height / 2);
        y_end = std::min(img_height, y_start - window_height);

        if (y_end < 0 
            || y_start > image.rows 
            || x_start < 0  
            || x_end > image.cols 
            || current_center.x < 0 
            || current_center.x > image.cols 
            || current_center.y < 0 
            || current_center.y > image.rows) {
            break;
        }

        middle_row = (y_start + y_end) / 2;
        row_pixels = image.row(middle_row).colRange(x_start, x_end);

        found = false;

        for (int i = row_pixels.cols - 1; i >= 0; --i) {
            if (row_pixels.at<uchar>(0, i) > 0) {
                //RCLCPP_INFO(this->get_logger(), "i: %d", i);
                current_center = cv::Point(x_start + i, middle_row);
                //RCLCPP_INFO(this->get_logger(), "new_center: (%d, %d)", new_center.x, new_center.y);
                found = true;
                break;
            }
        }
        if (!found) {
            retry = 10;
            dx = 0;
            dy = -1;
            if (center_line_points.size() >= 2) {
                cv::Point first_point = center_line_points[0];
                cv::Point last_point = center_line_points.back();

                dx = static_cast<double>(last_point.x - first_point.x);
                dy = static_cast<double>(last_point.y - first_point.y);
                magnitude = std::max(1e-6, std::sqrt(dx * dx + dy * dy));
                dx /= magnitude;
                dy /= magnitude;

                if (dy == 0) {
                    dy = -1e-6;
                }

                if (std::abs(-dx * window_height / dy) > 1) {
                    distance = -dx * window_height / dy;
                }
                else {
                    if (std::abs(-dx * window_height / dy) == 0) {
                        distance == 1e-6;
                    } else {
                        distance = std::abs(-dx * window_height / dy) / (-dx * window_height / dy);
                    }
                }
            }
            for (int attempt = 0; attempt < retry; ++attempt) {
                current_center.x += static_cast<int>(distance);
                x_start = std::max(0, current_center.x - window_width / 2);
                x_end = std::min(img_width, current_center.x + window_width / 2);
                y_start = std::max(0, current_center.y - window_height / 2);
                y_end = std::min(img_height, y_start - window_height);

                if (y_end < 0
                    || y_start > image.rows
                    || x_start < 0
                    || x_end > image.cols
                    || current_center.x < 0
                    || current_center.x > image.cols
                    || current_center.y < 0
                    || current_center.y > image.rows) {
                    break;
                }

                middle_row = (y_start + y_end) / 2;
                row_pixels = image.row(middle_row).colRange(x_start, x_end);
                current_center.x = (x_start + x_end) / 2;
                current_center.y = (y_start + y_end) / 2;

                if (y_end < 0
                    || y_start > image.rows
                    || x_start < 0
                    || x_end > image.cols
                    || current_center.x < 0
                    || current_center.x > image.cols
                    || current_center.y < 0
                    || current_center.y > image.rows) {
                    break;
                }

                for (int i = row_pixels.cols - 1; i >= 0; --i) {
                    if (row_pixels.at<uchar>(0, i) > 0) {
                        current_center = cv::Point(x_start + i, middle_row);
                        found = true;
                        break;
                    }
                }

                if (found) {
                    break;
                }
            }

            if (!found) {
                break;
            }
        }

        x_start = std::max(0, current_center.x - window_width / 2);
        y_start = std::max(0, current_center.y - window_height / 2);

        if (y_end < 0
            || y_start > image.rows
            || x_start < 0
            || x_end > image.cols
            || current_center.x < 0
            || current_center.x > image.cols
            || current_center.y < 0
            || current_center.y > image.rows) {
            break;
        }
        rectangles.push_back(cv::Rect(x_start, y_start, window_width, window_height));

    }
    return std::make_pair(center_line_points, rectangles);
}