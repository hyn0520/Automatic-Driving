//
// Created by psaf on 30.10.24.
//
#include "psaf_lane_detection_outside/lane_detection_outside_node.hpp"
#include <atomic> // 用于线程安全的计数器
std::atomic<int> file_counter(0);

LaneDetectionOutsideNode::LaneDetectionOutsideNode() : Node(LANE_DETECTION_OUTSIDE_NODE) {

    image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            CAM_TOPIC_RGB, 10, std::bind(&LaneDetectionOutsideNode::imageCallback, this, std::placeholders::_1));

    lane_markings_publisher_ = this->create_publisher<psaf_interfaces::msg::LaneMarking>(LANE_MARKINGS_TOPIC, 10);

    lane_marking_.resize(3);
}


//call back function:+
void LaneDetectionOutsideNode::imageCallback(sensor_msgs::msg::Image::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "image with resolution: %d * %d", msg->width, msg->height);

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
//    std::string filename = "/home/psaf/Pictures/dataset2/image_1981_20241111_213828.png";
    //std::string filename = "/home/psaf/Pictures/dataset2/image_640_20241111_213743.png";
    //cv::Mat a = cv::imread(filename);

    auto start = std::chrono::steady_clock::now();
    processImageOutercircle(current_image_, 10);
    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    RCLCPP_INFO(this->get_logger(), "=====================================================Image Processing time: %ld ms", duration);

    publishLaneMarkings(lane_marking_);
    lane_marking_.at(0).clear();
    lane_marking_.at(1).clear();
    lane_marking_.at(2).clear();
}




//publish function:
void LaneDetectionOutsideNode::publishLaneMarkings(const std::vector<std::vector<cv::Point>>& lane_marking_){
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
//                const auto& last_point = LaneMarking_msg.left_lane.back();
//                RCLCPP_INFO(this->get_logger(), "points: %d %d", last_point.x, last_point.y);
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

void LaneDetectionOutsideNode::processImageOutercircle(const Mat& img, int padding) {
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

    for (const auto& contour : contours) {
        double area = contourArea(contour);
        double perimeter = arcLength(contour, true);
        RotatedRect rect = minAreaRect(contour);
        double width = rect.size.width;
        double height = rect.size.height;
        double center_Point = rect.center.x;

        Mat mask = Mat::zeros(binary_eagle_padded.size(), CV_8UC1);
        drawContours(mask, vector<vector<Point>>{contour}, -1, Scalar(255), FILLED);

        Mat intersection_1;
        bitwise_and(fixed_rect_mask_1, mask, intersection_1);
        bool has_intersection_1 = countNonZero(intersection_1) > 0;

        if (has_intersection_1 && area > 1500 && max(height, width) > 150) {
            // intersecting_contour_candidate.push_back(make_pair(contour, max(width, height)));
            intersecting_contour_candidate.push_back(make_pair(contour, center_Point));
        }

        if (area > 500 && perimeter / area > 0.001) {
            double aspect_ratio = max(height, width) / min(height, width);

            if (min(width, height) > 5 && aspect_ratio > 3) {
                filtered_contours.push_back(contour);
            }
        }
    }

    if (!intersecting_contour_candidate.empty()) {


//         auto max_element_it = max_element(
//             intersecting_contour_candidate.begin(),
//             intersecting_contour_candidate.end(),
//             [](const pair<vector<Point>, double>& a, const pair<vector<Point>, double>& b) {
//                 return a.second < b.second;
//             });
//         intersecting_contour = max_element_it->first;


        // 找到中心点 x 坐标最小的轮廓
        auto min_element_it = min_element(
                intersecting_contour_candidate.begin(),
                intersecting_contour_candidate.end(),
                [](const pair<vector<Point>, double>& a, const pair<vector<Point>, double>& b) {
                    return a.second < b.second;
                }
        );
        intersecting_contour= min_element_it->first;

        for (const auto& point : intersecting_contour) {
            int x = point.x;
            int y = point.y;
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

    for (const auto& contour : filtered_contours) {
        bool keep_contour = true;
        if (!intersecting_contour.empty()) {
            for (const auto& point : contour) {
                int x = point.x;
                int y = point.y;
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

//        std::ostringstream oss;
//        for (const auto& point : y_max_points) {
//            oss << point << " ";
//        }
//        RCLCPP_INFO(this->get_logger(), "y_max_points: %s", oss.str().c_str());

        right_result = sliding_window_sampling_right_line(binary_output_right, right_lane_basepoint);
        right_lane = right_result.first;



//        if (!y_max_points.empty()) {
//
//
//            Point target_point(330, 300);
//            auto center_lane_basepoint = *min_element(y_max_points.begin(), y_max_points.end(),
//                                                      [&target_point](const Point& a, const Point& b) {
//                                                          return norm(a - target_point) < norm(b - target_point);
//                                                      });
//
//            center_result = sliding_window_sampling_center_line(binary_output_center, center_lane_basepoint);
//            center_lane = center_result.first;
//        }
        if (!y_max_points.empty()) {
            Point center_lane_basepoint;
            std::vector<Point> y_max_points_sorted = y_max_points;
            std::sort(y_max_points_sorted.begin(), y_max_points_sorted.end(), [](const Point& a, const Point& b) {
                return a.y > b.y;
            });

            // 如果只有一个点
            if (y_max_points_sorted.size() == 1) {
                center_lane_basepoint = y_max_points_sorted[0];
            } else {
                // 找到 y 最大的两个点
                std::vector<Point> top_two_y_points = {y_max_points_sorted[0], y_max_points_sorted[1]};

                // 在这两个点中找到 x 最大的点
                center_lane_basepoint = *std::max_element(top_two_y_points.begin(), top_two_y_points.end(), [](const Point& a, const Point& b) {
                    return a.x < b.x;
                });
            }

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
            Point center_lane_basepoint;
            std::vector<Point> y_max_points_sorted = y_max_points;
            std::sort(y_max_points_sorted.begin(), y_max_points_sorted.end(), [](const Point& a, const Point& b) {
                return a.y > b.y;
            });

            if (y_max_points_sorted.size() == 1) {
                center_lane_basepoint = y_max_points_sorted[0];
            } else {
                std::vector<Point> top_two_y_points = {y_max_points_sorted[0], y_max_points_sorted[1]};

                center_lane_basepoint = *std::max_element(top_two_y_points.begin(), top_two_y_points.end(), [](const Point& a, const Point& b) {
                    return a.x < b.x;
                });
            }

            center_result = sliding_window_sampling_center_line(binary_output_center, center_lane_basepoint);
            center_lane = center_result.first;
        }
        else {
            center_lane.clear();
        }

        left_lane.clear();
    }

//    if (!intersecting_contour.empty()) {
//        final_contours.push_back(intersecting_contour);
//    }
//
//    for (const auto& contour : final_contours) {
//        drawContours(output_img, vector<vector<Point>>{contour}, -1, Scalar(255, 255, 0), 2);
//
//        RotatedRect rect = minAreaRect(contour);
//        Point2f box[4];
//        rect.points(box);
//        for (int i = 0; i < 4; ++i) {
//            line(output_img, box[i], box[(i + 1) % 4], Scalar(0, 0, 255), 2);
//        }
//    }
//
//    rectangle(output_img, Point(250, 300), Point(580, 490), Scalar(0, 255, 255), 2);
//
//    if (!right_lane.empty()) {
//        for (const auto& rect : right_result.second) {
//            int x_start = rect.x;
//            int y_start = rect.y;
//            int x_end = rect.x + rect.width;
//            int y_end = rect.y + rect.height;
//            rectangle(output_img, Point(x_start, y_start), Point(x_end, y_end), Scalar(0, 255, 0), 2);
//        }
//        for (const auto& point : right_lane) {
//            circle(output_img, point, 5, Scalar(0, 0, 255), FILLED);
//        }
//    }
//
//    if (!center_lane.empty()) {
//        for (const auto& rect : center_result.second) {
//            int x_start = rect.x;
//            int y_start = rect.y;
//            int x_end = rect.x + rect.width;
//            int y_end = rect.y + rect.height;
//            rectangle(output_img, Point(x_start, y_start), Point(x_end, y_end), Scalar(0, 255, 0), 2);
//        }
//        for (const auto& point : center_lane) {
//            circle(output_img, point, 5, Scalar(255, 0, 0), FILLED);
//        }
//    }

    lane_marking_[0] = left_lane;
    lane_marking_[1] = center_lane;
    lane_marking_[2] = right_lane;

    //std::cerr << "==================================0Input image is empty or not the correct size." << std::endl;

//     RCLCPP_INFO(this->get_logger(), "Right lane points:");
//        for (const auto& point :  lane_marking_[2]) {
//            RCLCPP_INFO(this->get_logger(), "Point: x: %d, y: %d", point.x, point.y);
//        }


//     显示结果（用于可视化）
    //imshow("Output Image", output_img);  // 显示结果图像
    //waitKey(0);  // 等待按键
    //int current_counter = file_counter.fetch_add(1);
//
    // 生成文件名
//    std::stringstream filename;
//    filename << "/home/psaf/Pictures/lane_detection/image_trans_" << current_counter << ".png";
//    cv::imwrite(filename.str(), output_img);

//    auto end = std::chrono::steady_clock::now();
//    auto duration = chrono::duration_cast<chrono::milliseconds>(end - start).count();

}


std::pair<std::vector<cv::Point>, std::vector<cv::Rect>> LaneDetectionOutsideNode::sliding_window_sampling_right_line(
        const cv::Mat& image, cv::Point base_point, int window_width, int window_height) {
    std::vector<cv::Point> right_lane;
    cv::Point current_center = base_point;
    int img_height = image.rows;
    int img_width = image.cols;
    std::vector<cv::Rect> rectangles;

    while (true) {
        right_lane.push_back(current_center);
        int cx = current_center.x;
        int cy = current_center.y;

        int x_start = std::max(0, cx - window_width / 2);
        int x_end = std::min(img_width, cx + window_width / 2);
        int y_start = std::max(0, cy - window_height / 2);
        int y_end = std::min(img_height, y_start - window_height);

        RCLCPP_INFO(this->get_logger(), "old center: 1");

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

        int middle_row = (y_start + y_end) / 2;
        cv::Mat row_pixels = image.row(middle_row).colRange(x_start, x_end);
        bool found = false;
        for (int i = 0; i < row_pixels.cols; ++i) {
            if (row_pixels.at<uchar>(0, i) > 0) {
                current_center = cv::Point(x_start + i, middle_row);
                found = true;
                break;
            }
        }
        //RCLCPP_INFO(this->get_logger(), "Found: %s", found ? "true" : "false");
        if (!found) {
            break;
        }

        x_start = std::max(0, current_center.x - window_width / 2);
        y_start = std::max(0, current_center.y - window_height / 2);

        RCLCPP_INFO(this->get_logger(), "old center: 2");

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




std::pair<std::vector<cv::Point>, std::vector<cv::Rect>> LaneDetectionOutsideNode::sliding_window_sampling_center_line(
        const cv::Mat& image, cv::Point base_point, int window_width, int window_height) {
    std::vector<cv::Point> center_line_points;
    cv::Point current_center = base_point;
    int img_height = image.rows;
    int img_width = image.cols;
    std::vector<cv::Rect> rectangles;
    double distance;

    while (true) {
        center_line_points.push_back(current_center);
        int cx = current_center.x;
        int cy = current_center.y;

        int x_start = std::max(0, cx - window_width / 2);
        int x_end = std::min(img_width, cx + window_width / 2);
        int y_start = std::max(0, cy - window_height / 2);
        int y_end = std::min(img_height, y_start - window_height);
        //int y_end = std::min(img_height, cy + window_height / 2);

        RCLCPP_INFO(this->get_logger(), "old center: 3");

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

//        if (y_start >= y_end || x_start >= x_end)
//            break;

        int middle_row = (y_start + y_end) / 2;
        cv::Mat row_pixels = image.row(middle_row).colRange(x_start, x_end);


        bool found = false;

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
            int retry = 10;
            double dx = 0, dy = -1;
            if (center_line_points.size() >= 2) {
                cv::Point first_point = center_line_points[0];
                cv::Point last_point = center_line_points.back();

                dx = static_cast<double>(last_point.x - first_point.x);
                dy = static_cast<double>(last_point.y - first_point.y);
                double magnitude = std::max(1e-6, std::sqrt(dx * dx + dy * dy));
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
//            double distance = -dx * window_height / dy;
//            if (std::abs(distance) < 1.0) {
//                distance = (distance >= 0) ? 1.0 : -1.0;
//            }
            }
            //bool retry_found = false;
            for (int attempt = 0; attempt < retry; ++attempt) {
                current_center.x += static_cast<int>(distance);
                x_start = std::max(0, current_center.x - window_width / 2);
                x_end = std::min(img_width, current_center.x + window_width / 2);
                y_start = std::max(0, current_center.y - window_height / 2);
                y_end = std::min(img_height, y_start - window_height);
                //y_end = std::min(img_height, cy + window_height / 2);

                RCLCPP_INFO(this->get_logger(), "old center: 4");

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

//                if (y_start >= y_end || x_start >= x_end) {
//                    break;
//                }

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
//                        cx = x_start + i;
//                        cy = middle_row;
                        found = true;
                        //retry_found = true;
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

        RCLCPP_INFO(this->get_logger(), "old center: 5");

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

        //RCLCPP_INFO(this->get_logger(), "old center: (%d, %d)", current_center.x, current_center.y);
        //current_center = new_center;
        //RCLCPP_INFO(this->get_logger(), "Updated center: (%d, %d)", new_center.x, new_center.y);

    }

    return std::make_pair(center_line_points, rectangles);
}
