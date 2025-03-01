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
    //RCLCPP_INFO(this->get_logger(), "image with resolution: %d * %d", msg->width, msg->height);

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
    //std::string filename = "/home/psaf/Pictures/dataset2/image_330_20241111_213733.png";
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
    Mat bgrImg;
    if (img.channels() == 1) {
        cvtColor(img, bgrImg, COLOR_GRAY2BGR);  // 如果是灰度图像，转换为BGR格式
    } else {
        bgrImg = img;  // 否则直接赋值
    }

    Mat gray;
    
    cvtColor(bgrImg, gray, COLOR_BGR2GRAY);  // 转换为灰度图像
    Mat blurred;
    GaussianBlur(gray, blurred, Size(9, 9), 0);  // 对灰度图像进行高斯模糊

    Mat binary;
    adaptiveThreshold(blurred, binary, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 301, -70);  // 自适应阈值处理

    // 透视变换
    cv::Mat homography = cv::Mat(3, 3, CV_64F, homography_matrix).clone();
    Mat binary_eagle;
    auto start = std::chrono::steady_clock::now();

    warpPerspective(binary, binary_eagle, homography, Size(640, 480));
    int original_height = binary.rows;
    int original_width = binary.cols;

    // 添加黑色边框以避免边缘效应
    Mat binary_eagle_padded;
    copyMakeBorder(binary_eagle, binary_eagle_padded, padding, padding, padding, padding, BORDER_CONSTANT, Scalar(0));

    // 图像预处理和边缘检测
    Mat blurred_bird;
    GaussianBlur(binary_eagle_padded, blurred_bird, Size(5, 5), 0);
    Mat edges;
    Canny(blurred_bird, edges, 10, 150);

    // 膨胀和形态学闭运算
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    Mat edges_dilated;
    dilate(edges, edges_dilated, kernel, Point(-1, -1), 1);

    kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    Mat edges_closed;
    morphologyEx(edges_dilated, edges_closed, MORPH_CLOSE, kernel);

    // 轮廓检测
    vector<vector<Point>> contours;
    findContours(edges_closed, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    cvtColor(binary_eagle_padded, output_img, COLOR_GRAY2BGR);

    // 定义固定矩形区域
    vector<Point> fixed_rectangle_1 = { Point(320, 490), Point(320, 300), Point(580, 300), Point(580, 490) };
    Mat fixed_rect_mask_1 = Mat::zeros(binary_eagle_padded.size(), CV_8UC1);
    fillPoly(fixed_rect_mask_1, vector<vector<Point>>{fixed_rectangle_1}, Scalar(255));

    unordered_map<int, int> rightmost_x_by_row;
    vector<vector<Point>> filtered_contours;
    vector<pair<vector<Point>, double>> intersecting_contour_candidate;
    vector<Point> intersecting_contour;

    // 处理轮廓并应用过滤条件
    for (const auto& contour : contours) {
        double area = contourArea(contour);
        double perimeter = arcLength(contour, true);
        RotatedRect rect = minAreaRect(contour);
        double width = rect.size.width;
        double height = rect.size.height;
        double aspect_ratio = max(height, width) / min(height, width);
        Mat mask = Mat::zeros(binary_eagle_padded.size(), CV_8UC1);
        drawContours(mask, vector<vector<Point>>{contour}, -1, Scalar(255), FILLED);
        Mat intersection_1;
        bitwise_and(fixed_rect_mask_1, mask, intersection_1);
        bool has_intersection_1 = countNonZero(intersection_1) > 0;

        if (has_intersection_1 && area > 3000) {
            intersecting_contour_candidate.push_back(make_pair(contour, max(width, height)));
        }

        if (area > 600 && perimeter / area > 0.001 && min(width, height) > 5 && aspect_ratio > 3) {
            filtered_contours.push_back(contour);
        }
    }

    // 保留相交的轮廓
    if (!intersecting_contour_candidate.empty()) {
        auto max_element_it = max_element(intersecting_contour_candidate.begin(), intersecting_contour_candidate.end(),
                                          [](const pair<vector<Point>, double>& a, const pair<vector<Point>, double>& b) {
                                              return a.second < b.second;
                                          });
        intersecting_contour = max_element_it->first;

        // 找到每一行的最左侧的点，更新 rightmost_x_by_row
        for (const auto& point : intersecting_contour) {
            int x = point.x;
            int y = point.y;
            if (rightmost_x_by_row.find(y) == rightmost_x_by_row.end()) {
                rightmost_x_by_row[y] = x;
            }
            else {
                rightmost_x_by_row[y] = min(rightmost_x_by_row[y], x);
            }
        }

        // 找到轮廓最上面的点
        auto topmost_point = *min_element(intersecting_contour.begin(), intersecting_contour.end(),
                                          [](const Point& a, const Point& b) { return a.y < b.y; });
        int topmost_x = topmost_point.x;
        int topmost_y = topmost_point.y;

        // 将这条直线加入 rightmost_x_by_row 作为分界线
        for (int y = 0; y <= topmost_y; ++y) {
            if (rightmost_x_by_row.find(y) == rightmost_x_by_row.end()) {
                rightmost_x_by_row[y] = topmost_x;
            }
            else {
                rightmost_x_by_row[y] = min(rightmost_x_by_row[y], topmost_x);
            }
        }
    }
    else {
        intersecting_contour.clear();
    }

    if (!intersecting_contour.empty() &&
        find(filtered_contours.begin(), filtered_contours.end(), intersecting_contour) == filtered_contours.end()) {
        filtered_contours.push_back(intersecting_contour);
    }

    vector<vector<Point>> final_contours;
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
        if (keep_contour || contour == intersecting_contour) {
            final_contours.push_back(contour);
        }
    }

    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    RCLCPP_INFO(this->get_logger(), "=====================================================Find contour Processing time: %ld ms", duration);


    // 生成和聚类骨架线
    vector<Mat> final_skeleton = generateSkeletons(final_contours, binary_eagle_padded.size());
    final_skeleton = clusterSkeletons(final_skeleton, 120);
    
    int max_avg_x_idx = findSkeletonWithMaxAvgX(final_skeleton);

    vector<vector<Point>> two_lanes;

    std::vector<cv::Point> right_lane;
    std::vector<cv::Point> center_lane;
    std::vector<cv::Point> left_lane;
    for (size_t idx = 0; idx < final_skeleton.size(); ++idx) {
        Mat skeleton = final_skeleton[idx];
        vector<Point> skeleton_points;
        findNonZero(skeleton, skeleton_points);
        Mat new_skeleton = Mat::zeros(binary_eagle_padded.size(), CV_8UC1);

        // 检查骨架线是否与固定矩形区域相交
        Mat intersection;
        bitwise_and(skeleton, fixed_rect_mask_1, intersection);
        bool has_intersection_1 = countNonZero(intersection) > 0;

        unordered_map<int, int> current_points_by_row;
        for (const auto& point : skeleton_points) {
            int x = point.x;
            int y = point.y;
            if (current_points_by_row.find(y) == current_points_by_row.end()) {
                current_points_by_row[y] = x;
            }
            else {
                if (idx == max_avg_x_idx && has_intersection_1) {
                    current_points_by_row[y] = min(current_points_by_row[y], x);
                }
                else {
                    current_points_by_row[y] = max(current_points_by_row[y], x);
                }
            }
        }

        for (const auto& [y, x] : current_points_by_row) {
            new_skeleton.at<uchar>(y, x) = 255;
        }

        if (idx == max_avg_x_idx && has_intersection_1) {
            right_lane = samplingPoints(new_skeleton);
        }
        else {
            two_lanes.push_back(samplingPoints(new_skeleton));
        }
    }

    vector<pair<double, vector<Point>>> lane_avg_values;
    for (const auto& lane : two_lanes) {
        if (!lane.empty()) {
            double avg_x = accumulate(lane.begin(), lane.end(), 0.0,
                                      [](double sum, const Point& p) { return sum + p.x; }) / lane.size();
            lane_avg_values.push_back(make_pair(avg_x, lane));
        }
    }

    sort(lane_avg_values.begin(), lane_avg_values.end(),
         [](const pair<double, vector<Point>>& a, const pair<double, vector<Point>>& b) {
             return a.first > b.first;
         });

    if (lane_avg_values.size() >= 2) {
        center_lane = lane_avg_values[0].second;
        left_lane = lane_avg_values[1].second;
    }
    else if (lane_avg_values.size() == 1) {
        center_lane = lane_avg_values[0].second;
    }

    if (final_skeleton.size() > 3 || final_skeleton.empty()) {
        left_lane.clear();
        center_lane.clear();
    }

    lane_marking_[0] = left_lane;
    lane_marking_[1] = center_lane;
    lane_marking_[2] = right_lane;

//    RCLCPP_INFO(this->get_logger(), "Right lane points:");
//    for (const auto& point :  lane_marking_[2]) {
//        RCLCPP_INFO(this->get_logger(), "Point: x: %d, y: %d", point.x, point.y);
//    }


    // 绘制最终轮廓和采样点
    Scalar color_list[3] = { Scalar(0, 0, 255), Scalar(255, 0, 0), Scalar(128, 0, 128) };

    for (const auto& contour : final_contours) {
        drawContours(output_img, vector<vector<Point>>{contour}, -1, Scalar(255, 255, 0), 2);
        RotatedRect rect = minAreaRect(contour);
        Point2f box_points[4];
        rect.points(box_points);
        vector<Point> box;
        for (int i = 0; i < 4; ++i) {
            box.push_back(box_points[i]);
        }
        drawContours(output_img, vector<vector<Point>>{box}, 0, Scalar(0, 0, 255), 2);
    }

    drawSamplingPoints(output_img, right_lane, color_list[0]);
    drawSamplingPoints(output_img, center_lane, color_list[1]);
    drawSamplingPoints(output_img, left_lane, color_list[2]);

    // 绘制固定位置的黄色矩形框
    rectangle(output_img, Point(320, 300), Point(580, 490), Scalar(0, 255, 255), 2);

    // 显示结果（用于可视化）
    imshow("Output Image", output_img);  // 显示结果图像
//    //waitKey(0);  // 等待按键
//    int current_counter = file_counter.fetch_add(1);
//
//    // 生成文件名
//    std::stringstream filename;
//    filename << "/home/psaf/Pictures/lane_detection/image_trans_" << current_counter << ".png";
//    cv::imwrite(filename.str(), output_img);
}


// draw sample points from detected lane
void LaneDetectionOutsideNode::drawSamplingPoints(Mat& img, const vector<Point>& sampledPoints, const Scalar& color) {
    for (const auto& point : sampledPoints) {
        circle(img, Point(point.x, point.y), 6, color, -1);  // 在每个采样点处绘制一个圆点
    }
}

// sample points from skeleton image
vector<Point> LaneDetectionOutsideNode::samplingPoints(const Mat& skeleton) {
    vector<Point> points;
    findNonZero(skeleton, points);  // 查找骨架中所有非零点（即骨架点）
    sort(points.begin(), points.end(), [](const Point& a, const Point& b) { return a.y > b.y; }); // Sort by y coordinate from largest to smallest
    vector<Point> sampledPoints;
    for (size_t i = 0; i < points.size(); i += 15) {
        sampledPoints.push_back(points[i]);  // 每隔15个点采样一次
    }
    return sampledPoints;
}

// 找到具有最大平均x坐标的骨架的索引
int LaneDetectionOutsideNode::findSkeletonWithMaxAvgX(const vector<Mat>& skeletons) {
    double maxAvgX = -numeric_limits<double>::infinity();  // 初始化最大平均x坐标为负无穷大
    int maxIdx = -1;  // 初始化最大索引为-1
    for (size_t idx = 0; idx < skeletons.size(); ++idx) {
        vector<Point> points;
        findNonZero(skeletons[idx], points);  // 获取骨架中的所有点
        if (!points.empty()) {
            double avgX = 0;
            for (const auto& point : points) {
                avgX += point.x;  // 累加所有点的x坐标
            }
            avgX /= points.size();  // 计算平均x坐标
            if (avgX > maxAvgX) {  // 如果当前骨架的平均x坐标大于最大值
                maxAvgX = avgX;
                maxIdx = idx;  // 更新最大索引
            }
        }
    }
    return maxIdx;
}

// 对骨架进行聚类
vector<Mat> LaneDetectionOutsideNode::clusterSkeletons(const vector<Mat>& skeletons, int radius) {
    vector<Mat> clusteredSkeletons;
    vector<bool> skeletonVisited(skeletons.size(), false);  // 初始化骨架访问标志为false
    vector<vector<Point>> skeletonPointsList(skeletons.size());
    for (size_t i = 0; i < skeletons.size(); ++i) {
        findNonZero(skeletons[i], skeletonPointsList[i]);  // 获取每个骨架的所有点
    }
    for (size_t i = 0; i < skeletonPointsList.size(); ++i) {
        if (skeletonVisited[i]) continue;  // 如果骨架已经被访问，跳过

        vector<Mat> newCluster = {skeletons[i]};  // 创建新的聚类，加入当前骨架
        skeletonVisited[i] = true;  // 标记为已访问
        bool addedToCluster = true;
        vector<Point> allPoints = skeletonPointsList[i];  // 当前聚类中的所有点

        while (addedToCluster) {
            addedToCluster = false;
            for (size_t j = 0; j < skeletonPointsList.size(); ++j) {
                if (skeletonVisited[j]) continue;  // 如果骨架已经被访问，跳过
                for (const auto& point : skeletonPointsList[j]) {
                    bool withinRadius = false;
                    for (const auto& allPoint : allPoints) {
                        if (norm(point - allPoint) <= radius) {  // 判断点之间的距离是否小于指定半径
                            withinRadius = true;
                            break;
                        }
                    }
                    if (withinRadius) {
                        newCluster.push_back(skeletons[j]);  // 将骨架加入到当前聚类
                        skeletonVisited[j] = true;  // 标记为已访问
                        allPoints.insert(allPoints.end(), skeletonPointsList[j].begin(), skeletonPointsList[j].end());  // 更新所有点集合
                        addedToCluster = true;
                        break;
                    }
                }
            }
        }
        Mat mergedSkeleton = Mat::zeros(skeletons[0].size(), CV_8UC1);  // 创建空白骨架图像
        for (const auto& skel : newCluster) {
            bitwise_or(mergedSkeleton, skel, mergedSkeleton);  // 合并骨架
        }
        clusteredSkeletons.push_back(mergedSkeleton);  // 将合并后的骨架加入聚类结果
    }
    return clusteredSkeletons;
}

// 生成骨架图
vector<Mat> LaneDetectionOutsideNode::generateSkeletons(const vector<vector<Point>>& contours, Size binaryShape) {
    vector<Mat> finalSkeleton;
    for (const auto& contour : contours) {
        Mat mask = Mat::zeros(binaryShape, CV_8UC1);  // 创建与图像大小相同的空掩码
        drawContours(mask, vector<vector<Point>>{contour}, -1, Scalar(255), FILLED);  // 在掩码上绘制轮廓
        Mat skeleton;
        thinning(mask, skeleton, THINNING_GUOHALL);  // 对掩码进行骨架化处理（郭-霍尔算法）
        thinning(skeleton, skeleton, THINNING_ZHANGSUEN);  // 再次骨架化处理（张-孙算法）
        Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));  // 创建3x3矩形结构元素
        morphologyEx(skeleton, skeleton, MORPH_CLOSE, kernel);  // 对骨架进行闭运算
        finalSkeleton.push_back(skeleton);  // 将骨架加入最终结果
    }
    return finalSkeleton;
}
