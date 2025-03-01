/* LaneDetectionNode.cpp */
#include "psaf_lane_detection_outside/lane_detection_outside_node.hpp"
#include <iostream>
#include <filesystem>
#include "std_msgs/msg/int32.hpp"
//28.11 + purepursuit 

namespace fs = std::filesystem;

LaneDetectionOutsideNode::LaneDetectionOutsideNode(): Node("lane_detection_node")
{
    image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
              CAM_TOPIC_RGB, 10, std::bind(&LaneDetectionOutsideNode::imageCallback, this, std::placeholders::_1));
    //num_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
     //         "/number_topic", 10, std::bind(&LaneDetectionOutsideNode::numCallback, this, std::placeholders::_1));

    lane_markings_publisher_ = this->create_publisher<psaf_interfaces::msg::LaneMarking>(LANE_MARKINGS_TOPIC, 10);

    lane_marking_.resize(3);
}
void LaneDetectionOutsideNode::numCallback(const std_msgs::msg::Int32::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "received",msg->data);
    printf("1");
    std::string filename = "/home/psaf/Pictures/test_img.png";
    cv::Mat a = cv::imread(filename);
    auto start = std::chrono::steady_clock::now();
    processImage(a);
    
    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    RCLCPP_INFO(this->get_logger(), "Processing time: %ld ms", duration);
    publishLaneMarkings(lane_marking_);
}
void LaneDetectionOutsideNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg){
    //RCLCPP_INFO(this->get_logger(), "image with resolution: %d * %d", msg->width, msg->height);
    RCLCPP_INFO(this->get_logger(), "received",msg->data);
    
    //transform sensor message to opencv form
    cv::Mat resized_image, warped_image;
    cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

    current_image_ = cv_ptr->image;
    //std::string filename = "/home/yinan/Downloads/testimg.png";
    //cv::Mat a = cv::imread(filename);

    auto start = std::chrono::steady_clock::now();
    processImage(current_image_);
    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    
    RCLCPP_INFO(this->get_logger(), "=================================================================Processing time: %ld ms", duration);

//    std::stringstream filename_2;
//    filename_2 << "/home/psaf/Pictures/image_trans_" << ".png";
//    cv::imwrite(filename_2.str(), a);
    publishLaneMarkings(lane_marking_);
}

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


//publish function:



void LaneDetectionOutsideNode::processImage(const cv::Mat& img)

{



    if (img.empty()) {
        std::cerr << "Input image is empty." << std::endl;

        return ;
    }
    //printf("1");

    // Convert RGB to BGR
    //cv::Mat bgrImg;
    //cv::cvtColor(img, bgrImg, cv::COLOR_RGB2BGR);

    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    
    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size(9, 9), 0);
    int blockSize = 61;
    int constSubtrahend = -70;

    auto start = std::chrono::steady_clock::now();
    cv::Mat small_blurred;
    resize(blurred, small_blurred, Size(), 0.25, 0.25);
    cv::Mat small_binary;
    adaptiveThreshold(small_blurred, small_binary, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, blockSize, constSubtrahend);
    cv::Mat binary;
    resize(small_binary, binary, blurred.size());
    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();


//    cv::Mat binary;
//    int blockSize = 125;
//    int constSubtrahend = -50;
//    auto start = std::chrono::steady_clock::now();
//    cv::adaptiveThreshold(blurred, binary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, blockSize, constSubtrahend);
//    auto end = std::chrono::steady_clock::now();
//    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    
    RCLCPP_INFO(this->get_logger(), "=================================================================ADP time: %ld ms", duration);
    cv::Mat homographyMatrix = (cv::Mat_<double>(3, 3) << -3.0381, -8.6729, 2377.9484,
                                                         0.2921, -16.0217, 2881.3326,
                                                         0.0005, -0.0268, 1.0);

    cv::Mat transformed;
    cv::warpPerspective(binary, transformed, homographyMatrix, cv::Size(640, 480));
    std::stringstream filename;
    filename << "/home/psaf/Pictures/trans/image_" <<img_count<< ".png";
    cv::imwrite(filename.str(), transformed);
    img_count++;
    
    cv::Mat imgCopy;
    cv::cvtColor(transformed, imgCopy, cv::COLOR_GRAY2BGR);
    
    //cv::String outputFilename = "/home/psaf/Pictures/output_img.png";

    extractLaneMarkings(transformed, imgCopy);
    //cv::imwrite(outputFilename, imgCopy);
    
    std::vector<std::vector<std::pair<int, int>>> lanes;
    for (const auto& point : right_lane) {
        lanes.push_back({{point.x, point.y}});
    }
    for (const auto& point : center_lane) {
        lanes.push_back({{point.x, point.y}});
    }

}

void LaneDetectionOutsideNode::extractLaneMarkings(const cv::Mat& img, cv::Mat& debugImage){


    int sl_width = 60;
    int sl_height = 40;
    
    if (img.empty() || img.cols != 640 || img.rows != 480) {
        std::cerr << "Input image is empty or not the correct size." << std::endl;
        return;
    }

    auto basePoints = getBasePoints(img, debugImage);
//    if (basePoints.empty()) {
//        std::cerr << "No base points found in image." << std::endl;
//        return;
//    }

    BasePointInfo bpInfo = getBasePointsInformation(basePoints, img.size());
    std::vector<cv::Point> tempLeftPoints, tempCenterPoints, tempRightPoints;
    int sumNonZeroCenter = 0, sumNonZeroRight = 0;
    int windowCount = 0, emptyCenterWindows = 0;
    bool centerIsOk = true, rightIsOk = true;

    if (startpoint.x!= 0||startpoint.y!= 0) {
        int yBegin = startpoint.y;
        for (int y = yBegin; y > 100; y -= 15) {
            if (bpInfo.searchC) {
                bpInfo.cBase = std::max(sl_width, std::min(bpInfo.cBase, img.cols - sl_width));
                cv::Rect cWindow(bpInfo.cBase - sl_width, y - sl_height, sl_width * 2, sl_height);
                cv::Mat subImg = img(cWindow);

                if (windowCount <= 3) {
                    sumNonZeroCenter += cv::countNonZero(subImg);
                }

                auto [maxPoint, valid] = getMaxContour(subImg);
                if (valid && centerIsOk) {
                    int cx = maxPoint.x;
                    int cy = maxPoint.y;
                    cv::Point p(bpInfo.cBase - sl_width + cx, y - cy);
                    tempCenterPoints.push_back(p);
                    bpInfo.cBase = bpInfo.cBase - sl_width + cx;
                } else {
                    emptyCenterWindows++;
                    if (emptyCenterWindows >= 2) {
                        centerIsOk = false;
                    }
                }

//                if (!debugImage.empty()) {
//                    if (centerIsOk) {
//                        cv::rectangle(debugImage, cWindow, cv::Scalar(255, 255, 255), 1);
//                    }
//                }
            }
        }
    }

    if (second_startpoint.x!=0 ||second_startpoint.y!= 0) {
        int yBegin = second_startpoint.y;
        int xBegin = second_startpoint.x;
        for (int y = yBegin; y > 100; y -= 15) {
            if (bpInfo.searchC) {
                bpInfo.cBase = std::max(sl_width, std::min(xBegin, img.cols - sl_width));
                cv::Rect cWindow(bpInfo.cBase - sl_width, y - sl_height, sl_width * 2, sl_height);
                cv::Mat subImg = img(cWindow);

                auto [maxPoint, valid] = getMaxContour(subImg);
                if (valid && centerIsOk) {
                    int cx = maxPoint.x;
                    int cy = maxPoint.y;
                    cv::Point p(bpInfo.cBase - sl_width + cx, y - cy);
                    tempCenterPoints.push_back(p);
                    xBegin = xBegin - sl_width + cx;
                } else {
                    emptyCenterWindows++;
                    if (emptyCenterWindows >= 2) {
                        centerIsOk = false;
                    }
                }

                if (!debugImage.empty()) {
                    if (centerIsOk) {
                        cv::rectangle(debugImage, cWindow, cv::Scalar(255, 255, 255), 1);
                    }
                }
            }
        }
    }

    for (int y = 400; y > 40; y -= 40) {
        if (bpInfo.searchR) {
            bpInfo.rBase = std::max(sl_width, std::min(bpInfo.rBase, img.cols - sl_width));
            cv::Rect rWindow(bpInfo.rBase - sl_width, y - sl_height, sl_width * 2, sl_height);
            cv::Mat subImg = img(rWindow);

            if (windowCount <= 3) {
                sumNonZeroRight += cv::countNonZero(subImg);
            }

            auto [maxPoint, valid] = getMaxContour(subImg);
            if (valid && rightIsOk) {
                int cx = maxPoint.x;
                int cy = maxPoint.y;
                cv::Point toAdd;
                if (cx - sl_width > 25) {
                    toAdd = cv::Point(bpInfo.rBase, y - cy);
                } else {
                    toAdd = cv::Point(bpInfo.rBase - sl_width + cx, y - cy);
                    bpInfo.rBase = bpInfo.rBase - sl_width + cx;
                }
                tempRightPoints.push_back(toAdd);
            } else {
                rightIsOk = false;
            }
            windowCount++;

//            if (!debugImage.empty()) {
//                if (rightIsOk) {
//                    cv::rectangle(debugImage, rWindow, cv::Scalar(255, 255, 255), 1);
//                }
//            }
        }
    }

    //right_lane = tempRightPoints;
    //center_lane = tempCenterPoints;
    std::vector<cv::Point> tempCenterList;    // Draw lane points on the debug image
    for(const auto& point : tempCenterPoints) {
        cv::circle(debugImage, point, 5, cv::Scalar(0, 0, 255), -1);
        cv::putText(debugImage, "(" + std::to_string(point.x) + ", " + std::to_string(point.y) + ")", point + cv::Point(10, -10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1, cv::LINE_AA);
        tempCenterList.push_back(cv::Point(point.x,point.y));
    }
    center_lane = tempCenterList;

    std::vector<cv::Point> tempRightList;
    for(const auto& point : tempRightPoints) {
        cv::circle(debugImage, point, 5, cv::Scalar(0, 255, 0), -1);
        cv::putText(debugImage, "(" + std::to_string(point.x) + ", " + std::to_string(point.y) + ")", point + cv::Point(10, -10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1, cv::LINE_AA);
        tempRightList.push_back(cv::Point(point.x,point.y));
    }
    right_lane = tempRightList;
    for (size_t i = 0; i<right_lane.size(); ++i){
        std::cout<< "Point"<< i <<":("
                 << right_lane[i].x<< ","
                 << right_lane[i].y<< ")"
                 << std::endl;   
    } 
    //left_lane = left_lane()
    lane_marking_[0] = left_lane;
    lane_marking_[1] = center_lane;
    lane_marking_[2] = right_lane;

//    std::stringstream filename;
//    filename << "/home/psaf/Pictures/trans/image_" <<img_count<< ".png";
//    cv::imwrite(filename.str(), debugImage);
//    img_count++;

//    lane_markings = {tempLeftPoints, tempCenterPoints, tempRightPoints};
}

std::pair<cv::Point, bool> LaneDetectionOutsideNode::getMaxContour(const cv::Mat& img){
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(img, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    if (!contours.empty()) {
        auto maxContour = *std::max_element(contours.begin(), contours.end(), [](const auto& c1, const auto& c2) {
            return cv::contourArea(c1) < cv::contourArea(c2);
        });
        auto moment = cv::moments(maxContour);
        if (moment.m00 != 0) {
            int cx = static_cast<int>(moment.m10 / moment.m00);
            int cy = static_cast<int>(moment.m01 / moment.m00);
            return {cv::Point(cx, cy), true};
        }
    }
    return {cv::Point(), false};
}

std::vector<cv::Point> LaneDetectionOutsideNode::getBasePoints(const cv::Mat& img, cv::Mat& debugImage){
    std::vector<cv::Point> result;
    cv::Mat subImg = img(cv::Rect(0, 0, 640, 480));
    std::vector<std::vector<cv::Point>> contours;
    std::vector<std::vector<cv::Point>> filteredContours;
    cv::findContours(subImg, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    int minArea = 350;
    int maxArea = 1500;

    double maxMaxY = std::numeric_limits<double>::lowest();
    double maxMaxX = std::numeric_limits<double>::lowest();
    double secondMaxY = std::numeric_limits<double>::lowest();
    double secondMaxX = std::numeric_limits<double>::lowest();

    // Filter contours based on area and other conditions
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area >= minArea && area <= maxArea) {
            std::vector<int> contourXCoords, contourYCoords;
            for (const auto& pt : contour) {
                contourXCoords.push_back(pt.x);
                contourYCoords.push_back(pt.y);
            }

            int maxX = *std::max_element(contourXCoords.begin(), contourXCoords.end());
            int maxY = *std::max_element(contourYCoords.begin(), contourYCoords.end());
            int minX = *std::min_element(contourXCoords.begin(), contourXCoords.end());
            int minY = *std::min_element(contourYCoords.begin(), contourYCoords.end());

            if ((maxX - minX) >= (maxY - minY) * 4) {
                continue;
            }
            if ((maxX - minX) * 8 <= (maxY - minY)) {
                continue;
            }
            if ((maxX > 440) || (minY < 70) || (minX < 5)  || (maxY > 400)) {
                continue;
            }
            filteredContours.push_back(contour);
        }
    }

    const std::vector<cv::Point>* maxContour = nullptr;
    // Find the contour with the largest max y value
    for (const auto& contour : filteredContours) {
        cv::drawContours(debugImage, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(128, 0, 128), 2);
        std::vector<int> contourXCoords, contourYCoords;
        for (const auto& pt : contour) {
            contourXCoords.push_back(pt.x);
            contourYCoords.push_back(pt.y);
        }
        for (size_t i = 0; i < contourYCoords.size(); ++i) {
            if (contourYCoords[i] > maxMaxY && maxMaxX - contourXCoords[i] < 150) {
                maxMaxY = contourYCoords[i];
                maxMaxX = contourXCoords[i];
                maxContour = &contour;
            }
        }
    }

    if (maxContour) {
        filteredContours.erase(std::remove(filteredContours.begin(), filteredContours.end(), *maxContour), filteredContours.end());
    }

    // Find the second largest y value if(filtered_contours)
    for (const auto& contour : filteredContours) {
        std::vector<int> contourXCoords, contourYCoords;
        for (const auto& pt : contour) {
            contourXCoords.push_back(pt.x);
            contourYCoords.push_back(pt.y);
        }
        for (size_t i = 0; i < contourYCoords.size(); ++i) {
            if (contourYCoords[i] > secondMaxY && std::abs(secondMaxX - contourXCoords[i]) < 100) {
                secondMaxY = contourYCoords[i];
                secondMaxX = contourXCoords[i];
            }
        }
    }

    if (maxMaxX != std::numeric_limits<double>::lowest() && maxMaxY != std::numeric_limits<double>::lowest()) {
        startpoint = cv::Point(static_cast<int>(maxMaxX), static_cast<int>(maxMaxY));
    }

    if (secondMaxX != std::numeric_limits<double>::lowest() && secondMaxY != std::numeric_limits<double>::lowest()) {
        second_startpoint = cv::Point(static_cast<int>(secondMaxX), static_cast<int>(secondMaxY));
    }

    if (maxMaxX != std::numeric_limits<double>::lowest() && maxMaxY != std::numeric_limits<double>::lowest()) {
        cv::circle(debugImage, cv::Point(maxMaxX, maxMaxY), 5, cv::Scalar(0, 255, 0), -1);
        cv::putText(debugImage, "(" + std::to_string(static_cast<int>(maxMaxX)) + ", " + std::to_string(static_cast<int>(maxMaxY)) + ")",
                    cv::Point(maxMaxX + 10, maxMaxY - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    }

//    if (secondMaxX != std::numeric_limits<double>::lowest() && secondMaxY != std::numeric_limits<double>::lowest()) {
//        cv::circle(debugImage, cv::Point(secondMaxX, secondMaxY), 5, cv::Scalar(0, 255, 0), -1);
//        cv::putText(debugImage, "(" + std::to_string(static_cast<int>(secondMaxX)) + ", " + std::to_string(static_cast<int>(secondMaxY)) + ")",
//                    cv::Point(secondMaxX + 10, secondMaxY - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
//    }

    cv::Point lastBasePoint;
    for (int x = 320; x < img.cols - 140; x += 64) {
        cv::Rect hWindow(x, 350, 64, 70);
        cv::Mat subWindow = img(hWindow);
        int non_zero_count = cv::countNonZero(subWindow);
        if (non_zero_count <80 ){
            continue;
        }
        auto [basePoint, valid] = getMaxContour(subWindow);

        if (valid) {
            basePoint.x += x;
            basePoint.y += 385;

            if (startpoint.x !=0 && startpoint.y !=0 && basePoint != cv::Point() ) {
                if (std::abs(basePoint.x - startpoint.x) >= 110) {
                        result.push_back(basePoint);
                        //lastBasePoint = basePoint;
                    }
            else {
                result.push_back(basePoint);
                //lastBasePoint = basePoint;
            }
//                if (!debugImage.empty()) {
//                    cv::rectangle(debugImage, cv::Rect(x, 330, 64, 60), cv::Scalar(0, 0, 255));
//                    cv::drawMarker(debugImage, basePoint, cv::Scalar(0, 0, 255));
//                }
            }
        }
    }

    return result;
}

//void LaneDetectionNode::drawTextAndPoint(cv::Mat& image, const cv::Point& point, const std::string& text) {
//    cv::circle(image, point, 5, cv::Scalar(0, 255, 0), -1);
//    cv::putText(image, text, point + cv::Point(10, -10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1, cv::LINE_AA);
//}
BasePointInfo LaneDetectionOutsideNode::getBasePointsInformation(const std::vector<cv::Point>& basePoints, const cv::Size& imageSize){
    BasePointInfo info = { 0, 0, 0, false, false, false };
    if (startpoint.x != 0 || startpoint.y != 0) {
        info.cBase = startpoint.x;
        info.searchC = true;
    }
    if (basePoints.size() >=1 ) {
        info.rBase = basePoints[0].x;
        info.searchR = true;
    }
    return info;
}
    /*
    if (basePoints.size() == 1 && (startpoint.x != 0 || startpoint.y != 0)) {
        
    }
    else if (basePoints.size() == 1 && (startpoint.x == 0 && startpoint.y == 0)) {
        info.rBase = basePoints[0].x;
        info.searchR = true;
    }
    else if (basePoints.size() > 1 && (startpoint.x == 0 && startpoint.y == 0)) {
        if (basePoints[0].x > imageSize.width * 2 / 5) {
            info.rBase = basePoints[0].x;
            info.searchR = true;
        }
        else {
            info.cBase = basePoints[0].x;
            info.searchC = true;
            info.rBase = basePoints[1].x;
            info.searchR = true;
        }
    }
     */
    



//int main() {
//    std::string inputDirectory = "D:/AF/test/imagetest";
//    std::string outputDirectory = "D:/AF/test/test_result";
//
//    if (!fs::exists(outputDirectory)) {
//        fs::create_directory(outputDirectory);
//    }
//
//    std::vector<double> processingTimes;
//
//    for (const auto& entry : fs::directory_iterator(inputDirectory)) {
//        if (entry.is_regular_file()) {
//            std::string imagePath = entry.path().string();
//            cv::Mat img = cv::imread(imagePath);
//            if (img.empty()) {
//                std::cerr << "Unable to read image: " << imagePath << std::endl;
//                continue;
//            }
//
//            LaneDetectionNode laneDetection;
//            auto startTime = std::chrono::high_resolution_clock::now();
//            auto output = laneDetection.processImage(img);
//            auto endTime = std::chrono::high_resolution_clock::now();
//
//            double processingTime = std::chrono::duration<double>(endTime - startTime).count();
//            processingTimes.push_back(processingTime);
//
//            std::string outputPath = outputDirectory + "/processed_" + entry.path().filename().string();
//            if (!laneDetection.right_lane.empty()) {
//                cv::imwrite(outputPath, img);
//                std::cout << "Processed and saved: " << outputPath << std::endl;
//            }
//        }
//    }
//
//    if (!processingTimes.empty()) {
//        double avgTime = std::accumulate(processingTimes.begin(), processingTimes.end(), 0.0) / processingTimes.size();
//        auto [minIt, maxIt] = std::minmax_element(processingTimes.begin(), processingTimes.end());
//
//        std::cout << "\nAll images processed." << std::endl;
//        std::cout << "Average processing time: " << avgTime << " seconds" << std::endl;
//        std::cout << "Max processing time: " << *maxIt << " seconds" << std::endl;
//        std::cout << "Min processing time: " << *minIt << " seconds" << std::endl;
//    }
//
//    std::cout << "All images processed." << std::endl;
//    return 0;
//}
