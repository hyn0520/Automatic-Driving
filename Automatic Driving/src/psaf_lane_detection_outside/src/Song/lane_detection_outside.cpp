
// Created by psaf on 30.10.24.

#include "rclcpp/rclcpp.hpp"
#include "psaf_lane_detection_outside/lane_detection_outside_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<LaneDetectionOutsideNode> node = std::make_shared<LaneDetectionOutsideNode>();
    double update_frequency = node->declare_parameter("update_frequency", 2.0);
    rclcpp::WallRate rate(update_frequency);

    while (rclcpp::ok()) {
        auto start = std::chrono::steady_clock::now();
        rclcpp::spin_some(node);
        //node->update();
        rate.sleep();
        //auto end = std::chrono::steady_clock::now();
        //auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        //RCLCPP_INFO(node->get_logger(), "===================================================== Lane_detection process time: %ld ms", duration);
    }

//    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
//    rclcpp::spin_some(node);
//    rclcpp::shutdown();
}


//#include "rclcpp/rclcpp.hpp"
//#include "psaf_lane_detection_outside/lane_detection_outside_node.hpp"
//#include <fstream>
//#include <chrono>
//
//int main(int argc, char *argv[]) {
//    rclcpp::init(argc, argv);
//
//    std::shared_ptr<LaneDetectionOutsideNode> node = std::make_shared<LaneDetectionOutsideNode>();
//    double update_frequency = node->declare_parameter("update_frequency", 8.0);
//    rclcpp::WallRate rate(update_frequency);
//
//    // 打开文件用于保存 duration
//    std::ofstream duration_file("lane_detection_durations.txt", std::ios::out | std::ios::app);
//    if (!duration_file.is_open()) {
//        RCLCPP_ERROR(node->get_logger(), "Failed to open file for writing durations!");
//        return 1;
//    }
//
//    while (rclcpp::ok()) {
//        auto start = std::chrono::steady_clock::now();
//        rclcpp::spin_some(node);
//        // node->update();
//        rate.sleep();
//        auto end = std::chrono::steady_clock::now();
//        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
//
//        // 打印日志
//        RCLCPP_INFO(node->get_logger(), "===================================================== Lane_detection process time: %ld ms", duration);
//
//        // 写入文件
//        duration_file << duration << " ms" << std::endl;
//    }
//
//    // 关闭文件
//    duration_file.close();
//
//}
