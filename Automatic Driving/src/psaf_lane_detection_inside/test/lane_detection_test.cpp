/**
* @brief This file contains the unit tests for psaf_lane_detection_inside
*/

#include "gtest/gtest.h"
#include "psaf_lane_detection_inside/lane_detection_inside_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/int16.hpp"

/**
 * @brief Test suite for LaneDetectionInsideNode
 */
class LaneDetectionInsideNodeTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<LaneDetectionInsideNode>();
    }

    void TearDown() override {
        rclcpp::shutdown();
    }

    std::shared_ptr<LaneDetectionInsideNode> node_;
};

// Einfache Tests zum Einstieg
TEST(FirstStepsSampleTest, BasicSanityCheck) {
    EXPECT_EQ(1, 1);
}

// Test: StateCallback setzt den aktuellen Zustand korrekt
TEST_F(LaneDetectionInsideNodeTest, StateCallbackUpdatesState) {
    auto msg = std::make_shared<std_msgs::msg::Int16>();
    msg->data = 5;
    
    node_->stateCallback(msg);
    
    EXPECT_EQ(node_->current_state_, 5);
}

// Test: DistanceCallback setzt den Hinder-Flag korrekt
TEST_F(LaneDetectionInsideNodeTest, DistanceCallbackSetsHinderFlag) {
    auto msg = std::make_shared<sensor_msgs::msg::Range>();

    msg->range = 0.5;
    node_->distanceCallback(msg);
    EXPECT_TRUE(node_->hinder);

    msg->range = 2.0;
    node_->distanceCallback(msg);
    EXPECT_FALSE(node_->hinder);
}

// Test: ImageCallback verarbeitet ungültige Bilder fehlerfrei
TEST_F(LaneDetectionInsideNodeTest, ImageCallbackHandlesInvalidImage) {
    auto msg = std::make_shared<sensor_msgs::msg::Image>();
    
    // Simuliert einen Fehlerhaften Input
    ASSERT_NO_THROW(node_->imageCallback(msg));
}

// Test: PublishLaneMarkings funktioniert fehlerfrei
TEST_F(LaneDetectionInsideNodeTest, PublishLaneMarkingsPublishesMessage) {
    auto lane_markings = std::vector<std::vector<cv::Point>>{
        {cv::Point(100, 200)}, 
        {cv::Point(150, 250)}, 
        {cv::Point(200, 300)}
    };

    ASSERT_NO_THROW(node_->publishLaneMarkings(lane_markings));
}

// Test: ProcessImageNormal verarbeitet ein Dummy-Bild
TEST_F(LaneDetectionInsideNodeTest, ProcessImageNormalHandlesImageProcessing) {
    cv::Mat test_image = cv::Mat::zeros(480, 640, CV_8UC3);
    
    ASSERT_NO_THROW(node_->processImageNormal(test_image, 10));
}

// Test: ProcessImageReverse verarbeitet ein Dummy-Bild
TEST_F(LaneDetectionInsideNodeTest, ProcessImageReverseHandlesImageProcessing) {
    cv::Mat test_image = cv::Mat::zeros(480, 640, CV_8UC3);
    
    ASSERT_NO_THROW(node_->processImageReverse(test_image, 10));
}

/**
 * @brief Main function for Google Test execution
 */
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
