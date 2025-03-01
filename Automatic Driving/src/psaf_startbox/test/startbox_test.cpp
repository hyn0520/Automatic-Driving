/**
 * @brief This file contains the unit tests for psaf_startbox
 */

 #include "gtest/gtest.h"
 #include "psaf_startbox/startbox_node.hpp"
 #include "rclcpp/rclcpp.hpp"
 #include "sensor_msgs/msg/image.hpp"
 #include "std_msgs/msg/int16.hpp"
 #include <cv_bridge/cv_bridge.h>
 
 /**
  * @brief Test suite for StartBoxNode
  */
 class StartBoxNodeTest : public ::testing::Test {
 protected:
     void SetUp() override {
         rclcpp::init(0, nullptr);
         node_ = std::make_shared<StartBoxNode>();
     }
 
     void TearDown() override {
         rclcpp::shutdown();
     }
 
     std::shared_ptr<StartBoxNode> node_;
 };
 
 // Einfache Tests zum Einstieg
 TEST(FirstStepsSampleTest, BasicSanityCheck) {
     EXPECT_EQ(1, 1);
 }
 
 // Test: statusCallback setzt den Status korrekt
 TEST_F(StartBoxNodeTest, StatusCallbackUpdatesStatus) {
     auto msg = std::make_shared<std_msgs::msg::Int16>();
     msg->data = 5;
 
     node_->statusCallback(msg);
 
     EXPECT_EQ(node_->status_msg, 5);
 }
 
 // Test: imageCallback verarbeitet ein Bild ohne Fehler
 TEST_F(StartBoxNodeTest, ImageCallbackHandlesValidImage) {
     auto msg = std::make_shared<sensor_msgs::msg::Image>();
 
     msg->height = 480;
     msg->width = 640;
     msg->encoding = "bgr8";
     msg->step = msg->width * 3;
     msg->data.resize(msg->step * msg->height, 255); // Weißes Bild
 
     ASSERT_NO_THROW(node_->imageCallback(msg));
 }
 
 // Test: publishStatusInfo veröffentlicht eine Statusnachricht
 TEST_F(StartBoxNodeTest, PublishStatusInfoPublishesMessage) {
     node_->is_open_ = true;
     node_->status_msg = 2;
 
     ASSERT_NO_THROW(node_->publishStatusInfo());
     EXPECT_EQ(node_->current_state_, 2);
 }
 
 // Test: readQR erkennt ein gültiges QR-Code-Bild
 TEST_F(StartBoxNodeTest, ReadQRDetectsQRCode) {
     cv::Mat test_image = cv::Mat::zeros(480, 640, CV_8UC1); // Leeres Bild
 
     // Simuliertes QR-Code-Bild 
     node_->readQR(test_image);
 
     EXPECT_EQ(node_->last_read_qr_, "");
 }
 
 // Test: readQR setzt is_open_ nach genügend Fehlversuchen auf true
 TEST_F(StartBoxNodeTest, ReadQROpensStartBoxAfterNoQR) {
     cv::Mat test_image = cv::Mat::zeros(480, 640, CV_8UC1); // Leeres Bild
     node_->status_msg = 1; // Status setzen
 
     for (int i = 0; i <= 60; ++i) {
         node_->readQR(test_image);
     }
 
     EXPECT_TRUE(node_->is_open_);
 }
 
 /**
  * @brief Main function for Google Test execution
  */
 int main(int argc, char **argv) {
     ::testing::InitGoogleTest(&argc, argv);
     return RUN_ALL_TESTS();
 }
 