/**
 * @brief This file contains the unit tests for psaf_controller
 */

 #include "gtest/gtest.h"
 #include "psaf_controller/controller_node.hpp"
 #include "rclcpp/rclcpp.hpp"
 #include "std_msgs/msg/int16.hpp"
 #include "std_msgs/msg/int32.hpp"
 #include "psaf_interfaces/msg/trajectory.hpp"
 
 /**
  * @brief Test suite for ControllerNode
  */
 class ControllerNodeTest : public ::testing::Test {
 protected:
     void SetUp() override {
         rclcpp::init(0, nullptr);
         node_ = std::make_shared<ControllerNode>();
     }
 
     void TearDown() override {
         rclcpp::shutdown();
     }
 
     std::shared_ptr<ControllerNode> node_;
 };
 
 // Einfache Tests zum Einstieg
 TEST(FirstStepsSampleTest, BasicSanityCheck) {
     EXPECT_EQ(1, 1);
 }
 
 // Test: StateCallback setzt den aktuellen Zustand und Geschwindigkeitsgrenzen korrekt
 TEST_F(ControllerNodeTest, StateCallbackUpdatesStateAndSpeedLimits) {
     auto msg = std::make_shared<std_msgs::msg::Int16>();
 
     msg->data = 1;  // Outer Circle
     node_->stateCallback(msg);
     EXPECT_EQ(node_->current_state_, 1);
     EXPECT_EQ(node_->v_max, 1000.0);
     EXPECT_EQ(node_->v_min, 850.0);
 
     msg->data = 2;  // Inner Circle
     node_->stateCallback(msg);
     EXPECT_EQ(node_->v_max, 750.0);
     EXPECT_EQ(node_->v_min, 600.0);
 
     msg->data = 3;  // Sign Detection
     node_->stateCallback(msg);
     EXPECT_EQ(node_->v_max, 400.0);
     EXPECT_EQ(node_->v_min, 400.0);
 }
 
 // Test: trajectoryCallback speichert die empfangene Trajektorie
 TEST_F(ControllerNodeTest, TrajectoryCallbackStoresTrajectory) {
     auto msg = std::make_shared<psaf_interfaces::msg::Trajectory>();
 
     // Dummy-Trajektorie mit zwei Punkten
     psaf_interfaces::msg::FloatPoint point1, point2;
     point1.x = 100.0;
     point1.y = 200.0;
     point2.x = 150.0;
     point2.y = 250.0;
 
     msg->trajectory.push_back(point1);
     msg->trajectory.push_back(point2);
 
     node_->trajectoryCallback(msg);
 
     EXPECT_EQ(node_->global_trajectory.size(), 2);
     EXPECT_EQ(node_->global_trajectory[0][0], 100.0);
     EXPECT_EQ(node_->global_trajectory[0][1], 200.0);
 }
 
 // Test: signCallback speichert die empfangene Sign-ID
 TEST_F(ControllerNodeTest, SignCallbackStoresSignId) {
     auto msg = std::make_shared<std_msgs::msg::Int32>();
     msg->data = 3;  // Tempo-30-Schild
 
     node_->signCallback(msg);
     EXPECT_EQ(node_->sign_id, 3);
 }
 
 // Test: calculate_path_curvature berechnet eine gültige Krümmung
 TEST_F(ControllerNodeTest, CalculatePathCurvatureComputesCurvature) {
     std::vector<std::vector<double>> trajectory = {
         {0.0, 0.0}, {10.0, 5.0}, {20.0, 15.0}, {30.0, 30.0}
     };
     
     double curvature = node_->calculate_path_curvature(trajectory, 3);
     EXPECT_GE(curvature, 0.0);
 }
 
 // Test: adaptive_lookahead berechnet einen gültigen Wert
 TEST_F(ControllerNodeTest, AdaptiveLookaheadComputesValidValue) {
     double lookahead = node_->adaptive_lookahead(300, 0.05);
     EXPECT_GE(lookahead, 800.0);
     EXPECT_LE(lookahead, 1000.0);
 }
 
 // Test: pure_pursuit_target berechnet eine gültige Lenkung und Geschwindigkeit
 TEST_F(ControllerNodeTest, PurePursuitTargetCalculatesValidSteeringAndSpeed) {
     std::vector<std::vector<double>> trajectory = {
         {0.0, 0.0}, {10.0, 5.0}, {20.0, 15.0}, {30.0, 30.0}
     };
 
     auto result = node_->pure_pursuit_target(trajectory);
 
     EXPECT_GE(result.first, -M_PI / 6);
     EXPECT_LE(result.first, M_PI / 6);
     EXPECT_GE(result.second, node_->v_min);
     EXPECT_LE(result.second, node_->v_max);
 }
 
 /**
  * @brief Main function for Google Test execution
  */
 int main(int argc, char **argv) {
     ::testing::InitGoogleTest(&argc, argv);
     return RUN_ALL_TESTS();
 }
 