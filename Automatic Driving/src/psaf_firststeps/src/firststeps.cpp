#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/int16.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"


class ControllerToVehicleNode : public rclcpp::Node {
public:
    ControllerToVehicleNode() : Node("controller_to_vehicle") {
        // Parameter laden
        this->declare_parameter<int>("axis_steering", 0);
        this->declare_parameter<int>("axis_speed", 3);
        this->declare_parameter<double>("scale_steering", 1.0);
        this->declare_parameter<double>("scale_speed", 1.0);
        this->declare_parameter<int>("enable_button", 6);
        this->declare_parameter<int>("enable_reverse_button", 7);

        // Parameterwerte abrufen
        axis_steering_ = this->get_parameter("axis_steering").as_int();
        axis_speed_ = this->get_parameter("axis_speed").as_int();
        scale_steering_ = this->get_parameter("scale_steering").as_double();
        scale_speed_ = this->get_parameter("scale_speed").as_double();
        enable_button_ = this->get_parameter("enable_button").as_int();
        enable_reverse_button_ = this->get_parameter("enable_reverse_button").as_int();

        // Subscriber und Publisher erstellen
        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&ControllerToVehicleNode::joyCallback, this, std::placeholders::_1));

        steering_publisher_ = this->create_publisher<std_msgs::msg::Int16>("uc_bridge/set_steering", 10);
        speed_publisher_ = this->create_publisher<std_msgs::msg::Int16>("uc_bridge/set_motor_level_forward", 10);
        backward_speed_publisher_ = this->create_publisher<std_msgs::msg::Int16>("uc_bridge/set_motor_level_backward", 10);

        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
                "color/image_raw", 10, std::bind(&ControllerToVehicleNode::imageCallback, this, std::placeholders::_1));

    }

private:
    // Callback-Funktion für Joy-Nachrichten
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        // Prüfen, ob der Aktivierungsbutton gedrückt ist
   
	    // Lenkung berechnen
	    int steering_value = static_cast<int>(msg->axes[axis_steering_] * scale_steering_ * -300);
	    auto steering_msg = std_msgs::msg::Int16();
	    steering_msg.data = steering_value;
	    steering_publisher_->publish(steering_msg);

	    // Geschwindigkeit berechnen
	    int speed_value = static_cast<int>(msg->axes[axis_speed_] * scale_speed_ * 1000);

        auto speed_msg = std_msgs::msg::Int16();

	    // Prüfen, ob der Rückwärtsbutton gedrückt ist
	    if (msg->buttons[enable_reverse_button_] == 1) {
            speed_msg.data = -1 * speed_value;;  // Rückwärtsfahrt
            backward_speed_publisher_->publish(speed_msg);
	    }


	    speed_msg.data = speed_value;
	    speed_publisher_->publish(speed_msg);
       
    }
    void imageCallback(sensor_msgs::msg::Image::SharedPtr msg){
        //RCLCPP_INFO(this->get_logger(), "image with resolution: %d * %d", msg->width, msg->height);

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::stringstream filename;
        filename << "/home/psaf/Pictures/image_" << image_count_ << "_" << std::put_time(std::localtime(&now_time), "%Y%m%d_%H%M%S") << ".png";


//        cv::imwrite(filename.str(), cv_ptr->image);
        RCLCPP_INFO(this->get_logger(), "Image saved to: %s", filename.str().c_str());


        image_count_++;
    }


    // Parameter
    int axis_steering_;
    int axis_speed_;
    double scale_steering_;
    double scale_speed_;
    int enable_button_;
    int enable_reverse_button_;
    size_t image_count_;

    // Subscriber und Publisher
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr steering_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr speed_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr backward_speed_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    while (rclcpp::ok()){
    rclcpp::spin(std::make_shared<ControllerToVehicleNode>());
    }
    
    rclcpp::shutdown();
    return 0;
}

