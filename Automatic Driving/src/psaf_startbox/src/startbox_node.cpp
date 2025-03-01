//
// Created by psaf on 18.12.24.
//
#include "psaf_startbox/startbox_node.hpp"

/**
 * @class StartBoxNode
 * @brief ROS2-Node zur Verarbeitung von Kamerabildern und QR-Codes in der Startbox
 */

/**
 * @brief Konstruktor der StartBoxNode-Klasse.
 * Erstellt Abonnenten für Kamera- und Statusnachrichten sowie einen Publisher für Statusinformationen.
 */
StartBoxNode::StartBoxNode() : Node("startbox") {

    image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            CAM_TOPIC_RGB, 10, std::bind(&StartBoxNode::imageCallback, this, std::placeholders::_1));

    status_subscriber_ = this->create_subscription<std_msgs::msg::Int16>(
            "StatusInfo", 10, std::bind(&StartBoxNode::statusCallback, this, std::placeholders::_1));

    state_publisher_ = this->create_publisher<std_msgs::msg::Int16>(
            "StateInfo", 10);
}

/**
 * @brief Callback-Funktion für den Status-Subscriber.
 * Speichert die empfangene Statusnachricht.
 * 
 * @param msg Empfangene Statusnachricht als Int16
 */
void StartBoxNode::statusCallback(std_msgs::msg::Int16::SharedPtr msg){
    status_msg = msg->data;
}


/**
 * @brief Callback-Funktion für den Kamera-Subscriber.
 * Konvertiert das empfangene Bild, verarbeitet es und liest QR-Codes.
 * 
 * @param msg Empfangene Bildnachricht
 */
void StartBoxNode::imageCallback(sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    current_image_ = cv_ptr->image;

    if (current_image_.cols != 640 || current_image_.rows != 480) {
        cv::resize(current_image_, current_image_, cv::Size(640, 480));
    }

    cv::cvtColor(current_image_, current_image_, cv::COLOR_BGR2GRAY);

    readQR(current_image_);
    publishStatusInfo();

}

/**
 * @brief Veröffentlicht den aktuellen Status, wenn die Startbox geöffnet ist.
 * Falls der aktuelle Status ungleich 0 ist, wird der Node heruntergefahren.
 */
void StartBoxNode::publishStatusInfo(){
    if (is_open_) {
        current_state_ = status_msg;
        std_msgs::msg::Int16 msg;
        msg.data = static_cast<int>(current_state_);
        state_publisher_->publish(msg);
    }

    if (current_state_ != 0) {
        // shutdown the node since it is not needed anymore
        rclcpp::shutdown();
    }
}

/**
 * @brief Liest QR-Codes aus einem gegebenen Bild und aktualisiert den Status.
 * 
 * @param image Referenz auf das zu analysierende Bild
 */
void StartBoxNode::readQR(cv::Mat & image)
{
    zbar::Image image_to_zbar(image.cols, image.rows, "Y800", image.data, image.cols * image.rows);
    zbar::ImageScanner scanner;
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
    int n = scanner.scan(image_to_zbar);
    if (n > 0) {
        for (zbar::Image::SymbolIterator symbol = image_to_zbar.symbol_begin();
             symbol != image_to_zbar.symbol_end(); ++symbol)
        {
            if (symbol->get_type() == zbar::ZBAR_QRCODE) {
                last_read_qr_ = symbol->get_data();

                if (last_read_qr_ == "STOP") {
                    detected_at_least_once_ = true;
                    no_qr_msg_counter_ = 0;
                }
                break;
            }
        }
    } else {
        last_read_qr_ = "";
        if (status_msg > 0) {
            no_qr_msg_counter_++;
        }
    }
    RCLCPP_ERROR(this->get_logger(), "no_qr_msg_counter_: %d", no_qr_msg_counter_);
    if (no_qr_msg_counter_ > 60) {
        is_open_ = true;
    }
}