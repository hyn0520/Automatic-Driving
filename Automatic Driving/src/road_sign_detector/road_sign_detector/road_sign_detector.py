import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from std_msgs.msg import Int16
import cv2
from cv_bridge import CvBridge
import torch
import time
import torch.nn as nn


class RoadSignDetector(Node):
    """ROS2 Node for detecting road signs using YOLOv5.
    
    This node subscribes to image data from a camera and detects road signs using a trained YOLOv5 model.
    The detected sign's class index is then published.
    """   

    def __init__(self):
        super().__init__('road_sign_detector')

        self.status = -1

        self.status_subscription = self.create_subscription(
            Int16,
            '/StateInfo',
            self.status_callback,
            10
        )

        # Subscribe to the Camera Photos topic
        self.image_subscription = self.create_subscription(
            Image,
            "color/image_raw",
            self.image_callback,
            10
        )

        # Release test results
        self.detection_publisher = self.create_publisher(Int32, 'road_sign_class', 10)

        # Initialize tools and models
        self.bridge = CvBridge()
        self.model = self.initialize_yolov5()
        self.labels = ['Richtungsweiser', 'fußgaenger', 'stop', 'tempo 30', 'tempo 30 zone beginn',
                       'tempo 30 zone ende', 'vorfahrt achten']

    def initialize_yolov5(self):
        """Loads the YOLOv5 model for road sign detection.

        Returns:
            torch.nn.Module: The loaded YOLOv5 model.
        """              
        try:
            model_path = '/home/psaf/ws-template/src/road_sign_detector/yolov5.pt' 
            model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path, force_reload=True)

            self.get_logger().info('YOLOv5 custom model loaded successfully!')
            return model
        except Exception as e:
            self.get_logger().error(f'Error loading YOLOv5 model: {e}')
            raise

    def status_callback(self, msg):
        """Callback function to update the current status.

        Args:
            msg (std_msgs.msg.Int16): The received status message.
        """        
        self.status = msg.data
        self.get_logger().info(f"Updated status to: {self.status}")

    def image_callback(self, msg):
        """Callback function to process incoming images and detect road signs.

        Args:
            msg (sensor_msgs.msg.Image): The received image message.
        """    
        if self.status != 3:
            self.get_logger().info("Skipping inference as status is not 3.")
            return

        try:
            # Convert ROS image messages to OpenCV images
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge exception: {e}')
            return

        # reasoning
        results = self.model(cv_image)

        # Get test results
        largest_box, largest_label, largest_conf, largest_area = self.parse_yolo_results(results)
        self.get_logger().info(f"area:{largest_area:.2f}")

        # If a valid result is detected
        if largest_box is not None and largest_area >= 10000.0:
            self.get_logger().info(f"Largest Detection: {self.labels[largest_label]} with confidence {largest_conf:.2f}")
            detection_value = largest_label  # Returns the corresponding index value
        else:
            self.get_logger().info("No valid labels detected, publishing 7.")
            detection_value = 7  # The specified tag was not detected

        # Release test results
        detection_msg = Int32()
        detection_msg.data = detection_value
        self.detection_publisher.publish(detection_msg)


    def parse_yolo_results(self, results):
        """Parses the YOLOv5 detection results to find the largest detected object.

        Args:
            results (torch.Tensor): YOLOv5 inference results.

        Returns:
            tuple: (largest bounding box, class index, confidence, area of the detection)
        """
        largest_area = 0
        largest_box = None
        largest_label = None
        largest_conf = None

        for *box, conf, cls in results.xyxy[0]:  # Iterate through each detection
            x1, y1, x2, y2 = map(int, box)
            area = (x2 - x1) * (y2 - y1)  # Calculate area
            if area > largest_area:
                largest_area = area
                largest_box = [x1, y1, x2, y2]
                largest_label = int(cls)  # Category Index
                largest_conf = float(conf)  # Confidence

        return largest_box, largest_label, largest_conf, largest_area


    def draw_detections(self, image, boxes, labels, confs):
        """Draws bounding boxes and labels on the image.

        Args:
            image (numpy.ndarray): The input image.
            boxes (list): List of bounding box coordinates.
            labels (list): List of detected class indices.
            confs (list): List of detection confidence values.
        """
        for box, cls, conf in zip(boxes, labels, confs):
            x1, y1, x2, y2 = map(int, box)
            label = f"{self.model.names[cls]}: {conf:.2f}"  # Get Category Name

            # Draw the detection box
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Draw Labels
            cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


def main(args=None):
    """Main function to initialize and run the node."""
    
    rclpy.init(args=args)
    node = RoadSignDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
