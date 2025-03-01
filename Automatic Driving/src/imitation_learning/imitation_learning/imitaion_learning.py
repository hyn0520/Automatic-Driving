#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import torch
import torch.nn as nn
import numpy as np
import cv2

def process_img(img):
    """
    单通道自定义预处理
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (9, 9), 0)
    block_size = 61
    const_subtrahend = -70
    small_blurred = cv2.resize(blurred, None, fx=0.25, fy=0.25)
    small_binary = cv2.adaptiveThreshold(
        small_blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,
        block_size, const_subtrahend
    )
    binary = cv2.resize(small_binary, (blurred.shape[1], blurred.shape[0]))

    homography_data = [
        [-3.038122819655312, -8.672877526884868, 2377.9484334015165],
        [0.29213993514510084, -16.02172757573472, 2881.3325514309886],
        [0.0004933373163602723, -0.02681487260493437, 1.0]
    ]
    homography = np.array(homography_data, dtype=np.float64)
    binary_eagle = cv2.warpPerspective(binary, homography, (640, 480))

    resized = cv2.resize(binary_eagle, (240, 120))
    normalized = resized / 255.0
    final_image = np.expand_dims(normalized, axis=0).astype(np.float32)  # (1,120,240)
    return final_image

class PilotNetSingleChannel(nn.Module):
    def __init__(self):
        super(PilotNetSingleChannel, self).__init__()
        self.features = nn.Sequential(
            nn.Conv2d(1, 32, kernel_size=5, stride=2),
            nn.BatchNorm2d(32),
            nn.ReLU(inplace=True),
            nn.Conv2d(32, 48, kernel_size=5, stride=2),
            nn.BatchNorm2d(48),
            nn.ReLU(inplace=True),
            nn.Conv2d(48, 64, kernel_size=5, stride=2),
            nn.BatchNorm2d(64),
            nn.ReLU(inplace=True),
            nn.Conv2d(64, 96, kernel_size=3, stride=2),
            nn.BatchNorm2d(96),
            nn.ReLU(inplace=True),
            nn.Conv2d(96, 128, kernel_size=3, stride=1),
            nn.BatchNorm2d(128),
            nn.ReLU(inplace=True),
        )

        self.avgpool = nn.AdaptiveAvgPool2d((4, 4))
        self.regressor = nn.Sequential(
            nn.Linear(128 * 4 * 4, 256),
            nn.ReLU(inplace=True),
            nn.Linear(256, 64),
            nn.ReLU(inplace=True),
            nn.Linear(64, 1)
        )

    def forward(self, x):
        x = self.features(x)
        x = self.avgpool(x)
        x = torch.flatten(x, 1)
        x = self.regressor(x)
        return x

class PilotNetNodeSingleChannel(Node):
    def __init__(self):
        super().__init__('pilotnet_node_single_channel')
        self.subscription = self.create_subscription(
            Image,
            '/color/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()

        # 加载模型
        self.device = torch.device("cpu")  # or "cuda"
        self.model = PilotNetSingleChannel().to(self.device)
        model_path = "pilotnet_single_channel.pth"
        self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model.eval()

        self.steering_publisher = self.create_publisher(Int16, '/uc_bridge/set_steering', 10)
        self.motor_publisher = self.create_publisher(Int16, '/uc_bridge/set_motor_level_forward', 10)
        self.get_logger().info("SingleChannel PilotNetNode init done. Model loaded.")

    def image_callback(self, msg):
        steering_msg = Int16()
        motor_msg = Int16()

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # 裁掉上1/3

        # process_img -> (1,120,240)
        processed = process_img(cv_image)
        # 加 batch 维 -> (1,1,120,240)
        input_tensor = torch.from_numpy(processed).unsqueeze(0).to(self.device)

        with torch.no_grad():
            output = self.model(input_tensor)
            ang_pred_norm = output[0].item()

        steering_value = int(ang_pred_norm * 300.0)
        steering_msg.data = steering_value
        motor_msg.data = 300

        self.steering_publisher.publish(steering_msg)
        self.motor_publisher.publish(motor_msg)
        self.get_logger().info(f"[1CH] Predicted angle: {steering_value}")


def main(args=None):
    rclpy.init(args=args)
    node = PilotNetNodeSingleChannel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

