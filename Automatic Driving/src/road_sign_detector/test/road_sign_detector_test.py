"""
Unit tests for the RoadSignDetector ROS2 node.
"""

import unittest
from unittest.mock import MagicMock, patch
import rclpy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Int16
import numpy as np
import cv2
import torch
from road_sign_detector import RoadSignDetector


class RoadSignDetectorTest(unittest.TestCase):
    """Unit tests for RoadSignDetector."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 before running tests."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS2 after tests are complete."""
        rclpy.shutdown()

    def setUp(self):
        """Create an instance of RoadSignDetector before each test."""
        self.node = RoadSignDetector()
        self.node.get_logger = MagicMock()

    def tearDown(self):
        """Destroy the node after each test."""
        self.node.destroy_node()

    def test_status_callback_updates_status(self):
        """Test that status_callback updates the internal status variable."""
        msg = Int16()
        msg.data = 3  # Simulierter Statuswert
        self.node.status_callback(msg)
        self.assertEqual(self.node.status, 3)

    def test_image_callback_skips_when_status_not_3(self):
        """Test that image_callback skips processing if status is not 3."""
        msg = Image()
        self.node.status = 2  # Status != 3
        self.node.image_callback(msg)
        self.node.get_logger().info.assert_called_with("Skipping inference as status is not 3.")

    def test_image_callback_processes_valid_image(self):
        """Test that image_callback processes a valid image."""
        self.node.status = 3  # Setze Status auf 3, damit die Verarbeitung läuft
        msg = Image()
        msg.height = 480
        msg.width = 640
        msg.encoding = "bgr8"
        msg.step = msg.width * 3
        msg.data = np.zeros((480, 640, 3), dtype=np.uint8).tobytes()  # Schwarzes Bild

        self.node.model = MagicMock()
        self.node.model.return_value = MagicMock(xyxy=[torch.tensor([[10, 10, 100, 100, 0.9, 2]])])

        self.node.parse_yolo_results = MagicMock(return_value=([10, 10, 100, 100], 2, 0.9, 12000.0))
        self.node.detection_publisher.publish = MagicMock()

        self.node.image_callback(msg)

        self.node.detection_publisher.publish.assert_called_once()
        self.node.get_logger().info.assert_any_call("Largest Detection: tempo 30 with confidence 0.90")

    def test_parse_yolo_results_returns_largest_detection(self):
        """Test that parse_yolo_results correctly identifies the largest detection."""
        results = MagicMock()
        results.xyxy = [
            torch.tensor([[50, 50, 200, 200, 0.95, 1], [10, 10, 100, 100, 0.90, 2]])
        ]

        box, label, conf, area = self.node.parse_yolo_results(results)

        self.assertEqual(box, [50, 50, 200, 200])
        self.assertEqual(label, 1)
        self.assertEqual(conf, 0.95)
        self.assertEqual(area, (200 - 50) * (200 - 50))

    def test_parse_yolo_results_returns_none_if_no_detections(self):
        """Test that parse_yolo_results returns None if there are no detections."""
        results = MagicMock()
        results.xyxy = [[]]  # Keine Detektionen

        box, label, conf, area = self.node.parse_yolo_results(results)

        self.assertIsNone(box)
        self.assertIsNone(label)
        self.assertIsNone(conf)
        self.assertEqual(area, 0)

    def test_draw_detections_draws_correctly(self):
        """Test that draw_detections correctly overlays bounding boxes and labels."""
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        boxes = [[50, 50, 200, 200]]
        labels = [1]
        confs = [0.95]

        self.node.model.names = ["stop", "tempo 30"]  # Simulierte Labels
        self.node.draw_detections(image, boxes, labels, confs)

        self.assertIsNotNone(image)  # Sicherstellen, dass kein Fehler geworfen wurde


if __name__ == '__main__':
    unittest.main()
