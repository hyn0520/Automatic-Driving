"""
Unit tests for the SteeringNode in imitation_learning.py.
"""

import unittest
from unittest.mock import MagicMock, patch
import rclpy
from sensor_msgs.msg import Image, Range
from std_msgs.msg import Int16
import numpy as np
import cv2
import torch
from imitation_learning import SteeringNode


class SteeringNodeTest(unittest.TestCase):
    """Unit tests for SteeringNode."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 before running tests."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS2 after tests are complete."""
        rclpy.shutdown()

    def setUp(self):
        """Create an instance of SteeringNode before each test."""
        self.node = SteeringNode()
        self.node.get_logger = MagicMock()  # Verhindert echte Logger-Ausgabe

    def tearDown(self):
        """Destroy the node after each test."""
        self.node.destroy_node()

    def test_status_callback_updates_status(self):
        """Test that status_callback updates the internal status variable."""
        msg = Int16()
        msg.data = 4  # Simulierter Statuswert
        self.node.status_callback(msg)
        self.assertEqual(self.node.status, 4)

    def test_distance_callback_detects_hinder(self):
        """Test that distance_callback correctly detects an obstacle."""
        msg = Range()
        msg.range = 0.5  # Simulierter Wert unter der Hindernisschwelle
        self.node.distance_callback(msg)
        self.assertTrue(self.node.hinder)

    def test_distance_callback_no_hinder_when_far(self):
        """Test that distance_callback does not trigger hinder when far enough away."""
        msg = Range()
        msg.range = 1.5  # Simulierter Wert über der Hindernisschwelle
        self.node.distance_callback(msg)
        self.assertFalse(self.node.hinder)

    def test_preprocess_image_transforms_correctly(self):
        """Test that preprocess_image correctly processes an image."""
        test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        processed = self.node.preprocess_image(test_image)
        
        self.assertEqual(processed.shape, (1, 224, 224), "Processed image should have shape (1, 224, 224)")
        self.assertTrue(np.max(processed) <= 1.0 and np.min(processed) >= 0.0, "Processed image should be normalized")

    @patch.object(torch.nn.Module, "forward", return_value=torch.tensor([[100.0]]))
    def test_image_callback_correctly_processes_image(self, mock_forward):
        """Test that image_callback correctly processes an image and publishes commands."""
        self.node.status = 5  # Status, in dem das Fahrzeug sich bewegt

        msg = Image()
        msg.height = 480
        msg.width = 640
        msg.encoding = "bgr8"
        msg.step = msg.width * 3
        msg.data = np.zeros((480, 640, 3), dtype=np.uint8).tobytes()  # Schwarzes Bild

        self.node.steering_publisher.publish = MagicMock()
        self.node.motor_publisher.publish = MagicMock()

        self.node.image_callback(msg)

        self.node.steering_publisher.publish.assert_called_once()
        self.node.motor_publisher.publish.assert_called_once()
        self.node.get_logger().info.assert_any_call("Path clear. Moving forward.")

    def test_error_callback_stores_error_value(self):
        """Test that error_callback correctly stores error values."""
        msg = Int16()
        msg.data = 15
        self.node.error_callback(msg)
        self.assertEqual(self.node.error, 15)


if __name__ == '__main__':
    unittest.main()
