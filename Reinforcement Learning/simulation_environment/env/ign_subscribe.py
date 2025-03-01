#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

# 下面是 Ignition Transport 的 Python 接口示例引入
# 不同版本可能包名或导入方法不一样，请根据你实际环境修改
import ignition_transport_py  # 假设你的 Python 有这个模块
# Pose_V 的 Protobuf python stubs，你需要先生成 pose_v_pb2.py
import pose_v_pb2  # 这是你自己通过 protoc 生成的

class MixedIgnRosNode(Node):
    """
    1. 在 Ignition Transport 层面订阅 /world/car_world/dynamic_pose/info (Pose_V)
    2. 在 ROS 2 层面发布 /set_steering_level_msg (std_msgs/Float64)
    """

    def __init__(self):
        super().__init__('mixed_ign_ros_node')

        # =========== ROS 2 Publisher ============
        # 你已有的控制话题, 假设类型是 Float64
        self.steer_pub = self.create_publisher(Float64, '/set_steering_level_msg', 10)
        self.get_logger().info("ROS publisher set up for /set_steering_level_msg")

        # =========== Ignition Transport Node ============
        self.ign_node = ignition_transport_py.Node()
        
        # 要订阅的 Ignition 话题, 如果是 /world/car_world/pose/info 也可以写这里
        self.pose_topic = "/world/car_world/dynamic_pose/info"

        # 订阅 Pose_V 消息(二进制), 回调函数 ign_pose_callback
        subscribed_ok = self.ign_node.subscribe(self.pose_topic, self.ign_pose_callback)
        if not subscribed_ok:
            self.get_logger().error(f"Failed to subscribe to Ign topic [{self.pose_topic}]")
        else:
            self.get_logger().info(f"Subscribed to Ign topic: {self.pose_topic}")

        # 用于保存当前车辆姿态
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # =========== 定时器 =============
        # 因为 ignition_transport_py 可能没有自动 spin，需要我们自己调用 spin_once()
        # 同时也可以周期发布一个简单的转向命令作为测试
        timer_period = 1.0  # 每1秒执行一次
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.steer_val = 0.3

    def ign_pose_callback(self, msg_bytes: bytes):
        """
        Ignition Transport 收到 Pose_V 消息后的回调。
        msg_bytes 是 protobuf 的序列化字节流，需要用 pose_v_pb2.Pose_V() 反序列化。
        """
        pose_v = pose_v_pb2.Pose_V()
        pose_v.ParseFromString(msg_bytes)

        # 遍历所有对象的 pose
        for p in pose_v.pose:
            # p.name 一般是 "vehicle_blue" 或 "vehicle_blue::link_name"
            if "vehicle_blue" in p.name:
                # 提取位置
                self.current_x = p.position.x
                self.current_y = p.position.y

                # 提取四元数
                ox = p.orientation.x
                oy = p.orientation.y
                oz = p.orientation.z
                ow = p.orientation.w
                # 计算 yaw (Z轴旋转)
                siny_cosp = 2.0 * (ow * oz + ox * oy)
                cosy_cosp = 1.0 - 2.0 * (oy * oy + oz * oz)
                self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

                self.get_logger().info(
                    f"Ign Pose: name={p.name}, x={self.current_x:.2f}, "
                    f"y={self.current_y:.2f}, yaw={self.current_yaw:.2f}"
                )

    def timer_callback(self):
        """
        每秒调用一次，用来：
          - spin Ign Node (获取新消息)
          - 发布一个转向命令（演示）
        """

        # 让 ign_node 处理订阅队列
        self.ign_node.spin_once()

        # 发布一个简单的转向命令（仅演示）
        msg = Float64()
        msg.data = self.steer_val
        self.steer_pub.publish(msg)
        self.get_logger().info(f"Publish steering={self.steer_val:.2f}")

        # 下次换个方向
        self.steer_val = -self.steer_val


def main(args=None):
    rclpy.init(args=args)
    node = MixedIgnRosNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down by Ctrl+C")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
