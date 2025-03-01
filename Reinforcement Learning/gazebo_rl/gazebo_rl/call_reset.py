#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class ResetClient(Node):
    def __init__(self):
        super().__init__('reset_client')
        self.cli = self.create_client(Trigger, '/reset_env')
        # 等待服务出现
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /reset_env service...")

        self.get_logger().info("Service /reset_env is available. Ready to reset.")

    def do_reset(self):
        req = Trigger.Request()
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            success = future.result().success
            msg = future.result().message
            self.get_logger().info(f"Reset result: success={success}, message={msg}")
        else:
            self.get_logger().error("Service call failed or returned None")

def main(args=None):
    rclpy.init(args=args)
    node = ResetClient()
    node.do_reset()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
