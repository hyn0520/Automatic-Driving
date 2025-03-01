import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
import cv2
import gym
import numpy as np
import time
import json

class GazeboCarEnv(gym.Env):
    def __init__(self):
        super(GazeboCarEnv, self).__init__()

        rclpy.init()
        self.node = rclpy.create_node('gazebo_car_env')
        self.reset_cli = self.node.creat_client(Trigger, '/reset_env')
        self.subscription
        self.br = CvBridge()
        self.subscription = self.node.create_subscription(
            Image,
            '/color/image_raw',  
            self.listener_callback,
            10  
        )
        self.red_count = 0
        self.green_count = 0
        self.no_detect = 0
        
        self.pub_steer = self.node.create_publisher(
            Float64,
            '/set_steering_level_msg',
            10
        )

        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32)
     
        self.observation_space = gym.spaces.Box(
            low=np.array([-1000.0, -1000.0, -1.57, -100.0, -3.14], dtype=np.float32),
            high=np.array([ 1000.0,  1000.0,  1.57,  100.0,  3.14], dtype=np.float32),
            shape=(5,),
            dtype=np.float32
        )

        self.max_steer_angle = 0.5  # 假设弧度(约28.6度)
        self.time_step = 0.1  # 每个step等待0.1秒
        self.episode_steps = 0
        self.max_episode_steps = 1000  # 超过此数则 done
    
    def callback(self, msg):
            self.get_logger().info(f"Received image with resolution: {msg.width}x{msg.height}")
            
            try:
                cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                    # 绘制左下角的 ROI 框
                left_top_left = (0, 330)  # 左上角坐标
                left_bottom_right = (180, 480)  # 右下角坐标
                left_roi = cv_image[left_top_left[1]: left_bottom_right[1], left_top_left[0]: left_bottom_right[0]]
                cv2.rectangle(cv_image, left_top_left, left_bottom_right, (0, 255, 0), 2)  # 绿色框，线宽为2

                # 绘制右下角的 ROI 框
                right_top_left = (520, 250)  # 左上角坐标
                right_bottom_right = (640, 480)  # 右下角坐标
                right_roi = cv_image[right_top_left[1]:right_bottom_right[1], right_top_left[0]:right_bottom_right[0]]
                cv2.rectangle(cv_image, right_top_left, right_bottom_right, (0, 0, 255), 2)  # 红色框，线宽为2

                green_lower = np.array([30, 120, 30])
                green_upper = np.array([100, 255, 100])
                green_mask = cv2.inRange(right_roi, green_lower, green_upper)
                green_pixels = np.column_stack(np.where(green_mask > 0))

                # 检测左下角框内的红色像素
                red_lower = np.array([0, 0, 120])  # 红色下界
                red_upper = np.array([80, 80, 255])  # 红色上界
                red_mask = cv2.inRange(left_roi, red_lower, red_upper)
                red_pixels = np.column_stack(np.where(red_mask > 0))  # 获取红色像素的位置

                # self.get_logger().info(f"Red pixels in left ROI: {len(red_pixels)}")
                self.get_logger().info(f"Green pixels in right ROI: {len(green_pixels)}")

                if len(red_pixels) > 0:
                    rightmost_red = red_pixels[np.argmax(red_pixels[:, 1])]
                    self.get_logger().info(f"Rightmost red pixel in left ROI: {rightmost_red}")
                else:
                    self.get_logger().info("No red pixels found in left ROI.")

                # 找到最右侧绿色像素的位置
                if len(green_pixels) > 0:
                    rightmost_green = green_pixels[np.argmax(green_pixels[:, 1])]
                    self.get_logger().info(f"Rightmost green pixel in right ROI: {rightmost_green}")
                else:
                    self.get_logger().info("No green pixels found in right ROI.")

                # 使用 CvBridge 将 ROS 图像消息转换为 OpenCV 格式
                # 在窗口中显示图像
                cv2.imshow("Camera Image", cv_image)
                cv2.waitKey(1)  # 刷新窗口
            except Exception as e:
                self.get_logger().error(f"Failed to convert image: {e}")
        
    def reset(self):
        self.get_logger().info("Waiting for /reset_env service-------------------------")
        req = Trigger.Request()
        future = self.reset_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            success = future.result().success
            msg = future.result().message
            self.node.get_logger().info(f"Called /reset_env: success={success}, message={msg}")
        else:
            self.node.get_logger().error("Service call /reset_env failed or returned None")
        self.episode_steps = 0
        start_time = time.time()
        wait_duration = 0.5
        while (time.time() - start_time) < wait_duration:
            rclpy.spin_once(self.node, timeout_sec=0.01)


        obs = self._get_observation()
        return obs



def step(self, action):
        #这里的action来自策略网络？
        steer_value = float(action[0] * self.max_steer_angle)
        msg = Float64()
        msg.data = steer_value
        self.pub_steer.publish(msg)

        # 2) 等待一个仿真周期 (简单做法)
        #    如果你想更精细控制，可以用 /world/.../pause+step 方式
        start_time = time.time()
        while time.time() - start_time < self.time_step:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        
        # 3) 获取新的观测
        obs = self._get_observation()

        # 4) 计算奖励
        reward = self._compute_reward(obs, steer_value)

        # 5) 判断是否结束
        self.episode_steps += 1
        done = False
        if self.episode_steps >= self.max_episode_steps:
            done = True
        # 也可根据越界判断 done

        # 6) info
        info = {}

        return obs, reward, done, info

    def _get_observation(self):
        

    def _compute_reward(self, obs, steer):
        

    def close(self):
        rclpy.shutdown()
        return

