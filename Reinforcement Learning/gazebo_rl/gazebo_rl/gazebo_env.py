import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
import cv2
import gymnasium as gym
import numpy as np
import time
import json
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped

def find_nearest_point(vehicle_x, vehicle_y):
    filename = "/home/yinan/simulation_ws/src/gazebo_rl/gazebo_rl/sorted_points.json"
    with open (filename, 'r') as f:
        points = json.load(f)
    nearest_point = None
    min_distance = float("inf")
    for pt in points:
        dx = pt["x_world"] - vehicle_x
        dy = pt["y_world"] - vehicle_y
        id = pt["id"]
        dist = math.hypot(dx, dy)
        if dist < min_distance:
            min_distance = dist
            nearest_point = pt
            nearest_id = id
    # print("----------neareast_point----------:", nearest_point)
    def get_wrapped_id(current_id, offset, total_points=102):
        return ((current_id - 1 + offset) % total_points) + 1
    id_a = get_wrapped_id(nearest_id, -1)
    id_b = get_wrapped_id(nearest_id, -3)
    def get_point_by_id(target_id):
        # with open (filename, 'r') as f:
        #     points = json.load(f)
        for pt in points:
            if pt["id"] == target_id:
                return pt
        return None
    pt1 = get_point_by_id(id_a)
    pt2 = get_point_by_id(id_b)
    dx = pt2["x_world"] - pt1["x_world"]
    dy = pt2["y_world"] - pt1["y_world"]
    # print("----------pt1----------:", pt1)
    # print("----------pt2----------:", pt2)
    desired_angle = math.atan2(dy, dx)
    desired_angle = math.degrees(desired_angle)
    # print("----------desired_angle----------:", desired_angle)
    return desired_angle
    # return nearest_point, min_distance


class GazeboCarEnv(gym.Env):
    # def __init__(self, init_node=True):
    #     # super(GazeboCarEnv, self).__init__()
    #     super().__init__()
    #     if init_node:
    #        rclpy.init()
    #     self.node = rclpy.create_node('gazebo_car_env')
    def __init__(self, node_name='gazebo_car_env', init_node=False):
        super().__init__()
        self._init_node = init_node
        if self._init_node:
            # 仅当需要时初始化节点
            rclpy.init()
            self.node = rclpy.create_node(node_name)
        else:
            # 使用全局默认节点
            self.node = rclpy.create_node(node_name)
        self.reset_cli = self.node.create_client(Trigger, '/reset_env')
        # qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,depth=1)
        qos_profile = QoSProfile(depth=1)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.br = CvBridge()
        self.img_subscription = self.node.create_subscription(
            Image,
            '/color/image_raw',  
            self.img_callback,
            qos_profile
        )
        self.img_subscription
        self.pos_subscription = self.node.create_subscription(
            PoseStamped,
            '/vehicle_pose',
            self.pos_callback,
            10

        )
        self.pos_subscription
        self.red_count = 0
        self.green_count = 0
        self.white_count = 0
        self.last_image = None
        self.no_detect = 0
        
        self.pub_steer = self.node.create_publisher(
            Float64,
            '/set_steering_level_msg',
            10
        )
        self.pub_motor = self.node.create_publisher(
            Float64,
            '/set_motor_level_msg',
            10
        )
        
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32)
        self.observation_space = gym.spaces.Box(low=0, high=255, shape=(84,84,3), dtype=np.uint8)

        self.max_steer_angle = 400  # 假设弧度(约28.6度)
        self.time_step = 0.01  # 每个step等待0.1秒
        self.episode_steps = 0
        self.max_episode_steps = 1300  # 超过此数则 done
        self.current_steering = 0
        self.last_steering = 0
        self.heading_error = None
        self.last_heading_log_time = time.time()
        self.heading_log_interval = 0.5  # 1秒间隔
        self.consecutive_loss_frames = 0
        self.episode_sum = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
    
    def img_callback(self, msg):
        # self.get_logger().info(f"Received image with resolution: {msg.width}x{msg.height}")
        try:
            cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w = cv_image.shape[:2]
            top_crop = h // 4
            #self.node.get_logger().info(f"image shape: {cv_image.shape}")
            cv_image = cv_image[top_crop:h, 0:w]
            #self.node.get_logger().info(f"resized image shape: {cv_image.shape}")
            self.last_image = cv_image
                # 绘制左下角的 ROI 框
            left_top_left = (40, 190)  # 左上角坐标
            left_bottom_right = (180, 360)  # 右下角坐标
            left_roi = cv_image[left_top_left[1]: left_bottom_right[1], left_top_left[0]: left_bottom_right[0]]
            cv2.rectangle(cv_image, left_top_left, left_bottom_right, (0, 255, 0), 2)  # 绿色框，线宽为2

            mid_top_left = (120, 270)  # 左上角坐标
            mid_bottom_right = (580, 360)  # 右下角坐标
            mid_roi = cv_image[mid_top_left[1]: mid_bottom_right[1], mid_top_left[0]: mid_bottom_right[0]]
            cv2.rectangle(cv_image, mid_top_left, mid_bottom_right, (255, 255, 0), 2)  
            # 绘制右下角的 ROI 框
            right_top_left = (520, 130)  # 左上角坐标
            right_bottom_right = (640, 360)  # 右下角坐标
            right_roi = cv_image[right_top_left[1]:right_bottom_right[1], right_top_left[0]:right_bottom_right[0]]
            cv2.rectangle(cv_image, right_top_left, right_bottom_right, (0, 0, 255), 2)  # 红色框，线宽为2
            

            green_lower = np.array([30, 120, 30])
            green_upper = np.array([100, 255, 100])
            green_mask = cv2.inRange(right_roi, green_lower, green_upper)
            green_pixels = np.column_stack(np.where(green_mask > 0))
            cv2.putText(cv_image, f"Green: {len(green_pixels)}", (525, 150),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

            # 检测左下角框内的红色像素
            red_lower = np.array([0, 0, 120])  # 红色下界
            red_upper = np.array([80, 80, 255])  # 红色上界
            red_mask = cv2.inRange(left_roi, red_lower, red_upper)
            red_pixels = np.column_stack(np.where(red_mask > 0))  # 获取红色像素的位置
            
            # white_lower = np.array([200, 200, 200])  # 白色下界
            # white_upper = np.array([255, 255, 255])  # 白色上界
            # white_mask = cv2.inRange(mid_roi, white_lower, white_upper)
            hls_image = cv2.cvtColor(mid_roi, cv2.COLOR_BGR2HLS)
            # white: 高亮度 + 低饱和度
            lower_white = np.array([0, 200, 0], dtype=np.uint8)
            upper_white = np.array([180, 255, 80], dtype=np.uint8)
            white_mask = cv2.inRange(hls_image, lower_white, upper_white)

            white_pixels = np.column_stack(np.where(white_mask > 0))  # 获取白色像素的位置
            
            # current_time = time.time()
            # if current_time - self.last_heading_log_time > self.heading_log_interval:
            #     self.node.get_logger().info(f"Green pixels in right ROI: {len(green_pixels)}")
            #     if len(white_pixels) > 480:
            #         self.node.get_logger().info(f"white pixels in mid ROI: {len(white_pixels)}")
            #     self.last_heading_log_time = current_time
            
      
            # if len(red_pixels) > 0:
            #     rightmost_red = red_pixels[np.argmax(red_pixels[:, 1])]
            #     #self.get_logger().info(f"Rightmost red pixel in left ROI: {rightmost_red}")
            # else:
            #     self.node.get_logger().info("No red pixels found in left ROI.")

            # 找到最右侧绿色像素的位置
            # if len(green_pixels) > 0:
            #     rightmost_green = green_pixels[np.argmax(green_pixels[:, 1])]
            #     #self.get_logger().info(f"Rightmost green pixel in right ROI: {rightmost_green}")
            # else:
            #     self.node.get_logger().info("No green pixels found in right ROI.")
            self.red_count = len(red_pixels)
            self.green_count = len(green_pixels)
            self.white_count = len(white_pixels)-1426 + 80
            cv2.putText(cv_image, f"Red: {len(red_pixels)}", (45, 210),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
            cv2.putText(cv_image, f"White: {self.white_count}", (225, 280),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
            # 使用 CvBridge 将 ROS 图像消息转换为 OpenCV 格式
            # 在窗口中显示图像
            cv2.imshow("Camera Image", cv_image)
            cv2.waitKey(1)  # 刷新窗口
        except Exception as e:
            self.node.get_logger().error(f"Failed to convert image: {e}")
    
    def pos_callback(self, msg):
        try:
            pos_x = round(msg.pose.position.x, 1)
            pos_y = round(msg.pose.position.y, 1)
            pos_z = msg.pose.position.z
            x = msg.pose.orientation.x
            y = msg.pose.orientation.y
            z = msg.pose.orientation.z
            w = msg.pose.orientation.w
            # self.get_logger().info(f"x, y coordinate: {pos_x}, {pos_y}")
            sinr_cosp = 2 * (w * x + y * z)
            cosr_cosp = 1 - 2 * (x * x + y * y)
            roll = math.atan2(sinr_cosp, cosr_cosp)
            # 计算 pitch (y 轴旋转)
            sinp = 2 * (w * y - z * x)
            # 限制 asin 输入的范围在 [-1, 1]
            if abs(sinp) >= 1:
                pitch = math.copysign(math.pi / 2, sinp)
            else:
                pitch = math.asin(sinp)
            # 计算 yaw (z 轴旋转)
            siny_cosp = 2 * (w * z + x * y)
            cosy_cosp = 1 - 2 * (y * y + z * z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            # 转换为角度
            # roll_deg = roll * 180 / math.pi
            # pitch_deg = pitch * 180 / math.pi
            yaw_deg = round(yaw * 180 / math.pi, 1)
            # self.get_logger().info(f"yaw_degree: {yaw_deg}")

            desired_angle = find_nearest_point(pos_x, pos_y)
            heading_error = np.abs(yaw_deg - desired_angle)
            self.heading_error = (heading_error + 180) % 360 - 180
            current_time = time.time()
            # if current_time - self.last_heading_log_time > self.heading_log_interval:
            #     self.node.get_logger().info(f"Heading Error: {self.heading_error}")
            #     self.last_heading_log_time = current_time
            # self.node.get_logger().info(f"Heading Error:{self.heading_error}")
        except Exception as e:
            self.node.get_logger().error(f"No Position Value Available")

    def reset(self, *, seed=None, options=None):

        #self.node.get_logger().info("Waiting for /reset_env service-------------------------")
        motor_msg = Float64()
        motor_msg.data = 0.0
        self.pub_motor.publish(motor_msg)
        req = Trigger.Request()
        future = self.reset_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            success = future.result().success
            msg = future.result().message
            #self.node.get_logger().info(f"Called /reset_env: success={success}, message={msg}")
        else:
            self.node.get_logger().error("Service call /reset_env failed or returned None")
        self.episode_steps = 0
        start_time = time.time()
        wait_duration = 0.5
        while (time.time() - start_time) < wait_duration:
            rclpy.spin_once(self.node, timeout_sec=0.01)


        obs = self._get_observation()
        info = {}
        
        return obs, info

    def step(self, action):
        #这里的action来自策略网络
        steer_value = float(action[0] * self.max_steer_angle)
        self.current_steering = steer_value

        steer_msg = Float64()
        steer_msg.data = steer_value
        motor_msg = Float64()
        motor_msg.data = 1200.0 #600-700
        self.pub_steer.publish(steer_msg)
        self.pub_motor.publish(motor_msg)
        # 2) 等待一个仿真周期 (简单做法)
        #    如果你想更精细控制，可以用 /world/.../pause+step 方式
        start_time = time.time()
        while time.time() - start_time < self.time_step:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        
        # 3) 获取新的观测
        obs = self._get_observation()
        # 4) 计算奖励
        reward = self._compute_reward(obs, steer_value)
        self.episode_sum += reward
        self.last_steering = steer_value
        # 5) 判断是否结束
        self.episode_steps += 1
        done = False
        truncated = False
        # if self.red_count == 0 and self.green_count == 0:
        #     done = True
        # if self.red_count == 0 and self.green_count == 0 :
        #     self.consecutive_loss_frames += 1
        # else:
        #     self.consecutive_loss_frames = 0

        # if self.consecutive_loss_frames >= 10:
        #     done = True
            #self.node.get_logger().info("Condition met for 15 consecutive frames, done!")
        if abs(self.heading_error) > 40 :
            #self.node.get_logger().info(f"Heading Error: {self.heading_error}")
            done = True
        # 6) info
        info = {}
        if self.episode_steps >= self.max_episode_steps:
            truncated = True
        if done or truncated:
            self.node.get_logger().info(
            f"Episode Ended. steps={self.episode_steps}, total_reward={self.episode_sum:.2f}")
            self.episode_sum = 0.0
        return obs, reward, done, truncated, info

    def _get_observation(self):

        if not hasattr(self, 'last_image') or self.last_image is None:
        # 如果目前还没有收到任何图像，就返回一个空图或随机 
            obs = np.zeros((84, 84, 3), dtype=np.uint8)
            return obs

        h, w, _ = self.last_image.shape
        top_crop = h // 4
        cropped = self.last_image[top_crop:h, 0:w]
        # 2) 缩放到 84x84, 这样常见于Atari style输入
        obs = cv2.resize(cropped, (84, 84))
        
        return obs

    def _compute_reward(self, obs, steer):
        k1 = 0.005
        k2 = 0.12
        left_reward = 0
        right_reward = 0
        steering_reward = 0
        heading_reward = 0
        mid_reward = 0
        boundary_punishment = 0.0
        living_reward = 1.0
        distance = 100
        episode_reward = 0.0
        steering_penalty = 0.0
        # if self.red_count < 250:
        #    left_reward = -1.5 #-1.0
        # if 250 < self.red_count <500:
        #     left_reward = 1.0
        # if self.red_count > 500:
        #     left_reward = 2.5 #2.0
        # if self.green_count > 50:
        #    right_reward = 1.5

        # if (self.red_count != 0 or self.white_count > 0) and self.green_count < 250:
        #     right_reward = -2.5 #-1.0
        # if 250 < self.green_count <450:
        #     right_reward = -1.0
        if self.green_count > 300:
            right_reward = 2.0 #2.0
        if self.red_count > 0:
           left_reward = 1.5
        if self.white_count > 0:#480
           mid_reward = -8.0#-2.0

        if np.abs(self.current_steering - self.last_steering) > 600:
           steering_reward = -5.0
        if np.abs(self.current_steering) > 750 :
           steering_penalty = -2.0
        if np.abs(self.heading_error) < 12:
           heading_reward = 1.5 * math.exp(-k1 * (np.abs(self.heading_error) ** 2))
        if np.abs(self.heading_error) > 12:
           heading_reward = -1.5 * (math.exp(k2 * abs(np.abs(self.heading_error) - 12)) - 1)
        if self.red_count == 0 and self.green_count == 0:
            boundary_punishment = -1.0
        else :
            boundary_punishment = 0.0
        if self.episode_steps == 200:
            episode_reward = 100
        if self.episode_steps == 400:
            episode_reward = 200
        if self.episode_steps == 600:
            episode_reward = 300
        if self.episode_steps == 800:
            episode_reward = 400
        if self.episode_steps == 750:
            episode_reward = 500
        if self.episode_steps == 1000:
            episode_reward = 600
        

        
        
        reward = left_reward + right_reward + mid_reward + steering_reward + heading_reward + boundary_punishment + living_reward + episode_reward + steering_penalty
        # current_time = time.time()
        # if current_time - self.last_heading_log_time > self.heading_log_interval:
        #     self.node.get_logger().info(f"current_reward: {self.heading_error}")
        #     self.last_heading_log_time = current_time
        return reward

    def close(self):
        # rclpy.shutdown()
        self.node.destroy_node()
        return

