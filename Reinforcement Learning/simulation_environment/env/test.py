import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image  
from sensor_msgs.msg import Imu
from cv_bridge import CvBridge
import cv2
import numpy as np
import math


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('sensor_subscriber')
        # 创建订阅器订阅 /color/image_raw 话题
        self.subscription_img = self.create_subscription(
            Image,
            '/color/image_raw',  # 替换为实际话题名称
            self.listener_callback,
            10  # 队列大小
        )
        self.subscription_img  # 防止被垃圾回收
        self.br = CvBridge()  # 初始化 CvBridge，用于消息和 OpenCV 图像的转换
        
        self.subscription_imu = self.create_subscription(
            Imu,
            '/imu_data',
            self.imu_callback,
            10

        )
        self.subscription_imu

    def imu_callback(self, msg):
        try:
            x, y, z, w =msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w
            
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
            self.get_logger().info(f"yaw_degree: {yaw_deg}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

         
            
            

    def listener_callback(self, msg):
        #self.get_logger().info(f"Received image with resolution: {msg.width}x{msg.height}")
        
        
        try:
            cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w = cv_image.shape[:2]
            top_crop = h // 4 
            cv_image = cv_image[top_crop:h, 0:w]
                # 绘制左下角的 ROI 框
            left_top_left = (50, 210)  # 左上角坐标
            left_bottom_right = (230, 480)  # 右下角坐标
            left_roi = cv_image[left_top_left[1]: left_bottom_right[1], left_top_left[0]: left_bottom_right[0]]
            cv2.rectangle(cv_image, left_top_left, left_bottom_right, (0, 255, 0), 2)  # 绿色框，线宽为2

            # 绘制右下角的 ROI 框
            right_top_left = (520, 130)  # 左上角坐标
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
            # self.get_logger().info(f"Green pixels in right ROI: {len(green_pixels)}")

            if len(red_pixels) > 0:
                mean_row = int(np.mean(red_pixels[:, 0]))
                mean_col = int(np.mean(red_pixels[:, 1]))
                #self.get_logger().info(f"Red centroid in left ROI => (row={mean_row}, col={mean_col})")

                # 把ROI内的质心投影回主图像坐标
                # ROI左上角在主图像里是 left_top_left=(x0,y0)
                # 注意: row=vertical方向 => y, col=horizontal => x
                centroid_x = left_top_left[0] + mean_col
                centroid_y = left_top_left[1] + mean_row

                # 在主图像上画一个小圆标记该质心
                cv2.circle(cv_image, (centroid_x, centroid_y), 5, (0, 255, 0), -1)
            else:
                self.get_logger().info("No red pixels found in left ROI.")
            #     rightmost_red = red_pixels[np.argmax(red_pixels[:, 1])]
            #     self.get_logger().info(f"Rightmost red pixel in left ROI: {rightmost_red}")
            # else:
            #     self.get_logger().info("No red pixels found in left ROI.")
            # 找到最右侧绿色像素的位置

            if len(green_pixels) > 0:
                mean_row_g = int(np.mean(green_pixels[:, 0]))
                mean_col_g = int(np.mean(green_pixels[:, 1]))
                #self.get_logger().info(f"Green centroid in right ROI => (row={mean_row_g}, col={mean_col_g})")

                # 投影回主图像坐标
                centroid_xg = right_top_left[0] + mean_col_g
                centroid_yg = right_top_left[1] + mean_row_g

                cv2.circle(cv_image, (centroid_xg, centroid_yg), 5, (0, 0, 255), -1)
            else:
                self.get_logger().info("No green pixels found in right ROI.")

            #     rightmost_green = green_pixels[np.argmax(green_pixels[:, 1])]
            #     self.get_logger().info(f"Rightmost green pixel in right ROI: {rightmost_green}")
            # else:
            #     self.get_logger().info("No green pixels found in right ROI.")

            # 使用 CvBridge 将 ROS 图像消息转换为 OpenCV 格式
            # 在窗口中显示图像
            cv2.imshow("Camera Image", cv_image)
            cv2.waitKey(1)  # 刷新窗口
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()  # 关闭所有 OpenCV 窗口

if __name__ == '__main__':
    main()

# import math

# # 四元数数据
# x = -0.9478
# y = -0.002089
# z = -0.1818
# w = 0.984809780588085962

# # 计算 roll (x 轴旋转)
# sinr_cosp = 2 * (w * x + y * z)
# cosr_cosp = 1 - 2 * (x * x + y * y)
# roll = math.atan2(sinr_cosp, cosr_cosp)

# # 计算 pitch (y 轴旋转)
# sinp = 2 * (w * y - z * x)
# # 限制 asin 输入的范围在 [-1, 1]
# if abs(sinp) >= 1:
#     pitch = math.copysign(math.pi / 2, sinp)
# else:
#     pitch = math.asin(sinp)

# # 计算 yaw (z 轴旋转)
# siny_cosp = 2 * (w * z + x * y)
# cosy_cosp = 1 - 2 * (y * y + z * z)
# yaw = math.atan2(siny_cosp, cosy_cosp)

# # 转换为角度
# roll_deg = roll * 180 / math.pi
# pitch_deg = pitch * 180 / math.pi
# yaw_deg = yaw * 180 / math.pi

# print("Roll: {:.6f}°".format(roll_deg))
# print("Pitch: {:.6f}°".format(pitch_deg))
# print("Yaw: {:.6f}°".format(yaw_deg))
