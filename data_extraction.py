import sqlite3
import os
import cv2
import pandas as pd
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from cv_bridge import CvBridge
import bisect


# 提取单个 bag 的数据
def extract_single_ros2_bag(bag_path, output_dir):
    bridge = CvBridge()
    #connection = sqlite3.connect(os.path.join(bag_path, "metadata.db3"))
    connection = sqlite3.connect(bag_path)
    cursor = connection.cursor()
    #SELECT topic_id, COUNT(*) FROM messages GROUP BY topic_id;
    cursor.execute("SELECT topic_id, COUNT(*) FROM messages GROUP BY topic_id")
    topic_counts = cursor.fetchall()
    print("topic_id distribution in messages:", topic_counts)

    # 获取所有话题信息
    cursor.execute("SELECT id, name, type FROM topics")
    topics = cursor.fetchall()
    topic_dict = {name: (id, type) for id, name, type in topics}
    print(topic_dict)

    # 确定目标话题
    image_topic = "/color/image_raw"
    #motor_topic = "/uc_bridge/set_motor_level_forward"
    steering_topic = "/uc_bridge/set_steering"

    if image_topic not in topic_dict or  steering_topic not in topic_dict:
        raise ValueError("One or more required topics are not in the bag file.")

    image_topic_id = topic_dict[image_topic][0]
    #motor_topic_id = topic_dict[motor_topic][0]
    steering_topic_id = topic_dict[steering_topic][0]
    #image_topic_id = 1
    #steering_topic_id = 2
    # 创建输出目录
    os.makedirs(output_dir, exist_ok=True)

    # 数据存储
    camera_data = []
    motor_data = []
    steering_data = []

    # 提取消息
    cursor.execute("SELECT timestamp, topic_id, data FROM messages")
    for timestamp, topic_id, data in cursor:
        if topic_id == image_topic_id:
            img_msg = deserialize_message(data, get_message(topic_dict[image_topic][1]))
            cv_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
            image_path = os.path.join(output_dir, f"{timestamp}.jpg")
            cv2.imwrite(image_path, cv_img)
            camera_data.append({"timestamp": timestamp / 1e9, "image_path": image_path})
        # elif topic_id == motor_topic_id:
        #     motor_msg = deserialize_message(data, get_message(topic_dict[motor_topic][1]))
        #     motor_data.append({"timestamp": timestamp / 1e9, "linear_velocity": motor_msg.data})
        elif topic_id == steering_topic_id:
            steering_msg = deserialize_message(data, get_message(topic_dict[steering_topic][1]))
            steering_data.append({"timestamp": timestamp / 1e9, "angular_velocity": steering_msg.data})

    # 转换为 DataFrame
    camera_df = pd.DataFrame(camera_data)
    #motor_df = pd.DataFrame(motor_data)
    steering_df = pd.DataFrame(steering_data)

    return camera_df, steering_df

# 提取多个 bag 的数据并合并
def extract_and_merge_ros2_bags(bag_paths, output_dir):
    all_camera_data = []
    all_motor_data = []
    all_steering_data = []

    # 遍历所有 bag 路径
    for i, bag_path in enumerate(bag_paths):
        print(f"Processing bag {i+1}/{len(bag_paths)}: {bag_path}")
        # 使用单个 bag 提取函数
        camera_df,  steering_df = extract_single_ros2_bag(bag_path, output_dir)
        all_camera_data.append(camera_df)
        #all_motor_data.append(motor_df)
        all_steering_data.append(steering_df)

    # 合并所有的 DataFrame
    merged_camera_df = pd.concat(all_camera_data, ignore_index=True)
    #merged_motor_df = pd.concat(all_motor_data, ignore_index=True)
    merged_steering_df = pd.concat(all_steering_data, ignore_index=True)

    return merged_camera_df, merged_steering_df


# 同步数据
def synchronize_data(camera_df, steering_df):
    synchronized_data = []

    for _, row in camera_df.iterrows():
        # 查找最近的未来控制量（速度和转向）
        #future_motor = motor_df[motor_df['timestamp'] >= row['timestamp']]
        future_steering = steering_df[steering_df['timestamp'] >= row['timestamp']]

        if not future_steering.empty:
            #closest_motor = future_motor.iloc[0]
            closest_steering = future_steering.iloc[0]
            synchronized_data.append({
                "timestamp": row['timestamp'],
                "image_path": row['image_path'],
                
                "angular_velocity": closest_steering['angular_velocity']
            })
        else:
            # 如果没有未来控制量，则填充空值
            synchronized_data.append({
                "timestamp": row['timestamp'],
                "image_path": row['image_path'],
                
                "angular_velocity": None
            })

    return pd.DataFrame(synchronized_data)



def main():
    # 指定多个 ros2 bag 文件的路径
    bag_paths = [
        "/home/yinan/PSAF/Behaviour Clone/my_data_1000/my_data_0.db3"
    ]
    output_dir = "/home/yinan/PSAF/Behaviour Clone/output_1000"  # 数据保存目录

    # 提取并合并数据
    camera_df, steering_df = extract_and_merge_ros2_bags(bag_paths, output_dir)

    # 同步数据
    synchronized_df = synchronize_data(camera_df, steering_df)

    # 保存到 CSV
    synchronized_df.to_csv(os.path.join(output_dir, "synchronized_data.csv"), index=False)
    print(f"Synchronized data saved to {os.path.join(output_dir, 'synchronized_data.csv')}")
    
if __name__ == "__main__":
    main()

# # 提取单个 bag 的数据
# def extract_single_ros2_bag(bag_path, output_dir):
#     bridge = CvBridge()
#     connection = sqlite3.connect(os.path.join(bag_path, "metadata.db3"))
#     cursor = connection.cursor()

#     # 获取所有话题信息
#     cursor.execute("SELECT id, name, type FROM topics")
#     topics = cursor.fetchall()
#     topic_dict = {name: (id, type) for id, name, type in topics}
#     print(topic_dict)

#     # 确定目标话题
#     image_topic = "/color/image_raw"
#     motor_topic = "/uc_bridge/set_motor_level_forward"
#     steering_topic = "/uc_bridge/set_steering"

#     if image_topic not in topic_dict or motor_topic not in topic_dict or steering_topic not in topic_dict:
#         raise ValueError("One or more required topics are not in the bag file.")

#     image_topic_id = topic_dict[image_topic][0]
#     motor_topic_id = topic_dict[motor_topic][0]
#     steering_topic_id = topic_dict[steering_topic][0]

#     # 创建输出目录
#     os.makedirs(output_dir, exist_ok=True)

#     # 数据存储
#     camera_data = []
#     motor_data = []
#     steering_data = []

#     # 提取消息
#     cursor.execute("SELECT timestamp, topic_id, data FROM messages")
#     for timestamp, topic_id, data in cursor:
#         if topic_id == image_topic_id:
#             img_msg = deserialize_message(data, get_message(topic_dict[image_topic][1]))
#             cv_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
#             image_path = os.path.join(output_dir, f"{timestamp}.jpg")
#             cv2.imwrite(image_path, cv_img)
#             camera_data.append({"timestamp": timestamp / 1e9, "image_path": image_path})
#         elif topic_id == motor_topic_id:
#             motor_msg = deserialize_message(data, get_message(topic_dict[motor_topic][1]))
#             motor_data.append({"timestamp": timestamp / 1e9, "linear_velocity": motor_msg.data})
#         elif topic_id == steering_topic_id:
#             steering_msg = deserialize_message(data, get_message(topic_dict[steering_topic][1]))
#             steering_data.append({"timestamp": timestamp / 1e9, "angular_velocity": steering_msg.data})

#     # 转换为 DataFrame
#     camera_df = pd.DataFrame(camera_data)
#     motor_df = pd.DataFrame(motor_data)
#     steering_df = pd.DataFrame(steering_data)

#     return camera_df, motor_df, steering_df

# # 提取多个 bag 的数据并合并
# def extract_and_merge_ros2_bags(bag_paths, output_dir):
#     all_camera_data = []
#     all_motor_data = []
#     all_steering_data = []

#     # 遍历所有 bag 路径
#     for i, bag_path in enumerate(bag_paths):
#         print(f"Processing bag {i+1}/{len(bag_paths)}: {bag_path}")
#         # 使用单个 bag 提取函数
#         camera_df, motor_df, steering_df = extract_single_ros2_bag(bag_path, output_dir)
#         all_camera_data.append(camera_df)
#         all_motor_data.append(motor_df)
#         all_steering_data.append(steering_df)

#     # 合并所有的 DataFrame
#     merged_camera_df = pd.concat(all_camera_data, ignore_index=True)
#     merged_motor_df = pd.concat(all_motor_data, ignore_index=True)
#     merged_steering_df = pd.concat(all_steering_data, ignore_index=True)

#     return merged_camera_df, merged_motor_df, merged_steering_df


# # 同步数据
# def synchronize_data(camera_df, motor_df, steering_df):
#     synchronized_data = []

#     for _, row in camera_df.iterrows():
#         # 查找最近的未来控制量（速度和转向）
#         future_motor = motor_df[motor_df['timestamp'] >= row['timestamp']]
#         future_steering = steering_df[steering_df['timestamp'] >= row['timestamp']]

#         if not future_motor.empty and not future_steering.empty:
#             closest_motor = future_motor.iloc[0]
#             closest_steering = future_steering.iloc[0]
#             synchronized_data.append({
#                 "timestamp": row['timestamp'],
#                 "image_path": row['image_path'],
#                 "linear_velocity": closest_motor['linear_velocity'],
#                 "angular_velocity": closest_steering['angular_velocity']
#             })
#         else:
#             # 如果没有未来控制量，则填充空值
#             synchronized_data.append({
#                 "timestamp": row['timestamp'],
#                 "image_path": row['image_path'],
#                 "linear_velocity": None,
#                 "angular_velocity": None
#             })

#     return pd.DataFrame(synchronized_data)



# def main():
#     # 指定多个 ros2 bag 文件的路径
#     bag_paths = [
#         "/home/yinan/PSAF/Behaviour Clone/my_data/my_data_0.db3"
#     ]
#     output_dir = "/home/yinan/PSAF/Behaviour Clone/my_data/output_data"  # 数据保存目录

#     # 提取并合并数据
#     camera_df, motor_df, steering_df = extract_and_merge_ros2_bags(bag_paths, output_dir)

#     # 同步数据
#     synchronized_df = synchronize_data(camera_df, motor_df, steering_df)

#     # 保存到 CSV
#     synchronized_df.to_csv(os.path.join(output_dir, "synchronized_data.csv"), index=False)
#     print(f"Synchronized data saved to {os.path.join(output_dir, 'synchronized_data.csv')}")
    
# if __name__ == "__main__":
#     main()

