import json
import math

def pixel_to_world(px, py, img_width, img_height):
    """
    将 PNG 像素坐标 (px, py) 转成 Gazebo 世界坐标 (Xw, Yw).
    根据上文假设:
      - 纹理坐标 (S, T) = (px/img_width, 1 - py/img_height) [如需要翻转Y]
      - 平面局部坐标 (xl, yl) = (2S - 1, 2T - 1)
      - DAE 中缩放 scaleX=2.38, scaleY=3.8
      - 模型在 SDF 中 pose: 平移(2, 2.3), yaw=1.57
    """

    # 1) 计算纹理坐标 (S, T)
    #   如果你的 PNG (0,0) 在左上，则 T 要做 1 - (py / H)，
    #   或者你可以直接 T = py/H (看具体贴图方式).
    S = px / float(img_width)
    T = 1 - (py / float(img_height))  # 假定需要垂直翻转

    # 2) 局部平面坐标 x_local, y_local ∈ [-1, 1]
    x_local = 2.0 * S - 1.0
    y_local = 2.0 * T - 1.0

    # 3) 缩放
    scale_x = 2.38
    scale_y = 3.8
    Xp = scale_x * x_local
    Yp = scale_y * y_local

    # 4) 旋转 + 平移 (yaw=1.57 约等于 90°)
    #    旋转矩阵 Rz(1.57) 大约是 [[0, -1],[1, 0]]
    yaw = 1.57
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    Xr = Xp * cos_yaw - Yp * sin_yaw
    Yr = Xp * sin_yaw + Yp * cos_yaw

    # 平移
    Tx = 2.0
    Ty = 2.3
    Xw = Tx + Xr
    Yw = Ty + Yr

    return (Xw, Yw)

def main():
    # 假定你的 JSON 文件中是一个列表，每个元素 { "x": ..., "y": ... }
    # 例如 "centroids.json"
    json_file = "/home/yinan/simulation_ws/src/simulation/data/centroids.json"

    # 假设你的 PNG 尺寸 (如果是 2000x1500 之类，修改此处为实际宽高)
    IMG_WIDTH = 2275
    IMG_HEIGHT = 3553

    with open(json_file, "r", encoding="utf-8") as f:
        data = json.load(f)

    results = []
    for item in data:
        px = item["x"]
        py = item["y"]
        Xw, Yw = pixel_to_world(px, py, IMG_WIDTH, IMG_HEIGHT)
        results.append({"x_world": Xw, "y_world": Yw})

    # 你可以把 results 存回一个新 JSON，或者直接打印
    out_file = "/home/yinan/simulation_ws/src/simulation/centroids_world.json"
    with open(out_file, "w", encoding="utf-8") as f_out:
        json.dump(results, f_out, indent=2, ensure_ascii=False)

    print(f"转换完成，已输出到 {out_file}")

if __name__ == "__main__":
    main()

# # 读取 JSON 文件中的质心点
# def load_centroids_from_json(json_path):
#     with open(json_path, "r") as json_file:
#         centroids = json.load(json_file)
#     return centroids

# # 坐标转换函数
# def convert_to_world_coordinates(centroids, image_path, S_x, S_y, pose, theta):
#     """
#     将 JSON 文件中的质心点从图像像素坐标转换到 Gazebo 世界坐标。
#     """
#     # 加载图片，获取图像的宽和高
#     image = cv2.imread(image_path)
#     if image is None:
#         raise FileNotFoundError(f"图像路径无效: {image_path}")
#     H, W = image.shape[:2]  # 图像高度和宽度

#     # 纹理坐标计算
#     texture_coords = [(point["x"] / W, point["y"] / H) for point in centroids]

#     # 网格坐标计算
#     x_min, y_min = -S_x / 2, -S_y / 2  # 网格原点居中
#     mesh_coords = [(s * S_x + x_min, t * S_y + y_min) for (s, t) in texture_coords]

#     # SDF 中的位姿和旋转角度
#     pose_x, pose_y, pose_z = pose
#     R = np.array([
#         [math.cos(theta), -math.sin(theta)],
#         [math.sin(theta), math.cos(theta)]
#     ])

#     # 转换到世界坐标
#     world_coords = [(R @ np.array([x, y]) + np.array([pose_x, pose_y])).tolist() for (x, y) in mesh_coords]
#     return world_coords

# if __name__ == "__main__":
#     # JSON 文件路径
#     json_path = "D:/AF/simulation-master/shaped_track_2/centroids.json"
#     image_path = "D:/AF/simulation-master/shaped_track_2/shaped_track_2.png"

#     # 读取质心点
#     centroids = load_centroids_from_json(json_path)

#     # 纹理坐标和网格坐标参数
#     S_x = 2.38  # 网格 X 方向大小
#     S_y = 3.8   # 网格 Y 方向大小
#     pose = (2, 2.3, 0)  # SDF 文件中的位姿
#     theta = 1.57  # 弧度（旋转角度）

#     # 进行坐标转换
#     world_coords = convert_to_world_coordinates(centroids, image_path, S_x, S_y, pose, theta)

#     # 打印世界坐标结果
#     print("世界坐标：", world_coords)

#     # 将世界坐标保存为 JSON 文件
#     output_json_path = "D:/AF/simulation-master/shaped_track_2/world_coordinates.json"
#     with open(output_json_path, "w") as json_file:
#         json.dump(world_coords, json_file, indent=4)
#     print(f"世界坐标已保存到: {output_json_path}")

# image = cv2.imread("D:/AF/simulation-master/shaped_track_2/shaped_track_2.png")
# H, W = image.shape[:2]  # 图像高度和宽度
#
# texture_coords = [(u / W, v / H) for (v, u) in points]
# print("纹理坐标：", texture_coords)
# S_x = 2.38  # 网格 X 方向大小
# S_y = 3.8   # 网格 Y 方向大小
#
# x_min, y_min = -S_x / 2, -S_y / 2  # 网格原点居中
# mesh_coords = [(s * S_x + x_min, t * S_y + y_min) for (s, t) in texture_coords]
# print("网格坐标：", mesh_coords)
#
# # SDF 中的 pose 信息
# pose_x, pose_y, pose_z = 2, 2.3, 0
# theta = 1.57  # 弧度
#
# # 构造旋转矩阵
# R = np.array([
#     [math.cos(theta), -math.sin(theta)],
#     [math.sin(theta), math.cos(theta)]
# ])
#
# # 转换到世界坐标
# world_coords = [(R @ np.array([x, y]) + np.array([pose_x, pose_y])).tolist() for (x, y) in mesh_coords]
# print("世界坐标：", world_coords)
