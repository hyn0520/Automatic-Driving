import json
import math

# 从文件中读取 JSON 数据
def read_json_file(file_path):
    with open(file_path, 'r') as file:
        return json.load(file)

# 计算点的极坐标角度（相对于质心）
def calculate_polar_angle(point, center):
    dx = point["x_world"] - center["x_world"]
    dy = point["y_world"] - center["y_world"]
    return math.atan2(dy, dx)

# 计算所有点的质心
def calculate_center(points):
    x_sum = sum(point["x_world"] for point in points)
    y_sum = sum(point["y_world"] for point in points)
    return {"x_world": x_sum / len(points), "y_world": y_sum / len(points)}

# 按顺时针方向排序点
def sort_points_clockwise(points):
    center = calculate_center(points)
    points_with_angle = [
        {
            **point,
            "angle": calculate_polar_angle(point, center)
        } for point in points
    ]
    # 按角度降序排列（顺时针方向）
    sorted_points = sorted(points_with_angle, key=lambda p: p["angle"], reverse=True)
    return sorted_points

# 保存排序结果到文件
def save_to_file(data, file_path):
    with open(file_path, 'w') as file:
        json.dump(data, file, indent=4)

# 主函数
if __name__ == "__main__":
    # 输入和输出文件路径
    input_file = "/home/yinan/simulation_ws/src/simulation/centroids_world.json"  # 替换为你的 JSON 文件路径
    output_file = "/home/yinan/simulation_ws/src/simulation/sorted_points.json"  # 排序后的输出文件路径

    # 读取点数据
    points = read_json_file(input_file)

    # 排序点并编号
    sorted_points = sort_points_clockwise(points)
    for idx, point in enumerate(sorted_points):
        point["id"] = idx + 1

    # 保存结果
    save_to_file(sorted_points, output_file)

    print(f"已完成排序，结果保存到 {output_file}")
