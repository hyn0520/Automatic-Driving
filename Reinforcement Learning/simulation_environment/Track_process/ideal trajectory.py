import cv2
import numpy as np
import json

def calculate_contour_direction(contour):
    """
    计算轮廓的延伸方向，返回质心和主方向向量。
    """
    contour_points = contour.reshape(-1, 2).astype(np.float32)

    # 计算质心
    M = cv2.moments(contour)
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    centroid = np.array([cx, cy])

    # 使用PCA计算主方向
    eigenvectors = cv2.PCACompute2(contour_points, mean=np.empty(0))
    main_direction_2d = eigenvectors[0]
    main_direction = main_direction_2d.flatten()
    # 在 flatten 后，对向量做单位化
    norm = np.linalg.norm(main_direction)
    if norm > 1e-6:
        main_direction = main_direction / norm

    return centroid, main_direction

def generate_ideal_points(centroid, main_direction, step_length=50, num_points=3):
    """
    根据主方向和质心生成理想的估计点。
    """
    if len(main_direction) != 2:
        raise ValueError(f"main_direction 的维度不正确: {main_direction}")
    # 计算垂线方向
    perpendicular_direction = np.array([-main_direction[1], main_direction[0]], dtype=np.float32)

    # 根据垂线方向生成估计点
    ideal_points = []
    # for i in range(-num_points // 2, num_points // 2 + 1):  # 从质心前后生成多个点
    #     offset = step_length * i
    #     ideal_point = centroid + offset * perpendicular_direction
    #     ideal_points.append(ideal_point)
    offset = step_length * 1
    ideal_point = centroid + offset * perpendicular_direction
    ideal_points.append(ideal_point)
    return np.array(ideal_points)

def process_image(image_path, area_threshold=100, json_output_path=None):
    """
    处理图像：找到中间虚线的轮廓，生成理想点并绘制结果。
    """
    # 读取图像
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(f"图像路径无效: {image_path}")

    # 二值化处理
    _, binary_image = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY)

    # 查找轮廓
    contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 区分小轮廓（中间虚线）和大轮廓（外侧车道/内侧车道）
    small_contours = []
    large_contours = []

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < area_threshold:
            small_contours.append(contour)  # 面积小的轮廓
        else:
            large_contours.append(contour)  # 面积大的轮廓

    # 初始化可视化图像
    visual_image = cv2.cvtColor(binary_image, cv2.COLOR_GRAY2BGR)
    centroids = []
    # 处理中间虚线轮廓
    for contour in small_contours:
        centroid, main_direction = calculate_contour_direction(contour)  # 计算质心和方向
        ideal_points = generate_ideal_points(centroid, main_direction, step_length=30, num_points=5)  # 生成理想点
        centroids.append({"x": int(centroid[0]), "y": int(centroid[1])})
        # 绘制质心
        cv2.circle(visual_image, tuple(centroid.astype(int)), 5, (0, 0, 255), -1)

        # 绘制理想点
        # for pt in ideal_points:
        #     cv2.circle(visual_image, tuple(pt.astype(int)), 5, (255, 0, 0), -1)

    # 绘制轮廓（可选）
    cv2.drawContours(visual_image, large_contours, -1, (0, 255, 0), 2)  # 绿色：大轮廓
    cv2.drawContours(visual_image, small_contours, -1, (0, 255, 255), 1)  # 黄色：小轮廓

    if json_output_path:
        with open(json_output_path, "w") as json_file:
            json.dump(centroids, json_file, indent=4)
        print(f"质心坐标已保存到: {json_output_path}")
    return visual_image

if __name__ == "__main__":
    # 输入图像路径
    image_path = "D:/AF/simulation-master/shaped_track_2/shaped_track_2.png"  # 替换为实际路径
    output_image_path = "D:/AF/simulation-master/shaped_track_2/output.png"
    json_output_path = "D:/AF/simulation-master/shaped_track_2/centroids.json"
    # 处理图像并生成结果
    processed_image = process_image(image_path, area_threshold=2000, json_output_path=json_output_path)  # 设定阈值区分大/小轮廓

    # 显示结果
    cv2.imshow("Processed Image", processed_image)
    cv2.imwrite(output_image_path, processed_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
