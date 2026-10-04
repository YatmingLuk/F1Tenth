#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import yaml
import csv
import os
from scipy.interpolate import splprep, splev

class MapCenterlineNode(Node):
    def __init__(self):
        super().__init__('map_centerline_node')

        # 1. 宣告 ROS 2 參數，並提供預設值
        self.declare_parameter('yaml_path', '/home/chanky/my_map.yaml')
        self.declare_parameter('png_path', '/home/chanky/my_map.pgm')
        self.declare_parameter('output_csv', '/home/chanky/newf1_ws/src/car_control/car_control/waypoints/waypoints.csv')

        # 2. 獲取參數值
        self.yaml_path = self.get_parameter('yaml_path').get_parameter_value().string_value
        self.png_path = self.get_parameter('png_path').get_parameter_value().string_value
        self.output_csv = self.get_parameter('output_csv').get_parameter_value().string_value

        self.get_logger().info("--- 開始讀地圖計算中心線 ---")
        self.process_map()

    def process_map(self):
        # 檢查檔案是否存在
        if not os.path.exists(self.yaml_path):
            self.get_logger().error(f"找不到 YAML 檔: {self.yaml_path}")
            return
        if not os.path.exists(self.png_path):
            self.get_logger().error(f"找不到圖片檔: {self.png_path}")
            return

        # 1. 讀取 YAML 配置
        with open(self.yaml_path, 'r') as f:
            map_data = yaml.safe_load(f)
        
        res = map_data['resolution']        
        origin_x = map_data['origin'][0]    
        origin_y = map_data['origin'][1]    
        
        img = cv2.imread(self.png_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            self.get_logger().error(f"無法讀取圖片: {self.png_path}")
            return
        h, w = img.shape

        # 2. 處理地圖：讓賽道變純白 (255)，牆壁變黑 (0)
        _, binary = cv2.threshold(img, 240, 255, cv2.THRESH_BINARY)
        kernel = np.ones((3,3), np.uint8) 
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

        # 3. 提取輪廓
        contours, hierarchy = cv2.findContours(binary, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
        
        if hierarchy is None or len(contours) < 2:
            self.get_logger().error("找不到足夠的輪廓，請確認地圖是否包含封閉賽道")
            return

        # 找出面積最大的輪廓作為外牆，第二大的作為內牆
        sorted_indices = np.argsort([cv2.contourArea(c) for c in contours])[::-1]
        out_wall = contours[sorted_indices[0]][:, 0, :] 
        in_wall = contours[sorted_indices[1]][:, 0, :]  

        raw_center_x = []
        raw_center_y = []

        # 4. 逐點對點邏輯
        sample_dists = [np.linalg.norm(out_wall - in_wall[idx], axis=1).min() for idx in range(0, len(in_wall), 10)]
        median_track_width = np.median(sample_dists)

        for i in range(len(in_wall)):
            p_in = in_wall[i]
            dists = np.linalg.norm(out_wall - p_in, axis=1)
            nearest_idx = np.argmin(dists)
            
            if dists[nearest_idx] > (median_track_width * 1.4):
                continue
                
            p_out = out_wall[nearest_idx]
            mid_c = (p_out[0] + p_in[0]) / 2.0
            mid_r = (p_out[1] + p_in[1]) / 2.0
            
            world_x = origin_x + (mid_c * res)
            world_y = origin_y + ((h - mid_r) * res)
            
            raw_center_x.append(world_x)
            raw_center_y.append(world_y)

        # 5. 運用週期性三次樣條 (Periodic Spline)
        pts = np.array([raw_center_x, raw_center_y])
        tck, u = splprep(pts, s=0.005, per=True)
        
        u_new = np.linspace(0, 1, 500)
        smooth_x, smooth_y = splev(u_new, tck)
        center_points = list(zip(smooth_x, smooth_y))
        
        # 建立輸出目錄（如果不存在的話）
        os.makedirs(os.path.dirname(self.output_csv), exist_ok=True)

        # 6. 儲存為 CSV
        with open(self.output_csv, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y'])
            for p in center_points:
                writer.writerow(p)

        self.get_logger().info(f"完美的中心線處理完成！")
        self.get_logger().info(f"生成點數: {len(center_points)} -> 檔案儲存至: {self.output_csv}")

def main(args=None):
    rclpy.init(args=args)
    node = MapCenterlineNode()
    # 由於此任務是一次性的（生完 CSV 即可結束），不需要 rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
