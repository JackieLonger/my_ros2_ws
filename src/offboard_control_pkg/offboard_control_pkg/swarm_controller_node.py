#!/usr/bin/env python3
"""
Swarm Controller Node
- 訂閱所有無人機的 link_quality，收集目標位置
- 每 5 秒執行一次 K-Means，從 4 個目標選出 3 個最優聚類中心
- 使用匈牙利演算法（scipy.optimize.linear_sum_assignment）分配無人機
- 發佈目標點給各無人機
- 在 RViz 上可視化 3 個聚類中心（藍色球體）
"""

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition
from std_msgs.msg import String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, ReliabilityPolicy
import json
import numpy as np
import math
from sklearn.cluster import KMeans
from scipy.optimize import linear_sum_assignment
from datetime import datetime
import threading
import time

class SwarmController(Node):
    def __init__(self):
        super().__init__('swarm_controller_node')
        
        self.drone_ids = [1, 2, 3]
        self.num_drones = len(self.drone_ids)
        
        # 訂閱所有無人機的位置
        self.drone_positions = {did: {'x': 0.0, 'y': 0.0, 'z': -5.0} for did in self.drone_ids}
        self.drone_local_positions = {did: {'x': 0.0, 'y': 0.0, 'z': 0.0} for did in self.drone_ids}
        
        # 訊號收集
        self.target_data = {}  # {target_name: [positions, rssi, ...]}
        self.target_positions = []  # N x 2 array of [x, y]
        
        # 聚類結果
        self.centroids = None
        self.assignments = None  # drone_id -> centroid_idx
        
        # PX4 使用 BEST_EFFORT QoS
        from rclpy.qos import HistoryPolicy, DurabilityPolicy
        qos_sub_px4 = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        qos_pub = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        
        # 訂閱無人機位置（用於計算成本矩陣）
        for did in self.drone_ids:
            self.create_subscription(
                VehicleLocalPosition,
                f'/px4_{did}/fmu/out/vehicle_local_position',
                lambda msg, drone_id=did: self.vehicle_position_cb(msg, drone_id),
                qos_sub_px4
            )
        
        # 訂閱所有無人機的 link_quality
        for did in self.drone_ids:
            self.create_subscription(
                String,
                f'/px4_{did}/link_quality',
                lambda msg, drone_id=did: self.link_quality_cb(msg, drone_id),
                qos_pub
            )
        
        # 發佈目標點給各無人機的 setpoint
        self.setpoint_pubs = {}
        for did in self.drone_ids:
            self.setpoint_pubs[did] = self.create_publisher(
                Point,
                f'/px4_{did}/laptop/setpoint',
                qos_pub
            )
        
        # 發佈聚類中心的 RViz 標記
        self.centroid_marker_pub = self.create_publisher(
            MarkerArray,
            '/swarm/centroids',
            qos_pub
            )
        
        # 定時執行 K-Means + 拍賣演算法
        self.create_timer(5.0, self.run_swarm_algorithm)
        self.create_timer(1.0, self.publish_centroid_markers)
        
        self.get_logger().info("Swarm Controller started")
    
    def vehicle_position_cb(self, msg, drone_id):
        """訂閱無人機位置（NED，用於成本計算）"""
        self.drone_local_positions[drone_id] = {
            'x': float(msg.x),
            'y': float(msg.y),
            'z': float(msg.z)
        }
    
    def link_quality_cb(self, msg, drone_id):
        """收集訊號數據，記錄目標位置"""
        try:
            data = json.loads(msg.data)
            target_id = data.get('target_id')
            
            if target_id not in self.target_data:
                self.target_data[target_id] = []
            
            # 記錄這個目標的位置（使用 NED 坐標）
            target_x = data.get('target_x', 0.0)
            target_y = data.get('target_y', 0.0)
            
            self.target_data[target_id].append({
                'x': target_x,
                'y': target_y,
                'rssi': data.get('forward_rssi', -100),
                'snr': data.get('forward_snr', -10)
            })
        except Exception as e:
            self.get_logger().error(f"Failed to parse link_quality: {e}")
    
    def run_swarm_algorithm(self):
        """執行 K-Means + 匈牙利演算法"""
        # 檢查無人機是否已起飛（z < -1.0 表示離地）
        airborne_count = 0
        for did in self.drone_ids:
            if self.drone_local_positions[did]['z'] < -1.0:
                airborne_count += 1
        
        if airborne_count == 0:
            self.get_logger().info("等待無人機起飛...")
            return
        
        # 收集所有已知的目標位置
        target_positions = []
        target_names = []
        
        for target_name, records in self.target_data.items():
            if records:
                # 取平均位置
                avg_x = np.mean([r['x'] for r in records])
                avg_y = np.mean([r['y'] for r in records])
                target_positions.append([avg_x, avg_y])
                target_names.append(target_name)
        
        if len(target_positions) < self.num_drones:
            self.get_logger().warn(
                f"Not enough targets ({len(target_positions)}) for {self.num_drones} drones"
            )
            return
        
        target_positions = np.array(target_positions)
        self.target_positions = target_positions
        
        # K-Means 聚類：將 N 個目標聚成 3 個中心
        try:
            kmeans = KMeans(n_clusters=self.num_drones, random_state=42, n_init=10)
            kmeans.fit(target_positions)
            self.centroids = kmeans.cluster_centers_
            
            self.get_logger().info(
                f"K-Means completed: {len(target_positions)} targets -> {self.num_drones} centroids"
            )
        except Exception as e:
            self.get_logger().error(f"K-Means failed: {e}")
            return
        
        # 計算成本矩陣：cost[i][j] = distance from drone i to centroid j
        cost_matrix = np.zeros((self.num_drones, self.num_drones))
        
        for i, did in enumerate(self.drone_ids):
            drone_x = self.drone_local_positions[did]['x']
            drone_y = self.drone_local_positions[did]['y']
            
            for j, centroid in enumerate(self.centroids):
                centroid_x, centroid_y = centroid
                distance = math.sqrt((drone_x - centroid_x)**2 + (drone_y - centroid_y)**2)
                cost_matrix[i][j] = distance
        
        # 匈牙利演算法：找最優分配
        drone_indices, centroid_indices = linear_sum_assignment(cost_matrix)
        
        # 建立分配映射
        self.assignments = {}
        for drone_idx, centroid_idx in zip(drone_indices, centroid_indices):
            did = self.drone_ids[drone_idx]
            self.assignments[did] = centroid_idx
            
            centroid = self.centroids[centroid_idx]
            self.get_logger().info(
                f"Drone {did} -> Centroid {centroid_idx} at [{centroid[0]:.2f}, {centroid[1]:.2f}]"
            )
            
            # 發佈目標點（使用 NED 座標，drone_node 會處理 ENU 轉換）
            setpoint = Point()
            setpoint.x = centroid[0]      # NED X
            setpoint.y = centroid[1]      # NED Y
            setpoint.z = 5.0              # ENU 高度（正值向上）
            
            self.setpoint_pubs[did].publish(setpoint)
    
    def publish_centroid_markers(self):
        """發佈 RViz 標記，顯示 3 個聚類中心"""
        if self.centroids is None:
            return
        
        markers = MarkerArray()
        
        for i, centroid in enumerate(self.centroids):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "swarm_centroids"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # 使用 NED 座標
            marker.pose.position.x = centroid[0]
            marker.pose.position.y = centroid[1]
            marker.pose.position.z = 0.0
            
            marker.pose.orientation.w = 1.0
            marker.scale.x = 1.5
            marker.scale.y = 1.5
            marker.scale.z = 1.5
            
            # 藍色球體
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 0.6
            
            markers.markers.append(marker)
        
        self.centroid_marker_pub.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    node = SwarmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
