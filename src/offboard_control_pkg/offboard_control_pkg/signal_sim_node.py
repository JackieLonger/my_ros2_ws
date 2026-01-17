#!/usr/bin/env python3
"""
Signal Simulator Node
- 定義 4 個靜止地面訊號源
- 訂閱各無人機位置，計算到各目標的距離
- 模擬 RSSI/SNR，發佈 link_quality JSON
- 在 RViz 上可視化目標位置（紅色球體）
"""

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, ReliabilityPolicy
import json
import math
import numpy as np
from datetime import datetime

class SignalSimulator(Node):
    def __init__(self):
        super().__init__('signal_sim_node')
        
        # 4 個靜止地面目標（Gazebo NED 座標）
        self.targets = {
            'target_A': {'x': 10.0, 'y': 10.0, 'z': 0.0},   # 前右
            'target_B': {'x': 15.0, 'y': -5.0, 'z': 0.0},   # 右後
            'target_C': {'x': -10.0, 'y': 8.0, 'z': 0.0},   # 左前
            'target_D': {'x': -5.0, 'y': -12.0, 'z': 0.0},  # 左後
        }
        
        # 無人機 ID
        self.drone_ids = [1, 2, 3]
        
        # 訂閱各無人機位置
        self.drone_positions = {}
        # PX4 使用 BEST_EFFORT QoS
        from rclpy.qos import HistoryPolicy, DurabilityPolicy
        qos_sub = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        qos_pub = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        
        for did in self.drone_ids:
            self.create_subscription(
                VehicleLocalPosition,
                f'/px4_{did}/fmu/out/vehicle_local_position',
                lambda msg, drone_id=did: self.vehicle_position_cb(msg, drone_id),
                qos_sub
            )
            self.drone_positions[did] = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        # 為各無人機建立 link_quality 發佈者
        self.link_quality_pubs = {}
        for did in self.drone_ids:
            self.link_quality_pubs[did] = self.create_publisher(
                String,
                f'/px4_{did}/link_quality',
                qos_pub
            )
        
        # 發佈 RViz 標記（目標位置）
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/signal_sim/markers',
            qos_pub
        )
        
        # 發佈無人機位置標記
        self.drone_marker_pub = self.create_publisher(
            MarkerArray,
            '/signal_sim/drones',
            qos_pub
        )
        
        # 定時發佈訊號與標記
        self.create_timer(0.5, self.publish_signals)
        self.create_timer(1.0, self.publish_markers)
        self.create_timer(0.2, self.publish_drone_markers)  # 5Hz 更新無人機位置
        
        self.get_logger().info("Signal Simulator started")
        
        # 除錯計數器
        self.pos_received_count = {did: 0 for did in self.drone_ids}
    
    def vehicle_position_cb(self, msg, drone_id):
        """訂閱無人機位置（NED 座標）"""
        self.drone_positions[drone_id] = {
            'x': float(msg.x),
            'y': float(msg.y),
            'z': float(msg.z)
        }
        self.pos_received_count[drone_id] += 1
        
        # 每 50 次打印一次位置（約 5 秒一次）
        if self.pos_received_count[drone_id] % 50 == 1:
            self.get_logger().info(
                f"[Drone {drone_id}] pos: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}"
            )
    
    def calculate_rssi(self, distance):
        """模擬 RSSI = -30 - 25*log10(distance) + noise"""
        if distance < 0.1:
            distance = 0.1
        rssi = -30 - 25 * math.log10(distance) + np.random.normal(0, 2)
        return max(-120, min(-20, rssi))  # 限制在 -120 到 -20 dBm
    
    def calculate_snr(self, rssi):
        """簡化 SNR 計算"""
        snr = rssi + 100  # 簡單映射
        return max(-10, min(20, snr))  # 限制在 -10 到 20 dB
    
    def publish_signals(self):
        """計算與發佈各無人機到各目標的訊號"""
        for did in self.drone_ids:
            drone_pos = self.drone_positions[did]
            drone_x, drone_y = drone_pos['x'], drone_pos['y']
            
            # 每個無人機對每個目標發佈訊號
            for target_name, target_pos in self.targets.items():
                target_x, target_y = target_pos['x'], target_pos['y']
                
                # 計算 3D 距離
                distance = math.sqrt(
                    (drone_x - target_x)**2 + 
                    (drone_y - target_y)**2 + 
                    (drone_pos['z'])**2
                )
                
                # 計算 RSSI 和 SNR（往返各一次）
                forward_rssi = self.calculate_rssi(distance)
                forward_snr = self.calculate_snr(forward_rssi)
                return_rssi = self.calculate_rssi(distance)
                return_snr = self.calculate_snr(return_rssi)
                
                # 組成 JSON
                data = {
                    'timestamp': datetime.now().isoformat(),
                    'status': 'Success',
                    'target_id': target_name,
                    'target_x': float(target_x),
                    'target_y': float(target_y),
                    'target_z': float(target_pos['z']),
                    'forward_rssi': float(forward_rssi),
                    'forward_snr': float(forward_snr),
                    'return_rssi': float(return_rssi),
                    'return_snr': float(return_snr),
                    'drone_x': float(drone_x),
                    'drone_y': float(drone_y),
                    'drone_z': float(drone_pos['z']),
                    'distance': float(distance)
                }
                
                # 發佈到 /px4_{id}/link_quality
                msg = String()
                msg.data = json.dumps(data)
                self.link_quality_pubs[did].publish(msg)
    
    def publish_markers(self):
        """發佈 RViz 標記，顯示 4 個目標位置"""
        markers = MarkerArray()
        marker_id = 0
        
        # 轉換座標系：NED -> ENU
        # NED: X=北, Y=東, Z=下  →  ENU: X=東, Y=北, Z=上
        # 轉換：x_enu = y_ned, y_enu = x_ned, z_enu = -z_ned
        
        for target_name, target_pos in self.targets.items():
            # 球體標記
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "signal_targets"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # 使用 NED 座標（PX4 標準）
            marker.pose.position.x = target_pos['x']
            marker.pose.position.y = target_pos['y']
            marker.pose.position.z = target_pos['z']
            
            marker.pose.orientation.w = 1.0
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            
            # 紅色球體
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.7
            
            markers.markers.append(marker)
            marker_id += 1
            
            # 文字標記（目標名稱）
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "signal_labels"
            text_marker.id = marker_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = target_pos['x']
            text_marker.pose.position.y = target_pos['y']
            text_marker.pose.position.z = target_pos['z'] + 1.5  # 球體上方
            
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = target_name
            
            markers.markers.append(text_marker)
            marker_id += 1
        
        self.marker_pub.publish(markers)
    
    def publish_drone_markers(self):
        """發佈無人機位置標記（綠色球體）"""
        markers = MarkerArray()
        
        # 無人機顏色：綠色
        colors = [
            (0.0, 1.0, 0.0),  # Drone 1: 綠色
            (1.0, 1.0, 0.0),  # Drone 2: 黃色
            (1.0, 0.5, 0.0),  # Drone 3: 橙色
        ]
        
        for i, did in enumerate(self.drone_ids):
            pos = self.drone_positions[did]
            
            # NED -> ENU 座標轉換（RViz 使用 ENU）
            enu_x = pos['x']   # NED North = ENU East (簡化：保持 x)
            enu_y = pos['y']   # NED East = ENU North (簡化：保持 y)
            enu_z = -pos['z']  # NED Down = -ENU Up
            
            # 無人機球體
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "drones"
            marker.id = did
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = enu_x
            marker.pose.position.y = enu_y
            marker.pose.position.z = enu_z
            
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.8
            marker.scale.y = 0.8
            marker.scale.z = 0.8
            
            color = colors[i % len(colors)]
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 1.0
            
            markers.markers.append(marker)
            
            # 無人機標籤
            label = Marker()
            label.header.frame_id = "map"
            label.header.stamp = self.get_clock().now().to_msg()
            label.ns = "drone_labels"
            label.id = did + 100
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            
            label.pose.position.x = enu_x
            label.pose.position.y = enu_y
            label.pose.position.z = enu_z + 1.0
            
            label.pose.orientation.w = 1.0
            label.scale.z = 0.6
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = f"Drone_{did}"
            
            markers.markers.append(label)
        
        self.drone_marker_pub.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    node = SignalSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
