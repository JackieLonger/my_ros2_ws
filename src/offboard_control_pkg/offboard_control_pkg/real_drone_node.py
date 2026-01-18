#!/usr/bin/env python3
"""
real_drone_node.py - 針對真實 PX4 v1.14.3 無人機的最終修正版本

修正重點：
1. 加入 time.sleep 防止指令發送太快導致通訊堵塞 (解決影片中轉一下就停的問題)。
2. 修改起飛判斷邏輯：直接檢查 arming_state，解決氣壓計漂移導致無法起飛的問題。
3. QoS 設定維持 VOLATILE 以匹配真實機。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleGlobalPosition, VehicleLocalPosition
import math
import json
import time  # <--- 必要：用於指令延遲

class DroneNode(Node):
    def __init__(self):
        super().__init__('drone_control_node')

        self.declare_parameter('drone_id', 1)
        self.declare_parameter('require_gps', True)
        self.declare_parameter('gps_eph_threshold', 2.5) # 放寬一點精度要求
        
        self.drone_id = self.get_parameter('drone_id').get_parameter_value().integer_value
        self.require_gps = self.get_parameter('require_gps').get_parameter_value().bool_value
        self.gps_eph_threshold = self.get_parameter('gps_eph_threshold').get_parameter_value().double_value
        self.target_ns = f'px4_{self.drone_id}'

        self.get_logger().info(f"[{self.target_ns}] === 真實機 Offboard 控制版 (最終修正) ===")
        self.get_logger().info(f"[{self.target_ns}] Drone ID: {self.drone_id}, Require GPS: {self.require_gps}")

        # QoS 設定 (適配真實機 PX4)
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        cmd_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Publishers
        self.offboard_ctrl_pub = self.create_publisher(OffboardControlMode, f'/{self.target_ns}/fmu/in/offboard_control_mode', px4_qos)
        self.traj_pub = self.create_publisher(TrajectorySetpoint, f'/{self.target_ns}/fmu/in/trajectory_setpoint', px4_qos)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, f'/{self.target_ns}/fmu/in/vehicle_command', px4_qos)

        # Subscribers
        self.status_sub = self.create_subscription(VehicleStatus, f'/{self.target_ns}/fmu/out/vehicle_status', self.status_cb, px4_qos)
        self.global_pos_sub = self.create_subscription(VehicleGlobalPosition, f'/{self.target_ns}/fmu/out/vehicle_global_position', self.global_pos_cb, px4_qos)
        self.local_pos_sub = self.create_subscription(VehicleLocalPosition, f'/{self.target_ns}/fmu/out/vehicle_local_position', self.local_pos_cb, px4_qos)
        
        # Laptop Commands
        self.create_subscription(String, f'/{self.target_ns}/laptop/action', self.action_cb, cmd_qos)
        self.create_subscription(Point, f'/{self.target_ns}/laptop/setpoint', self.setpoint_cb, cmd_qos)
        self.create_subscription(String, f'/{self.target_ns}/laptop/scan_action', self.scan_action_cb, cmd_qos)
        
        # Scan Components
        self.scan_control_pub = self.create_publisher(Bool, f'/{self.target_ns}/scan_control', cmd_qos)
        self.scan_ready_pub = self.create_publisher(String, f'/{self.target_ns}/scan_ready', cmd_qos)
        self.link_quality_sub = self.create_subscription(String, f'/{self.target_ns}/link_quality', self.link_quality_cb, cmd_qos)

        # Variables
        self.target_pos_ned = [0.0, 0.0, 0.0]
        self.origin_ref_ned = [0.0, 0.0, 0.0]
        self.current_local_ned = [0.0, 0.0, 0.0]
        
        self.is_connected = False
        self.pre_flight_checks_pass = False
        self.arming_state = 0
        self.got_global_pos = False
        self.current_eph = 999.9
        
        self.scan_results = {}
        self.expected_trackers = ["!e2e5b7c4", "!e2e5b8f8"]
        self.scan_completed = False
        
        self.status_check_count = 0

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info(f"[{self.target_ns}] 等待 PX4 連線...")

    def status_cb(self, msg):
        self.pre_flight_checks_pass = msg.pre_flight_checks_pass
        self.is_connected = True
        try:
            self.arming_state = int(msg.arming_state)
        except Exception:
            pass
        
        self.status_check_count += 1
        if self.status_check_count % 40 == 0: # 降低 log 頻率
            armed_str = "🔓 ARMED" if self.arming_state == 2 else "🔒 DISARMED"
            checks_str = "✅ PASS" if self.pre_flight_checks_pass else "❌ FAIL"
            self.get_logger().info(f"[{self.target_ns}] 狀態: {armed_str} | Preflight: {checks_str}")

    def global_pos_cb(self, msg):
        self.got_global_pos = True
        self.current_eph = msg.eph

    def local_pos_cb(self, msg: VehicleLocalPosition):
        try:
            self.current_local_ned = [float(msg.x), float(msg.y), float(msg.z)]
        except Exception:
            pass

    def setpoint_cb(self, msg):
        # ENU -> NED
        self.target_pos_ned = [msg.y, msg.x, -msg.z]

    def action_cb(self, msg):
        cmd = msg.data.upper()
        if cmd == "TAKEOFF_CHECK":
            self.perform_safety_takeoff()
        elif cmd == "LAND":
            self.get_logger().info(f"[{self.target_ns}] 執行降落...")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        elif cmd == "DISARM":
            self.get_logger().info(f"[{self.target_ns}] 強制上鎖...")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        elif cmd == "RESET_ORIGIN":
            self.origin_ref_ned = list(self.current_local_ned)
            self.get_logger().info(f"[{self.target_ns}] 重置相對原點 (PX4 內部原點未變)")

    def scan_action_cb(self, msg):
        cmd = msg.data.upper()
        if cmd == "START_SCAN":
            self.scan_results = {}
            self.scan_completed = False
            out = Bool(); out.data = True
            self.scan_control_pub.publish(out)
        elif cmd == "STOP_SCAN":
            out = Bool(); out.data = False
            self.scan_control_pub.publish(out)

    def link_quality_cb(self, msg):
        try:
            data = json.loads(msg.data)
            target_id = data.get('target_id')
            status = data.get('status')
            self.scan_results[target_id] = status
            
            if all(t in self.scan_results for t in self.expected_trackers):
                self.scan_completed = True
                out = String()
                out.data = json.dumps({"status": "completed", "results": self.scan_results})
                self.scan_ready_pub.publish(out)
        except Exception:
            pass

    def perform_safety_takeoff(self):
        """修正後的起飛流程"""
        self.get_logger().info(f"[{self.target_ns}] ===== 起飛檢查 =====")

        if not self.is_connected:
            self.get_logger().error("❌ 未連接 PX4")
            return
        if not self.pre_flight_checks_pass:
            self.get_logger().error("❌ Preflight Checks 失敗 (請檢查 QGC)")
            return
        
        if self.require_gps:
            if not self.got_global_pos or self.current_eph > self.gps_eph_threshold:
                self.get_logger().error(f"❌ GPS 精度不足: {self.current_eph:.2f}m")
                return

        # --- 核心修正：使用 arming_state 判斷 ---
        is_armed = (self.arming_state == 2)
        
        if is_armed:
            self.get_logger().info(f"ℹ️  已解鎖 -> 僅發送 Offboard 模式維持控制")
            for _ in range(5):
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                time.sleep(0.1)
        else:
            self.get_logger().info(f"✅ 未解鎖 -> 執行標準起飛 (Offboard + Arm)")
            self.get_logger().info(f"⚠️ 請確認遙控器油門已拉到最低！")
            
            # 1. 設定目標高度
            self.target_pos_ned = [0.0, 0.0, -5.0]
            
            # 2. 先切換模式 (給予緩衝時間)
            self.get_logger().info(">>> 1. 切換 Offboard 模式...")
            for i in range(5):
                self.publish_offboard_control_mode()
                self.publish_trajectory_setpoint(0.0, 0.0, 0.0) # 先送 0,0,0 防止暴衝
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                time.sleep(0.2) # 每個指令間隔 0.2 秒
            
            # 3. 再執行解鎖
            self.get_logger().info(">>> 2. 執行解鎖 (Arm)...")
            for i in range(5):
                self.publish_offboard_control_mode() # 持續發送 heartbeat
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                time.sleep(0.5) # 解鎖需要較長時間確認
                
            self.get_logger().info("✅ 起飛指令發送完畢")

    def timer_callback(self):
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint(self.target_pos_ned[0], self.target_pos_ned[1], self.target_pos_ned[2])

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_ctrl_pub.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        # 加上原點偏移量
        msg.position = [
            float(self.origin_ref_ned[0] + x),
            float(self.origin_ref_ned[1] + y),
            float(z)
        ]
        msg.yaw = float('nan')
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.traj_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1 # 真實機通常為 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DroneNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()