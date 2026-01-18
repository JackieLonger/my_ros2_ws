#!/usr/bin/env python3
"""
real_drone_node.py - 針對真實 PX4 v1.14.3 無人機的修正版本

關鍵修改：
1. QoS: TRANSIENT_LOCAL -> VOLATILE (真實機 PX4 廣播的是 Volatile，匹配才能收到)
2. Target System ID: drone_id + 1 -> 1 (真實機 MAV_SYS_ID 通常預設 1)
3. GPS 檢查: 可根據飛行環境移除 (室內飛行需移除)
4. 日誌改進: 更清楚的狀態提示與故障排查信息
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleGlobalPosition, VehicleLocalPosition
import math
import json

class DroneNode(Node):
    def __init__(self):
        super().__init__('drone_control_node')

        self.declare_parameter('drone_id', 1)
        self.declare_parameter('require_gps', True)  # 新增參數：是否需要 GPS (室內可設為 False)
        
        self.drone_id = self.get_parameter('drone_id').get_parameter_value().integer_value
        self.require_gps = self.get_parameter('require_gps').get_parameter_value().bool_value
        self.target_ns = f'px4_{self.drone_id}'

        self.get_logger().info(f"[{self.target_ns}] === 真實機 Offboard 控制版 ===")
        self.get_logger().info(f"[{self.target_ns}] Drone ID: {self.drone_id}, Require GPS: {self.require_gps}")

        # ============================================================
        # 【修正 1】QoS 設定：改為 VOLATILE (真實機 PX4 v1.14 的廣播模式)
        # 原因：
        # - 模擬器 SITL: Transient Local (保留歷史)
        # - 真實機 PX4: Volatile (射後不理，為了省頻寬)
        # - ROS 2 QoS 匹配規則：訂閱者要求不能高於發布者
        # - 若訂閱者要 Transient Local，發布者只有 Volatile，連線會失敗且不報錯
        # ============================================================
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,  # ← 【重要】改為 VOLATILE
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        cmd_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Publishers
        self.offboard_ctrl_pub = self.create_publisher(OffboardControlMode, f'/{self.target_ns}/fmu/in/offboard_control_mode', px4_qos)
        self.traj_pub = self.create_publisher(TrajectorySetpoint, f'/{self.target_ns}/fmu/in/trajectory_setpoint', px4_qos)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, f'/{self.target_ns}/fmu/in/vehicle_command', px4_qos)

        # Subscribers (PX4 Telemetry)
        self.status_sub = self.create_subscription(VehicleStatus, f'/{self.target_ns}/fmu/out/vehicle_status', self.status_cb, px4_qos)
        self.global_pos_sub = self.create_subscription(VehicleGlobalPosition, f'/{self.target_ns}/fmu/out/vehicle_global_position', self.global_pos_cb, px4_qos)
        self.local_pos_sub = self.create_subscription(VehicleLocalPosition, f'/{self.target_ns}/fmu/out/vehicle_local_position', self.local_pos_cb, px4_qos)
        
        # Subscribers (筆電指令)
        self.create_subscription(String, f'/{self.target_ns}/laptop/action', self.action_cb, cmd_qos)
        self.create_subscription(Point, f'/{self.target_ns}/laptop/setpoint', self.setpoint_cb, cmd_qos)
        self.create_subscription(String, f'/{self.target_ns}/laptop/scan_action', self.scan_action_cb, cmd_qos)
        
        # 掃描相關 Publishers/Subscribers
        self.scan_control_pub = self.create_publisher(Bool, f'/{self.target_ns}/scan_control', cmd_qos)
        self.scan_ready_pub = self.create_publisher(String, f'/{self.target_ns}/scan_ready', cmd_qos)
        self.link_quality_sub = self.create_subscription(String, f'/{self.target_ns}/link_quality', self.link_quality_cb, cmd_qos)

        # 狀態變數
        # 預設目標設為 0,0,0 (地面)，避免一啟動就飛走
        self.target_pos_ned = [0.0, 0.0, 0.0]
        # 原點參考（NED），RESET_ORIGIN 時更新為當前 Local Position
        self.origin_ref_ned = [0.0, 0.0, 0.0]
        self.current_local_ned = [0.0, 0.0, 0.0]
        
        # 安全檢查變數
        self.is_connected = False
        self.pre_flight_checks_pass = False
        self.arming_state = 0
        self.got_global_pos = False
        self.current_eph = 999.9
        
        # 掃描狀態變數
        self.scan_results = {}  # {"!e2e5b7c4": "Success", "!e2e5b8f8": "Timeout"}
        self.expected_trackers = ["!e2e5b7c4", "!e2e5b8f8"]  # 預期的兩個 tracker ID
        self.scan_completed = False
        
        # 統計變數
        self.status_check_count = 0
        self.last_log_time = 0

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info(f"[{self.target_ns}] 節點初始化完成，等待 PX4 連線...")

    def status_cb(self, msg):
        """接收飛行狀態"""
        self.pre_flight_checks_pass = msg.pre_flight_checks_pass
        self.is_connected = True
        try:
            self.arming_state = int(msg.arming_state)
        except Exception:
            pass
        
        # 每 20 次回調 (~1 秒) 記錄一次狀態，避免日誌過多
        self.status_check_count += 1
        if self.status_check_count % 20 == 0:
            armed_str = "🔓 ARMED" if self.arming_state == 2 else "🔒 DISARMED"
            checks_str = "✅ PASS" if self.pre_flight_checks_pass else "❌ FAIL"
            self.get_logger().info(f"[{self.target_ns}] 狀態: {armed_str} | Preflight: {checks_str}")

    def global_pos_cb(self, msg):
        """接收全局位置 (GPS)"""
        self.got_global_pos = True
        self.current_eph = msg.eph

    def local_pos_cb(self, msg: VehicleLocalPosition):
        """接收本地位置 (NED)"""
        try:
            self.current_local_ned = [float(msg.x), float(msg.y), float(msg.z)]
        except Exception as e:
            self.get_logger().error(f"[{self.target_ns}] 解析本地位置失敗: {e}")

    def setpoint_cb(self, msg):
        """接收筆電的移動指令 (ROS2 ENU) 並轉換為 PX4 NED"""
        # ============================================================
        # 坐標系轉換：ROS2 ENU -> PX4 NED
        # ENU: X=East, Y=North, Z=Up
        # NED: X=North, Y=East, Z=Down
        # 轉換公式：
        #   X_ned = Y_enu  (North = ENU's North/Y)
        #   Y_ned = X_enu  (East = ENU's East/X)
        #   Z_ned = -Z_enu (Down = -Up)
        # ============================================================
        x_enu = msg.x
        y_enu = msg.y
        z_enu = msg.z
        
        x_ned = y_enu
        y_ned = x_enu
        z_ned = -z_enu
        
        self.target_pos_ned = [x_ned, y_ned, z_ned]
        self.get_logger().debug(f"[{self.target_ns}] 坐標轉換: ENU({x_enu:.1f},{y_enu:.1f},{z_enu:.1f}) -> NED({x_ned:.1f},{y_ned:.1f},{z_ned:.1f})")

    def action_cb(self, msg):
        """接收筆電的動作指令 (起飛, 降落, 上鎖, 重置原點)"""
        cmd = msg.data.upper()
        if cmd == "TAKEOFF_CHECK":
            self.perform_safety_takeoff()
        elif cmd == "LAND":
            self.get_logger().info(f"[{self.target_ns}] 執行降落...")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        elif cmd == "DISARM":
            self.get_logger().info(f"[{self.target_ns}] 強制上鎖 (Disarm)")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        elif cmd == "RESET_ORIGIN":
            # 設定新的原點參考為當前 Local Position（NED）
            self.origin_ref_ned = list(self.current_local_ned)
            self.get_logger().info(f"[{self.target_ns}] 重置原點為本地 NED {self.origin_ref_ned}")
    
    def scan_action_cb(self, msg):
        """接收筆電的掃描控制命令 (START_SCAN / STOP_SCAN)"""
        cmd = msg.data.upper()
        if cmd == "START_SCAN":
            self.get_logger().info(f"[{self.target_ns}] 掃描開始...")
            # 清空之前的掃描結果
            self.scan_results = {}
            self.scan_completed = False
            # 發送啟動命令給 fast_scan_node
            scan_msg = Bool()
            scan_msg.data = True
            self.scan_control_pub.publish(scan_msg)
        elif cmd == "STOP_SCAN":
            self.get_logger().info(f"[{self.target_ns}] 掃描停止")
            scan_msg = Bool()
            scan_msg.data = False
            self.scan_control_pub.publish(scan_msg)
    
    def link_quality_cb(self, msg):
        """接收 fast_scan_node 回傳的連線品質數據"""
        try:
            data = json.loads(msg.data)
            target_id = data.get('target_id')
            status = data.get('status')
            
            # 記錄結果
            self.scan_results[target_id] = status
            self.get_logger().debug(f"[{self.target_ns}] 掃描結果: {target_id} = {status}")
            
            # 檢查是否所有 tracker 都已掃描完成
            if all(tracker in self.scan_results for tracker in self.expected_trackers):
                self.scan_completed = True
                self.get_logger().info(f"[{self.target_ns}] ✅ 掃描完成！結果: {self.scan_results}")
                # 發布掃描完成通知給筆電
                ready_msg = String()
                ready_msg.data = json.dumps({
                    "status": "completed",
                    "results": self.scan_results
                })
                self.scan_ready_pub.publish(ready_msg)
        except Exception as e:
            self.get_logger().error(f"[{self.target_ns}] 掃描數據解析失敗: {e}")

    def perform_safety_takeoff(self):
        """執行安全起飛檢查與起飛程序"""
        self.get_logger().info(f"[{self.target_ns}] ===== 起飛前檢查開始 =====")

        # 檢查 1: 連接狀態
        if not self.is_connected:
            self.get_logger().error(f"[{self.target_ns}] ❌ 未連接 PX4 (無 vehicle_status 數據)")
            self.get_logger().error(f"[{self.target_ns}]    ℹ️ 檢查: MicroXRCEAgent 是否啟動，Serial 連線是否正常")
            return
        self.get_logger().info(f"[{self.target_ns}] ✅ PX4 已連接")

        # 檢查 2: Preflight Checks
        if not self.pre_flight_checks_pass:
            self.get_logger().error(f"[{self.target_ns}] ❌ Preflight Checks 未通過")
            self.get_logger().error(f"[{self.target_ns}]    ℹ️ 檢查: 磁力計、陀螺儀、加速度計、EEPROM")
            self.get_logger().error(f"[{self.target_ns}]    ℹ️ 在 QGC 檢查詳細錯誤，可能需要校正傳感器或 Compass Calibration")
            return
        self.get_logger().info(f"[{self.target_ns}] ✅ Preflight Checks 通過")

        # 檢查 3: GPS 檢查 (可根據飛行環境跳過)
        if self.require_gps:
            if not self.got_global_pos:
                self.get_logger().error(f"[{self.target_ns}] ❌ 無 GPS 數據")
                self.get_logger().error(f"[{self.target_ns}]    ℹ️ 室外飛行需要 GPS，請等待 GPS 解鎖 (查看 QGC 狀態)")
                self.get_logger().error(f"[{self.target_ns}]    ℹ️ 室內飛行請用 --ros-args -p require_gps:=false 啟動本節點")
                return
            if self.current_eph > 1.5:
                self.get_logger().error(f"[{self.target_ns}] ❌ GPS 精度不足: {self.current_eph:.2f}m (需要 < 1.5m)")
                self.get_logger().error(f"[{self.target_ns}]    ℹ️ 請等待 GPS 精度改善，或在室內環境改用光流/Vicon")
                return
            self.get_logger().info(f"[{self.target_ns}] ✅ GPS 就緒 (精度: {self.current_eph:.2f}m)")
        else:
            self.get_logger().info(f"[{self.target_ns}] ⏭️  GPS 檢查已跳過 (室內飛行模式)")

        # 檢查 4: 高度檢查
        current_height = -self.current_local_ned[2]  # NED z 為負數，取反得高度
        in_air = current_height > 0.5
        
        if in_air:
            # 已在空中：僅保證 Offboard，不重設高度與重複 Arm
            self.get_logger().info(f"[{self.target_ns}] ℹ️  飛機已在空中 (高度: {current_height:.2f}m)")
            self.get_logger().info(f"[{self.target_ns}] ✅ 起飛前檢查全部通過 -> 切換 Offboard 模式")
            for _ in range(3):
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)  # Offboard
        else:
            # 地面待命：執行解鎖 + 起飛
            self.get_logger().info(f"[{self.target_ns}] ✅ 起飛前檢查全部通過")
            self.get_logger().info(f"[{self.target_ns}] >>> 執行 Offboard 起飛流程...")
            
            # 設定起飛目標高度 (-5.0 NED = 5 米上升)
            self.target_pos_ned = [0.0, 0.0, -5.0]
            
            # 發送多次命令確保飛控接收 (通常需要 5-10 個 heartbeat)
            for i in range(10):
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)  # Offboard
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)  # Arm
            
            self.get_logger().info(f"[{self.target_ns}] 💫 指令已發送，無人機應該開始爬升...")

    def timer_callback(self):
        """定時器回調 (20Hz): 持續發送 Offboard 心跳 + 目標位置"""
        # 持續發送 Offboard 模式控制信號 (必須持續送，否則飛控會超時退出 Offboard)
        self.publish_offboard_control_mode()
        # 持續發送目標位置 (Setpoint 改變時飛機自動跟隨)
        self.publish_trajectory_setpoint(self.target_pos_ned[0], self.target_pos_ned[1], self.target_pos_ned[2])

    def publish_offboard_control_mode(self):
        """發送 Offboard 控制模式"""
        msg = OffboardControlMode()
        msg.position = True  # 啟用位置控制
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_ctrl_pub.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z):
        """發送軌跡設定點 (NED 座標)"""
        msg = TrajectorySetpoint()
        # 注意：位置包含原點偏移 (X/Y)，但 Z 為絕對值 (NED 座標中的 z)
        pos_ned = [
            float(self.origin_ref_ned[0] + x),
            float(self.origin_ref_ned[1] + y),
            float(z)
        ]
        msg.position = pos_ned
        msg.yaw = float('nan')  # 不控制偏航角
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.traj_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        """發送飛行指令到飛控"""
        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        
        # ============================================================
        # 【修正 2】Target System ID: 改為 1 (移除 + 1 邏輯)
        # 原因：
        # - 模擬器 SITL: 多機時需要 target_system = drone_id + 1
        #   (因為模擬器強制改了 MAV_SYS_ID，例如 ID=1 的機器的 MAV_SYS_ID=2)
        # - 真實機 PX4: MAV_SYS_ID 預設 1，無論有多少台無人機
        #   (不同機器靠 Namespace 區分，不靠 MAV_SYS_ID 區分)
        # - 若錯誤設定 target_system，飛控會直接忽略命令
        # ============================================================
        msg.target_system = 1  # ← 【重要】改為 1，移除 + 1
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
