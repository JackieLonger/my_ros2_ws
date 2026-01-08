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
        self.drone_id = self.get_parameter('drone_id').get_parameter_value().integer_value
        self.target_ns = f'px4_{self.drone_id}'

        self.get_logger().info(f"[{self.target_ns}] 純 Offboard 起飛版 (含安全檢查)")

        # QoS
        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)
        cmd_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Publishers
        self.offboard_ctrl_pub = self.create_publisher(OffboardControlMode, f'/{self.target_ns}/fmu/in/offboard_control_mode', px4_qos)
        self.traj_pub = self.create_publisher(TrajectorySetpoint, f'/{self.target_ns}/fmu/in/trajectory_setpoint', px4_qos)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, f'/{self.target_ns}/fmu/in/vehicle_command', px4_qos)

        # Subscribers
        self.status_sub = self.create_subscription(VehicleStatus, f'/{self.target_ns}/fmu/out/vehicle_status', self.status_cb, px4_qos)
        self.global_pos_sub = self.create_subscription(VehicleGlobalPosition, f'/{self.target_ns}/fmu/out/vehicle_global_position', self.global_pos_cb, px4_qos)
        self.local_pos_sub = self.create_subscription(VehicleLocalPosition, f'/{self.target_ns}/fmu/out/vehicle_local_position', self.local_pos_cb, px4_qos)
        
        # 筆電指令
        self.create_subscription(String, f'/{self.target_ns}/laptop/action', self.action_cb, cmd_qos)
        self.create_subscription(Point, f'/{self.target_ns}/laptop/setpoint', self.setpoint_cb, cmd_qos)
        self.create_subscription(String, f'/{self.target_ns}/laptop/scan_action', self.scan_action_cb, cmd_qos)
        
        # 掃描相關 Publisher/Subscriber
        self.scan_control_pub = self.create_publisher(Bool, f'/{self.target_ns}/scan_control', cmd_qos)
        self.scan_ready_pub = self.create_publisher(String, f'/{self.target_ns}/scan_ready', cmd_qos)
        self.link_quality_sub = self.create_subscription(String, f'/{self.target_ns}/link_quality', self.link_quality_cb, cmd_qos)

        # 變數
        # 預設目標設為 0,0,0 (地面)，避免一啟動就飛走
        self.target_pos_ned = [0.0, 0.0, 0.0]
        # 原點參考（NED），RESET_ORIGIN 時更新為當前 Local Position
        self.origin_ref_ned = [0.0, 0.0, 0.0]
        self.current_local_ned = [0.0, 0.0, 0.0]
        
        # 安全變數
        self.is_connected = False
        self.pre_flight_checks_pass = False
        self.arming_state = 0
        self.got_global_pos = False
        self.current_eph = 999.9
        
        # 掃描變數
        self.scan_results = {}  # {"!e2e5b7c4": "Success", "!e2e5b8f8": "Timeout"}
        self.expected_trackers = ["!e2e5b7c4", "!e2e5b8f8"]  # 預期的兩個 tracker ID
        self.scan_completed = False 

        self.timer = self.create_timer(0.05, self.timer_callback)

    def status_cb(self, msg):
        self.pre_flight_checks_pass = msg.pre_flight_checks_pass
        self.is_connected = True
        try:
            self.arming_state = int(msg.arming_state)
        except Exception:
            pass

    def global_pos_cb(self, msg):
        self.got_global_pos = True
        self.current_eph = msg.eph

    def local_pos_cb(self, msg: VehicleLocalPosition):
        # PX4 Local Position: NED frame
        try:
            self.current_local_ned = [float(msg.x), float(msg.y), float(msg.z)]
        except Exception:
            pass

    def setpoint_cb(self, msg):
           # 接收筆電的移動指令 (ROS2 ENU) 並轉換為 PX4 NED
           # ENU to NED conversion:
           #   X_ned = Y_enu  (North = ENU's North/Y)
           #   Y_ned = X_enu  (East = ENU's East/X)
           #   Z_ned = -Z_enu (Down = -Up)
           x_enu = msg.x
           y_enu = msg.y
           z_enu = msg.z
        
           x_ned = y_enu
           y_ned = x_enu
           z_ned = -z_enu
        
           self.target_pos_ned = [x_ned, y_ned, z_ned]
           self.get_logger().info(f"[COORD] ENU({x_enu:.1f},{y_enu:.1f},{z_enu:.1f}) -> NED({x_ned:.1f},{y_ned:.1f},{z_ned:.1f})")

    def action_cb(self, msg):
        cmd = msg.data.upper()
        if cmd == "TAKEOFF_CHECK":
            self.perform_safety_takeoff()
        elif cmd == "LAND":
            self.get_logger().info("執行降落")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        elif cmd == "DISARM":
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        elif cmd == "RESET_ORIGIN":
            # 設定新的原點參考為當前 Local Position（NED）
            self.origin_ref_ned = list(self.current_local_ned)
            self.get_logger().info(f"[ORIGIN] 重置原點為 Local NED {self.origin_ref_ned}")
    
    def scan_action_cb(self, msg):
        """接收筆電的掃描控制命令"""
        cmd = msg.data.upper()
        if cmd == "START_SCAN":
            self.get_logger().info("[SCAN] 啟動掃描")
            # 清空之前的掃描結果
            self.scan_results = {}
            self.scan_completed = False
            # 發送啟動命令給 fast_scan_node
            scan_msg = Bool()
            scan_msg.data = True
            self.scan_control_pub.publish(scan_msg)
        elif cmd == "STOP_SCAN":
            self.get_logger().info("[SCAN] 停止掃描")
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
            self.get_logger().info(f"[SCAN] 收到 {target_id} 結果: {status}")
            
            # 檢查是否所有 tracker 都已掃描完成
            if all(tracker in self.scan_results for tracker in self.expected_trackers):
                self.scan_completed = True
                self.get_logger().info(f"[SCAN] ✅ 掃描完成！結果: {self.scan_results}")
                # 發布掃描完成通知給筆電
                ready_msg = String()
                ready_msg.data = json.dumps({
                    "status": "completed",
                    "results": self.scan_results
                })
                self.scan_ready_pub.publish(ready_msg)
        except Exception as e:
            self.get_logger().error(f"[SCAN] 解析失敗: {e}")

    def perform_safety_takeoff(self):
        self.get_logger().info("開始起飛前檢查...")

        if not self.is_connected:
            self.get_logger().error("[FAIL] 未連接 PX4")
            return
        if not self.pre_flight_checks_pass:
            self.get_logger().error("[FAIL] Preflight Checks 未通過")
            return
        if not self.got_global_pos:
            self.get_logger().error("[FAIL] 無 GPS 數據")
            return
        if self.current_eph > 1.5:
            self.get_logger().error(f"[FAIL] GPS 精度不足: {self.current_eph:.2f}m")
            return

        # 判斷是否已在空中（Local NED z < -0.5 視為離地）
        in_air = False
        try:
            in_air = float(self.current_local_ned[2]) < -0.5
        except Exception:
            in_air = False

        # --- 檢查通過 ---
        if in_air:
            # 已在空中：僅保證 Offboard，不重設高度與重複 Arm
            self.get_logger().info(f"[PASS] 檢查通過，飛機已在空中 -> 僅切換 Offboard 模式")
            for _ in range(3):
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0) # Offboard
        else:
            self.get_logger().info(f"[PASS] 檢查通過 (EPH {self.current_eph:.2f}m) -> 執行 Offboard 起飛")
            # 1. 直接將目標高度設為 -5.0 (NED 座標，負數為上)
            self.target_pos_ned = [0.0, 0.0, -5.0]
            # 2. 強制切換 Offboard 並解鎖
            for _ in range(5):
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0) # Offboard
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0) # Arm

    def timer_callback(self):
        # 持續發送 Offboard 心跳
        self.publish_offboard_control_mode()
        # 持續發送目標位置 (這是關鍵，只要 target_pos_ned 改變，飛機就會動)
        self.publish_trajectory_setpoint(self.target_pos_ned[0], self.target_pos_ned[1], self.target_pos_ned[2])

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True; msg.velocity = False; msg.acceleration = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_ctrl_pub.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        # 注意：PX4 使用 NED 座標，z 為負數代表高度
        # 原點偏移僅套用於 X/Y，Z 不做累加避免高度疊加
        pos_ned = [
            float(self.origin_ref_ned[0] + x),
            float(self.origin_ref_ned[1] + y),
            float(z)
        ]
        msg.position = pos_ned
        msg.yaw = float('nan')
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.traj_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1; msg.param2 = param2; msg.command = command
        msg.target_system = self.drone_id + 1; msg.target_component = 1
        msg.source_system = 1; msg.source_component = 1; msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args); node = DroneNode()
    rclpy.spin(node); node.destroy_node(); rclpy.shutdown()