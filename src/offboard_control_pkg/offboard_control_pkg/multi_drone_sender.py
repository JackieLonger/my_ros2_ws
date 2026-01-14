import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, ReliabilityPolicy
import sys, termios, tty, time
import threading
import json
import csv
from datetime import datetime
import os

class Colors:
    HEADER = '\033[95m'; GREEN = '\033[92m'; YELLOW = '\033[93m'; RED = '\033[91m'; ENDC = '\033[0m'

class MissionControl(Node):
    def __init__(self):
        super().__init__('mission_control')
        # 以 ROS 參數 drone_ids 支援多台無人機，預設 "1,2"
        self.declare_parameter('drone_ids', '1,2')
        ids_raw = self.get_parameter('drone_ids').get_parameter_value().string_value
        parsed_ids = []
        try:
            for token in ids_raw.split(','):
                token = token.strip()
                if token.isdigit():
                    parsed_ids.append(int(token))
        except Exception:
            parsed_ids = []
        # 去重與排序，並提供預設
        self.ids = sorted(set(parsed_ids)) if parsed_ids else [1, 2]
        self.target_mode = "ALL"
        
        # 參數設定
        # ROS2 ENU: Z 向上為正，移動距離統一為正值，方向在 setpoint 決定
        self.takeoff_height = 5.0   # 上升 5 公尺 (ENU)
        self.move_front = 5.0       # 前進距離
        self.move_back = 5.0        # 後退距離
        self.move_left = 5.0        # 左移距離
        self.move_right = 5.0       # 右移距離
        
        # 掃描模式參數
        self.scan_step = 3.0  # 掃描時每次移動距離（公尺）
        self.scan_timeout = 90.0  # 每個點等待掃描完成的超時時間（秒）
        
        self.action_pubs = {}
        self.setpoint_pubs = {}
        self.scan_action_pubs = {}  # 掃描控制
        self.scan_ready_subs = {}   # 掃描完成通知
        self.link_quality_subs = {}  # 訊號品質訂閱
        
        # 掃描狀態變數
        self.scan_ready_flags = {did: False for did in self.ids}  # 記錄每架無人機的掃描完成狀態
        self.mission_stop_flags = {did: False for did in self.ids}  # 任務停止標誌（per-drone）
        self.scan_stop_flags = {did: False for did in self.ids}     # 掃描停止標誌（per-drone）
        
        # 歷史訊號記錄：{drone_id: [{"position": (x,y,z), "tracker_id": "!xxx", "snr": 8.5, "rssi": -85, ...}, ...]}
        self.signal_history = {did: [] for did in self.ids}
        self.current_scan_position = {did: (0.0, 0.0, -5.0) for did in self.ids}  # 當前掃描位置
        self.current_scan_phase = {did: 'INIT' for did in self.ids}  # 當前掃描階段
        self.scan_origin = {did: (0.0, 0.0) for did in self.ids}  # 掃描原點
        self.best_positions = {did: None for did in self.ids}  # 最佳訊號位置：{"position": (x,y,z), "avg_snr": 10.5, ...}
        # 即時 CSV 紀錄檔（掃描開始時建立）：{drone_id: csv_path}
        self.scan_csv_paths = {did: None for did in self.ids}

        # 設定 CSV 紀錄目錄至工作區: my_ros2_ws/signal_log
        # 若目錄不存在則建立
        self.log_dir = "/home/jackiehp/my_ros2_ws/signal_log"
        try:
            os.makedirs(self.log_dir, exist_ok=True)
        except Exception:
            pass
        
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        for did in self.ids:
            self.action_pubs[did] = self.create_publisher(String, f'/px4_{did}/laptop/action', qos)
            self.setpoint_pubs[did] = self.create_publisher(Point, f'/px4_{did}/laptop/setpoint', qos)
            self.scan_action_pubs[did] = self.create_publisher(String, f'/px4_{did}/laptop/scan_action', qos)
            self.scan_ready_subs[did] = self.create_subscription(
                String, f'/px4_{did}/scan_ready', 
                lambda msg, drone_id=did: self.scan_ready_cb(msg, drone_id), qos
            )
            self.link_quality_subs[did] = self.create_subscription(
                String, f'/px4_{did}/link_quality',
                lambda msg, drone_id=did: self.link_quality_cb(msg, drone_id), qos
            )
        
        self.print_ui()

    def print_ui(self):
        print("\033c", end="")
        target_str = "【全體 All】" if self.target_mode == "ALL" else f"【單機 ID: {self.target_mode}】"
        print(f"{Colors.HEADER}=== 分離式控制台 (手動流程) ==={Colors.ENDC}")
        print(f"當前控制: {Colors.GREEN}{target_str}{Colors.ENDC}")
        print(f"--------------------------------------------")
        print(f"[T]     起飛 (Takeoff) -> 懸停")
        print(f"[Space] 開始移動 (Mission: 前進->返回)")
        print(f"{Colors.YELLOW}[F]     網格掃描+驗證 (Scan+Verify){Colors.ENDC}")
        print(f"[H]     匯出歷史為CSV檔案")
        print(f"[L]     降落 (Land)")
        print(f"[R]     上鎖重置 (Disarm) - 降落後必按")
        print(f"--------------------------------------------")
        ids_hint = "/".join(str(d) for d in self.ids)
        print(f"[{ids_hint}/A] 切換飛機")
        print(f"[Ctrl+C] 離開")

    def set_target(self, mode):
        self.target_mode = mode; self.print_ui()
    
    def scan_ready_cb(self, msg, drone_id):
        """接收無人機掃描完成通知"""
        try:
            data = json.loads(msg.data)
            if data.get('status') == 'completed':
                self.scan_ready_flags[drone_id] = True
                self.get_logger().info(f"✅ Drone {drone_id} 掃描完成")
                # 掃描完成後計算最佳位置
                self.calculate_best_position(drone_id)
        except Exception as e:
            self.get_logger().error(f"解析 scan_ready 失敗: {e}")
    
    def link_quality_cb(self, msg, drone_id):
        """接收並記錄訊號品質數據"""
        try:
            data = json.loads(msg.data)
            # 記錄當前位置的訊號數據
            record = {
                "timestamp": data.get("timestamp"),
                "drone_id": drone_id,
                "scan_phase": self.current_scan_phase[drone_id],
                "tracker_id": data.get("target_id"),
                "status": data.get("status"),
                "forward_rssi": data.get("forward_rssi", 0),
                "forward_snr": data.get("forward_snr", 0),
                "return_rssi": data.get("return_rssi", 0),
                "return_snr": data.get("return_snr", 0),
                "position": self.current_scan_position[drone_id],
                "origin": self.scan_origin[drone_id]
            }
            self.signal_history[drone_id].append(record)
            self.get_logger().info(
                f"📊 Drone {drone_id} [{record['scan_phase']}] @ {record['position']}: "
                f"{record['tracker_id']} F-SNR={record['forward_snr']:.1f} R-SNR={record['return_snr']:.1f}dB"
            )

            # 若已建立 CSV 檔，寫入即時紀錄
            csv_path = self.scan_csv_paths.get(drone_id)
            if csv_path:
                # 計算單筆 quality_score（來回平均 + 歸一化 + 權重 0.6/0.4）
                avg_snr = (float(record["forward_snr"]) + float(record["return_snr"])) / 2.0
                avg_rssi = (float(record["forward_rssi"]) + float(record["return_rssi"])) / 2.0
                norm_snr = max(0.0, min(1.0, (avg_snr + 10.0) / 25.0))
                norm_rssi = max(0.0, min(1.0, (avg_rssi + 120.0) / 100.0))
                quality_score = 0.6 * norm_snr + 0.4 * norm_rssi

                # 時間戳轉換成 yyyy-MM-dd HH:mm:ss.SSS
                try:
                    ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                except Exception:
                    ts = str(record.get("timestamp", datetime.now().timestamp()))

                pos_x, pos_y, pos_z = record["position"]
                org_x, org_y = record["origin"]

                row = [
                    ts,
                    str(drone_id),
                    record["scan_phase"],
                    record["tracker_id"],
                    record["status"],
                    f"{float(record['forward_rssi']):.1f}",
                    f"{float(record['forward_snr']):.2f}",
                    f"{float(record['return_rssi']):.1f}",
                    f"{float(record['return_snr']):.2f}",
                    f"{quality_score:.4f}",
                    f"{float(pos_x):.3f}", f"{float(pos_y):.3f}", f"{float(pos_z):.3f}",
                    f"{float(org_x):.3f}", f"{float(org_y):.3f}"
                ]
                try:
                    with open(csv_path, 'a', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow(row)
                except Exception as e:
                    self.get_logger().error(f"CSV 寫入失敗 ({csv_path}): {e}")
        except Exception as e:
            self.get_logger().error(f"解析 link_quality 失敗: {e}")
    
    def calculate_best_position(self, drone_id):
        """計算該無人機的最佳訊號位置（每個點位的來回 RSSI 和 SNR 各自平均後比較）"""
        if not self.signal_history[drone_id]:
            return
        
        # 按位置分組，收集每個點位的所有 Tracker 數據（包含失敗的掃描）
        position_records = {}
        for record in self.signal_history[drone_id]:
            pos = record["position"]
            if pos not in position_records:
                position_records[pos] = []
            
            # 記錄所有掃描結果，失敗的給予零值
            position_records[pos].append(record)
        
        # 計算每個位置的平均來回訊號品質
        position_stats = {}
        for pos, records in position_records.items():
            # 對每個 Tracker 記錄，計算來回平均
            avg_snr_list = []
            avg_rssi_list = []
            
            for rec in records:
                # 每筆記錄的來回平均（失敗的用零值）
                if rec["status"] == "Success":
                    avg_snr = (rec["forward_snr"] + rec["return_snr"]) / 2.0
                    avg_rssi = (rec["forward_rssi"] + rec["return_rssi"]) / 2.0
                else:
                    # 失敗的掃描給予最低品質評分
                    avg_snr = -10.0  # 最低 SNR
                    avg_rssi = -120.0  # 最低 RSSI
                avg_snr_list.append(avg_snr)
                avg_rssi_list.append(avg_rssi)
            
            # 該位置所有 Tracker 的平均值
            position_stats[pos] = {
                "avg_snr": sum(avg_snr_list) / len(avg_snr_list),
                "avg_rssi": sum(avg_rssi_list) / len(avg_rssi_list),
                "sample_count": len(records)
            }
        
        # 找出綜合評分最高的位置（SNR 60% + RSSI 40%）
        best_pos = None
        best_score = float('-inf')
        
        for pos, stats in position_stats.items():
            # 標準化 SNR：假設範圍 -10 到 15 dB
            norm_snr = (stats["avg_snr"] + 10) / 25.0  # 歸一化到 0-1
            norm_snr = max(0.0, min(1.0, norm_snr))
            
            # 標準化 RSSI：假設範圍 -120 到 -20 dBm
            norm_rssi = (stats["avg_rssi"] + 120) / 100.0  # 歸一化到 0-1
            norm_rssi = max(0.0, min(1.0, norm_rssi))
            
            # 綜合評分：SNR 60% + RSSI 40%
            combined_score = 0.6 * norm_snr + 0.4 * norm_rssi
            
            if combined_score > best_score:
                best_score = combined_score
                best_pos = pos
                self.best_positions[drone_id] = {
                    "position": pos,
                    "avg_snr": stats["avg_snr"],
                    "avg_rssi": stats["avg_rssi"],
                    "combined_score": combined_score,
                    "sample_count": stats["sample_count"]
                }
        
        if self.best_positions[drone_id]:
            best = self.best_positions[drone_id]
            self.get_logger().info(
                f"🏆 Drone {drone_id} 最佳位置: {best['position']} "
                f"(綜合評分: {best['combined_score']:.3f}, SNR: {best['avg_snr']:.1f}dB, "
                f"RSSI: {best['avg_rssi']:.1f}dBm, 樣本: {best['sample_count']})"
            )
    
    def send_scan_action(self, action):
        """發送掃描控制命令"""
        targets = self.ids if self.target_mode == "ALL" else [self.target_mode]
        msg = String(); msg.data = action
        for did in targets: 
            self.scan_action_pubs[did].publish(msg)

    def send_scan_action_per_drone(self, did, action):
        msg = String(); msg.data = action
        self.scan_action_pubs[did].publish(msg)

    def send_action(self, action):
        targets = self.ids if self.target_mode == "ALL" else [self.target_mode]
        msg = String(); msg.data = action
        for did in targets: self.action_pubs[did].publish(msg)

    def send_setpoint(self, x, y, z):
        targets = self.ids if self.target_mode == "ALL" else [self.target_mode]
        msg = Point(); msg.x = float(x); msg.y = float(y); msg.z = float(z)
        for did in targets: self.setpoint_pubs[did].publish(msg)

    def send_setpoint_per_drone(self, did, x, y, z):
        msg = Point(); msg.x = float(x); msg.y = float(y); msg.z = float(z)
        self.setpoint_pubs[did].publish(msg)

    # --- 功能 1: 僅起飛 ---
    def perform_takeoff(self):
        print(f"\n{Colors.YELLOW}>>> 執行起飛程序...{Colors.ENDC}")
        
        # 停止任何掃描（僅針對當前目標）
        targets = self.ids if self.target_mode == "ALL" else [self.target_mode]
        for did in targets:
            self.scan_stop_flags[did] = True
        self.send_scan_action("STOP_SCAN")
        
        # 重置原點與掃描相關數據
        for did in targets:
            self.current_scan_position[did] = (0.0, 0.0, self.takeoff_height)
            self.current_scan_phase[did] = 'INIT'
            self.scan_origin[did] = (0.0, 0.0)
            self.scan_ready_flags[did] = False
            self.signal_history[did] = []
            self.best_positions[did] = None
        
        print(f"{Colors.GREEN}>>> 已重置原點與掃描數據{Colors.ENDC}")
        
        # 發送原點重置與檢查指令，Drone 端更新原點後切換 Offboard 並飛到預設高度
        self.send_action("RESET_ORIGIN")
        self.send_action("TAKEOFF_CHECK")
        # 發送 setpoint 建立 offboard 原點
        time.sleep(0.5)
        self.send_setpoint(0.0, 0.0, self.takeoff_height)
        print(f">>> 指令已發送，等待無人機爬升至懸停高度...")

    # --- 功能 2: 執行移動任務 ---
    def perform_mission(self):
        # 停止任何掃描（僅針對當前目標）
        targets = self.ids if self.target_mode == "ALL" else [self.target_mode]
        for did in targets:
            self.scan_stop_flags[did] = True
        self.send_scan_action("STOP_SCAN")
        # 重置原點與掃描相關數據
        for did in targets:
            self.current_scan_position[did] = (0.0, 0.0, self.takeoff_height)
            self.current_scan_phase[did] = 'INIT'
            self.scan_origin[did] = (0.0, 0.0)
            self.scan_ready_flags[did] = False
            self.signal_history[did] = []
            self.best_positions[did] = None
        
        print(f"{Colors.GREEN}>>> 已重置原點與掃描數據{Colors.ENDC}")
        
        # 重置任務停止標誌（僅針對當前目標）
        for did in targets:
            self.mission_stop_flags[did] = False
            self.scan_stop_flags[did] = True
        
        def mission_thread():
            print(f"\n{Colors.YELLOW}>>> 開始執行移動任務...{Colors.ENDC}")

            # 確保已起飛/Offboard；若尚未起飛，先觸發一次起飛檢查
            self.send_action("RESET_ORIGIN")
            self.send_action("TAKEOFF_CHECK")
            time.sleep(5.0)
            if any(self.mission_stop_flags[did] for did in targets):
                return
            # 使用各無人機當前高度，不再強制重設高度
            z_current = {}
            for did in targets:
                z_current[did] = self.current_scan_position[did][2]
            
            # 1. 前進
            print(f">>> 前進 {self.move_front} 米...")
            for did in targets:
                self.send_setpoint_per_drone(did, self.move_front, 0.0, z_current[did])
            time.sleep(8.0) # 等待飛行
            if any(self.mission_stop_flags[did] for did in targets):
                return

            #back
            print(f">>> 後退 {self.move_back} 米...")
            for did in targets:
                self.send_setpoint_per_drone(did, -self.move_back, 0.0, z_current[did])
            time.sleep(8.0)
            if any(self.mission_stop_flags[did] for did in targets):
                return

            #left
            print(f">>> 左移 {self.move_left} 米...")
            for did in targets:
                self.send_setpoint_per_drone(did, 0.0, self.move_left, z_current[did])
            time.sleep(8.0)
            if any(self.mission_stop_flags[did] for did in targets):
                return
            
            #right
            print(f">>> 右移 {self.move_right} 米...")
            for did in targets:
                self.send_setpoint_per_drone(did, 0.0, -self.move_right, z_current[did])
            time.sleep(8.0)
            if any(self.mission_stop_flags[did] for did in targets):
                return

            # 2. 返回
            print(f">>> 返回原點...")
            for did in targets:
                self.send_setpoint_per_drone(did, 0.0, 0.0, z_current[did])
            time.sleep(8.0)
            if any(self.mission_stop_flags[did] for did in targets):
                return
            
            print(f"{Colors.GREEN}>>> 移動結束，目前懸停中{Colors.ENDC}")
            print("提示：可按 [L] 降落，或再次按 [Space] 重跑移動")
            
        t = threading.Thread(target=mission_thread)
        t.start()

    # --- 功能 3: 降落 ---
    def perform_land(self):
        print(f"\n{Colors.RED}>>> 執行降落...{Colors.ENDC}")
        
        # 停止正在執行的任務（僅針對當前目標）
        targets = self.ids if self.target_mode == "ALL" else [self.target_mode]
        for did in targets:
            self.mission_stop_flags[did] = True
            self.scan_stop_flags[did] = True
        self.send_scan_action("STOP_SCAN")
        time.sleep(0.5)  # 等待執行緒檢查標誌
        
        self.send_action("LAND")
        
        # 降落時重置原點與掃描相關數據
        for did in targets:
            self.current_scan_position[did] = (0.0, 0.0, self.takeoff_height)
            self.current_scan_phase[did] = 'INIT'
            self.scan_origin[did] = (0.0, 0.0)
            self.scan_ready_flags[did] = False
            self.signal_history[did] = []
            self.best_positions[did] = None
        
        # 發送原點重置與 setpoint (0,0,0) 重置 offboard 原點
        time.sleep(1.0)  # 等待降落指令執行
        self.send_action("RESET_ORIGIN")
        self.send_setpoint(0.0, 0.0, 0.0)
        
        print(f"{Colors.GREEN}>>> 已重置原點與掃描數據{Colors.ENDC}")

    # --- 功能 4: 上鎖 (重置) ---
    def perform_disarm(self):
        print(f"\n{Colors.RED}>>> 強制上鎖 (Disarm){Colors.ENDC}")
        self.send_action("DISARM")
        print(">>> 狀態已重置，可重新執行起飛 [T]")
    
    # --- 功能 6: 網格掃描+驗證最佳位置 ---
    def perform_grid_scan(self):
        """
        網格掃描+驗證模式：
        1. 記錄當前位置為起點 (x0, y0, z0)
        2. 移動順序：前 -> 後 -> 左 -> 右
        3. 每次移動後，啟動掃描並等待所有無人機報告掃描完成
        4. 掃描完成後，移動到最佳位置並重新掃描驗證
        """
        # 開始新的掃描：停止任何正在執行的移動任務，清除掃描停止標誌（僅針對當前目標）
        targets = self.ids if self.target_mode == "ALL" else [self.target_mode]
        for did in targets:
            self.mission_stop_flags[did] = True
            self.scan_stop_flags[did] = False
        # 重置原點（Drone端會將原點設為當前位置，避免掃描開始時座標偏移）
        self.send_action("RESET_ORIGIN")
        # 保險起見，停止上一輪掃描
        self.send_scan_action("STOP_SCAN")
        
        # 注意：targets 在這裡已經定義，下面不用再定義

        # 為本次掃描建立每機 CSV 檔案
        try:
            os.makedirs(self.log_dir, exist_ok=True)
        except Exception:
            pass
        session_ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        for did in targets:
            path = os.path.join(self.log_dir, f"flight_{session_ts}_drone{did}.csv")
            self.scan_csv_paths[did] = path
            try:
                with open(path, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        'timestamp','drone_id','scan_phase','target_id','status',
                        'forward_rssi','forward_snr','return_rssi','return_snr','quality_score',
                        'pos_x','pos_y','pos_z','origin_x','origin_y'
                    ])
            except Exception as e:
                self.get_logger().error(f"CSV 檔建立失敗 ({path}): {e}")
        
        def scan_thread():
            print(f"\n{Colors.YELLOW}{'='*60}{Colors.ENDC}")
            print(f"{Colors.YELLOW}>>> 開始網格掃描+驗證模式...{Colors.ENDC}")
            print(f"{Colors.YELLOW}{'='*60}{Colors.ENDC}\n")
            
            # 當前位置作為起點 (假設已經在懸停狀態)
            base_x, base_y = 0.0, 0.0
            # 使用每台無人機當前高度
            z_current = {}
            targets = self.ids if self.target_mode == "ALL" else [self.target_mode]
            for did in targets:
                z_current[did] = self.current_scan_position[did][2]
            
            
            # 記錄掃描原點
            for did in targets:
                self.scan_origin[did] = (base_x, base_y)
            
            # 定義掃描點位 (相對於起點的偏移)
                # ROS2 ENU: X=forward, Y=left, Z=up
            scan_points = [
                (base_x + self.scan_step, base_y, "FORWARD"),  # 前 (+x)
                (base_x - self.scan_step, base_y, "BACKWARD"), # 後 (-x)
                    (base_x, base_y + self.scan_step, "LEFT"),     # 左 (+y) 
                    (base_x, base_y - self.scan_step, "RIGHT"),    # 右 (-y)
            ]
            
            # 階段 1: 網格掃描
            print(f"{Colors.GREEN}【階段 1/2】網格掃描{Colors.ENDC}\n")
            for i, (x, y, phase) in enumerate(scan_points, 1):
                if any(self.scan_stop_flags[did] for did in targets):
                    print(f"{Colors.RED}>>> 掃描中止{Colors.ENDC}")
                    return
                print(f"{Colors.YELLOW}--- 點 {i}/4: [{phase}] ({x:.1f}, {y:.1f}) ---{Colors.ENDC}")
                
                # 1. 移動到目標點
                # 1. 移動到目標點（保持各自當前高度）
                for did in targets:
                    self.send_setpoint_per_drone(did, x, y, z_current[did])
                # 更新當前掃描位置和階段（保持各自當前高度）
                for did in targets:
                    self.current_scan_position[did] = (x, y, z_current[did])
                    self.current_scan_phase[did] = phase
                
                print(f">>> 等待飛行到位... (5秒)")
                time.sleep(5.0)
                if any(self.scan_stop_flags[did] for did in targets):
                    print(f"{Colors.RED}>>> 掃描中止{Colors.ENDC}")
                    return
                
                # 2. 清空之前的掃描狀態（僅針對當前目標）
                for did in targets:
                    self.scan_ready_flags[did] = False
                
                # 3. 啟動掃描
                print(f">>> 啟動 Meshtastic 掃描...")
                self.send_scan_action("START_SCAN")
                
                # 4. 等待所有無人機掃描完成
                print(f">>> 等待掃描完成... (最多 {self.scan_timeout:.0f}秒)")
                start_time = time.time()
                
                while time.time() - start_time < self.scan_timeout:
                    if any(self.scan_stop_flags[did] for did in targets):
                        print(f"{Colors.RED}>>> 掃描中止{Colors.ENDC}")
                        return
                    if all(self.scan_ready_flags.get(did, False) for did in targets):
                        print(f"{Colors.GREEN}>>> ✅ 所有無人機掃描完成！{Colors.ENDC}")
                        break
                    time.sleep(0.5)
                else:
                    print(f"{Colors.RED}>>> ⚠️  掃描超時，繼續下一個點{Colors.ENDC}")
                
                # 5. 停止掃描
                self.send_scan_action("STOP_SCAN")
                print("")
            
            # 階段 2: 驗證最佳位置
            print(f"\n{Colors.GREEN}{'='*60}{Colors.ENDC}")
            print(f"{Colors.GREEN}【階段 2/2】驗證最佳位置{Colors.ENDC}")
            print(f"{Colors.GREEN}{'='*60}{Colors.ENDC}\n")
            
            for drone_id in targets:
                if not self.best_positions[drone_id]:
                    print(f"{Colors.RED}>>> Drone {drone_id} 無最佳位置數據，跳過驗證{Colors.ENDC}")
                    continue
                
                best = self.best_positions[drone_id]
                best_pos = best["position"]
                x, y, z = best_pos
                
                print(f"{Colors.YELLOW}=== Drone {drone_id} 驗證 ==={Colors.ENDC}")
                print(f">>> 移動到最佳位置 {best_pos}")
                print(f"    (歷史綜合評分: {best['combined_score']:.3f}, SNR: {best['avg_snr']:.1f}dB, RSSI: {best['avg_rssi']:.1f}dBm)")
                
                # 1. 移動到最佳位置（僅針對該無人機）
                self.send_setpoint_per_drone(drone_id, x, y, z)
                self.current_scan_position[drone_id] = best_pos
                self.current_scan_phase[drone_id] = "VERIFY"
                print(f">>> 等待飛行到位... (5秒)")
                time.sleep(5.0)
                
                # 2. 清空掃描狀態
                self.scan_ready_flags[drone_id] = False
                
                # 3. 啟動驗證掃描（僅針對該無人機）
                print(f">>> 啟動驗證掃描...")
                self.send_scan_action_per_drone(drone_id, "START_SCAN")
                
                # 4. 等待掃描完成
                print(f">>> 等待掃描完成... (最多 {self.scan_timeout:.0f}秒)")
                start_time = time.time()
                
                while time.time() - start_time < self.scan_timeout:
                    if self.scan_ready_flags.get(drone_id, False):
                        print(f"{Colors.GREEN}>>> ✅ Drone {drone_id} 驗證掃描完成！{Colors.ENDC}")
                        # 驗證完成後，將最佳位置設為新的原點
                        self.send_action("RESET_ORIGIN")
                        break
                    time.sleep(0.5)
                else:
                    print(f"{Colors.RED}>>> ⚠️  Drone {drone_id} 掃描超時{Colors.ENDC}")
                
                # 5. 停止掃描（僅針對該無人機）
                self.send_scan_action_per_drone(drone_id, "STOP_SCAN")
                
                # 6. 比較驗證結果
                print(f"\n{Colors.YELLOW}>>> 驗證結果比較:{Colors.ENDC}")
                # 找出該位置的最新掃描記錄（驗證階段）
                verify_records = [r for r in self.signal_history[drone_id] 
                                 if r["position"] == best_pos and r["scan_phase"] == "VERIFY" 
                                 and r["status"] == "Success"]
                
                if len(verify_records) >= 2:  # 至少有兩個 Tracker 的驗證結果
                    # 計算驗證階段的平均來回訊號
                    verify_avg_snr_list = []
                    verify_avg_rssi_list = []
                    for rec in verify_records:
                        verify_avg_snr_list.append((rec["forward_snr"] + rec["return_snr"]) / 2.0)
                        verify_avg_rssi_list.append((rec["forward_rssi"] + rec["return_rssi"]) / 2.0)
                    
                    verify_snr = sum(verify_avg_snr_list) / len(verify_avg_snr_list)
                    verify_rssi = sum(verify_avg_rssi_list) / len(verify_avg_rssi_list)
                    
                    # 計算驗證階段的綜合評分（SNR 60% + RSSI 40%）
                    norm_verify_snr = (verify_snr + 10) / 25.0
                    norm_verify_snr = max(0.0, min(1.0, norm_verify_snr))
                    norm_verify_rssi = (verify_rssi + 120) / 100.0
                    norm_verify_rssi = max(0.0, min(1.0, norm_verify_rssi))
                    verify_score = 0.6 * norm_verify_snr + 0.4 * norm_verify_rssi
                    
                    print(f"    歷史綜合評分: {best['combined_score']:.3f} (SNR: {best['avg_snr']:.1f}dB, RSSI: {best['avg_rssi']:.1f}dBm)")
                    print(f"    驗證綜合評分: {verify_score:.3f} (SNR: {verify_snr:.1f}dB, RSSI: {verify_rssi:.1f}dBm)")
                    diff = verify_score - best['combined_score']
                    if abs(diff) < 0.05:  # 綜合評分差異小於 0.05
                        print(f"    {Colors.GREEN}✅ 驗證成功！位置穩定 (誤差 {diff:+.3f}){Colors.ENDC}")
                    else:
                        print(f"    {Colors.YELLOW}⚠️  訊號差異較大 (誤差 {diff:+.3f}){Colors.ENDC}")
                print("")
            
            # 返回起點（各自回到起始高度）
            print(f"{Colors.GREEN}>>> 返回起點...{Colors.ENDC}")
            for did in targets:
                self.send_setpoint_per_drone(did, base_x, base_y, z_current[did])
            time.sleep(5.0)
            
            print(f"\n{Colors.GREEN}{'='*60}{Colors.ENDC}")
            print(f"{Colors.GREEN}>>> 網格掃描+驗證完成！{Colors.ENDC}")
            print(f"{Colors.GREEN}{'='*60}{Colors.ENDC}\n")
            print("提示：按 [H] 匯出 CSV，按 [L] 降落")
        
        t = threading.Thread(target=scan_thread)
        t.start()
    
    # --- 功能 7: 匯出訊號歷史為 CSV ---
    def export_signal_history_csv(self):
        """將每架無人機的訊號歷史記錄匯出為獨立的 CSV 檔案"""
        print(f"\n{Colors.YELLOW}>>> 開始匯出訊號歷史...{Colors.ENDC}")
        
        # 使用工作區的 signal_log 目錄
        log_dir = self.log_dir
        os.makedirs(log_dir, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        for drone_id in self.ids:
            if not self.signal_history[drone_id]:
                print(f"{Colors.YELLOW}Drone {drone_id}: 無記錄，跳過{Colors.ENDC}")
                continue
            
            filename = f"flight_{timestamp}_drone{drone_id}.csv"
            filepath = os.path.join(log_dir, filename)
            
            try:
                with open(filepath, 'w', newline='') as csvfile:
                    fieldnames = [
                        'timestamp', 'drone_id', 'scan_phase', 'target_id', 'status',
                        'forward_rssi', 'forward_snr', 'return_rssi', 'return_snr',
                        'quality_score', 'pos_x', 'pos_y', 'pos_z', 'origin_x', 'origin_y'
                    ]
                    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                    writer.writeheader()
                    
                    for record in self.signal_history[drone_id]:
                        # 計算 quality_score（綜合評分的標準化值）
                        forward_snr = record['forward_snr']
                        return_snr = record['return_snr']
                        # 假設 SNR 範圍 -10 到 15 dB，標準化到 0-1
                        quality_score = ((forward_snr + return_snr) / 2.0 + 10) / 25.0
                        quality_score = max(0.0, min(1.0, quality_score))  # 限制在 0-1 範圍
                        
                        pos = record['position']
                        origin = record['origin']
                        
                        writer.writerow({
                            'timestamp': record['timestamp'],
                            'drone_id': record['drone_id'],
                            'scan_phase': record['scan_phase'],
                            'target_id': record['tracker_id'],
                            'status': record['status'],
                            'forward_rssi': f"{record['forward_rssi']:.1f}",
                            'forward_snr': f"{record['forward_snr']:.2f}",
                            'return_rssi': f"{record['return_rssi']:.1f}",
                            'return_snr': f"{record['return_snr']:.2f}",
                            'quality_score': f"{quality_score:.4f}",
                            'pos_x': f"{pos[0]:.3f}",
                            'pos_y': f"{pos[1]:.3f}",
                            'pos_z': f"{pos[2]:.3f}",
                            'origin_x': f"{origin[0]:.3f}",
                            'origin_y': f"{origin[1]:.3f}"
                        })
                
                print(f"{Colors.GREEN}✅ Drone {drone_id}: {filepath} ({len(self.signal_history[drone_id])} 筆記錄){Colors.ENDC}")
            
            except Exception as e:
                print(f"{Colors.RED}❌ Drone {drone_id} 匯出失敗: {e}{Colors.ENDC}")
        
        print(f"\n{Colors.GREEN}>>> 匯出完成！檔案位置: {log_dir}{Colors.ENDC}")
        input("按 Enter 返回...")
        self.print_ui()
    
def get_key():
    fd = sys.stdin.fileno(); old = termios.tcgetattr(fd)
    try: tty.setraw(fd); ch = sys.stdin.read(1)
    finally: termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

def main(args=None):
    rclpy.init(args=args); node = MissionControl()
    try:
        while rclpy.ok():
            key = get_key()
            if key.isdigit():
                did = int(key)
                if did in node.ids:
                    node.set_target(did)
            elif key == 'a' or key == 'A': node.set_target("ALL")
            
            elif key == 't' or key == 'T': node.perform_takeoff()  # 獨立起飛
            elif key == ' ': node.perform_mission()                # 獨立移動
            elif key == 'f' or key == 'F': node.perform_grid_scan()  # 網格掃描+驗證
            elif key == 'h' or key == 'H': node.export_signal_history_csv()  # 匯出 CSV
            elif key == 'l' or key == 'L': node.perform_land()     # 獨立降落
            elif key == 'r' or key == 'R': node.perform_disarm()   # 獨立上鎖
            
            elif key == '\x03': break
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()