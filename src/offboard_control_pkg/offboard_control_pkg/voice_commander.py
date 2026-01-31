# 檔名: voice_commander.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from RealtimeSTT import AudioToTextRecorder
import ollama
import json
import threading
import sys

class VoiceCommander(Node):
    def __init__(self):
        super().__init__('voice_commander')
        self.publisher_ = self.create_publisher(String, '/cmd_voice', 10)
        self.get_logger().info("🎙️ 語音聲控節點啟動 (GPU Mode)...")
        
        # 啟動錄音執行緒
        self.recorder_thread = threading.Thread(target=self.start_recording)
        self.recorder_thread.daemon = True
        self.recorder_thread.start()

    def start_recording(self):
        # ==========================================
        # 關鍵修正: 設定麥克風 Index
        # 根據您之前的 arecord 測試結果:
        # 11 = 'default' (系統預設，通常最穩)
        # 8  = 'sof-hda-dsp' (硬體直連)
        # 如果沒聲音，請將 11 改成 8 試試看
        # ==========================================
        TARGET_DEVICE_INDEX = 8 

        try:
            self.get_logger().info(f"正在初始化 RealtimeSTT (Device Index: {TARGET_DEVICE_INDEX})...")
            
            # 初始化 RealtimeSTT
            recorder = AudioToTextRecorder(
                model="medium", 
                language="zh", 
                device="cuda", 
                spinner=False,
                input_device_index=TARGET_DEVICE_INDEX, # <--- 修正 ALSA 錯誤的關鍵
                silero_sensitivity=0.4,                 # 降低 VAD 靈敏度避免雜訊觸發
                post_speech_silence_duration=0.6        # 說完話後 0.6秒 視為結束
            )
            print(">>> 語音系統就緒，請下指令 (例如: '全體起飛', '1號機開始掃描')...")
            
            while rclpy.ok():
                text = recorder.text() # 阻塞等待語音
                if text:
                    self.process_instruction(text)
        
        except Exception as e:
            self.get_logger().error(f"❌ 麥克風或模型初始化失敗: {e}")
            self.get_logger().error("建議: 檢查 input_device_index 或執行 'sudo usermod -aG audio $USER'")

    def process_instruction(self, text):
        print(f"👂 收到語音: {text}")
        try:
            # 使用 Qwen 2.5 解析意圖
            # 注意: 請確保您的 Ollama 服務已經啟動 (ollama serve)
            response = ollama.chat(model='qwen2.5:3b', messages=[
                {
                    'role': 'system',
                    'content': (
                        "你是無人機控制中樞。將語音轉為 JSON，嚴格遵守以下規則：\n"
                        "1. target: 'ALL' 或 數字ID (如 1, 2)。\n"
                        "2. action: 'TAKEOFF'(起飛), 'LAND'(降落), 'MISSION'(移動任務), 'SCAN'(網格掃描), 'EXPORT'(匯出), 'DISARM'(上鎖)。\n"
                        "3. 範例: '全部起飛'->{'target':'ALL', 'action':'TAKEOFF'}\n"
                        "4. 範例: '一號機掃描'->{'target': 1, 'action':'SCAN'}\n"
                        "5. 只回傳 JSON 字串，不要其他廢話，不要 Markdown 格式。"
                    )
                },
                {'role': 'user', 'content': text}
            ])
            
            # 清理回應中的 markdown 符號 (有些模型會自作聰明加 ```json)
            raw_content = response['message']['content']
            json_str = raw_content.replace("```json", "").replace("```", "").strip()
            
            # 嘗試解析 JSON 確保格式正確再發送
            # 這一步是為了防止 LLM 吐出壞掉的 JSON 導致接收端崩潰
            valid_json = json.loads(json_str) 
            final_msg_str = json.dumps(valid_json)

            msg = String()
            msg.data = final_msg_str
            self.publisher_.publish(msg)
            print(f"🚀 發送指令: {final_msg_str}")

        except json.JSONDecodeError:
            print(f"⚠️ LLM 回傳了無效的 JSON: {raw_content}")
        except Exception as e:
            print(f"❌ 解析失敗: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()