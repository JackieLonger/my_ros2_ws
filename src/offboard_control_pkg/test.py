import pyaudio

p = pyaudio.PyAudio()

print(f"\n{'Index':<5} {'Device Name':<40} {'In Channels'}")
print("-" * 60)

for i in range(p.get_device_count()):
    info = p.get_device_info_by_index(i)
    # 只顯示有輸入能力 (麥克風) 的裝置
    if info['maxInputChannels'] > 0:
        print(f"{i:<5} {info['name']:<40} {info['maxInputChannels']}")

p.terminate()
