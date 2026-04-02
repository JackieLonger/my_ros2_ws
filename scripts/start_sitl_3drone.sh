#!/bin/bash
# start_sitl_3drone.sh — 一鍵啟動 PX4 SITL 3 架無人機 + MicroXRCEAgent
#
# 使用方式：
#   cd ~/my_ros2_ws
#   bash scripts/start_sitl_3drone.sh
#
# 這個腳本會：
#   1. 啟動 Gazebo + 3 架 PX4 iris SITL 實例 (背景)
#   2. 啟動 MicroXRCEAgent (前景，Ctrl+C 結束全部)
#
# 啟動後，在其他終端機執行 ROS2 節點：
#   Terminal 2: ros2 launch offboard_control_pkg sitl_circle_test.launch.py
#   Terminal 3: ros2 run offboard_control_pkg multi_drone_sender --ros-args -p drone_ids:="1,2,3"

set -e

PX4_DIR="$HOME/PX4-Autopilot"
BUILD_DIR="$PX4_DIR/build/px4_sitl_default"

if [ ! -d "$PX4_DIR" ]; then
    echo "ERROR: PX4-Autopilot not found at $PX4_DIR"
    exit 1
fi

if [ ! -f "$BUILD_DIR/bin/px4" ]; then
    echo "ERROR: PX4 SITL not built. Run: cd $PX4_DIR && make px4_sitl_default gazebo-classic"
    exit 1
fi

echo "============================================"
echo "  PX4 SITL 3-Drone + MicroXRCEAgent"
echo "============================================"

# Cleanup function
cleanup() {
    echo ""
    echo ">>> Shutting down..."
    pkill -x px4 2>/dev/null || true
    pkill gzclient 2>/dev/null || true
    pkill gzserver 2>/dev/null || true
    pkill MicroXRCEAgent 2>/dev/null || true
    echo ">>> Done."
}
trap cleanup SIGINT SIGTERM EXIT

# Kill previous instances
echo ">>> Killing old instances..."
pkill -x px4 2>/dev/null || true
pkill gzclient 2>/dev/null || true
pkill gzserver 2>/dev/null || true
pkill MicroXRCEAgent 2>/dev/null || true
sleep 2

# Source Gazebo environment
source "$PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash" "$PX4_DIR" "$BUILD_DIR"
export PX4_SIM_MODEL=gazebo-classic_iris

# ROS2 Gazebo plugins (if ROS2 is available)
ros_args=""
if [[ -n "$ROS_VERSION" ]] && [ "$ROS_VERSION" == "2" ]; then
    ros_args="-s libgazebo_ros_init.so -s libgazebo_ros_factory.so"
fi

# Start Gazebo server
echo ">>> Starting Gazebo server (empty world)..."
gzserver "$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world" --verbose $ros_args &
GZSERVER_PID=$!
sleep 5

# Spawn 3 PX4 instances (instance 1, 2, 3)
for i in 1 2 3; do
    echo ">>> Spawning PX4 instance $i (iris_$i)..."

    # Create working directory
    working_dir="$BUILD_DIR/rootfs/$i"
    mkdir -p "$working_dir"

    # Start PX4
    pushd "$working_dir" > /dev/null
    "$BUILD_DIR/bin/px4" -i $i -d "$BUILD_DIR/etc" > out.log 2> err.log &
    popd > /dev/null

    # Generate SDF model
    Y=$((3 * $i))
    python3 \
        "$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/scripts/jinja_gen.py" \
        "$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf.jinja" \
        "$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic" \
        --mavlink_tcp_port $((4560 + $i)) \
        --mavlink_udp_port $((14560 + $i)) \
        --mavlink_id $((1 + $i)) \
        --gst_udp_port $((5600 + $i)) \
        --video_uri $((5600 + $i)) \
        --mavlink_cam_udp_port $((14530 + $i)) \
        --output-file "/tmp/iris_${i}.sdf"

    # Spawn model in Gazebo
    gz model --spawn-file="/tmp/iris_${i}.sdf" --model-name="iris_${i}" -x 0.0 -y "$Y" -z 0.83
    sleep 1
done

echo ""
echo ">>> 3 PX4 SITL instances launched (px4_1, px4_2, px4_3)"
echo ">>> Namespaces: /px4_1/fmu/*, /px4_2/fmu/*, /px4_3/fmu/*"
echo ""

# Start Gazebo client
echo ">>> Starting Gazebo client..."
gzclient &
sleep 2

# Start MicroXRCEAgent (foreground — all PX4 instances connect to port 8888)
echo ""
echo "============================================"
echo "  MicroXRCEAgent running on UDP port 8888"
echo "  Press Ctrl+C to stop everything"
echo "============================================"
echo ""
echo "  接下來請開新的終端機執行："
echo "  Terminal 2: cd ~/my_ros2_ws && source install/setup.bash && ros2 launch offboard_control_pkg sitl_circle_test.launch.py"
echo "  Terminal 3: cd ~/my_ros2_ws && source install/setup.bash && ros2 run offboard_control_pkg multi_drone_sender --ros-args -p drone_ids:='1,2,3'"
echo ""

MicroXRCEAgent udp4 -p 8888
