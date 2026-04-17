#!/bin/bash
# run_all_simulations.sh
# 运行所有实验的模拟模式

set -e
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}] {message}"

source /opt/ros/humble/setup.bash
source /home/jetson/ros2_ws/install/setup.bash

LOG_DIR="/home/jetson/exp1_logs"
mkdir -p "$LOG_DIR"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

echo "=========================================="
echo "FormicaBot 实验 1-7 模拟模式运行"
echo "时间: $(date)"
echo "=========================================="

run_exp() {
    local name="$1"
    local cmd="$2"
    local extra="$3"
    local timeout="${4:-300}"
    
    echo ""
    echo ">>> 开始: $name"
    echo "    命令: ros2 run formica_experiments $cmd $extra"
    
    LOG_FILE="$LOG_DIR/${name}_${TIMESTAMP}.log"
    
    timeout "$timeout" bash -c "ros2 run formica_experiments $cmd $extra 2>&1 | tee $LOG_FILE" || local status=$?
    
    if [ -f "$LOG_FILE" ]; then
        local lines=$(wc -l < "$LOG_FILE")
        echo "    日志: $LOG_FILE ($lines 行)"
        if grep -q "PASS" "$LOG_FILE" 2>/dev/null; then
            echo "    结果: ✓ PASS"
        elif grep -q "FAIL" "$LOG_FILE" 2>/dev/null; then
            echo "    结果: ✗ FAIL"
        elif grep -q "complete" "$LOG_FILE" 2>/dev/null; then
            echo "    结果: ✓ 完成"
        fi
    fi
    echo ""
}

# 实验 1 - 传感器标定 (需要实际硬件，无法模拟)
echo ""
echo "=========================================="
echo ">>> 实验 1 (传感器标定) - 需要实际硬件"
echo "=========================================="
echo "跳过: ros2 run formica_experiments exp1_calibration"
echo "说明: 需要 LiDAR, IMU, 摄像头等实际传感器"

# 实验 2 - 功率分析 (支持 mock_power)
run_exp "exp2_power" "exp2_power" "--ros-args -p mock_power:=true -p enable_robot_motion:=false" "120"

# 实验 3 - SLAM 建图 (支持 mock_map)
run_exp "exp3_slam" "exp3_slam" "--ros-args -p mock_map:=true -p auto_run:=true" "180"

# 实验 4 - 迷宫导航 (支持 mock_nav)
run_exp "exp4_maze" "exp4_maze" "--ros-args -p mock_nav:=true" "120"

# 实验 5 - 障碍容错 (支持 mock_nav)
run_exp "exp5_fault" "exp5_fault" "--ros-args -p mock_nav:=true" "120"

# 实验 6 - CNN 检测 (支持 auto_run)
run_exp "exp6_cnn" "exp6_cnn" "--ros-args -p auto_run:=true" "120"

# 实验 7 - 信息素轨迹 (支持 mock_sensors + auto_run) - 已测试通过
run_exp "exp7_pheromone" "exp7_pheromone" "--ros-args -p mock_sensors:=true -p auto_run:=true" "180"

echo ""
echo "=========================================="
echo "所有模拟实验完成！"
echo "完成时间: $(date)"
echo "=========================================="

echo ""
echo "--- 生成的数据文件 ---"
ls -lah ~/formica_experiments/data/exp*_$(date +%Y%m%d)*.csv 2>/dev/null | head -20 || echo "无今日数据"

echo ""
echo "--- 实验日志文件 ---"
ls -lah "$LOG_DIR"/*_${TIMESTAMP}.log 2>/dev/null | head -20 || echo "无日志文件"