#!/bin/bash
# run_all_experiments.sh - 按顺序运行实验 1-7
# 自动发送 Enter 键跳过交互步骤

set -e
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}] {message}"

source /opt/ros/humble/setup.bash
source /home/jetson/ros2_ws/install/setup.bash

LOG_DIR="/home/jetson/exp1_logs"
mkdir -p "$LOG_DIR"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

# 实验配置：(ros2_run命令, 名称, 日志前缀, 额外参数)
EXPERIMENTS=(
    "exp1_calibration:Experiment 1 - 传感器标定:exp1_calibration:--模拟模式跳过交互"
    "exp2_power:Experiment 2 - 功率分析:exp2_power:--无交互"
    "exp3_slam:Experiment 3 - SLAM建图:exp3_slam:--有交互"
    "exp4_maze:Experiment 4 - 迷宫导航:exp4_maze:--有交互"
    "exp5_fault:Experiment 5 - 障碍容错:exp5_fault:--有交互"
    "exp6_cnn:Experiment 6 - CNN检测:exp6_cnn:--有交互"
    "exp7_pheromone:Experiment 7 - 信息素轨迹:exp7_pheromone:--自动模式"
)

run_experiment() {
    local cmd="$1"
    local name="$2"
    local prefix="$3"
    local extra="$4"

    echo ""
    echo "=========================================="
    echo "开始: $name"
    echo "=========================================="

    LOG_FILE="$LOG_DIR/${prefix}_${TIMESTAMP}.log"

    case "$cmd" in
        exp1_calibration)
            # 实验1: 自动发送 Enter 跳过所有交互
            timeout 300 bash -c "yes '' | ros2 run formica_experiments $cmd 2>&1 | tee $LOG_FILE" || true
            ;;
        exp2_power)
            # 实验2: 短时间运行 (5分钟) 用于测试
            MISSION_DURATION_S=300 timeout 320 bash -c "ros2 run formica_experiments $cmd 2>&1 | tee $LOG_FILE" || true
            ;;
        exp3_slam)
            # 实验3: 自动发送 Enter
            timeout 600 bash -c "yes '' | ros2 run formica_experiments $cmd 2>&1 | tee $LOG_FILE" || true
            ;;
        exp4_maze)
            # 实验4: 自动发送 Enter
            timeout 600 bash -c "yes '' | ros2 run formica_experiments $cmd 2>&1 | tee $LOG_FILE" || true
            ;;
        exp5_fault)
            # 实验5: 自动发送 Enter
            timeout 600 bash -c "yes '' | ros2 run formica_experiments $cmd 2>&1 | tee $LOG_FILE" || true
            ;;
        exp6_cnn)
            # 实验6: 自动发送 Enter
            timeout 600 bash -c "yes '' | ros2 run formica_experiments $cmd 2>&1 | tee $LOG_FILE" || true
            ;;
        exp7_pheromone)
            # 实验7: 使用模拟模式和自动运行
            timeout 600 bash -c "ros2 run formica_experiments $cmd --ros-args -p mock_sensors:=true -p auto_run:=true 2>&1 | tee $LOG_FILE" || true
            ;;
    esac

    echo ""
    echo "--- $name 完成 ---"
    echo "日志: $LOG_FILE"
}

echo "=========================================="
echo "FormicaBot 实验 1-7 自动化运行"
echo "时间: $(date)"
echo "日志目录: $LOG_DIR"
echo "=========================================="

TOTAL=${#EXPERIMENTS[@]}
for i in "${!EXPERIMENTS[@]}"; do
    idx=$((i+1))
    IFS=':' read -r cmd name prefix extra <<< "${EXPERIMENTS[$i]}"
    echo ""
    echo ">>> [$idx/$TOTAL] $name"
    run_experiment "$cmd" "$name" "$prefix" "$extra"
done

echo ""
echo "=========================================="
echo "所有实验运行完成！"
echo "完成时间: $(date)"
echo "=========================================="

echo ""
echo "--- 生成的日志文件 ---"
ls -lah "$LOG_DIR"/*_${TIMESTAMP}.log 2>/dev/null || echo "无日志文件"

echo ""
echo "--- 实验数据目录 ---"
ls -lah ~/formica_experiments/data/ 2>/dev/null | tail -20 || echo "无数据目录"
