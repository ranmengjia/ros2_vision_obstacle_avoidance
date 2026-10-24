#!/usr/bin/env bash
# v1.1 步骤8：三场景 A/B 对照实验
# 用法: bash tools/eval/run_ab.sh [每场景重复次数] [单次采集秒数]
set +u   # ROS 的 setup.bash 会引用未定义变量，不能开 nounset

RUNS=${1:-3}
DURATION=${2:-40}
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUT_DIR="$ROOT/docs/experiments/data"
mkdir -p "$OUT_DIR"

source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"
export ROS_DOMAIN_ID=42

cleanup() {
  pkill -9 -f "ros2 launch robot_description" 2>/dev/null
  pkill -9 -f "ros2 launch vision_perception" 2>/dev/null
  pkill -9 -f "perception_node --ros-args" 2>/dev/null
  pkill -9 -f "ros_gz_sim" 2>/dev/null
  pkill -9 -f "parameter_bridge" 2>/dev/null
  pkill -9 -f "robot_state_publisher" 2>/dev/null
  pkill -9 -f "gz sim" 2>/dev/null
  sleep 2
}
trap cleanup EXIT INT TERM

for SCENE in obstacle_course dim_course low_contrast_course; do
  for RUN in $(seq 1 "$RUNS"); do
    echo "=== 场景 $SCENE | 第 $RUN 次 ==="
    cleanup
    ros2 launch robot_description simulation.launch.py \
      world_name:="${SCENE}.world" > "/tmp/eval_sim_${SCENE}_${RUN}.log" 2>&1 &
    sleep 12
    ros2 launch vision_perception perception.launch.py \
      > "/tmp/eval_perc_${SCENE}_${RUN}.log" 2>&1 &
    sleep 5
    python3 "$ROOT/tools/eval/eval_session.py" \
      --scene "$SCENE" --run "$RUN" --duration "$DURATION" \
      --out "$OUT_DIR/${SCENE}_r${RUN}.json" \
      | tail -30
  done
done

cleanup
echo "=== 全部实验完成，数据目录: $OUT_DIR ==="
