#!/usr/bin/env bash
# sim2sim 三种配置（把"腿"和"臂"的数据来源拆开，方便分开调 / 合并调）
#
#   bash src/M20_sdk_deploy/scripts/run_sim2sim.sh full     # 全仿真（默认）：MuJoCo 同时发腿和臂
#   bash src/M20_sdk_deploy/scripts/run_sim2sim.sh legs     # 只调腿：MuJoCo 不发臂反馈，
#                                                          #   臂由 arm_stub_node 保持默认姿态
#   bash src/M20_sdk_deploy/scripts/run_sim2sim.sh mixed    # 腿仿真 + 臂走"外部传输"：
#                                                          #   arm_controller(IK) + arm_teleop(键盘)
#                                                          #   + 假 CAN 驱动（或真机 agx 驱动）
#
# 共同点：rl_deploy 看到的接口完全一样（腿走 /JOINTS_DATA，臂走 /ARM_JOINTS_DATA），
# 所以"分开调"得到的结论可以直接用在"合并调"和真机上。
#
# 环境变量：
#   M20_USE_VIEWER=1        开 MuJoCo 窗口（默认 0 = 无头）
#   M20_ARM_TRANSPORT=real  mixed 配置下用真机臂（需要先起 agx_arm_ctrl），
#                           默认 sim = 用 arm_stub_node 的"假 CAN 驱动"
#   M20_ROS_DOMAIN_ID       默认 1
#
# Ctrl-C 退出会一并收掉本脚本启动的所有节点。臂的键盘控制在**另一个终端**：
#   python3 src/M20_sdk_deploy/interface/robot/simulation/arm_teleop_node.py

set -e
cd "$(dirname "$0")/../../.." || exit 1     # 仓库根目录
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID="${M20_ROS_DOMAIN_ID:-1}"

CONFIG="${1:-full}"
SIM=src/M20_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py
STUB=src/M20_sdk_deploy/interface/robot/simulation/arm_stub_node.py
ARM_CTRL=src/M20_sdk_deploy/interface/robot/simulation/arm_controller.py
ARM_REAL=src/M20_sdk_deploy/interface/robot/simulation/arm_real_adapter.py

PIDS=()
start() {  # start <名字> <命令...>
  local name="$1"; shift
  echo "[run_sim2sim] 启动 $name: $*"
  "$@" >"/tmp/run_sim2sim_${name}.log" 2>&1 &
  PIDS+=("$!")
}
cleanup() {
  echo
  echo "[run_sim2sim] 收工（Ctrl-C）..."
  for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null || true; done
  sleep 1
  for p in "${PIDS[@]}"; do kill -9 "$p" 2>/dev/null || true; done
  # 兜底：按"本仓库的这几个仿真脚本"再扫一遍（避免 rclpy 节点没退出干净）
  pkill -9 -f "M20_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py" 2>/dev/null || true
  pkill -9 -f "M20_sdk_deploy/interface/robot/simulation/arm_stub_node.py" 2>/dev/null || true
  pkill -9 -f "M20_sdk_deploy/interface/robot/simulation/arm_controller.py" 2>/dev/null || true
  wait 2>/dev/null || true
  exit 0
}
trap cleanup INT TERM

case "$CONFIG" in
  full)
    echo "[run_sim2sim] 配置 full：MuJoCo 同时提供腿和臂（最接近训练时的闭环）"
    start sim python3 "$SIM"
    ;;
  legs)
    echo "[run_sim2sim] 配置 legs：只调腿。MuJoCo 不发 /ARM_JOINTS_DATA，"
    echo "              臂由假臂节点保持默认姿态（等价于『臂没装』）。"
    M20_SIM_NO_ARM_DATA=1 start sim python3 "$SIM"
    start arm_stub python3 "$STUB"
    ;;
  mixed)
    echo "[run_sim2sim] 配置 mixed：腿来自 MuJoCo，臂走独立传输。"
    M20_SIM_NO_ARM_DATA=1 start sim python3 "$SIM"
    if [ "${M20_ARM_TRANSPORT:-sim}" = "real" ]; then
      echo "[run_sim2sim] 臂传输 = real（需要先起 agx_arm_ctrl；本脚本只起桥接）"
      start arm_real python3 "$ARM_REAL"
    else
      echo "[run_sim2sim] 臂传输 = sim（假 CAN 驱动：按 /ARM_JOINTS_CMD 一阶跟随）"
      M20_ARM_STUB_FOLLOW=1 start arm_stub python3 "$STUB"
    fi
    start arm_ctrl python3 "$ARM_CTRL"
    echo "[run_sim2sim] 机械臂键盘请在**另一个终端**运行（需要 TTY）："
    echo "               python3 src/M20_sdk_deploy/interface/robot/simulation/arm_teleop_node.py"
    ;;
  *)
    echo "用法: $0 {full|legs|mixed}"
    exit 2
    ;;
esac

echo "[run_sim2sim] 再开一个终端跑策略："
echo "               ros2 run m20_sdk_deploy rl_deploy   # z 站立 → c 进 RL"
echo "[run_sim2sim] 日志：/tmp/run_sim2sim_*.log ；Ctrl-C 收工。"
wait
