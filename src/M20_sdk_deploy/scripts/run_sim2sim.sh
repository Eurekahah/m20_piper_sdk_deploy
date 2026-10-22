#!/usr/bin/env bash
# sim2sim 三种配置（把"腿"和"臂"的数据来源拆开，方便分开调 / 合并调）
#
#   bash src/M20_sdk_deploy/scripts/run_sim2sim.sh full     # 全仿真（默认）：MuJoCo 同时发腿和臂
#   bash src/M20_sdk_deploy/scripts/run_sim2sim.sh legs     # 只调腿：MuJoCo 不发臂反馈，
#                                                          #   臂由 arm_stub_node 保持默认姿态
#   bash src/M20_sdk_deploy/scripts/run_sim2sim.sh arm      # **只调臂**：本脚本只起仿真；
#                                                          #   IK + 键盘由统一控制终端启动
#   bash src/M20_sdk_deploy/scripts/run_sim2sim.sh mixed    # 腿仿真 + 臂走"外部传输"：
#                                                          #   假 CAN 驱动（或真机 agx 驱动）；
#                                                          #   IK + 键盘 + 策略由统一控制终端启动
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
# Ctrl-C 退出会一并收掉本脚本启动的所有节点。IK、策略和臂键盘统一在另一个终端：
#   python3 src/M20_sdk_deploy/scripts/keyboard_teleop_supervisor.py --profile sim

set -e
cd "$(dirname "$0")/../../.." || exit 1     # 仓库根目录
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID="${M20_ROS_DOMAIN_ID:-1}"

if [ "${M20_SIM2SIM_ALLOW_STALE:-0}" != "1" ]; then
  stale_nodes="$(pgrep -af '[m]ujoco_simulation_ros2.py|install/m20_sdk_deploy/lib/m20_sdk_deploy/[r]l_deploy' || true)"
  if [ -n "$stale_nodes" ]; then
    echo "[run_sim2sim] 检测到残留仿真或策略进程，拒绝启动第二个实例：" >&2
    echo "$stale_nodes" >&2
    echo "[run_sim2sim] 先结束这些进程，或显式设置 M20_SIM2SIM_ALLOW_STALE=1" >&2
    exit 2
  fi
fi

CONFIG="${1:-full}"
SIM=src/M20_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py
STUB=src/M20_sdk_deploy/interface/robot/simulation/arm_stub_node.py
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
  # 兜底：只再扫**本脚本一定会自己启动的那个**（MuJoCo 仿真节点），避免 rclpy 没退干净。
  # ⚠️ 不要按名字去 pkill arm_controller.py / arm_stub_node.py：
  #   按用法文档，arm_controller(IK) 是操作员**在另一个终端自己起的**，这里一刀切会把
  #   他的终端进程杀掉（表现就是"我在 T1 按了 Ctrl+C，IK 那个终端没了"）。
  #   本脚本自己起的那几个已经在上面的 PIDS 循环里收掉了。
  #   需要恢复"全清"的老行为：M20_SIM2SIM_CLEANUP_ALL=1
  pkill -9 -f "M20_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py" 2>/dev/null || true
  if [ "${M20_SIM2SIM_CLEANUP_ALL:-0}" = "1" ]; then
    pkill -9 -f "M20_sdk_deploy/interface/robot/simulation/arm_stub_node.py" 2>/dev/null || true
    pkill -9 -f "M20_sdk_deploy/interface/robot/simulation/arm_controller.py" 2>/dev/null || true
  fi
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
    start sim env M20_SIM_NO_ARM_DATA=1 python3 "$SIM"
    start arm_stub python3 "$STUB"
    ;;
  arm)
    echo "[run_sim2sim] 配置 arm：只调臂 —— 本终端只起 MuJoCo；"
    echo "              IK + 臂键盘由统一控制终端启动。"
    start sim python3 "$SIM"
    echo "[run_sim2sim] 统一控制终端（IK + 臂键盘，不起 rl_deploy）："
    echo "               python3 src/M20_sdk_deploy/scripts/keyboard_teleop_supervisor.py --profile arm"
    wait
    ;;
  mixed)
    echo "[run_sim2sim] 配置 mixed：腿来自 MuJoCo，臂走独立传输。"
    start sim env M20_SIM_NO_ARM_DATA=1 python3 "$SIM"
    if [ "${M20_ARM_TRANSPORT:-sim}" = "real" ]; then
      echo "[run_sim2sim] 臂传输 = real（需要先起 agx_arm_ctrl；本脚本只起桥接）"
      start arm_real python3 "$ARM_REAL"
    else
      echo "[run_sim2sim] 臂传输 = sim（假 CAN 驱动：按 /ARM_JOINTS_CMD 一阶跟随）"
      start arm_stub env M20_ARM_STUB_FOLLOW=1 python3 "$STUB"
    fi
    echo "[run_sim2sim] IK、臂键盘和 rl_deploy 在统一控制终端启动："
    echo "               python3 src/M20_sdk_deploy/scripts/keyboard_teleop_supervisor.py --profile sim"
    ;;
  *)
    echo "用法: $0 {full|legs|arm|mixed}"
    exit 2
    ;;
esac

echo "[run_sim2sim] 再开一个统一控制终端（策略 + IK + 臂键盘）："
echo "               python3 src/M20_sdk_deploy/scripts/keyboard_teleop_supervisor.py --profile sim"
echo "               legs 配置改用 --profile legs；arm 配置改用 --profile arm"
echo "[run_sim2sim] 日志：/tmp/run_sim2sim_*.log ；Ctrl-C 收工。"
wait
