#!/usr/bin/env bash
# L0→L3 一键验收（在容器 m20_piper_ros 内、仓库根目录执行）
#
#   bash tests/run_all.sh            # 完整：编译 + L1 + L3' + 五档 L3
#   SKIP_BUILD=1 bash tests/run_all.sh
#   MODES="hold rl" bash tests/run_all.sh
#
# 判据（详细口径见 docs/DEVELOPMENT_zh.md）：
#   * L1  check_policy_interface.py   —— 布局/形状/名字/原生序/数值对照
#   * L3' check_mjcf_contract.py      —— MJCF 与训练配置逐项对照（不允许 FAIL）
#   * L3  每档：按契约文档的高度/倾角判据；
#         `rl` 档因为入口边缘发散（DEF-018），用 --repeat 判失败率 ≤ 1/3，其余档 0 失败。
# 注意：不要 `set -u` —— ROS 的 setup.bash 里会引用未定义变量，会把脚本打断
cd "$(dirname "$0")/.." || exit 1
source /opt/ros/humble/setup.bash
source install/setup.bash

SCRIPTS=src/M20_sdk_deploy/scripts
MODES=${MODES:-"hold wheel_step no_arm arm_only arm_mixed rl walk arm arm_move push"}
fail=0
step() { printf '\n=========== %s ===========\n' "$1"; }
report() { if [ "$1" -eq 0 ]; then echo "  -> OK: $2"; else echo "  -> FAIL: $2"; fail=1; fi }

if [ "${SKIP_BUILD:-0}" != "1" ]; then
  step "L0 编译"
  colcon build --packages-up-to m20_sdk_deploy \
    --cmake-args -DBUILD_PLATFORM=x86 -DSIM2SIM=ON >/tmp/run_all_build.log 2>&1
  report $? "colcon build（日志 /tmp/run_all_build.log）"
fi

step "L1 离线策略验收"
python3 "$SCRIPTS/check_policy_interface.py"
report $? "check_policy_interface.py"

step "L1 统一键盘控制终端"
python3 tests/keyboard_teleop_supervisor_check.py
report $? "keyboard_teleop_supervisor_check.py"

step "L3' MJCF 与训练配置对照"
python3 "$SCRIPTS/check_mjcf_contract.py"
report $? "check_mjcf_contract.py"

step "L3 命令组合扫描 command_sweep${SWEEP_ARGS:+ $SWEEP_ARGS}"
# 22 个命令阶段（vx 0.2~1.5 / vy / wz / 组合 / 机身姿态）连起来跑一遍，逐段判"有没有摔 + 跟不跟"。
# 比逐档 smoke 覆盖广得多，代价约 2.5 min。SWEEP=0 可跳过；SWEEP_ARGS 可传额外参数
# （例如 SWEEP_ARGS="--only 'vx +1.5'" 或 "--repeat 3"）。
if [ "${SWEEP:-1}" = "1" ]; then
  python3 tests/command_sweep.py --repeat 1 ${SWEEP_ARGS:-}
  report $? "command_sweep.py"
fi

step "L2 真机臂桥接（假 agx 反馈）"
python3 tests/arm_real_bridge_check.py
report $? "arm_real_bridge_check.py"

step "L3 VR→臂链路（假头显：侧移 + 前伸 + 手腕三轴转动，DEF-044）"
# 25 s：断言 IK 残差 ≤5 mm、不下发未收敛解、不撞关节限位、|dq| ≤4 rad/s、
# 力矩 ≤40 N·m、跟手误差 p95 ≤40 mm。对 2026-10-12 之前的映射会 FAIL（见 DEF-044）。
# VRA=0 可跳过。
if [ "${VRA:-1}" = "1" ]; then
  python3 tests/vr_arm_chain_check.py --out /tmp/m20_vr_arm_check
  report $? "vr_arm_chain_check.py"
fi

step "L3 VR 复位语义（机身/臂/速度回默认 + 交还控制权，DEF-046）"
# 设备级 + 端到端两个场景（default = 复位交还控制权；keepactive = 复位保持激活，
# 机身锚点"被归档"那个 bug 就在这个分支）。进 RL 的入口瞬态（DEF-018）会自动重跑。
# VRR=0 可跳过；VRR_SCENARIOS 可覆盖（例如 VRR_SCENARIOS=default 只跑一遍）。
if [ "${VRR:-1}" = "1" ]; then
  python3 tests/vr_reset_semantics_check.py --out /tmp/m20_vr_reset_check \
    --scenario "${VRR_SCENARIOS:-both}"
  report $? "vr_reset_semantics_check.py"
fi

for m in $MODES; do
  step "L3 sim2sim --mode $m"
  if [ "$m" = "rl" ] || [ "$m" = "walk" ] || [ "$m" = "arm_move" ] || [ "$m" = "no_arm" ] || [ "$m" = "arm_mixed" ]; then
    # 这两档都有"进 RL 入口瞬态"，用失败率判（见 DEF-018）
    python3 tests/sim2sim_smoke.py --mode "$m" --duration 20 --repeat 6
  elif [ "$m" = "hold" ]; then
    python3 tests/sim2sim_smoke.py --mode "$m" --duration 12
  elif [ "$m" = "wheel_step" ]; then
    python3 tests/sim2sim_smoke.py --mode "$m" --duration 12
  else
    python3 tests/sim2sim_smoke.py --mode "$m" --duration 25
  fi
  report $? "sim2sim_smoke --mode $m"
done

step "总结"
if [ "$fail" -eq 0 ]; then
  echo "全部通过。"
else
  echo "有失败项，见上面 - > FAIL 行。"
fi
exit "$fail"
