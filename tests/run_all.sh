#!/usr/bin/env bash
# L0→L3 一键验收（在容器 m20_piper_ros 内、仓库根目录执行）
#
#   bash tests/run_all.sh            # 完整：编译 + L1 + L3' + 五档 L3
#   SKIP_BUILD=1 bash tests/run_all.sh
#   MODES="hold rl" bash tests/run_all.sh
#
# 判据（详细口径见 docs/review/WORKFLOW_zh.md）：
#   * L1  check_policy_interface.py   —— 布局/形状/名字/原生序/数值对照
#   * L3' check_mjcf_contract.py      —— MJCF 与训练配置逐项对照（不允许 FAIL）
#   * L3  每档：按契约文档的高度/倾角判据；
#         `rl` 档因为入口边缘发散（DEF-018），用 --repeat 判失败率 ≤ 1/3，其余档 0 失败。
# 注意：不要 `set -u` —— ROS 的 setup.bash 里会引用未定义变量，会把脚本打断
cd "$(dirname "$0")/.." || exit 1
source /opt/ros/humble/setup.bash
source install/setup.bash

SCRIPTS=src/M20_sdk_deploy/scripts
MODES=${MODES:-"hold rl walk arm push"}
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

step "L3' MJCF 与训练配置对照"
python3 "$SCRIPTS/check_mjcf_contract.py"
report $? "check_mjcf_contract.py"

for m in $MODES; do
  step "L3 sim2sim --mode $m"
  if [ "$m" = "rl" ]; then
    python3 tests/sim2sim_smoke.py --mode "$m" --duration 20 --repeat 6
  elif [ "$m" = "hold" ]; then
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
