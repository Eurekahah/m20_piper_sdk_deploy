#!/usr/bin/env bash
# 一条命令录一段真机 `/JOINTS_DATA`（16 关节 torque + motion_temp + driver_temp + IMU）。
#
# **只订阅，不控制** —— 所以原厂控制器撑着腿（不进 SDK 模式）还是我们的 rl_deploy
# 在跑，都能用；也不用停机器人。
#
# 用法（在 M20 上，仓库根目录）::
#
#   bash src/M20_sdk_deploy/scripts/record_joint_data.sh                # tag=vendorstand，10 分钟
#   bash src/M20_sdk_deploy/scripts/record_joint_data.sh vendorstand 600  # 自定义 tag / 秒数
#   TAG=hipxscan1 DURATION=180 bash src/M20_sdk_deploy/scripts/record_joint_data.sh
#   DURATION=0  ...          # 一直录到 Ctrl+C
#
# 产出 `~/joint_logs/<tag>.csv`（+ 同名 `.meta.json`）。**已存在不会覆盖**，会自动改成
# `<tag>_2.csv`、`_3.csv`…（真机时钟可能是错的，所以别靠时间戳区分，用 tag 更可靠）。
#
# 跑完在**开发机**上取回 + 分析::
#   scp 'user@10.21.31.103:~/joint_logs/<tag>.*' tmp/real_logs/      # M20 的密码是一个单引号 '
#   python3 src/M20_sdk_deploy/scripts/joint_log_report.py tmp/real_logs/<tag>.csv --ref standup
#
# 说明：本脚本不设 `ROS_DOMAIN_ID` —— **机器人自带服务用默认域**（不设就是 0），
# 设错了会"一帧都收不到"。下面会把当前域/RMW 打出来给你核对。

REPO_ROOT=$(cd "$(dirname "$0")/../../.." && pwd)
TAG=${1:-${TAG:-vendorstand}}
DURATION=${2:-${DURATION:-600}}

echo "[rec] 仓库根目录: $REPO_ROOT"
cd "$REPO_ROOT" || exit 1

# --- 环境：真机 Foxy / 开发容器 humble 都能用 ---
if [ -f /opt/ros/foxy/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/foxy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  echo "[rec] 找不到 /opt/ros/{foxy,humble}/setup.bash —— 先确认 ROS 装在哪" >&2
fi
if [ -f /opt/robot/scripts/setup_ros2.sh ]; then
  # shellcheck disable=SC1091
  source /opt/robot/scripts/setup_ros2.sh
fi
if [ -f "$REPO_ROOT/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$REPO_ROOT/install/setup.bash"
fi

echo "[rec] ROS_DOMAIN_ID='${ROS_DOMAIN_ID-（未设 = 默认域 0）}'  RMW='${RMW_IMPLEMENTATION-（默认）}'"

# --- 前置检查：话题在不在 ---
if command -v ros2 >/dev/null 2>&1; then
  if ! ros2 topic list 2>/dev/null | grep -qx '/JOINTS_DATA'; then
    echo "[rec] ❌ 看不到 /JOINTS_DATA。按顺序查：" >&2
    echo "      ① 机器人服务在跑吗？  ps -ef | grep -i drdds ;  ros2 node list" >&2
    echo "      ② ROS_DOMAIN_ID 与机器人服务一致吗？（自带服务通常是**默认域**：unset ROS_DOMAIN_ID）" >&2
    echo "      ③ RMW_IMPLEMENTATION 一致吗？（见 /opt/robot/scripts/setup_ros2.sh）" >&2
    echo "      ④ ros2 daemon stop 后再试一次" >&2
    exit 3
  fi
  echo "[rec] ✅ /JOINTS_DATA 在（下面开始录）"
fi

python3 "$REPO_ROOT/src/M20_sdk_deploy/scripts/record_joint_data.py" \
        --duration "$DURATION" \
        --out "$HOME/joint_logs/$TAG.csv"
rc=$?

echo
echo "[rec] 下一步（**开发机**上）："
echo "   scp 'user@10.21.31.103:~/joint_logs/$TAG.*' tmp/real_logs/"
echo "   python3 src/M20_sdk_deploy/scripts/joint_log_report.py tmp/real_logs/$TAG.csv --ref standup"
exit $rc
