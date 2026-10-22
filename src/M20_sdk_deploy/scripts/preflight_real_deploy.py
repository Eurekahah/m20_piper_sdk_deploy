#!/usr/bin/env python3
"""真机部署前检查（无臂 / 有臂 / 混合，一次说清该跑什么）。

回答两个问题：
  1. 我手上这个策略**要不要机械臂的数据**？（看 `policy_layout.json` 的观测构成）
  2. 现在这台机器/这个编译产物处于什么状态？（编译选项、节点、话题发布者）

用法（仓库根目录，容器或真机上都可）：

  python3 src/M20_sdk_deploy/scripts/preflight_real_deploy.py
  python3 src/M20_sdk_deploy/scripts/preflight_real_deploy.py --policy-dir <目录>
  python3 src/M20_sdk_deploy/scripts/preflight_real_deploy.py --offline   # 只看文件，不问 ROS

退出码：0 = 没有阻断项；1 = 有阻断项（按输出里的"必须做"处理）。
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]          # 仓库根目录
PKG = REPO / "src/M20_sdk_deploy"
DEFAULT_POLICY = PKG / "policy/m20_piper_flat_20260930"
BUILD_FLAGS = REPO / "build/m20_sdk_deploy/CMakeFiles/rl_deploy.dir/flags.make"
RL_DEPLOY_BIN = REPO / "install/m20_sdk_deploy/lib/m20_sdk_deploy/rl_deploy"

blockers: list[str] = []
notes: list[str] = []


def say(ok: bool | None, title: str, detail: str = "") -> None:
    tag = "OK  " if ok else ("注意" if ok is None else "必须")
    print(f"  [{tag}] {title}")
    if detail:
        for line in detail.splitlines():
            print(f"         {line}")


def blocker(msg: str) -> None:
    blockers.append(msg)


def run(cmd: list[str], timeout: int = 8) -> str | None:
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.stdout if r.returncode == 0 else None
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None


# ---------------------------------------------------------------------------
# 1. 策略：要不要臂？
# ---------------------------------------------------------------------------
def check_policy(policy_dir: Path) -> dict:
    print("\n[1] 策略接口")
    onnx, layout = policy_dir / "policy.onnx", policy_dir / "policy_layout.json"
    if not onnx.is_file() or not layout.is_file():
        say(False, f"{policy_dir} 里缺 policy.onnx / policy_layout.json",
            "训练侧 export_deploy_policy.py 会把两者一起导出（见 policy/README.md）")
        blocker("策略目录不完整")
        return {}
    cfg = json.loads(layout.read_text())
    obs = cfg["policy_obs_dim"]
    act = cfg["action_dim"]
    step = cfg["history_single_step_dim"]
    steps = cfg["history_length"]

    # 从维度反推观测构成（本仓库支持的结构见 docs/CONTRACT_zh.md）
    n_joints = None
    has_ee = None
    for n in (24, 16, 22):
        for ee in (7, 0):
            if obs == 3 + 3 + 3 + n + n + act + ee + 3:
                n_joints, has_ee = n, ee > 0
    say(True, f"policy = {policy_dir.name}: obs {obs} / history {steps}x{step} / action {act}")
    if n_joints is None:
        say(False, "观测构成无法识别（不是本仓库 runner 支持的布局）",
            f"obs={obs} action={act}\n"
            "本 runner 目前只支持 '3+3+3+N+N+action+（可选7）+3'，N=24（含臂）或 16（不含臂）")
        blocker(f"策略布局 {obs}/{act} 不被 runner 支持")
        return {"obs": obs, "action": act, "needs_arm": None}

    needs_arm = n_joints == 24
    say(True, f"关节数 = {n_joints}（原生序），ee_goal = {'有' if has_ee else '无'}"
              f" ⇒ 策略{'**需要**' if needs_arm else '**不需要**'}机械臂数据")
    if needs_arm:
        notes.append("策略观测里含机械臂：真机没装臂时必须提供臂反馈（arm_stub_node.py）")
    if cfg.get("kind") != "history":
        say(False, f"layout kind = {cfg.get('kind')}（runner 只接受 history）",
            "别用 play.py 导出的 actor-only policy.pt/onnx")
        blocker("策略不是 history 类型")
    return {"obs": obs, "action": act, "needs_arm": needs_arm, "n_joints": n_joints}


# ---------------------------------------------------------------------------
# 2. 编译产物
# ---------------------------------------------------------------------------
def check_build() -> None:
    print("\n[2] 编译产物")
    if not RL_DEPLOY_BIN.is_file():
        say(False, "没找到 install/m20_sdk_deploy/lib/m20_sdk_deploy/rl_deploy",
            "先编译：colcon build --packages-up-to m20_sdk_deploy \\\n"
            "        --cmake-args -DBUILD_PLATFORM=x86 -DSIM2SIM=ON   # 仿真\n"
            "        --cmake-args -DBUILD_PLATFORM=arm               # 真机（不带 SIM2SIM）")
        blocker("rl_deploy 未编译")
        return
    sim2sim = None
    if BUILD_FLAGS.is_file():
        sim2sim = "-DSIM2SIM" in BUILD_FLAGS.read_text()
    say(sim2sim is not True if sim2sim is not None else None,
        f"编译选项 SIM2SIM = {sim2sim}",
        ("真机要**不带** -DSIM2SIM 的产物（带 SIM2SIM 会走恒等标定 M20SimInterface，"
         "真机上关节坐标系是错的）") if sim2sim else "真机构建（M20Interface，带 dir/offset 标定）")
    if sim2sim:
        blocker("当前是 SIM2SIM 构建，不能上真机")
    arch = run(["file", "-b", str(RL_DEPLOY_BIN)]) or ""
    say(None, f"可执行文件: {arch.strip() or '未知'}")
    if arch and "aarch64" in arch and "arm" not in os.uname().machine:
        notes.append("交叉编译产物是 aarch64：要在机器人上运行（scp 过去）")


# ---------------------------------------------------------------------------
# 3. 节点脚本是否齐全
# ---------------------------------------------------------------------------
def check_scripts() -> None:
    print("\n[3] 节点脚本")
    for name, path in (
        ("keyboard_teleop_supervisor.py（统一控制）",
         PKG / "scripts/keyboard_teleop_supervisor.py"),
        ("arm_stub_node.py（无臂时用）", PKG / "interface/robot/simulation/arm_stub_node.py"),
        ("arm_controller.py（有臂、要动）", PKG / "interface/robot/simulation/arm_controller.py"),
        ("arm_teleop_node.py（臂键盘）", PKG / "interface/robot/simulation/arm_teleop_node.py"),
        ("arm_real_adapter.py（真臂传输）", PKG / "interface/robot/simulation/arm_real_adapter.py"),
    ):
        say(path.is_file(), name)


# ---------------------------------------------------------------------------
# 4. 话题（需要 ROS 环境）
# ---------------------------------------------------------------------------
def check_topics() -> dict:
    print("\n[4] 运行时话题")
    if shutil.which("ros2") is None:
        say(None, "没找到 ros2 命令（先 source /opt/ros/<distro>/setup.bash 与 install/setup.bash）")
        return {}
    listing = run(["ros2", "topic", "list"], timeout=15)
    if listing is None:
        say(None, "ros2 topic list 不可用（ROS daemon 没起？）")
        return {}
    have = {t.strip() for t in listing.splitlines() if t.strip()}
    info: dict = {}
    for t in ("/JOINTS_DATA", "/IMU_DATA", "/JOINTS_CMD",
              "/ARM_JOINTS_DATA", "/ARM_TELEOP_STATE", "/ARM_JOINTS_CMD"):
        if t not in have:
            say(None if t != "/JOINTS_DATA" else False, f"{t}: 不存在")
            info[t] = False
            continue
        detail = run(["ros2", "topic", "info", "-v", t], timeout=15) or ""
        pubs = len(re.findall(r"node name:", detail)) // 2 if "node name:" in detail else 0
        subs = len(re.findall(r"node namespace:", detail))
        info[t] = True
        say(True, f"{t}: 存在（发布者 {pubs}）")
    if not info.get("/JOINTS_DATA"):
        # 只看到自己的 /rosout + /parameter_events = 根本没连上机器人的 ROS 图，
        # 这几乎总是 domain / RMW / 机器人服务的问题，不是我们的代码（DEF-024 的 A 类）
        if len(have) <= 3:
            blocker("话题列表几乎是空的（只有 /rosout、/parameter_events）：这个 shell 没看到"
                    "机器人的 ROS 2 图。\n"
                    "    依次检查：\n"
                    "      ① ROS_DOMAIN_ID 是否与机器人服务一致（我们文档里 sim2sim 用 1，"
                    "机器人服务通常用默认 0 —— 先 `unset ROS_DOMAIN_ID` 再 `ros2 topic list`）；\n"
                    "      ② RMW 是否一致（看 /opt/robot/scripts/setup_ros2.sh 设了什么，"
                    "`echo $RMW_IMPLEMENTATION`）；\n"
                    "      ③ 机器人服务在不在（`ps -ef | grep -i drdds`、"
                    "`systemctl list-units | grep -i ros`、`ros2 node list`）。\n"
                    "    期望看到 /JOINTS_DATA /IMU_DATA /BATTERY_DATA /JOINTS_CMD。")
        else:
            blocker("没有 /JOINTS_DATA：腿的反馈没上来（真机上要先进 SDK 模式；"
                    "仿真里先起 MuJoCo 节点）")
    if info.get("/ARM_JOINTS_DATA"):
        say(True, "臂反馈已在发布（真臂或假臂节点在跑）")
    else:
        notes.append("没有 /ARM_JOINTS_DATA：如果策略需要臂数据，请起 arm_stub_node.py 或真臂驱动")
    return info


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy-dir", default=str(DEFAULT_POLICY))
    ap.add_argument("--offline", action="store_true", help="不检查 ROS 话题")
    args = ap.parse_args()

    print(f"[preflight] 仓库 = {REPO}")
    pol = check_policy(Path(args.policy_dir).resolve())
    check_build()
    check_scripts()
    topics = {} if args.offline else check_topics()

    print("\n[结论]")
    if pol.get("needs_arm") is True:
        print("  策略**需要**机械臂数据（观测 24 关节 + ee_goal）。")
        if topics.get("/ARM_JOINTS_DATA"):
            print("  → 臂反馈已经在发（真臂 or 假臂节点），可直接启动统一控制终端。")
        else:
            print("  → 臂没装/没上电：**先起假臂节点**")
            print("     python3 src/M20_sdk_deploy/interface/robot/simulation/arm_stub_node.py")
        print("  然后：")
        print("     python3 src/M20_sdk_deploy/scripts/keyboard_teleop_supervisor.py "
              "--profile real     # z 站立 → c 进 RL")
    elif pol.get("needs_arm") is False:
        print("  策略**不需要**机械臂数据 → 直接部署腿（不需要假臂节点）：")
        print("     python3 src/M20_sdk_deploy/scripts/keyboard_teleop_supervisor.py "
              "--profile legs")
    else:
        print("  策略布局无法识别，见上面 [1] 的说明。")

    for n in notes:
        print(f"  注意：{n}")
    if blockers:
        print("\n  必须处理（阻断项）：")
        for b in blockers:
            print(f"    - {b}")
    else:
        print("\n  没有阻断项。")
    return 1 if blockers else 0


if __name__ == "__main__":
    sys.exit(main())
