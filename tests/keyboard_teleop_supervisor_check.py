#!/usr/bin/env python3
"""Unit/process-level checks for the unified keyboard supervisor."""

from __future__ import annotations

import importlib.util
import os
import signal
import subprocess
import sys
import tempfile
import textwrap
import time
from types import SimpleNamespace
from unittest import mock
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
SUPERVISOR = REPO / "src/M20_sdk_deploy/scripts/keyboard_teleop_supervisor.py"


def load_supervisor():
    spec = importlib.util.spec_from_file_location(
        "keyboard_teleop_supervisor", SUPERVISOR)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def check_routes() -> list[str]:
    supervisor = load_supervisor()
    cases = {
        b"w": ("rl",),
        b"C": ("rl",),
        b"8": ("arm",),
        b"+": ("arm",),
        b"g": ("rl", "arm"),
        b"L": ("rl", "arm"),
        b" ": (),
    }
    failures = []
    for key, expected in cases.items():
        actual = supervisor.route_key(key)
        if actual != expected:
            failures.append(f"{key!r}: expected {expected}, got {actual}")
    return failures


def check_topic_readiness_timeout() -> list[str]:
    supervisor = load_supervisor()
    calls = []

    def fake_run(*args, **kwargs):
        calls.append((args, kwargs))
        return SimpleNamespace(stdout="Type: drdds/msg/JointsData\n"
                                      "Publisher count: 1\n"
                                      "Subscription count: 0\n")

    with mock.patch.object(supervisor.subprocess, "run", side_effect=fake_run):
        count = supervisor.topic_publisher_count("/JOINTS_DATA")

    failures = []
    if count != 1:
        failures.append(f"topic publisher 解析失败：expected 1, got {count}")
    if not calls:
        failures.append("topic_publisher_count 没有调用 ros2 topic info")
    else:
        timeout_s = calls[0][1].get("timeout")
        if timeout_s is None or timeout_s < 5.0:
            failures.append(
                f"topic info 单次超时过短：{timeout_s!r}s（M20 Foxy 实测约 2.6s）")
    return failures


def check_profiles() -> list[str]:
    supervisor = load_supervisor()
    cases = {
        ("--profile", "sim"): ["ik", "arm", "rl"],
        ("--profile", "real"): ["adapter", "ik", "arm", "rl"],
        ("--profile", "real", "--no-deploy"): ["adapter", "ik", "arm"],
        ("--profile", "arm"): ["ik", "arm"],
        ("--profile", "legs"): ["rl"],
    }
    failures = []
    for argv, expected in cases.items():
        args = supervisor.parse_args(list(argv))
        actual = [spec.label for spec in supervisor.build_child_specs(args)]
        if actual != expected:
            failures.append(f"{argv}: expected {expected}, got {actual}")
    return failures


def check_process_filter() -> list[str]:
    supervisor = load_supervisor()
    cases = {
        "/root/install/m20_sdk_deploy/lib/m20_sdk_deploy/rl_deploy": ("rl",),
        "/root/src/M20_sdk_deploy/interface/robot/simulation/arm_controller.py": ("ik",),
        "/root/src/M20_sdk_deploy/interface/robot/simulation/arm_teleop_node.py": ("arm",),
    }
    failures = []
    for cmdline, expected in cases.items():
        actual = supervisor.process_labels(cmdline)
        if actual != expected:
            failures.append(f"{cmdline}: expected {expected}, got {actual}")
    # 未请求 IK 时，已有 IK 不应阻止 supervisor 启动。
    args = supervisor.parse_args([
        "--profile", "real", "--no-arm-controller",
        "--no-readiness-check",
    ])
    wanted = {spec.label for spec in supervisor.build_child_specs(args)}
    if "ik" in wanted:
        failures.append("--no-arm-controller 仍把 ik 放入启动和冲突检测集合")

    marker = "/root/src/M20_sdk_deploy/interface/robot/simulation/arm_controller.py"
    fake = subprocess.Popen(
        [sys.executable, "-c", "import time; time.sleep(10)", marker],
        start_new_session=True,
    )
    try:
        time.sleep(0.2)
        rl_matches = supervisor.existing_node_processes(("rl",))
        if any(str(fake.pid) in line for line in rl_matches):
            failures.append("只检查 rl 时错误匹配了 arm_controller")
        ik_matches = supervisor.existing_node_processes(("ik",))
        if not any(str(fake.pid) in line for line in ik_matches):
            failures.append("检查 ik 时没有匹配已存在的 arm_controller")
    finally:
        try:
            os.killpg(fake.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        fake.wait()
    return failures


FAKE_CHILD = """\
import sys

label = sys.argv[1]
while True:
    data = sys.stdin.buffer.read(1)
    if not data:
        break
    print(f"{label}:{data.decode(errors='replace')}", flush=True)
"""


def check_process_routing() -> list[str]:
    with tempfile.TemporaryDirectory(prefix="m20_kb_supervisor_") as tmp:
        fake = Path(tmp) / "fake_child.py"
        fake.write_text(textwrap.dedent(FAKE_CHILD), encoding="utf-8")

        def command(label: str) -> str:
            return f"{sys.executable} {fake} {label}"

        process = subprocess.Popen(
            [
                sys.executable,
                str(SUPERVISOR),
                "--profile", "sim",
                "--startup-delay", "0",
                "--no-readiness-check",
                "--rl-deploy", command("rl"),
                "--arm-controller", command("ik"),
                "--arm-teleop", command("arm"),
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        assert process.stdin is not None
        process.stdin.write(b"w8Gq")
        process.stdin.flush()
        time.sleep(0.5)
        process.stdin.write(b"\x03")
        process.stdin.flush()
        try:
            output, _ = process.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            output, _ = process.communicate()
            return ["supervisor 没有在收到 Ctrl+C 后退出"]

    text = output.decode(errors="replace")
    failures = []
    if process.returncode != 0:
        failures.append(f"supervisor 返回码 {process.returncode}")
    for expected in ("[rl] rl:w", "[rl] rl:G", "[rl] rl:q",
                     "[arm] arm:8", "[arm] arm:G"):
        if expected not in text:
            failures.append(f"缺少输出 {expected!r}")
    if "rl:8" in text:
        failures.append("小键盘 8 被错误转发给 rl_deploy")
    if "arm:w" in text or "arm:q" in text:
        failures.append("移动键被错误转发给 arm_teleop")
    if "\n[ik] " in text:
        failures.append("IK 节点不应收到键盘字节")
    if failures:
        failures.append("supervisor 输出：\n" + text)
    return failures


def main() -> int:
    failures = (check_routes() + check_topic_readiness_timeout() +
                check_profiles() + check_process_filter() +
                check_process_routing())
    if failures:
        print("[keyboard-supervisor] FAIL")
        for failure in failures:
            print("  -", failure)
        return 1
    print("[keyboard-supervisor] PASS：按键分流、G/L 双发、统一退出")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
