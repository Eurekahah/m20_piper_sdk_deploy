#!/usr/bin/env python3
"""Run the M20 policy and Piper keyboard arm chain in one terminal.

Both ``rl_deploy`` and ``arm_teleop_node.py`` normally read stdin directly, so
they cannot share a terminal safely.  This supervisor is the only stdin reader
and forwards each key to the component that owns it:

  * W/S/A/D/Q/E, Z/C/R/X and body-pose keys -> rl_deploy
  * numpad keys -> arm_teleop_node.py
  * G/L -> both (gripper toggle / full reset)

It also owns the child-process lifecycle.  If any child exits, all remaining
children are stopped so a failed IK or policy process cannot leave a second
publisher behind.
"""

from __future__ import annotations

import argparse
import os
import selectors
import shlex
import shutil
import signal
import subprocess
import sys
import termios
import time
import tty
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


REPO_ROOT = Path(__file__).resolve().parents[3]
SIM_DIR = REPO_ROOT / "src/M20_sdk_deploy/interface/robot/simulation"
DEFAULT_RL_DEPLOY = (
    REPO_ROOT / "install/m20_sdk_deploy/lib/m20_sdk_deploy/rl_deploy"
)
DEFAULT_ARM_CONTROLLER = SIM_DIR / "arm_controller.py"
DEFAULT_ARM_TELEOP = SIM_DIR / "arm_teleop_node.py"
DEFAULT_ARM_ADAPTER = SIM_DIR / "arm_real_adapter.py"

# Keep these sets identical to the bindings documented by rl_deploy and
# arm_teleop_node.py.  G/L intentionally overlap because both subsystems own a
# gripper/reset action.
LEG_KEYS = frozenset("wasdqerzc xhjnb[]".replace(" ", ""))
ARM_KEYS = frozenset("0123456789.+-")
BOTH_KEYS = frozenset("gl")
STOP_BYTES = frozenset((b"\x03", b"\x1b"))
NODE_PROCESS_MARKERS = {
    "rl": "/install/m20_sdk_deploy/lib/m20_sdk_deploy/rl_deploy",
    "ik": "/interface/robot/simulation/arm_controller.py",
    "arm": "/interface/robot/simulation/arm_teleop_node.py",
}
# On the M20's ROS 2 Foxy environment, a cold `ros2 topic info` can take
# >2.5 s just to complete DDS graph discovery. Keep the probe bounded, but do
# not kill a healthy discovery before it can return the publisher count.
TOPIC_INFO_TIMEOUT_S = 10.0


@dataclass(frozen=True)
class ChildSpec:
    label: str
    command: list[str]
    startup_delay_s: float = 0.0


@dataclass
class Child:
    spec: ChildSpec
    process: subprocess.Popen
    output_buffer: bytearray


def route_key(raw_byte: bytes) -> tuple[str, ...]:
    """Return the labels that should receive one stdin byte."""
    if not raw_byte or len(raw_byte) != 1:
        return ()
    try:
        key = raw_byte.decode("ascii").lower()
    except UnicodeDecodeError:
        return ()
    if key in BOTH_KEYS:
        return ("rl", "arm")
    if key in ARM_KEYS:
        return ("arm",)
    if key in LEG_KEYS:
        return ("rl",)
    return ()


def _command(value: str) -> list[str]:
    command = shlex.split(value)
    if not command:
        raise argparse.ArgumentTypeError("命令不能为空")
    return command


def build_child_specs(args: argparse.Namespace) -> list[ChildSpec]:
    specs: list[ChildSpec] = []
    profile = args.profile

    if profile == "real" and not args.no_arm_adapter:
        specs.append(ChildSpec(
            "adapter",
            _command(args.arm_adapter),
        ))

    if profile in ("sim", "real", "arm") and not args.no_arm_controller:
        specs.append(ChildSpec(
            "ik",
            _command(args.arm_controller),
        ))

    if profile != "legs" and not args.no_arm_teleop:
        specs.append(ChildSpec(
            "arm",
            _command(args.arm_teleop),
        ))

    if profile != "arm" and not args.no_deploy:
        # Let adapter/IK initialize and advertise their publishers before
        # rl_deploy checks whether it may own /ARM_JOINTS_CMD (DEF-026).
        specs.append(ChildSpec(
            "rl",
            _command(args.rl_deploy),
            startup_delay_s=args.startup_delay,
        ))
    return specs


def topic_publisher_count(topic: str, timeout_s: float = TOPIC_INFO_TIMEOUT_S) -> int | None:
    try:
        result = subprocess.run(
            ["ros2", "topic", "info", topic],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout_s,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    for line in result.stdout.splitlines():
        if not line.startswith("Publisher count:"):
            continue
        try:
            return int(line.rsplit(":", 1)[1].strip())
        except ValueError:
            return None
    return None


def wait_for_topic_publishers(topic: str, timeout_s: float) -> int | None:
    """Wait until a ROS topic has a publisher and return its count."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        remaining_s = max(0.1, deadline - time.monotonic())
        count = topic_publisher_count(
            topic, timeout_s=min(TOPIC_INFO_TIMEOUT_S, remaining_s))
        if count:
            return count
        time.sleep(0.25)
    return None


def process_labels(cmdline: str) -> tuple[str, ...]:
    return tuple(
        label for label, marker in NODE_PROCESS_MARKERS.items()
        if marker in cmdline
    )


def existing_node_processes(labels: Iterable[str]) -> list[str]:
    wanted = set(labels)
    markers = tuple(
        marker for label, marker in NODE_PROCESS_MARKERS.items()
        if label in wanted
    )
    if not markers:
        return []
    found = []
    for proc_path in Path("/proc").iterdir():
        if not proc_path.name.isdigit() or int(proc_path.name) == os.getpid():
            continue
        try:
            cmdline = (proc_path / "cmdline").read_bytes().replace(
                b"\0", b" ").decode(errors="replace").strip()
        except OSError:
            continue
        if any(marker in cmdline for marker in markers):
            found.append(f"{proc_path.name} {cmdline}")
    return found


class KeyboardSupervisor:
    def __init__(self, specs: Iterable[ChildSpec], readiness_topics: Iterable[str],
                 readiness_timeout_s: float):
        self._specs = list(specs)
        self._readiness_topics = list(readiness_topics)
        self._readiness_timeout_s = readiness_timeout_s
        self._children: dict[str, Child] = {}
        self._pending = list(self._specs)
        self._selector = selectors.DefaultSelector()
        self._stop = False
        self._failed = False
        self._old_stdin = None
        self._stdin_is_tty = False

    def _setup_stdin(self) -> None:
        try:
            self._old_stdin = termios.tcgetattr(sys.stdin.fileno())
            # cbreak keeps ISIG enabled, so Ctrl+C still reaches the supervisor.
            tty.setcbreak(sys.stdin.fileno())
            self._stdin_is_tty = True
        except (termios.error, OSError):
            self._old_stdin = None
            self._stdin_is_tty = False

    def _restore_stdin(self) -> None:
        if self._old_stdin is not None:
            try:
                termios.tcsetattr(
                    sys.stdin.fileno(), termios.TCSADRAIN, self._old_stdin)
            except (termios.error, OSError):
                pass
            self._old_stdin = None

    @staticmethod
    def _write_all(pipe, payload: bytes) -> None:
        view = memoryview(payload)
        while view:
            written = os.write(pipe.fileno(), view)
            if written <= 0:
                raise BrokenPipeError("child stdin closed")
            view = view[written:]

    def _spawn(self, spec: ChildSpec) -> None:
        print(f"[supervisor] 启动 {spec.label}: {' '.join(spec.command)}",
              flush=True)
        command = spec.command
        if spec.label == "rl" and shutil.which("stdbuf"):
            # C++ iostream switches to block buffering when stdout is a pipe.
            command = ["stdbuf", "-oL", "-eL", *command]
        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"
        process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            bufsize=0,
            start_new_session=True,
            env=env,
        )
        child = Child(spec=spec, process=process, output_buffer=bytearray())
        self._children[spec.label] = child
        self._selector.register(
            process.stdout, selectors.EVENT_READ, ("output", spec.label))

    def _start_due_children(self, now_s: float) -> None:
        started_at = getattr(self, "_started_at", now_s)
        due = [
            spec for spec in self._pending
            if now_s - started_at >= spec.startup_delay_s
        ]
        for spec in due:
            self._pending.remove(spec)
            try:
                self._spawn(spec)
            except OSError as exc:
                print(
                    f"[supervisor] 启动 {spec.label} 失败: {exc}",
                    file=sys.stderr,
                    flush=True,
                )
                self._failed = True
                self._stop = True

    def _emit_lines(self, child: Child, data: bytes) -> None:
        if not data:
            return
        child.output_buffer.extend(data)
        while True:
            newline = child.output_buffer.find(b"\n")
            if newline < 0:
                break
            line = bytes(child.output_buffer[:newline + 1])
            del child.output_buffer[:newline + 1]
            text = line.decode(errors="replace")
            print(f"[{child.spec.label}] {text}", end="", flush=True)

    def _flush_output(self, child: Child) -> None:
        if child.output_buffer:
            text = bytes(child.output_buffer).decode(errors="replace")
            print(f"[{child.spec.label}] {text}", end="", flush=True)
            child.output_buffer.clear()

    def _dispatch(self, raw_byte: bytes) -> None:
        if raw_byte in STOP_BYTES:
            print("\n[supervisor] 收到停止键，正在收工...", flush=True)
            self._stop = True
            return

        labels = route_key(raw_byte)
        if not labels:
            return
        for label in labels:
            child = self._children.get(label)
            if child is None or child.process.poll() is not None:
                continue
            try:
                self._write_all(child.process.stdin, raw_byte)
            except (BrokenPipeError, OSError):
                print(
                    f"[supervisor] {label} 的 stdin 已关闭",
                    file=sys.stderr,
                    flush=True,
                )
                self._failed = True
                self._stop = True
                return

    def _handle_output(self, label: str) -> None:
        child = self._children[label]
        try:
            data = os.read(child.process.stdout.fileno(), 4096)
        except OSError:
            data = b""
        if data:
            self._emit_lines(child, data)
            return

        self._flush_output(child)
        self._selector.unregister(child.process.stdout)
        return_code = child.process.wait()
        if not self._stop:
            print(
                f"[supervisor] {label} 已退出（code={return_code}），"
                "停止其余节点",
                file=sys.stderr,
                flush=True,
            )
            self._failed = return_code != 0
            self._stop = True

    def _signal_stop(self, signum, _frame) -> None:
        print(
            f"\n[supervisor] 收到 signal {signum}，正在收工...",
            flush=True,
        )
        self._stop = True

    def _stop_children(self) -> None:
        running = [
            child for child in self._children.values()
            if child.process.poll() is None
        ]
        for child in running:
            try:
                child.process.stdin.close()
            except OSError:
                pass

        def signal_group(sig) -> None:
            for child in running:
                if child.process.poll() is None:
                    try:
                        os.killpg(child.process.pid, sig)
                    except ProcessLookupError:
                        pass

        signal_group(signal.SIGINT)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if all(child.process.poll() is not None for child in running):
                break
            time.sleep(0.05)

        if any(child.process.poll() is None for child in running):
            signal_group(signal.SIGTERM)
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                if all(child.process.poll() is not None for child in running):
                    break
                time.sleep(0.05)

        for child in running:
            if child.process.poll() is None:
                try:
                    os.killpg(child.process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            child.process.wait()

    def run(self) -> int:
        if not self._specs:
            print("[supervisor] 没有需要启动的节点", file=sys.stderr)
            return 2

        for topic in self._readiness_topics:
            print(f"[supervisor] 等待 {topic} 就绪...", flush=True)
            count = wait_for_topic_publishers(topic, self._readiness_timeout_s)
            if count is None:
                print(
                    f"[supervisor] 等待 {topic} 超时 "
                    f"({self._readiness_timeout_s:.1f} s)，未启动任何节点",
                    file=sys.stderr,
                    flush=True,
                )
                return 2
            if count != 1:
                print(
                    f"[supervisor] {topic} 有 {count} 个发布者；"
                    "请先清理残留节点",
                    file=sys.stderr,
                    flush=True,
                )
                return 2
            print(f"[supervisor] {topic} 已就绪（1 个发布者）", flush=True)

        existing = existing_node_processes(
            spec.label for spec in self._specs)
        if existing:
            print("[supervisor] 检测到已运行的控制节点；请先清理：", file=sys.stderr)
            for line in existing:
                print("  " + line, file=sys.stderr)
            return 2

        old_handlers = {}
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            old_handlers[sig] = signal.getsignal(sig)
            signal.signal(sig, self._signal_stop)

        self._setup_stdin()
        self._selector.register(sys.stdin, selectors.EVENT_READ, ("stdin", ""))
        self._started_at = time.monotonic()

        try:
            while not self._stop:
                self._start_due_children(time.monotonic())
                timeout = 0.1
                if self._pending:
                    timeout = min(timeout, 0.05)
                for key, _mask in self._selector.select(timeout=timeout):
                    event_type, label = key.data
                    if event_type == "stdin":
                        try:
                            raw = os.read(sys.stdin.fileno(), 64)
                        except OSError:
                            raw = b""
                        if not raw:
                            print(
                                "\n[supervisor] stdin 已关闭，正在收工...",
                                flush=True,
                            )
                            self._stop = True
                            break
                        for raw_byte in raw:
                            self._dispatch(bytes((raw_byte,)))
                            if self._stop:
                                break
                    else:
                        self._handle_output(label)
                    if self._stop:
                        break
        finally:
            self._stop_children()
            self._restore_stdin()
            self._selector.close()
            for sig, handler in old_handlers.items():
                signal.signal(sig, handler)

        print("[supervisor] 所有节点已停止", flush=True)
        return 1 if self._failed else 0


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="单终端运行 M20 策略、Piper IK 和键盘遥操作")
    parser.add_argument(
        "--profile",
        choices=("sim", "real", "arm", "legs"),
        default="sim",
        help="sim=策略+IK+臂键盘；real=adapter+策略+IK+臂键盘；"
             "arm=仅IK+臂键盘；legs=仅策略",
    )
    parser.add_argument(
        "--startup-delay",
        type=float,
        default=1.0,
        help="启动 adapter/IK/arm_teleop 后，延迟多少秒再启动 rl_deploy（默认 1.0）",
    )
    parser.add_argument(
        "--readiness-timeout",
        type=float,
        default=10.0,
        help="等待 /JOINTS_DATA 等关键话题的最长时间（默认 10 s）",
    )
    parser.add_argument(
        "--no-readiness-check",
        action="store_true",
        help="跳过启动前的话题发布者检查",
    )
    parser.add_argument("--no-deploy", action="store_true", help="不启动 rl_deploy")
    parser.add_argument(
        "--no-arm-controller", action="store_true", help="不启动 IK 节点")
    parser.add_argument(
        "--no-arm-teleop", action="store_true", help="不启动臂键盘节点")
    parser.add_argument(
        "--no-arm-adapter", action="store_true", help="real 模式下不启动真臂 adapter")
    parser.add_argument(
        "--rl-deploy",
        default=os.environ.get(
            "M20_KEYBOARD_SUPERVISOR_RL_DEPLOY", str(DEFAULT_RL_DEPLOY)),
        help="rl_deploy 命令或路径",
    )
    parser.add_argument(
        "--arm-controller",
        default=os.environ.get(
            "M20_KEYBOARD_SUPERVISOR_ARM_CONTROLLER",
            f"{sys.executable} {DEFAULT_ARM_CONTROLLER}"),
        help="arm_controller 命令",
    )
    parser.add_argument(
        "--arm-teleop",
        default=os.environ.get(
            "M20_KEYBOARD_SUPERVISOR_ARM_TELEOP",
            f"{sys.executable} {DEFAULT_ARM_TELEOP}"),
        help="arm_teleop 命令",
    )
    parser.add_argument(
        "--arm-adapter",
        default=os.environ.get(
            "M20_KEYBOARD_SUPERVISOR_ARM_ADAPTER",
            f"{sys.executable} {DEFAULT_ARM_ADAPTER}"),
        help="arm_real_adapter 命令",
    )
    args = parser.parse_args(argv)
    if args.startup_delay < 0:
        parser.error("--startup-delay 不能小于 0")
    if args.readiness_timeout < 0:
        parser.error("--readiness-timeout 不能小于 0")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    specs = build_child_specs(args)
    readiness_topics = [] if args.no_readiness_check else ["/JOINTS_DATA"]
    if not args.no_readiness_check and args.profile in ("sim", "arm"):
        readiness_topics.append("/ARM_JOINTS_DATA")
    return KeyboardSupervisor(
        specs, readiness_topics, args.readiness_timeout).run()


if __name__ == "__main__":
    raise SystemExit(main())
