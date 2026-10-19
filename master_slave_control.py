#!/usr/bin/env python3
"""Start, stop, and inspect the local GELLO/IIMT control service."""

from __future__ import annotations

import argparse
import hashlib
import ipaddress
import json
import os
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path


PROJECT = Path(__file__).resolve().parent
STATE_DIR = Path("/tmp/gello_iimt_master_slave")
CONFIG_PATH = PROJECT / "configs/iimt_master_slave.json"
LEGACY_SERVER_PID = STATE_DIR / "server.pid"
CONTROL_PID = STATE_DIR / "control.pid"
CONTROL_LOG = STATE_DIR / "control.log"
SERVER_SIGNATURE = STATE_DIR / "server.signature"
SERVER_CODE_FILES = (
    PROJECT / "experiments/launch_nodes.py",
    PROJECT / "gello/robots/iimt_robot.py",
    PROJECT / "gello/zmq_core/robot_node.py",
)


def _server_pid(index: int) -> Path:
    return STATE_DIR / f"server_{index}.pid"


def _server_log(index: int) -> Path:
    return STATE_DIR / f"server_{index}.log"


def _load_followers() -> list[dict[str, object]]:
    config = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    raw_followers = config.get("followers")
    if raw_followers is None:
        raw_followers = [
            {
                "name": "从臂1",
                "enabled": True,
                "robot_ip": config.get("robot_ip", ""),
                "robot_port": config.get("robot_port", 6001),
            }
        ]
    if not isinstance(raw_followers, list) or not 1 <= len(raw_followers) <= 2:
        raise ValueError("followers must contain one or two follower configurations")

    followers: list[dict[str, object]] = []
    for index in range(2):
        raw = raw_followers[index] if index < len(raw_followers) else {}
        if not isinstance(raw, dict):
            raise ValueError(f"follower {index + 1} configuration must be an object")
        enabled = bool(raw.get("enabled", index == 0))
        robot_ip = str(raw.get("robot_ip", "")).strip()
        if enabled:
            if not robot_ip:
                raise ValueError(f"从臂{index + 1} 已启用，但 IP 地址为空")
            try:
                ipaddress.ip_address(robot_ip)
            except ValueError as error:
                raise ValueError(f"从臂{index + 1} IP 地址无效：{robot_ip}") from error
        port = int(raw.get("robot_port", 6001 + index))
        if not 1 <= port <= 65535:
            raise ValueError(f"从臂{index + 1} 本机端口无效：{port}")
        followers.append(
            {
                "name": str(raw.get("name", f"从臂{index + 1}")),
                "enabled": enabled,
                "robot_ip": robot_ip,
                "robot_port": port,
            }
        )
    enabled_ips = [str(item["robot_ip"]) for item in followers if item["enabled"]]
    if len(enabled_ips) != len(set(enabled_ips)):
        raise ValueError("已启用的两台从臂不能使用相同的 IP 地址")
    if not enabled_ips:
        raise ValueError("至少需要启用一台从机械臂")
    return followers


def _read_pid(path: Path) -> int | None:
    try:
        return int(path.read_text().strip())
    except (FileNotFoundError, ValueError):
        return None


def _alive(pid: int | None) -> bool:
    if pid is None:
        return False
    try:
        stat = Path(f"/proc/{pid}/stat")
        if stat.exists() and stat.read_text().split()[2] == "Z":
            return False
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False


def _server_code_signature(
    followers: list[dict[str, object]] | None = None,
) -> str:
    if followers is None:
        followers = _load_followers()
    digest = hashlib.sha256()
    for path in SERVER_CODE_FILES:
        digest.update(str(path.relative_to(PROJECT)).encode("utf-8"))
        digest.update(path.read_bytes())
    runtime_config = [
        {
            "enabled": item["enabled"],
            "robot_ip": item["robot_ip"],
            "robot_port": item["robot_port"],
        }
        for item in followers
    ]
    digest.update(json.dumps(runtime_config, sort_keys=True).encode("utf-8"))
    return digest.hexdigest()


def _stop_server_process(pid: int | None) -> None:
    if _alive(pid):
        os.killpg(pid, signal.SIGINT)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and _alive(pid):
            time.sleep(0.05)
    if _alive(pid):
        os.killpg(pid, signal.SIGTERM)


def _launch(command: list[str], log_path: Path) -> int:
    env = os.environ.copy()
    env["PYTHONPATH"] = str(PROJECT)
    env["PYTHONUNBUFFERED"] = "1"
    log = log_path.open("a", buffering=1)
    process = subprocess.Popen(
        command,
        cwd=PROJECT,
        env=env,
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        text=True,
    )
    return process.pid


def _wait_for_port(port: int, timeout: float = 5.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.2):
                return
        except OSError:
            time.sleep(0.2)
    raise RuntimeError(f"robot server did not open port {port}")


def start() -> None:
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    if _alive(_read_pid(CONTROL_PID)):
        raise RuntimeError("master/slave control is already running; use status or stop")

    followers = _load_followers()
    expected_signature = _server_code_signature(followers)
    try:
        running_signature = SERVER_SIGNATURE.read_text().strip()
    except FileNotFoundError:
        running_signature = ""

    legacy_pid = _read_pid(LEGACY_SERVER_PID)
    if _alive(legacy_pid):
        print("stopping legacy single-arm SDK service...", flush=True)
        _stop_server_process(legacy_pid)
    LEGACY_SERVER_PID.unlink(missing_ok=True)

    if running_signature != expected_signature:
        print("robot IP/configuration or server code changed; restarting SDK services...", flush=True)
        for index in (1, 2):
            _stop_server_process(_read_pid(_server_pid(index)))
            _server_pid(index).unlink(missing_ok=True)
        SERVER_SIGNATURE.unlink(missing_ok=True)

    started_here: list[tuple[int, int]] = []
    server_pids: list[tuple[int, int]] = []
    for index, follower in enumerate(followers, start=1):
        pid_path = _server_pid(index)
        existing_pid = _read_pid(pid_path)
        if not bool(follower["enabled"]):
            _stop_server_process(existing_pid)
            pid_path.unlink(missing_ok=True)
            continue
        if _alive(existing_pid):
            pid = int(existing_pid)
            print(f"reusing {follower['name']} SDK server (pid={pid})...", flush=True)
        else:
            pid = _launch(
                [
                    sys.executable,
                    "experiments/launch_nodes.py",
                    "--robot", "iimt",
                    "--robot-ip", str(follower["robot_ip"]),
                    "--robot-port", str(follower["robot_port"]),
                    "--hostname", "127.0.0.1",
                    "--iimt-hz", "250",
                    "--allow-manual-servo",
                    "--iimt-max-relative-travel", "0",
                ],
                _server_log(index),
            )
            pid_path.write_text(f"{pid}\n")
            started_here.append((index, pid))
            print(
                f"starting {follower['name']} SDK server: "
                f"robot={follower['robot_ip']}, local_port={follower['robot_port']}",
                flush=True,
            )
        server_pids.append((index, pid))

    try:
        for follower in followers:
            if bool(follower["enabled"]):
                _wait_for_port(int(follower["robot_port"]))
        SERVER_SIGNATURE.write_text(expected_signature + "\n")
        control_command = [sys.executable, "experiments/iimt_master_slave.py"]
        if not started_here:
            control_command.append("--resume-servo-alignment")
        control_pid = _launch(control_command, CONTROL_LOG)
        CONTROL_PID.write_text(f"{control_pid}\n")
    except Exception:
        for index, pid in started_here:
            if _alive(pid):
                os.killpg(pid, signal.SIGTERM)
            _server_pid(index).unlink(missing_ok=True)
        if started_here:
            SERVER_SIGNATURE.unlink(missing_ok=True)
        raise
    pid_text = ", ".join(f"server{index}={pid}" for index, pid in server_pids)
    print(f"started: {pid_text}, controller={control_pid}")
    print(f"status: {sys.executable} {Path(__file__).name} status")
    print(f"logs:   {sys.executable} {Path(__file__).name} logs")
    print(f"stop:   {sys.executable} {Path(__file__).name} stop")


def stop() -> None:
    controller_pid = _read_pid(CONTROL_PID)
    if _alive(controller_pid):
        os.killpg(controller_pid, signal.SIGINT)
        # Keep the robot server alive while the controller clears the unfinished
        # Servoj trajectory through ZMQ.
        controller_deadline = time.monotonic() + 4.0
        while time.monotonic() < controller_deadline and _alive(controller_pid):
            time.sleep(0.05)
    if _alive(controller_pid):
        os.killpg(controller_pid, signal.SIGTERM)
    CONTROL_PID.unlink(missing_ok=True)
    print("master/slave control stopped; robot SDK server kept ready for restart")


def shutdown() -> None:
    stop()
    for index in (1, 2):
        _stop_server_process(_read_pid(_server_pid(index)))
        _server_pid(index).unlink(missing_ok=True)
    _stop_server_process(_read_pid(LEGACY_SERVER_PID))
    LEGACY_SERVER_PID.unlink(missing_ok=True)
    SERVER_SIGNATURE.unlink(missing_ok=True)
    print("all robot SDK servers fully shut down")


def get_status() -> dict[str, tuple[bool, int | None]]:
    """Return process state for UI and command-line frontends."""
    server_1 = _read_pid(_server_pid(1))
    server_2 = _read_pid(_server_pid(2))
    controller = _read_pid(CONTROL_PID)
    return {
        "server_1": (_alive(server_1), server_1),
        "server_2": (_alive(server_2), server_2),
        "controller": (_alive(controller), controller),
    }


def status() -> None:
    state = get_status()
    controller_alive, controller_pid = state["controller"]
    for index in (1, 2):
        server_alive, server_pid = state[f"server_{index}"]
        print(
            f"server {index}:   "
            f"{'running' if server_alive else 'stopped'} ({server_pid})"
        )
    print(
        "controller: "
        f"{'running' if controller_alive else 'stopped'} ({controller_pid})"
    )


def get_logs(lines: int = 40) -> str:
    """Return recent service logs without printing them."""
    sections = []
    log_files = (
        ("server 1", _server_log(1)),
        ("server 2", _server_log(2)),
        ("controller", CONTROL_LOG),
    )
    for label, path in log_files:
        sections.append(f"== {label}: {path} ==\n")
        if path.exists():
            content = "".join(path.read_text(errors="replace").splitlines(True)[-lines:])
            sections.append(content)
            if content and not content.endswith("\n"):
                sections.append("\n")
        else:
            sections.append("no log yet\n")
        sections.append("\n")
    return "".join(sections).rstrip() + "\n"


def logs() -> None:
    print(get_logs(), end="")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "command", choices=("start", "stop", "shutdown", "status", "logs")
    )
    command = parser.parse_args().command
    {
        "start": start,
        "stop": stop,
        "shutdown": shutdown,
        "status": status,
        "logs": logs,
    }[command]()


if __name__ == "__main__":
    main()
