#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
左臂控制器端 ZMQ v0 service。

部署位置:
    /root/IIMT-SDK-left/python/sdk_demo_py/zmq_left_iimt_v0.py

职责:
    - 绑定 tcp://0.0.0.0:5555，接收高频 servo_pose / servoj_joints 命令
    - 绑定 tcp://0.0.0.0:5556，接收阻塞式 home_movel / stop 命令并返回 ack
    - 绑定 tcp://0.0.0.0:5557，接收 XR router 发来的夹爪 open/close/enable/disable 命令
    - 在控制器端独占调用 API_Servoc / API_Movel / API_ModbusRtuRWEndTool

本机电脑只负责 XR 解析、相机/关节/夹爪状态读取和保存，不再本地执行机械臂控制。
"""

from __future__ import annotations

import argparse
import ctypes
import os
from queue import Empty, Queue
import threading
import time
from typing import Any, Dict, cast

import zmq

from iimt_api import (  # noqa: F403
    API_Login,
    API_GetRobotJointRad,
    API_GetRobotTcpPosition,
    API_ModbusRtuRWEndTool,
    API_Movel,
    API_Servoc,
    API_Servoj,
    API_Stop,
    MovelParam,
    ServocParam,
    ServojParam,
)
import iimt_api as _iimt_api
from iimt_ctypes import ModbusRTU_Frame, ModbusRTU_Request


BooleanOptionalAction: Any = getattr(argparse, "BooleanOptionalAction", None)
if BooleanOptionalAction is None:
    class _BooleanOptionalAction(argparse.Action):
        """Python 3.8 fallback for --foo / --no-foo boolean switches."""

        def __init__(self, option_strings, dest, default=None, required=False, help=None, **kwargs):
            expanded = []
            for option in option_strings:
                expanded.append(option)
                if option.startswith("--"):
                    expanded.append("--no-" + option[2:])
            super().__init__(
                option_strings=expanded,
                dest=dest,
                nargs=0,
                default=default,
                required=required,
                help=help,
                **kwargs,
            )

        def __call__(self, parser, namespace, values, option_string=None):
            setattr(namespace, self.dest, not str(option_string).startswith("--no-"))

    BooleanOptionalAction = _BooleanOptionalAction


def env_float(name: str, default: float) -> float:
    """读取 launcher/start_zmq_left_iimt.sh 传入的浮点参数；为空或非法时用默认值。"""
    value = os.getenv(name)
    if value is None or value.strip() == "":
        return float(default)
    try:
        return float(value)
    except ValueError:
        print(f"[config] invalid {name}={value!r}, use default {default}")
        return float(default)


def env_pose(name: str, default: tuple[float, ...]) -> tuple[float, ...]:
    """读取逗号分隔的 6D TCP home pose；为空或非法时用默认值。"""
    value = os.getenv(name, "").strip()
    if not value:
        return default
    try:
        pose = tuple(float(part.strip()) for part in value.split(","))
    except ValueError:
        print(f"[config] invalid {name}={value!r}, use default {default}")
        return default
    if len(pose) != 6:
        print(f"[config] invalid {name} length={len(pose)}, use default {default}")
        return default
    return pose


ARM_BIND = "tcp://0.0.0.0:5555"
COMMAND_BIND = "tcp://0.0.0.0:5556"
GRIPPER_BIND = "tcp://0.0.0.0:5557"
STATE_BIND = "tcp://0.0.0.0:7020"
STATE_QUEUE_MAXSIZE = 500
LEFT_HOME_POSE = env_pose("IIMT_LEFT_HOME_POSE", (-0.569950, -0.048710, 0.556020, 3.051580, 0.053010, -1.247870))
ARM_CONTROL_HZ = env_float("IIMT_ARM_CONTROL_HZ", 250.0)
# 超过这个时间没收到新的 servo_pose，就停止继续执行旧目标，避免松开侧键后还反复发旧点。
# IIMT_ARM_TARGET_STALE_SEC 不传或填 auto 时，按 ARM_CONTROL_HZ 自动计算。
_stale_env = os.getenv("IIMT_ARM_TARGET_STALE_SEC", "").strip().lower()
ARM_TARGET_STALE_SEC = (
    max(5.0 / ARM_CONTROL_HZ, 0.025)
    if _stale_env in ("", "auto", "none", "-")
    else env_float("IIMT_ARM_TARGET_STALE_SEC", max(5.0 / ARM_CONTROL_HZ, 0.025))
)
ARM_RECV_STATS_INTERVAL = env_float("IIMT_ARM_RECV_STATS_INTERVAL", 1.0)

SERVOC_A = env_float("IIMT_SERVOC_A", 7.0)
SERVOC_V = env_float("IIMT_SERVOC_V", 5.0)
SERVOC_T = env_float("IIMT_SERVOC_T", 1.0 / ARM_CONTROL_HZ)
SERVOC_LOOKAHEAD = env_float("IIMT_SERVOC_LOOKAHEAD", 0.04)
SERVOJ_A = env_float("IIMT_SERVOJ_A", 5.0)
SERVOJ_V = env_float("IIMT_SERVOJ_V", 5.0)
SERVOJ_T = env_float("IIMT_SERVOJ_T", 1.0 / ARM_CONTROL_HZ)
SERVOJ_LOOKAHEAD = env_float("IIMT_SERVOJ_LOOKAHEAD", 0.04)
SERVOJ_GAIN = env_float("IIMT_SERVOJ_GAIN", 0.4)
MOVEL_HOME_V = env_float("IIMT_MOVEL_HOME_V", 100.0)
MOVEL_HOME_TOOL_V = env_float("IIMT_MOVEL_HOME_TOOL_V", 1000.0)
MOVEL_HOME_TOOL_A = env_float("IIMT_MOVEL_HOME_TOOL_A", 1000.0)
MOVEL_HOME_TIMEOUT = env_float("IIMT_MOVEL_HOME_TIMEOUT", 30.0)

LOG_DETAIL_INTERVAL = max(0.5, ARM_RECV_STATS_INTERVAL)


DEFAULT_SLAVE = 0x09
ENABLE_ADDR = 1000
PARAM_ADDR = 1001
OPEN_POS = 0
CLOSE_POS = 255
DEFAULT_SPEED = 255
DEFAULT_TARGET_FORCE = 20
DEFAULT_TIMEOUT = 2
STATUS_ADDR = 2000
JOINT_STATE_HZ = env_float("IIMT_JOINT_STATE_HZ", 100.0)
TCP_STATE_HZ = env_float("IIMT_TCP_STATE_HZ", 100.0)
GRIPPER_STATE_HZ = env_float("IIMT_GRIPPER_STATE_HZ", 15.0)

GRIPPER_IO_LOCK = threading.RLock()
GRIPPER_COMMAND_PENDING = threading.Event()
GRIPPER_STATUS_PAUSE_LOCK = threading.Lock()
GRIPPER_STATUS_PAUSE_UNTIL = 0.0
GRIPPER_STATUS_PAUSE_AFTER_CMD_SEC = 0.20
GRIPPER_STATUS_READ_TIMEOUT = 1
CLAMP_STATES = {
    0: "正向指定位置移动",
    1: "张开方向检测到物体",
    2: "闭合方向检测到物体",
    3: "到达指定位置，未检测到物体",
}


class StatsHub:
    """线程安全统计中心。

    各工作线程只更新计数/耗时，统一由 stats_report_loop 打印，避免多线程
    同时 print 把日志刷乱。
    """

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.start = time.monotonic()
        self.arm_recv = {
            "seq": 0,
            "total": 0,
            "count": 0,
            "servoc": 0,
            "servoj": 0,
            "dt_sum": 0.0,
            "dt_max": 0.0,
            "last_recv": None,
        }
        self.command = {
            "seq": 0,
            "count": 0,
            "ok": 0,
            "fail": 0,
            "last": "-",
            "call_ms_sum": 0.0,
            "call_ms_max": 0.0,
        }
        self.arm_exec = {
            "total": 0,
            "ok_total": 0,
            "fail_total": 0,
            "count": 0,
            "ok": 0,
            "fail": 0,
            "last": "-",
            "recv_dt_sum": 0.0,
            "recv_dt_max": 0.0,
            "exec_dt_sum": 0.0,
            "exec_dt_max": 0.0,
            "age_sum": 0.0,
            "age_max": 0.0,
            "loop_gap_sum": 0.0,
            "loop_gap_max": 0.0,
            "loop_gap_count": 0,
            "call_ms_sum": 0.0,
            "call_ms_max": 0.0,
            "stale_drop": 0,
            "no_target": 0,
        }
        self.state = {
            "pub_sent": 0,
            "pub_fail": 0,
            "pub_qsize": 0,
            "joints_seq": 0,
            "joints_fail": 0,
            "tcp_seq": 0,
            "tcp_fail": 0,
            "gripper_seq": 0,
            "gripper_fail": 0,
        }
        self.state_read = {
            "joints_count": 0,
            "joints_read_ms_sum": 0.0,
            "joints_read_ms_max": 0.0,
            "tcp_count": 0,
            "tcp_read_ms_sum": 0.0,
            "tcp_read_ms_max": 0.0,
            "gripper_count": 0,
            "gripper_read_ms_sum": 0.0,
            "gripper_read_ms_max": 0.0,
        }
        self.gripper_read_detail = {
            "api_ms_sum": 0.0,
            "api_ms_max": 0.0,
            "parse_ms_sum": 0.0,
            "parse_ms_max": 0.0,
            "active_count": 0,
            "active_ms_sum": 0.0,
            "active_ms_max": 0.0,
            "idle_count": 0,
            "idle_ms_sum": 0.0,
            "idle_ms_max": 0.0,
            "timeout_like": 0,
            "skip": 0,
            "last_ret": "-",
            "last_len": 0,
            "last_reason": "-",
            "last_raw": "-",
        }
        self.arm_active = False

    def mark_arm_recv(self, cmd: str, now: float) -> int:
        with self.lock:
            s = self.arm_recv
            s["seq"] += 1
            s["total"] += 1
            s["count"] += 1
            if cmd == "servo_pose":
                s["servoc"] += 1
            elif cmd == "servoj_joints":
                s["servoj"] += 1
            last = s["last_recv"]
            if last is not None:
                dt = max(0.0, now - float(last))
                s["dt_sum"] += dt
                s["dt_max"] = max(float(s["dt_max"]), dt)
            s["last_recv"] = now
            return int(s["seq"])

    def mark_command(self, cmd: str, ok: bool, call_ms: float) -> int:
        with self.lock:
            s = self.command
            s["seq"] += 1
            s["count"] += 1
            s["ok"] += int(ok)
            s["fail"] += int(not ok)
            s["last"] = cmd
            s["call_ms_sum"] += max(0.0, float(call_ms))
            s["call_ms_max"] = max(float(s["call_ms_max"]), max(0.0, float(call_ms)))
            return int(s["seq"])

    def mark_arm_loop_gap(self, gap: float) -> None:
        with self.lock:
            s = self.arm_exec
            s["loop_gap_sum"] += max(0.0, gap)
            s["loop_gap_max"] = max(float(s["loop_gap_max"]), max(0.0, gap))
            s["loop_gap_count"] += 1

    def mark_no_target(self) -> None:
        with self.lock:
            self.arm_exec["no_target"] += 1
            self.arm_active = False

    def mark_stale_drop(self) -> None:
        with self.lock:
            self.arm_exec["stale_drop"] += 1
            self.arm_active = False

    def mark_arm_exec(
        self,
        ok: bool,
        cmd: str,
        recv_dt: float | None,
        exec_dt: float | None,
        target_age: float,
        call_ms: float,
    ) -> None:
        with self.lock:
            s = self.arm_exec
            s["total"] += 1
            s["ok_total"] += int(ok)
            s["fail_total"] += int(not ok)
            s["count"] += 1
            s["ok"] += int(ok)
            s["fail"] += int(not ok)
            s["last"] = cmd
            if recv_dt is not None:
                s["recv_dt_sum"] += max(0.0, recv_dt)
                s["recv_dt_max"] = max(float(s["recv_dt_max"]), max(0.0, recv_dt))
            if exec_dt is not None:
                s["exec_dt_sum"] += max(0.0, exec_dt)
                s["exec_dt_max"] = max(float(s["exec_dt_max"]), max(0.0, exec_dt))
            s["age_sum"] += max(0.0, target_age)
            s["age_max"] = max(float(s["age_max"]), max(0.0, target_age))
            s["call_ms_sum"] += max(0.0, call_ms)
            s["call_ms_max"] = max(float(s["call_ms_max"]), max(0.0, call_ms))
            self.arm_active = bool(ok and cmd == "servo_pose")

    def is_arm_active(self) -> bool:
        with self.lock:
            return bool(self.arm_active)

    def mark_state_pub(self, sent: int, fail: int, qsize: int) -> None:
        with self.lock:
            self.state["pub_sent"] = sent
            self.state["pub_fail"] = fail
            self.state["pub_qsize"] = qsize

    def mark_state_stream(self, name: str, seq: int, fail: int,
                          read_ms: float | None = None) -> None:
        with self.lock:
            self.state[f"{name}_seq"] = seq
            self.state[f"{name}_fail"] = fail
            if read_ms is not None:
                s = self.state_read
                s[f"{name}_count"] += 1
                s[f"{name}_read_ms_sum"] += max(0.0, float(read_ms))
                s[f"{name}_read_ms_max"] = max(
                    float(s[f"{name}_read_ms_max"]),
                    max(0.0, float(read_ms)),
                )

    def mark_gripper_read_detail(self, diag: dict) -> None:
        with self.lock:
            d = self.gripper_read_detail
            total_ms = max(0.0, float(diag.get("total_ms", 0.0)))
            api_ms = max(0.0, float(diag.get("api_ms", 0.0)))
            parse_ms = max(0.0, float(diag.get("parse_ms", 0.0)))
            d["api_ms_sum"] += api_ms
            d["api_ms_max"] = max(float(d["api_ms_max"]), api_ms)
            d["parse_ms_sum"] += parse_ms
            d["parse_ms_max"] = max(float(d["parse_ms_max"]), parse_ms)
            if diag.get("arm_active"):
                d["active_count"] += 1
                d["active_ms_sum"] += total_ms
                d["active_ms_max"] = max(float(d["active_ms_max"]), total_ms)
            else:
                d["idle_count"] += 1
                d["idle_ms_sum"] += total_ms
                d["idle_ms_max"] = max(float(d["idle_ms_max"]), total_ms)
            if diag.get("timeout_like"):
                d["timeout_like"] += 1
            if diag.get("skip"):
                d["skip"] += 1
            d["last_ret"] = diag.get("ret", "-")
            d["last_len"] = int(diag.get("raw_len", 0) or 0)
            d["last_reason"] = str(diag.get("fail_reason", "-"))
            d["last_raw"] = str(diag.get("raw_hex", "-"))

    def snapshot_and_reset_window(self) -> dict:
        with self.lock:
            arm_recv = dict(self.arm_recv)
            arm_exec = dict(self.arm_exec)
            state = dict(self.state)
            state_read = dict(self.state_read)
            gripper_read_detail = dict(self.gripper_read_detail)
            self.arm_recv.update({
                "count": 0,
                "servoc": 0,
                "servoj": 0,
                "dt_sum": 0.0,
                "dt_max": 0.0,
            })
            command = dict(self.command)
            self.command.update({
                "count": 0,
                "ok": 0,
                "fail": 0,
                "call_ms_sum": 0.0,
                "call_ms_max": 0.0,
            })
            self.arm_exec.update({
                "count": 0,
                "ok": 0,
                "fail": 0,
                "recv_dt_sum": 0.0,
                "recv_dt_max": 0.0,
                "exec_dt_sum": 0.0,
                "exec_dt_max": 0.0,
                "age_sum": 0.0,
                "age_max": 0.0,
                "loop_gap_sum": 0.0,
                "loop_gap_max": 0.0,
                "loop_gap_count": 0,
                "call_ms_sum": 0.0,
                "call_ms_max": 0.0,
                "stale_drop": 0,
                "no_target": 0,
            })
            for name in ("joints", "tcp", "gripper"):
                self.state_read[f"{name}_count"] = 0
                self.state_read[f"{name}_read_ms_sum"] = 0.0
                self.state_read[f"{name}_read_ms_max"] = 0.0
            self.gripper_read_detail.update({
                "api_ms_sum": 0.0,
                "api_ms_max": 0.0,
                "parse_ms_sum": 0.0,
                "parse_ms_max": 0.0,
                "active_count": 0,
                "active_ms_sum": 0.0,
                "active_ms_max": 0.0,
                "idle_count": 0,
                "idle_ms_sum": 0.0,
                "idle_ms_max": 0.0,
                "timeout_like": 0,
                "skip": 0,
            })
        return {
            "arm_recv": arm_recv,
            "arm_exec": arm_exec,
            "command": command,
            "state": state,
            "state_read": state_read,
            "gripper_read_detail": gripper_read_detail,
        }


def stats_report_loop(stats: StatsHub, state_queue: Queue | None,
                      stop_event: threading.Event) -> None:
    """统一打印运行状态。"""
    last = time.monotonic()
    while not stop_event.wait(LOG_DETAIL_INTERVAL):
        now = time.monotonic()
        elapsed = max(1e-9, now - last)
        snap = stats.snapshot_and_reset_window()
        recv = snap["arm_recv"]
        exec_s = snap["arm_exec"]
        command = snap["command"]
        state = snap["state"]
        state_read = snap["state_read"]
        grip_detail = snap["gripper_read_detail"]

        recv_count = int(recv["count"])
        recv_denom = max(1, recv_count - 1)
        exec_count = int(exec_s["count"])
        exec_denom = max(1, exec_count)
        exec_dt_denom = max(1, exec_count - 1)
        loop_denom = max(1, int(exec_s["loop_gap_count"]))
        qsize = state_queue.qsize() if state_queue is not None else int(state["pub_qsize"])

        def read_line(name: str, target_hz: float) -> str:
            count = int(state_read[f"{name}_count"])
            denom = max(1, count)
            return (
                f"{name:<7}: hz={count / elapsed:6.1f}/{target_hz:<5.1f} "
                f"read_ms_avg/max={float(state_read[f'{name}_read_ms_sum']) / denom:6.2f}/"
                f"{float(state_read[f'{name}_read_ms_max']):6.2f} "
                f"seq/fail={int(state[f'{name}_seq'])}/{int(state[f'{name}_fail'])}"
            )

        print("", flush=True)
        print("=" * 96, flush=True)
        print(
            f"[LEFT-ZMQ-V0] uptime={now - stats.start:7.1f}s | "
            f"control={ARM_CONTROL_HZ:.1f}Hz | servoc_t={SERVOC_T:.4f}s | servoj_t={SERVOJ_T:.4f}s | "
            f"stale={ARM_TARGET_STALE_SEC * 1000.0:.1f}ms",
            flush=True,
        )
        print(
            f"arm recv : hz={recv_count / elapsed:6.1f} "
            f"dt_avg/max={float(recv['dt_sum']) / recv_denom * 1000.0:6.2f}/"
            f"{float(recv['dt_max']) * 1000.0:6.2f}ms | "
            f"servoc={int(recv['servoc']):4d} servoj={int(recv['servoj']):4d} seq={int(recv['seq'])}",
            flush=True,
        )
        print(
            f"arm exec : hz={exec_count / elapsed:6.1f}/{ARM_CONTROL_HZ:<5.1f} "
            f"ok/fail={int(exec_s['ok']):4d}/{int(exec_s['fail']):2d} "
            f"total={int(exec_s['ok_total'])}/{int(exec_s['fail_total'])} | "
            f"last={exec_s['last']} stale/no_target={int(exec_s['stale_drop'])}/{int(exec_s['no_target'])}",
            flush=True,
        )
        print(
            f"           exec_dt_avg/max={float(exec_s['exec_dt_sum']) / exec_dt_denom * 1000.0:6.2f}/"
            f"{float(exec_s['exec_dt_max']) * 1000.0:6.2f}ms | "
            f"age_avg/max={float(exec_s['age_sum']) / exec_denom * 1000.0:6.2f}/"
            f"{float(exec_s['age_max']) * 1000.0:6.2f}ms | "
            f"call_avg/max={float(exec_s['call_ms_sum']) / exec_denom:6.3f}/"
            f"{float(exec_s['call_ms_max']):6.3f}ms",
            flush=True,
        )
        print(
            f"           loop_gap_avg/max={float(exec_s['loop_gap_sum']) / loop_denom * 1000.0:6.2f}/"
            f"{float(exec_s['loop_gap_max']) * 1000.0:6.2f}ms",
            flush=True,
        )
        print("state read:", flush=True)
        print(f"  {read_line('joints', JOINT_STATE_HZ)}", flush=True)
        print(f"  {read_line('tcp', TCP_STATE_HZ)}", flush=True)
        print(f"  {read_line('gripper', GRIPPER_STATE_HZ)}", flush=True)
        grip_count = max(1, int(state_read["gripper_count"]))
        active_count = max(1, int(grip_detail["active_count"]))
        idle_count = max(1, int(grip_detail["idle_count"]))
        print(
            f"  grip detail: api_ms_avg/max={float(grip_detail['api_ms_sum']) / grip_count:6.2f}/"
            f"{float(grip_detail['api_ms_max']):6.2f} | "
            f"parse_ms_avg/max={float(grip_detail['parse_ms_sum']) / grip_count:5.2f}/"
            f"{float(grip_detail['parse_ms_max']):5.2f}",
            flush=True,
        )
        print(
            f"               active_ms_avg/max/cnt={float(grip_detail['active_ms_sum']) / active_count:6.2f}/"
            f"{float(grip_detail['active_ms_max']):6.2f}/{int(grip_detail['active_count'])} | "
            f"idle_ms_avg/max/cnt={float(grip_detail['idle_ms_sum']) / idle_count:6.2f}/"
            f"{float(grip_detail['idle_ms_max']):6.2f}/{int(grip_detail['idle_count'])} | "
            f"timeout_like={int(grip_detail['timeout_like'])} skip={int(grip_detail['skip'])}",
            flush=True,
        )
        print(
            f"               last_ret={grip_detail['last_ret']} last_len={int(grip_detail['last_len'])} "
            f"last_reason={grip_detail['last_reason']} last_raw={grip_detail['last_raw']}",
            flush=True,
        )
        command_count = int(command["count"])
        command_denom = max(1, command_count)
        print(
            f"command  : hz={command_count / elapsed:6.1f} ok/fail={int(command['ok'])}/{int(command['fail'])} "
            f"last={command['last']} call_avg/max={float(command['call_ms_sum']) / command_denom:6.2f}/"
            f"{float(command['call_ms_max']):6.2f}ms seq={int(command['seq'])}",
            flush=True,
        )
        print(
            f"state pub : sent/fail/q={int(state['pub_sent'])}/{int(state['pub_fail'])}/{qsize}",
            flush=True,
        )
        print("=" * 96, flush=True)
        last = now


def clamp_u8(value: int) -> int:
    """限制到 Jodell 夹爪常用的 0~255 参数范围。"""
    return max(0, min(255, int(value)))


def pack_position(position: int) -> int:
    """把 0~255 的夹爪目标位置打包成寄存器 1001 的值。"""
    return (clamp_u8(position) << 8) & 0xFFFF


def pack_speed_force(speed: int, target_force: int) -> int:
    """把目标速度和目标力打包成寄存器 1002 的值。"""
    return clamp_u8(speed) | (clamp_u8(target_force) << 8)


def put_latest(queue: Queue, item: dict) -> None:
    """只保留队列里的最新命令，避免控制器积压旧目标。"""
    while True:
        try:
            queue.get_nowait()
        except Empty:
            break
    queue.put(item)


def pause_gripper_status(seconds: float = GRIPPER_STATUS_PAUSE_AFTER_CMD_SEC) -> None:
    """暂停夹爪状态读取一小段时间，把 Modbus 通道优先让给控制命令。"""
    global GRIPPER_STATUS_PAUSE_UNTIL
    with GRIPPER_STATUS_PAUSE_LOCK:
        GRIPPER_STATUS_PAUSE_UNTIL = max(
            GRIPPER_STATUS_PAUSE_UNTIL,
            time.monotonic() + float(seconds),
        )


def gripper_status_paused() -> bool:
    """判断当前是否处于夹爪控制命令保护窗口。"""
    with GRIPPER_STATUS_PAUSE_LOCK:
        return time.monotonic() < GRIPPER_STATUS_PAUSE_UNTIL


def bind_arm_socket(ctx: zmq.Context) -> zmq.Socket:
    """创建机械臂高频 action SUB socket。"""
    sock = ctx.socket(zmq.SUB)
    sock.bind(ARM_BIND)
    sock.setsockopt_string(zmq.SUBSCRIBE, "")
    sock.setsockopt(zmq.RCVHWM, 1)
    return sock


def bind_command_socket(ctx: zmq.Context) -> zmq.Socket:
    """创建阻塞 command RPC socket。"""
    sock = ctx.socket(zmq.REP)
    sock.bind(COMMAND_BIND)
    return sock


def bind_gripper_socket(ctx: zmq.Context) -> zmq.Socket:
    """创建夹爪命令 PULL socket。"""
    sock = ctx.socket(zmq.PULL)
    sock.bind(GRIPPER_BIND)
    sock.setsockopt(zmq.RCVHWM, 20)
    return sock


def bind_state_socket(ctx: zmq.Context) -> zmq.Socket:
    """创建状态 PUB socket，给本地 record_raw.py 订阅。"""
    sock = ctx.socket(zmq.PUB)
    sock.bind(STATE_BIND)
    sock.setsockopt(zmq.SNDHWM, 200)
    return sock


def publish_state(state_queue: Queue | None, msg: dict) -> None:
    """把状态消息交给唯一的 PUB 线程发送，避免 ZMQ socket 跨线程共享。"""
    if state_queue is None:
        return
    while True:
        try:
            state_queue.put_nowait(msg)
            return
        except Exception:
            try:
                state_queue.get_nowait()
            except Empty:
                return


def state_publisher_loop(state_sock: zmq.Socket, state_queue: Queue,
                         stats: StatsHub,
                         stop_event: threading.Event) -> None:
    """唯一拥有 7020 PUB socket 的线程。

    pyzmq socket 不能跨线程并发使用。关节、夹爪状态和夹爪命令事件
    都先进入 state_queue，再由这个线程统一 send_json。外部 topic 和字段
    保持不变。
    """
    sent = 0
    fail = 0
    while not stop_event.is_set() or not state_queue.empty():
        try:
            msg = state_queue.get(timeout=0.05)
        except Empty:
            msg = None
        if msg is not None:
            try:
                state_sock.send_json(msg)
                sent += 1
            except zmq.ZMQError as exc:
                fail += 1
                print(f"[state-pub] send failed: {exc}", flush=True)
        stats.mark_state_pub(sent, fail, state_queue.qsize())


def arm_recv_loop(sock: zmq.Socket, queue: Queue,
                  stats: StatsHub,
                  stop_event: threading.Event) -> None:
    """接收机械臂高频 action，只保留最新目标。"""
    poller = zmq.Poller()
    poller.register(sock, zmq.POLLIN)
    while not stop_event.is_set():
        events = dict(poller.poll(20))
        now = time.monotonic()
        if sock in events:
            try:
                msg = sock.recv_json(flags=zmq.NOBLOCK)
            except zmq.ZMQError:
                continue
            if not isinstance(msg, dict):
                continue
            msg = cast(Dict[str, Any], msg)
            if msg.get("arm", "left") != "left":
                continue
            cmd = str(msg.get("cmd", ""))
            if cmd not in {"servo_pose", "servoj_joints", "hold"}:
                continue
            msg["_remote_recv_mono"] = now
            recv_seq = stats.mark_arm_recv(cmd, now)
            msg["_remote_recv_seq"] = recv_seq
            put_latest(queue, msg)


def gripper_recv_loop(sock: zmq.Socket, queue: Queue, stop_event: threading.Event) -> None:
    """接收夹爪命令。"""
    poller = zmq.Poller()
    poller.register(sock, zmq.POLLIN)
    while not stop_event.is_set():
        events = dict(poller.poll(20))
        if sock not in events:
            continue
        try:
            msg = sock.recv_json(flags=zmq.NOBLOCK)
        except zmq.ZMQError:
            continue
        if not isinstance(msg, dict):
            continue
        msg = cast(Dict[str, Any], msg)
        if msg.get("arm", "left") != "left":
            continue
        if str(msg.get("cmd", "")) in {"open", "close", "enable", "disable"}:
            GRIPPER_COMMAND_PENDING.set()
            pause_gripper_status()
        put_latest(queue, msg)


def write_gripper_registers(
    handle,
    start_addr: int,
    values: list[int],
    timeout: int = DEFAULT_TIMEOUT,
    locked: bool = False,
) -> bool:
    """用 API_ModbusRtuRWEndTool 写夹爪寄存器。"""
    req = ModbusRTU_Request()
    rsp = ModbusRTU_Frame()
    req.slave_addr = DEFAULT_SLAVE
    req.func_code = 0x10
    req.start_addr = int(start_addr)
    req.data_number = len(values)
    req.data_len = len(values) * 2
    for idx, value in enumerate(values):
        word = int(value) & 0xFFFF
        req.data[idx * 2] = (word >> 8) & 0xFF
        req.data[idx * 2 + 1] = word & 0xFF
    if locked:
        return bool(API_ModbusRtuRWEndTool(handle, req, rsp, timeout))
    with GRIPPER_IO_LOCK:
        return bool(API_ModbusRtuRWEndTool(handle, req, rsp, timeout))


def parse_registers(frame: ModbusRTU_Frame) -> list[int]:
    """解析 Modbus 0x04 输入寄存器响应。"""
    data_len = int(frame.data_len)
    if data_len < 3 or int(frame.data[1]) != 0x04:
        return []
    byte_count = int(frame.data[2])
    values = []
    start = 3
    end = min(start + byte_count, data_len)
    for i in range(start, end - 1, 2):
        values.append((int(frame.data[i]) << 8) | int(frame.data[i + 1]))
    return values


def frame_hex(frame: ModbusRTU_Frame, max_bytes: int = 32) -> str:
    """把 Modbus 响应前几个字节转成 hex，方便判断底层返回是否正常。"""
    data_len = int(frame.data_len)
    if data_len <= 0:
        return "-"
    end = min(data_len, max_bytes)
    return " ".join(f"{int(frame.data[i]) & 0xFF:02X}" for i in range(end))


def read_gripper_registers(handle, start_addr: int, count: int,
                           timeout: int = GRIPPER_STATUS_READ_TIMEOUT) -> tuple[bool, list[int], dict]:
    """读取夹爪输入寄存器。

    状态读取是低优先级：只尝试非阻塞拿锁。拿不到锁说明控制命令正在执行，
    本次状态采样直接跳过，避免 open/close 排在读取后面等待。
    """
    req = ModbusRTU_Request()
    rsp = ModbusRTU_Frame()
    req.slave_addr = DEFAULT_SLAVE
    req.func_code = 0x04
    req.start_addr = int(start_addr)
    req.data_number = int(count)
    req.data_len = 0
    diag = {
        "ret": "-",
        "raw_len": 0,
        "raw_hex": "-",
        "api_ms": 0.0,
        "parse_ms": 0.0,
        "fail_reason": "-",
        "timeout_like": False,
        "skip": False,
    }
    if GRIPPER_COMMAND_PENDING.is_set() or gripper_status_paused():
        diag["skip"] = True
        diag["fail_reason"] = "paused_for_command"
        return False, [], diag
    if not GRIPPER_IO_LOCK.acquire(blocking=False):
        diag["skip"] = True
        diag["fail_reason"] = "io_lock_busy"
        return False, [], diag
    try:
        if GRIPPER_COMMAND_PENDING.is_set() or gripper_status_paused():
            diag["skip"] = True
            diag["fail_reason"] = "paused_after_lock"
            return False, [], diag
        api_t0 = time.perf_counter()
        ret = API_ModbusRtuRWEndTool(handle, req, rsp, timeout)
        diag["api_ms"] = (time.perf_counter() - api_t0) * 1000.0
        diag["ret"] = int(ret) if isinstance(ret, (bool, int)) else str(ret)
        ok = bool(ret)
    finally:
        GRIPPER_IO_LOCK.release()
    diag["raw_len"] = int(rsp.data_len)
    diag["raw_hex"] = frame_hex(rsp)
    if not ok:
        diag["fail_reason"] = "api_return_false"
        diag["timeout_like"] = diag["api_ms"] >= float(timeout) * 1000.0 * 0.8
        return False, [], diag
    parse_t0 = time.perf_counter()
    values = parse_registers(rsp)
    diag["parse_ms"] = (time.perf_counter() - parse_t0) * 1000.0
    if len(values) < count:
        diag["fail_reason"] = f"short_values_{len(values)}"
        return False, values, diag
    return True, values, diag


def read_gripper_status(handle) -> tuple[dict | None, dict]:
    """读取夹爪状态，返回可直接发送的 dict。"""
    total_t0 = time.perf_counter()
    ok, values, diag = read_gripper_registers(handle, STATUS_ADDR, 3)
    diag["total_ms"] = (time.perf_counter() - total_t0) * 1000.0
    remote_mono = time.monotonic()
    remote_system = time.time()
    if not ok or len(values) < 3:
        if diag.get("fail_reason", "-") == "-":
            diag["fail_reason"] = "invalid_values"
        return None, diag
    state = (values[0] >> 6) & 0x03
    position = values[1] >> 8
    return {
        "remote_system_time": remote_system,
        "remote_mono_time": remote_mono,
        "left_gripper_position": int(position),
        "left_gripper_state": int(state),
        "left_gripper_state_text": CLAMP_STATES.get(state, "未知"),
        "left_gripper_target_force": int(DEFAULT_TARGET_FORCE),
        "left_gripper_motion_feedback": int(values[2] >> 8),
        "left_gripper_raw_2000": int(values[0]),
        "left_gripper_raw_2001": int(values[1]),
        "left_gripper_raw_2002": int(values[2]),
        "left_gripper_open": bool(position < 128),
    }, diag


def _api_login() -> tuple[bool, Any]:
    """把 SDK 可能返回 None 的 API_Login 统一成可安全解包的结果。"""
    result = API_Login()
    if result is None:
        return False, None
    return bool(result[0]), result[1]


def _api_get_joint_rad(handle) -> tuple[bool, Any]:
    """把关节读取 API 的可选返回值收窄。"""
    result = API_GetRobotJointRad(handle)
    if result is None:
        return False, None
    return bool(result[0]), result[1]


def _api_get_tcp_position(handle) -> tuple[bool, Any]:
    """把 TCP 读取 API 的可选返回值收窄。"""
    result = API_GetRobotTcpPosition(handle)
    if result is None:
        return False, None
    return bool(result[0]), result[1]


def joint_state_publish_loop(state_queue: Queue | None, joint_handle,
                             stats: StatsHub,
                             stop_event: threading.Event) -> None:
    """远端独立发布关节状态，避免被夹爪 Modbus 读取拖慢。

    本地 record_raw.py 收到消息时会再打本地 mono_time，
    用于和本地相机 mono_time 同一时间轴同步。
    """
    joint_interval = 1.0 / JOINT_STATE_HZ
    next_joint = time.monotonic()
    joint_seq = 0
    joint_fail = 0
    while not stop_event.is_set():
        now = time.monotonic()
        if now >= next_joint:
            read_t0 = time.perf_counter()
            ok, joint_rad = _api_get_joint_rad(joint_handle)
            read_ms = (time.perf_counter() - read_t0) * 1000.0
            remote_mono = time.monotonic()
            if ok and joint_rad is not None:
                joint_seq += 1
                publish_state(state_queue, {
                    "topic": "left.joints",
                    "arm": "left",
                    "seq": joint_seq,
                    "remote_system_time": time.time(),
                    "remote_mono_time": remote_mono,
                    "joints": [float(joint_rad.robotRad[i]) for i in range(6)],
                    "valid": True,
                })
            else:
                joint_fail += 1
            next_joint += joint_interval
            if next_joint < remote_mono - joint_interval:
                next_joint = remote_mono + joint_interval

            stats.mark_state_stream("joints", joint_seq, joint_fail, read_ms)
        stop_event.wait(0.0005)


def tcp_state_publish_loop(state_queue: Queue | None, tcp_handle,
                           stats: StatsHub,
                           stop_event: threading.Event) -> None:
    """远端独立发布 TCP 末端位姿，用于 XR monitor/后续原始数据记录。

    频率默认 30Hz，避免在高频 Servoc 时给控制器端额外增加过多 SDK 读取压力。
    """
    tcp_interval = 1.0 / TCP_STATE_HZ
    next_tcp = time.monotonic()
    tcp_seq = 0
    tcp_fail = 0
    while not stop_event.is_set():
        now = time.monotonic()
        if now >= next_tcp:
            read_t0 = time.perf_counter()
            ok, tcp_position = _api_get_tcp_position(tcp_handle)
            read_ms = (time.perf_counter() - read_t0) * 1000.0
            remote_mono = time.monotonic()
            if ok and tcp_position is not None:
                tcp = [float(tcp_position.robotTcp[i]) for i in range(6)]
                tcp_seq += 1
                publish_state(state_queue, {
                    "topic": "left.tcp",
                    "arm": "left",
                    "seq": tcp_seq,
                    "remote_system_time": time.time(),
                    "remote_mono_time": remote_mono,
                    "tcp": tcp,
                    "tcp_xyz": tcp[:3],
                    "tcp_rotvec": tcp[3:],
                    "valid": True,
                })
            else:
                tcp_fail += 1
            next_tcp += tcp_interval
            if next_tcp < remote_mono - tcp_interval:
                next_tcp = remote_mono + tcp_interval

            stats.mark_state_stream("tcp", tcp_seq, tcp_fail, read_ms)
        stop_event.wait(0.0005)


def gripper_state_publish_loop(state_queue: Queue | None, gripper_handle,
                               stats: StatsHub,
                               stop_event: threading.Event) -> None:
    """远端独立发布夹爪状态；Modbus 慢也不影响关节状态发布。"""
    gripper_interval = 1.0 / GRIPPER_STATE_HZ
    next_gripper = time.monotonic()
    gripper_seq = 0
    gripper_fail = 0
    while not stop_event.is_set():
        now = time.monotonic()
        if now >= next_gripper:
            read_t0 = time.perf_counter()
            status, diag = read_gripper_status(gripper_handle)
            read_ms = (time.perf_counter() - read_t0) * 1000.0
            diag["total_ms"] = read_ms
            diag["arm_active"] = stats.is_arm_active()
            remote_mono = time.monotonic()
            if status is not None:
                gripper_seq += 1
                status.update({
                    "topic": "left.gripper",
                    "arm": "left",
                    "seq": gripper_seq,
                    "valid": True,
                })
                publish_state(state_queue, status)
            else:
                gripper_fail += 1
            next_gripper += gripper_interval
            if next_gripper < remote_mono - gripper_interval:
                next_gripper = remote_mono + gripper_interval

            stats.mark_state_stream("gripper", gripper_seq, gripper_fail, read_ms)
            stats.mark_gripper_read_detail(diag)
            if read_ms >= 100.0 and not diag.get("skip"):
                print(
                    f"[GRIP-READ-SLOW] cost={read_ms:.1f}ms "
                    f"arm_active={bool(diag.get('arm_active'))} "
                    f"ret={diag.get('ret')} len={int(diag.get('raw_len', 0) or 0)} "
                    f"api_ms={float(diag.get('api_ms', 0.0)):.1f} "
                    f"parse_ms={float(diag.get('parse_ms', 0.0)):.2f} "
                    f"reason={diag.get('fail_reason', '-')} raw={diag.get('raw_hex', '-')}",
                    flush=True,
                )
        stop_event.wait(0.001)


def move_gripper(handle, position: int, speed: int, target_force: int) -> tuple[bool, dict[str, float]]:
    """写目标位置/速度/目标力，然后执行动作。

    夹爪上使能在 service 启动时做一次。open/close 时不再重复 enable，
    只做“写参数 + 执行动作”，减少 Modbus 往返和响应延迟。
    """
    timings = {}
    GRIPPER_COMMAND_PENDING.set()
    pause_gripper_status()
    lock_start = time.perf_counter()
    GRIPPER_IO_LOCK.acquire()
    timings["wait_ms"] = (time.perf_counter() - lock_start) * 1000.0
    try:
        param_start = time.perf_counter()
        ok_param = write_gripper_registers(
            handle,
            PARAM_ADDR,
            [pack_position(position), pack_speed_force(speed, target_force)],
            locked=True,
        )
        timings["param_ms"] = (time.perf_counter() - param_start) * 1000.0
        run_start = time.perf_counter()
        ok_run = write_gripper_registers(handle, ENABLE_ADDR, [9], locked=True)
        timings["run_ms"] = (time.perf_counter() - run_start) * 1000.0
    finally:
        GRIPPER_IO_LOCK.release()
        pause_gripper_status()
        GRIPPER_COMMAND_PENDING.clear()
    return bool(ok_param and ok_run), timings


def enable_gripper(handle, enable: bool) -> bool:
    """上使能或下使能夹爪。"""
    return write_gripper_registers(handle, ENABLE_ADDR, [1 if enable else 0])


def publish_gripper_command(
    state_queue: Queue | None,
    cmd: str,
    ok: bool,
    position: int | None,
    speed: int,
    target_force: int,
    cost_ms: float,
    timings: dict[str, float],
) -> None:
    """发布夹爪控制事件，给本地 record_raw.py 保存 gripper_command.jsonl。

    这是训练用的稳定夹爪信号：命令发出后立即有事件，不依赖慢速 Modbus
    状态轮询。实际 gripper status 仍然单独发布，用于诊断。
    """
    if cmd == "open":
        gripper_open = True
    elif cmd == "close":
        gripper_open = False
    else:
        gripper_open = None
    publish_state(state_queue, {
        "topic": "left.gripper_cmd",
        "arm": "left",
        "cmd": cmd,
        "ok": bool(ok),
        "remote_system_time": time.time(),
        "remote_mono_time": time.monotonic(),
        "left_gripper_command": cmd,
        "left_gripper_open": gripper_open,
        "left_gripper_target_position": position,
        "left_gripper_target_speed": int(speed),
        "left_gripper_target_force": int(target_force),
        "left_gripper_command_cost_ms": round(float(cost_ms), 3),
        "left_gripper_command_wait_ms": round(float(timings.get("wait_ms", 0.0)), 3),
        "left_gripper_command_param_ms": round(float(timings.get("param_ms", 0.0)), 3),
        "left_gripper_command_run_ms": round(float(timings.get("run_ms", 0.0)), 3),
    })


def gripper_worker(handle, queue: Queue, state_queue: Queue | None,
                   stop_event: threading.Event) -> None:
    """执行夹爪命令。"""
    while not stop_event.is_set():
        try:
            msg = queue.get(timeout=0.05)
        except Empty:
            continue
        cmd = str(msg.get("cmd", ""))
        speed = int(msg.get("speed", DEFAULT_SPEED))
        target_force = int(msg.get("target_force", DEFAULT_TARGET_FORCE))
        start = time.perf_counter()
        timings = {}
        position = None
        if cmd == "open":
            position = OPEN_POS
            ok, timings = move_gripper(handle, position, speed, target_force)
        elif cmd == "close":
            position = CLOSE_POS
            ok, timings = move_gripper(handle, position, speed, target_force)
        elif cmd == "enable":
            ok = enable_gripper(handle, True)
        elif cmd == "disable":
            ok = enable_gripper(handle, False)
        else:
            ok = False
        cost_ms = (time.perf_counter() - start) * 1000.0
        timing_text = ""
        if timings:
            timing_text = (
                f" wait={timings.get('wait_ms', 0.0):.1f}ms"
                f" param={timings.get('param_ms', 0.0):.1f}ms"
                f" run={timings.get('run_ms', 0.0):.1f}ms"
            )
        print(
            f"[gripper] {cmd} {'ok' if ok else 'failed'} "
            f"cost={cost_ms:.1f}ms{timing_text}",
            flush=True,
        )
        publish_gripper_command(
            state_queue=state_queue,
            cmd=cmd,
            ok=ok,
            position=position,
            speed=speed,
            target_force=target_force,
            cost_ms=cost_ms,
            timings=timings,
        )


def optional_api(name: str):
    return getattr(_iimt_api, name, None)


def clear_robot_state(handle, label: str) -> None:
    """尽量清掉未完成轨迹/错误；旧 SDK 缺函数时跳过。"""
    try:
        API_Stop(handle)
        time.sleep(0.02)
        clear_fn = optional_api("API_ClearUnfinishedTrajectory")
        if clear_fn is not None:
            clear_fn(handle)
            time.sleep(0.02)
        reset_fn = optional_api("API_ResetRobotError")
        if reset_fn is not None:
            reset_fn(handle)
    except Exception as exc:
        print(f"[{label}] clear state failed: {exc}", flush=True)


def run_home(handle, pose: list[float] | tuple[float, ...] | None = None, msg: dict | None = None) -> bool:
    """回到固定左臂 home 位，默认使用 IIMT_LEFT_HOME_POSE。"""
    target_pose = list(pose if pose is not None else LEFT_HOME_POSE)
    print("[arm] home start", flush=True)
    print(f"[arm] home target={target_pose}", flush=True)
    clear_robot_state(handle, "home")
    time.sleep(0.10)
    movel = MovelParam()
    movel.p = (ctypes.c_double * 6)(*[float(value) for value in target_pose[:6]])
    movel.v = ctypes.c_float(float((msg or {}).get("movel_v", MOVEL_HOME_V)))
    movel.tool_v = ctypes.c_float(float((msg or {}).get("movel_tool_v", MOVEL_HOME_TOOL_V)))
    movel.tool_a = ctypes.c_float(float((msg or {}).get("movel_tool_a", MOVEL_HOME_TOOL_A)))
    movel.r = ctypes.c_float(0)
    timeout = int(float((msg or {}).get("timeout", MOVEL_HOME_TIMEOUT)))
    move_ok = bool(API_Movel(handle, movel, True, timeout))
    print(f"[arm] home {'ok' if move_ok else 'failed'}", flush=True)
    clear_robot_state(handle, "after_home")
    return move_ok


def run_servoc(handle, msg: dict) -> bool:
    """执行一帧 API_Servoc。"""
    pos = msg.get("pos") or [0.0, 0.0, 0.0]
    rot = msg.get("rot") or [0.0, 0.0, 0.0]
    if len(pos) != 3 or len(rot) != 3:
        return False
    target = [float(x) for x in pos] + [float(x) for x in rot]
    servoc = ServocParam()
    servoc.p = (ctypes.c_double * 6)(*target)
    servoc.a = ctypes.c_float(float(msg.get("servoc_a", SERVOC_A)))
    servoc.v = ctypes.c_float(float(msg.get("servoc_v", SERVOC_V)))
    servoc.t = ctypes.c_float(float(msg.get("servoc_t", SERVOC_T)))
    servoc.lookahead_time = ctypes.c_float(float(msg.get("servoc_lookahead", SERVOC_LOOKAHEAD)))
    return bool(API_Servoc(handle, servoc))


def run_servoj(handle, msg: dict) -> bool:
    """执行一帧 API_Servoj。"""
    joints = msg.get("joints")
    if not isinstance(joints, list) or len(joints) != 6:
        return False
    servoj = ServojParam()
    for idx in range(6):
        servoj.q[idx] = float(joints[idx])
    servoj.a = ctypes.c_float(float(msg.get("servoj_a", SERVOJ_A)))
    servoj.v = ctypes.c_float(float(msg.get("servoj_v", SERVOJ_V)))
    servoj.t = ctypes.c_float(float(msg.get("servoj_t", SERVOJ_T)))
    servoj.lookahead_time = ctypes.c_float(float(msg.get("servoj_lookahead", SERVOJ_LOOKAHEAD)))
    servoj.gain = ctypes.c_float(float(msg.get("servoj_gain", SERVOJ_GAIN)))
    return bool(API_Servoj(handle, servoj))


def command_rpc_loop(sock: zmq.Socket, handle, state_queue: Queue | None,
                     latest_ref: dict, latest_lock: threading.Lock,
                     arm_api_lock: threading.Lock,
                     stats: StatsHub,
                     stop_event: threading.Event) -> None:
    """处理低频阻塞命令，Home/Stop 不和高频 action 混在一起。"""
    poller = zmq.Poller()
    poller.register(sock, zmq.POLLIN)
    while not stop_event.is_set():
        events = dict(poller.poll(50))
        if sock not in events:
            continue
        try:
            msg = sock.recv_json(flags=zmq.NOBLOCK)
        except zmq.ZMQError:
            continue
        if not isinstance(msg, dict):
            continue
        msg = cast(Dict[str, Any], msg)
        cmd = str(msg.get("cmd", ""))
        seq = int(msg.get("seq", 0) or 0)
        start = time.perf_counter()
        ok = False
        error = ""
        try:
            if cmd in {"home_movel", "home"}:
                pose_value = msg.get("pose")
                if not isinstance(pose_value, list) or len(pose_value) != 6:
                    pose = list(LEFT_HOME_POSE)
                else:
                    pose = [float(value) for value in pose_value]
                with latest_lock:
                    latest_ref["msg"] = None
                with arm_api_lock:
                    ok = run_home(handle, pose, msg)
            elif cmd == "stop":
                with latest_lock:
                    latest_ref["msg"] = None
                with arm_api_lock:
                    clear_robot_state(handle, "stop")
                ok = True
            elif cmd == "clear_error":
                with arm_api_lock:
                    clear_robot_state(handle, "clear_error")
                ok = True
            elif cmd == "power_on":
                power_fn = optional_api("API_PowerOn")
                with arm_api_lock:
                    if power_fn is None:
                        error = "missing API_PowerOn"
                    else:
                        ok = bool(power_fn(handle))
            elif cmd == "release_brake":
                brake_fn = optional_api("API_ReleaseBrake")
                with arm_api_lock:
                    if brake_fn is None:
                        error = "missing API_ReleaseBrake"
                    else:
                        ok = bool(brake_fn(handle))
            elif cmd == "power_off":
                power_fn = optional_api("API_PowerOff")
                with latest_lock:
                    latest_ref["msg"] = None
                with arm_api_lock:
                    clear_robot_state(handle, "power_off")
                    if power_fn is None:
                        error = "missing API_PowerOff"
                    else:
                        ok = bool(power_fn(handle))
            else:
                error = f"unknown command: {cmd}"
        except Exception as exc:
            error = repr(exc)
        cost_ms = (time.perf_counter() - start) * 1000.0
        stats.mark_command(cmd, ok, cost_ms)
        ack = {
            "type": "command_ack",
            "cmd": cmd,
            "arm": "left",
            "seq": seq,
            "ok": bool(ok),
            "error": error,
            "cost_ms": round(float(cost_ms), 3),
            "remote_system_time": time.time(),
            "remote_mono_time": time.monotonic(),
        }
        try:
            sock.send_json(ack)
        except zmq.ZMQError as exc:
            print(f"[command] reply failed: {exc}", flush=True)
        publish_state(state_queue, {
            "topic": "left.command_ack",
            **ack,
        })
        print(
            f"[command] {cmd} {'ok' if ok else 'failed'} "
            f"cost={cost_ms:.1f}ms error={error}",
            flush=True,
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="IIMT left controller-side ZMQ v0 service")
    parser.add_argument("--arm-control", action=BooleanOptionalAction, default=True,
                        help="是否启动机械臂高频控制: 5555 SUB + API_Servoc/API_Servoj")
    parser.add_argument("--arm-command", action=BooleanOptionalAction, default=True,
                        help="是否启动机械臂阻塞命令: 5556 REP + API_Movel/API_Stop")
    parser.add_argument("--gripper-control", action=BooleanOptionalAction, default=True,
                        help="是否启动夹爪控制: 5557 PULL + Modbus open/close")
    parser.add_argument("--joint-state", action=BooleanOptionalAction, default=True,
                        help="是否启动关节状态发布: 100Hz -> 7020 PUB")
    parser.add_argument("--tcp-state", action=BooleanOptionalAction, default=True,
                        help="是否启动末端 TCP 位姿状态发布: 30Hz -> 7020 PUB")
    parser.add_argument("--gripper-state", action=BooleanOptionalAction, default=True,
                        help="是否启动夹爪状态发布: 15Hz -> 7020 PUB")
    args = parser.parse_args()

    if not any([args.arm_control, args.arm_command, args.gripper_control, args.joint_state, args.tcp_state, args.gripper_state]):
        raise SystemExit("no module enabled")

    state_pub_enabled = args.arm_command or args.gripper_control or args.joint_state or args.tcp_state or args.gripper_state

    print("=" * 58, flush=True)
    print("  IIMT left controller-side ZMQ v0 service", flush=True)
    print("=" * 58, flush=True)
    print(
        "[modules] "
        f"arm_control={'ON' if args.arm_control else 'OFF'} "
        f"arm_command={'ON' if args.arm_command else 'OFF'} "
        f"gripper_control={'ON' if args.gripper_control else 'OFF'} "
        f"joint_state={'ON' if args.joint_state else 'OFF'} "
        f"tcp_state={'ON' if args.tcp_state else 'OFF'} "
        f"gripper_state={'ON' if args.gripper_state else 'OFF'}",
        flush=True,
    )
    print(
        "[config] "
        f"ARM_CONTROL_HZ={ARM_CONTROL_HZ:.3f} "
        f"ARM_TARGET_STALE_SEC={ARM_TARGET_STALE_SEC:.4f} "
        f"ARM_RECV_STATS_INTERVAL={ARM_RECV_STATS_INTERVAL:.3f} "
        f"SERVOC_A={SERVOC_A:.3f} "
        f"SERVOC_V={SERVOC_V:.3f} "
        f"SERVOC_T={SERVOC_T:.6f} "
        f"SERVOC_LOOKAHEAD={SERVOC_LOOKAHEAD:.4f} "
        f"SERVOJ_A={SERVOJ_A:.3f} "
        f"SERVOJ_V={SERVOJ_V:.3f} "
        f"SERVOJ_T={SERVOJ_T:.6f} "
        f"SERVOJ_LOOKAHEAD={SERVOJ_LOOKAHEAD:.4f} "
        f"SERVOJ_GAIN={SERVOJ_GAIN:.3f}",
        flush=True,
    )

    arm_handle = None
    grip_handle = None
    joint_state_handle = None
    tcp_state_handle = None
    grip_state_handle = None

    if args.arm_control or args.arm_command:
        arm_ok, arm_handle = _api_login()
        print(f"[arm] login={arm_ok} handle={getattr(arm_handle, 'value', arm_handle)}", flush=True)
        if not arm_ok:
            raise SystemExit("[arm] API_Login failed")
    else:
        print("[arm] control/command disabled; no arm API_Login, no 5555/5556 bind", flush=True)

    if args.gripper_control:
        grip_ok, grip_handle = _api_login()
        print(f"[gripper] login={grip_ok} handle={getattr(grip_handle, 'value', grip_handle)}", flush=True)
        if not grip_ok:
            raise SystemExit("[gripper] API_Login failed")
        grip_enable_ok = enable_gripper(grip_handle, True)
        print(f"[gripper] startup enable {'ok' if grip_enable_ok else 'failed'}", flush=True)
    else:
        print("[gripper] control disabled; no gripper control API_Login, no 5557 bind", flush=True)

    if args.joint_state:
        joint_state_ok, joint_state_handle = _api_login()
        print(
            f"[state-joints] login={joint_state_ok} "
            f"handle={getattr(joint_state_handle, 'value', joint_state_handle)}",
            flush=True,
        )
        if not joint_state_ok:
            raise SystemExit("[state-joints] API_Login failed")
    else:
        print("[state-joints] disabled; no joint state API_Login", flush=True)

    if args.tcp_state:
        tcp_state_ok, tcp_state_handle = _api_login()
        print(
            f"[state-tcp] login={tcp_state_ok} "
            f"handle={getattr(tcp_state_handle, 'value', tcp_state_handle)}",
            flush=True,
        )
        if not tcp_state_ok:
            raise SystemExit("[state-tcp] API_Login failed")
    else:
        print("[state-tcp] disabled; no tcp state API_Login", flush=True)

    if args.gripper_state:
        grip_state_ok, grip_state_handle = _api_login()
        print(
            f"[state-gripper] login={grip_state_ok} "
            f"handle={getattr(grip_state_handle, 'value', grip_state_handle)}",
            flush=True,
        )
        if not grip_state_ok:
            raise SystemExit("[state-gripper] API_Login failed")
    else:
        print("[state-gripper] disabled; no gripper state API_Login", flush=True)

    ctx = zmq.Context()
    arm_sock = bind_arm_socket(ctx) if args.arm_control else None
    command_sock = bind_command_socket(ctx) if args.arm_command else None
    gripper_sock = bind_gripper_socket(ctx) if args.gripper_control else None
    state_sock = bind_state_socket(ctx) if state_pub_enabled else None
    state_queue: Queue | None = Queue(maxsize=STATE_QUEUE_MAXSIZE) if state_pub_enabled else None
    arm_queue: Queue = Queue(maxsize=1)
    gripper_queue: Queue = Queue(maxsize=1)
    stop_event = threading.Event()
    stats = StatsHub()
    latest_ref = {"msg": None}
    latest_lock = threading.Lock()
    arm_api_lock = threading.Lock()

    threads = []
    threads.append(threading.Thread(target=stats_report_loop, args=(stats, state_queue, stop_event), daemon=True))
    if state_pub_enabled:
        threads.append(threading.Thread(target=state_publisher_loop, args=(state_sock, state_queue, stats, stop_event), daemon=True))
    if args.arm_control:
        threads.append(threading.Thread(target=arm_recv_loop, args=(arm_sock, arm_queue, stats, stop_event), daemon=True))
    if args.arm_command:
        threads.append(threading.Thread(
            target=command_rpc_loop,
            args=(command_sock, arm_handle, state_queue, latest_ref, latest_lock, arm_api_lock, stats, stop_event),
            daemon=True,
        ))
    if args.gripper_control:
        threads.append(threading.Thread(target=gripper_recv_loop, args=(gripper_sock, gripper_queue, stop_event), daemon=True))
        threads.append(threading.Thread(target=gripper_worker, args=(grip_handle, gripper_queue, state_queue, stop_event), daemon=True))
    if args.joint_state:
        threads.append(threading.Thread(target=joint_state_publish_loop, args=(state_queue, joint_state_handle, stats, stop_event), daemon=True))
    if args.tcp_state:
        threads.append(threading.Thread(target=tcp_state_publish_loop, args=(state_queue, tcp_state_handle, stats, stop_event), daemon=True))
    if args.gripper_state:
        threads.append(threading.Thread(target=gripper_state_publish_loop, args=(state_queue, grip_state_handle, stats, stop_event), daemon=True))

    for thread in threads:
        thread.start()

    if args.arm_control:
        print(f"[arm] SUB bind {ARM_BIND}", flush=True)
    if args.arm_command:
        print(f"[command] REP bind {COMMAND_BIND}", flush=True)
    if args.gripper_control:
        print(f"[gripper] PULL bind {GRIPPER_BIND}", flush=True)
    if state_pub_enabled:
        print(f"[state] PUB bind {STATE_BIND}", flush=True)
    print("waiting commands...", flush=True)

    total = 0
    ok_count = 0
    fail_count = 0
    last_cmd = "-"
    last_recv_mono = None
    last_exec_mono = None
    last_loop_mono = None
    control_interval = 1.0 / ARM_CONTROL_HZ
    next_control_mono = time.monotonic()
    last_exec_recv_seq = None
    try:
        while True:
            loop_mono = time.monotonic()
            if last_loop_mono is not None:
                loop_gap = max(0.0, loop_mono - last_loop_mono)
                stats.mark_arm_loop_gap(loop_gap)
            last_loop_mono = loop_mono

            if not args.arm_control:
                stop_event.wait(1.0)
                continue

            while True:
                try:
                    msg = arm_queue.get_nowait()
                except Empty:
                    break
                cmd = str(msg.get("cmd", ""))
                if cmd in {"servo_pose", "servoj_joints"} and msg.get("is_active", True):
                    with latest_lock:
                        latest_ref["msg"] = msg
                elif cmd == "hold":
                    with latest_lock:
                        latest_ref["msg"] = None

            now = time.monotonic()
            sleep_sec = next_control_mono - now
            if sleep_sec > 0:
                stop_event.wait(min(sleep_sec, 0.002))
                continue
            if now - next_control_mono > control_interval:
                next_control_mono = now
            else:
                next_control_mono += control_interval

            with latest_lock:
                latest_msg = latest_ref["msg"]
            if latest_msg is None:
                stats.mark_no_target()
                continue
            msg = latest_msg
            recv_mono = float(msg.get("_remote_recv_mono", now))
            if now - recv_mono > ARM_TARGET_STALE_SEC:
                with latest_lock:
                    latest_ref["msg"] = None
                stats.mark_stale_drop()
                continue

            cmd = str(msg.get("cmd", "servo_pose"))
            exec_mono = time.monotonic()
            recv_seq = msg.get("_remote_recv_seq")
            recv_dt_for_stat = None
            if recv_seq != last_exec_recv_seq:
                if last_recv_mono is not None:
                    recv_dt = max(0.0, recv_mono - last_recv_mono)
                    recv_dt_for_stat = recv_dt
                last_recv_mono = recv_mono
                last_exec_recv_seq = recv_seq
            exec_dt_for_stat = None
            if last_exec_mono is not None:
                exec_dt = max(0.0, exec_mono - last_exec_mono)
                exec_dt_for_stat = exec_dt
            target_age = max(0.0, exec_mono - recv_mono)

            call_start = time.perf_counter()
            with arm_api_lock:
                if cmd == "servoj_joints":
                    ok = run_servoj(arm_handle, msg)
                else:
                    ok = run_servoc(arm_handle, msg)
            call_ms = (time.perf_counter() - call_start) * 1000.0
            last_exec_mono = exec_mono
            total += 1
            ok_count += int(ok)
            fail_count += int(not ok)
            last_cmd = cmd
            stats.mark_arm_exec(ok, cmd, recv_dt_for_stat, exec_dt_for_stat, target_age, call_ms)
    except KeyboardInterrupt:
        print("\nstopping...", flush=True)
    finally:
        stop_event.set()
        for thread in threads:
            thread.join(timeout=1.0)
        for sock in (arm_sock, command_sock, gripper_sock, state_sock):
            if sock is not None:
                sock.close(0)
        ctx.term()
        print(
            f"[arm] stopped total={total} ok={ok_count} fail={fail_count} last={last_cmd}",
            flush=True,
        )



if __name__ == "__main__":
    main()
