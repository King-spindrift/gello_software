#!/usr/bin/env python3
"""Desktop control panel for the GELLO/IIMT master-slave system."""

from __future__ import annotations

import contextlib
import io
import ipaddress
import json
import queue
import sys
import threading
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import font as tkfont
from tkinter import scrolledtext, ttk


PROJECT_ROOT = Path(__file__).resolve().parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import master_slave_control as control


BACKGROUND = "#0B1220"
SURFACE = "#111B2E"
SURFACE_ALT = "#17243A"
BORDER = "#263650"
TEXT = "#F4F7FB"
MUTED = "#93A4BD"
GREEN = "#2ED39A"
BLUE = "#4C8DFF"
AMBER = "#F3B64C"
RED = "#FF6B75"


class ControlPanel:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.events: queue.Queue[tuple[str, str, str]] = queue.Queue()
        self.busy = False
        self.buttons: list[tk.Button] = []
        self.config_path = PROJECT_ROOT / "configs/iimt_master_slave.json"
        self.config = self._load_config()

        self._configure_window()
        self._configure_fonts()
        self._initialize_setting_vars()
        self._build_layout()
        self._refresh_status()
        self._poll_events()
        self._schedule_status_refresh()

    def _configure_window(self) -> None:
        self.root.title("GELLO · IIMT  主从机械臂控制台")
        self.root.geometry("1180x940")
        self.root.minsize(980, 760)
        self.root.configure(bg=BACKGROUND)
        self.root.protocol("WM_DELETE_WINDOW", self._close_panel)

    def _configure_fonts(self) -> None:
        tkfont.nametofont("TkDefaultFont").configure(
            family="Noto Sans CJK SC", size=11
        )
        tkfont.nametofont("TkTextFont").configure(
            family="Noto Sans Mono CJK SC", size=10
        )
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure(
            "Vertical.TScrollbar",
            background=SURFACE_ALT,
            troughcolor=SURFACE,
            bordercolor=SURFACE,
            arrowcolor=MUTED,
        )

    def _load_config(self) -> dict:
        return json.loads(self.config_path.read_text(encoding="utf-8"))

    def _initialize_setting_vars(self) -> None:
        raw_followers = self.config.get("followers")
        if raw_followers is None:
            raw_followers = [
                {
                    "enabled": True,
                    "robot_ip": self.config.get("robot_ip", ""),
                    "robot_port": self.config.get("robot_port", 6001),
                }
            ]
        followers = list(raw_followers[:2])
        while len(followers) < 2:
            followers.append(
                {
                    "enabled": False,
                    "robot_ip": "",
                    "robot_port": 6001 + len(followers),
                }
            )
        self.follower_enabled_vars = [
            tk.BooleanVar(value=bool(item.get("enabled", index == 0)))
            for index, item in enumerate(followers)
        ]
        self.follower_ip_vars = [
            tk.StringVar(value=str(item.get("robot_ip", "")))
            for item in followers
        ]
        legacy_gain = float(self.config.get("joint_gain", 1.0))
        self.joint_gain_vars = []
        for item in followers:
            gains = item.get("joint_gains", [legacy_gain] * 6)
            if not isinstance(gains, list) or len(gains) != 6:
                gains = [legacy_gain] * 6
            self.joint_gain_vars.append(
                [tk.StringVar(value=str(value)) for value in gains]
            )

    def _build_layout(self) -> None:
        arm_motion_enabled = bool(self.config.get("arm_motion_enabled", True))

        outer = tk.Frame(self.root, bg=BACKGROUND, padx=34, pady=28)
        outer.pack(fill="both", expand=True)

        header = tk.Frame(outer, bg=BACKGROUND)
        header.pack(fill="x")
        title_group = tk.Frame(header, bg=BACKGROUND)
        title_group.pack(side="left")
        tk.Label(
            title_group,
            text="MASTER · SLAVE",
            bg=BACKGROUND,
            fg=BLUE,
            font=("Noto Sans CJK SC", 10, "bold"),
        ).pack(anchor="w")
        tk.Label(
            title_group,
            text="主从机械臂控制台",
            bg=BACKGROUND,
            fg=TEXT,
            font=("Noto Sans CJK SC", 25, "bold"),
        ).pack(anchor="w", pady=(2, 0))
        tk.Label(
            title_group,
            text=(
                "GELLO 主臂  →  IIMT 从臂  ·  250 Hz"
                if arm_motion_enabled
                else "夹爪调试模式  ·  从臂六轴运动已禁用"
            ),
            bg=BACKGROUND,
            fg=MUTED if arm_motion_enabled else AMBER,
            font=("Noto Sans CJK SC", 11),
        ).pack(anchor="w", pady=(4, 0))

        self.system_badge = tk.Label(
            header,
            text="●  正在读取状态",
            bg=SURFACE,
            fg=MUTED,
            padx=16,
            pady=9,
            font=("Noto Sans CJK SC", 10, "bold"),
        )
        self.system_badge.pack(side="right", anchor="n", pady=5)

        body = tk.Frame(outer, bg=BACKGROUND)
        body.pack(fill="both", expand=True, pady=(26, 0))
        body.grid_columnconfigure(0, weight=0, minsize=430)
        body.grid_columnconfigure(1, weight=1)
        body.grid_rowconfigure(0, weight=1)

        controls = tk.Frame(body, bg=SURFACE, padx=22, pady=22)
        controls.grid(row=0, column=0, sticky="nsew", padx=(0, 18))
        controls.grid_columnconfigure(0, weight=1)

        tk.Label(
            controls,
            text="系统状态",
            bg=SURFACE,
            fg=TEXT,
            font=("Noto Sans CJK SC", 13, "bold"),
        ).grid(row=0, column=0, sticky="w")

        status_row = tk.Frame(controls, bg=SURFACE)
        status_row.grid(row=1, column=0, sticky="ew", pady=(14, 22))
        status_row.grid_columnconfigure(0, weight=1)
        status_row.grid_columnconfigure(1, weight=1)
        status_row.grid_columnconfigure(2, weight=1)
        self.server_1_card = self._status_card(status_row, "从臂1 SDK", 0)
        self.server_2_card = self._status_card(status_row, "从臂2 SDK", 1)
        self.controller_card = self._status_card(status_row, "遥操控制端", 2)

        tk.Frame(controls, bg=BORDER, height=1).grid(
            row=2, column=0, sticky="ew", pady=(0, 20)
        )
        tk.Label(
            controls,
            text="运行配置",
            bg=SURFACE,
            fg=TEXT,
            font=("Noto Sans CJK SC", 13, "bold"),
        ).grid(row=3, column=0, sticky="w", pady=(0, 12))

        settings = tk.Frame(controls, bg=SURFACE)
        settings.grid(row=4, column=0, sticky="ew")
        settings.grid_columnconfigure(1, weight=1)
        for index in range(2):
            checkbox = tk.Checkbutton(
                settings,
                text=f"启用从臂{index + 1}",
                variable=self.follower_enabled_vars[index],
                bg=SURFACE,
                activebackground=SURFACE,
                fg=TEXT,
                activeforeground=TEXT,
                selectcolor=SURFACE_ALT,
                font=("Noto Sans CJK SC", 9),
                cursor="hand2",
            )
            checkbox.grid(row=index, column=0, sticky="w", pady=4)
            entry = tk.Entry(
                settings,
                textvariable=self.follower_ip_vars[index],
                bg="#09101D",
                fg=TEXT,
                insertbackground=TEXT,
                relief="flat",
                highlightthickness=1,
                highlightbackground=BORDER,
                highlightcolor=BLUE,
                font=("Noto Sans Mono CJK SC", 10),
            )
            entry.grid(row=index, column=1, sticky="ew", padx=(10, 0), pady=4, ipady=6)

        for arm_index in range(2):
            tk.Label(
                settings,
                text=f"从臂{arm_index + 1} 关节增益",
                bg=SURFACE,
                fg=MUTED,
                font=("Noto Sans CJK SC", 9, "bold"),
            ).grid(
                row=2 + arm_index * 2,
                column=0,
                columnspan=2,
                sticky="w",
                pady=(9, 3),
            )
            gain_row = tk.Frame(settings, bg=SURFACE)
            gain_row.grid(
                row=3 + arm_index * 2,
                column=0,
                columnspan=2,
                sticky="ew",
            )
            for joint_index in range(6):
                gain_row.grid_columnconfigure(joint_index, weight=1)
                cell = tk.Frame(gain_row, bg=SURFACE)
                cell.grid(
                    row=0,
                    column=joint_index,
                    sticky="ew",
                    padx=(0 if joint_index == 0 else 3, 0),
                )
                tk.Label(
                    cell,
                    text=f"J{joint_index + 1}",
                    bg=SURFACE,
                    fg=MUTED,
                    font=("Noto Sans CJK SC", 8),
                ).pack(anchor="w")
                tk.Entry(
                    cell,
                    textvariable=self.joint_gain_vars[arm_index][joint_index],
                    width=5,
                    bg="#09101D",
                    fg=TEXT,
                    insertbackground=TEXT,
                    relief="flat",
                    highlightthickness=1,
                    highlightbackground=BORDER,
                    highlightcolor=BLUE,
                    justify="center",
                    font=("Noto Sans Mono CJK SC", 9),
                ).pack(fill="x", ipady=5)

        save_button = tk.Button(
            controls,
            text="保存 IP 与增益配置",
            command=self._save_settings,
            bg=SURFACE_ALT,
            activebackground="#20314D",
            fg=GREEN,
            activeforeground=GREEN,
            relief="flat",
            borderwidth=0,
            highlightthickness=1,
            highlightbackground=BORDER,
            padx=12,
            pady=8,
            cursor="hand2",
            font=("Noto Sans CJK SC", 9, "bold"),
        )
        save_button.grid(row=5, column=0, sticky="ew", pady=(10, 16))
        self.buttons.append(save_button)

        tk.Frame(controls, bg=BORDER, height=1).grid(
            row=6, column=0, sticky="ew", pady=(0, 18)
        )
        tk.Label(
            controls,
            text="快捷操作",
            bg=SURFACE,
            fg=TEXT,
            font=("Noto Sans CJK SC", 13, "bold"),
        ).grid(row=7, column=0, sticky="w", pady=(0, 13))

        self._action_button(
            controls,
            8,
            "1",
            "启动系统",
            "连接并进入主从控制",
            BLUE,
            self._start_system,
        )
        self._action_button(
            controls,
            9,
            "2",
            "停止控制",
            "释放主臂，保留 SDK 连接",
            RED,
            lambda: self._run_action("停止控制", control.stop),
        )
        self._action_button(
            controls,
            10,
            "3",
            "刷新状态",
            "读取当前进程运行状态",
            GREEN,
            self._show_status,
        )
        self._action_button(
            controls,
            11,
            "4",
            "查看日志",
            "显示服务端与控制端日志",
            AMBER,
            self._show_logs,
        )
        self._action_button(
            controls,
            12,
            "0",
            "退出界面",
            "不会停止后台控制服务",
            MUTED,
            self._close_panel,
        )

        tk.Label(
            controls,
            text="紧急情况请使用硬件急停",
            bg=SURFACE,
            fg=AMBER,
            font=("Noto Sans CJK SC", 9),
        ).grid(row=13, column=0, sticky="w", pady=(18, 0))

        console = tk.Frame(body, bg=SURFACE, padx=22, pady=22)
        console.grid(row=0, column=1, sticky="nsew")
        console.grid_rowconfigure(1, weight=1)
        console.grid_columnconfigure(0, weight=1)

        console_header = tk.Frame(console, bg=SURFACE)
        console_header.grid(row=0, column=0, sticky="ew", pady=(0, 13))
        tk.Label(
            console_header,
            text="运行信息",
            bg=SURFACE,
            fg=TEXT,
            font=("Noto Sans CJK SC", 13, "bold"),
        ).pack(side="left")
        self.activity_label = tk.Label(
            console_header,
            text="就绪",
            bg=SURFACE,
            fg=MUTED,
            font=("Noto Sans CJK SC", 10),
        )
        self.activity_label.pack(side="right")

        self.console = scrolledtext.ScrolledText(
            console,
            wrap="word",
            state="disabled",
            bg="#09101D",
            fg="#C9D5E7",
            insertbackground=TEXT,
            selectbackground="#254B7A",
            relief="flat",
            borderwidth=0,
            padx=16,
            pady=14,
            font=("Noto Sans Mono CJK SC", 10),
        )
        self.console.grid(row=1, column=0, sticky="nsew")
        self.console.tag_configure("time", foreground="#62738E")
        self.console.tag_configure("success", foreground=GREEN)
        self.console.tag_configure("error", foreground=RED)
        self.console.tag_configure("heading", foreground=BLUE)
        self._append_console("控制面板已就绪。请选择左侧操作。", "success")

    def _status_card(
        self, parent: tk.Widget, title: str, column: int
    ) -> dict[str, tk.Label]:
        frame = tk.Frame(parent, bg=SURFACE_ALT, padx=13, pady=12)
        frame.grid(
            row=0,
            column=column,
            sticky="ew",
            padx=(0, 4) if column == 0 else ((4, 0) if column == 2 else 4),
        )
        tk.Label(
            frame,
            text=title,
            bg=SURFACE_ALT,
            fg=MUTED,
            font=("Noto Sans CJK SC", 9),
        ).pack(anchor="w")
        state = tk.Label(
            frame,
            text="●  检查中",
            bg=SURFACE_ALT,
            fg=MUTED,
            font=("Noto Sans CJK SC", 10, "bold"),
        )
        state.pack(anchor="w", pady=(5, 0))
        pid = tk.Label(
            frame,
            text="PID —",
            bg=SURFACE_ALT,
            fg="#687A96",
            font=("Noto Sans CJK SC", 8),
        )
        pid.pack(anchor="w", pady=(2, 0))
        return {"state": state, "pid": pid}

    def _action_button(
        self,
        parent: tk.Widget,
        row: int,
        number: str,
        title: str,
        subtitle: str,
        accent: str,
        command,
    ) -> None:
        button = tk.Button(
            parent,
            text=f"{number}    {title}\n      {subtitle}",
            command=command,
            anchor="w",
            justify="left",
            bg=SURFACE_ALT,
            activebackground="#20314D",
            fg=TEXT,
            activeforeground=TEXT,
            disabledforeground="#64748B",
            relief="flat",
            borderwidth=0,
            highlightthickness=1,
            highlightbackground=BORDER,
            highlightcolor=accent,
            padx=16,
            pady=10,
            cursor="hand2",
            font=("Noto Sans CJK SC", 10, "bold"),
        )
        button.grid(row=row, column=0, sticky="ew", pady=4)
        self.buttons.append(button)

    def _save_settings(self, announce: bool = True) -> bool:
        try:
            enabled = [variable.get() for variable in self.follower_enabled_vars]
            ips = [variable.get().strip() for variable in self.follower_ip_vars]
            if not any(enabled):
                raise ValueError("至少需要启用一台从机械臂")
            for index, (is_enabled, robot_ip) in enumerate(
                zip(enabled, ips), start=1
            ):
                if not is_enabled:
                    continue
                if not robot_ip:
                    raise ValueError(f"从臂{index} 已启用，请输入 IP 地址")
                try:
                    ipaddress.ip_address(robot_ip)
                except ValueError as error:
                    raise ValueError(
                        f"从臂{index} IP 地址无效：{robot_ip}"
                    ) from error
            active_ips = [ip for flag, ip in zip(enabled, ips) if flag]
            if len(active_ips) != len(set(active_ips)):
                raise ValueError("两台已启用的从臂不能使用相同 IP")
            joint_gains: list[list[float]] = []
            for arm_index, variables in enumerate(
                self.joint_gain_vars, start=1
            ):
                arm_gains = []
                for joint_index, variable in enumerate(variables, start=1):
                    try:
                        gain = float(variable.get().strip())
                    except ValueError as error:
                        raise ValueError(
                            f"从臂{arm_index} J{joint_index} 增益不是有效数字"
                        ) from error
                    if not -10.0 <= gain <= 10.0:
                        raise ValueError(
                            f"从臂{arm_index} J{joint_index} 增益必须在 "
                            "-10 到 10 之间"
                        )
                    arm_gains.append(gain)
                joint_gains.append(arm_gains)

            followers = []
            for index in range(2):
                followers.append(
                    {
                        "name": f"从臂{index + 1}",
                        "enabled": enabled[index],
                        "robot_ip": ips[index],
                        "robot_port": 6001 + index,
                        "joint_gains": joint_gains[index],
                    }
                )
            self.config["followers"] = followers
            self.config.pop("joint_gain", None)
            temporary = self.config_path.with_suffix(".json.tmp")
            temporary.write_text(
                json.dumps(self.config, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            temporary.replace(self.config_path)
            for arm_index, gains in enumerate(joint_gains):
                for joint_index, gain in enumerate(gains):
                    self.joint_gain_vars[arm_index][joint_index].set(f"{gain:g}")
            if announce:
                state = control.get_status()
                suffix = (
                    "。当前系统正在运行，停止后重新启动才会生效。"
                    if state["controller"][0]
                    else "。下次启动时生效。"
                )
                self._append_console("IP 与关节增益已保存" + suffix, "success")
            return True
        except (OSError, ValueError) as error:
            self._append_console(f"保存配置失败：{error}", "error")
            return False

    def _start_system(self) -> None:
        if self._save_settings(announce=False):
            self._run_action("启动系统", control.start)

    def _append_console(self, message: str, tag: str | None = None) -> None:
        stamp = datetime.now().strftime("%H:%M:%S")
        self.console.configure(state="normal")
        self.console.insert("end", f"[{stamp}] ", "time")
        self.console.insert("end", message.rstrip() + "\n", tag or "")
        self.console.see("end")
        self.console.configure(state="disabled")

    def _replace_console(self, content: str) -> None:
        self.console.configure(state="normal")
        self.console.delete("1.0", "end")
        self.console.insert("end", content)
        self.console.see("end")
        self.console.configure(state="disabled")

    def _set_busy(self, busy: bool, label: str = "") -> None:
        self.busy = busy
        for button in self.buttons:
            button.configure(state="disabled" if busy else "normal")
        self.activity_label.configure(
            text=label if busy else "就绪",
            fg=AMBER if busy else MUTED,
        )
        self.root.configure(cursor="watch" if busy else "")

    def _run_action(self, name: str, action) -> None:
        if self.busy:
            return
        self._set_busy(True, f"正在{name}…")
        self._append_console(f"正在{name}…", "heading")

        def worker() -> None:
            output = io.StringIO()
            try:
                with contextlib.redirect_stdout(output):
                    action()
                self.events.put(("success", name, output.getvalue().strip()))
            except Exception as error:
                details = output.getvalue().strip()
                message = f"{type(error).__name__}: {error}"
                if details:
                    message = details + "\n" + message
                self.events.put(("error", name, message))

        threading.Thread(target=worker, daemon=True, name="panel-action").start()

    def _poll_events(self) -> None:
        try:
            while True:
                kind, name, message = self.events.get_nowait()
                self._set_busy(False)
                if message:
                    self._append_console(message, kind)
                result = "完成" if kind == "success" else "失败"
                self._append_console(f"{name}{result}。", kind)
                self._refresh_status()
        except queue.Empty:
            pass
        self.root.after(120, self._poll_events)

    def _refresh_status(self) -> None:
        state = control.get_status()
        server_1_alive, server_1_pid = state["server_1"]
        server_2_alive, server_2_pid = state["server_2"]
        controller_alive, controller_pid = state["controller"]
        enabled_1 = self.follower_enabled_vars[0].get()
        enabled_2 = self.follower_enabled_vars[1].get()
        self._update_status_card(
            self.server_1_card, server_1_alive, server_1_pid, enabled_1
        )
        self._update_status_card(
            self.server_2_card, server_2_alive, server_2_pid, enabled_2
        )
        self._update_status_card(
            self.controller_card, controller_alive, controller_pid
        )

        required_servers_ready = (
            (not enabled_1 or server_1_alive)
            and (not enabled_2 or server_2_alive)
        )
        any_server_alive = server_1_alive or server_2_alive
        if required_servers_ready and controller_alive:
            text, color = "●  系统运行中", GREEN
        elif any_server_alive and not controller_alive:
            text, color = "●  SDK 待机", AMBER
        elif controller_alive or (any_server_alive and not required_servers_ready):
            text, color = "●  状态异常", RED
        else:
            text, color = "●  系统已停止", MUTED
        self.system_badge.configure(text=text, fg=color)

    def _schedule_status_refresh(self) -> None:
        if not self.busy:
            self._refresh_status()
        self.root.after(1000, self._schedule_status_refresh)

    @staticmethod
    def _update_status_card(
        card: dict[str, tk.Label],
        alive: bool,
        pid: int | None,
        enabled: bool = True,
    ) -> None:
        if not enabled:
            card["state"].configure(text="●  未启用", fg=MUTED)
            card["pid"].configure(text="PID —")
            return
        card["state"].configure(
            text="●  运行中" if alive else "●  已停止",
            fg=GREEN if alive else MUTED,
        )
        card["pid"].configure(
            text=f"PID {pid}" if alive and pid is not None else "PID —"
        )

    def _show_status(self) -> None:
        self._refresh_status()
        state = control.get_status()
        controller_alive, controller_pid = state["controller"]
        controller_pid_text = (
            controller_pid if controller_alive and controller_pid is not None else "—"
        )
        lines = []
        for index in (1, 2):
            enabled = self.follower_enabled_vars[index - 1].get()
            alive, pid = state[f"server_{index}"]
            if not enabled:
                lines.append(f"从臂{index} SDK：未启用")
                continue
            pid_text = pid if alive and pid is not None else "—"
            lines.append(
                f"从臂{index} SDK：{'运行中' if alive else '已停止'}"
                f"（PID {pid_text}）"
            )
        lines.append(
            f"遥操控制端：{'运行中' if controller_alive else '已停止'}"
            f"（PID {controller_pid_text}）"
        )
        message = "\n".join(lines)
        tag = "success" if controller_alive else "heading"
        self._append_console(message, tag)

    def _show_logs(self) -> None:
        try:
            self._replace_console(control.get_logs(lines=80))
            self.activity_label.configure(text="日志已刷新", fg=GREEN)
            self.root.after(
                1500,
                lambda: self.activity_label.configure(text="就绪", fg=MUTED),
            )
        except Exception as error:
            self._append_console(f"读取日志失败：{error}", "error")

    def _close_panel(self) -> None:
        if self.busy:
            self._append_console("当前操作尚未完成，请稍候再退出。", "error")
            return
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    ControlPanel(root)
    root.mainloop()


if __name__ == "__main__":
    main()
