"""IIMT robot adapter for the GELLO Robot/ZMQ interface."""

from __future__ import annotations

import ctypes
import math
import threading
import time
from pathlib import Path
from typing import Dict, Optional, Sequence

import numpy as np


class JointRad(ctypes.Structure):
    _fields_ = [("robotRad", ctypes.c_double * 6)]


class Text32(ctypes.Structure):
    _fields_ = [("description", ctypes.c_char * 32)]


class ServojParam(ctypes.Structure):
    _fields_ = [
        ("q", ctypes.c_double * 6),
        ("a", ctypes.c_float),
        ("v", ctypes.c_float),
        ("t", ctypes.c_float),
        ("lookahead_time", ctypes.c_float),
        ("gain", ctypes.c_float),
    ]


class MovejParam(ctypes.Structure):
    _fields_ = [
        ("q", ctypes.c_double * 6),
        ("v", ctypes.c_float),
        ("a", ctypes.c_float),
        ("j", ctypes.c_float),
        ("r", ctypes.c_float),
        ("t", ctypes.c_float),
    ]


class ModbusRTUFrame(ctypes.Structure):
    _fields_ = [
        ("data", ctypes.c_ubyte * 256),
        ("data_len", ctypes.c_ushort),
    ]


class ModbusRTURequest(ctypes.Structure):
    _fields_ = [
        ("slave_addr", ctypes.c_ubyte),
        ("func_code", ctypes.c_ubyte),
        ("start_addr", ctypes.c_ushort),
        ("data_number", ctypes.c_ushort),
        ("data", ctypes.c_ubyte * 250),
        ("data_len", ctypes.c_ushort),
    ]


def _clamp_u8(value: int) -> int:
    return max(0, min(255, int(value)))


def _pack_gripper_position(position: int) -> int:
    """Pack a 0=open, 255=closed target for register 1001."""
    return (_clamp_u8(position) << 8) & 0xFFFF


def _pack_gripper_speed_force(speed: int, force: int) -> int:
    """Pack speed and force for register 1002."""
    return _clamp_u8(speed) | (_clamp_u8(force) << 8)


def _binary_gripper_target(trigger_position: float, current_state: float) -> float:
    """Apply a Schmitt trigger so encoder noise cannot cause double actions."""
    trigger = float(trigger_position)
    closed = float(current_state) >= 0.5
    if closed:
        return 0.0 if trigger <= 0.45 else 1.0
    return 1.0 if trigger >= 0.55 else 0.0


def _text(value: Text32) -> str:
    return bytes(value.description).split(b"\0", 1)[0].decode("utf-8", "replace")


class IIMTRobot:
    """Expose an IIMT5 as a seven-channel GELLO robot.

    Channels 0-5 are sent to API_Servoj. Channel 6 controls the end-tool
    gripper through API_ModbusRtuRWEndTool on a dedicated SDK connection.
    """

    def __init__(
        self,
        robot_ip: str = "192.168.1.123",
        robot_port: int = 29999,
        sdk_library: Optional[str] = None,
        control_hz: float = 50.0,
        max_relative_travel: float = 0.50,
        max_step: float = 0.02,
        max_tracking_error: float = 0.30,
        state_check_interval: float = 0.50,
        allow_manual_servo: bool = False,
        gripper_enabled: bool = True,
        gripper_speed: int = 255,
        gripper_force: int = 20,
        gripper_command_hz: float = 50.0,
        gripper_deadband: int = 1,
        gripper_timeout: int = 2,
    ):
        if control_hz <= 0:
            raise ValueError("control_hz must be positive")
        if max_relative_travel < 0:
            raise ValueError("max_relative_travel must be non-negative")
        if min(max_step, max_tracking_error) <= 0:
            raise ValueError("step and tracking-error limits must be positive")
        if gripper_command_hz <= 0 or gripper_timeout <= 0:
            raise ValueError("gripper command frequency and timeout must be positive")
        if gripper_deadband < 0:
            raise ValueError("gripper deadband must be non-negative")

        if sdk_library is None:
            project_root = Path(__file__).resolve().parents[2]
            sdk_library = str(
                project_root
                / "third_party/IIMT-SDK/python/linux/x64/lib/libiimtcl.so"
            )
        sdk_path = Path(sdk_library).expanduser().resolve()
        if not sdk_path.is_file():
            raise FileNotFoundError(f"IIMT SDK library not found: {sdk_path}")

        self._sdk = ctypes.CDLL(str(sdk_path))
        self._configure_api()
        self._state_handle = self._login(robot_ip, robot_port, "state")
        try:
            self._servo_handle = self._login(robot_ip, robot_port, "servo")
            self._move_handle = self._login(robot_ip, robot_port, "movej")
            self._gripper_handle = self._login(robot_ip, robot_port, "gripper")
        except Exception:
            if hasattr(self, "_move_handle"):
                self._sdk.API_Logout(ctypes.byref(self._move_handle))
            if hasattr(self, "_servo_handle"):
                self._sdk.API_Logout(ctypes.byref(self._servo_handle))
            self._sdk.API_Logout(ctypes.byref(self._state_handle))
            raise
        self._period = 1.0 / control_hz
        self._max_relative_travel = max_relative_travel
        self._max_step = max_step
        self._max_tracking_error = max_tracking_error
        self._state_check_interval = state_check_interval
        self._allow_manual_servo = allow_manual_servo
        self._gripper_enabled = bool(gripper_enabled)
        self._gripper_speed = _clamp_u8(gripper_speed)
        self._gripper_force = _clamp_u8(gripper_force)
        self._gripper_period = 1.0 / gripper_command_hz
        self._gripper_deadband = int(gripper_deadband)
        self._gripper_timeout = int(gripper_timeout)
        self._robot_ip = robot_ip
        self._robot_port = robot_port
        self._last_state_check = 0.0
        self._lock = threading.Lock()
        self._closed = False
        self._move_active = False
        self._move_error: Optional[str] = None
        self._move_thread: Optional[threading.Thread] = None
        self._move_handle_open = True
        self._motion_stopped = True
        self._servo_used = False
        self._shutdown_requested = False
        self._gripper_condition = threading.Condition()
        self._gripper_pending: Optional[int] = None
        self._gripper_last_queued: Optional[int] = None
        self._gripper_last_reported: Optional[int] = None
        self._gripper_shutdown = False
        self._gripper_thread: Optional[threading.Thread] = None
        self._gripper_last_slow_log = 0.0

        initial = self._read_arm_joints()
        self._initial_arm_joints = initial.copy()
        self._last_command = initial.copy()
        self._last_observation = initial.copy()
        self._last_observation_time = time.monotonic()
        self._virtual_gripper = 0.0
        problems = self._readiness_problems()
        if problems:
            print(
                "IIMT connected in read-only/not-ready state: "
                + ", ".join(problems)
                + ". Motion commands will be rejected until all conditions are satisfied."
            )
        else:
            print("IIMT connected and ready for guarded servo commands.")
        if self._gripper_enabled:
            self._gripper_thread = threading.Thread(
                target=self._gripper_worker,
                name="iimt-gripper",
                daemon=True,
            )
            self._gripper_thread.start()

    def _configure_api(self) -> None:
        handle = ctypes.c_long
        self._sdk.API_Login.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.POINTER(handle)]
        self._sdk.API_Login.restype = ctypes.c_int
        self._sdk.API_Logout.argtypes = [ctypes.POINTER(handle)]
        self._sdk.API_Logout.restype = ctypes.c_int
        self._sdk.API_GetRobotJointRad.argtypes = [handle, ctypes.POINTER(JointRad)]
        self._sdk.API_GetRobotJointRad.restype = ctypes.c_int
        self._sdk.API_IsInRemoteControl.argtypes = [handle, ctypes.POINTER(Text32)]
        self._sdk.API_IsInRemoteControl.restype = ctypes.c_int
        self._sdk.API_GetOperationalMode.argtypes = [handle, ctypes.POINTER(Text32)]
        self._sdk.API_GetOperationalMode.restype = ctypes.c_int
        self._sdk.API_GetSafetyStatus.argtypes = [handle, ctypes.POINTER(Text32)]
        self._sdk.API_GetSafetyStatus.restype = ctypes.c_int
        self._sdk.API_GetRobotMode.argtypes = [handle, ctypes.POINTER(Text32)]
        self._sdk.API_GetRobotMode.restype = ctypes.c_int
        self._sdk.API_GetRobotErrorCode.argtypes = [handle, ctypes.POINTER(ctypes.c_int)]
        self._sdk.API_GetRobotErrorCode.restype = ctypes.c_int
        self._sdk.API_Servoj.argtypes = [handle, ctypes.POINTER(ServojParam)]
        self._sdk.API_Servoj.restype = ctypes.c_int
        self._sdk.API_Movej.argtypes = [
            handle,
            ctypes.POINTER(MovejParam),
            ctypes.c_bool,
            ctypes.c_int,
        ]
        self._sdk.API_Movej.restype = ctypes.c_int
        self._sdk.API_ClearUnfinishedTrajectory.argtypes = [handle]
        self._sdk.API_ClearUnfinishedTrajectory.restype = ctypes.c_int
        self._sdk.API_ModbusRtuRWEndTool.argtypes = [
            handle,
            ctypes.POINTER(ModbusRTURequest),
            ctypes.POINTER(ModbusRTUFrame),
            ctypes.c_int,
        ]
        self._sdk.API_ModbusRtuRWEndTool.restype = ctypes.c_int

    def _login(self, ip: str, port: int, purpose: str) -> ctypes.c_long:
        handle = ctypes.c_long()
        if not self._sdk.API_Login(ip.encode(), port, ctypes.byref(handle)):
            raise RuntimeError(f"IIMT {purpose} login failed: {ip}:{port}")
        return handle

    def _read_text(self, function_name: str) -> str:
        value = Text32()
        function = getattr(self._sdk, function_name)
        if not function(self._state_handle, ctypes.byref(value)):
            raise RuntimeError(f"{function_name} failed")
        return _text(value)

    def _read_arm_joints(self) -> np.ndarray:
        value = JointRad()
        if not self._sdk.API_GetRobotJointRad(self._state_handle, ctypes.byref(value)):
            raise RuntimeError("API_GetRobotJointRad failed")
        joints = np.asarray(list(value.robotRad), dtype=float)
        if not np.all(np.isfinite(joints)):
            raise RuntimeError("IIMT returned non-finite joint positions")
        return joints

    def _readiness_problems(self) -> list[str]:
        remote = self._read_text("API_IsInRemoteControl").strip().lower()
        operation = self._read_text("API_GetOperationalMode").strip().upper()
        safety = self._read_text("API_GetSafetyStatus").strip().upper()
        robot_mode = self._read_text("API_GetRobotMode").strip().upper()
        error_code = ctypes.c_int()
        if not self._sdk.API_GetRobotErrorCode(
            self._state_handle, ctypes.byref(error_code)
        ):
            raise RuntimeError("API_GetRobotErrorCode failed")

        problems = []
        if not self._allow_manual_servo:
            if remote not in {"true", "1", "yes", "remote"}:
                problems.append(f"remote={remote}")
            if "AUTOMATIC" not in operation:
                problems.append(f"operation={operation}")
        if "NORMAL" not in safety or "ABNORMAL" in safety:
            problems.append(f"safety={safety}")
        if "POWER_OFF" in robot_mode or "EMERGENCY" in robot_mode or "ERROR" in robot_mode:
            problems.append(f"robot_mode={robot_mode}")
        if error_code.value != 0:
            problems.append(f"error_code={error_code.value}")
        return problems

    def _check_ready(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and now - self._last_state_check < self._state_check_interval:
            return
        problems = self._readiness_problems()
        if problems:
            raise RuntimeError("IIMT is not ready: " + ", ".join(problems))
        self._last_state_check = now

    def num_dofs(self) -> int:
        return 7

    def get_joint_state(self) -> np.ndarray:
        with self._lock:
            arm = self._read_arm_joints()
            return np.concatenate((arm, np.array([self._virtual_gripper])))

    def _write_gripper_registers(
        self, start_addr: int, values: Sequence[int]
    ) -> bool:
        request = ModbusRTURequest()
        response = ModbusRTUFrame()
        request.slave_addr = 0x09
        request.func_code = 0x10
        request.start_addr = int(start_addr)
        request.data_number = len(values)
        request.data_len = len(values) * 2
        for index, value in enumerate(values):
            word = int(value) & 0xFFFF
            request.data[index * 2] = (word >> 8) & 0xFF
            request.data[index * 2 + 1] = word & 0xFF
        return bool(
            self._sdk.API_ModbusRtuRWEndTool(
                self._gripper_handle,
                ctypes.byref(request),
                ctypes.byref(response),
                self._gripper_timeout,
            )
        )

    def _send_gripper_position(self, position: int) -> bool:
        parameters_ok = self._write_gripper_registers(
            1001,
            [
                _pack_gripper_position(position),
                _pack_gripper_speed_force(
                    self._gripper_speed, self._gripper_force
                ),
            ],
        )
        run_ok = self._write_gripper_registers(1000, [9])
        return parameters_ok and run_ok

    def _queue_gripper_target(self, normalized_position: float) -> None:
        if not self._gripper_enabled:
            return
        normalized = float(np.clip(normalized_position, 0.0, 1.0))
        # Calibration and encoder noise can leave the trigger a fraction short
        # of an endpoint.  Snap only the outer 2% so a released trigger always
        # means fully open and a fully pulled trigger always means fully closed;
        # the rest of the travel remains a linear proportional command.
        if normalized <= 0.02:
            position = 0
        elif normalized >= 0.98:
            position = 255
        else:
            position = int(round(normalized * 255))
        with self._gripper_condition:
            if (
                self._gripper_last_queued is not None
                and abs(position - self._gripper_last_queued)
                < self._gripper_deadband
            ):
                return
            self._gripper_pending = position
            self._gripper_last_queued = position
            self._gripper_condition.notify()

    def _gripper_worker(self) -> None:
        try:
            if self._write_gripper_registers(1000, [1]):
                print(
                    "IIMT gripper enabled; GELLO channel 7 uses binary control "
                    "(trigger >= 0.55: close, trigger <= 0.45: open)"
                )
            else:
                print("WARNING: IIMT gripper enable command was rejected")
        except Exception as error:
            print(f"WARNING: IIMT gripper enable failed: {error}")

        last_command_time = 0.0
        while True:
            with self._gripper_condition:
                while self._gripper_pending is None and not self._gripper_shutdown:
                    self._gripper_condition.wait()
                if self._gripper_shutdown:
                    return
                position = self._gripper_pending
                self._gripper_pending = None

            remaining = self._gripper_period - (time.monotonic() - last_command_time)
            if remaining > 0:
                time.sleep(remaining)
            with self._gripper_condition:
                if self._gripper_shutdown:
                    return
                if self._gripper_pending is not None:
                    position = self._gripper_pending
                    self._gripper_pending = None

            command_started = time.monotonic()
            try:
                command_ok = self._send_gripper_position(int(position))
                if not command_ok:
                    print(
                        f"WARNING: IIMT gripper target {position} was rejected; "
                        "re-enabling and retrying",
                        flush=True,
                    )
            except Exception as error:
                command_ok = False
                print(
                    f"WARNING: IIMT gripper target {position} failed: {error}; "
                    "re-enabling and retrying",
                    flush=True,
                )
            if not command_ok:
                try:
                    command_ok = self._write_gripper_registers(1000, [1])
                    if command_ok:
                        command_ok = self._send_gripper_position(int(position))
                except Exception as error:
                    command_ok = False
                    print(
                        f"WARNING: IIMT gripper recovery failed: {error}",
                        flush=True,
                    )
            if command_ok and position != self._gripper_last_reported:
                state = "CLOSED" if int(position) == 255 else "OPEN"
                print(
                    f"IIMT gripper command accepted: {state} ({position})",
                    flush=True,
                )
                self._gripper_last_reported = int(position)
            if not command_ok:
                # Allow the 250 Hz producer to enqueue the same target again
                # after a transient Modbus failure.
                with self._gripper_condition:
                    if self._gripper_pending is None:
                        self._gripper_last_queued = None
            command_elapsed = time.monotonic() - command_started
            now = time.monotonic()
            if command_elapsed >= 0.10 and now - self._gripper_last_slow_log >= 1.0:
                print(
                    "WARNING: IIMT gripper Modbus command is slow: "
                    f"{command_elapsed * 1000.0:.1f} ms",
                    flush=True,
                )
                self._gripper_last_slow_log = now
            last_command_time = time.monotonic()

    def _stop_gripper_worker(self) -> bool:
        thread = self._gripper_thread
        if thread is None:
            return True
        with self._gripper_condition:
            self._gripper_shutdown = True
            self._gripper_condition.notify_all()
        thread.join(timeout=max(3.0, self._gripper_timeout * 3.0))
        return not thread.is_alive()

    def command_gripper_state(self, position: float) -> None:
        """Command only the gripper using a noise-resistant binary threshold."""
        if not np.isfinite(position):
            raise ValueError("IIMT gripper command contains NaN or infinity")
        with self._lock:
            normalized = _binary_gripper_target(position, self._virtual_gripper)
            self._virtual_gripper = normalized
        self._queue_gripper_target(normalized)

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        requested = np.asarray(joint_state, dtype=float)
        if requested.shape != (7,):
            raise ValueError(f"IIMT expected 7 channels, got shape {requested.shape}")
        if not np.all(np.isfinite(requested)):
            raise ValueError("IIMT command contains NaN or infinity")

        with self._lock:
            self._check_ready()
            target = requested[:6]
            if self._max_relative_travel > 0 and np.max(
                np.abs(target - self._initial_arm_joints)
            ) > self._max_relative_travel:
                raise RuntimeError("IIMT command exceeded relative-travel limit")
            if np.max(np.abs(target - self._last_command)) > self._max_step:
                raise RuntimeError("IIMT command exceeded per-cycle step limit")

            actual = self._read_arm_joints()
            if np.max(np.abs(target - actual)) > self._max_tracking_error:
                raise RuntimeError("IIMT tracking error exceeded limit")

            parameter = ServojParam()
            for index, value in enumerate(target):
                parameter.q[index] = float(value)
            parameter.a = 7.0
            parameter.v = 5.0
            parameter.t = self._period
            parameter.lookahead_time = 0.0
            parameter.gain = 0.0
            if not self._sdk.API_Servoj(self._servo_handle, ctypes.byref(parameter)):
                raise RuntimeError("API_Servoj rejected the command")
            self._servo_used = True
            self._motion_stopped = False
            self._last_command = target.copy()
            self._virtual_gripper = _binary_gripper_target(
                requested[6], self._virtual_gripper
            )
            self._queue_gripper_target(self._virtual_gripper)

    def move_joint_state(
        self,
        joint_state: np.ndarray,
        velocity: float,
        acceleration: float,
        jerk: float,
        timeout: float,
        max_segment: float,
    ) -> None:
        """Run a segmented SDK MoveJ trajectory without blocking ZMQ."""
        requested = np.asarray(joint_state, dtype=float)
        if requested.shape != (7,):
            raise ValueError(f"IIMT expected 7 channels, got shape {requested.shape}")
        if not np.all(np.isfinite(requested)):
            raise ValueError("IIMT MoveJ command contains NaN or infinity")
        if min(velocity, acceleration, jerk, timeout, max_segment) <= 0:
            raise ValueError("IIMT MoveJ parameters and max segment must be positive")

        with self._lock:
            self._check_ready(force=True)
            if self._move_active:
                raise RuntimeError("an IIMT MoveJ trajectory is already active")
            if not self._move_handle_open:
                self._move_handle = self._login(
                    self._robot_ip, self._robot_port, "movej restart"
                )
                self._move_handle_open = True
                self._shutdown_requested = False
            target = requested[:6]
            start = self._read_arm_joints()
            segment_count = max(
                1, int(math.ceil(np.max(np.abs(target - start)) / max_segment))
            )
            waypoints = [
                start + (target - start) * (index / segment_count)
                for index in range(1, segment_count + 1)
            ]
            self._move_active = True
            self._move_error = None
            self._motion_stopped = False
            self._last_command = target.copy()
            self._virtual_gripper = float(np.clip(requested[6], 0.0, 1.0))

        def run_movej() -> None:
            error = None
            accepted = False
            attempts = 2
            per_attempt_timeout = max(3, min(8, int(math.ceil(timeout / attempts))))
            print(
                f"IIMT segmented MoveJ worker started: {segment_count} segments, "
                f"max {max_segment:.3f} rad each"
            )
            try:
                connection_needs_settle = True
                trajectory_clear_needed = True
                for segment_index, waypoint in enumerate(waypoints, start=1):
                    if self._shutdown_requested:
                        raise RuntimeError("MoveJ cancelled by shutdown request")
                    parameter = MovejParam()
                    for joint_index, value in enumerate(waypoint):
                        parameter.q[joint_index] = float(value)
                    parameter.v = float(velocity)
                    parameter.a = float(acceleration)
                    parameter.j = float(jerk)
                    parameter.r = 0.0
                    parameter.t = 0.0
                    accepted = False
                    for attempt in range(1, attempts + 1):
                        if connection_needs_settle:
                            time.sleep(0.75)
                            connection_needs_settle = False
                        if trajectory_clear_needed:
                            if not self._sdk.API_ClearUnfinishedTrajectory(
                                self._move_handle
                            ):
                                raise RuntimeError(
                                    "API_ClearUnfinishedTrajectory failed before MoveJ"
                                )
                            trajectory_clear_needed = False
                            print("IIMT unfinished trajectory state cleared")
                        print(
                            f"IIMT MoveJ segment {segment_index}/{segment_count}, "
                            f"attempt {attempt}/{attempts}"
                        )
                        accepted = bool(
                            self._sdk.API_Movej(
                                self._move_handle,
                                ctypes.byref(parameter),
                                True,
                                per_attempt_timeout,
                            )
                        )
                        if accepted:
                            break
                        with self._lock:
                            problems = self._readiness_problems()
                        if problems:
                            error = (
                                f"MoveJ segment {segment_index} was rejected and "
                                "IIMT entered a fault state: " + ", ".join(problems)
                            )
                            break
                        if self._move_handle_open:
                            self._sdk.API_Logout(ctypes.byref(self._move_handle))
                            self._move_handle_open = False
                        if attempt < attempts:
                            self._move_handle = self._login(
                                self._robot_ip, self._robot_port, "movej retry"
                            )
                            self._move_handle_open = True
                            connection_needs_settle = True
                            trajectory_clear_needed = True
                    if error is not None:
                        break
                    if not accepted:
                        error = (
                            f"API_Movej rejected segment {segment_index}/"
                            f"{segment_count} after {attempts} attempts"
                        )
                        break
                    print(f"IIMT MoveJ segment {segment_index}/{segment_count} completed")
                    if segment_index < segment_count:
                        time.sleep(0.3)
            except Exception as exception:
                error = f"API_Movej failed: {exception}"
            finally:
                try:
                    if self._move_handle_open:
                        self._sdk.API_Logout(ctypes.byref(self._move_handle))
                        self._move_handle_open = False
                except Exception as exception:
                    if error is None:
                        error = f"MoveJ connection cleanup failed: {exception}"
                # Give the controller time to leave target-move mode before the
                # first Servoj command arrives on its dedicated connection.
                if error is None:
                    time.sleep(0.5)
                with self._lock:
                    self._move_error = error
                    self._move_active = False
                if error is None:
                    print("IIMT MoveJ worker completed and released its connection")
                else:
                    print(f"IIMT MoveJ worker failed: {error}")

        self._move_thread = threading.Thread(
            target=run_movej, name="iimt-movej", daemon=True
        )
        self._move_thread.start()

    def get_move_status(self) -> Dict[str, object]:
        with self._lock:
            return {
                "active": self._move_active,
                "error": self._move_error,
                "servo_used": self._servo_used,
            }

    def stop_motion(self) -> None:
        """Clear SDK trajectory state so a later process can start with MoveJ."""
        with self._lock:
            if self._motion_stopped:
                return
            self._shutdown_requested = True
            state_clear_ok = bool(
                self._sdk.API_ClearUnfinishedTrajectory(self._state_handle)
            )
            servo_clear_ok = bool(
                self._sdk.API_ClearUnfinishedTrajectory(self._servo_handle)
            )
            clear_results = [state_clear_ok, servo_clear_ok]
            move_clear_ok = None
            if self._move_handle_open:
                move_clear_ok = bool(
                    self._sdk.API_ClearUnfinishedTrajectory(self._move_handle)
                )
                clear_results.append(move_clear_ok)
            if not any(clear_results):
                raise RuntimeError(
                    "failed to clear IIMT trajectory state "
                    f"(state={state_clear_ok}, servo={servo_clear_ok}, "
                    f"movej={move_clear_ok})"
                )
            time.sleep(0.3)
            self._last_command = self._read_arm_joints()
            self._motion_stopped = True
            print("IIMT trajectory state cleared before shutdown")

    def get_observations(self) -> Dict[str, np.ndarray]:
        with self._lock:
            now = time.monotonic()
            arm = self._read_arm_joints()
            elapsed = max(now - self._last_observation_time, 1e-6)
            arm_velocity = (arm - self._last_observation) / elapsed
            self._last_observation = arm.copy()
            self._last_observation_time = now
            positions = np.concatenate((arm, np.array([self._virtual_gripper])))
            velocities = np.concatenate((arm_velocity, np.array([0.0])))
            return {
                "joint_positions": positions,
                "joint_velocities": velocities,
                "ee_pos_quat": np.zeros(7, dtype=float),
                "gripper_position": np.array(self._virtual_gripper),
            }

    def close(self) -> None:
        gripper_stopped = self._stop_gripper_worker()
        with self._lock:
            if self._closed:
                return
            if not self._motion_stopped:
                state_clear_ok = bool(
                    self._sdk.API_ClearUnfinishedTrajectory(self._state_handle)
                )
                servo_clear_ok = bool(
                    self._sdk.API_ClearUnfinishedTrajectory(self._servo_handle)
                )
                if state_clear_ok or servo_clear_ok:
                    self._motion_stopped = True
                    print("IIMT trajectory state cleared during close")
                else:
                    print(
                        "WARNING: IIMT close could not clear trajectory state"
                    )
            if self._move_handle_open and not self._move_active:
                self._sdk.API_Logout(ctypes.byref(self._move_handle))
                self._move_handle_open = False
            if gripper_stopped:
                self._sdk.API_Logout(ctypes.byref(self._gripper_handle))
            else:
                print("WARNING: IIMT gripper worker did not stop; handle kept open")
            self._sdk.API_Logout(ctypes.byref(self._servo_handle))
            self._sdk.API_Logout(ctypes.byref(self._state_handle))
            self._closed = True

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
