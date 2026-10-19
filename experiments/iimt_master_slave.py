"""Current-pose GELLO -> IIMT master/slave controller."""

from __future__ import annotations

import json
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import ruckig
import tyro

from gello.agents.gello_agent import GelloAgent
from gello.zmq_core.robot_node import ZMQClientRobot


@dataclass
class Args:
    config: str = "configs/iimt_master_slave.json"
    resume_servo_alignment: bool = False


@dataclass(frozen=True)
class FollowerSpec:
    name: str
    robot_ip: str
    robot_port: int
    joint_gains: tuple[float, float, float, float, float, float]


@dataclass
class FollowerRuntime:
    spec: FollowerSpec
    client: ZMQClientRobot
    origin: np.ndarray
    otg: ruckig.Ruckig
    trajectory_input: ruckig.InputParameter
    trajectory_output: ruckig.OutputParameter
    point_filter: PointDeltaHysteresis
    damping_filter: CriticallyDampedSecondOrder


def _load_follower_specs(config: dict) -> list[FollowerSpec]:
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
    legacy_gain = float(config.get("joint_gain", 1.0))
    specs = []
    for index, item in enumerate(raw_followers):
        if not bool(item.get("enabled", index == 0)):
            continue
        raw_gains = item.get("joint_gains", [legacy_gain] * 6)
        if not isinstance(raw_gains, list) or len(raw_gains) != 6:
            raise ValueError(f"follower {index + 1} joint_gains must contain 6 values")
        gains = tuple(float(value) for value in raw_gains)
        if any(not -10.0 <= value <= 10.0 for value in gains):
            raise ValueError(
                f"follower {index + 1} joint gains must be between -10 and 10"
            )
        specs.append(
            FollowerSpec(
                name=str(item.get("name", f"从臂{index + 1}")),
                robot_ip=str(item.get("robot_ip", "")),
                robot_port=int(item.get("robot_port", 6001 + index)),
                joint_gains=gains,
            )
        )
    if not specs:
        raise ValueError("at least one follower must be enabled")
    return specs


def _wrapped_delta(current: np.ndarray, previous: np.ndarray) -> np.ndarray:
    delta = current - previous
    delta[:6] = (delta[:6] + np.pi) % (2 * np.pi) - np.pi
    return delta


class PointDeltaHysteresis:
    """Reject sub-threshold point-to-point leader motion without chatter."""

    def __init__(self, size: int, threshold: float, hysteresis: float):
        if not 0 <= hysteresis < threshold:
            raise ValueError("point hysteresis must satisfy 0 <= hysteresis < threshold")
        self.value = np.zeros(size, dtype=float)
        self.threshold = threshold
        self.hysteresis = hysteresis

    def update(self, sample: np.ndarray) -> np.ndarray:
        difference = sample - self.value
        changed = np.abs(difference) >= self.threshold
        self.value[changed] = (
            sample[changed] - np.sign(difference[changed]) * self.hysteresis
        )
        return self.value.copy()


class CriticallyDampedSecondOrder:
    """Exact per-step update of a critically damped second-order filter."""

    def __init__(self, size: int, natural_frequency: float, period: float):
        if natural_frequency <= 0 or period <= 0:
            raise ValueError("second-order filter parameters must be positive")
        self.position = np.zeros(size, dtype=float)
        self.velocity = np.zeros(size, dtype=float)
        self.omega = natural_frequency
        self.period = period

    def update(self, target: np.ndarray) -> np.ndarray:
        error = self.position - target
        combined = self.velocity + self.omega * error
        decay = np.exp(-self.omega * self.period)
        new_error = (error + combined * self.period) * decay
        self.velocity = (
            self.velocity - self.omega * combined * self.period
        ) * decay
        self.position = target + new_error
        return self.position.copy()


def _move_follower_to_leader_pose(
    robot: ZMQClientRobot,
    target: np.ndarray,
    velocity: float,
    acceleration: float,
    jerk: float,
    max_segment: float,
    tolerance: float,
    servo_blend_threshold: float,
    resume_servo_alignment: bool,
    timeout: float,
    hz: float,
) -> None:
    parameters = (
        velocity,
        acceleration,
        jerk,
        max_segment,
        tolerance,
        servo_blend_threshold,
        timeout,
        hz,
    )
    if min(parameters) <= 0:
        raise ValueError("MoveJ parameters, tolerance, timeout, and frequency must be positive")
    if servo_blend_threshold < tolerance:
        raise ValueError("servo blend threshold must be greater than or equal to tolerance")
    current = np.asarray(robot.get_joint_state(), dtype=float)
    target = target.copy()
    target[6:] = current[6:]
    initial_error = float(np.max(np.abs(current[:6] - target[:6])))
    if initial_error <= tolerance:
        print(f"IIMT follower already matches the GELLO pose (error {initial_error:.3f} rad)")
        return
    if resume_servo_alignment:
        print(
            f"IIMT startup offset {initial_error:.3f} rad on a resumed controller "
            "session; MoveJ is disabled and alignment will use Ruckig + Servoj"
        )
        return
    session_status = robot.get_move_status()
    if bool(session_status.get("servo_used", False)):
        print(
            f"IIMT startup offset {initial_error:.3f} rad on a resumed Servoj "
            "session; avoiding MoveJ and blending with Ruckig + Servoj"
        )
        return
    if initial_error <= servo_blend_threshold:
        print(
            f"IIMT startup offset {initial_error:.3f} rad is small; skipping MoveJ "
            "and blending to the target with Ruckig + Servoj"
        )
        return

    segment_count = max(1, int(np.ceil(initial_error / max_segment)))
    print(
        f"IIMT startup offset {initial_error:.3f} rad; using {segment_count} "
        f"MoveJ segment(s) at v={velocity:.2f}, a={acceleration:.2f}, j={jerk:.2f}"
    )
    print("Moving IIMT follower to the current GELLO pose with API_Movej")
    robot.move_joint_state(
        target,
        velocity,
        acceleration,
        jerk,
        timeout,
        max_segment,
    )
    error = initial_error
    deadline = time.monotonic() + timeout
    period = 1.0 / hz
    while time.monotonic() < deadline:
        actual = np.asarray(robot.get_joint_state(), dtype=float)
        error = float(np.max(np.abs(actual[:6] - target[:6])))
        move_status = robot.get_move_status()
        if move_status["error"]:
            raise RuntimeError(str(move_status["error"]))
        if error <= tolerance and not move_status["active"]:
            print(f"IIMT follower matched the GELLO pose (error {error:.3f} rad)")
            return
        time.sleep(period)
    raise RuntimeError(
        f"IIMT MoveJ did not reach the GELLO pose within {timeout:.1f} seconds; "
        f"last error {error:.3f} rad"
    )


def _run_gripper_only(
    leader: GelloAgent,
    followers: list[tuple[FollowerSpec, ZMQClientRobot]],
    hz: float,
) -> None:
    """Forward only GELLO channel 7 to every follower."""
    period = 1.0 / hz
    print(
        "GRIPPER-ONLY MODE: follower arm motion is disabled; "
        f"only GELLO channel 7 controls {len(followers)} gripper(s)"
    )
    with ThreadPoolExecutor(max_workers=len(followers)) as executor:
        while True:
            started = time.monotonic()
            leader_state = np.asarray(leader.act({}), dtype=float)
            gripper = float(np.clip(leader_state[6], 0.0, 1.0))
            futures = [
                executor.submit(client.command_gripper_state, gripper)
                for _, client in followers
            ]
            for future in futures:
                future.result()
            remaining = period - (time.monotonic() - started)
            if remaining > 0:
                time.sleep(remaining)


def main(args: Args) -> None:
    config_path = Path(args.config).expanduser().resolve()
    config = json.loads(config_path.read_text(encoding="utf-8"))
    hz = float(config["control_hz"])
    home_hz = float(config["home_hz"])
    period = 1.0 / hz
    follower_specs = _load_follower_specs(config)
    master_reference = np.asarray(config["master_home"], dtype=float)
    follower_reference = np.asarray(config["follower_home"], dtype=float)
    if master_reference.shape != (7,) or follower_reference.shape != (7,):
        raise ValueError("master_home and follower_home must each contain 7 values")
    joint_directions = np.asarray(config["joint_directions"], dtype=float)
    if joint_directions.shape != (6,) or not np.all(np.isin(joint_directions, (-1, 1))):
        raise ValueError("joint_directions must contain six values, each either -1 or 1")

    followers: list[tuple[FollowerSpec, ZMQClientRobot]] = []
    leader = None
    try:
        followers = [
            (
                spec,
                ZMQClientRobot(
                    port=spec.robot_port, host="127.0.0.1", timeout_ms=1000
                ),
            )
            for spec in follower_specs
        ]
        leader = GelloAgent(port=config["gello_port"])
        # GELLO starts torque-free.  Its current pose becomes the master origin;
        # neither arm is first driven back to the stored calibration pose.
        leader.set_torque_mode(False)
        if not bool(config.get("arm_motion_enabled", True)):
            _run_gripper_only(leader, followers, hz)
            return
        leader_origin = np.asarray(leader.act({}), dtype=float)
        initial_master_delta = (
            _wrapped_delta(leader_origin, master_reference)[:6]
            * joint_directions
        )
        follower_origins: list[np.ndarray] = []
        for spec, _ in followers:
            follower_origin = follower_reference.copy()
            follower_origin[:6] += initial_master_delta * np.asarray(
                spec.joint_gains, dtype=float
            )
            follower_origins.append(follower_origin)
        print(f"GELLO startup joints: {np.round(leader_origin[:6], 4).tolist()}")
        for (spec, _), origin in zip(followers, follower_origins):
            print(
                f"{spec.name} ({spec.robot_ip}) alignment target: "
                f"{np.round(origin[:6], 4).tolist()}; "
                f"joint gains={list(spec.joint_gains)}"
            )
        with ThreadPoolExecutor(max_workers=len(followers)) as executor:
            alignment_futures = [
                executor.submit(
                    _move_follower_to_leader_pose,
                    client,
                    origin,
                    float(config["follower_movej_velocity"]),
                    float(config["follower_movej_acceleration"]),
                    float(config["follower_movej_jerk"]),
                    float(config["follower_movej_max_segment"]),
                    float(config["follower_home_tolerance"]),
                    float(config["follower_servo_blend_threshold"]),
                    args.resume_servo_alignment,
                    float(config["follower_home_movej_timeout"]),
                    home_hz,
                )
                for (_, client), origin in zip(followers, follower_origins)
            ]
            for future in alignment_futures:
                future.result()
            state_futures = [
                executor.submit(client.get_joint_state) for _, client in followers
            ]
            current_states = [
                np.asarray(future.result(), dtype=float)
                for future in state_futures
            ]

        previous_leader = leader_origin.copy()
        accumulated_delta = np.zeros(7, dtype=float)
        max_velocity = float(config["max_velocity"])
        max_acceleration = float(config["max_acceleration"])
        max_jerk = float(config["max_jerk"])
        runtimes: list[FollowerRuntime] = []
        for (spec, client), origin, current in zip(
            followers, follower_origins, current_states
        ):
            otg = ruckig.Ruckig(6, period)
            trajectory_input = ruckig.InputParameter(6)
            trajectory_output = ruckig.OutputParameter(6)
            trajectory_input.current_position = current[:6].tolist()
            trajectory_input.current_velocity = [0.0] * 6
            trajectory_input.current_acceleration = [0.0] * 6
            trajectory_input.target_position = origin[:6].tolist()
            trajectory_input.target_velocity = [0.0] * 6
            trajectory_input.target_acceleration = [0.0] * 6
            trajectory_input.max_velocity = [max_velocity] * 6
            trajectory_input.max_acceleration = [max_acceleration] * 6
            trajectory_input.max_jerk = [max_jerk] * 6
            runtimes.append(
                FollowerRuntime(
                    spec,
                    client,
                    origin,
                    otg,
                    trajectory_input,
                    trajectory_output,
                    PointDeltaHysteresis(
                        6,
                        float(config["point_delta_threshold"]),
                        float(config["point_delta_hysteresis"]),
                    ),
                    CriticallyDampedSecondOrder(
                        6,
                        float(config["second_order_natural_frequency"]),
                        period,
                    ),
                )
            )
        print(
            "RUNNING: current-pose alignment complete; "
            f"{len(runtimes)} follower(s) active"
        )

        with ThreadPoolExecutor(max_workers=len(runtimes)) as executor:
            while True:
                started = time.monotonic()
                current_leader = np.asarray(leader.act({}), dtype=float)
                accumulated_delta += _wrapped_delta(current_leader, previous_leader)
                previous_leader = current_leader.copy()

                commands: list[np.ndarray] = []
                for runtime in runtimes:
                    mapped_delta = (
                        accumulated_delta[:6]
                        * joint_directions
                        * np.asarray(runtime.spec.joint_gains, dtype=float)
                    )
                    point_delta = runtime.point_filter.update(mapped_delta)
                    damped_delta = runtime.damping_filter.update(point_delta)
                    runtime.trajectory_input.target_position = (
                        runtime.origin[:6] + damped_delta
                    ).tolist()
                    result = runtime.otg.update(
                        runtime.trajectory_input, runtime.trajectory_output
                    )
                    if result not in (
                        ruckig.Result.Working,
                        ruckig.Result.Finished,
                    ):
                        raise RuntimeError(
                            f"{runtime.spec.name} Ruckig update failed: {result}"
                        )
                    command = runtime.origin.copy()
                    command[:6] = np.asarray(
                        runtime.trajectory_output.new_position, dtype=float
                    )
                    command[6] = np.clip(current_leader[6], 0.0, 1.0)
                    commands.append(command)

                command_futures = [
                    executor.submit(runtime.client.command_joint_state, command)
                    for runtime, command in zip(runtimes, commands)
                ]
                for future in command_futures:
                    future.result()
                for runtime in runtimes:
                    runtime.trajectory_output.pass_to_input(
                        runtime.trajectory_input
                    )

                remaining = period - (time.monotonic() - started)
                if remaining > 0:
                    time.sleep(remaining)
    except KeyboardInterrupt:
        print("Stopping master/slave control")
    finally:
        if leader is not None:
            try:
                leader.set_torque_mode(False)
            except Exception as error:
                print(f"WARNING: failed to release GELLO torque: {error}")
        for spec, follower in followers:
            try:
                follower.stop_motion()
            except Exception as error:
                print(
                    f"WARNING: failed to cleanly stop {spec.name} motion: {error}"
                )
                cleanup_client = None
                try:
                    cleanup_client = ZMQClientRobot(
                        port=spec.robot_port,
                        host="127.0.0.1",
                        timeout_ms=3000,
                    )
                    cleanup_client.stop_motion()
                    print(
                        f"{spec.name} trajectory cleanup succeeded on a fresh "
                        "ZMQ connection"
                    )
                except Exception as cleanup_error:
                    print(
                        f"WARNING: fresh-connection cleanup for {spec.name} failed: "
                        f"{cleanup_error}"
                    )
                finally:
                    if cleanup_client is not None:
                        cleanup_client.close()
            finally:
                follower.close()


if __name__ == "__main__":
    main(tyro.cli(Args))
