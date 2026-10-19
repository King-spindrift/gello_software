import time
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

from gello.robots.robot import Robot


class DynamixelRobot(Robot):
    """A class representing a UR robot."""

    def __init__(
        self,
        joint_ids: Sequence[int],
        joint_offsets: Optional[Sequence[float]] = None,
        joint_signs: Optional[Sequence[int]] = None,
        real: bool = False,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 57600,
        gripper_config: Optional[Tuple[int, float, float]] = None,
        start_joints: Optional[np.ndarray] = None,
    ):
        from gello.dynamixel.driver import (
            DynamixelDriver,
            DynamixelDriverProtocol,
            FakeDynamixelDriver,
        )

        print(f"attempting to connect to port: {port}")
        self.gripper_open_close: Optional[Tuple[float, float]]
        if gripper_config is not None:
            assert joint_offsets is not None
            assert joint_signs is not None

            # joint_ids.append(gripper_config[0])
            # joint_offsets.append(0.0)
            # joint_signs.append(1)
            joint_ids = tuple(joint_ids) + (gripper_config[0],)
            joint_offsets = tuple(joint_offsets) + (0.0,)
            joint_signs = tuple(joint_signs) + (1,)
            self.gripper_open_close = (
                gripper_config[1] * np.pi / 180,
                gripper_config[2] * np.pi / 180,
            )
        else:
            self.gripper_open_close = None

        self._joint_ids = joint_ids
        self._driver: DynamixelDriverProtocol

        if joint_offsets is None:
            self._joint_offsets = np.zeros(len(joint_ids))
        else:
            self._joint_offsets = np.array(joint_offsets)

        if joint_signs is None:
            self._joint_signs = np.ones(len(joint_ids))
        else:
            self._joint_signs = np.array(joint_signs)

        assert len(self._joint_ids) == len(self._joint_offsets), (
            f"joint_ids: {len(self._joint_ids)}, "
            f"joint_offsets: {len(self._joint_offsets)}"
        )
        assert len(self._joint_ids) == len(self._joint_signs), (
            f"joint_ids: {len(self._joint_ids)}, "
            f"joint_signs: {len(self._joint_signs)}"
        )
        assert np.all(
            np.abs(self._joint_signs) == 1
        ), f"joint_signs: {self._joint_signs}"

        if real:
            self._driver = DynamixelDriver(joint_ids, port=port, baudrate=baudrate)
            self._driver.set_torque_mode(False)
        else:
            self._driver = FakeDynamixelDriver(joint_ids)
        self._torque_on = False
        self._last_pos = None
        self._alpha = 0.99

        if start_joints is not None:
            # loop through all joints and add +- 2pi to the joint offsets to get the closest to start joints
            new_joint_offsets = []
            current_joints = self.get_joint_state()
            assert current_joints.shape == start_joints.shape
            if gripper_config is not None:
                current_joints = current_joints[:-1]
                start_joints = start_joints[:-1]
            for idx, (c_joint, s_joint, joint_offset) in enumerate(
                zip(current_joints, start_joints, self._joint_offsets)
            ):
                new_joint_offsets.append(
                    np.pi
                    * 2
                    * np.round((-s_joint + c_joint) / (2 * np.pi))
                    * self._joint_signs[idx]
                    + joint_offset
                )
            if gripper_config is not None:
                new_joint_offsets.append(self._joint_offsets[-1])
            self._joint_offsets = np.array(new_joint_offsets)

    def num_dofs(self) -> int:
        return len(self._joint_ids)

    def get_joint_state(self) -> np.ndarray:
        pos = (self._driver.get_joints() - self._joint_offsets) * self._joint_signs
        assert len(pos) == self.num_dofs()

        if self.gripper_open_close is not None:
            # map pos to [0, 1]
            g_pos = (pos[-1] - self.gripper_open_close[0]) / (
                self.gripper_open_close[1] - self.gripper_open_close[0]
            )
            g_pos = min(max(0, g_pos), 1)
            pos[-1] = g_pos

        if self._last_pos is None:
            self._last_pos = pos
        else:
            # exponential smoothing
            pos = self._last_pos * (1 - self._alpha) + pos * self._alpha
            self._last_pos = pos

        return pos

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        joint_state = np.asarray(joint_state, dtype=float)
        if joint_state.shape != (self.num_dofs(),):
            raise ValueError(f"expected {self.num_dofs()} joints, got {joint_state.size}")
        logical = joint_state.copy()
        if self.gripper_open_close is not None:
            logical[-1] = self.gripper_open_close[0] + np.clip(
                logical[-1], 0.0, 1.0
            ) * (self.gripper_open_close[1] - self.gripper_open_close[0])
        raw = logical * self._joint_signs + self._joint_offsets
        self._driver.set_joints(raw.tolist())

    def align_arm_to_joint_state(
        self,
        target_arm: np.ndarray,
        *,
        max_step: float = 0.003,
        rate_hz: float = 30.0,
        tolerance: float = 0.02,
        max_travel: float = 1.5,
        release_countdown: int = 5,
        release_torque: bool = True,
        position_p_gain=None,
    ) -> None:
        """Drive the GELLO arm slowly to a target, then release its torque."""
        arm_dofs = self.num_dofs() - (1 if self.gripper_open_close is not None else 0)
        target_arm = np.asarray(target_arm, dtype=float)
        if target_arm.shape != (arm_dofs,):
            raise ValueError(f"expected {arm_dofs} arm joints, got {target_arm.size}")
        if max_step <= 0 or rate_hz <= 0 or tolerance <= 0 or max_travel <= 0:
            raise ValueError("alignment safety parameters must be positive")
        if release_countdown < 0:
            raise ValueError("release_countdown must not be negative")

        current_raw = self._driver.get_joints()
        current_logical = (
            current_raw[:arm_dofs] - self._joint_offsets[:arm_dofs]
        ) * self._joint_signs[:arm_dofs]
        # Each revolute target has equivalent 2*pi representations. Choose the
        # one nearest the present GELLO position to avoid unnecessary rotation.
        target_near = target_arm + 2 * np.pi * np.round(
            (current_logical - target_arm) / (2 * np.pi)
        )
        delta = target_near - current_logical
        largest_move = float(np.max(np.abs(delta)))
        if largest_move > max_travel:
            raise RuntimeError(
                f"GELLO alignment requires {largest_move:.3f} rad, "
                f"above --gello-align-max-travel={max_travel:.3f}"
            )

        steps = max(1, int(np.ceil(largest_move / max_step)))
        target_raw = current_raw.copy()
        target_raw[:arm_dofs] = (
            target_near * self._joint_signs[:arm_dofs]
            + self._joint_offsets[:arm_dofs]
        )

        # Prime the current goal before torque-on so stale registers cannot
        # produce a jump. Keep the gripper at its measured raw position.
        # Extended position mode allows the nearest equivalent target across a
        # 0/2*pi encoder boundary instead of forcing the long way around.
        self._driver.set_operating_mode(4)
        self._driver.verify_operating_mode(4)
        if position_p_gain is not None:
            self._driver.set_position_p_gain(position_p_gain, arm_dofs)
        self._driver.prepare_joints(current_raw.tolist())
        self.set_torque_mode(True)
        alignment_succeeded = False
        try:
            for index in range(1, steps + 1):
                ratio = index / steps
                command_raw = current_raw + ratio * (target_raw - current_raw)
                self._driver.set_joints(command_raw.tolist())
                time.sleep(1.0 / rate_hz)

            time.sleep(0.2)
            measured_raw = self._driver.get_joints()
            measured = (
                measured_raw[:arm_dofs] - self._joint_offsets[:arm_dofs]
            ) * self._joint_signs[:arm_dofs]
            error = np.abs(measured - target_near)
            if float(np.max(error)) > tolerance:
                raise RuntimeError(
                    f"GELLO alignment error {float(np.max(error)):.3f} rad "
                    f"exceeds tolerance {tolerance:.3f}"
                )
            alignment_succeeded = True
            if release_torque:
                print("GELLO is aligned and holding position; support it by hand now")
                for seconds in range(release_countdown, 0, -1):
                    print(f"GELLO motor torque releases in {seconds}...")
                    time.sleep(1.0)
        finally:
            if release_torque or not alignment_succeeded:
                self.set_torque_mode(False)

    def set_torque_mode(self, mode: bool):
        if mode == self._torque_on:
            return
        self._driver.set_torque_mode(mode)
        self._torque_on = mode

    def get_observations(self) -> Dict[str, np.ndarray]:
        return {"joint_state": self.get_joint_state()}
