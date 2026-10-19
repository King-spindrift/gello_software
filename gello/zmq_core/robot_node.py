import pickle
import threading
from typing import Any, Dict

import numpy as np
import zmq

from gello.robots.robot import Robot

DEFAULT_ROBOT_PORT = 6000


class ZMQServerRobot:
    def __init__(
        self,
        robot: Robot,
        port: int = DEFAULT_ROBOT_PORT,
        host: str = "127.0.0.1",
    ):
        self._robot = robot
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.REP)
        addr = f"tcp://{host}:{port}"
        debug_message = f"Robot Sever Binding to {addr}, Robot: {robot}"
        print(debug_message)
        self._timout_message = f"Timeout in Robot Server, Robot: {robot}"
        self._socket.bind(addr)
        self._stop_event = threading.Event()

    def serve(self) -> None:
        """Serve the leader robot state over ZMQ."""
        self._socket.setsockopt(zmq.RCVTIMEO, 1000)  # Set timeout to 1000 ms
        while not self._stop_event.is_set():
            try:
                # Wait for next request from client
                message = self._socket.recv()
                request = pickle.loads(message)

                try:
                    method = request.get("method")
                    args = request.get("args", {})
                    result: Any
                    if method == "num_dofs":
                        result = self._robot.num_dofs()
                    elif method == "get_joint_state":
                        result = self._robot.get_joint_state()
                    elif method == "command_joint_state":
                        result = self._robot.command_joint_state(**args)
                    elif method == "command_gripper_state":
                        command = getattr(self._robot, "command_gripper_state")
                        result = command(**args)
                    elif method == "move_joint_state":
                        result = self._robot.move_joint_state(**args)
                    elif method == "get_move_status":
                        result = self._robot.get_move_status()
                    elif method == "stop_motion":
                        result = self._robot.stop_motion()
                    elif method == "get_observations":
                        result = self._robot.get_observations()
                    else:
                        raise NotImplementedError(f"Invalid method: {method}")
                except Exception as error:
                    result = {
                        "__rpc_error__": f"{type(error).__name__}: {error}"
                    }
                    print(f"Robot request failed: {result['__rpc_error__']}")

                self._socket.send(pickle.dumps(result))
            except zmq.Again:
                # Timeout occurred - don't spam the console
                pass
            except zmq.ContextTerminated:
                break
            except zmq.ZMQError:
                if self._stop_event.is_set():
                    break
                raise

    def stop(self) -> None:
        """Signal the server to stop serving."""
        self._stop_event.set()

    def close(self) -> None:
        self.stop()
        if hasattr(self._robot, "close"):
            self._robot.close()
        self._socket.close(linger=0)
        self._context.term()


class ZMQClientRobot(Robot):
    """A class representing a ZMQ client for a leader robot."""

    def __init__(
        self,
        port: int = DEFAULT_ROBOT_PORT,
        host: str = "127.0.0.1",
        timeout_ms: int = 500,
    ):
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.REQ)
        self._socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        self._socket.setsockopt(zmq.SNDTIMEO, timeout_ms)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.connect(f"tcp://{host}:{port}")

    def num_dofs(self) -> int:
        """Get the number of joints in the robot.

        Returns:
            int: The number of joints in the robot.
        """
        request = {"method": "num_dofs"}
        result = self._request(request)
        return result

    def _request(self, request: Dict[str, Any], timeout_ms: int | None = None):
        previous_receive_timeout = self._socket.getsockopt(zmq.RCVTIMEO)
        previous_send_timeout = self._socket.getsockopt(zmq.SNDTIMEO)
        if timeout_ms is not None:
            self._socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
            self._socket.setsockopt(zmq.SNDTIMEO, timeout_ms)
        try:
            self._socket.send(pickle.dumps(request))
            result = pickle.loads(self._socket.recv())
        except zmq.Again as error:
            raise RuntimeError("ZMQ timeout - robot may be disconnected") from error
        finally:
            if timeout_ms is not None:
                self._socket.setsockopt(zmq.RCVTIMEO, previous_receive_timeout)
                self._socket.setsockopt(zmq.SNDTIMEO, previous_send_timeout)
        # Robot methods are allowed to return their own status field named
        # "error" (for example get_move_status).  Only the private envelope
        # key below represents an RPC execution failure.
        if isinstance(result, dict) and "__rpc_error__" in result:
            raise RuntimeError(result["__rpc_error__"])
        # Compatibility with an already-running server from before the RPC
        # envelope change.  get_move_status legitimately owns an "error"
        # field; other methods used the old field only for execution errors.
        if (
            isinstance(result, dict)
            and result.get("error") is not None
            and request.get("method") != "get_move_status"
        ):
            raise RuntimeError(result["error"])
        return result

    def get_joint_state(self) -> np.ndarray:
        """Get the current state of the leader robot.

        Returns:
            T: The current state of the leader robot.
        """
        request = {"method": "get_joint_state"}
        return self._request(request)

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        """Command the leader robot to the given state.

        Args:
            joint_state (T): The state to command the leader robot to.
        """
        request = {
            "method": "command_joint_state",
            "args": {"joint_state": joint_state},
        }
        return self._request(request)

    def command_gripper_state(self, position: float) -> None:
        """Command only channel 7 without sending an arm target."""
        request = {
            "method": "command_gripper_state",
            "args": {"position": float(position)},
        }
        return self._request(request)

    def move_joint_state(
        self,
        joint_state: np.ndarray,
        velocity: float,
        acceleration: float,
        jerk: float,
        timeout: float,
        max_segment: float,
    ) -> None:
        request = {
            "method": "move_joint_state",
            "args": {
                "joint_state": joint_state,
                "velocity": velocity,
                "acceleration": acceleration,
                "jerk": jerk,
                "timeout": timeout,
                "max_segment": max_segment,
            },
        }
        return self._request(request)

    def get_move_status(self) -> Dict[str, object]:
        return self._request({"method": "get_move_status"})

    def stop_motion(self) -> None:
        return self._request({"method": "stop_motion"}, timeout_ms=3000)

    def get_observations(self) -> Dict[str, np.ndarray]:
        """Get the current observations of the leader robot.

        Returns:
            Dict[str, np.ndarray]: The current observations of the leader robot.
        """
        request = {"method": "get_observations"}
        return self._request(request)

    def close(self) -> None:
        """Close the ZMQ socket and context."""
        self._socket.close(linger=0)
        self._context.term()
