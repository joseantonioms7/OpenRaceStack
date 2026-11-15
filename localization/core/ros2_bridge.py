"""
ROS 2 Transport and Cross-Platform Protocol Bridge for OpenRaceStack.

Provides standardized message schemas and converters conforming to ROS 2 standard topics:
nav_msgs/msg/Odometry, sensor_msgs/msg/Imu, tf2_msgs/msg/TFMessage, and
ackermann_msgs/msg/AckermannDriveStamped. Features Race Control flag state machines,
ASIL-D watchdog heartbeat monitoring, and zero-dependency standalone fallback.
"""

import math
import time
from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum
from typing import Any

from .dynamics import VehicleDynamicsState
from .vehicle_interface import VehicleControlCommand, VehicleTelemetry


class RaceControlFlag(str, Enum):
    """Standard FIA / Autonomous Racing competition flag states."""
    GREEN = "green"           # Full course green, nominal racing allowed
    YELLOW = "yellow"         # Caution / localized hazard, speed restricted, no overtaking
    PURPLE = "purple"         # Code 60 / Pit speed limiter active
    RED = "red"               # Session stopped immediately, controlled emergency stop
    BLACK = "black"           # Disqualification / immediate pit return
    CHECKERED = "checkered"   # Session finished, in-lap cool-down


@dataclass
class ROS2Header:
    """ROS 2 std_msgs/msg/Header equivalent schema."""
    stamp_sec: int
    stamp_nanosec: int
    frame_id: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "stamp": {"sec": self.stamp_sec, "nanosec": self.stamp_nanosec},
            "frame_id": self.frame_id,
        }

    @classmethod
    def now(cls, frame_id: str = "base_link") -> "ROS2Header":
        now = time.time()
        sec = int(now)
        nanosec = int((now - sec) * 1e9)
        return cls(stamp_sec=sec, stamp_nanosec=nanosec, frame_id=frame_id)


class ROS2MessageConverter:
    """
    Translates OpenRaceStack dynamics, telemetry, and control states to standard
    ROS 2 message payload dictionaries matching official IDL schemas.
    """

    @staticmethod
    def state_to_odometry_msg(
        state: VehicleDynamicsState,
        frame_id: str = "map",
        child_frame_id: str = "base_link",
        cov_pos: float = 0.05,
        cov_vel: float = 0.02,
    ) -> dict[str, Any]:
        """Converts VehicleDynamicsState into nav_msgs/msg/Odometry schema."""
        header = ROS2Header.now(frame_id=frame_id).to_dict()

        # Quaternion from 2D yaw angle psi
        half_yaw = state.psi * 0.5
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)

        # 6x6 Pose Covariance
        pose_cov = [0.0] * 36
        pose_cov[0] = cov_pos**2      # X
        pose_cov[7] = cov_pos**2      # Y
        pose_cov[35] = 0.01**2        # Yaw

        # 6x6 Twist Covariance
        twist_cov = [0.0] * 36
        twist_cov[0] = cov_vel**2     # Vx
        twist_cov[7] = cov_vel**2     # Vy
        twist_cov[35] = 0.02**2       # Wz

        return {
            "header": header,
            "child_frame_id": child_frame_id,
            "pose": {
                "pose": {
                    "position": {"x": state.x, "y": state.y, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": qz, "w": qw},
                },
                "covariance": pose_cov,
            },
            "twist": {
                "twist": {
                    "linear": {"x": state.vx, "y": state.vy, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": state.omega_z},
                },
                "covariance": twist_cov,
            },
        }

    @staticmethod
    def telemetry_to_imu_msg(
        telemetry: VehicleTelemetry,
        frame_id: str = "imu_link",
    ) -> dict[str, Any]:
        """Converts VehicleTelemetry into sensor_msgs/msg/Imu schema."""
        header = ROS2Header.now(frame_id=frame_id).to_dict()

        imu_cov = [0.0] * 9
        imu_cov[0] = 0.01
        imu_cov[4] = 0.01
        imu_cov[8] = 0.01

        return {
            "header": header,
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            "orientation_covariance": [-1.0] + [0.0] * 8, # -1 indicates unknown absolute orientation
            "angular_velocity": {"x": 0.0, "y": 0.0, "z": telemetry.yaw_rate_radps},
            "angular_velocity_covariance": imu_cov,
            "linear_acceleration": {"x": telemetry.accel_x_mps2, "y": telemetry.accel_y_mps2, "z": 9.80665},
            "linear_acceleration_covariance": imu_cov,
        }

    @staticmethod
    def control_to_ackermann_msg(
        cmd: VehicleControlCommand,
        frame_id: str = "base_link",
    ) -> dict[str, Any]:
        """Converts VehicleControlCommand into ackermann_msgs/msg/AckermannDriveStamped schema."""
        header = ROS2Header.now(frame_id=frame_id).to_dict()
        return {
            "header": header,
            "drive": {
                "steering_angle": cmd.steering_angle_rad,
                "steering_angle_velocity": cmd.steering_rate_radps,
                "speed": cmd.target_speed_mps,
                "acceleration": cmd.acceleration_mps2,
                "jerk": 0.0,
            },
        }

    @staticmethod
    def ackermann_msg_to_control(msg: dict[str, Any]) -> VehicleControlCommand:
        """Parses an ackermann_msgs/msg/AckermannDriveStamped dictionary into VehicleControlCommand."""
        drive = msg.get("drive", {})
        return VehicleControlCommand(
            steering_angle_rad=float(drive.get("steering_angle", 0.0)),
            steering_rate_radps=float(drive.get("steering_angle_velocity", 0.0)),
            target_speed_mps=float(drive.get("speed", 0.0)),
            acceleration_mps2=float(drive.get("acceleration", 0.0)),
        )

    @staticmethod
    def state_to_tf_msg(
        state: VehicleDynamicsState,
        parent_frame: str = "map",
        child_frame: str = "base_link",
    ) -> dict[str, Any]:
        """Converts state into tf2_msgs/msg/TFMessage with TransformStamped schema."""
        header = ROS2Header.now(frame_id=parent_frame).to_dict()
        half_yaw = state.psi * 0.5
        return {
            "transforms": [
                {
                    "header": header,
                    "child_frame_id": child_frame,
                    "transform": {
                        "translation": {"x": state.x, "y": state.y, "z": 0.0},
                        "rotation": {
                            "x": 0.0,
                            "y": 0.0,
                            "z": math.sin(half_yaw),
                            "w": math.cos(half_yaw),
                        },
                    },
                }
            ]
        }


@dataclass
class ROS2BridgeConfig:
    """Configuration for ROS 2 pub/sub transport and race safety watchdog."""
    odom_topic: str = "/odom"
    imu_topic: str = "/imu/data"
    drive_topic: str = "/drive"
    flag_topic: str = "/race_control/flag"
    watchdog_timeout_ms: float = 100.0   # Preemption if no command received within timeout
    yellow_flag_speed_mps: float = 12.0  # Speed cap under caution yellow flag
    enable_strict_safety: bool = True    # Enforce flag overrides and watchdog cuts


class ROS2Bridge:
    """
    Cross-platform ROS 2 Bridge managing topic message translations,
    Race Control flag state machine, and fail-safe safety preemption.
    """

    def __init__(self, config: ROS2BridgeConfig | None = None):
        self.config = config or ROS2BridgeConfig()
        self.current_flag = RaceControlFlag.GREEN
        self.last_drive_command: VehicleControlCommand = VehicleControlCommand()
        self.last_drive_timestamp: float = time.time()
        self.is_heartbeat_alive: bool = True

        # Callbacks
        self._drive_listeners: list[Callable[[VehicleControlCommand], None]] = []
        self._flag_listeners: list[Callable[[RaceControlFlag], None]] = []

    def set_race_flag(self, flag: RaceControlFlag | str) -> None:
        """Updates competition race control flag and notifies listeners."""
        new_flag = RaceControlFlag(flag) if isinstance(flag, str) else flag
        self.current_flag = new_flag
        for listener in self._flag_listeners:
            listener(new_flag)

    def register_drive_callback(self, cb: Callable[[VehicleControlCommand], None]) -> None:
        """Registers a callback to receive verified, flag-filtered drive commands."""
        self._drive_listeners.append(cb)

    def register_flag_callback(self, cb: Callable[[RaceControlFlag], None]) -> None:
        """Registers a callback for race control flag changes."""
        self._flag_listeners.append(cb)

    def receive_incoming_drive_msg(self, msg: dict[str, Any]) -> VehicleControlCommand:
        """
        Receives raw incoming ROS 2 ackermann message, checks watchdog timeout,
        applies race control flag overrides, and dispatches to listeners.
        """
        t_now = time.time()
        cmd = ROS2MessageConverter.ackermann_msg_to_control(msg)
        self.last_drive_timestamp = t_now
        self.is_heartbeat_alive = True

        # Safety filter based on Race Control flag
        if self.config.enable_strict_safety:
            if self.current_flag == RaceControlFlag.RED:
                # Immediate controlled emergency stop
                cmd.target_speed_mps = 0.0
                cmd.acceleration_mps2 = -8.0
                cmd.emergency_stop = True

            elif self.current_flag in (RaceControlFlag.YELLOW, RaceControlFlag.PURPLE):
                # Clamp speed to caution maximum
                if cmd.target_speed_mps > self.config.yellow_flag_speed_mps:
                    cmd.target_speed_mps = self.config.yellow_flag_speed_mps

        self.last_drive_command = cmd

        for listener in self._drive_listeners:
            listener(cmd)

        return cmd

    def check_watchdog(self) -> bool:
        """
        Evaluates communication heartbeat. If command latency exceeds watchdog_timeout_ms,
        triggers emergency stop and sets heartbeat dead flag.
        """
        elapsed_ms = (time.time() - self.last_drive_timestamp) * 1000.0
        if elapsed_ms > self.config.watchdog_timeout_ms:
            self.is_heartbeat_alive = False
            # Emergency fallback command
            estop_cmd = VehicleControlCommand(
                emergency_stop=True,
                target_speed_mps=0.0,
                acceleration_mps2=-8.0,
            )
            for listener in self._drive_listeners:
                listener(estop_cmd)
            return False
        return True
