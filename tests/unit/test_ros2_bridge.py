"""
Unit Tests for ROS 2 Transport Bridge, Message Converters, and Watchdog Safety Preemption.
"""

import math
import time

from Localization.core.dynamics import VehicleDynamicsState
from Localization.core.ros2_bridge import (
    RaceControlFlag,
    ROS2Bridge,
    ROS2BridgeConfig,
    ROS2MessageConverter,
)
from Localization.core.vehicle_interface import VehicleControlCommand, VehicleTelemetry


def test_ros2_odometry_message_converter():
    """Verify conversion of VehicleDynamicsState to standard nav_msgs/msg/Odometry schema."""
    state = VehicleDynamicsState(x=12.5, y=-4.2, psi=math.pi / 4, vx=15.0, vy=-0.2, omega_z=0.08)

    msg = ROS2MessageConverter.state_to_odometry_msg(state, frame_id="map", child_frame_id="base_link")

    # Header check
    assert msg["header"]["frame_id"] == "map"
    assert msg["child_frame_id"] == "base_link"

    # Pose position
    pos = msg["pose"]["pose"]["position"]
    assert pos["x"] == 12.5
    assert pos["y"] == -4.2
    assert pos["z"] == 0.0

    # Pose orientation quaternion for psi = pi/4 -> qz = sin(pi/8), qw = cos(pi/8)
    rot = msg["pose"]["pose"]["orientation"]
    assert math.isclose(rot["z"], math.sin(math.pi / 8), abs_tol=1e-4)
    assert math.isclose(rot["w"], math.cos(math.pi / 8), abs_tol=1e-4)

    # 6x6 covariance
    assert len(msg["pose"]["covariance"]) == 36
    assert len(msg["twist"]["covariance"]) == 36

    # Twist
    twist = msg["twist"]["twist"]
    assert twist["linear"]["x"] == 15.0
    assert twist["linear"]["y"] == -0.2
    assert twist["angular"]["z"] == 0.08


def test_ros2_imu_message_converter():
    """Verify conversion of VehicleTelemetry to standard sensor_msgs/msg/Imu schema."""
    telem = VehicleTelemetry(yaw_rate_radps=0.15, accel_x_mps2=2.4, accel_y_mps2=-1.2)

    msg = ROS2MessageConverter.telemetry_to_imu_msg(telem, frame_id="imu_link")

    assert msg["header"]["frame_id"] == "imu_link"
    assert msg["angular_velocity"]["z"] == 0.15
    assert msg["linear_acceleration"]["x"] == 2.4
    assert msg["linear_acceleration"]["y"] == -1.2
    assert math.isclose(msg["linear_acceleration"]["z"], 9.80665, abs_tol=1e-3)


def test_ros2_ackermann_round_trip_conversion():
    """Verify lossless conversion between VehicleControlCommand and ackermann message."""
    cmd_orig = VehicleControlCommand(
        steering_angle_rad=0.25,
        steering_rate_radps=1.2,
        target_speed_mps=18.5,
        acceleration_mps2=3.0,
    )

    # To ROS 2 dictionary
    msg = ROS2MessageConverter.control_to_ackermann_msg(cmd_orig)
    assert msg["drive"]["steering_angle"] == 0.25
    assert msg["drive"]["speed"] == 18.5

    # Back to VehicleControlCommand
    cmd_parsed = ROS2MessageConverter.ackermann_msg_to_control(msg)
    assert cmd_parsed.steering_angle_rad == cmd_orig.steering_angle_rad
    assert cmd_parsed.steering_rate_radps == cmd_orig.steering_rate_radps
    assert cmd_parsed.target_speed_mps == cmd_orig.target_speed_mps
    assert cmd_parsed.acceleration_mps2 == cmd_orig.acceleration_mps2


def test_ros2_tf_message_converter():
    """Verify conversion of state into tf2_msgs/msg/TFMessage schema."""
    state = VehicleDynamicsState(x=5.0, y=10.0, psi=0.0, vx=0.0, vy=0.0, omega_z=0.0)
    tf_msg = ROS2MessageConverter.state_to_tf_msg(state, parent_frame="map", child_frame="base_link")

    assert len(tf_msg["transforms"]) == 1
    tf = tf_msg["transforms"][0]
    assert tf["child_frame_id"] == "base_link"
    assert tf["transform"]["translation"]["x"] == 5.0
    assert tf["transform"]["translation"]["y"] == 10.0


def test_race_flag_preemption_and_speed_capping():
    """Verify ROS 2 bridge enforces Race Control flag preemption rules."""
    bridge = ROS2Bridge(ROS2BridgeConfig(yellow_flag_speed_mps=10.0))

    received_cmds = []
    bridge.register_drive_callback(lambda cmd: received_cmds.append(cmd))

    # 1. Under GREEN flag: high-speed command passes unmodified
    bridge.set_race_flag(RaceControlFlag.GREEN)
    raw_green = {"drive": {"speed": 25.0, "steering_angle": 0.05, "acceleration": 2.0}}
    cmd_green = bridge.receive_incoming_drive_msg(raw_green)
    assert cmd_green.target_speed_mps == 25.0
    assert not cmd_green.emergency_stop

    # 2. Under YELLOW flag: speed is capped to 10.0 m/s
    bridge.set_race_flag(RaceControlFlag.YELLOW)
    raw_yellow = {"drive": {"speed": 25.0, "steering_angle": 0.05, "acceleration": 2.0}}
    cmd_yellow = bridge.receive_incoming_drive_msg(raw_yellow)
    assert cmd_yellow.target_speed_mps == 10.0

    # 3. Under RED flag: speed is 0.0, braking applied, emergency stop flagged
    bridge.set_race_flag(RaceControlFlag.RED)
    raw_red = {"drive": {"speed": 15.0, "steering_angle": 0.0, "acceleration": 1.0}}
    cmd_red = bridge.receive_incoming_drive_msg(raw_red)
    assert cmd_red.target_speed_mps == 0.0
    assert cmd_red.emergency_stop
    assert cmd_red.acceleration_mps2 < 0.0


def test_ros2_watchdog_timeout():
    """Verify watchdog detects missing control heartbeats and triggers fail-safe."""
    bridge = ROS2Bridge(ROS2BridgeConfig(watchdog_timeout_ms=50.0))

    estop_events = []
    bridge.register_drive_callback(lambda cmd: estop_events.append(cmd) if cmd.emergency_stop else None)

    # Fresh message received
    bridge.receive_incoming_drive_msg({"drive": {"speed": 10.0}})
    assert bridge.check_watchdog()

    # Sleep longer than watchdog timeout (60 ms > 50 ms)
    time.sleep(0.065)

    # Watchdog check should fail and dispatch emergency stop command
    is_alive = bridge.check_watchdog()
    assert not is_alive
    assert len(estop_events) > 0
    assert estop_events[-1].emergency_stop
