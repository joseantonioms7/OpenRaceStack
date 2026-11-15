"""
Unit Tests for Vehicle Hardware Abstraction Layer (HAL) and Platform Interfaces.
"""

import math

import numpy as np

from Localization.core.dynamics import AV21_PARAMS, F1TENTH_PARAMS
from Localization.core.vehicle_interface import (
    DallaraAV21VehicleInterface,
    F1TenthVehicleInterface,
    SimulatedVehicleInterface,
    VehicleControlCommand,
    VehicleStatus,
)


def test_simulated_vehicle_lifecycle():
    """Verify SimulatedVehicleInterface lifecycle transitions and actuator lag."""
    sim = SimulatedVehicleInterface(params=F1TENTH_PARAMS, initial_speed_mps=5.0)

    # Initial state is DISCONNECTED
    assert sim.status == VehicleStatus.DISCONNECTED

    # Connect
    assert sim.connect()
    assert sim.status == VehicleStatus.READY

    # Send command
    cmd = VehicleControlCommand(steering_angle_rad=0.10, acceleration_mps2=2.0, target_speed_mps=8.0)
    assert sim.send_command(cmd)
    assert sim.status == VehicleStatus.ARMED_AUTONOMOUS

    # Read telemetry
    telemetry = sim.receive_telemetry()
    assert telemetry.speed_mps >= 4.0
    assert len(telemetry.wheel_speeds_radps) == 4
    assert not telemetry.is_estop_engaged

    # Trigger E-STOP
    sim.trigger_emergency_stop()
    assert sim.status == VehicleStatus.EMERGENCY_STOP
    telem_estop = sim.receive_telemetry()
    assert telem_estop.is_estop_engaged

    # Clear E-STOP
    assert sim.clear_emergency_stop()
    assert sim.status == VehicleStatus.READY

    # Disconnect
    sim.disconnect()
    assert sim.status == VehicleStatus.DISCONNECTED


def test_f1tenth_speed_and_pwm_conversions():
    """Verify F1TENTH ERPM speed conversion and servo PWM microsecond scaling."""
    f1 = F1TenthVehicleInterface(params=F1TENTH_PARAMS)
    f1.connect()

    # 1. Zero speed -> 0 ERPM
    assert f1.speed_to_erpm(0.0) == 0

    # 2. Linear speed 5 m/s -> positive ERPM
    erpm_5 = f1.speed_to_erpm(5.0)
    assert erpm_5 > 2000

    # 3. Neutral steering (0 rad) -> 1500 us PWM
    pwm_center = f1.steering_to_pwm(0.0)
    assert math.isclose(pwm_center, 1500.0, abs_tol=1e-3)

    # 4. Maximum left steer (+24 deg) -> 2000 us PWM
    pwm_left = f1.steering_to_pwm(F1TENTH_PARAMS.max_steering_angle)
    assert math.isclose(pwm_left, 2000.0, abs_tol=1e-3)

    # 5. Maximum right steer (-24 deg) -> 1000 us PWM
    pwm_right = f1.steering_to_pwm(-F1TENTH_PARAMS.max_steering_angle)
    assert math.isclose(pwm_right, 1000.0, abs_tol=1e-3)

    # 6. Over-range angle clamped to [1000, 2000] us
    assert f1.steering_to_pwm(1.0) == 2000.0
    assert f1.steering_to_pwm(-1.0) == 1000.0


def test_dallara_av21_can_interface():
    """Verify Dallara AV-21 IndyCar Drive-by-Wire brake pressure and error checking."""
    av21 = DallaraAV21VehicleInterface(params=AV21_PARAMS, max_brake_pressure_bar=75.0)
    av21.connect()

    # Acceleration > 0 -> 0 bar brake pressure
    assert av21.compute_brake_pressure(3.0) == 0.0

    # Full braking (max decel) -> 75 bar brake pressure
    p_full = av21.compute_brake_pressure(AV21_PARAMS.max_decel)
    assert math.isclose(p_full, 75.0, abs_tol=1e-3)

    # Half braking -> ~37.5 bar
    p_half = av21.compute_brake_pressure(0.5 * AV21_PARAMS.max_decel)
    assert math.isclose(p_half, 37.5, abs_tol=1.0)

    # Out-of-bounds steering angle command triggers ERROR status
    cmd_illegal = VehicleControlCommand(steering_angle_rad=np.radians(45.0)) # AV-21 limit is ~15 deg
    assert not av21.send_command(cmd_illegal)
    assert av21.status == VehicleStatus.ERROR


def test_simulated_vehicle_reset_and_determinism():
    """Verify SimulatedVehicleInterface reset behavior and seeded noise determinism."""
    # 1. Determinism with identical seeds
    sim1 = SimulatedVehicleInterface(params=F1TENTH_PARAMS, initial_speed_mps=5.0, seed=42)
    sim2 = SimulatedVehicleInterface(params=F1TENTH_PARAMS, initial_speed_mps=5.0, seed=42)

    sim1.connect()
    sim2.connect()

    t1 = sim1.receive_telemetry()
    t2 = sim2.receive_telemetry()

    assert math.isclose(t1.speed_mps, t2.speed_mps, rel_tol=1e-9)
    assert math.isclose(t1.yaw_rate_radps, t2.yaw_rate_radps, rel_tol=1e-9)

    # 2. Reset restores state and status
    sim1.send_command(VehicleControlCommand(steering_angle_rad=0.2, acceleration_mps2=3.0))
    assert sim1.status == VehicleStatus.ARMED_AUTONOMOUS
    assert sim1.state.vx > 5.0

    # Reset
    assert sim1.reset(seed=42)
    assert sim1.status == VehicleStatus.READY
    assert sim1.state.vx == 5.0
    assert sim1.state.x == 0.0
    assert sim1.state.y == 0.0
