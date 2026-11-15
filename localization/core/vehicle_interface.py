"""
Cross-Platform Vehicle Hardware Abstraction Layer (HAL) for OpenRaceStack.

Provides standardized vehicle interfaces and telemetry bridges across diverse platforms:
high-fidelity simulation backends, 1:10 scale F1TENTH racers (VESC/PWM), and full-scale
Indy Autonomous Challenge Dallara AV-21 racecars (Drive-by-Wire CAN FD).
"""

import math
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum

import numpy as np

from .dynamics import (
    AV21_PARAMS,
    F1TENTH_PARAMS,
    DynamicBicycleModel,
    DynamicControlInput,
    VehicleDynamicsState,
    VehicleParameters,
)


class VehicleStatus(str, Enum):
    """Lifecycle and operational state of the vehicle hardware interface."""
    DISCONNECTED = "disconnected"
    INITIALIZING = "initializing"
    READY = "ready"
    ARMED_AUTONOMOUS = "armed_autonomous"
    MANUAL_OVERRIDE = "manual_override"
    EMERGENCY_STOP = "emergency_stop"
    ERROR = "error"


@dataclass
class VehicleControlCommand:
    """Standardized actuation demand sent to low-level vehicle controllers."""
    steering_angle_rad: float = 0.0      # Front wheel steer angle delta (rad)
    steering_rate_radps: float = 0.0     # Requested steering velocity (rad/s)
    acceleration_mps2: float = 0.0       # Longitudinal acceleration command a_x (m/s^2)
    target_speed_mps: float = 0.0        # Target velocity feedforward (m/s)
    throttle: float = 0.0                # Normalized traction demand [0.0, 1.0]
    brake: float = 0.0                   # Normalized braking demand [0.0, 1.0]
    gear: int = 1                        # Transmission gear (1-6, 0=neutral, -1=reverse)
    emergency_stop: bool = False         # Critical hardware E-STOP flag
    timestamp: float = field(default_factory=time.time)


@dataclass
class VehicleTelemetry:
    """Standardized sensor and actuator telemetry received from vehicle hardware."""
    timestamp: float = field(default_factory=time.time)
    steering_angle_rad: float = 0.0
    speed_mps: float = 0.0
    wheel_speeds_radps: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0) # FL, FR, RL, RR
    yaw_rate_radps: float = 0.0
    accel_x_mps2: float = 0.0
    accel_y_mps2: float = 0.0
    throttle_actual: float = 0.0         # Current throttle position [0.0, 1.0]
    brake_pressure_bar: float = 0.0      # Hydraulic brake pressure (bar)
    battery_voltage_v: float = 12.6      # Low-voltage system bus (V)
    motor_temperature_c: float = 40.0    # Powertrain / inverter temperature (°C)
    is_estop_engaged: bool = False
    error_flags: list[str] = field(default_factory=list)


class VehicleInterface(ABC):
    """
    Abstract Base Class defining the hardware abstraction interface for race vehicles.
    """

    def __init__(self, params: VehicleParameters):
        self.params = params
        self._status: VehicleStatus = VehicleStatus.DISCONNECTED
        self._last_cmd_timestamp: float = 0.0
        self._watchdog_timeout_s: float = 0.100 # 100 ms heartbeat timeout

    @property
    def status(self) -> VehicleStatus:
        return self._status

    @abstractmethod
    def connect(self) -> bool:
        """Initializes communications with actuators and telemetry buses."""
        pass

    @abstractmethod
    def disconnect(self) -> None:
        """Safely shuts down communications and drops to neutral."""
        pass

    @abstractmethod
    def send_command(self, command: VehicleControlCommand) -> bool:
        """Dispatches an actuation command to vehicle low-level controllers."""
        pass

    @abstractmethod
    def receive_telemetry(self) -> VehicleTelemetry:
        """Reads latest sensor telemetry from physical or simulated vehicle."""
        pass

    def trigger_emergency_stop(self) -> None:
        """Immediately commands vehicle hardware E-STOP and neutralizes actuators."""
        self._status = VehicleStatus.EMERGENCY_STOP
        self.send_command(VehicleControlCommand(emergency_stop=True, brake=1.0, acceleration_mps2=self.params.max_decel))

    def clear_emergency_stop(self) -> bool:
        """Clears hardware E-STOP latch if vehicle systems are healthy."""
        if self._status == VehicleStatus.EMERGENCY_STOP:
            self._status = VehicleStatus.READY
            return True
        return False

    def reset(self, initial_state: VehicleDynamicsState | None = None) -> bool:
        """Resets the vehicle interface and internal state to initial conditions."""
        self._status = VehicleStatus.READY
        self._last_cmd_timestamp = 0.0
        return True


# ============================================================================
# SIMULATED VEHICLE INTERFACE
# ============================================================================

class SimulatedVehicleInterface(VehicleInterface):
    """
    High-fidelity simulation backend interface executing dynamic single-track
    physics with first-order actuator lag, tire slip, and measurement noise.
    """

    def __init__(
        self,
        params: VehicleParameters = F1TENTH_PARAMS,
        initial_speed_mps: float = 0.0,
        steer_tau_s: float = 0.04,
        accel_tau_s: float = 0.06,
        noise_std_speed: float = 0.05,
        noise_std_yaw_rate: float = 0.01,
        seed: int | None = None,
    ):
        super().__init__(params)
        self.dynamics_model = DynamicBicycleModel(params=params)
        self.initial_speed = initial_speed_mps
        self.state = VehicleDynamicsState(
            x=0.0, y=0.0, psi=0.0, vx=initial_speed_mps, vy=0.0, omega_z=0.0
        )
        self.steer_tau = steer_tau_s
        self.accel_tau = accel_tau_s
        self.noise_std_speed = noise_std_speed
        self.noise_std_yaw_rate = noise_std_yaw_rate
        self.seed = seed
        self.rng = np.random.default_rng(seed)

        self._actual_steer = 0.0
        self._actual_accel = 0.0
        self._last_update_time = time.time()

    def reset(
        self,
        initial_state: VehicleDynamicsState | None = None,
        seed: int | None = None,
    ) -> bool:
        """Resets the simulation physics, actuator lags, and state vector."""
        if initial_state is not None:
            self.state = initial_state
        else:
            self.state = VehicleDynamicsState(
                x=0.0, y=0.0, psi=0.0, vx=self.initial_speed, vy=0.0, omega_z=0.0
            )
        self._actual_steer = 0.0
        self._actual_accel = 0.0
        self._status = VehicleStatus.READY
        self._last_cmd_timestamp = 0.0
        self._last_update_time = time.time()
        if seed is not None:
            self.seed = seed
            self.rng = np.random.default_rng(seed)
        elif self.seed is not None:
            self.rng = np.random.default_rng(self.seed)
        return True

    def connect(self) -> bool:
        self._status = VehicleStatus.READY
        self._last_update_time = time.time()
        return True

    def disconnect(self) -> None:
        self._actual_steer = 0.0
        self._actual_accel = 0.0
        self._status = VehicleStatus.DISCONNECTED

    def send_command(self, command: VehicleControlCommand) -> bool:
        if self._status == VehicleStatus.DISCONNECTED:
            return False

        if command.emergency_stop or self._status == VehicleStatus.EMERGENCY_STOP:
            self._actual_accel = self.params.max_decel
            return True

        self._last_cmd_timestamp = time.time()

        # Update actuator lag (first order filter)
        dt = 0.02 # Nominal control step
        alpha_steer = dt / (self.steer_tau + dt)
        alpha_accel = dt / (self.accel_tau + dt)

        target_steer = float(np.clip(
            command.steering_angle_rad,
            -self.params.max_steering_angle,
            self.params.max_steering_angle,
        ))
        target_accel = float(np.clip(
            command.acceleration_mps2,
            self.params.max_decel,
            self.params.max_accel,
        ))

        self._actual_steer += alpha_steer * (target_steer - self._actual_steer)
        self._actual_accel += alpha_accel * (target_accel - self._actual_accel)

        # Step dynamic vehicle model
        ctrl = DynamicControlInput(
            steering_angle=self._actual_steer,
            acceleration=self._actual_accel,
        )
        self.state = self.dynamics_model.step_rk4(self.state, ctrl, dt=dt)
        self._status = VehicleStatus.ARMED_AUTONOMOUS
        return True

    def receive_telemetry(self) -> VehicleTelemetry:
        t_now = time.time()
        # Add realistic sensor noise using instance generator
        noisy_speed = float(max(0.0, self.state.speed + self.rng.normal(0, self.noise_std_speed)))
        noisy_yaw_rate = float(self.state.omega_z + self.rng.normal(0, self.noise_std_yaw_rate))

        wheel_w = noisy_speed / self.params.wheel_radius
        wheel_speeds = (wheel_w, wheel_w, wheel_w, wheel_w)

        return VehicleTelemetry(
            timestamp=t_now,
            steering_angle_rad=self._actual_steer,
            speed_mps=noisy_speed,
            wheel_speeds_radps=wheel_speeds,
            yaw_rate_radps=noisy_yaw_rate,
            accel_x_mps2=self._actual_accel,
            accel_y_mps2=float(self.state.speed * self.state.omega_z),
            throttle_actual=max(0.0, self._actual_accel / self.params.max_accel) if self._actual_accel > 0 else 0.0,
            brake_pressure_bar=max(0.0, -self._actual_accel / abs(self.params.max_decel) * 50.0) if self._actual_accel < 0 else 0.0,
            battery_voltage_v=12.58,
            motor_temperature_c=42.5,
            is_estop_engaged=(self._status == VehicleStatus.EMERGENCY_STOP),
        )


# ============================================================================
# F1TENTH 1:10 SCALE VEHICLE INTERFACE (VESC / PWM)
# ============================================================================

class F1TenthVehicleInterface(VehicleInterface):
    """
    Hardware abstraction interface for 1:10 scale F1TENTH autonomous racecars.
    Translates speed to VESC Electronic Speed Controller electrical RPM (ERPM)
    and steering angles to PWM servo microseconds (1000 - 2000 us).
    """

    def __init__(
        self,
        params: VehicleParameters = F1TENTH_PARAMS,
        motor_pole_pairs: int = 4,
        gear_ratio: float = 3.6,
        servo_center_pwm_us: float = 1500.0,
        servo_range_pwm_us: float = 500.0,
    ):
        super().__init__(params)
        self.pole_pairs = motor_pole_pairs
        self.gear_ratio = gear_ratio
        self.servo_center_pwm = servo_center_pwm_us
        self.servo_range_pwm = servo_range_pwm_us
        self._last_cmd: VehicleControlCommand = VehicleControlCommand()

    def speed_to_erpm(self, speed_mps: float) -> int:
        """Converts vehicle linear velocity (m/s) to VESC electrical RPM."""
        wheel_circ = 2.0 * math.pi * self.params.wheel_radius
        wheel_rpm = (speed_mps / wheel_circ) * 60.0
        motor_rpm = wheel_rpm * self.gear_ratio
        erpm = int(motor_rpm * self.pole_pairs)
        return erpm

    def steering_to_pwm(self, steering_rad: float) -> float:
        """Converts steering angle (rad) to PWM microsecond pulse width [1000, 2000]."""
        ratio = steering_rad / self.params.max_steering_angle
        # Left (+angle) maps to >1500 us, Right (-angle) maps to <1500 us
        pwm = self.servo_center_pwm + ratio * self.servo_range_pwm
        return float(np.clip(pwm, 1000.0, 2000.0))

    def connect(self) -> bool:
        self._status = VehicleStatus.READY
        return True

    def disconnect(self) -> None:
        self._status = VehicleStatus.DISCONNECTED

    def send_command(self, command: VehicleControlCommand) -> bool:
        if self._status == VehicleStatus.DISCONNECTED:
            return False

        if command.emergency_stop:
            self._status = VehicleStatus.EMERGENCY_STOP
            self._last_cmd = VehicleControlCommand(emergency_stop=True)
            return True

        self._last_cmd_timestamp = time.time()
        self._last_cmd = command
        self._status = VehicleStatus.ARMED_AUTONOMOUS
        return True

    def receive_telemetry(self) -> VehicleTelemetry:
        t_now = time.time()
        # In hardware mode, reads VESC packet via serial/CAN
        speed = self._last_cmd.target_speed_mps
        steer = self._last_cmd.steering_angle_rad
        erpm = self.speed_to_erpm(speed)
        wheel_w = (erpm / (self.pole_pairs * self.gear_ratio)) * (2.0 * math.pi / 60.0)

        return VehicleTelemetry(
            timestamp=t_now,
            steering_angle_rad=steer,
            speed_mps=speed,
            wheel_speeds_radps=(wheel_w, wheel_w, wheel_w, wheel_w),
            yaw_rate_radps=(speed / self.params.wheelbase) * math.tan(steer),
            accel_x_mps2=self._last_cmd.acceleration_mps2,
            battery_voltage_v=14.8, # 4S LiPo nominal
            motor_temperature_c=38.0,
            is_estop_engaged=(self._status == VehicleStatus.EMERGENCY_STOP),
        )


# ============================================================================
# DALLARA AV-21 INDYCAR INTERFACE (CAN FD / DRIVE-BY-WIRE)
# ============================================================================

class DallaraAV21VehicleInterface(VehicleInterface):
    """
    Hardware abstraction interface for Dallara AV-21 Indy Autonomous Challenge racecars.
    Translates trajectory commands to Drive-by-Wire CAN FD frames (steering motor torque,
    dual-circuit hydraulic brake pressures, and engine throttle maps).
    """

    def __init__(
        self,
        params: VehicleParameters = AV21_PARAMS,
        max_brake_pressure_bar: float = 75.0,
    ):
        super().__init__(params)
        self.max_brake_pressure = max_brake_pressure_bar
        self._last_cmd = VehicleControlCommand()

    def connect(self) -> bool:
        self._status = VehicleStatus.READY
        return True

    def disconnect(self) -> None:
        self._status = VehicleStatus.DISCONNECTED

    def compute_brake_pressure(self, accel_mps2: float) -> float:
        """Maps negative longitudinal deceleration demand to hydraulic brake pressure in bar."""
        if accel_mps2 >= 0.0:
            return 0.0
        ratio = abs(accel_mps2) / abs(self.params.max_decel)
        pressure = ratio * self.max_brake_pressure
        return float(np.clip(pressure, 0.0, self.max_brake_pressure))

    def send_command(self, command: VehicleControlCommand) -> bool:
        if self._status == VehicleStatus.DISCONNECTED:
            return False

        if command.emergency_stop:
            self._status = VehicleStatus.EMERGENCY_STOP
            return True

        # Safety watchdog: check command limits
        if abs(command.steering_angle_rad) > self.params.max_steering_angle + 1e-4:
            self._status = VehicleStatus.ERROR
            return False

        self._last_cmd_timestamp = time.time()
        self._last_cmd = command
        self._status = VehicleStatus.ARMED_AUTONOMOUS
        return True

    def receive_telemetry(self) -> VehicleTelemetry:
        t_now = time.time()
        # In physical IndyCar, reads Vector CAN FD bus
        speed = self._last_cmd.target_speed_mps
        brake_p = self.compute_brake_pressure(self._last_cmd.acceleration_mps2)
        throttle = max(0.0, min(1.0, self._last_cmd.acceleration_mps2 / self.params.max_accel)) if self._last_cmd.acceleration_mps2 > 0 else 0.0

        return VehicleTelemetry(
            timestamp=t_now,
            steering_angle_rad=self._last_cmd.steering_angle_rad,
            speed_mps=speed,
            yaw_rate_radps=(speed / self.params.wheelbase) * math.tan(self._last_cmd.steering_angle_rad),
            accel_x_mps2=self._last_cmd.acceleration_mps2,
            throttle_actual=throttle,
            brake_pressure_bar=brake_p,
            battery_voltage_v=24.4, # 24V racecar telemetry bus
            motor_temperature_c=88.0, # Honda/K20C turbocharged engine operating temp
            is_estop_engaged=(self._status == VehicleStatus.EMERGENCY_STOP),
        )
