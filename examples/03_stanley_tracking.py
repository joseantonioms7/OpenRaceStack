"""
Example 03: Stanley (Stanford) Trajectory Tracking (OpenRaceStack)
Demonstrates the non-linear Stanley front-axle tracking controller.
"""

import math

import numpy as np

from Localization.core.dynamics import (
    F1TENTH_PARAMS,
    DynamicControlInput,
    KinematicBicycleModel,
    KinematicVehicleState,
)
from Localization.core.stanley import StanleyConfig, StanleyController


def run_stanley_demo():
    print("=" * 70)
    print("OpenRaceStack — Example 03: Stanley Front-Axle Tracking Controller")
    print("=" * 70)

    # Reference straight path along X axis
    x = np.linspace(0.0, 60.0, 120)
    y = np.zeros_like(x)
    path = np.column_stack((x, y))

    config = StanleyConfig(k_e=1.4, k_soft=0.8, k_yaw=1.0)
    controller = StanleyController(config=config, wheelbase_m=F1TENTH_PARAMS.wheelbase)
    model = KinematicBicycleModel(params=F1TENTH_PARAMS)

    # Vehicle starts 1.2 meters to the right (y = -1.2)
    state = KinematicVehicleState(x=0.0, y=-1.2, psi=0.0, v=4.5)
    dt = 0.05

    print(f"\nInitial Offset: y={state.y:.2f}m, Heading Error: {math.degrees(state.psi):.1f} deg")
    print(f"{'Time (s)':<10} {'Position (x, y)':<22} {'Cross-Track (m)':<18} {'Heading Err (deg)':<18} {'Steer (deg)':<12}")
    print("-" * 80)

    for step in range(35):
        t = step * dt
        cmd = controller.compute_control(
            vehicle_x=state.x,
            vehicle_y=state.y,
            vehicle_theta=state.psi,
            vehicle_v=state.v,
            reference_path=path,
            dt=dt,
        )

        if step % 5 == 0:
            pos_str = f"({state.x:.2f}, {state.y:.2f})"
            steer_deg = math.degrees(cmd.steering_angle_rad)
            h_err_deg = math.degrees(cmd.heading_error_rad)
            print(f"{t:<10.2f} {pos_str:<22} {cmd.cross_track_error_m:<18.3f} {h_err_deg:<18.2f} {steer_deg:<12.2f}")

        ctrl = DynamicControlInput(steering_angle=cmd.steering_angle_rad, acceleration=cmd.acceleration_mps2)
        state = model.step_rk4(state, ctrl, dt=dt)

    print("\nTracking completed. Stanley controller asymptotically restored zero lateral offset.")


if __name__ == "__main__":
    run_stanley_demo()
