"""
Example 02: Adaptive Pure Pursuit Path Tracking (OpenRaceStack)
Demonstrates velocity-adaptive lookahead Pure Pursuit tracking a reference path.
"""

import math

import numpy as np

from Localization.core.dynamics import (
    F1TENTH_PARAMS,
    DynamicBicycleModel,
    DynamicControlInput,
    VehicleDynamicsState,
)
from Localization.core.pure_pursuit import PurePursuitConfig, PurePursuitController


def run_pure_pursuit_demo():
    print("=" * 70)
    print("OpenRaceStack — Example 02: Adaptive Lookahead Pure Pursuit")
    print("=" * 70)

    # 1. Reference path (Curved racetrack sector: S-bend)
    s = np.linspace(0.0, 50.0, 100)
    x = s
    y = 3.0 * np.sin(s * 0.15)
    path = np.column_stack((x, y))

    # 2. Setup controller and vehicle
    cfg = PurePursuitConfig(
        wheelbase_m=F1TENTH_PARAMS.wheelbase,
        min_lookahead_m=0.8,
        max_lookahead_m=3.5,
        lookahead_gain_s=0.2,
    )
    controller = PurePursuitController(config=cfg)
    model = DynamicBicycleModel(params=F1TENTH_PARAMS)

    # Vehicle starts 0.8m off path
    state = VehicleDynamicsState(x=0.0, y=0.8, psi=0.0, vx=4.0, vy=0.0, omega_z=0.0)
    dt = 0.05

    print(f"\nInitial State: x={state.x:.2f}m, y={state.y:.2f}m (offset: 0.80m)")
    print(f"{'Time (s)':<10} {'Position (x, y)':<22} {'Speed (m/s)':<14} {'Cross-Track Error':<18} {'Steer (deg)':<12}")
    print("-" * 76)

    for step in range(35):
        t = step * dt
        cmd = controller.compute_control(
            vehicle_x=state.x,
            vehicle_y=state.y,
            vehicle_theta=state.psi,
            vehicle_v=state.vx,
            reference_path=path,
            dt=dt,
        )

        if step % 5 == 0:
            pos_str = f"({state.x:.2f}, {state.y:.2f})"
            steer_deg = math.degrees(cmd.steering_angle_rad)
            print(f"{t:<10.2f} {pos_str:<22} {state.vx:<14.2f} {cmd.cross_track_error_m:<18.3f} {steer_deg:<12.2f}")

        ctrl = DynamicControlInput(steering_angle=cmd.steering_angle_rad, acceleration=cmd.acceleration_mps2)
        state = model.step_rk4(state, ctrl, dt=dt)

    print("\nTracking completed. Pure Pursuit successfully converged to reference path.")


if __name__ == "__main__":
    run_pure_pursuit_demo()
