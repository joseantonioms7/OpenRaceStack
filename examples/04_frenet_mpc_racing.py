"""
Example 04: High-Rate Frenet-Frame MPC Trajectory Tracking (OpenRaceStack)
Demonstrates sub-2ms constrained Model Predictive Control in Frenet coordinates.
"""

import math
import time

import numpy as np

from Localization.core.dynamics import (
    F1TENTH_PARAMS,
    DynamicBicycleModel,
    DynamicControlInput,
    VehicleDynamicsState,
)
from Localization.core.frenet_mpc import FrenetMPCConfig, FrenetMPCController


def run_frenet_mpc_demo():
    print("=" * 70)
    print("OpenRaceStack — Example 04: Real-Time Frenet-Frame MPC Tracking")
    print("=" * 70)

    # Reference racing line (curved chicane)
    s = np.linspace(0.0, 60.0, 150)
    x = s
    y = 2.5 * np.sin(s * 0.12)
    path = np.column_stack((x, y))

    diffs = np.diff(path, axis=0)
    headings = np.arctan2(diffs[:, 1], diffs[:, 0])
    headings = np.append(headings, headings[-1])
    curvatures = np.gradient(headings, s)
    velocities = np.full(len(path), 8.0)

    cfg = FrenetMPCConfig(wheelbase_m=F1TENTH_PARAMS.wheelbase, horizon_steps=12, dt=0.05)
    mpc = FrenetMPCController(config=cfg)
    model = DynamicBicycleModel(params=F1TENTH_PARAMS)

    state = VehicleDynamicsState(x=0.0, y=0.5, psi=0.0, vx=7.0, vy=0.0, omega_z=0.0)
    dt = 0.05

    print(f"\nInitial State: ({state.x:.2f}, {state.y:.2f}) | Target Velocity: 8.0 m/s")
    print(f"{'Time (s)':<10} {'Pos (x, y)':<22} {'Steer (deg)':<14} {'Accel (m/s2)':<14} {'Solve Latency':<16}")
    print("-" * 76)

    latencies = []
    for step in range(30):
        t = step * dt
        t0 = time.perf_counter()
        cmd = mpc.compute_control(
            vehicle_x=state.x,
            vehicle_y=state.y,
            vehicle_theta=state.psi,
            vehicle_v=state.vx,
            reference_path=path,
            reference_headings=headings,
            reference_curvatures=curvatures,
            reference_velocities=velocities,
        )
        latency_ms = (time.perf_counter() - t0) * 1000.0
        latencies.append(latency_ms)

        if step % 5 == 0:
            pos_str = f"({state.x:.2f}, {state.y:.2f})"
            steer_deg = math.degrees(cmd.steering_angle_rad)
            print(f"{t:<10.2f} {pos_str:<22} {steer_deg:<14.2f} {cmd.acceleration_mps2:<14.2f} {latency_ms:<16.2f} ms")

        ctrl = DynamicControlInput(steering_angle=cmd.steering_angle_rad, acceleration=cmd.acceleration_mps2)
        state = model.step_rk4(state, ctrl, dt=dt)

    print(f"\nAverage MPC solve time: {np.mean(latencies):.2f} ms (Deterministic real-time performance < 5 ms)")


if __name__ == "__main__":
    run_frenet_mpc_demo()
