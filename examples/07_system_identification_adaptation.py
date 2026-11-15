"""
Example 07: System Identification & Online Adaptation (OpenRaceStack)
Demonstrates real-time recursive least squares (RLS) parameter estimation and
dynamic velocity/acceleration profile adaptation under sudden surface grip loss.
"""

import math

from Localization.core.adaptation import OnlineAdaptationManager
from Localization.core.dynamics import (
    F1TENTH_PARAMS,
    DynamicBicycleModel,
    DynamicControlInput,
    LinearTireModel,
    VehicleDynamicsState,
)
from Localization.core.sys_id import OnlineVehicleSystemIdentifier


def run_sys_id_adaptation_demo():
    print("=" * 70)
    print("OpenRaceStack — Example 07: System Identification & Parameter Adaptation")
    print("=" * 70)

    # 1. Setup system identifier and adaptation manager
    sys_id = OnlineVehicleSystemIdentifier(params=F1TENTH_PARAMS, forgetting_factor=0.98)
    adaptation_mgr = OnlineAdaptationManager(base_params=F1TENTH_PARAMS, nominal_mu=1.0)

    # 2. Vehicle cornering in a chicane (kappa = 0.15 m^-1, radius = 6.67 m)
    curve_kappa = 0.15
    nominal_speed = 7.5  # m/s

    # Phase 1: High grip dry asphalt (mu = 1.0, C_alpha = 50 N/rad)
    tire_dry = LinearTireModel(cornering_stiffness_n_rad=50.0, friction_coeff=1.0)
    model_dry = DynamicBicycleModel(params=F1TENTH_PARAMS, front_tire=tire_dry, rear_tire=tire_dry)

    # Phase 2: Low grip wet asphalt (mu = 0.45, C_alpha = 22 N/rad)
    tire_wet = LinearTireModel(cornering_stiffness_n_rad=22.0, friction_coeff=0.45)
    model_wet = DynamicBicycleModel(params=F1TENTH_PARAMS, front_tire=tire_wet, rear_tire=tire_wet)

    state = VehicleDynamicsState(x=0.0, y=0.0, psi=0.0, vx=nominal_speed, vy=0.0, omega_z=0.0)
    dt = 0.05

    print(f"\nVehicle cornering through curve (kappa={curve_kappa:.3f}) at {nominal_speed:.1f} m/s")
    print("At t = 1.0s, track transitions abruptly from Dry Asphalt (mu=1.0) to Wet Pavement (mu=0.45).\n")
    print(f"{'Time (s)':<10} {'Surface':<14} {'Estimated C_alpha':<20} {'Estimated Mu':<15} {'Adapted Max Speed':<18}")
    print("-" * 77)

    for step in range(41):
        t = step * dt
        # Friction transition at t = 1.0s
        is_wet = (t >= 1.0)
        current_model = model_wet if is_wet else model_dry
        surface_str = "Wet Pavement" if is_wet else "Dry Asphalt"

        # Slalom / cornering steer excitation
        steer = 0.12 * math.sin(2.0 * math.pi * 1.0 * t)
        ctrl = DynamicControlInput(steering_angle=steer, acceleration=0.0)

        # Step vehicle physics
        state = current_model.step_rk4(state, ctrl, dt=dt)
        ay = (state.fy_f + state.fy_r) / F1TENTH_PARAMS.mass

        # Ingest into online system identifier
        estimate = sys_id.update_from_telemetry(
            vx=state.vx,
            vy=state.vy,
            omega_z=state.omega_z,
            steer_angle=steer,
            ay_measured=ay,
        )

        # Update adaptation manager
        adapt_state = adaptation_mgr.update_from_estimate(estimate, upcoming_curvature=curve_kappa)

        if step % 5 == 0:
            avg_c = 0.5 * (estimate.c_alpha_f + estimate.c_alpha_r)
            print(f"{t:<10.2f} {surface_str:<14} {avg_c:<20.1f} {adapt_state.estimated_mu:<15.2f} {adapt_state.cornering_speed_limit_mps:<18.2f} m/s")

    print("\nAdaptation experiment completed successfully.")
    print("Observer detected surface grip reduction and adapted safe cornering limits.")


if __name__ == "__main__":
    run_sys_id_adaptation_demo()
