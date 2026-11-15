"""
Example 01: Vehicle Dynamics & RK4 Integration (OpenRaceStack)
Demonstrates high-fidelity dynamic single-track bicycle simulation with
Pacejka tire model, longitudinal load transfer, and RK4 numerical integration.
"""

from Localization.core.dynamics import (
    AV21_PARAMS,
    F1TENTH_PARAMS,
    DynamicBicycleModel,
    DynamicControlInput,
    VehicleDynamicsState,
)


def run_dynamics_demo():
    print("=" * 70)
    print("OpenRaceStack — Example 01: High-Fidelity Vehicle Dynamics Simulation")
    print("=" * 70)

    # 1. Instantiate 1:10 scale F1TENTH and full-scale IndyCar AV-21
    f1_model = DynamicBicycleModel(params=F1TENTH_PARAMS)
    av_model = DynamicBicycleModel(params=AV21_PARAMS)

    f1_state = VehicleDynamicsState(x=0.0, y=0.0, psi=0.0, vx=6.0, vy=0.0, omega_z=0.0)
    av_state = VehicleDynamicsState(x=0.0, y=0.0, psi=0.0, vx=35.0, vy=0.0, omega_z=0.0)

    # Step steer command (+0.10 rad) and acceleration (2.0 m/s^2)
    f1_ctrl = DynamicControlInput(steering_angle=0.10, acceleration=2.0)
    av_ctrl = DynamicControlInput(steering_angle=0.05, acceleration=3.0)

    dt = 0.05
    print("\nSimulating 1.0 second cornering step response at 20 Hz (dt = 0.05s)...\n")
    print(f"{'Time (s)':<10} {'F1TENTH Pos (x, y)':<25} {'F1TENTH YawRate':<18} {'AV-21 Pos (x, y)':<25}")
    print("-" * 80)

    for step in range(21):
        t = step * dt
        if step % 5 == 0:
            f1_pos = f"({f1_state.x:.2f}, {f1_state.y:.2f})"
            av_pos = f"({av_state.x:.2f}, {av_state.y:.2f})"
            print(f"{t:<10.2f} {f1_pos:<25} {f1_state.omega_z:<18.3f} {av_pos:<25}")

        f1_state = f1_model.step_rk4(f1_state, f1_ctrl, dt=dt)
        av_state = av_model.step_rk4(av_state, av_ctrl, dt=dt)

    print("\nSimulation complete. Weight transfer & tire slip successfully computed.")


if __name__ == "__main__":
    run_dynamics_demo()
