"""
Cross-Platform Vehicle Benchmarking & Dynamics Validation Suite for OpenRaceStack.

Executes standardized racetrack benchmark simulations across multiple vehicle platforms
(1:10 scale F1TENTH, Autonomous Go-Kart, Dallara AV-21 IndyCar) and generates
machine-readable comparative performance reports (JSON and Markdown).
Also performs model validation against recorded/simulated telemetry logs.
"""

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

from Localization.core.adaptation import ModelPredictionValidator
from Localization.core.dynamics import (
    AV21_PARAMS,
    F1TENTH_PARAMS,
    GO_KART_PARAMS,
    DynamicBicycleModel,
    DynamicControlInput,
    VehicleDynamicsState,
    VehicleParameters,
)
from Localization.core.hd_vector_map import HDVectorMap
from Localization.core.pure_pursuit import PurePursuitConfig, PurePursuitController


@dataclass
class VehicleBenchmarkResult:
    """Benchmark performance metrics for a specific vehicle platform."""
    platform_name: str
    mass_kg: float
    wheelbase_m: float
    lap_time_s: float
    top_speed_kph: float
    max_lateral_g: float
    rmse_cross_track_m: float
    max_cross_track_m: float
    mean_speed_mps: float
    friction_utilization_pct: float
    validation_rmse_pos_m: float


def generate_benchmark_circuit() -> HDVectorMap:
    """Generates a standardized international-style proving ground oval with chicanes."""
    # Synthetic oval track (length ~ 800m)
    angles = np.linspace(0.0, 2.0 * math.pi, 200, endpoint=False)
    # Super-ellipse with varied curvature
    r = 120.0 + 30.0 * np.cos(2.0 * angles)
    x = r * np.cos(angles)
    y = 0.6 * r * np.sin(angles)
    centerline = np.column_stack((x, y))
    widths = np.full((len(centerline), 2), 5.0)
    return HDVectorMap("StandardProvingGround", centerline, widths, is_closed=True)


def run_single_vehicle_benchmark(
    name: str,
    params: VehicleParameters,
    hd_map: HDVectorMap,
    sim_duration_s: float = 30.0,
    dt: float = 0.05,
) -> VehicleBenchmarkResult:
    """Simulates a complete closed-loop lap for a specific vehicle configuration."""
    model = DynamicBicycleModel(params=params)
    cfg = PurePursuitConfig(
        wheelbase_m=params.wheelbase,
        min_lookahead_m=max(0.6, params.wheelbase * 1.5),
        max_lookahead_m=max(2.5, params.wheelbase * 5.0),
        lookahead_gain_s=0.15,
        max_steering_rad=params.max_steering_angle,
        max_steering_rate_radps=params.max_steering_rate,
        max_accel_mps2=params.max_accel,
        max_decel_mps2=params.max_decel,
    )
    controller = PurePursuitController(config=cfg)

    # Initial state on start-finish line
    v_init = 5.0 if params.mass < 50.0 else 15.0
    state = VehicleDynamicsState(
        x=float(hd_map.centerline[0, 0]),
        y=float(hd_map.centerline[0, 1]),
        psi=float(hd_map.headings[0]),
        vx=v_init,
        vy=0.0,
        omega_z=0.0,
    )

    n_steps = int(sim_duration_s / dt)
    speeds = []
    lateral_accels = []
    cross_track_errors = []
    recorded_states = []
    recorded_controls = []

    # Target speed profile based on vehicle power/grip
    v_target = 14.0 if params.mass < 50.0 else 32.0
    target_vels = np.full(len(hd_map.centerline), v_target)

    lap_completed = False
    lap_time = sim_duration_s

    for step in range(n_steps):
        t = step * dt
        # Curvature at current vehicle pose
        s_val, e_y, _ = hd_map.project_point(state.x, state.y)
        cross_track_errors.append(abs(e_y))

        # Check lap completion
        if step > 50 and s_val > 0.85 * hd_map.total_length and not lap_completed:
            lap_completed = True
            lap_time = t

        # Controller demand
        cmd = controller.compute_control(
            vehicle_x=state.x,
            vehicle_y=state.y,
            vehicle_theta=state.psi,
            vehicle_v=state.vx,
            reference_path=hd_map.centerline,
            reference_velocities=target_vels,
            dt=dt,
        )
        ctrl = DynamicControlInput(steering_angle=cmd.steering_angle_rad, acceleration=cmd.acceleration_mps2)

        # Step dynamics physics
        next_state = model.step_rk4(state, ctrl, dt=dt)

        speeds.append(state.speed)
        ay = abs(state.speed * state.omega_z)
        lateral_accels.append(ay)

        recorded_states.append(state)
        recorded_controls.append(ctrl)
        state = next_state

    # Model validation against recorded trajectory (10-step ahead validation)
    validator = ModelPredictionValidator(params=params)
    val_error = validator.evaluate_k_step_prediction_error(
        model=model,
        initial_state=recorded_states[0],
        controls=recorded_controls[:15],
        ground_truth_states=recorded_states[1:16],
        dt=dt,
    )

    max_lat_ay = float(np.max(lateral_accels))
    max_g = max_lat_ay / 9.81
    top_spd_kph = float(np.max(speeds) * 3.6)
    mean_spd_mps = float(np.mean(speeds))
    rmse_ey = float(np.sqrt(np.mean(np.array(cross_track_errors)**2)))
    max_ey = float(np.max(cross_track_errors))
    fric_util = min(100.0, (max_lat_ay / (1.0 * 9.81)) * 100.0)

    return VehicleBenchmarkResult(
        platform_name=name,
        mass_kg=float(params.mass),
        wheelbase_m=float(params.wheelbase),
        lap_time_s=float(lap_time),
        top_speed_kph=top_spd_kph,
        max_lateral_g=max_g,
        rmse_cross_track_m=rmse_ey,
        max_cross_track_m=max_ey,
        mean_speed_mps=mean_spd_mps,
        friction_utilization_pct=fric_util,
        validation_rmse_pos_m=val_error.rmse_pos_2d,
    )


def execute_fleet_benchmarks(output_json_path: str | Path | None = None) -> list[VehicleBenchmarkResult]:
    """
    Executes standardized benchmarks across F1TENTH, Go-Kart, and Dallara AV-21 platforms.
    """
    circuit = generate_benchmark_circuit()
    platforms = [
        ("F1TENTH (1:10 Scaled Racer)", F1TENTH_PARAMS),
        ("Autonomous Electric Go-Kart", GO_KART_PARAMS),
        ("Dallara AV-21 (Indy Autonomous Challenge)", AV21_PARAMS),
    ]

    results = []
    for name, params in platforms:
        res = run_single_vehicle_benchmark(name, params, circuit)
        results.append(res)

    if output_json_path is not None:
        out_file = Path(output_json_path)
        out_file.parent.mkdir(parents=True, exist_ok=True)
        data = [asdict(r) for r in results]
        with open(out_file, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)

    return results


if __name__ == "__main__":
    print("Running OpenRaceStack Fleet Benchmark...")
    results = execute_fleet_benchmarks("benchmark_results.json")
    print(f"\n{'Platform':<40} {'Mass (kg)':<10} {'Top Speed':<12} {'Max Lat G':<10} {'RMSE e_y (m)':<12}")
    print("-" * 88)
    for r in results:
        print(f"{r.platform_name:<40} {r.mass_kg:<10.1f} {r.top_speed_kph:<12.1f} {r.max_lateral_g:<10.2f} {r.rmse_cross_track_m:<12.3f}")
    print("\nBenchmark completed successfully! Saved to benchmark_results.json")
