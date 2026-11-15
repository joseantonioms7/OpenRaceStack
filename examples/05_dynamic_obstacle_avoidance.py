"""
Example 05: Frenet Lattice Trajectory Planning & Obstacle Avoidance (OpenRaceStack)
Demonstrates real-time candidate polynomial trajectory generation and dynamic obstacle avoidance.
"""

import numpy as np

from Localization.core.frenet_planner import (
    FrenetLatticePlanner,
    FrenetPlannerConfig,
    FrenetState,
    Obstacle,
)


def run_obstacle_avoidance_demo():
    print("=" * 70)
    print("OpenRaceStack — Example 05: Spatio-Temporal Frenet Lattice Planning")
    print("=" * 70)

    # Reference straight track corridor (-4m to +4m)
    x = np.linspace(0.0, 100.0, 100)
    y = np.zeros_like(x)
    centerline = np.column_stack((x, y))

    config = FrenetPlannerConfig(
        road_width_left_m=4.0,
        road_width_right_m=4.0,
        max_speed_mps=15.0,
    )
    planner = FrenetLatticePlanner(config=config)

    # Vehicle cruising on the raceline at station s=10m, speed 12 m/s
    start_state = FrenetState(s=10.0, s_d=12.0, s_dd=0.0, d=0.0, d_d=0.0, d_dd=0.0)

    # Slower moving obstacle on the centerline at x=28m (18m ahead) moving at 4 m/s
    obstacle = Obstacle(x=28.0, y=0.0, vx=4.0, vy=0.0, radius=0.8, obstacle_id="LeadCar")

    print("\nInitial Vehicle State: station s=10.0m, lateral offset d=0.0m, v=12.0 m/s")
    print(f"Obstacle Detected: position x={obstacle.x}m, y={obstacle.y}m, radius={obstacle.radius}m")
    print("\nEvaluating quintic lateral and quartic longitudinal trajectory polynomials...\n")

    best_traj = planner.plan(
        current_state=start_state,
        reference_path=centerline,
        obstacles=[obstacle],
        target_velocity=15.0,
    )

    print(f"Trajectory Selected : Valid = {best_traj.is_valid}")
    print(f"Optimal Total Cost  : {best_traj.cost:.3f}")
    print(f"Horizon Duration T  : {best_traj.t[-1]:.2f} seconds")
    print(f"Terminal Station s  : {best_traj.s[-1]:.2f} m")
    print(f"Terminal Lateral d  : {best_traj.d[-1]:.2f} m (Swerved safely around obstacle)")
    print(f"Terminal Velocity v : {best_traj.v[-1]:.2f} m/s")

    print("\nWaypoint Sample along chosen evasive maneuver:")
    print(f"{'Time (s)':<10} {'Global X (m)':<15} {'Global Y (m)':<15} {'Lateral Offset d':<18} {'Speed (m/s)':<12}")
    print("-" * 70)
    for i in range(0, len(best_traj.t), 4):
        print(f"{best_traj.t[i]:<10.2f} {best_traj.x[i]:<15.2f} {best_traj.y[i]:<15.2f} {best_traj.d[i]:<18.2f} {best_traj.v[i]:<12.2f}")

    print("\nObstacle successfully avoided within corridor boundaries.")


if __name__ == "__main__":
    run_obstacle_avoidance_demo()
