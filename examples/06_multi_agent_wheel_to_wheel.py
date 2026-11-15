"""
Example 06: Multi-Agent Wheel-to-Wheel Racing Simulation (OpenRaceStack)
Demonstrates multi-car simulation with aerodynamic wake drafting, dirty air,
obstacle avoidance, and dynamic leaderboard timing.
"""

import math

import numpy as np

from Localization.core.dynamics import F1TENTH_PARAMS
from Localization.core.hd_vector_map import HDVectorMap
from Localization.core.multi_agent_sim import (
    MultiAgentSimulationEngine,
    RacingAgent,
)


def run_multi_agent_demo():
    print("=" * 70)
    print("OpenRaceStack — Example 06: Multi-Agent Autonomous Racing Simulation")
    print("=" * 70)

    # 1. Oval racetrack
    angles = np.linspace(0.0, 2.0 * math.pi, 100, endpoint=False)
    x = 60.0 * np.cos(angles)
    y = 30.0 * np.sin(angles)
    centerline = np.column_stack((x, y))
    widths = np.full((100, 2), 4.5)
    hd_map = HDVectorMap("OvalRacetrack", centerline, widths, is_closed=True)

    sim = MultiAgentSimulationEngine(hd_map=hd_map, dt=0.05)

    # 2. Lead Car (Car 01)
    car_lead = RacingAgent(agent_id="Apex-01 (Lead)", params=F1TENTH_PARAMS, target_speed_mps=12.0)
    car_lead.state.x = float(centerline[0, 0])
    car_lead.state.y = float(centerline[0, 1])
    car_lead.state.psi = float(hd_map.headings[0])
    car_lead.state.vx = 10.0

    # 3. Trailing Car (Car 02) closely behind to experience slipstream drafting
    car_trail = RacingAgent(agent_id="Apex-02 (Chaser)", params=F1TENTH_PARAMS, target_speed_mps=13.0)
    car_trail.state.x = float(centerline[96, 0])
    car_trail.state.y = float(centerline[96, 1])
    car_trail.state.psi = float(hd_map.headings[96])
    car_trail.state.vx = 11.5

    sim.add_agent(car_lead)
    sim.add_agent(car_trail)

    print("\nEnrolled 2 autonomous racing agents:")
    print("  - Agent 1: Lead vehicle (Target: 12 m/s)")
    print("  - Agent 2: Chaser in slipstream (Target: 13 m/s)")
    print("\nExecuting 1.5 seconds of wheel-to-wheel simulation (30 cycles @ 20 Hz)...\n")
    print(f"{'Time (s)':<10} {'Lead Pos':<20} {'Chaser Pos':<20} {'Drafting Drag Mult':<20} {'Dirty Air Grip':<15}")
    print("-" * 85)

    for step in range(31):
        sim.step()
        if step % 5 == 0:
            lead_pos = f"({car_lead.state.x:.1f}, {car_lead.state.y:.1f})"
            chaser_pos = f"({car_trail.state.x:.1f}, {car_trail.state.y:.1f})"
            drag_str = f"{car_trail.drafting_drag_multiplier:.2f} (-{(1.0 - car_trail.drafting_drag_multiplier)*100:.0f}%)"
            grip_str = f"{car_trail.dirty_air_downforce_multiplier:.2f}"
            print(f"{sim.sim_time:<10.2f} {lead_pos:<20} {chaser_pos:<20} {drag_str:<20} {grip_str:<15}")

    print(f"\nSimulation complete. Total simulated time: {sim.sim_time:.2f}s | Collisions detected: {len(sim.collision_events)}")


if __name__ == "__main__":
    run_multi_agent_demo()
