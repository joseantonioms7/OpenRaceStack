"""
Unit Tests for Multi-Agent Racing Simulation Engine, Drafting, and Collisions.
"""

import math

import numpy as np
import pytest

from Localization.core.dynamics import F1TENTH_PARAMS
from Localization.core.hd_vector_map import HDVectorMap
from Localization.core.multi_agent_sim import (
    MultiAgentSimulationEngine,
    RacingAgent,
)


@pytest.fixture
def oval_hd_map():
    """Synthetic oval track HD Vector Map."""
    angles = np.linspace(0.0, 2.0 * math.pi, 120, endpoint=False)
    x = 50.0 * np.cos(angles)
    y = 25.0 * np.sin(angles)
    centerline = np.column_stack((x, y))
    widths = np.full((120, 2), 4.0)
    return HDVectorMap("OvalSimCircuit", centerline, widths, is_closed=True)


def test_aerodynamic_drafting_and_dirty_air(oval_hd_map):
    """Verify trailing car in slipstream receives drag reduction and dirty air downforce drop."""
    sim = MultiAgentSimulationEngine(hd_map=oval_hd_map)

    # Lead car at (50, 0) (s ~ 0)
    lead_car = RacingAgent(agent_id="car_lead", params=F1TENTH_PARAMS, target_speed_mps=15.0)
    lead_car.state.x = 50.0
    lead_car.state.y = 0.0

    # Trailing car closely behind (s ~ total_length - 5.0m)
    trail_car = RacingAgent(agent_id="car_trail", params=F1TENTH_PARAMS, target_speed_mps=15.0)
    # Position trailing car 6 meters behind on the track
    s_trail = (oval_hd_map.total_length - 6.0) % oval_hd_map.total_length
    idx_trail = int(np.searchsorted(oval_hd_map.s, s_trail) - 1)
    trail_car.state.x = oval_hd_map.centerline[idx_trail, 0]
    trail_car.state.y = oval_hd_map.centerline[idx_trail, 1]

    sim.add_agent(lead_car)
    sim.add_agent(trail_car)

    sim.compute_aerodynamic_interactions()

    # Trailing car should receive drag reduction (multiplier < 1.0)
    assert trail_car.drafting_drag_multiplier < 1.0
    assert trail_car.dirty_air_downforce_multiplier < 1.0

    # Lead car has clean air
    assert lead_car.drafting_drag_multiplier == 1.0
    assert lead_car.dirty_air_downforce_multiplier == 1.0


def test_mutual_collision_detection(oval_hd_map):
    """Verify mutual collision detector records collision event when cars overlap."""
    sim = MultiAgentSimulationEngine(hd_map=oval_hd_map)

    car_a = RacingAgent(agent_id="car_a", params=F1TENTH_PARAMS)
    car_a.state.x = 20.0
    car_a.state.y = 10.0
    car_a.state.vx = 12.0

    car_b = RacingAgent(agent_id="car_b", params=F1TENTH_PARAMS)
    # Placed 0.15m away (inside track_width collision threshold of 0.40m)
    car_b.state.x = 20.15
    car_b.state.y = 10.0
    car_b.state.vx = 8.0

    sim.add_agent(car_a)
    sim.add_agent(car_b)

    collisions = sim.check_mutual_collisions()
    assert len(collisions) == 1
    assert collisions[0].agent_a_id == "car_a"
    assert collisions[0].agent_b_id == "car_b"
    assert math.isclose(collisions[0].relative_speed_mps, 4.0, abs_tol=0.1)


def test_multi_agent_simulation_step(oval_hd_map):
    """Verify simulation steps multiple vehicles simultaneously along track."""
    sim = MultiAgentSimulationEngine(hd_map=oval_hd_map, dt=0.05)

    car_1 = RacingAgent(agent_id="racer_1", target_speed_mps=12.0)
    car_1.state.x = oval_hd_map.centerline[0, 0]
    car_1.state.y = oval_hd_map.centerline[0, 1]
    car_1.state.psi = oval_hd_map.headings[0]
    car_1.state.vx = 10.0

    car_2 = RacingAgent(agent_id="racer_2", target_speed_mps=11.0)
    # Staggered behind by 15 meters
    car_2.state.x = oval_hd_map.centerline[10, 0]
    car_2.state.y = oval_hd_map.centerline[10, 1]
    car_2.state.psi = oval_hd_map.headings[10]
    car_2.state.vx = 9.0

    sim.add_agent(car_1)
    sim.add_agent(car_2)

    # Step simulation 10 cycles (0.5 seconds)
    for _ in range(10):
        sim.step()

    assert sim.sim_time > 0.45
    assert car_1.state.speed > 5.0
    assert car_2.state.speed > 5.0
    assert car_1.total_race_time_s > 0.45
