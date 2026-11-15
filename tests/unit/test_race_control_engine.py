"""
Unit Tests for Race Control, Leaderboard Scoring, Penalties, and Flag Management.
"""

import math

import numpy as np
import pytest

from Localization.core.hd_vector_map import HDVectorMap
from Localization.core.multi_agent_sim import (
    MultiAgentSimulationEngine,
    RacingAgent,
)
from Localization.core.race_control_engine import (
    RaceControlEngine,
    RaceSessionState,
)
from Localization.core.ros2_bridge import RaceControlFlag


@pytest.fixture
def sim_and_race_control():
    """Returns an initialized simulation and race control engine on a circular circuit."""
    angles = np.linspace(0.0, 2.0 * math.pi, 100, endpoint=False)
    x = 40.0 * np.cos(angles)
    y = 40.0 * np.sin(angles)
    centerline = np.column_stack((x, y))
    widths = np.full((100, 2), 4.0)
    hd_map = HDVectorMap("CircleRace", centerline, widths, is_closed=True)

    sim = MultiAgentSimulationEngine(hd_map=hd_map, dt=0.05)

    p1 = RacingAgent(agent_id="car_leader", target_speed_mps=15.0)
    p1.state.x = centerline[0, 0]
    p1.state.y = centerline[0, 1]
    p1.state.vx = 14.0

    p2 = RacingAgent(agent_id="car_chaser", target_speed_mps=14.0)
    # Staggered 10m behind
    p2.state.x = centerline[8, 0]
    p2.state.y = centerline[8, 1]
    p2.state.vx = 13.0

    sim.add_agent(p1)
    sim.add_agent(p2)

    rc = RaceControlEngine(sim_engine=sim, target_laps=3)
    return sim, rc


def test_race_control_lifecycle_and_flags(sim_and_race_control):
    """Verify Race Control state machine transitions and flag dispatches."""
    sim, rc = sim_and_race_control

    assert rc.session_state == RaceSessionState.IDLE

    # 1. Start race
    rc.start_race()
    assert rc.session_state == RaceSessionState.GREEN_RACING
    assert rc.flag == RaceControlFlag.GREEN

    # 2. Safety car deployment
    rc.deploy_safety_car()
    assert rc.session_state == RaceSessionState.SAFETY_CAR
    assert rc.flag == RaceControlFlag.YELLOW

    # 3. Resume green
    rc.resume_racing()
    assert rc.session_state == RaceSessionState.GREEN_RACING
    assert rc.flag == RaceControlFlag.GREEN

    # 4. Red flag
    rc.throw_red_flag()
    assert rc.session_state == RaceSessionState.RED_FLAG
    assert rc.flag == RaceControlFlag.RED


def test_race_control_leaderboard_and_gaps(sim_and_race_control):
    """Verify official leaderboard rankings, gaps to leader, and intervals."""
    sim, rc = sim_and_race_control
    rc.start_race()

    # Step race 15 cycles (0.75s)
    for _ in range(15):
        sim.step()
        rc.update()

    assert len(rc.leaderboard) == 2
    p1_entry = rc.leaderboard[0]
    p2_entry = rc.leaderboard[1]

    # P1 gap is 0.0s
    assert p1_entry.position == 1
    assert p1_entry.gap_to_leader_s == 0.0

    # P2 interval is non-negative
    assert p2_entry.position == 2
    assert p2_entry.gap_to_leader_s >= 0.0
    assert p2_entry.interval_to_ahead_s >= 0.0


def test_race_control_track_limits_penalty(sim_and_race_control):
    """Verify 3 track limits violations award a 5-second penalty."""
    sim, rc = sim_and_race_control
    rc.start_race()

    # Move p2 completely off-track (e.g. radius 100m, way past the 44m outer limit)
    sim.agents[1].state.x = 100.0
    sim.agents[1].state.y = 0.0

    # Trigger 3 updates off-track
    for _ in range(3):
        rc.update()

    p2_id = sim.agents[1].agent_id
    assert rc.track_limits_violations[p2_id] == 3
    # Penalties accrued
    assert sim.agents[1].penalties_s == 5.0
    # Status shows penalty
    assert any("PENALTY" in entry.status for entry in rc.leaderboard if entry.agent_id == p2_id)
