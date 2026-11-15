"""
Unit Tests for Game-Theoretic Overtaking Decision Engine and Tactical Racing Logic.
"""

from Localization.core.game_theoretic_planner import (
    GameTheoreticOvertakingPlanner,
    RacingManeuver,
)


def test_game_theoretic_free_track():
    """Verify planner recommends MAINTAIN_RACELINE when no competitor is in proximity."""
    planner = GameTheoreticOvertakingPlanner()

    # Competitor is 40m ahead
    decision = planner.evaluate_attacker_strategy(
        s_attacker=10.0,
        d_attacker=0.0,
        v_attacker=15.0,
        s_defender=50.0,
        d_defender=0.0,
        v_defender=14.0,
    )

    assert decision.maneuver == RacingManeuver.MAINTAIN_RACELINE
    assert decision.risk_index < 0.20
    assert not decision.has_significant_overlap


def test_game_theoretic_slipstream_drafting():
    """Verify attacker chooses SLIPSTREAM_FOLLOW on straights when within 10m."""
    planner = GameTheoreticOvertakingPlanner()

    # Trailing by 12 meters on a straight
    decision = planner.evaluate_attacker_strategy(
        s_attacker=20.0,
        d_attacker=0.2,
        v_attacker=18.0,
        s_defender=32.0,
        d_defender=-0.5,
        v_defender=17.0,
        approaching_corner=False,
    )

    assert decision.maneuver == RacingManeuver.SLIPSTREAM_FOLLOW
    # Should tuck directly into defender's slipstream (d = -0.5)
    assert decision.target_d == -0.5
    assert decision.target_speed_mps > 18.0


def test_game_theoretic_inside_overtake():
    """Verify attacker dives inside when approaching corner and space is available."""
    planner = GameTheoreticOvertakingPlanner()

    # Trailing by 3 meters into a left-hand corner, defender on right side (d = -0.5)
    decision = planner.evaluate_attacker_strategy(
        s_attacker=45.0,
        d_attacker=0.0,
        v_attacker=16.0,
        s_defender=48.0,
        d_defender=-0.5,
        v_defender=14.5,
        approaching_corner=True,
        corner_direction="left",
    )

    assert decision.maneuver == RacingManeuver.INSIDE_OVERTAKE
    # Inside line for left turn is positive d
    assert decision.target_d > 0.0


def test_game_theoretic_emergency_yield():
    """Verify attacker yields when closing in at critically dangerous low TTC."""
    planner = GameTheoreticOvertakingPlanner()

    # Attacker closing rapidly (gap 0.3m, speed delta 6 m/s -> TTC = 0.05s)
    decision = planner.evaluate_attacker_strategy(
        s_attacker=50.0,
        d_attacker=0.0,
        v_attacker=20.0,
        s_defender=50.3,
        d_defender=0.0,
        v_defender=14.0,
    )

    assert decision.maneuver == RacingManeuver.YIELD_SAFETY
    assert decision.risk_index > 0.85
    assert decision.target_speed_mps < 14.0


def test_game_theoretic_defender_leaves_room():
    """Verify defender leaves space when attacker has significant overlap at apex."""
    planner = GameTheoreticOvertakingPlanner(vehicle_length_m=0.50)

    # Attacker front axle is alongside defender's rear axle (gap = 0.3m <= 0.5m)
    def_decision = planner.evaluate_defender_strategy(
        s_defender=50.3,
        d_defender=0.0,
        v_defender=14.0,
        s_attacker=50.0,
        d_attacker=0.8,
        v_attacker=15.0,
        approaching_corner=True,
    )

    assert def_decision.has_significant_overlap
    assert def_decision.rule_compliant
    # Defender shifts away from attacker (moves to negative d)
    assert def_decision.target_d < 0.0
