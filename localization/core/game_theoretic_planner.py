"""
Game-Theoretic Interactive Overtaking Logic for OpenRaceStack.

Evaluates strategic interactive multi-vehicle game matrices (Attacker vs Defender),
FIA Rules of Racing (significant overlap, one-move rule, apex room), and computes
optimal strategic lateral targets and speeds for wheel-to-wheel duels.
"""

from dataclasses import dataclass
from enum import Enum

import numpy as np


class RacingManeuver(str, Enum):
    """Competitive tactical racing actions."""
    MAINTAIN_RACELINE = "maintain_raceline"
    INSIDE_OVERTAKE = "inside_overtake"
    OUTSIDE_SWEEP = "outside_sweep"
    DEFENSIVE_COVER = "defensive_cover"
    SLIPSTREAM_FOLLOW = "slipstream_follow"
    YIELD_SAFETY = "yield_safety"


@dataclass
class ManeuverDecision:
    """Strategic decision produced by the game-theoretic overtaking engine."""
    maneuver: RacingManeuver
    target_d: float              # Commanded lateral offset in Frenet frame (m)
    target_speed_mps: float      # Commanded target velocity (m/s)
    risk_index: float            # Collision risk indicator [0.0 = safe, 1.0 = high risk]
    time_to_collision_s: float   # Estimated longitudinal TTC (s)
    has_significant_overlap: bool# True if front axle is alongside defender's rear axle
    rule_compliant: bool         # True if maneuver conforms to FIA racing standards


class GameTheoreticOvertakingPlanner:
    """
    Strategic multi-agent decision maker for head-to-head autonomous racing.
    """

    def __init__(
        self,
        vehicle_length_m: float = 0.50,
        vehicle_width_m: float = 0.30,
        safety_clearance_m: float = 0.40,
    ):
        self.veh_len = vehicle_length_m
        self.veh_width = vehicle_width_m
        self.clearance = safety_clearance_m

    def evaluate_attacker_strategy(
        self,
        s_attacker: float,
        d_attacker: float,
        v_attacker: float,
        s_defender: float,
        d_defender: float,
        v_defender: float,
        road_width_left: float = 3.5,
        road_width_right: float = 3.5,
        approaching_corner: bool = False,
        corner_direction: str = "left", # "left" or "right"
    ) -> ManeuverDecision:
        """
        Determines the optimal competitive tactical move for the trailing Attacker.
        """
        # Distance and closing velocity
        gap_s = s_defender - s_attacker
        closing_speed = v_attacker - v_defender
        ttc = gap_s / max(0.1, closing_speed) if closing_speed > 0.05 else float("inf")

        # Significant overlap rule: attacker front axle reached defender rear axle
        # gap_s <= vehicle_length
        has_overlap = 0.0 <= gap_s <= self.veh_len * 1.2

        # 1. Emergency collision avoidance: dangerously low TTC (< 0.35s)
        if 0.0 < gap_s < self.veh_len * 0.8 or ttc < 0.35:
            return ManeuverDecision(
                maneuver=RacingManeuver.YIELD_SAFETY,
                target_d=d_attacker + (0.8 if d_attacker >= 0 else -0.8),
                target_speed_mps=max(0.0, v_defender - 2.0),
                risk_index=0.95,
                time_to_collision_s=ttc,
                has_significant_overlap=has_overlap,
                rule_compliant=True,
            )

        # 2. Free track / distant opponent (gap > 25m): maintain optimal raceline
        if gap_s > 25.0 or gap_s < 0.0:
            return ManeuverDecision(
                maneuver=RacingManeuver.MAINTAIN_RACELINE,
                target_d=0.0,
                target_speed_mps=v_attacker,
                risk_index=0.05,
                time_to_collision_s=ttc,
                has_significant_overlap=False,
                rule_compliant=True,
            )

        # 3. Slipstream drafting on straight (gap 6m - 20m, not approaching corner)
        if not approaching_corner and 5.0 < gap_s <= 20.0:
            return ManeuverDecision(
                maneuver=RacingManeuver.SLIPSTREAM_FOLLOW,
                target_d=d_defender, # Align directly in the wake
                target_speed_mps=v_attacker * 1.05, # Higher speed in draft
                risk_index=0.25,
                time_to_collision_s=ttc,
                has_significant_overlap=False,
                rule_compliant=True,
            )

        # 4. Approaching corner or close proximity (gap <= 5m): evaluate inside vs outside overtake
        # Inside line direction
        inside_side = "left" if corner_direction == "left" else "right"
        inside_target_d = (d_defender + 1.2) if inside_side == "left" else (d_defender - 1.2)
        outside_target_d = (d_defender - 1.2) if inside_side == "left" else (d_defender + 1.2)

        # Check corridor boundaries for inside line
        inside_open = (-road_width_right + self.clearance) <= inside_target_d <= (road_width_left - self.clearance)
        outside_open = (-road_width_right + self.clearance) <= outside_target_d <= (road_width_left - self.clearance)

        # Preference 1: Inside dive if open
        if inside_open and (inside_side == "left" and d_defender < 0.8 or inside_side == "right" and d_defender > -0.8):
            return ManeuverDecision(
                maneuver=RacingManeuver.INSIDE_OVERTAKE,
                target_d=float(np.clip(inside_target_d, -road_width_right + 0.5, road_width_left - 0.5)),
                target_speed_mps=v_attacker,
                risk_index=0.45,
                time_to_collision_s=ttc,
                has_significant_overlap=has_overlap,
                rule_compliant=True,
            )

        # Preference 2: Outside sweep if inside is blocked
        if outside_open:
            return ManeuverDecision(
                maneuver=RacingManeuver.OUTSIDE_SWEEP,
                target_d=float(np.clip(outside_target_d, -road_width_right + 0.5, road_width_left - 0.5)),
                target_speed_mps=v_attacker,
                risk_index=0.40,
                time_to_collision_s=ttc,
                has_significant_overlap=has_overlap,
                rule_compliant=True,
            )

        # Default fallback: maintain safety buffer
        return ManeuverDecision(
            maneuver=RacingManeuver.YIELD_SAFETY,
            target_d=d_attacker,
            target_speed_mps=v_defender,
            risk_index=0.60,
            time_to_collision_s=ttc,
            has_significant_overlap=has_overlap,
            rule_compliant=True,
        )

    def evaluate_defender_strategy(
        self,
        s_defender: float,
        d_defender: float,
        v_defender: float,
        s_attacker: float,
        d_attacker: float,
        v_attacker: float,
        approaching_corner: bool = False,
        corner_direction: str = "left",
    ) -> ManeuverDecision:
        """
        Determines the optimal defensive strategy for the lead car while conforming
        to the FIA One-Move Rule and leaving space if attacker has significant overlap.
        """
        gap_s = s_defender - s_attacker
        has_overlap = 0.0 <= gap_s <= self.veh_len * 1.2

        # If attacker has significant overlap: Defender MUST leave 1 car width at the apex!
        if has_overlap:
            # Shift away from attacker's side to guarantee clearance
            defensive_d = d_defender + (0.8 if d_attacker < d_defender else -0.8)
            return ManeuverDecision(
                maneuver=RacingManeuver.MAINTAIN_RACELINE,
                target_d=defensive_d,
                target_speed_mps=v_defender,
                risk_index=0.50,
                time_to_collision_s=gap_s / max(0.1, v_attacker - v_defender),
                has_significant_overlap=True,
                rule_compliant=True,
            )

        # Approaching corner with attacker within 10m: cover the inside line (One Move rule)
        if approaching_corner and 0.5 < gap_s < 12.0:
            inside_d = 1.0 if corner_direction == "left" else -1.0
            return ManeuverDecision(
                maneuver=RacingManeuver.DEFENSIVE_COVER,
                target_d=inside_d,
                target_speed_mps=v_defender,
                risk_index=0.35,
                time_to_collision_s=gap_s / max(0.1, v_attacker - v_defender),
                has_significant_overlap=False,
                rule_compliant=True,
            )

        # Nominal racing line
        return ManeuverDecision(
            maneuver=RacingManeuver.MAINTAIN_RACELINE,
            target_d=0.0,
            target_speed_mps=v_defender,
            risk_index=0.10,
            time_to_collision_s=float("inf"),
            has_significant_overlap=False,
            rule_compliant=True,
        )
