"""
Race Control and Fleet Leaderboard Engine for OpenRaceStack.

Coordinates competitive racing sessions: grid start sequences, multi-agent leaderboard
ranking, delta gap calculations, track limits penalty enforcement, and automated safety car
deployments upon racing incidents.
"""

from dataclasses import dataclass
from enum import Enum

from .hd_vector_map import HDVectorMap
from .multi_agent_sim import MultiAgentSimulationEngine, RacingAgent
from .ros2_bridge import RaceControlFlag


class RaceSessionState(str, Enum):
    """Overall competition lifecycle state."""
    IDLE = "idle"
    FORMATION_LAP = "formation_lap"
    GREEN_RACING = "green_racing"
    FULL_COURSE_YELLOW = "full_course_yellow"
    SAFETY_CAR = "safety_car"
    RED_FLAG = "red_flag"
    CHECKERED = "checkered"


@dataclass
class LeaderboardEntry:
    """Official timing and scoring entry for an individual racing competitor."""
    position: int
    agent_id: str
    current_lap: int
    current_sector: int
    gap_to_leader_s: float       # Time delta relative to P1 (+seconds)
    interval_to_ahead_s: float   # Time delta relative to preceding position (+seconds)
    last_lap_s: float
    best_lap_s: float
    penalties_s: float           # Accrued time penalties (seconds)
    track_limits_count: int      # Count of track boundary infringements
    status: str = "RACING"       # RACING, DNF, PENALTY, FINISHED


class RaceControlEngine:
    """
    Official Race Control and live competition timing scoring system.
    """

    def __init__(
        self,
        sim_engine: MultiAgentSimulationEngine,
        target_laps: int = 5,
    ):
        self.sim = sim_engine
        self.hd_map: HDVectorMap = sim_engine.hd_map
        self.target_laps = target_laps

        self.session_state = RaceSessionState.IDLE
        self.flag = RaceControlFlag.GREEN
        self.leaderboard: list[LeaderboardEntry] = []
        self.track_limits_violations: dict[str, int] = {}
        self.winner_id: str | None = None
        self.race_elapsed_s: float = 0.0

    def start_race(self) -> None:
        """Initiates official green flag racing sequence."""
        self.session_state = RaceSessionState.GREEN_RACING
        self.flag = RaceControlFlag.GREEN
        self.race_elapsed_s = 0.0
        for agent in self.sim.agents:
            self.track_limits_violations[agent.agent_id] = 0
            agent.lap_start_time = 0.0

    def deploy_safety_car(self) -> None:
        """Deploys Safety Car / Full Course Yellow following an incident."""
        self.session_state = RaceSessionState.SAFETY_CAR
        self.flag = RaceControlFlag.YELLOW

    def resume_racing(self) -> None:
        """Returns to full-speed green flag racing."""
        self.session_state = RaceSessionState.GREEN_RACING
        self.flag = RaceControlFlag.GREEN

    def throw_red_flag(self) -> None:
        """Halts race session immediately for safety."""
        self.session_state = RaceSessionState.RED_FLAG
        self.flag = RaceControlFlag.RED

    def update(self) -> None:
        """
        Processes simulation step: evaluates track limits, calculates leaderboards,
        awards penalties, and monitors session completion.
        """
        if self.session_state not in (RaceSessionState.GREEN_RACING, RaceSessionState.SAFETY_CAR, RaceSessionState.FULL_COURSE_YELLOW):
            return

        self.race_elapsed_s += self.sim.dt

        # 1. Track Limits Enforcement
        for agent in self.sim.agents:
            if agent.is_dnf:
                continue
            is_inside = self.hd_map.is_inside_track(agent.state.x, agent.state.y, safety_margin=-0.05)
            if not is_inside:
                count = self.track_limits_violations.get(agent.agent_id, 0) + 1
                self.track_limits_violations[agent.agent_id] = count
                # 3 infringements -> 5-second penalty
                if count % 3 == 0:
                    agent.penalties_s += 5.0

        # 2. Sort leaderboard standings
        # Rank by: (is_dnf, -current_lap, -track_progress, elapsed_time + penalties)
        def sort_key(a: RacingAgent):
            s_prog, _, _ = self.hd_map.project_point(a.state.x, a.state.y)
            return (
                1 if a.is_dnf else 0,
                -a.current_lap,
                -s_prog,
                a.total_race_time_s + a.penalties_s,
            )

        sorted_agents = sorted(self.sim.agents, key=sort_key)

        # 3. Compute gaps and intervals
        new_leaderboard: list[LeaderboardEntry] = []
        p1_time = sorted_agents[0].total_race_time_s + sorted_agents[0].penalties_s if sorted_agents else 0.0

        for rank, agent in enumerate(sorted_agents, start=1):
            s_val, _, _ = self.hd_map.project_point(agent.state.x, agent.state.y)
            adj_time = agent.total_race_time_s + agent.penalties_s

            gap_lead = max(0.0, adj_time - p1_time) if rank > 1 else 0.0
            prev_adj = sorted_agents[rank - 2].total_race_time_s + sorted_agents[rank - 2].penalties_s if rank > 1 else adj_time
            interval = max(0.0, adj_time - prev_adj) if rank > 1 else 0.0

            status_str = "DNF" if agent.is_dnf else ("FINISHED" if self.session_state == RaceSessionState.CHECKERED else "RACING")
            if agent.penalties_s > 0 and not agent.is_dnf:
                status_str = f"PENALTY (+{int(agent.penalties_s)}s)"

            entry = LeaderboardEntry(
                position=rank,
                agent_id=agent.agent_id,
                current_lap=agent.current_lap,
                current_sector=agent.current_sector,
                gap_to_leader_s=round(gap_lead, 3),
                interval_to_ahead_s=round(interval, 3),
                last_lap_s=round(agent.last_lap_time_s, 3),
                best_lap_s=round(agent.best_lap_time_s, 3) if agent.best_lap_time_s < 999.0 else 0.0,
                penalties_s=agent.penalties_s,
                track_limits_count=self.track_limits_violations.get(agent.agent_id, 0),
                status=status_str,
            )
            new_leaderboard.append(entry)

        self.leaderboard = new_leaderboard

        # 4. Checkered Flag: leader finishes target laps
        if sorted_agents and sorted_agents[0].current_lap > self.target_laps:
            self.session_state = RaceSessionState.CHECKERED
            self.flag = RaceControlFlag.CHECKERED
            self.winner_id = sorted_agents[0].agent_id
