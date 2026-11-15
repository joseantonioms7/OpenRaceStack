"""
Multi-Agent Autonomous Racing Simulation Engine for OpenRaceStack.

Simulates simultaneous wheel-to-wheel racing of multiple autonomous vehicles on the same circuit,
including mutual oriented collision detection, aerodynamic wake drafting / slipstream effects,
dirty air downforce reduction, and dynamic leaderboard timing.
"""

import math
from dataclasses import dataclass, field

import numpy as np

from .dynamics import (
    F1TENTH_PARAMS,
    DynamicBicycleModel,
    DynamicControlInput,
    VehicleDynamicsState,
    VehicleParameters,
)
from .frenet_mpc import FrenetMPCConfig, FrenetMPCController
from .frenet_planner import FrenetLatticePlanner, FrenetPlannerConfig, FrenetState, Obstacle
from .hd_vector_map import HDVectorMap


@dataclass
class RacingAgent:
    """Individual autonomous race vehicle agent competing in multi-agent fleet."""
    agent_id: str
    params: VehicleParameters = F1TENTH_PARAMS
    target_speed_mps: float = 16.0
    color_hex: str = "#00E5FF"
    
    # Internal controllers and models
    dynamics: DynamicBicycleModel = field(init=False)
    mpc: FrenetMPCController = field(init=False)
    planner: FrenetLatticePlanner = field(init=False)
    state: VehicleDynamicsState = field(init=False)

    # Racing metrics and timing
    current_lap: int = 1
    lap_start_time: float = 0.0
    current_sector: int = 1
    last_lap_time_s: float = 0.0
    best_lap_time_s: float = float("inf")
    total_race_time_s: float = 0.0
    is_dnf: bool = False
    penalties_s: float = 0.0
    drafting_drag_multiplier: float = 1.0   # Aerodynamic wake drag factor
    dirty_air_downforce_multiplier: float = 1.0 # Aerodynamic wake downforce factor

    def __post_init__(self):
        self.dynamics = DynamicBicycleModel(params=self.params)
        self.mpc = FrenetMPCController(FrenetMPCConfig(wheelbase_m=self.params.wheelbase, horizon_steps=12, dt=0.05))
        self.planner = FrenetLatticePlanner(FrenetPlannerConfig(
            max_speed_mps=self.target_speed_mps * 1.2,
            road_width_left_m=4.0,
            road_width_right_m=4.0,
        ))
        self.state = VehicleDynamicsState(x=0.0, y=0.0, psi=0.0, vx=10.0, vy=0.0, omega_z=0.0)


@dataclass
class CollisionEvent:
    """Record of physical contact between two racing agents."""
    timestamp: float
    agent_a_id: str
    agent_b_id: str
    location: tuple[float, float]
    relative_speed_mps: float


class MultiAgentSimulationEngine:
    """
    Fleet-scale multi-vehicle autonomous racing simulation coordinator.
    """

    def __init__(
        self,
        hd_map: HDVectorMap,
        agents: list[RacingAgent] | None = None,
        dt: float = 0.05,
    ):
        self.hd_map = hd_map
        self.dt = dt
        self.agents: list[RacingAgent] = agents or []
        self.collision_events: list[CollisionEvent] = []
        self.sim_time: float = 0.0

    def add_agent(self, agent: RacingAgent) -> None:
        """Enrolls an agent into the racing fleet."""
        self.agents.append(agent)

    def reset(self) -> None:
        """Resets the simulation time and clears collision events and agent states."""
        self.sim_time = 0.0
        self.collision_events.clear()
        for agent in self.agents:
            agent.current_lap = 1
            agent.lap_start_time = 0.0
            agent.current_sector = 1
            agent.last_lap_time_s = 0.0
            agent.total_race_time_s = 0.0
            agent.is_dnf = False
            agent.penalties_s = 0.0
            agent.drafting_drag_multiplier = 1.0
            agent.dirty_air_downforce_multiplier = 1.0

    def compute_aerodynamic_interactions(self) -> None:
        """
        Calculates slipstream wake drafting (drag reduction) and dirty air
        turbulent wake (downforce reduction) for trailing vehicles.
        """
        n_agents = len(self.agents)
        for i in range(n_agents):
            agent_tail = self.agents[i]
            # Reset multipliers to nominal
            agent_tail.drafting_drag_multiplier = 1.0
            agent_tail.dirty_air_downforce_multiplier = 1.0

            s_tail, d_tail, th_tail = self.hd_map.project_point(agent_tail.state.x, agent_tail.state.y)

            for j in range(n_agents):
                if i == j:
                    continue
                agent_lead = self.agents[j]
                s_lead, d_lead, _ = self.hd_map.project_point(agent_lead.state.x, agent_lead.state.y)

                # Distance ahead along track
                ds = (s_lead - s_tail) % self.hd_map.total_length
                dd = abs(d_lead - d_tail)

                # Slipstream active if trailing by 0.5m to 20m and laterally aligned within 1.2m
                if 0.5 < ds < 20.0 and dd < 1.2:
                    # Drag drops up to 35% in direct slipstream
                    drag_reduction = 0.35 * math.exp(-ds / 8.0)
                    agent_tail.drafting_drag_multiplier = min(
                        agent_tail.drafting_drag_multiplier,
                        1.0 - drag_reduction,
                    )
                    # Downforce drops up to 25% in dirty air
                    downforce_loss = 0.25 * math.exp(-ds / 10.0)
                    agent_tail.dirty_air_downforce_multiplier = min(
                        agent_tail.dirty_air_downforce_multiplier,
                        1.0 - downforce_loss,
                    )

    def check_mutual_collisions(self) -> list[CollisionEvent]:
        """
        Performs dual-circle body collision checking between all agent pairs.
        """
        detected: list[CollisionEvent] = []
        n_agents = len(self.agents)

        for i in range(n_agents):
            for j in range(i + 1, n_agents):
                a1 = self.agents[i]
                a2 = self.agents[j]

                # Center distance
                dist = math.hypot(a1.state.x - a2.state.x, a1.state.y - a2.state.y)
                # Combined collision envelope radius
                r_thresh = a1.params.track_width + a2.params.track_width

                if dist < r_thresh:
                    rel_speed = math.hypot(a1.state.vx - a2.state.vx, a1.state.vy - a2.state.vy)
                    event = CollisionEvent(
                        timestamp=self.sim_time,
                        agent_a_id=a1.agent_id,
                        agent_b_id=a2.agent_id,
                        location=(0.5 * (a1.state.x + a2.state.x), 0.5 * (a1.state.y + a2.state.y)),
                        relative_speed_mps=float(rel_speed),
                    )
                    detected.append(event)
                    self.collision_events.append(event)

        return detected

    def step(self) -> None:
        """
        Advances the entire multi-agent racing simulation by one time step dt:
        1. Computes aerodynamic wake interactions (slipstream/dirty air).
        2. Plans obstacle avoidance trajectories treating other cars as dynamic obstacles.
        3. Tracks trajectories using Frenet MPC.
        4. Integrates vehicle physical dynamics with Pacejka tires.
        5. Detects inter-vehicle collisions and updates lap records.
        """
        self.compute_aerodynamic_interactions()

        # Step each agent
        for i, agent in enumerate(self.agents):
            if agent.is_dnf:
                continue

            # Represent all other active cars as dynamic obstacles
            obstacles = []
            for j, other in enumerate(self.agents):
                if i != j and not other.is_dnf:
                    obs = Obstacle(
                        x=other.state.x,
                        y=other.state.y,
                        vx=other.state.vx * math.cos(other.state.psi),
                        vy=other.state.vx * math.sin(other.state.psi),
                        radius=other.params.track_width * 0.8,
                        obstacle_id=other.agent_id,
                    )
                    obstacles.append(obs)

            # 1. Project current pose to Frenet frame
            s_val, e_y, th_ref = self.hd_map.project_point(agent.state.x, agent.state.y)
            delta_yaw = agent.state.psi - th_ref
            s_d = max(0.5, agent.state.vx * math.cos(delta_yaw))
            d_d = agent.state.vx * math.sin(delta_yaw)

            frenet_state = FrenetState(s=s_val, s_d=s_d, s_dd=0.0, d=e_y, d_d=d_d, d_dd=0.0)

            # 2. Local Lattice Trajectory Planning around obstacles
            traj = agent.planner.plan(
                current_state=frenet_state,
                reference_path=self.hd_map.centerline,
                obstacles=obstacles,
                target_velocity=agent.target_speed_mps,
            )

            # 3. Path reference for tracking
            if traj.is_valid and len(traj.x) > 2:
                ref_path = np.column_stack((traj.x, traj.y))
                ref_headings = traj.yaw
                ref_curvatures = traj.curvature
                ref_vels = traj.v
            else:
                ref_path = self.hd_map.centerline
                ref_headings = self.hd_map.headings
                ref_curvatures = np.zeros(len(self.hd_map.centerline))
                ref_vels = np.full(len(self.hd_map.centerline), agent.target_speed_mps)

            # 4. MPC Trajectory Tracking
            cmd = agent.mpc.compute_control(
                vehicle_x=agent.state.x,
                vehicle_y=agent.state.y,
                vehicle_theta=agent.state.psi,
                vehicle_v=agent.state.vx,
                reference_path=ref_path,
                reference_headings=ref_headings,
                reference_curvatures=ref_curvatures,
                reference_velocities=ref_vels,
            )

            # 5. Apply aerodynamic wake drag multiplier
            mod_accel = cmd.acceleration_mps2
            if mod_accel > 0:
                # Drafting boost: lower drag allows faster acceleration
                mod_accel += (1.0 - agent.drafting_drag_multiplier) * 1.5

            # 6. Physical Dynamics Integration (RK4 with Pacejka tires)
            ctrl = DynamicControlInput(
                steering_angle=cmd.steering_angle_rad,
                acceleration=mod_accel,
            )
            agent.state = agent.dynamics.step_rk4(agent.state, ctrl, dt=self.dt)

            # 7. Sector and Lap Timing
            agent.total_race_time_s += self.dt
            new_s, _, _ = self.hd_map.project_point(agent.state.x, agent.state.y)
            agent.current_sector = self.hd_map.get_sector(new_s)

            # Lap completion check: wrapped from >0.9*total_length to <0.1*total_length
            if s_val > 0.85 * self.hd_map.total_length and new_s < 0.15 * self.hd_map.total_length:
                lap_duration = agent.total_race_time_s - agent.lap_start_time
                agent.last_lap_time_s = lap_duration
                if lap_duration < agent.best_lap_time_s:
                    agent.best_lap_time_s = lap_duration
                agent.current_lap += 1
                agent.lap_start_time = agent.total_race_time_s

        # Collision detection
        self.check_mutual_collisions()
        self.sim_time += self.dt
