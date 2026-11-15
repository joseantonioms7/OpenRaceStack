"""
Sensor Diagnostics & Fault Injection Router.
"""

from fastapi import APIRouter

from ...schemas.sensor import FaultInjectionRequest, SensorHealthItem, SensorMatrixResponse
from ...services.simulation_service import simulation_service

router = APIRouter(prefix="/sensors", tags=["Sensor Diagnostics"])


@router.get("/matrix", response_model=SensorMatrixResponse)
def get_sensor_health_matrix():
    """
    Returns sensor health statuses, message frequencies, timeout starvation counters,
    and Chi-squared outlier rejections.
    """
    snap = simulation_service.fusion.get_telemetry_snapshot()
    health_dict = snap.get("sensor_health", {})
    
    items: list[SensorHealthItem] = []
    for s_name, data in health_dict.items():
        items.append(
            SensorHealthItem(
                sensor_name=s_name,
                is_healthy=data.get("healthy", True),
                status_message=data.get("status", "NOMINAL"),
                message_rate_hz=10.0 if s_name == "gnss" else (20.0 if s_name in ["lidar", "camera"] else 100.0),
                total_updates=data.get("total_updates", 0),
                consecutive_rejections=data.get("consecutive_rejections", 0),
                total_rejections=data.get("total_rejections", 0),
                last_nis=data.get("last_nis", 0.0),
                timeout_counter=data.get("timeout_counter", 0),
            )
        )

    all_healthy = all(i.is_healthy for i in items)
    return SensorMatrixResponse(
        overall_system_healthy=all_healthy,
        primary_source=snap.get("active_source", "gnss"),
        confidence_score=round(snap.get("confidence", 0.95), 3),
        sensors=items,
        rejection_rates={i.sensor_name: round(i.total_rejections / max(1, i.total_updates), 4) for i in items}
    )


@router.post("/inject-fault")
def inject_sensor_fault(req: FaultInjectionRequest):
    """
    Simulates proving ground disturbances: GNSS outage, multipath bias, or camera dropouts.
    """
    from Localization.core.stress_replay_engine import (
        DisturbanceType,
        ScheduledDisturbance,
        StressScenarioDefinition,
    )
    
    dist_type = DisturbanceType.GNSS_OUTAGE if req.fault_type == "dropout" else DisturbanceType.URBAN_MULTIPATH
    speed = float(simulation_service.fusion.ekf.x[3, 0]) if simulation_service.fusion else 15.0
    start_s = float(simulation_service.sim_index * 0.385)
    end_s = start_s + max(10.0, speed * req.duration_sec)
    
    dist = ScheduledDisturbance(
        disturbance_type=dist_type,
        start_station_m=start_s,
        end_station_m=end_s,
        severity_factor=req.magnitude,
        description=f"Operator Injected {req.fault_type} on {req.sensor_name}"
    )
    if not simulation_service.stress_engine.active_scenario:
        simulation_service.stress_engine.active_scenario = StressScenarioDefinition(
            scenario_id="custom_fault",
            name="Custom Injected Fault Scenario",
            track_name=simulation_service.active_track_name,
            description="Dynamic fault injection",
            target_laps=3,
            target_speed_mps=speed,
            disturbances=[]
        )
    simulation_service.stress_engine.active_scenario.disturbances.append(dist)
    
    return {
        "status": "success",
        "message": f"Injected '{req.fault_type}' on '{req.sensor_name}' from station {start_s:.1f}m to {end_s:.1f}m."
    }
