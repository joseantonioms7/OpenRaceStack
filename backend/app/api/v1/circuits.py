"""
Circuit Management Router.
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from ...schemas.circuit import CircuitDetail, CircuitSummary
from ...services.circuit_service import circuit_service
from ...services.simulation_service import simulation_service

router = APIRouter(prefix="/circuits", tags=["Circuit Management"])


class SelectCircuitPayload(BaseModel):
    circuit_name: str


@router.get("", response_model=list[CircuitSummary])
def list_circuits():
    """Lists all 23 available international Grand Prix proving ground circuits."""
    return circuit_service.get_all_circuits(active_track=simulation_service.active_track_name)


@router.get("/{track_name}", response_model=CircuitDetail)
def get_circuit_details(track_name: str):
    """Retrieves full geometry, waypoint coordinates, and racelines for a circuit."""
    detail = circuit_service.get_circuit_detail(track_name)
    if not detail:
        raise HTTPException(status_code=404, detail=f"Circuit '{track_name}' not found.")
    detail.is_active = (detail.name.lower() == simulation_service.active_track_name.lower())
    return detail


@router.get("/{track_name}/evaluate")
def evaluate_circuit(track_name: str):
    """Returns proving ground evaluation metrics, difficulty ratings, and sector splits."""
    eval_res = circuit_service.evaluate_circuit(track_name)
    if not eval_res:
        raise HTTPException(status_code=404, detail=f"Circuit '{track_name}' not found.")
    return eval_res


@router.get("/{track_name}/vector-map")
def get_circuit_vector_map(track_name: str):
    """Returns full sub-centimeter HD Vector Map GeoJSON with centerline, boundaries, and sectors."""
    v_map = circuit_service.get_hd_vector_map(track_name)
    if not v_map:
        raise HTTPException(status_code=404, detail=f"HD Vector map for circuit '{track_name}' not found.")
    return v_map


@router.post("/select")
def select_active_circuit(payload: SelectCircuitPayload):
    """Dynamically loads and switches the active proving ground circuit."""
    success = simulation_service.select_circuit(payload.circuit_name)
    if not success:
        raise HTTPException(status_code=400, detail=f"Could not load circuit '{payload.circuit_name}'.")
    return {
        "status": "success",
        "active_circuit": simulation_service.active_track_name,
        "message": f"Successfully switched to {simulation_service.active_track_name}"
    }
