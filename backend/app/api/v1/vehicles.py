"""
Vehicle Management Router.
"""


from fastapi import APIRouter, HTTPException

from ...schemas.vehicle import VehicleCreate, VehicleResponse, VehicleUpdate
from ...services.simulation_service import simulation_service
from ...services.vehicle_service import vehicle_service

router = APIRouter(prefix="/vehicles", tags=["Vehicle Management"])


@router.get("", response_model=list[VehicleResponse])
def list_vehicles():
    """Lists all vehicles registered in the validation fleet catalog."""
    return vehicle_service.list_vehicles()


@router.post("", response_model=VehicleResponse, status_code=201)
def create_vehicle(data: VehicleCreate):
    """Registers a new vehicle platform into the validation fleet."""
    if vehicle_service.get_vehicle(data.id):
        raise HTTPException(status_code=400, detail=f"Vehicle with ID '{data.id}' already exists.")
    return vehicle_service.create_vehicle(data)


@router.get("/fleet-comparison")
def get_fleet_comparison():
    """Returns comparative benchmark matrix across all fleet vehicle archetypes."""
    return vehicle_service.get_fleet_comparison()


@router.get("/{vehicle_id}", response_model=VehicleResponse)
def get_vehicle(vehicle_id: str):
    """Retrieves vehicle specifications and sensor extrinsics."""
    v = vehicle_service.get_vehicle(vehicle_id)
    if not v:
        raise HTTPException(status_code=404, detail="Vehicle not found")
    return v


@router.put("/{vehicle_id}", response_model=VehicleResponse)
def update_vehicle(vehicle_id: str, data: VehicleUpdate):
    """Updates vehicle parameters and configuration."""
    updated = vehicle_service.update_vehicle(vehicle_id, data)
    if not updated:
        raise HTTPException(status_code=404, detail="Vehicle not found")
    return updated


@router.delete("/{vehicle_id}")
def delete_vehicle(vehicle_id: str):
    """Deletes a vehicle asset from catalog."""
    success = vehicle_service.delete_vehicle(vehicle_id)
    if not success:
        raise HTTPException(status_code=404, detail="Vehicle not found")
    return {"status": "success", "message": f"Vehicle {vehicle_id} deleted"}


@router.post("/{vehicle_id}/select")
def select_active_vehicle(vehicle_id: str):
    """Sets the designated vehicle asset as the active test platform."""
    v = vehicle_service.get_vehicle(vehicle_id)
    if not v:
        raise HTTPException(status_code=404, detail="Vehicle not found")
    simulation_service.select_vehicle(v.id, wheelbase_m=v.wheelbase_m)
    return {"status": "success", "active_vehicle": v.id}
