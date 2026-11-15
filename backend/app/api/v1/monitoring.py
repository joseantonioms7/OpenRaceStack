"""
Real-Time Monitoring Router.
Exposes 20 Hz WebSocket telemetry streaming and emergency stop control.
"""

import logging
from typing import Any

from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from pydantic import BaseModel

from ...core.redis import broker
from ...services.simulation_service import simulation_service

router = APIRouter(prefix="/monitoring", tags=["Real-Time Monitoring"])
logger = logging.getLogger(__name__)


class EStopPayload(BaseModel):
    active: bool


@router.websocket("/ws")
async def websocket_telemetry_stream(websocket: WebSocket):
    """
    Subscribes client to real-time 20 Hz telemetry stream broadcast over Redis/In-Memory broker.
    """
    await websocket.accept()
    queue = await broker.subscribe("telemetry")
    logger.info("Client connected to telemetry WebSocket stream.")

    try:
        while True:
            # Pop next frame from broker queue and send to WebSocket client
            payload = await queue.get()
            await websocket.send_text(payload)
    except WebSocketDisconnect:
        logger.info("Client disconnected from telemetry WebSocket.")
    except Exception as e:
        logger.warning("WebSocket stream error: %s", e)
    finally:
        await broker.unsubscribe("telemetry", queue)


@router.get("/snapshot")
def get_current_telemetry_snapshot() -> dict[str, Any]:
    """Returns the most recent single telemetry frame."""
    if simulation_service.recent_frames:
        return simulation_service.recent_frames[-1]
    return simulation_service._step_simulation()


@router.post("/estop")
def trigger_emergency_stop(payload: EStopPayload) -> dict[str, Any]:
    """Actuates or releases vehicle Emergency Stop."""
    state = simulation_service.set_estop(payload.active)
    return {
        "status": "success",
        "estop_active": state,
        "message": "EMERGENCY STOP ACTUATED" if state else "E-Stop Released. Nominal Operations."
    }


@router.post("/reset-estop")
def reset_emergency_stop() -> dict[str, Any]:
    """Clears emergency stop and re-enables nominal autonomous guidance."""
    state = simulation_service.set_estop(False)
    return {
        "status": "success",
        "estop_active": state,
        "message": "Emergency stop cleared. Autonomous system ready."
    }


@router.get("/safety-events")
def get_safety_events() -> dict[str, Any]:
    """Returns log of safety interventions, disengagements, and boundary alarms."""
    events = simulation_service.get_safety_events()
    return {
        "total_events": len(events),
        "estop_active": simulation_service.estop_active,
        "events": events
    }
