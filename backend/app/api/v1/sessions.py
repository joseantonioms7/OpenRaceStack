"""
Test Session Management Router.
"""

from fastapi import APIRouter, HTTPException

from ...schemas.session import SessionCreate, SessionDetail, SessionResponse, SessionTransition
from ...services.session_service import session_service

router = APIRouter(prefix="/sessions", tags=["Test Session Management"])


@router.get("", response_model=list[SessionResponse])
def list_sessions():
    """Lists all historical and ongoing proving ground test campaigns."""
    return session_service.list_sessions()


@router.post("", response_model=SessionResponse, status_code=201)
def create_session(data: SessionCreate):
    """Initializes a new proving ground validation campaign."""
    return session_service.create_session(data)


@router.get("/active", response_model=SessionDetail | None)
def get_active_session():
    """Returns currently armed or executing session."""
    active = session_service.get_active_session()
    return active


@router.get("/{session_id}", response_model=SessionDetail)
def get_session_by_id(session_id: str):
    """Retrieves session details, sector times, and pass/fail verdicts."""
    sess = session_service.get_session(session_id)
    if not sess:
        raise HTTPException(status_code=404, detail="Session not found")
    return sess


@router.post("/{session_id}/transition", response_model=SessionResponse)
def transition_session(session_id: str, payload: SessionTransition):
    """Transitions session through lifecycle (arm, start, pause, stop, abort)."""
    updated = session_service.transition_session(payload.action)
    if not updated:
        raise HTTPException(status_code=400, detail="Could not transition session state.")
    return updated
