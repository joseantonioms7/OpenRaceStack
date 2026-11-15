"""
KPI Analysis & Proving Ground Verification Router.
"""


from fastapi import APIRouter

from ...schemas.kpi import KPIScorecardSchema
from ...services.report_service import report_service

router = APIRouter(prefix="/kpis", tags=["KPI Analysis & Scorecard"])


@router.get("/scorecard", response_model=KPIScorecardSchema)
def get_kpi_scorecard(session_id: str | None = None):
    """
    Evaluates proving ground regulatory scorecard against the 7 core safety rules:
    RMSE bounds, peak ceilings, corridor containment, attitude tracking, latency,
    outlier rejections, and sensor dropout recovery.
    """
    return report_service.evaluate_kpi_scorecard(session_id=session_id)
