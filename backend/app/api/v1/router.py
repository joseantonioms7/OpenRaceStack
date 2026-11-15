"""
Master API v1 Router.
Aggregates all 10 core autonomous vehicle validation pillars.
"""

from fastapi import APIRouter

from .analytics import router as analytics_router
from .circuits import router as circuits_router
from .kpis import router as kpis_router
from .monitoring import router as monitoring_router
from .replay import router as replay_router
from .reports import router as reports_router
from .sensors import router as sensors_router
from .sessions import router as sessions_router
from .vehicles import router as vehicles_router

api_router = APIRouter()

api_router.include_router(monitoring_router)
api_router.include_router(vehicles_router)
api_router.include_router(circuits_router)
api_router.include_router(sessions_router)
api_router.include_router(analytics_router)
api_router.include_router(sensors_router)
api_router.include_router(replay_router)
api_router.include_router(kpis_router)
api_router.include_router(reports_router)
