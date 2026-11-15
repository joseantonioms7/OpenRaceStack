"""
Localization & Vehicle Analytics Router.
"""

from fastapi import APIRouter

from ...schemas.analytics import AnalyticsSummarySchema, NavigationQualitySchema
from ...services.analytics_service import analytics_service

router = APIRouter(prefix="/analytics", tags=["Analytics & Localization Metrics"])


@router.get("/summary", response_model=AnalyticsSummarySchema)
def get_analytics_summary(max_points: int = 150):
    """
    Returns statistical localization error distributions, 3-sigma covariance bounds,
    and longitudinal/lateral vehicle dynamics samples.
    """
    return analytics_service.get_analytics_summary(max_points=max_points)


@router.get("/navigation", response_model=NavigationQualitySchema)
def get_navigation_quality(max_points: int = 150):
    """
    Returns Frenet lateral deviation, heading alignment, Stanford control tracking error plane,
    and actuator saturation rates.
    """
    return analytics_service.get_navigation_quality(max_points=max_points)
