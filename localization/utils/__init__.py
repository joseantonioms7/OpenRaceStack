"""
ApexPathAI - Localization & Validation Utilities.
Provides racetrack CSV/raceline dataset loaders, geodetic & Frenet metric evaluators.
"""

from .metrics import (
    calculate_advanced_metrics,
    generate_multi_track_report,
    generate_performance_report,
)
from .track_loader import TrackLoader

__all__ = [
    "TrackLoader",
    "calculate_advanced_metrics",
    "generate_performance_report",
    "generate_multi_track_report",
]
