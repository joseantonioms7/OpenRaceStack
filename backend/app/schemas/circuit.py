"""
Circuit Pydantic Schemas.
"""


from pydantic import BaseModel, ConfigDict, Field


class WaypointPoint(BaseModel):
    x: float
    y: float
    s: float | None = None
    psi: float | None = None
    kappa: float | None = None
    vx: float | None = None


class CircuitSummary(BaseModel):
    id: str
    name: str
    country: str = "International"
    length_m: float
    width_m: float
    waypoints_count: int
    has_raceline: bool
    has_raster_map: bool
    is_active: bool = False

    model_config = ConfigDict(from_attributes=True)


class CircuitDetail(CircuitSummary):
    resolution_m: float = 0.05
    origin_x: float = 0.0
    origin_y: float = 0.0
    centerline_preview: list[list[float]] = Field(default_factory=list, description="Downsampled 2D coordinates [x, y]")
    raceline_preview: list[list[float]] = Field(default_factory=list, description="Downsampled 2D raceline [x, y]")
