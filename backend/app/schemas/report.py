"""
Validation Dossier & Report Schemas.
"""

from datetime import datetime
from typing import Any

from pydantic import BaseModel, ConfigDict, Field


class ReportCreate(BaseModel):
    session_id: str
    title: str | None = None
    author: str | None = "Autonomous Safety Architect"
    format: str = Field(default="markdown", json_schema_extra={"example": "markdown"}, description="markdown, json, html")


class ReportResponse(BaseModel):
    id: str
    session_id: str
    title: str
    author: str
    verdict: str
    summary: str
    content_markdown: str
    data_snapshot: dict[str, Any]
    created_at: datetime

    model_config = ConfigDict(from_attributes=True)
