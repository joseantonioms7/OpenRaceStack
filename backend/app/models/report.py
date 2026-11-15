"""
Validation Report ORM Model.
Persists generated regulatory dossiers and safety scorecards.
"""

from datetime import datetime

from sqlalchemy import JSON, Column, DateTime, ForeignKey, String, Text

from ..core.database import Base


class ValidationReportModel(Base):
    __tablename__ = "validation_reports"

    id = Column(String(64), primary_key=True, index=True)
    session_id = Column(String(64), ForeignKey("test_sessions.id"), index=True)
    title = Column(String(256), nullable=False)
    author = Column(String(128), default="ApexPathAI Automated Verifier")
    verdict = Column(String(32), default="PASS")
    summary = Column(Text, nullable=True)
    content_markdown = Column(Text, nullable=True)
    data_snapshot = Column(JSON, nullable=True)
    created_at = Column(DateTime, default=datetime.utcnow)
