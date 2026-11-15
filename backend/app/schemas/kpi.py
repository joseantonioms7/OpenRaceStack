"""
KPI Analysis & Scorecard Schemas.
"""

from typing import Any

from pydantic import BaseModel


class KPIRuleResult(BaseModel):
    rule_id: str
    rule_name: str
    threshold: str
    measured_value: str
    passed: bool
    status: str  # PASS, CONDITIONAL, FAIL
    description: str


class KPIScorecardSchema(BaseModel):
    session_id: str
    track_name: str
    profile: str
    overall_verdict: str  # PASS, CONDITIONAL, FAIL
    compliance_score_pct: float
    rules: list[KPIRuleResult]
    summary_metrics: dict[str, Any]
