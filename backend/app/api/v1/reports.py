"""
Validation Reporting & Dossier Export Router.
"""

from fastapi import APIRouter, HTTPException
from fastapi.responses import HTMLResponse, PlainTextResponse

from ...schemas.report import ReportCreate, ReportResponse
from ...services.report_service import report_service

router = APIRouter(prefix="/reports", tags=["Validation Reporting"])

# In-memory store for generated reports
_REPORTS_STORE: dict[str, ReportResponse] = {}


@router.get("", response_model=list[ReportResponse])
def list_reports():
    """Lists all compiled validation dossiers."""
    return list(_REPORTS_STORE.values())


@router.post("", response_model=ReportResponse, status_code=201)
def generate_validation_report(req: ReportCreate):
    """Generates a formal proving ground compliance report with regulatory sign-off."""
    report = report_service.generate_report(req)
    _REPORTS_STORE[report.id] = report
    return report


@router.get("/{report_id}", response_model=ReportResponse)
def get_report(report_id: str):
    """Retrieves generated report metadata and Markdown content."""
    report = _REPORTS_STORE.get(report_id)
    if not report:
        raise HTTPException(status_code=404, detail="Report not found")
    return report


@router.get("/{report_id}/download")
def download_report(report_id: str, format: str = "markdown"):
    """Downloads validation dossier in Markdown or HTML print format."""
    report = _REPORTS_STORE.get(report_id)
    if not report:
        raise HTTPException(status_code=404, detail="Report not found")

    if format == "html":
        html = f"""<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>{report.title}</title>
<style>
body {{ font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Helvetica, Arial, sans-serif; max-width: 900px; margin: 40px auto; padding: 20px; line-height: 1.6; color: #1e293b; }}
h1, h2, h3 {{ color: #0f172a; }}
table {{ width: 100%; border-collapse: collapse; margin: 20px 0; }}
th, td {{ border: 1px solid #cbd5e1; padding: 10px 14px; text-align: left; }}
th {{ background: #f8fafc; }}
code {{ background: #f1f5f9; padding: 2px 6px; border-radius: 4px; font-family: monospace; }}
.badge {{ display: inline-block; padding: 4px 10px; border-radius: 9999px; font-weight: bold; font-size: 0.85rem; }}
.badge-pass {{ background: #dcfce7; color: #166534; }}
</style>
</head>
<body>
<div style="border-bottom: 2px solid #e2e8f0; padding-bottom: 15px; margin-bottom: 25px;">
<span class="badge badge-pass">{report.verdict} VERIFIED</span>
<h2>ApexPathAI Proving Ground Certification</h2>
</div>
<pre style="white-space: pre-wrap; font-family: inherit;">{report.content_markdown}</pre>
</body>
</html>"""
        return HTMLResponse(content=html, media_type="text/html")
    else:
        return PlainTextResponse(content=report.content_markdown, media_type="text/markdown")
