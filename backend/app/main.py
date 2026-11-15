"""
ApexPathAI Autonomous Vehicle Validation Platform — Backend Application.
"""

import logging
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles

from .api.v1.router import api_router
from .core.config import settings
from .core.database import init_db
from .core.redis import broker
from .services.simulation_service import simulation_service

# Configure structured logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("apexpath.backend")


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Lifecycle startup and teardown manager."""
    logger.info("Initializing ApexPathAI Autonomous Vehicle Validation Platform v%s...", settings.VERSION)
    
    # 1. Initialize Database Tables
    init_db()
    
    # 2. Connect Redis Broker
    await broker.connect()
    
    # 3. Start Autonomous 20 Hz Simulation Service
    await simulation_service.start()
    
    logger.info("ApexPathAI Backend services fully operational on port %d.", settings.PORT)
    yield
    
    # Shutdown sequence
    logger.info("Shutting down ApexPathAI simulation and broker connections...")
    await simulation_service.stop()


app = FastAPI(
    title=settings.PROJECT_NAME,
    description="""
# ApexPathAI Autonomous Vehicle Validation Platform API

Enterprise platform for autonomous driving proving ground verification, real-time telemetry streaming,
and regulatory safety compliance (ISO 26262 ASIL-D, Euro NCAP).

### 10 Core Pillars:
1. **Real-Time Monitoring:** 20 Hz low-latency WebSocket stream of vehicle pose, dynamics, and actuators.
2. **Vehicle Management:** Multi-vehicle fleet registry (F1TENTH, IndyCar AV-21, Robotaxi).
3. **Circuit Management:** 23 Grand Prix proving ground tracks with centimeter-grade racelines.
4. **Test Session Management:** State machine campaign lifecycle and sector lap timing.
5. **Localization Analytics:** Error-State Kalman Filter (15-DOF ESKF) vs. CTRV EKF comparative metrics.
6. **Vehicle Analytics:** Frenet frame cross-track errors, friction circle GG-diagrams, and MPC commands.
7. **Sensor Diagnostics:** Real-time health supervisor, Chi-squared NIS gating, and disturbance injection.
8. **Replay System:** Time-indexed frame recorder with seek scrubber and incident bookmarking.
9. **KPI Analysis:** Automated pass/fail verification against 7 core proving ground safety rules.
10. **Reporting:** Exportable regulatory dossiers in Markdown, HTML, and JSON.
""",
    version=settings.VERSION,
    lifespan=lifespan,
    docs_url="/docs",
    redoc_url="/redoc",
)

# CORS Middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.CORS_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Mount API v1 Routes
app.include_router(api_router, prefix=settings.API_V1_PREFIX)


@app.get("/health", tags=["System Health"])
def health_check():
    """Service health and autonomous stack status check."""
    return {
        "status": "healthy",
        "service": settings.PROJECT_NAME,
        "version": settings.VERSION,
        "active_circuit": simulation_service.active_track_name,
        "active_vehicle": simulation_service.active_vehicle_id,
        "simulation_running": simulation_service.is_running and not simulation_service.is_paused,
        "estop_active": simulation_service.estop_active,
    }


# Static Frontend Mounting (when built)
frontend_dist = Path(__file__).resolve().parent.parent.parent / "frontend" / "dist"
if frontend_dist.is_dir():
    app.mount("/", StaticFiles(directory=str(frontend_dist), html=True), name="frontend")
else:
    @app.get("/", include_in_schema=False)
    def root_fallback():
        return HTMLResponse(
            f"""<!DOCTYPE html>
<html>
<head>
    <title>{settings.PROJECT_NAME}</title>
    <style>
        body {{ font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; background: #0f172a; color: #f8fafc; display: flex; align-items: center; justify-content: center; height: 100vh; margin: 0; }}
        .card {{ background: #1e293b; padding: 40px; border-radius: 12px; border: 1px solid #334155; max-width: 650px; text-align: center; box-shadow: 0 20px 25px -5px rgba(0,0,0,0.5); }}
        h1 {{ color: #38bdf8; margin-top: 0; }}
        a {{ color: #38bdf8; text-decoration: none; font-weight: bold; }}
        .btn {{ display: inline-block; background: #0284c7; color: white; padding: 12px 24px; border-radius: 6px; margin: 10px 5px; }}
        .btn:hover {{ background: #0369a1; }}
        .badge {{ background: #065f46; color: #34d399; padding: 4px 10px; border-radius: 9999px; font-size: 0.8rem; font-weight: bold; }}
    </style>
</head>
<body>
    <div class="card">
        <span class="badge">API ACTIVE & HEALTHY</span>
        <h1>ApexPathAI v{settings.VERSION}</h1>
        <p>Autonomous Vehicle Localization, State Estimation & Proving Ground Validation Platform.</p>
        <div style="margin: 30px 0;">
            <a href="/docs" class="btn">Explore Interactive API Docs</a>
            <a href="/health" class="btn" style="background: #334155;">Health Check</a>
        </div>
        <p style="color: #94a3b8; font-size: 0.9rem;">
            Run frontend in <code>frontend/</code> via <code>npm run dev</code> or launch full multi-container stack with <code>docker compose up</code>.
        </p>
    </div>
</body>
</html>"""
        )


def start():
    """CLI entry point for running the ApexPathAI validation server."""
    import uvicorn
    uvicorn.run("backend.app.main:app", host=settings.HOST, port=settings.PORT, reload=settings.DEBUG)


if __name__ == "__main__":
    start()
