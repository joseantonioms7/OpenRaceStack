"""
Application Configuration and Environment Settings.
Supports PostgreSQL, Redis, and local development fallbacks.
"""

import os
from pathlib import Path

try:
    from pydantic import Field
    from pydantic_settings import BaseSettings, SettingsConfigDict
except ImportError:
    from pydantic import BaseModel as BaseSettings  # type: ignore
    from pydantic import Field
    SettingsConfigDict = dict  # type: ignore


class Settings(BaseSettings):
    PROJECT_NAME: str = "ApexPathAI AV Validation Platform"
    PROJECT_SLOGAN: str = "Enterprise Autonomous Vehicle Localization & Proving Ground Verification"
    VERSION: str = "2.0.0"
    API_V1_PREFIX: str = "/api/v1"
    
    # Server configuration
    HOST: str = "0.0.0.0"
    PORT: int = 8000
    DEBUG: bool = False
    CORS_ORIGINS: list[str] = ["*"]
    
    # Storage & Cache Configuration
    # Uses SQLite asynchronously for zero-config local onboarding, or PostgreSQL in production/Docker
    DATABASE_URL: str = Field(
        default="sqlite+aiosqlite:///./apexpath_validation.db",
        description="Async database connection string (PostgreSQL or SQLite)"
    )
    REDIS_URL: str = Field(
        default="redis://localhost:6379/0",
        description="Redis connection URL for pub/sub telemetry streaming"
    )
    REDIS_ENABLED: bool = False  # Enabled automatically if Redis is reachable
    
    # Proving Ground & Autonomous Simulation Parameters
    BASE_DIR: Path = Path(__file__).resolve().parent.parent.parent.parent
    DATASETS_DIR: Path = Path(__file__).resolve().parent.parent.parent.parent / "Localization" / "datasets"
    DEFAULT_CIRCUIT: str = "Monza"
    DEFAULT_VEHICLE_ID: str = "apex-f1tenth-01"
    SIMULATION_DT: float = 0.05  # 20 Hz (50 ms execution epoch)
    TELEMETRY_BROADCAST_HZ: float = 20.0
    
    # Mapbox & Visualization
    MAPBOX_ACCESS_TOKEN: str = os.getenv("MAPBOX_ACCESS_TOKEN", "")
    
    model_config = SettingsConfigDict(env_file=".env", extra="ignore")


settings = Settings()
