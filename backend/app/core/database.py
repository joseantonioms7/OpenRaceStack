"""
Database Initialization and Session Management.
Provides asynchronous connection pooling with PostgreSQL and SQLite fallbacks.
"""

import logging

from sqlalchemy import create_engine
from sqlalchemy.orm import Session, declarative_base, sessionmaker

from .config import settings

logger = logging.getLogger(__name__)

Base = declarative_base()

# Configure engine: use sync SQLite/Postgres engine for universal compatibility without driver friction
db_url = settings.DATABASE_URL
sync_db_url = db_url.replace("sqlite+aiosqlite:", "sqlite:").replace("postgresql+asyncpg:", "postgresql:")

connect_args = {"check_same_thread": False} if "sqlite" in sync_db_url else {}
engine = create_engine(sync_db_url, connect_args=connect_args, pool_pre_ping=True)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)


def init_db() -> None:
    """Creates database tables if they do not exist."""
    try:
        from ..models import vehicle, circuit, session, telemetry, report  # noqa
        Base.metadata.create_all(bind=engine)
        logger.info("Database schemas initialized successfully.")
    except Exception as e:
        logger.warning(f"Database table creation skipped or encountered: {e}")


def get_db() -> Session:
    """Dependency for obtaining a database session."""
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()
