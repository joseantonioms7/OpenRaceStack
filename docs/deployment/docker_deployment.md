# Production Deployment with Docker Compose 🐳

ApexPathAI provides fully containerized deployments utilizing Docker multi-stage builds and Docker Compose.

---

## Architecture Topology

The standard production deployment orchestrates four decoupled microservices:

| Service | Image Base | Internal Port | External Port | Purpose |
| :--- | :--- | :---: | :---: | :--- |
| **`frontend`** | `node:20-alpine` $\to$ `nginx:alpine` | 80 | **3000** | React 18 SPA with Nginx reverse proxy |
| **`backend`** | `python:3.11-slim` | 8000 | **8000** | FastAPI ASGI core & 20 Hz WebSocket broadcaster |
| **`db`** | `postgres:16-alpine` | 5432 | 5432 | Persistent relational storage (Sessions, Fleet, Reports) |
| **`redis`** | `redis:7-alpine` | 6379 | 6379 | High-throughput pub/sub telemetry cache |

---

## 🚀 Quick Deployment Guide

### 1. Configure Environment Variables

Create a production `.env` file based on `.env.example`:

```bash
cp .env.example .env
```

Key environment settings:

```dotenv
# Database Configuration
POSTGRES_USER=apexpath
POSTGRES_PASSWORD=apexpath_secret
POSTGRES_DB=apexpath_validation
DATABASE_URL=postgresql+asyncpg://apexpath:apexpath_secret@db:5432/apexpath_validation

# Redis Pub/Sub Telemetry Broker
REDIS_URL=redis://redis:6379/0
REDIS_ENABLED=true

# Server Settings
HOST=0.0.0.0
PORT=8000
DEBUG=false
CORS_ORIGINS=["http://localhost:3000", "http://127.0.0.1:3000"]

# Optional Mapbox Access Token
MAPBOX_ACCESS_TOKEN=pk.your_mapbox_token_here
```

### 2. Build and Launch Containers

```bash
# Build multi-stage images and start in background
docker compose up --build -d
```

Verify service status:

```bash
docker compose ps
```

Expected output:
```text
NAME                  IMAGE                  COMMAND                  SERVICE    STATUS              PORTS
apexpath-backend      apexpath-backend       "uvicorn backend.app…"   backend    running (healthy)   0.0.0.0:8000->8000/tcp
apexpath-frontend     apexpath-frontend      "/docker-entrypoint.…"   frontend   running             0.0.0.0:3000->80/tcp
apexpath-postgres     postgres:16-alpine     "docker-entrypoint.s…"   db         running (healthy)   0.0.0.0:5432->5432/tcp
apexpath-redis        redis:7-alpine         "docker-entrypoint.s…"   redis      running (healthy)   0.0.0.0:6379->6379/tcp
```

### 3. Monitoring Logs

```bash
# Tail live telemetry broadcast logs
docker compose logs -f backend

# Tail web server access logs
docker compose logs -f frontend
```

### 4. Stopping and Teardown

```bash
# Graceful stop
docker compose down

# Stop and purge database volumes (resets proving ground state)
docker compose down -v
```

---

## 🔒 Production Security Hardening

When deploying on public or corporate proving ground networks:

1. **TLS / HTTPS Termination:** Configure an edge load balancer or reverse proxy (e.g., Traefik, AWS ALB, Cloudflare) with valid SSL certificates.
2. **CORS Restrictions:** Set `CORS_ORIGINS` in `.env` to match your exact production domain.
3. **Database Secrets:** Generate a cryptographically strong database password (`openssl rand -hex 24`).
4. **Physical E-STOP Interlock:** Always enforce hardware-level interlocks on physical vehicles alongside software emergency stops (`POST /api/v1/monitoring/estop`).
