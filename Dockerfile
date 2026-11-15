# Multi-Stage Production Dockerfile for ApexPathAI
FROM python:3.11-slim AS builder

WORKDIR /app

# Install build essentials
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Install dependencies into wheels cache
COPY pyproject.toml requirements.txt ./
RUN pip install --upgrade pip && \
    pip wheel --no-cache-dir --no-deps --wheel-dir /app/wheels -r requirements.txt

# Final Runtime Image
FROM python:3.11-slim AS runner

WORKDIR /app

# Create non-root system user
RUN groupadd -r apexpath && useradd -r -g apexpath apexpath

# Install wheels from builder
COPY --from=builder /app/wheels /wheels
RUN pip install --no-cache-dir /wheels/* && rm -rf /wheels

# Copy application source and datasets
COPY pyproject.toml README.md LICENSE NOTICE ./
COPY Localization ./Localization
COPY backend ./backend

# Install ApexPathAI in editable mode
RUN pip install --no-cache-dir -e . --no-deps

# Set permissions
RUN chown -R apexpath:apexpath /app
USER apexpath

EXPOSE 8000

ENV PYTHONUNBUFFERED=1 \
    PORT=8000 \
    HOST=0.0.0.0

CMD ["python", "-m", "uvicorn", "backend.app.main:app", "--host", "0.0.0.0", "--port", "8000"]
