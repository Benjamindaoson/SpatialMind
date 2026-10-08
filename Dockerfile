# Portable local mission-control deployment; not a ROS/Gazebo runtime.
FROM python:3.11-slim
ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    SPATIALMIND_DATA_DIR=/data
WORKDIR /app
COPY pyproject.toml README.md LICENSE ./
COPY src ./src
RUN pip install --no-cache-dir ".[api]" \
    && useradd --system --uid 10001 --create-home spatialmind \
    && mkdir -p /data \
    && chown spatialmind:spatialmind /data
USER spatialmind
VOLUME ["/data"]
EXPOSE 8000
HEALTHCHECK --interval=30s --timeout=5s --retries=3 \
    CMD python -c "import os; from urllib.request import urlopen; urlopen('http://127.0.0.1:'+os.getenv('SPATIALMIND_HEALTH_PORT','8000')+'/healthz', timeout=3)"
CMD ["uvicorn", "spatialmind.api:app", "--host", "0.0.0.0", "--port", "8000"]
