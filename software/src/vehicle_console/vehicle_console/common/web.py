"""Shared HTTP security, WebSocket telemetry and installed frontend serving."""

import asyncio
import math
from pathlib import Path
from urllib.parse import urlsplit

from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from starlette.middleware.trustedhost import TrustedHostMiddleware


def create_web_app(lifespan, *, allowed_hosts=()):
    hosts = [host.strip() for host in allowed_hosts if host.strip()]
    # Explicit hosts only. Binding all interfaces must not disable Host checks.
    if any("*" in host or "/" in host or "://" in host for host in hosts):
        raise ValueError("allowed_hosts must contain explicit hostnames or IP addresses")
    app = FastAPI(lifespan=lifespan, docs_url=None, redoc_url=None, openapi_url=None)
    app.add_middleware(
        TrustedHostMiddleware,
        allowed_hosts=["127.0.0.1", "localhost", "[::1]", *hosts],
    )
    return app


async def accept_websocket(ws):
    origin = urlsplit(ws.headers.get("origin", ""))
    if origin.scheme not in ("http", "https") or origin.netloc != ws.headers.get("host"):
        await ws.close(code=1008)
        return False
    await ws.accept()
    return True


async def stream_states(send, snapshot, update_hz):
    while True:
        await send(snapshot())
        await asyncio.sleep(1.0 / update_hz)


def mount_frontend(app, static_dir):
    static_dir = Path(static_dir)
    if static_dir.is_dir():
        app.mount("/", StaticFiles(
            directory=str(static_dir), html=True, follow_symlink=True,
        ), name="web")
    else:
        @app.get("/")
        async def missing_build():
            return {"message": "Build vehicle_console/web: npm ci && npm run build, then colcon build"}


def positive_number(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise ValueError("must be a finite positive number")
    return number


def add_web_arguments(parser, *, default_port, default_hosts=""):
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=default_port)
    parser.add_argument("--allowed-hosts", default=default_hosts,
                        help="Comma-separated explicit remote Host names/IPs")
    parser.add_argument("--update-hz", type=positive_number, default=5.0)
    parser.add_argument("--static-dir")
