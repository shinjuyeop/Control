"""Existing Web browser fixture shared by both profiles."""
from contextlib import contextmanager
from pathlib import Path
import pytest

@contextmanager
def browser_session(client):
    playwright = pytest.importorskip("playwright.sync_api")
    import shutil
    import socket
    import uvicorn
    from fastapi.staticfiles import StaticFiles

    chrome = shutil.which("google-chrome")
    if chrome is None:
        pytest.skip("Chrome is not installed")
    static_dir = Path(__file__).parents[1] / "web/dist"
    if not (static_dir / "index.html").is_file():
        pytest.skip("Build the React console before the browser test")
    client.app.router.routes = [
        route
        for route in client.app.router.routes
        if getattr(route, "path", None) != "/"
    ]
    client.app.mount("/", StaticFiles(directory=static_dir, html=True))
    sock = socket.socket()
    sock.bind(("127.0.0.1", 0))
    address = f"http://127.0.0.1:{sock.getsockname()[1]}"
    server = uvicorn.Server(
        uvicorn.Config(client.app, lifespan="off", log_level="warning")
    )

    async def serve():
        await server.serve(sockets=[sock])

    task = client.portal.start_task_soon(serve)
    try:
        with playwright.sync_playwright() as browser_tools:
            browser = browser_tools.chromium.launch(
                executable_path=chrome, headless=True
            )
            try:
                page = browser.new_page(viewport={"width": 1440, "height": 1000})
                errors = []
                page.on("pageerror", lambda error: errors.append(str(error)))
                page.goto(address)
                yield page
                assert not errors
            finally:
                browser.close()
    finally:
        server.should_exit = True
        task.result(timeout=12)
        sock.close()
