"""Simple FastAPI app that serves a WebSocket endpoint to broadcast "important topics" live.

Run with:
    python -m uvicorn web.app:app --host 0.0.0.0 --port 8000

This module provides an API to push topic updates (POST /update) and a WebSocket endpoint
at /ws that clients connect to for live updates.
"""
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect, Form
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from typing import List
import asyncio

app = FastAPI(title="Psyche Topics Live")

templates = Jinja2Templates(directory="web/templates")
app.mount("/static", StaticFiles(directory="web/static"), name="static")


class ConnectionManager:
    def __init__(self):
        self.active: List[WebSocket] = []
        self.lock = asyncio.Lock()

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        async with self.lock:
            self.active.append(websocket)

    async def disconnect(self, websocket: WebSocket):
        async with self.lock:
            if websocket in self.active:
                self.active.remove(websocket)

    async def broadcast(self, message: str):
        async with self.lock:
            coros = [ws.send_text(message) for ws in self.active]
        if coros:
            await asyncio.gather(*coros, return_exceptions=True)


manager = ConnectionManager()

important_topics: List[str] = []


@app.get("/", response_class=HTMLResponse)
async def index(request: Request):
    # Render a simple page that opens a websocket and displays topics
    return templates.TemplateResponse("index.html", {"request": request})


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        # On connect, send the current topic list
        await websocket.send_text("INIT:" + "||".join(important_topics))
        while True:
            # Keep the connection alive; clients don't need to send data
            await websocket.receive_text()
    except WebSocketDisconnect:
        await manager.disconnect(websocket)


@app.post("/update")
async def post_update(topic: str = Form(...)):
    """HTTP endpoint to add an important topic. Accepts form field `topic`.

    Example:
        curl -X POST -F 'topic=gnss fix received' http://localhost:8000/update
    """
    important_topics.append(topic)
    # Broadcast the new topic to all connected websocket clients
    await manager.broadcast("ADD:" + topic)
    return {"status": "ok", "topic": topic}
