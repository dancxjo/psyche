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
from typing import List, Optional
import asyncio
import os
import logging
import json
import time

logger = logging.getLogger("psyche.web")
logging.basicConfig(level=logging.INFO)

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

# Track zenoh keys and their last seen payload/timestamp
zenoh_status: dict = {}


# Attempt to import zenoh-python and, if present, subscribe to configured keys
try:
    import zenoh
    ZENOH_AVAILABLE = True
    logger.info("zenoh-python available: automatic subscription enabled")
except Exception:
    zenoh = None
    ZENOH_AVAILABLE = False
    logger.info("zenoh-python not available: skipping automatic zenoh subscriptions")


async def zenoh_subscriber_loop():
    """If zenoh is available, open a session and subscribe to keys configured
    via environment variables. This runs as a background task started at app startup.
    Environment variables scanned (in order): `GNSS_IN_KEY`, `ZENOH_GNSS_KEY`, `ZENOH_KEYS`.
    `ZENOH_KEYS` may be a comma-separated list of keys.
    """
    if not ZENOH_AVAILABLE:
        return

    # collect keys from environment
    keys = []
    if os.getenv('GNSS_IN_KEY'):
        keys.append(os.getenv('GNSS_IN_KEY'))
    if os.getenv('ZENOH_GNSS_KEY'):
        keys.append(os.getenv('ZENOH_GNSS_KEY'))
    if os.getenv('ZENOH_KEYS'):
        keys += [k.strip() for k in os.getenv('ZENOH_KEYS').split(',') if k.strip()]
    if not keys:
        logger.info('No zenoh keys configured in environment; skipping subscription')
        return

    # initialize status entries
    for k in keys:
        zenoh_status[k] = {"last": None, "ts": None}

    # broadcast initial zenoh statuses to any connected clients
    try:
        await manager.broadcast("ZENOH_INIT:" + json.dumps({"keys": keys, "status": zenoh_status}))
    except Exception:
        logger.exception('Failed to broadcast zenoh init')

    try:
        z = zenoh.open()  # defaults; tweak via ZENOH_CONFIG env if needed
        logger.info('zenoh session opened')
    except Exception as e:
        logger.exception('Failed to open zenoh session: %s', e)
        return

    # callback for incoming samples
    def sample_cb(sample):
        try:
            # sample.payload is bytes-like; decode if possible
            payload = None
            try:
                payload = sample.payload.decode('utf-8')
            except Exception:
                payload = str(sample.payload)

            # schedule async handler to update status and broadcast JSON
            async def _handle():
                try:
                    zenoh_status[sample.key] = {"last": payload, "ts": time.time()}
                    payload_obj = {"key": sample.key, "payload": payload, "ts": zenoh_status[sample.key]["ts"]}
                    await manager.broadcast("ZENOH_UPD:" + json.dumps(payload_obj))
                except Exception:
                    logger.exception('error handling zenoh sample async')

            asyncio.get_event_loop().create_task(_handle())
        except Exception:
            logger.exception('error handling zenoh sample')

    # subscribe to keys
    subs = []
    for key in keys:
        try:
            subs.append(z.declare_subscriber(key, sample_cb))
            logger.info('Subscribed to zenoh key: %s', key)
        except Exception:
            logger.exception('Failed to subscribe to zenoh key: %s', key)

    # keep alive until the application is shutdown
    try:
        while True:
            await asyncio.sleep(60)
    finally:
        for s in subs:
            try:
                s.undeclare()
            except Exception:
                pass
        try:
            z.close()
        except Exception:
            pass


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
