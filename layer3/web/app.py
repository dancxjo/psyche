"""Simple FastAPI app to list ROS2 topics and stream topic data via WebSockets.

This implementation uses the `ros2` CLI via subprocess to list and echo topics
to avoid deep imports of every message type. It's a pragmatic approach that
works on systems where `ros2` is available in PATH (we source the ROS workspace
in the systemd unit before starting the server).
"""
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse
import asyncio
import shlex
from collections import defaultdict, deque
from typing import Dict, Set
import subprocess

app = FastAPI()

# Per-topic history (text lines) and connected websocket sets
HISTORY_LIMIT = 200
topic_history: Dict[str, deque] = defaultdict(lambda: deque(maxlen=HISTORY_LIMIT))
topic_clients: Dict[str, Set[WebSocket]] = defaultdict(set)
topic_tasks: Dict[str, asyncio.Task] = {}


async def ros2_echo_task(topic: str):
    # Start subprocess that echoes the topic in python-style key=value pairs (-p)
    proc = await asyncio.create_subprocess_exec('ros2', 'topic', 'echo', topic, '-p', stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.DEVNULL)
    try:
        while True:
            line = await proc.stdout.readline()
            if not line:
                await asyncio.sleep(0.1)
                continue
            text = line.decode('utf-8', errors='ignore')
            # store history
            topic_history[topic].append(text)
            # broadcast to connected websockets
            clients = list(topic_clients.get(topic, []))
            for ws in clients:
                try:
                    await ws.send_text(text)
                except Exception:
                    # ignore send errors; cleanup happens elsewhere
                    pass
    except asyncio.CancelledError:
        proc.kill()
        raise
    finally:
        try:
            proc.kill()
        except Exception:
            pass


@app.get('/topics')
async def list_topics():
    """Return a list of topics (names only)."""
    try:
        out = subprocess.check_output(['ros2', 'topic', 'list'], stderr=subprocess.DEVNULL)
        lines = out.decode('utf-8').splitlines()
        return JSONResponse(content={'topics': lines})
    except subprocess.CalledProcessError:
        return JSONResponse(content={'topics': []})


@app.get('/topic/{topic_name}/history')
async def topic_history_api(topic_name: str):
    hist = list(topic_history.get(topic_name, []))
    return JSONResponse(content={'topic': topic_name, 'history': hist})


@app.websocket('/ws/topic/{topic_name}')
async def websocket_topic(ws: WebSocket, topic_name: str):
    await ws.accept()
    # Register client
    topic_clients[topic_name].add(ws)
    # Start background ros2 echo if not running
    if topic_name not in topic_tasks:
        loop = asyncio.get_event_loop()
        topic_tasks[topic_name] = loop.create_task(ros2_echo_task(topic_name))
    # Send recent history on connect
    try:
        for line in topic_history.get(topic_name, []):
            await ws.send_text(line)
        while True:
            # Keep connection alive; actual sends come from background task
            await asyncio.sleep(1)
    except WebSocketDisconnect:
        pass
    finally:
        # Cleanup
        try:
            topic_clients[topic_name].discard(ws)
        except Exception:
            pass
        # If no more clients, cancel background task
        if not topic_clients[topic_name]:
            task = topic_tasks.pop(topic_name, None)
            if task is not None:
                task.cancel()
