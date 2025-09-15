Psyche -- Live Topics Web UI
================================

This small FastAPI application provides a web UI and WebSocket endpoint to display "important topics" live.

Run locally
-----------

1. Create a virtualenv and install dependencies:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

2. Start the app:

```bash
python -m uvicorn web.app:app --host 0.0.0.0 --port 8000
```

3. Open `http://<host>:8000/` and you should see the live UI.

Zenoh integration
-----------------
If `zenoh` Python bindings are installed on the host, the app will automatically attempt to open a zenoh session and subscribe to keys specified via environment variables:

- `GNSS_IN_KEY`
- `ZENOH_GNSS_KEY`
- `ZENOH_KEYS` (comma-separated list)

When zenoh messages arrive they'll be broadcast to connected WebSocket clients.

Systemd unit (example)
----------------------
Create `/etc/systemd/system/psyche-web.service` with the following contents (adjust paths/user):

```ini
[Unit]
Description=Psyche Live Topics Web UI
After=network.target

[Service]
User=pete
Group=www-data
WorkingDirectory=/opt/psyched
Environment=PYTHONUNBUFFERED=1
ExecStart=/opt/psyched/.venv/bin/python -m uvicorn web.app:app --host 0.0.0.0 --port 8000
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

Security note: `/update` endpoint is unauthenticated in this example. Restrict access by firewall, reverse proxy, or add token-based auth for production use.
