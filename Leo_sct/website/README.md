Edit robot topic namespaces in `robots.json`:

```json
{
  "robots": [
    { "name": "Leo-1", "ns": "robot_0" },
    { "name": "Leo-2", "ns": "robot_1" },
    { "name": "Leo-3", "ns": "robot_2" }
  ]
}
```

Run:

```bash
ros2 run rosbridge_server rosbridge_websocket
# Optional for fully local QR generation:
# python3 -m pip install qrcode[pil]
python3 website/server.py --host 0.0.0.0 --port 8080
```

In browser:

```text
http://localhost:8080
```

Session behavior:

- Phone/LAN clients stay active without a session timeout.
- Local loopback (`localhost` / `127.0.0.1`) keeps `Emergency Stop All`.
- Only one web client can control each robot at a time (server-side lease lock).
- Local `Emergency Stop All` activates a global emergency latch: remote clients are force-blocked.

Phone QR:

- Website shows a QR card with the phone URL.
- If Python `qrcode` is installed, QR is generated locally via `/api/qr`.
- Falls back to online QR service if local qrcode is unavailable.
- Optional override: set `LEO_WEB_PUBLIC_URL` before starting server.
