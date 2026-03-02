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
python3 -m http.server 8080
```

In browser:

```text
http://localhost:8080
```
