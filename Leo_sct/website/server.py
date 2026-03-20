#!/usr/bin/env python3
import argparse
import io
import ipaddress
import json
import os
import secrets
import socket
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from http import cookies
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional
from urllib.parse import parse_qs, urlsplit

try:
    import qrcode
except Exception:
    qrcode = None


SESSION_COOKIE = "leo_session"
LEASE_TIMEOUT_SECONDS = 12
WEB_ROOT = Path(__file__).resolve().parent


@dataclass
class SessionInfo:
    created_at: float
    local_emergency: bool


@dataclass
class LeaseInfo:
    owner_sid: str
    expires_at: float


SESSIONS: dict[str, SessionInfo] = {}
ROBOT_LEASES: dict[str, LeaseInfo] = {}
EMERGENCY_LATCH = {
    "active": False,
    "set_at": None,
}
STATE_LOCK = threading.Lock()


def is_loopback(address: str) -> bool:
    try:
        return ipaddress.ip_address(address).is_loopback
    except ValueError:
        return address in {"127.0.0.1", "::1", "::ffff:127.0.0.1"}


def host_header_is_loopback(raw_host: str) -> bool:
    host = (raw_host or "").strip().lower()
    if not host:
        return False
    if host.startswith("[") and "]" in host:
        host = host[1 : host.index("]")]
    elif ":" in host:
        host = host.rsplit(":", 1)[0]
    return host in {"localhost", "127.0.0.1", "::1"}


def iso_utc(ts: Optional[float]) -> Optional[str]:
    if ts is None:
        return None
    return datetime.fromtimestamp(ts, tz=timezone.utc).isoformat()


class LeoControlHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(WEB_ROOT), **kwargs)

    def do_GET(self):
        path = urlsplit(self.path).path
        if path == "/api/session":
            self._serve_session_policy()
            return
        if path == "/api/connect_info":
            self._serve_connect_info()
            return
        if path == "/api/qr":
            self._serve_qr()
            return
        super().do_GET()

    def do_POST(self):
        path = urlsplit(self.path).path
        if path == "/api/control/claim":
            self._handle_claim()
            return
        if path == "/api/control/release":
            self._handle_release()
            return
        if path == "/api/control/heartbeat":
            self._handle_heartbeat()
            return
        if path == "/api/emergency/activate":
            self._handle_emergency_activate()
            return
        if path == "/api/emergency/clear":
            self._handle_emergency_clear()
            return
        self._json_response(404, {"ok": False, "error": "not_found"})

    def _parse_cookie(self) -> cookies.SimpleCookie:
        jar = cookies.SimpleCookie()
        raw = self.headers.get("Cookie")
        if raw:
            jar.load(raw)
        return jar

    def _parse_json_body(self) -> dict:
        raw_len = self.headers.get("Content-Length")
        if not raw_len:
            return {}
        try:
            length = int(raw_len)
        except ValueError:
            return {}
        if length <= 0:
            return {}
        payload = self.rfile.read(length)
        if not payload:
            return {}
        try:
            value = json.loads(payload.decode("utf-8"))
            return value if isinstance(value, dict) else {}
        except Exception:
            return {}

    def _json_response(self, status: int, payload: dict, sid: Optional[str] = None, session: Optional[SessionInfo] = None):
        encoded = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        if sid and session:
            self._set_session_cookie(sid, session)
        self.end_headers()
        self.wfile.write(encoded)

    def _set_session_cookie(self, sid: str, session: SessionInfo):
        parts = [
            f"{SESSION_COOKIE}={sid}",
            "Path=/",
            "HttpOnly",
            "SameSite=Lax",
        ]
        self.send_header("Set-Cookie", "; ".join(parts))

    def _prune_expired(self):
        now = time.time()
        expired_leases = [
            robot_id
            for robot_id, lease in ROBOT_LEASES.items()
            if now >= lease.expires_at
        ]
        for robot_id in expired_leases:
            ROBOT_LEASES.pop(robot_id, None)

    def _get_or_create_session(self, force_new_requested: bool) -> tuple[str, SessionInfo, bool]:
        now = time.time()
        remote_address = self.client_address[0]
        local = is_loopback(remote_address) or host_header_is_loopback(self.headers.get("Host", ""))
        force_new = force_new_requested and local

        jar = self._parse_cookie()
        sid = jar.get(SESSION_COOKIE).value if jar.get(SESSION_COOKIE) else None
        session = SESSIONS.get(sid) if sid else None

        is_new = False
        if force_new or session is None or session.local_emergency != local:
            sid = secrets.token_urlsafe(18)
            session = SessionInfo(created_at=now, local_emergency=local)
            SESSIONS[sid] = session
            is_new = True

        return sid, session, is_new

    def _resolve_session(self) -> tuple[str, SessionInfo, bool]:
        force_new = parse_qs(urlsplit(self.path).query).get("renew", ["0"])[0] == "1"
        with STATE_LOCK:
            sid, session, _ = self._get_or_create_session(force_new_requested=force_new)
            self._prune_expired()
            return sid, session, False

    def _active_latch_for_session(self, session: SessionInfo) -> bool:
        return bool(EMERGENCY_LATCH["active"]) and not session.local_emergency

    def _session_payload(self, sid: str, session: SessionInfo) -> dict:
        now = time.time()
        owned = sorted(
            robot_id
            for robot_id, lease in ROBOT_LEASES.items()
            if lease.owner_sid == sid and now < lease.expires_at
        )
        return {
            "local_emergency": session.local_emergency,
            "lease_timeout_seconds": LEASE_TIMEOUT_SECONDS,
            "created_at": iso_utc(session.created_at),
            "expired": self._active_latch_for_session(session),
            "emergency_active": bool(EMERGENCY_LATCH["active"]),
            "emergency_set_at": iso_utc(EMERGENCY_LATCH["set_at"]),
            "owned_robot_ids": owned,
        }

    def _serve_session_policy(self):
        with STATE_LOCK:
            sid, session, _ = self._get_or_create_session(
                force_new_requested=parse_qs(urlsplit(self.path).query).get("renew", ["0"])[0] == "1"
            )
            self._prune_expired()
            payload = self._session_payload(sid, session)
        self._json_response(200, payload, sid=sid, session=session)

    def _detect_lan_ip(self) -> str:
        # Best-effort local LAN IP discovery for QR/connect URL.
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.connect(("8.8.8.8", 80))
                ip = sock.getsockname()[0]
                if ip and ip != "127.0.0.1":
                    return ip
        except Exception:
            pass
        return "127.0.0.1"

    def _serve_connect_info(self):
        host_hdr = self.headers.get("Host", "")
        split = host_hdr.split(":")
        port = split[1] if len(split) > 1 and split[1].isdigit() else str(self.server.server_port)

        public_override = os.environ.get("LEO_WEB_PUBLIC_URL", "").strip()
        if public_override:
            phone_url = public_override
        else:
            phone_url = f"http://{self._detect_lan_ip()}:{port}"

        payload = {
            "phone_url": phone_url,
            "localhost_url": f"http://localhost:{port}",
            "qr_local_generator": qrcode is not None,
        }
        self._json_response(200, payload)

    def _serve_qr(self):
        query = parse_qs(urlsplit(self.path).query)
        raw = query.get("url", [""])[0].strip()
        if not raw:
            self._json_response(400, {"ok": False, "error": "missing_url"})
            return
        if qrcode is None:
            self._json_response(503, {"ok": False, "error": "qr_generator_unavailable"})
            return

        img = qrcode.make(raw)
        buffer = io.BytesIO()
        img.save(buffer, format="PNG")
        data = buffer.getvalue()

        self.send_response(200)
        self.send_header("Content-Type", "image/png")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _require_active_session(self) -> tuple[Optional[str], Optional[SessionInfo], Optional[dict]]:
        with STATE_LOCK:
            sid, session, _ = self._get_or_create_session(False)
            self._prune_expired()
            return sid, session, None

    def _handle_claim(self):
        body = self._parse_json_body()
        robot_id = str(body.get("robot_id", "")).strip()
        if not robot_id:
            self._json_response(400, {"ok": False, "error": "missing_robot_id"})
            return

        sid, session, _ = self._require_active_session()

        now = time.time()
        with STATE_LOCK:
            if self._active_latch_for_session(session):
                payload = self._session_payload(sid, session)
                self._json_response(403, {"ok": False, "error": "emergency_active", "session": payload}, sid=sid, session=session)
                return

            lease = ROBOT_LEASES.get(robot_id)
            if lease and now >= lease.expires_at:
                ROBOT_LEASES.pop(robot_id, None)
                lease = None

            if lease and lease.owner_sid != sid:
                self._json_response(
                    409,
                    {
                        "ok": False,
                        "error": "robot_busy",
                        "robot_id": robot_id,
                        "lease_expires_at": iso_utc(lease.expires_at),
                    },
                    sid=sid,
                    session=session,
                )
                return

            ROBOT_LEASES[robot_id] = LeaseInfo(owner_sid=sid, expires_at=now + LEASE_TIMEOUT_SECONDS)
            payload = self._session_payload(sid, session)

        self._json_response(200, {"ok": True, "robot_id": robot_id, "lease_expires_at": iso_utc(now + LEASE_TIMEOUT_SECONDS), "session": payload}, sid=sid, session=session)

    def _handle_release(self):
        body = self._parse_json_body()
        robot_ids = body.get("robot_ids")
        if robot_ids is None:
            robot_ids = []
        if not isinstance(robot_ids, list):
            robot_ids = []

        sid, session, _ = self._require_active_session()

        with STATE_LOCK:
            self._prune_expired()
            ids = {str(x).strip() for x in robot_ids if str(x).strip()}
            if not ids:
                ids = {rid for rid, lease in ROBOT_LEASES.items() if lease.owner_sid == sid}

            for rid in ids:
                lease = ROBOT_LEASES.get(rid)
                if lease and lease.owner_sid == sid:
                    ROBOT_LEASES.pop(rid, None)

            payload = self._session_payload(sid, session)

        self._json_response(200, {"ok": True, "released": sorted(ids), "session": payload}, sid=sid, session=session)

    def _handle_heartbeat(self):
        body = self._parse_json_body()
        robot_ids = body.get("robot_ids")
        if not isinstance(robot_ids, list):
            robot_ids = []

        sid, session, _ = self._require_active_session()

        now = time.time()
        refreshed = []
        with STATE_LOCK:
            self._prune_expired()
            if self._active_latch_for_session(session):
                payload = self._session_payload(sid, session)
                self._json_response(403, {"ok": False, "error": "emergency_active", "session": payload}, sid=sid, session=session)
                return

            for rid in [str(x).strip() for x in robot_ids if str(x).strip()]:
                lease = ROBOT_LEASES.get(rid)
                if lease and lease.owner_sid == sid:
                    lease.expires_at = now + LEASE_TIMEOUT_SECONDS
                    refreshed.append(rid)
            payload = self._session_payload(sid, session)

        self._json_response(200, {"ok": True, "refreshed": refreshed, "session": payload}, sid=sid, session=session)

    def _handle_emergency_activate(self):
        sid, session, _ = self._require_active_session()

        if not session.local_emergency:
            self._json_response(403, {"ok": False, "error": "local_only"}, sid=sid, session=session)
            return

        with STATE_LOCK:
            EMERGENCY_LATCH["active"] = True
            EMERGENCY_LATCH["set_at"] = time.time()
            ROBOT_LEASES.clear()
            payload = self._session_payload(sid, session)

        self._json_response(200, {"ok": True, "session": payload}, sid=sid, session=session)

    def _handle_emergency_clear(self):
        sid, session, _ = self._require_active_session()

        if not session.local_emergency:
            self._json_response(403, {"ok": False, "error": "local_only"}, sid=sid, session=session)
            return

        with STATE_LOCK:
            EMERGENCY_LATCH["active"] = False
            EMERGENCY_LATCH["set_at"] = None
            payload = self._session_payload(sid, session)

        self._json_response(200, {"ok": True, "session": payload}, sid=sid, session=session)


def parse_args():
    parser = argparse.ArgumentParser(description="Leo control web server with session policy.")
    parser.add_argument("--host", default="0.0.0.0", help="Bind host (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=8080, help="Bind port (default: 8080)")
    return parser.parse_args()


def main():
    args = parse_args()
    server = ThreadingHTTPServer((args.host, args.port), LeoControlHandler)
    print(f"[leo-web] Serving {WEB_ROOT} on http://{args.host}:{args.port}")
    server.serve_forever()


if __name__ == "__main__":
    main()
