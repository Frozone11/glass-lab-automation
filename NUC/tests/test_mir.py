"""MiR REST client against a fake MiR (local HTTP server):  python -m pytest"""
import base64
import hashlib
import json
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

import config
from mir import MiR


@pytest.fixture
def fake_mir(monkeypatch):
    """Missions run 3 polls after the MiR is set to Ready; a wrong password gives 401, like the real API."""
    state = {"state_id": 4, "queue": {}, "next_id": 1, "puts": [], "abort": False}
    missions = [{"guid": "g-1", "name": "go to oven1"}, {"guid": "g-2", "name": "go home"}]
    auth = "Basic " + base64.b64encode(f"admin:{hashlib.sha256(b'secret').hexdigest()}".encode()).decode()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def reply(self, code, obj):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def body(self):
            return json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")

        def handle_all(self, method):
            if self.headers.get("Authorization") != auth:
                return self.reply(401, {"error": "unauthorized"})
            path = self.path.removeprefix("/api/v2.0.0")
            if method == "GET" and path == "/status":
                return self.reply(200, {"state_text": {3: "Ready", 4: "Pause"}[state["state_id"]],
                                        "battery_percentage": 87.5, "mission_text": "Waiting"})
            if method == "PUT" and path == "/status":
                state["state_id"] = self.body()["state_id"]
                state["puts"].append(state["state_id"])
                return self.reply(200, {})
            if method == "GET" and path == "/missions":
                return self.reply(200, missions)
            if method == "POST" and path == "/mission_queue":
                i = state["next_id"]
                state["next_id"] += 1
                state["queue"][i] = {"mission": self.body()["mission_id"], "polls": 0}
                return self.reply(201, {"id": i})
            m = re.fullmatch(r"/mission_queue/(\d+)", path)
            if method == "GET" and m:
                q = state["queue"][int(m[1])]
                if state["state_id"] == 3:
                    q["polls"] += 1
                if state["abort"]:
                    return self.reply(200, {"state": "Aborted"})
                return self.reply(200, {"state": "Done" if q["polls"] >= 3 else "Executing"})
            self.reply(404, {"error": "not found"})

        def do_GET(self):
            self.handle_all("GET")

        def do_PUT(self):
            self.handle_all("PUT")

        def do_POST(self):
            self.handle_all("POST")

    srv = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    monkeypatch.setattr(config, "MIR_IP", f"127.0.0.1:{srv.server_address[1]}")
    monkeypatch.setattr(config, "MIR_USER", "admin")
    monkeypatch.setattr(config, "MIR_PASSWORD", "secret")
    monkeypatch.setattr("mir.time.sleep", lambda s: None)
    yield state
    srv.shutdown()


def test_status_and_missions(fake_mir):
    assert MiR().status() == {"state": "Pause", "battery": 87.5, "mission": "Waiting"}
    assert MiR().missions() == ["go home", "go to oven1"]


def test_run_mission_plays_and_waits_until_done(fake_mir):
    lines = []
    MiR().run_mission("go to oven1", lines.append)
    assert fake_mir["queue"][1]["mission"] == "g-1"
    assert fake_mir["puts"] == [3]                          # pressed play (Ready)
    assert fake_mir["queue"][1]["polls"] >= 3 and "done" in lines[-1]


def test_aborted_mission_raises(fake_mir):
    fake_mir["abort"] = True
    with pytest.raises(RuntimeError, match="aborted"):
        MiR().run_mission("go to oven1", lambda line: None)


def test_stop_pauses_the_mir(fake_mir):
    with pytest.raises(RuntimeError, match="stopped by user"):
        MiR().run_mission("go to oven1", lambda line: None, stopped=lambda: True)
    assert fake_mir["puts"][-1] == 4 and fake_mir["state_id"] == 4


def test_unknown_mission_and_wrong_password(fake_mir, monkeypatch):
    with pytest.raises(RuntimeError, match="no MiR mission called 'nope'"):
        MiR().run_mission("nope", lambda line: None)
    monkeypatch.setattr(config, "MIR_PASSWORD", "wrong")
    with pytest.raises(RuntimeError, match="HTTP 401"):
        MiR().status()
