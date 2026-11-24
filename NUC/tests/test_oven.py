"""Oven door control against a fake ESP32 with the MVP2 firmware API (local HTTP server):  python -m pytest"""
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import pytest

import config
from oven import Oven


@pytest.fixture
def esp32(monkeypatch):
    """Door takes 0.3 s to move; wrong key -> 403, like the firmware."""
    state = {"door": "closed", "commands": [], "move_time": 0.3, "end_as": None}

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def reply(self, code, text):
            self.send_response(code)
            self.send_header("Content-Type", "text/plain")
            self.end_headers()
            self.wfile.write(text.encode())

        def do_GET(self):
            u = urlparse(self.path)
            if parse_qs(u.query).get("key") != ["oven_control"]:
                return self.reply(403, "forbidden")
            if u.path in ("/door/open", "/door/close"):
                target = "open" if u.path == "/door/open" else "closed"
                state["commands"].append(u.path)
                if state["door"] == target:
                    return self.reply(200, f"already {target}")
                state["door"] = "moving"
                threading.Timer(state["move_time"], lambda: state.update(door=state["end_as"] or target)).start()
                return self.reply(200, "opening" if target == "open" else "closing")
            if u.path == "/door/status":
                return self.reply(200, state["door"])
            if u.path == "/status":
                return self.reply(200, "error" if state["end_as"] else "done")
            if u.path == "/stop":
                state["commands"].append("/stop")
                return self.reply(200, "stopping")
            self.reply(404, "not found")

    srv = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    monkeypatch.setattr(config, "OVENS", {"oven1": {"host": f"127.0.0.1:{srv.server_address[1]}",
                                                    "key": "oven_control"}})
    monkeypatch.setattr(config, "OVEN_DOOR_TIMEOUT", 3)
    yield state
    srv.shutdown()


def quiet(line):
    pass


def test_open_and_close_wait_for_the_door(esp32):
    oven = Oven("oven1")
    t0 = time.time()
    oven.open(quiet)
    assert esp32["door"] == "open" and time.time() - t0 >= 0.3   # waited until the door was really open
    oven.close(quiet)
    assert esp32["door"] == "closed"
    assert esp32["commands"] == ["/door/open", "/door/close"]


def test_already_open_is_fine(esp32):
    esp32["door"] = "open"
    lines = []
    Oven("oven1").open(lines.append)
    assert "already open" in lines[0] and esp32["door"] == "open"


def test_failed_move_raises_with_nano_status(esp32):
    esp32["end_as"] = "unknown"  # e.g. Nano ERR: the ESP32 marks the door unknown
    with pytest.raises(RuntimeError, match="ended up 'unknown' instead of 'open'.*error"):
        Oven("oven1").open(quiet)


def test_stuck_door_times_out(esp32, monkeypatch):
    monkeypatch.setattr(config, "OVEN_DOOR_TIMEOUT", 0.6)
    esp32["move_time"] = 99
    with pytest.raises(RuntimeError, match="still moving after"):
        Oven("oven1").open(quiet)


def test_wrong_key_reported(esp32, monkeypatch):
    monkeypatch.setitem(config.OVENS["oven1"], "key", "nope")
    with pytest.raises(RuntimeError, match="HTTP 403"):
        Oven("oven1").door()


def test_unreachable_reported(monkeypatch):
    monkeypatch.setattr(config, "OVENS", {"oven1": {"host": "127.0.0.1:1", "key": "k"}})
    with pytest.raises(RuntimeError, match="not reachable"):
        Oven("oven1").door(timeout=1)


def test_unknown_oven():
    with pytest.raises(ValueError):
        Oven("nope")
