#!/usr/bin/env python3
"""
MVP2 web interface: manage and run recipes, control the hardware by hand, read logs.

  python web.py                   open http://127.0.0.1:8080
  python web.py --port 8000
  python web.py --host 0.0.0.0    reachable from other machines on the network (no login!)

Uses the config, recipe checks and hardware code from main.py and functions.py,
so main.py keeps working from the console. Don't run both against the hardware at once.
Only one thing runs at a time: a recipe or a manual command. STOP always works.
"""
import argparse
import json
import os
import re
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlparse

import main as mvp2
from functions import UR5, HardwareError, Oven

BASE_DIR = Path(__file__).resolve().parent
WEB_DIR = BASE_DIR / "web"
NAME_RE = re.compile(r"^[A-Za-z0-9_-]{1,64}$")
LOGFILE_RE = re.compile(r"^[A-Za-z0-9_.-]+\.log$")
HARDWARE_POLL_S = 2


class ApiError(Exception):
    def __init__(self, status, message):
        super().__init__(message)
        self.status = status


# ------------------------------- Log ---------------------------------

class WebLog(mvp2.Log):
    """Writes to console and file like main.py, and keeps recent lines for the page."""

    def __init__(self, path):
        super().__init__(path)
        self.lines = deque(maxlen=2000)
        self.seq = 0
        self.lock = threading.Lock()

    def __call__(self, msg):
        super().__call__(msg)
        with self.lock:
            self.seq += 1
            self.lines.append((self.seq, f"{time.strftime('%H:%M:%S')}  {msg}"))

    def since(self, n):
        with self.lock:
            return [{"id": i, "text": t} for i, t in self.lines if i > n], self.seq


# ---------------------------- Controller -----------------------------

class Controller:
    def __init__(self, log):
        self.log = log
        self.lock = threading.Lock()
        self.job = None
        self.thread = None
        self.runner = None
        self.abort = threading.Event()
        self.resume = threading.Event()
        self.stopper = mvp2.Runner(log, console=False)   # real hardware, used for STOP
        self.hardware = {"oven": None, "ur5": None, "updated": None}
        threading.Thread(target=self._poll_hardware, daemon=True).start()

    # --- hardware readouts (separate connections, short timeouts) ---
    def _poll_hardware(self):
        quiet = lambda msg: None
        oven = Oven(mvp2.OVEN_HOST, mvp2.OVEN_KEY, log=quiet, http_timeout=2)
        ur5 = UR5(mvp2.UR5_HOST, log=quiet)
        while True:
            hw = {}
            try:
                hw["oven"] = oven.status()
            except HardwareError as e:
                hw["oven"] = {"error": str(e)}
            try:
                hw["ur5"] = ur5.status()
            except (HardwareError, OSError) as e:
                hw["ur5"] = {"error": str(e)}
            hw["updated"] = time.time()
            self.hardware = hw
            time.sleep(HARDWARE_POLL_S)

    # --- jobs ---
    def busy(self):
        return self.thread is not None and self.thread.is_alive()

    def _start(self, job, target):
        with self.lock:
            if self.busy():
                raise ApiError(409, "Something is already running. Stop it or wait until it finishes.")
            self.abort.clear()
            self.resume.clear()
            job.update(state="running", started=time.time(), ended=None, message="")
            self.job = job
            self.thread = threading.Thread(target=self._run, args=(target,), daemon=True)
            self.thread.start()

    def _run(self, target):
        job = self.job
        try:
            target()
            job["state"] = "done"
        except mvp2.Aborted as e:
            job.update(state="stopped", message=str(e))
            self.log(f"STOPPED: {e}")
        except mvp2.RecipeError as e:
            job.update(state="error", message=str(e))
            self.log(f"RECIPE ERROR: {e}")
        except Exception as e:  # HardwareError or anything unexpected
            if self.abort.is_set():
                job.update(state="stopped", message="stopped by operator")
                self.log("STOPPED: stopped by operator")
            else:
                job.update(state="error", message=str(e))
                self.log(f"ERROR: {e}")
                self.stopper.stop_all()
        finally:
            job["ended"] = time.time()

    def _make_runner(self, dry):
        self.runner = mvp2.Runner(self.log, dry=dry, abort=self.abort, pause=self._pause,
                                  progress=self._progress, console=False)
        return self.runner

    def _pause(self, message):
        self.job.update(state="paused", message=message or "Paused")
        while not self.resume.wait(0.2):
            if self.abort.is_set():
                raise mvp2.Aborted("stopped while paused")
        self.resume.clear()
        self.job.update(state="running", message="")
        self.log("continued by operator")

    def _progress(self, step, total, lineno, tokens):
        self.job.update(step=step, total=total, line=lineno)

    def start_recipe(self, name, dry):
        path, steps = mvp2.load_recipe(name)   # RecipeError -> 400 before anything starts
        job = {"kind": "recipe", "name": path.stem, "dry": dry, "step": 0, "total": len(steps),
               "line": None, "steps": [{"line": n, "text": " ".join(t)} for n, t in steps]}
        self._start(job, lambda: self._make_runner(dry).run_recipe(name))

    def start_command(self, line):
        tokens = mvp2.parse_line(line)
        if not tokens:
            raise ApiError(400, "Type a command, for example: door open")
        if tokens[0] in ("run", "list", "help", "quit", "exit"):
            raise ApiError(400, f"'{tokens[0]}' only works in the console. Use the Run tab for recipes.")
        mvp2.check_step(tokens, None)
        text = " ".join(tokens)
        job = {"kind": "command", "name": text, "dry": False}

        def target():
            self.log(f"manual: {text}")
            self._make_runner(False).execute(tokens)
        self._start(job, target)

    def stop(self):
        self.abort.set()
        self.log("STOP pressed")
        self.stopper.stop_all()

    def continue_(self):
        if not self.job or self.job.get("state") != "paused":
            raise ApiError(409, "Nothing is waiting to continue.")
        self.resume.set()

    def state(self, since):
        lines, seq = self.log.since(since)
        job = dict(self.job) if self.job else None
        if job and self.runner and self.runner.wait_until and job["state"] == "running":
            job["wait_left"] = max(0, self.runner.wait_until - time.time())
        return {"job": job, "busy": self.busy(), "hardware": self.hardware,
                "log": lines, "log_seq": seq, "now": time.time()}


# ------------------------------ Recipes ------------------------------

def recipe_path(name):
    if not NAME_RE.match(name or ""):
        raise ApiError(400, "Use letters, numbers, - and _ in recipe names (max 64).")
    return mvp2.RECIPE_DIR / f"{name}.txt"


def recipe_info(path):
    text = path.read_text(encoding="utf-8")
    steps, errors = mvp2.check_recipe_text(text)
    return {"name": path.stem, "steps": len(steps), "modified": path.stat().st_mtime,
            "errors": [{"line": n, "message": m} for n, m in errors]}


def list_recipes():
    mvp2.RECIPE_DIR.mkdir(exist_ok=True)
    return [recipe_info(p) for p in sorted(mvp2.RECIPE_DIR.glob("*.txt"), key=lambda p: p.stem.lower())]


def get_recipe(name):
    path = recipe_path(name)
    if not path.is_file():
        raise ApiError(404, f"No recipe called {name}.")
    text = path.read_text(encoding="utf-8")
    steps, _ = mvp2.check_recipe_text(text)
    info = recipe_info(path)
    info.update(text=text, step_list=[{"line": n, "text": " ".join(t)} for n, t in steps])
    return info


def save_recipe(name, text, create):
    path = recipe_path(name)
    if create and path.exists():
        raise ApiError(409, f"A recipe called {name} already exists.")
    mvp2.RECIPE_DIR.mkdir(exist_ok=True)
    path.write_text(text.rstrip() + "\n", encoding="utf-8")
    return recipe_info(path)


def delete_recipe(name):
    path = recipe_path(name)
    if not path.is_file():
        raise ApiError(404, f"No recipe called {name}.")
    path.unlink()
    return {"deleted": name}


def validate_text(text):
    steps, errors = mvp2.check_recipe_text(text)
    return {"steps": len(steps), "errors": [{"line": n, "message": m} for n, m in errors]}


# ------------------------------- Logs --------------------------------

def list_logs():
    mvp2.LOG_DIR.mkdir(exist_ok=True)
    files = sorted(mvp2.LOG_DIR.glob("*.log"), key=lambda p: p.stat().st_mtime, reverse=True)
    return [{"name": p.name, "size": p.stat().st_size, "modified": p.stat().st_mtime} for p in files]


def read_log(name, max_bytes=300_000):
    if not LOGFILE_RE.match(name):
        raise ApiError(400, "Bad log file name.")
    path = mvp2.LOG_DIR / name
    if not path.is_file():
        raise ApiError(404, f"No log called {name}.")
    data = path.read_bytes()
    truncated = len(data) > max_bytes
    return {"name": name, "truncated": truncated,
            "text": data[-max_bytes:].decode("utf-8", errors="replace")}


# ------------------------------- HTTP --------------------------------

def make_handler(ctl):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):   # keep the console for the MVP2 log
            pass

        def _send(self, status, body, ctype="application/json"):
            data = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(status)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def _body(self):
            length = int(self.headers.get("Content-Length") or 0)
            if not length:
                return {}
            try:
                return json.loads(self.rfile.read(length))
            except json.JSONDecodeError:
                raise ApiError(400, "Body must be JSON.") from None

        def _handle(self, method):
            url = urlparse(self.path)
            parts = [unquote(p) for p in url.path.strip("/").split("/") if p]
            query = parse_qs(url.query)
            try:
                if method == "GET" and not parts:
                    return self._send(200, (WEB_DIR / "index.html").read_bytes(), "text/html; charset=utf-8")
                if not parts or parts[0] != "api":
                    raise ApiError(404, "Not found.")
                result = self._api(method, parts[1:], query)
                self._send(200, result)
            except ApiError as e:
                self._send(e.status, {"error": str(e)})
            except mvp2.RecipeError as e:
                self._send(400, {"error": str(e)})
            except Exception as e:
                ctl.log(f"web error: {e!r}")
                self._send(500, {"error": f"Server error: {e}"})

        def _api(self, method, p, query):
            route = (method, p[0] if p else "", len(p))
            if route == ("GET", "state", 1):
                return ctl.state(int(query.get("since", ["0"])[0]))
            if route == ("GET", "config", 1):
                return {"robot_programs": mvp2.ROBOT_PROGRAMS,
                        "needs_open_door": sorted(mvp2.NEEDS_OPEN_DOOR),
                        "max_temp_steps": mvp2.MAX_TEMP_STEPS,
                        "oven_host": mvp2.OVEN_HOST, "ur5_host": mvp2.UR5_HOST}
            if route == ("GET", "recipes", 1):
                return list_recipes()
            if route == ("GET", "recipes", 2):
                return get_recipe(p[1])
            if route == ("PUT", "recipes", 2):
                body = self._body()
                return save_recipe(p[1], str(body.get("text", "")), bool(body.get("create")))
            if route == ("DELETE", "recipes", 2):
                return delete_recipe(p[1])
            if route == ("POST", "validate", 1):
                return validate_text(str(self._body().get("text", "")))
            if route == ("POST", "run", 1):
                body = self._body()
                ctl.start_recipe(str(body.get("name", "")), bool(body.get("dry")))
                return {"ok": True}
            if route == ("POST", "command", 1):
                ctl.start_command(str(self._body().get("line", "")))
                return {"ok": True}
            if route == ("POST", "stop", 1):
                ctl.stop()
                return {"ok": True}
            if route == ("POST", "continue", 1):
                ctl.continue_()
                return {"ok": True}
            if route == ("GET", "logs", 1):
                return list_logs()
            if route == ("GET", "logs", 2):
                return read_log(p[1])
            raise ApiError(404, "Unknown API route.")

        def do_GET(self):
            self._handle("GET")

        def do_POST(self):
            self._handle("POST")

        def do_PUT(self):
            self._handle("PUT")

        def do_DELETE(self):
            self._handle("DELETE")

    return Handler


def main():
    ap = argparse.ArgumentParser(description="MVP2 web interface")
    ap.add_argument("--host", default="127.0.0.1", help="0.0.0.0 to allow other machines (no login!)")
    ap.add_argument("--port", type=int, default=8080)
    args = ap.parse_args()

    os.chdir(BASE_DIR)   # recipes/ and logs/ next to this file, like main.py
    mvp2.LOG_DIR.mkdir(exist_ok=True)
    log = WebLog(mvp2.LOG_DIR / f"web_{time.strftime('%Y%m%d_%H%M%S')}.log")
    ctl = Controller(log)

    server = ThreadingHTTPServer((args.host, args.port), make_handler(ctl))
    shown = "127.0.0.1" if args.host in ("0.0.0.0", "") else args.host
    log(f"web interface on http://{shown}:{args.port}  (Ctrl+C to quit)")
    if args.host == "0.0.0.0":
        log("warning: anyone on the network can open this page and move the hardware")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print()
        if ctl.busy():
            ctl.stop()
        log("web interface stopped")


if __name__ == "__main__":
    main()