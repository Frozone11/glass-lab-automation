"""Web control page for the NUC:  python web.py   then open http://<NUC-IP>:8000

One job (calibration, teach, pick, ...) runs at a time in a worker thread. When a job needs the user it calls
wait(msg): the page shows the message with Continue / Done buttons and the job blocks until one is clicked.
"""
import json
import queue
import re
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np

import config
from mir import MiR
from oven import Oven
from ur5.arm import UR5, load, save
from ur5.vision import DICT, Camera

PAGE = Path(__file__).parent / "web" / "index.html"
OVEN_STATE = {name: "unknown" for name in config.OVENS}   # kept fresh by watch_ovens()
MIR_STATE = {"state": "unknown", "battery": None, "mission": None, "missions": []}  # kept fresh by watch_mir()
NAME = re.compile(r"^[A-Za-z0-9_-]{1,30}$")


class Jobs:
    def __init__(self):
        self.name, self.msg, self.waiting, self.ur, self.stop_requested = None, "", False, None, False
        self.log = deque(maxlen=200)
        self._answers = queue.Queue()

    def say(self, line):
        self.log.append(time.strftime("%H:%M:%S  ") + line)

    def start(self, name, job):
        if self.name:
            raise ValueError(f"'{self.name}' is still running - finish or stop it first")
        self.name, self.msg, self.stop_requested = name, "", False
        threading.Thread(target=self._run, args=(job,), daemon=True).start()

    def _run(self, job):
        self.say(f"== {self.name} started")
        try:
            job(self)
            self.say(f"== {self.name} finished")
        except Exception as e:
            self.say(f"== {self.name} FAILED: {e}")
        finally:
            self.name, self.msg, self.waiting = None, "", False

    def wait(self, msg):
        """Called from the job thread: show msg, block until the user answers 'next' or 'done'."""
        while not self._answers.empty():
            self._answers.get()  # ignore clicks from before this question
        self.msg, self.waiting = msg, True
        answer = self._answers.get()
        self.waiting = False
        if answer == "stop":
            raise RuntimeError("stopped by user")
        return answer

    def answer(self, a):
        self._answers.put(a)

    def stop(self):
        self.say("STOP pressed")
        self.stop_requested = True
        try:
            MiR().pause()
        except Exception as e:
            self.say(f"   MiR: could not pause ({e})")
        if self.waiting:
            self.answer("stop")
        if self.ur is not None:
            self.ur.c.stopScript()  # halts a running move; the move then fails and the job ends
        for name in config.OVENS:   # and any moving oven door
            try:
                Oven(name).stop()
            except Exception as e:
                self.say(f"   {name}: could not stop ({e})")


def robot_job(cam, fn):
    """Job that connects to the UR5 for its duration."""
    def job(j):
        ur = UR5.connect(cam, j.wait, j.say)
        j.ur = ur
        try:
            fn(ur)
        finally:
            j.ur = None
            ur.close()
    return job


def report(ur, s):
    ur.log(f"tag position spread: mean {s.mean():.1f} mm, max {s.max():.1f} mm (want < 2-3 mm)")


def pick_or_place(ur, kind, name):
    getattr(ur, kind)(name)
    if "home" in load("poses"):
        ur.go("home")


def check(ur, name):
    ok, _, text = ur.check_parking(name)
    ur.log(("PARKING OK: " if ok else "PARKING NOT OK - move the MiR: ") + text)
    ur.log("   arm stays at the search pose so you can watch the camera - Go home before driving the MiR")


def drive(ur, mission, jobs):
    """Tuck the arm in first: the MiR must never drive with the arm out."""
    if "home" not in load("poses"):
        raise RuntimeError("no 'home' pose - save one first, the arm must be tucked in before the MiR drives")
    ur.go("home")
    MiR().run_mission(mission, ur.log, lambda: jobs.stop_requested)


def make_job(cam, action, d, jobs):
    """-> (job name, job) for a POST /api/<action>, or ValueError for bad input."""
    name = str(d.get("name", ""))
    if action in ("save_pose", "teach", "pick", "place", "check") and not NAME.match(name):
        raise ValueError("name: 1-30 letters, digits, - or _")
    if action in ("pick", "place", "check") and name not in load("targets"):
        raise ValueError(f"no target '{name}' - teach it first")
    if action == "intrinsics":
        return "camera calibration", lambda j: cam.calibrate(j.wait, j.say)
    if action == "handeye":
        return "hand-eye calibration", robot_job(cam, lambda ur: report(ur, ur.calibrate_handeye()))
    if action == "auto_handeye":
        return "hand-eye calibration", robot_job(cam, lambda ur: report(ur, ur.auto_handeye()))
    if action == "save_pose":
        return f"save pose {name}", robot_job(cam, lambda ur: (ur.freedrive(f"Move the arm to the '{name}' pose."),
                                                               ur.save_pose(name), ur.log(f"pose '{name}' saved")))
    if action == "teach":
        tag = int(d.get("tag", -1))
        if not 0 <= tag < 50:
            raise ValueError("tag: 0-49")
        oven = d.get("oven") or None
        if oven is not None and oven not in config.OVENS:
            raise ValueError(f"unknown oven '{oven}'")
        return f"teach {name}", robot_job(cam, lambda ur: (ur.teach(name, tag, oven),
                                                           ur.log(f"target '{name}' saved")))
    if action == "door":
        oven = str(d.get("oven", ""))
        if oven not in config.OVENS:
            raise ValueError(f"unknown oven '{oven}'")
        opening = bool(d.get("open"))
        return (f"{oven} door {'open' if opening else 'close'}",
                lambda j: Oven(oven).open(j.say) if opening else Oven(oven).close(j.say))
    if action in ("pick", "place"):
        return f"{action} {name}", robot_job(cam, lambda ur: pick_or_place(ur, action, name))
    if action == "search":
        return "go to search pose", robot_job(cam, lambda ur: ur.search())
    if action == "check":
        return f"check parking for {name}", robot_job(cam, lambda ur: check(ur, name))
    if action == "mir_go":
        mission = str(d.get("mission", ""))
        if not mission:
            raise ValueError("pick a MiR mission")
        return f"MiR: {mission}", robot_job(cam, lambda ur: drive(ur, mission, jobs))
    if action == "grip":
        close = bool(d.get("close"))
        return f"gripper {'close' if close else 'open'}", robot_job(cam, lambda ur: ur.gripper(close))
    if action == "home":
        if "home" not in load("poses"):
            raise ValueError("no 'home' pose - save one first")
        return "go home", robot_job(cam, lambda ur: ur.go("home"))
    raise ValueError(f"unknown action '{action}'")


def delete(jobs, d):
    """Remove a taught target or a saved pose (instant, not a job)."""
    kind, name = d.get("kind"), str(d.get("name", ""))
    if kind not in ("targets", "poses"):
        raise ValueError("kind must be targets or poses")
    if jobs.name:
        raise ValueError(f"'{jobs.name}' is still running - finish or stop it first")
    items = load(kind)
    if name not in items:
        raise ValueError(f"no {kind[:-1]} '{name}'")
    del items[name]
    save(kind, items)
    jobs.say(f"deleted {kind[:-1]} '{name}'")


def overlay(img, show_board):
    corners, ids, _ = cv2.aruco.ArucoDetector(DICT).detectMarkers(img)
    cv2.aruco.drawDetectedMarkers(img, corners, ids)
    if show_board:
        found, c = cv2.findChessboardCornersSB(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), config.BOARD)
        if found:
            cv2.drawChessboardCorners(img, config.BOARD, c, found)
    return img, [] if ids is None else sorted(int(i) for i in ids.ravel())


def status(cam, jobs):
    img = cam.latest()
    tags = overlay(img, False)[1] if img is not None else []
    return {
        "job": jobs.name, "msg": jobs.msg, "waiting": jobs.waiting, "log": list(jobs.log),
        "camera": img is not None, "tags_seen": tags,
        "intrinsics": cam.calib_error or "ok",
        "handeye": "ok" if (config.DATA / "handeye.npy").exists() else "not done",
        "poses": sorted(load("poses")),
        "targets": [{"name": k, "tag": UR5.tag(v), "oven": v.get("oven")} for k, v in sorted(load("targets").items())],
        "ovens": [{"name": k, "door": v} for k, v in sorted(OVEN_STATE.items())],
        "mir": MIR_STATE,
    }


def watch_ovens():
    """Poll every oven's door state in the background, so the page never waits on an ESP32."""
    while True:
        for name in config.OVENS:
            try:
                OVEN_STATE[name] = Oven(name).door(timeout=1) or "unknown"
            except Exception:
                OVEN_STATE[name] = "not reachable"
        time.sleep(2)


def watch_mir():
    """Poll the MiR's status (every 2 s) and mission list (every 30 s) in the background."""
    n = 0
    while True:
        try:
            MIR_STATE.update(MiR().status(timeout=1))
            if n % 15 == 0:
                MIR_STATE["missions"] = MiR().missions(timeout=2)
        except Exception:
            MIR_STATE.update(state="not reachable", battery=None, mission=None)
        n += 1
        time.sleep(2)


def make_server(cam, jobs, port=config.WEB_PORT):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def send(self, code, body, ctype="application/json"):
            body = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/":
                self.send(200, PAGE.read_bytes(), "text/html; charset=utf-8")
            elif self.path == "/status":
                self.send(200, status(cam, jobs))
            elif self.path.startswith("/stream"):
                self.stream()
            else:
                self.send(404, {"error": "not found"})

        def stream(self):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                while True:
                    img = cam.latest()
                    if img is None:
                        img = np.zeros((360, 640, 3), np.uint8)
                        cv2.putText(img, f"no image from camera {config.CAM}", (20, 180), 0, 0.8, (0, 0, 255), 2)
                    else:
                        img = overlay(img, jobs.name in ("camera calibration", "hand-eye calibration"))[0]
                    jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])[1].tobytes()
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                     + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                    time.sleep(0.1)
            except (BrokenPipeError, ConnectionResetError):
                pass  # browser closed the page

        def do_POST(self):
            action = self.path.removeprefix("/api/")
            try:
                d = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
                if action == "answer":
                    if d.get("key") not in ("next", "done"):
                        raise ValueError("key must be next or done")
                    jobs.answer(d["key"])
                elif action == "stop":
                    jobs.stop()
                elif action == "delete":
                    delete(jobs, d)
                elif action in ("mir_pause", "mir_resume"):  # instant, also while a job runs
                    try:
                        MiR().pause() if action == "mir_pause" else MiR().resume()
                    except RuntimeError as e:
                        raise ValueError(str(e))
                else:
                    jobs.start(*make_job(cam, action, d, jobs))
                self.send(200, {"ok": True})
            except (ValueError, json.JSONDecodeError) as e:
                self.send(400, {"error": str(e)})

    server = ThreadingHTTPServer(("", port), Handler)
    server.daemon_threads = True
    return server


if __name__ == "__main__":
    threading.Thread(target=watch_ovens, daemon=True).start()
    threading.Thread(target=watch_mir, daemon=True).start()
    srv = make_server(Camera(), Jobs())
    print(f"GlasLab control page on http://localhost:{srv.server_address[1]}")
    srv.serve_forever()
