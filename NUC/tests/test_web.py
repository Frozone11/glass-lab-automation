"""Web server tests with a fake camera, no robot:  python -m pytest"""
import json
import threading
import time
import urllib.error
import urllib.request

import cv2
import numpy as np
import pytest

import config
import web
from ur5.vision import DICT


@pytest.fixture(autouse=True)
def no_ovens(monkeypatch):
    monkeypatch.setattr(config, "OVENS", {})  # STOP would otherwise try to reach the real oven ESP32


def wait_until(cond, timeout=3):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.02)
    return False


def test_job_waits_for_user():
    jobs, answers = web.Jobs(), []

    def job(j):
        while (a := j.wait("go?")) == "next":
            answers.append(a)

    jobs.start("demo", job)
    for key in ("next", "next", "done"):
        assert wait_until(lambda: jobs.waiting)
        assert jobs.msg == "go?"
        jobs.answer(key)
        wait_until(lambda: not jobs.waiting)
    assert wait_until(lambda: jobs.name is None)
    assert answers == ["next", "next"] and "finished" in jobs.log[-1]


def test_stop_ends_waiting_job():
    jobs = web.Jobs()
    jobs.start("demo", lambda j: j.wait("go?"))
    assert wait_until(lambda: jobs.waiting)
    jobs.stop()
    assert wait_until(lambda: jobs.name is None)
    assert "stopped by user" in jobs.log[-1]


def test_second_job_refused_while_busy():
    jobs = web.Jobs()
    jobs.start("demo", lambda j: j.wait("go?"))
    assert wait_until(lambda: jobs.waiting)
    with pytest.raises(ValueError, match="still running"):
        jobs.start("other", lambda j: None)
    jobs.stop()


class FakeWebCam:
    calib_error = "no camera.npz - run intrinsics first"

    def __init__(self):
        self.img = np.full((480, 640, 3), 255, np.uint8)
        self.img[140:340, 220:420] = cv2.cvtColor(cv2.aruco.generateImageMarker(DICT, 3, 200), cv2.COLOR_GRAY2BGR)

    def latest(self):
        return self.img.copy()

    def calibrate(self, wait, log):
        while wait("show the board") == "next":
            log("capture")


@pytest.fixture
def server(tmp_path, monkeypatch):
    monkeypatch.setattr(config, "DATA", tmp_path)
    jobs = web.Jobs()
    srv = web.make_server(FakeWebCam(), jobs, port=0)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield f"http://127.0.0.1:{srv.server_address[1]}", jobs
    srv.shutdown()


def get(url):
    with urllib.request.urlopen(url, timeout=3) as r:
        return r.status, r.read()


def post(url, data=None):
    req = urllib.request.Request(url, json.dumps(data or {}).encode(), method="POST")
    try:
        with urllib.request.urlopen(req, timeout=3) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def test_page_and_status(server):
    url, _ = server
    code, body = get(url + "/")
    assert code == 200 and b"GlasLab control" in body
    s = json.loads(get(url + "/status")[1])
    assert s["camera"] and s["tags_seen"] == [3]
    assert s["job"] is None and s["targets"] == [] and s["handeye"] == "not done"


def test_stream_sends_jpeg_frames(server):
    url, _ = server
    with urllib.request.urlopen(url + "/stream", timeout=3) as r:
        head = r.read(300)
    assert head.startswith(b"--frame") and b"image/jpeg" in head


def test_bad_requests_rejected(server):
    url, _ = server
    assert post(url + "/api/teach", {"name": "bad name!", "tag": 1})[0] == 400
    assert post(url + "/api/teach", {"name": "oven1", "tag": 99})[0] == 400
    assert post(url + "/api/teach", {"name": "oven1", "tag": 1, "oven": "nope"})[0] == 400
    assert post(url + "/api/door", {"oven": "nope", "open": True})[0] == 400
    assert post(url + "/api/pick", {"name": "nothere"})[0] == 400
    assert post(url + "/api/home")[0] == 400          # no home pose saved yet
    assert post(url + "/api/answer", {"key": "x"})[0] == 400
    assert post(url + "/api/bogus")[0] == 400


def test_calibration_job_through_http(server):
    url, jobs = server
    assert post(url + "/api/intrinsics") == (200, {"ok": True})
    assert wait_until(lambda: json.loads(get(url + "/status")[1])["waiting"])
    post(url + "/api/answer", {"key": "next"})
    assert wait_until(lambda: "capture" in jobs.log[-1])
    assert wait_until(lambda: jobs.waiting)
    post(url + "/api/answer", {"key": "done"})
    assert wait_until(lambda: json.loads(get(url + "/status")[1])["job"] is None)
    assert "finished" in jobs.log[-1]


def test_delete_target_and_pose(server):
    url, _ = server
    (config.DATA / "targets.json").write_text(json.dumps({"spot": {"tag": 0}, "oven1": {"tag": 1}}))
    (config.DATA / "poses.json").write_text(json.dumps({"home": [0] * 6}))
    assert post(url + "/api/delete", {"kind": "targets", "name": "spot"}) == (200, {"ok": True})
    assert post(url + "/api/delete", {"kind": "poses", "name": "home"}) == (200, {"ok": True})
    s = json.loads(get(url + "/status")[1])
    assert s["targets"] == [{"name": "oven1", "tag": 1, "oven": None}] and s["poses"] == []
    assert post(url + "/api/delete", {"kind": "targets", "name": "spot"})[0] == 400   # already gone
    assert post(url + "/api/delete", {"kind": "../etc", "name": "x"})[0] == 400
