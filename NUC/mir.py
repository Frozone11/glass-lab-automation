"""MiR100 control through its REST API (v2.0.0): status, missions, pause/continue.
A station is a MiR mission (made in the MiR web interface), e.g. 'go to oven1'."""
import base64
import hashlib
import json
import time
import urllib.error
import urllib.request

import config

READY, PAUSE = 3, 4   # MiR state ids for PUT /status
_http = urllib.request.build_opener(urllib.request.ProxyHandler({}))  # talk to the MiR directly, never via a proxy


class MiR:
    def __init__(self):
        self.base = f"http://{config.MIR_IP}/api/v2.0.0"
        # MiR API auth: "Basic " + base64("user:" + sha256(password))
        token = f"{config.MIR_USER}:{hashlib.sha256(config.MIR_PASSWORD.encode()).hexdigest()}"
        self.headers = {"Authorization": "Basic " + base64.b64encode(token.encode()).decode(),
                        "Content-Type": "application/json", "Accept-Language": "en_US"}

    def _req(self, method, path, body=None, timeout=5):
        req = urllib.request.Request(self.base + path, None if body is None else json.dumps(body).encode(),
                                     self.headers, method=method)
        try:
            with _http.open(req, timeout=timeout) as r:
                text = r.read().decode()
                return json.loads(text) if text else None
        except urllib.error.HTTPError as e:
            raise RuntimeError(f"MiR {method} {path}: HTTP {e.code} {e.read().decode(errors='replace')[:200]}") \
                from None
        except (urllib.error.URLError, OSError) as e:
            raise RuntimeError(f"MiR not reachable at {config.MIR_IP}: {e}") from None

    def status(self, timeout=5):
        s = self._req("GET", "/status", timeout=timeout)
        return {"state": s.get("state_text"), "battery": s.get("battery_percentage"),
                "mission": s.get("mission_text")}

    def missions(self, timeout=5):
        return sorted(m["name"] for m in self._req("GET", "/missions", timeout=timeout))

    def pause(self):
        self._req("PUT", "/status", {"state_id": PAUSE})

    def resume(self):
        self._req("PUT", "/status", {"state_id": READY})

    def run_mission(self, name, log=print, stopped=lambda: False):
        """Queue a mission by name, set the MiR to Ready (play), and wait until it is done."""
        found = [m["guid"] for m in self._req("GET", "/missions") if m["name"] == name]
        if not found:
            raise RuntimeError(f"no MiR mission called '{name}'")
        entry = self._req("POST", "/mission_queue", {"mission_id": found[0]})
        self.resume()
        log(f"-> MiR: {name}")
        deadline = time.time() + config.MIR_MISSION_TIMEOUT
        while time.time() < deadline:
            if stopped():
                self.pause()
                raise RuntimeError("stopped by user - MiR paused")
            state = self._req("GET", f"/mission_queue/{entry['id']}")["state"]
            if state == "Done":
                log(f"   MiR: {name} done")
                return
            if state == "Aborted":
                raise RuntimeError(f"MiR mission '{name}' was aborted (check the MiR interface)")
            time.sleep(1)
        self.pause()
        raise RuntimeError(f"MiR mission '{name}' not done after {config.MIR_MISSION_TIMEOUT} s - MiR paused")
