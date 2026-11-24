"""Oven door control through the oven's ESP32 (HTTP, same firmware and API as the MVP2 control script).
Ovens (address + API key) are set in config.OVENS."""
import time
import urllib.error
import urllib.parse
import urllib.request

import config

# ESP32 firmware endpoints; all replies are plain text
ENDPOINTS = {
    "door_open": "/door/open",      # -> "opening" / "already open"
    "door_close": "/door/close",    # -> "closing" / "already closed"
    "door_status": "/door/status",  # -> "open" / "closed" / "moving" / "unknown"
    "stop": "/stop",                # -> "stopping"
    "nano_status": "/status",       # last Nano reply: idle/pending/moving/done/error/busy/stopping/timeout
}
_http = urllib.request.build_opener(urllib.request.ProxyHandler({}))  # talk to the ESP32 directly, never via a proxy


class Oven:
    def __init__(self, name):
        if name not in config.OVENS:
            raise ValueError(f"no oven '{name}' in config.OVENS")
        self.name = name
        self.host, self.key = config.OVENS[name]["host"], config.OVENS[name]["key"]

    def _get(self, endpoint, timeout=5):
        url = f"http://{self.host}{ENDPOINTS[endpoint]}?{urllib.parse.urlencode({'key': self.key})}"
        try:
            with _http.open(url, timeout=timeout) as r:
                return r.read().decode(errors="replace").strip()
        except urllib.error.HTTPError as e:
            raise RuntimeError(f"{self.name} {endpoint}: HTTP {e.code} {e.read().decode(errors='replace').strip()}") \
                from None
        except (urllib.error.URLError, OSError) as e:
            raise RuntimeError(f"{self.name} not reachable at {self.host}: {e}") from None

    def door(self, timeout=5):
        """'open', 'closed', 'moving' or 'unknown' (as stored by the ESP32, not measured)."""
        return self._get("door_status", timeout).lower()

    def open(self, log=print):
        self._move("open", "open", log)

    def close(self, log=print):
        self._move("close", "closed", log)

    def stop(self):
        self._get("stop", timeout=2)

    def _move(self, action, target, log):
        log(f"-> {self.name} door {action}: {self._get('door_' + action)}")
        deadline = time.time() + config.OVEN_DOOR_TIMEOUT
        while time.time() < deadline:
            state = self.door()
            if state == target:
                log(f"   {self.name} door is {state}")
                return
            if state != "moving":  # the move ended without reaching the target (Nano ERR/BUSY, stop or timeout)
                raise RuntimeError(f"{self.name} door ended up '{state}' instead of '{target}' "
                                   f"(Nano status: {self._get('nano_status')})")
            time.sleep(0.5)
        raise RuntimeError(f"{self.name} door still moving after {config.OVEN_DOOR_TIMEOUT} s "
                           f"(Nano status: {self._get('nano_status')})")
