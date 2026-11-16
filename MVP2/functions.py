"""
Hardware classes for MVP2.

  Oven  - talks to the ESP32 over HTTP
  UR5   - runs .urp programs through the UR Dashboard Server (port 29999)

Both classes take a `log` function (anything that accepts a string, e.g. print)
and a `dry` flag. In dry mode nothing is sent to the hardware.
Both raise HardwareError when something goes wrong.
"""
import socket
import time
import urllib.error
import urllib.parse
import urllib.request


class HardwareError(Exception):
    """Something went wrong while talking to the oven or the robot."""


def fmt_time(seconds):
    seconds = int(round(seconds))
    h, rest = divmod(seconds, 3600)
    m, s = divmod(rest, 60)
    return f"{h}:{m:02d}:{s:02d}" if h else f"{m}:{s:02d}"


# =============================== Oven ================================

class Oven:
    # Endpoints on the ESP32 firmware. All replies are plain text.
    ENDPOINTS = {
        "door_open":   "/door/open",    # -> "opening" / "already open"
        "door_close":  "/door/close",   # -> "closing" / "already closed"
        "door_status": "/door/status",  # -> "open" / "closed" / "moving" / "unknown"
        "temp_up":     "/temp/up",      # ?steps=N -> "temp up"
        "temp_down":   "/temp/down",    # ?steps=N -> "temp down"
        "stop":        "/stop",         # -> "stopping"
        "nano_status": "/status",       # last Nano reply: idle/pending/moving/done/error/busy/stopping/timeout
    }

    def __init__(self, host, key, log=print, dry=False,
                 http_timeout=5, door_timeout=60, press_time=0.6):
        self.host = host
        self.key = key
        self.log = log
        self.dry = dry
        self.http_timeout = http_timeout
        self.door_timeout = door_timeout   # must be longer than a full door move
        self.press_time = press_time       # PRESS_MS + RELEASE_MS on the ESP32, in seconds
        self.sim_door = "unknown"          # door state used in dry mode
        # Talk to the ESP32 directly, even if the PC has a system proxy configured
        self._http = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def _get(self, endpoint, **params):
        """Send a GET request and return the reply text."""
        params["key"] = self.key
        url = f"http://{self.host}{self.ENDPOINTS[endpoint]}?{urllib.parse.urlencode(params)}"
        if self.dry:
            return ""
        try:
            with self._http.open(url, timeout=self.http_timeout) as r:
                return r.read().decode(errors="replace").strip()
        except urllib.error.HTTPError as e:
            body = e.read().decode(errors="replace").strip()
            raise HardwareError(f"oven {endpoint}: HTTP {e.code} {body}") from None
        except (urllib.error.URLError, OSError) as e:
            raise HardwareError(f"oven not reachable at {self.host}: {e}") from None

    def door_state(self):
        if self.dry:
            return self.sim_door
        return self._get("door_status").lower()

    def status(self):
        if self.dry:
            return {"door": self.sim_door, "nano": "dry-run"}
        return {"door": self.door_state(), "nano": self._get("nano_status")}

    def open_door(self):
        self._move_door("open", "open")

    def close_door(self):
        self._move_door("close", "closed")

    def _move_door(self, action, target):
        self.log(f"oven: door {action}")
        reply = self._get(f"door_{action}")
        if self.dry:
            self.sim_door = target
            return
        self.log(f"oven: {reply}")
        deadline = time.time() + self.door_timeout
        while time.time() < deadline:
            state = self.door_state()
            if state == target:
                self.log(f"oven: door is {state}")
                return
            if state != "moving":
                # The move ended without reaching the target (Nano ERR/BUSY, stop or timeout)
                raise HardwareError(f"door ended up '{state}' instead of '{target}' "
                                    f"(Nano status: {self._get('nano_status')})")
            time.sleep(0.5)
        raise HardwareError(f"door still moving after {self.door_timeout}s "
                            f"(Nano status: {self._get('nano_status')})")

    def temp_up(self, steps):
        self._temp("up", steps)

    def temp_down(self, steps):
        self._temp("down", steps)

    def _temp(self, direction, steps):
        self.log(f"oven: temp {direction} {steps}")
        self._get(f"temp_{direction}", steps=steps)
        if not self.dry:
            # The ESP32 presses the button in the background; wait until it is done
            time.sleep(steps * self.press_time + 0.3)

    def stop(self):
        self._get("stop")


# ================================ UR5 ================================

class UR5:
    def __init__(self, host, log=print, dry=False, port=29999, program_timeout=300):
        self.host = host
        self.port = port
        self.log = log
        self.dry = dry
        self.program_timeout = program_timeout

    def _connect(self):
        try:
            sock = socket.create_connection((self.host, self.port), timeout=5)
        except OSError as e:
            raise HardwareError(f"UR5 not reachable at {self.host}:{self.port}: {e}") from None
        f = sock.makefile("rw", encoding="utf-8", newline="\n")
        f.readline()  # welcome banner
        return sock, f

    @staticmethod
    def _cmd(f, text):
        f.write(text + "\n")
        f.flush()
        return f.readline().strip()

    def _check_safety(self, f):
        mode = self._cmd(f, "safetymode")          # "Safetymode: NORMAL"
        if not any(ok in mode.upper() for ok in ("NORMAL", "REDUCED")):
            raise HardwareError(f"UR5 safety: {mode} (check the teach pendant)")

    def run_program(self, program):
        """Load and play a .urp file, and block until it has finished."""
        self.log(f"UR5: running {program}")
        if self.dry:
            return
        sock, f = self._connect()
        try:
            mode = self._cmd(f, "robotmode")       # "Robotmode: RUNNING"
            if "RUNNING" not in mode.upper():
                raise HardwareError(f"UR5 not ready ({mode}); power on and release brakes")
            self._check_safety(f)

            reply = self._cmd(f, f"load {program}")
            if not reply.lower().startswith("loading program"):
                raise HardwareError(f"UR5 load failed: {reply}")
            reply = self._cmd(f, "play")
            if not reply.lower().startswith("starting program"):
                raise HardwareError(f"UR5 play failed: {reply}")

            t0, seen_playing = time.time(), False
            while True:
                time.sleep(0.25)
                self._check_safety(f)
                state = self._cmd(f, "programState")   # "PLAYING x.urp" / "STOPPED x.urp" / "PAUSED x.urp"
                word = state.split()[0].upper() if state else ""
                if word == "PLAYING":
                    seen_playing = True
                elif word == "PAUSED":
                    raise HardwareError(f"UR5 program paused ({state}); resolve on the pendant")
                elif word == "STOPPED" and (seen_playing or time.time() - t0 > 2):
                    if not seen_playing:
                        self.log("UR5: warning, program was never seen running")
                    self.log(f"UR5: {program} finished ({fmt_time(time.time() - t0)})")
                    return
                if time.time() - t0 > self.program_timeout:
                    self._cmd(f, "stop")
                    raise HardwareError(f"UR5 program exceeded {self.program_timeout}s and was stopped")
        finally:
            sock.close()

    def status(self):
        if self.dry:
            return {"dry-run": True}
        sock, f = self._connect()
        try:
            return {cmd: self._cmd(f, cmd) for cmd in ("robotmode", "safetymode", "programState")}
        finally:
            sock.close()

    def stop(self):
        if self.dry:
            return
        sock, f = self._connect()
        try:
            self._cmd(f, "stop")
        finally:
            sock.close()