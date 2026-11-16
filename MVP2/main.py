#!/usr/bin/env python3
"""
MVP2 main script: reads recipes and runs them on the oven and the UR5.

Usage:
  python main.py                                 interactive prompt
  python main.py recipes/two_stage_heating.txt   run a recipe
  python main.py --dry-run recipes/x.txt         step through without hardware or waiting
"""
import argparse
import re
import threading
import time
from pathlib import Path

from functions import UR5, HardwareError, Oven, fmt_time

# =============================== CONFIG ===============================

OVEN_HOST = "192.168.12.50"
OVEN_KEY = "oven_control"
UR5_HOST = "192.168.12.148"

# Recipe name -> .urp file on the robot
ROBOT_PROGRAMS = {
    "pick":       "GlassLab_MVP2_pick.urp",        # pick crucible from idle position
    "place":      "GlassLab_MVP2_place.urp",       # place crucible in idle position
    "oven_pick":  "GlassLab_MVP2_oven_pick.urp",   # pick crucible from the oven
    "oven_place": "GlassLab_MVP2_oven_place.urp",  # place crucible in the oven
}
NEEDS_OPEN_DOOR = {"oven_pick", "oven_place"}      # programs that reach into the oven
MAX_TEMP_STEPS = 20                         # sanity limit per temp command (ESP32 allows 50)
DOOR_TIMEOUT = 30                           # s, must be longer than a full door move

RECIPE_DIR = Path("recipes")
LOG_DIR = Path("logs")

HELP = """\
Commands (same in recipe files and at the prompt):
  door open | door close
  temp up N | temp down N
  robot NAME           run the .urp mapped to NAME  (known: {robots})
  wait TIME            e.g. 30, 30s, 5m, 1.5h
  pause [message]      wait for the operator to press Enter
  say message          write a note to the log
  status               print oven and UR5 status
  stop                 stop oven and UR5 motion
Prompt only:
  list                 show recipes in {recipe_dir}/
  run NAME             validate and run a recipe (name or path)
  help | quit
Ctrl+C aborts the current step and stops oven and UR5 motion.
""".format(robots=", ".join(ROBOT_PROGRAMS), recipe_dir=RECIPE_DIR)

# =====================================================================


class RecipeError(Exception):
    """Bad command or recipe. Nothing has been sent to the hardware."""


class Aborted(Exception):
    """The operator stopped the run (used by the web interface)."""


class Log:
    def __init__(self, path=None):
        self.file = open(path, "a", encoding="utf-8") if path else None

    def __call__(self, msg):
        line = f"{time.strftime('%H:%M:%S')}  {msg}"
        print(line, flush=True)
        if self.file:
            self.file.write(f"{time.strftime('%Y-%m-%d')} {line}\n")
            self.file.flush()


# ------------------------------ Recipes ------------------------------

def parse_line(line):
    """Return a list of tokens, or None for blank/comment lines."""
    line = line.split("#", 1)[0].strip()
    if not line:
        return None
    tokens = line.split()
    tokens[0] = tokens[0].lower()
    if tokens[0] not in ("say", "pause", "run"):
        tokens = [t.lower() for t in tokens]
    return tokens


def parse_duration(text):
    m = re.fullmatch(r"(\d+(?:\.\d+)?)(s|m|min|h)?", text.lower())
    if not m:
        raise RecipeError(f"bad time '{text}' (use e.g. 30, 30s, 5m, 1.5h)")
    return float(m.group(1)) * {"s": 1, "m": 60, "min": 60, "h": 3600}[m.group(2) or "s"]


def check_step(tokens, door):
    """Validate one step and return the door state after it.
    door=None skips the door interlock (it is still checked live before robot moves)."""
    cmd, args = tokens[0], tokens[1:]
    if cmd == "door":
        if args not in (["open"], ["close"]):
            raise RecipeError("usage: door open | door close")
        return "open" if args[0] == "open" else "closed"
    if cmd == "temp":
        if len(args) != 2 or args[0] not in ("up", "down") or not args[1].isdigit():
            raise RecipeError("usage: temp up N | temp down N")
        if not 1 <= int(args[1]) <= MAX_TEMP_STEPS:
            raise RecipeError(f"temp steps must be 1..{MAX_TEMP_STEPS}")
        return door
    if cmd == "robot":
        if len(args) != 1 or args[0] not in ROBOT_PROGRAMS:
            raise RecipeError(f"usage: robot NAME  (known: {', '.join(ROBOT_PROGRAMS)})")
        if door is not None and args[0] in NEEDS_OPEN_DOOR and door != "open":
            raise RecipeError(f"'robot {args[0]}' needs the door open, but it is {door} here")
        return door
    if cmd == "wait":
        if len(args) != 1:
            raise RecipeError("usage: wait TIME")
        parse_duration(args[0])
        return door
    if cmd in ("say", "pause"):
        return door
    if cmd in ("status", "stop"):
        if args:
            raise RecipeError(f"'{cmd}' takes no arguments")
        return door
    raise RecipeError(f"unknown command '{cmd}' (type 'help')")


def find_recipe(name):
    """Accept a path, or a bare name from the recipes folder (with or without .txt)."""
    for candidate in (Path(name), RECIPE_DIR / name, RECIPE_DIR / f"{name}.txt"):
        if candidate.is_file():
            return candidate
    raise RecipeError(f"recipe not found: {name}")


def list_recipes():
    return sorted(p.stem for p in RECIPE_DIR.glob("*.txt"))


def check_recipe_text(text):
    """Validate recipe text without touching the hardware.
    Returns (steps, errors): steps is a list of (lineno, tokens), errors a list of
    (lineno, message). The door starts as 'unknown', so a recipe must open it
    before any robot program that reaches into the oven."""
    steps, errors, door = [], [], "unknown"
    for lineno, raw in enumerate(text.splitlines(), 1):
        tokens = parse_line(raw)
        if not tokens:
            continue
        try:
            if tokens[0] == "run":
                raise RecipeError("'run' only works at the prompt")
            door = check_step(tokens, door)
            steps.append((lineno, tokens))
        except RecipeError as e:
            errors.append((lineno, str(e)))
    if not steps and not errors:
        errors.append((0, "recipe has no steps"))
    return steps, errors


def load_recipe(name):
    """Find, read and validate a recipe before anything moves."""
    path = find_recipe(name)
    steps, errors = check_recipe_text(path.read_text(encoding="utf-8"))
    if errors:
        lineno, msg = errors[0]
        raise RecipeError(f"{path.name} line {lineno}: {msg}" if lineno else f"{path.name}: {msg}")
    return path, steps


# ------------------------------ Runner -------------------------------

class Runner:
    def __init__(self, log, dry=False, abort=None, pause=None, progress=None, console=True):
        """abort:    threading.Event; when set, the run stops at the next check (web STOP)
        pause:    function(message) called by 'pause' instead of waiting for Enter
        progress: function(step, total, lineno, tokens) called before each recipe step
        console:  show the wait countdown in the terminal"""
        self.log = log
        self.dry = dry
        self.abort = abort or threading.Event()
        self.pause_fn = pause
        self.progress_fn = progress
        self.console = console
        self.wait_until = None
        self.oven = Oven(OVEN_HOST, OVEN_KEY, log=log, dry=dry, door_timeout=DOOR_TIMEOUT)
        self.ur5 = UR5(UR5_HOST, log=log, dry=dry)

    def execute(self, tokens):
        cmd, args = tokens[0], tokens[1:]
        if cmd == "door":
            self.oven.open_door() if args[0] == "open" else self.oven.close_door()
        elif cmd == "temp":
            steps = int(args[1])
            self.oven.temp_up(steps) if args[0] == "up" else self.oven.temp_down(steps)
        elif cmd == "robot":
            if args[0] in NEEDS_OPEN_DOOR:
                state = self.oven.door_state()     # live interlock
                if state != "open":
                    raise HardwareError(f"refusing 'robot {args[0]}': oven reports door '{state}'")
            self.ur5.run_program(ROBOT_PROGRAMS[args[0]])
        elif cmd == "wait":
            self.wait(parse_duration(args[0]))
        elif cmd == "pause":
            self.log(f"pause: {' '.join(args) or 'paused'}")
            if self.dry:
                pass
            elif self.pause_fn:
                self.pause_fn(" ".join(args))
            else:
                input("  press Enter to continue (Ctrl+C to abort) ")
        elif cmd == "say":
            self.log(f"note: {' '.join(args)}")
        elif cmd == "status":
            self.log(f"oven: {self.oven.status()}")
            self.log(f"UR5: {self.ur5.status()}")
        elif cmd == "stop":
            self.stop_all()

    def wait(self, seconds):
        self.log(f"waiting {fmt_time(seconds)}")
        if self.dry:
            return
        end = time.time() + seconds
        self.wait_until = end
        try:
            while (left := end - time.time()) > 0:
                if self.abort.is_set():
                    raise Aborted("stopped during wait")
                if self.console:
                    print(f"\r  {fmt_time(left)} left   ", end="", flush=True)
                time.sleep(min(0.25, left))
        finally:
            self.wait_until = None
            if self.console:
                print("\r" + " " * 24 + "\r", end="", flush=True)

    def run_recipe(self, name):
        path, steps = load_recipe(name)
        self.log(f"recipe {path.name}: {len(steps)} steps{' (dry run)' if self.dry else ''}")
        t0 = time.time()
        for i, (lineno, tokens) in enumerate(steps, 1):
            if self.abort.is_set():
                raise Aborted("stopped by operator")
            if self.progress_fn:
                self.progress_fn(i, len(steps), lineno, tokens)
            self.log(f"[{i}/{len(steps)}] {' '.join(tokens)}")
            try:
                self.execute(tokens)
            except HardwareError as e:
                if self.abort.is_set():
                    raise Aborted("stopped by operator") from None
                raise HardwareError(f"line {lineno}: {e}") from None
            if self.abort.is_set():
                raise Aborted("stopped by operator")
        self.log(f"recipe done in {fmt_time(time.time() - t0)}")

    def stop_all(self):
        """Best-effort stop of oven and UR5 motion. Not a safety function: use the E-stop."""
        self.log("STOP: stopping oven and UR5")
        for name, fn in (("oven", self.oven.stop), ("UR5", self.ur5.stop)):
            try:
                fn()
            except Exception as e:
                self.log(f"  {name} stop failed: {e}")


# ------------------------------- CLI ---------------------------------

def interactive(runner, log):
    print(HELP)
    while True:
        try:
            line = input("mvp2> ")
        except (EOFError, KeyboardInterrupt):
            print()
            break
        tokens = parse_line(line)
        if not tokens:
            continue
        cmd = tokens[0]
        if cmd in ("quit", "exit"):
            break
        if cmd == "help":
            print(HELP)
            continue
        if cmd == "list":
            print("  " + ("\n  ".join(list_recipes()) or "(no recipes)"))
            continue
        try:
            if cmd == "run":
                if len(tokens) != 2:
                    raise RecipeError("usage: run NAME")
                runner.run_recipe(tokens[1])
            else:
                check_step(tokens, None)
                runner.execute(tokens)
        except RecipeError as e:
            print(f"  {e}")
        except HardwareError as e:
            log(f"ERROR: {e}")
            runner.stop_all()
        except KeyboardInterrupt:
            print()
            log("ABORTED by operator")
            runner.stop_all()


def main():
    ap = argparse.ArgumentParser(description="MVP2: oven + UR5 + recipes")
    ap.add_argument("recipe", nargs="?", help="recipe name or path (omit for interactive prompt)")
    ap.add_argument("--dry-run", action="store_true",
                    help="validate and step through without touching hardware or waiting")
    args = ap.parse_args()

    log_path = None
    if not args.dry_run:
        LOG_DIR.mkdir(exist_ok=True)
        log_path = LOG_DIR / f"mvp2_{time.strftime('%Y%m%d_%H%M%S')}.log"
    log = Log(log_path)
    runner = Runner(log, dry=args.dry_run)

    if not args.recipe:
        interactive(runner, log)
        return 0
    try:
        runner.run_recipe(args.recipe)
        return 0
    except RecipeError as e:
        log(f"RECIPE ERROR: {e}  (nothing was sent to the hardware)")
    except HardwareError as e:
        log(f"ERROR: {e}")
        runner.stop_all()
    except KeyboardInterrupt:
        print()
        log("ABORTED by operator")
        runner.stop_all()
    return 1


if __name__ == "__main__":
    raise SystemExit(main())