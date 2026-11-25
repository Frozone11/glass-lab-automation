"""Demos: short typed recipes for showing what the system can do, one command per line.

    mir <mission name>        drive the MiR (the arm goes home first)
    pick <target>             pick at a taught target (opens/closes the oven door if the target has one)
    place <target>            place at a taught target
    door open|close <oven>    move an oven door by hand
    home                      arm to the home pose
    wait <seconds>            pause
    # anything after a hash is a comment
"""
import time

import config
from ur5.arm import load


def parse(text, missions=()):
    """-> list of (line number, command, argument). Raises ValueError naming the first bad line.
    missions: MiR mission names, checked when known (empty = MiR not reachable, not checked)."""
    targets, ovens = load("targets"), config.OVENS
    steps = []
    for n, raw in enumerate(text.splitlines(), 1):
        line = raw.split("#")[0].strip()
        if not line:
            continue
        cmd, _, arg = line.partition(" ")
        cmd, arg = cmd.lower(), arg.strip()
        if cmd == "mir":
            if not arg:
                raise ValueError(f"line {n}: mir needs a mission name")
            if missions and arg not in missions:
                raise ValueError(f"line {n}: no MiR mission called '{arg}'")
        elif cmd in ("pick", "place"):
            if arg not in targets:
                raise ValueError(f"line {n}: no taught target '{arg}'")
        elif cmd == "door":
            how, _, oven = arg.partition(" ")
            if how not in ("open", "close") or oven.strip() not in ovens:
                raise ValueError(f"line {n}: use 'door open <oven>' or 'door close <oven>' "
                                 f"(ovens: {', '.join(ovens) or 'none'})")
            arg = (how, oven.strip())
        elif cmd == "home":
            if "home" not in load("poses"):
                raise ValueError(f"line {n}: no 'home' pose saved yet")
        elif cmd == "wait":
            try:
                arg = float(arg)
            except ValueError:
                raise ValueError(f"line {n}: wait needs a number of seconds") from None
            if not 0 < arg <= 3600:
                raise ValueError(f"line {n}: wait 0-3600 s")
        else:
            raise ValueError(f"line {n}: unknown command '{cmd}' (use mir, pick, place, door, home, wait)")
        steps.append((n, cmd, arg))
    if not steps:
        raise ValueError("the recipe is empty")
    return steps


def run(ur, steps, jobs, mir, oven):
    """Run parsed steps in order on a connected UR5. mir: a MiR client, oven: Oven class (passed in for tests).
    jobs.step tells the page which step is running."""
    for i, (n, cmd, arg) in enumerate(steps):
        if jobs.stop_requested:
            raise RuntimeError("stopped by user")
        jobs.step = i
        jobs.say(f"step {i + 1}/{len(steps)}: {cmd} {' '.join(arg) if isinstance(arg, tuple) else arg}".rstrip())
        if cmd == "mir":
            ur.go("home")  # never drive with the arm out
            mir.run_mission(arg, ur.log, lambda: jobs.stop_requested)
        elif cmd in ("pick", "place"):
            getattr(ur, cmd)(arg)
            if "home" in load("poses"):
                ur.go("home")
        elif cmd == "door":
            how, name = arg
            getattr(oven(name), "open" if how == "open" else "close")(ur.log)
        elif cmd == "home":
            ur.go("home")
        elif cmd == "wait":
            end = time.time() + arg
            while time.time() < end:
                if jobs.stop_requested:
                    raise RuntimeError("stopped by user")
                time.sleep(0.2)
    jobs.step = len(steps)  # all done
