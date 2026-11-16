# MVP2 control system

Runs MVP2: controls the oven (through the ESP32) and the UR5 (through its `.urp` programs), and runs recipes, which are simple step-by-step sequences of oven and robot actions.

## Files

```
main.py          config, recipe checking and running, console prompt
functions.py     Oven class (ESP32 over HTTP) and UR5 class (Dashboard Server)
web.py           web interface (uses main.py and functions.py)
web/index.html   the web page
recipes/         recipe files (.txt)
logs/            run logs, created automatically
```

Python 3.8 or newer. Standard library only, nothing to install.

## Setup

Set the config block at the top of `main.py`:

| Setting | Meaning |
|---|---|
| `OVEN_HOST`, `OVEN_KEY` | ESP32 IP address and API key |
| `UR5_HOST` | UR5 IP address |
| `ROBOT_PROGRAMS` | recipe name → `.urp` file on the robot |
| `NEEDS_OPEN_DOOR` | programs that reach into the oven |
| `DOOR_TIMEOUT` | seconds to wait for a door move, longer than a full move |

The `.urp` files must be on the robot. On an e-Series robot, the pendant must be in Remote Control mode.

## Running

**Console**

```bash
python3 main.py                        # interactive prompt
python3 main.py two_stage_heating      # run a recipe
python3 main.py --dry-run my_recipe    # check and step through without moving anything
```

**Web interface**

```bash
python3 web.py                         # open http://127.0.0.1:8080
python3 web.py --host 0.0.0.0          # reachable from other machines (no login!)
```

The web page has tabs to run recipes (with live progress), edit recipes, control the oven and robot by hand, and read logs. STOP, or Esc, stops the oven and the robot.

Don't run `main.py` and `web.py` against the hardware at the same time.

## Recipes

One command per line in a `.txt` file in `recipes/`. Anything after `#` is a comment.

| Command | Example |
|---|---|
| `door open` / `door close` | `door open` |
| `temp up N` / `temp down N` | `temp up 3` |
| `robot NAME` | `robot oven_place` |
| `wait TIME` | `wait 30`, `wait 30s`, `wait 5m`, `wait 1.5h` |
| `pause [message]` | `pause Check the crucible` |
| `say message` | `say Stage 1 done` |
| `status` | logs oven and robot status |
| `stop` | stops oven and robot motion |

Example:

```
robot pick
door open
robot oven_place
door close
temp up 3
wait 2m
```

The whole recipe is checked before anything moves. `oven_pick` and `oven_place` are refused unless the door is open. This is checked once in the recipe text and again live against the ESP32 before the robot moves.

## Safety

- An error or Ctrl+C (in the console) or STOP (on the web page) stops the oven stepper and the robot program. This is a convenience, not a safety function. The E-stop is still required.
- The door state is stored by the ESP32, not measured. If a door move is interrupted, the state becomes `unknown`. Bring the door back to fully closed with the raw stepper controls at `http://<ESP32 IP>/debug` before running again.
