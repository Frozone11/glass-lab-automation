# GlasLab NUC control

Runs on the NUC on the MiR and pulls MiR, UR5 and ovens together. For now: UR5 calibration, teaching and pick/place,
all from a web page.

```
web.py           web server: control page, live camera, jobs (python web.py)
demo.py          demo recipes (mir / pick / place / door / home / wait), checked then run step by step
mir.py           MiR100 REST API: status, missions (stations), pause/continue
oven.py          oven door open/close through the oven's ESP32 (HTTP, same firmware/API as MVP2), waits for the door
web/index.html   the control page
config.py        all settings (IPs, camera, tag size, gripper outputs, speeds, port)
ur5/vision.py    camera, ArUco/chessboard detection, transform math, hand-eye solver
ur5/arm.py       UR5 class: named poses, gripper, teach/pick/place relative to a tag
tests/           offline tests (fake robot + camera):  python -m pytest
glaslab.service  starts web.py on boot
~/glaslab-data/  calibration and taught poses, per device, created automatically (not in git)
```

## Setup
```bash
uv venv --python 3.12 && source .venv/bin/activate
uv pip install -r requirements.txt
python -m pytest
```
Edit `config.py`: `ROBOT_IP`, `CAM`, `TAG_SIZE` and `SQUARE` (measured), `GRIP_OPEN_DO` / `GRIP_CLOSE_DO`,
`OVENS` (each oven's ESP32 IP and API key, the same as `OVEN_HOST` / `OVEN_KEY` in MVP2), and `MIR_IP` /
`MIR_USER` / `MIR_PASSWORD`.

## Use
`python web.py`, then open http://localhost:8000 on the NUC, or http://<NUC-IP>:8000 from another computer
on the same network. When a step needs you, the page shows what to do: move the arm with the pendant, then
**Continue**. **Done** ends a loop (calibration captures). **STOP** halts the arm - the E-stop is still the real one.

1. Camera calibration - chessboard, ~20 captures (error < 0.5 px)
2. Hand-eye calibration - chessboard fixed, 12+ tilted poses (spread < 2-3 mm)
3. Save pose `home` - tucked in over the platform (the MiR only drives with the arm here)
4. Set search pose - one general view out over the side of the MiR at table height, used for every station
5. Teach a target per oven (e.g. `oven1`, tag 1 on the oven door, oven `oven1`), with the MiR parked so the tag is
   in view from the search pose: close view (door closed), clear pose (arm out of the door's way), the door opens,
   waypoints on the way in (e.g. the oven approach; Done when finished), approach straight above the spot, grasp.
   All of it is stored relative to the tag, so the oven can be moved: just park the MiR in front of it again.
6. MiR: run a mission (station, made in the MiR web interface) with Go - the arm goes home first.
   Check a target: is the MiR parked within the window (`PARK_TOL`, `PARK_YAW`)? If not, it says how far off.
7. Pick / Place

## Demos
The **Demos** menu shows a clean view for presenting: pick a demo, Run, and watch the steps tick off with the system
state, a small camera and the latest log beside them. Demos are typed recipes, one command per line, saved per device
in `~/glaslab-data/demos.json`:
```
mir go to oven1      # a MiR mission (the arm goes home first)
pick oven1           # a taught target
mir go to table
place spot
```
Also `door open|close <oven>`, `home`, `wait <seconds>`. The whole recipe is checked before anything moves.

## Run on boot
```bash
sudo cp glaslab.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now glaslab
```
The door state is stored by the ESP32, not measured. If a door move is interrupted (STOP also stops the door), the
state becomes `unknown`: bring the door back to fully closed with the raw stepper controls at
`http://<ESP32 IP>/debug` before running again.

Logs: `journalctl -u glaslab -f`. After a `git pull`: `sudo systemctl restart glaslab`.
The service assumes user `robotlab` and the repo in `~/glass-lab-automation`; edit the paths in
`glaslab.service` if that differs.
