# VT3 Glass Lab — Physical MiR100

ROS1 Noetic navigation for the physical MiR100 using `mir_actions/MirMoveBaseAction`.

**Robot IP:** `192.168.12.20`  
**ROS Master:** `http://192.168.12.20:11311`  
**Current map:** GlassLab1  
**Map ID:** `143e33ad-d6c5-11f1-aad3-b8aeed7197f8`

The system runs inside Docker and connects to the MiR over Wi-Fi. **Dry-run is enabled by default.**

## 1. Start Docker

On Omarchy:

```bash
docker exec -it vt3-ros1-gui bash
```

Inside Docker:

```bash
cd ~/catkin_ws/physical_mir/physical_mir_ros_scripts
```

`ros_config.py` automatically configures the ROS connection. Your laptop must be connected to `mir52`, and its IP must match the configuration.

Do not start another `roscore`, Gazebo, AMCL, or navigation stack.

## 2. Useful commands

| Purpose | Command |
|---|---|
| Robot position | `rostopic echo -n 1 /robot_pose` |
| Localization | `rostopic echo -n 1 /localization_score` |
| Navigation status | `rostopic echo -n 1 /move_base/status` |
| Check ROS connection | `rostopic list` |
| Validate configuration | `python3 mission_controller.py --validate-only` |
| Run tests | `python3 -m unittest discover -s tests -v` |
| Run dry-run | `python3 mission_controller.py` |

## 3. Mission configuration

Edit `mission_config.json`:

```json
{
  "approved_map_id": "143e33ad-d6c5-11f1-aad3-b8aeed7197f8",
  "navigation_timeout_seconds": 120,
  "waypoints": [
    {
      "x": 31.873,
      "y": 20.157,
      "orientation_deg": -91.9,
      "oven": false
    }
  ]
}
```

Coordinates are examples from a recorded robot pose. Verify each destination on the active map.

Set `oven: true` for waypoints requiring oven interaction.

## 4. Navigation

`physical_mir_navigation.py` communicates directly with the MiR's `/move_base` ROS action server.

It supports goal creation, progress monitoring, timeouts, goal-specific cancellation requests, and success/failure reporting.

Failed navigation stops the mission before subsequent waypoints or oven operations.

**Still to verify before live movement:**

- MiR-specific `collision_detection` and `collision_avoidance` settings
- Navigation result semantics and cancellation behaviour
- Correct localization, map coordinates, and robot safety state

Do not assume default collision-related values are safe.

## 5. Oven integration

`mission_controller.py` communicates with the existing `oven_node.py` through:

- `/oven/open` — ROS Trigger service
- `/oven/close` — ROS Trigger service
- `/oven/status` — ROS String topic

At an oven waypoint, the controller waits for `CLOSED`, opens the oven, waits for `OPEN`, closes it, and waits for `CLOSED` again.

Oven commands are disabled during dry-run.

## 6. Running a mission

**Dry-run (safe, no hardware commands):**

```bash
python3 mission_controller.py
```

**Live execution — only after all safety and navigation checks are complete:**

```bash
python3 mission_controller.py --execute --confirm-safe
```

The only manual execution confirmation is `--execute --confirm-safe`. Live mode checks the active map against `approved_map_id` and requires `Ready` in `Mission` mode before each waypoint.

During initial live preflight only, `--execute --confirm-safe` confirms that it is safe to resume a paused robot. The controller requires Mission mode, the approved map, and an empty `errors` list. For Pause (state 4), it sends `PUT /api/v2.0.0/status` with `{"state_id": 3}` and waits up to 10 seconds for Ready (state 3), rechecking state, map, mode, and errors on every poll. API errors, unexpected states, and timeout abort the mission. No errors or safety states are cleared. Later Pause states never trigger automatic resume. Dry-run is unchanged.

The endpoint and state IDs follow the operator-supplied MiR100 2.13.5.4 API documentation. This checkout had no REST authentication configuration. The controller accepts an externally supplied `MIR_API_AUTHORIZATION` environment variable containing the complete Authorization header value; it applies it to status GET and PUT requests and refuses resume if missing. Do not store credentials in source or mission JSON. If your deployment uses a different authentication provider, connect that provider before live use.

Before executing, confirm that the route is clear, localization is correct, safety systems are operational, and an independent stop method is available.

## 7. Development workflow

Codex edits files on Omarchy at:

`/home/hlf/workspace/vt3/physicalMir`

Copy changes into Docker:

```bash
docker cp /home/hlf/workspace/vt3/physicalMir/. \
  vt3-ros1-gui:/root/catkin_ws/physical_mir/physical_mir_ros_scripts/
```

Then run validation and dry-run inside Docker.

**Current status:** A physical mission completed with `SUCCEEDED` using `--execute --confirm-safe`. Development checks use mocks and do not send robot commands.