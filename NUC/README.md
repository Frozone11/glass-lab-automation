# GlasLab NUC control

Runs on the NUC on the MiR. Will pull together MiR, UR5 and ovens; for now only the UR5.

```
config.py        all settings (IPs, camera, tag size, gripper outputs, speeds)
ur5/vision.py    camera, AprilTag detection, transform math, hand-eye solver
ur5/arm.py       UR5 class: named poses, gripper, teach/pick/place relative to a tag
ur5_cli.py       command line for setup and testing
tests/           offline tests (fake robot + camera)
data/            calibration and taught poses (created by the CLI)
```

## Setup
```bash
pip install -r requirements.txt
python -m pytest              # offline tests, should all pass
```
Edit `config.py`: `ROBOT_IP`, `CAM`, `TAG_SIZE` (measured), `GRIP_OPEN_DO` / `GRIP_CLOSE_DO`.

## Milestone 1: place, drive away, come back, pick
1. `python ur5_cli.py intrinsics` - camera calibration with the chessboard (error < 0.5 px)
2. `python ur5_cli.py handeye 0` - tag 0 taped flat on the table, ~15 tilted poses (spread < 2-3 mm)
3. `python ur5_cli.py save-pose home` - a safe pose with the arm tucked in
4. Put the crucible on the table next to the tag.
   `python ur5_cli.py teach spot 0` - far search pose (~50-60 cm), close view (~30 cm), then the grasp pose
5. `python ur5_cli.py pick spot` - test with the MiR not moved
6. `python ur5_cli.py place spot`, drive the MiR away and back, `python ur5_cli.py pick spot`

Keep a hand on the E-stop for the first runs: the arm drives to computed poses.
