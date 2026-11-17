"""Command line for setting up and testing the UR5.

Setup (once):
    python ur5_cli.py intrinsics          # chessboard -> data/camera.npz
    python ur5_cli.py handeye 0           # fixed tag 0, ~15 freedrive poses -> data/handeye.npy
    python ur5_cli.py save-pose home      # freedrive to a safe home pose and save it
    python ur5_cli.py teach spot 0        # camera view + grasp pose relative to tag 0

Use:
    python ur5_cli.py grip close|open
    python ur5_cli.py place spot          # put the held crucible down at 'spot'
    python ur5_cli.py pick spot           # pick it up again (after driving away and back)
"""
import sys

from ur5 import UR5, Camera


def main(cmd, *args):
    cam = Camera()
    if cmd == "intrinsics":
        return cam.calibrate()
    ur = UR5.connect(cam)
    if cmd == "handeye":
        spread = ur.calibrate_handeye(int(args[0]))
        print(f"tag position spread: mean {spread.mean():.1f} mm, max {spread.max():.1f} mm (want < 2-3 mm)")
    elif cmd == "save-pose":
        ur.freedrive(f"Move to '{args[0]}'")
        ur.save_pose(args[0])
    elif cmd == "teach":
        ur.teach(args[0], int(args[1]))
    elif cmd == "grip":
        ur.gripper(close=args[0] == "close")
    elif cmd in ("pick", "place"):
        getattr(ur, cmd)(args[0])
        ur.go("home")
    else:
        print(__doc__)


if __name__ == "__main__":
    main(*sys.argv[1:]) if len(sys.argv) > 1 else print(__doc__)
