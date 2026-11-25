"""All settings in one place. Edit these to match the hardware."""
from pathlib import Path

ROBOT_IP = "192.168.12.148"      # UR5 controller
CAM = 0                         # cv2.VideoCapture index of the wrist camera (v4l2-ctl --list-devices)
CAM_RES = (1280, 720)           # recalibrate (intrinsics) if you change this

TAG_DICT = "DICT_4X4_50"        # ArUco 4x4: big cells, so it still reads well when printed small
TAG_SIZE = 0.0798               # edge of the BLACK square, metres - measure the print
BOARD, SQUARE = (9, 6), 0.025   # chessboard inner corners, square size in metres

GRIP_OPEN_DO, GRIP_CLOSE_DO = 0, 1   # control box digital outputs wired to the Schunk EGP
GRIP_TIME = 1.0                      # s to wait for the gripper to finish moving

APPROACH = 0.10                 # m straight above the grasp pose to come down from / go back up to
SPEED, ACC = 0.05, 0.2          # linear moves near the object (m/s, m/s^2) - slow until trusted
JSPEED, JACC = 0.5, 0.5         # joint moves (rad/s, rad/s^2)
MAX_OFFSET = 0.30               # refuse if the tag is further than this from where it was taught (wrong tag / bad detection)

# Oven ESP32s (same firmware as MVP2): name -> IP address and API key. Targets can be linked to an oven.
OVENS = {"oven1": {"host": "192.168.12.50", "key": "oven_control"}}
OVEN_DOOR_TIMEOUT = 30          # s to wait for a door move, must be longer than a full move

WEB_PORT = 8000                 # control page: http://<NUC-IP>:8000

DATA = Path.home() / "glaslab-data"   # calibration + taught poses: per device, outside the repo so git never sees it
