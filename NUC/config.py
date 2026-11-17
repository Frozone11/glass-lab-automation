"""All settings in one place. Edit these to match the hardware."""
from pathlib import Path

ROBOT_IP = "192.168.12.148"     # UR5 controller
CAM = 2                         # cv2.VideoCapture index of the wrist camera (v4l2-ctl --list-devices)
CAM_RES = (1280, 720)           # camera resolution

TAG_DICT = "DICT_4X4_50"        # ArUco 4x4: big cells, so it still reads well when printed small
TAG_SIZE = 0.039745             # edge of the BLACK square, metres - measure the print
BOARD, SQUARE = (9, 6), 0.0240633 # chessboard inner corners, square size in metres

GRIP_OPEN_DO, GRIP_CLOSE_DO = 0, 1   # control box digital outputs wired to the Schunk EGP
GRIP_TIME = 1.0                      # s to wait for the gripper to finish moving

APPROACH = 0.10                 # m to back off along tool z before/after grasping
SPEED, ACC = 0.05, 0.2          # linear moves near the object (m/s, m/s^2) - slow until trusted
JSPEED, JACC = 0.5, 0.5         # joint moves (rad/s, rad/s^2)
MAX_OFFSET = 0.30               # refuse if the tag is further than this from where it was taught (wrong tag / bad detection)

DATA = Path(__file__).parent / "data"   # calibration + taught poses end up here
