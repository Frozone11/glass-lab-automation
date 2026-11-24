"""UR5 control: named poses, gripper, and tag-relative pick/place."""
import json
import time

import cv2
import numpy as np

import config
from oven import Oven
from ur5.vision import BOARD_CENTRE, board_pose_from_corners, hand_eye, inv, orbit, refine_handeye, to_pose, to_T


def load(name):
    f = config.DATA / f"{name}.json"
    return json.loads(f.read_text()) if f.exists() else {}


def save(name, data):
    config.DATA.mkdir(parents=True, exist_ok=True)
    (config.DATA / f"{name}.json").write_text(json.dumps(data, indent=1))


class UR5:
    def __init__(self, c, r, io, cam, wait, log=print):
        """c/r/io: ur_rtde control/receive/io interfaces (use UR5.connect() for the real robot).
        wait(msg): show msg to the user, block until they answer 'next' or 'done'. log(line): progress output."""
        self.c, self.r, self.io, self.cam, self.wait, self.log = c, r, io, cam, wait, log
        f = config.DATA / "handeye.npy"
        self.X = np.load(f) if f.exists() else None

    @classmethod
    def connect(cls, cam, wait, log=print, ip=config.ROBOT_IP):
        import rtde_control, rtde_io, rtde_receive
        return cls(rtde_control.RTDEControlInterface(ip), rtde_receive.RTDEReceiveInterface(ip),
                   rtde_io.RTDEIOInterface(ip), cam, wait, log)

    def close(self):
        for i in (self.c, self.r, self.io):
            try:
                i.disconnect()
            except Exception:
                pass

    # ---- basics ----
    def move(self, kind, target, speed, acc, what):
        """moveJ/moveL that fails loudly: ur_rtde just returns False when the robot refuses a move."""
        self.log(f"-> {what}")
        if not self.c.isProgramRunning():  # e.g. stopped by jogging on the pendant: moves then "succeed" without moving
            self.log("   control script not running (pendant used?) - restarting it")
            self.c.reuploadScript()
        if not getattr(self.c, kind)(list(target), speed, acc):
            raise RuntimeError(f"{kind} to {what} refused - check the pendant: protective stop, popup open, "
                               f"brakes not released, or pose out of reach")

    def tcp(self):
        return to_T(self.r.getActualTCPPose())

    def freedrive(self, msg):
        """Hand the arm to the user (pendant jogging or its freedrive button); returns 'next' or 'done'.
        Our control script is stopped so the pendant can move the arm; move() restarts it when needed."""
        self.c.stopScript()
        answer = self.wait(msg + " Move the arm with the pendant.")
        time.sleep(0.5)  # let the arm settle before grabbing images
        return answer

    def gripper(self, close):
        self.io.setStandardDigitalOut(config.GRIP_OPEN_DO, not close)
        self.io.setStandardDigitalOut(config.GRIP_CLOSE_DO, close)
        time.sleep(config.GRIP_TIME)

    def go(self, name):
        """Joint move to a pose saved with save_pose (fixed relative to the MiR: home, park, ...)."""
        self.move("moveJ", load("poses")[name], config.JSPEED, config.JACC, name)

    def save_pose(self, name):
        poses = load("poses")
        poses[name] = list(self.r.getActualQ())
        save("poses", poses)

    # ---- vision ----
    def locate(self, tag_id):
        """T_base_tag = T_base_tcp @ T_tcp_cam @ T_cam_tag."""
        if self.X is None:
            raise RuntimeError(f"no {config.DATA / 'handeye.npy'} - run handeye first")
        T_ct = self.cam.detect(tag_id)
        if T_ct is None:
            raise RuntimeError(f"tag {tag_id} not visible")
        return self.tcp() @ self.X @ T_ct

    def calibrate_handeye(self):
        """Manual: user jogs to ~15 poses looking at the fixed chessboard -> handeye.npy. Returns spread in mm."""
        base_tcp, corners = [], []
        while self.freedrive(f"Hand-eye pose {len(base_tcp) + 1}: whole chessboard in view, 30-60 cm. Vary position AND "
                             f"tilt/roll a lot between poses. Continue = capture, Done = finish (12+ poses).") != "done":
            self._capture_board(base_tcp, corners, f"pose {len(base_tcp) + 1}")
        return self._finish_handeye(base_tcp, corners)

    def auto_handeye(self):
        """Automatic: chessboard fixed on the MiR deck. From the saved 'board_view' pose the arm wiggles for a rough
        calibration, then orbits the board centre with big tilts and rolls, and solves. Returns spread in mm."""
        if "board_view" not in load("poses"):
            self.freedrive("First time only: point the camera at the WHOLE chessboard on the MiR, ~35 cm away, roughly "
                           "straight on, with room around the arm. This view is saved as 'board_view'.")
            self.save_pose("board_view")
        self.go("board_view")
        base_tcp, corners = [], []
        if not self._capture_board(base_tcp, corners, "start"):
            raise RuntimeError("chessboard not visible from 'board_view' - delete that pose and set it again")
        T0 = self.tcp()
        # 1) small wiggles about the tool axes -> rough camera position on the wrist
        for axis in np.eye(3):
            for sign in (1, -1):
                W = np.eye(4)
                W[:3, :3] = cv2.Rodrigues(axis * sign * np.radians(6))[0]
                if self._visit(T0 @ W, "wiggle"):
                    self._capture_board(base_tcp, corners, "wiggle")
        if len(base_tcp) < 4:
            raise RuntimeError("board lost during the small moves - start with the board more centred in the image")
        poses = [board_pose_from_corners(c, self.cam.K, self.cam.dist) for c in corners]
        X = hand_eye(base_tcp, poses)
        # 2) orbit the board centre: same distance, board stays mid-image
        centre = (base_tcp[0] @ X @ poses[0] @ np.array([*BOARD_CENTRE, 0, 1]))[:3]
        for i, cam in enumerate(orbit(base_tcp[0] @ X, centre)):
            if self._visit(cam @ inv(X), f"orbit {i + 1}"):
                self._capture_board(base_tcp, corners, f"orbit {i + 1}")
        self.go("board_view")
        return self._finish_handeye(base_tcp, corners)

    def _visit(self, T, what):
        """Move to tool pose T if the robot allows it; False if it is outside the safety limits."""
        pose = to_pose(T)
        if not self.c.isPoseWithinSafetyLimits(pose):
            self.log(f"   {what}: outside safety limits, skipped")
            return False
        self.move("moveL", pose, config.SPEED * 3, config.ACC, what)
        time.sleep(0.3)  # let the arm settle
        return True

    def _capture_board(self, base_tcp, cam_board, what):
        """Store the tool pose and the averaged chessboard corners seen from it."""
        corners = self.cam.board_corners()
        if corners is None:
            self.log(f"   {what}: chessboard not found (whole board in view? glare?), skipped")
            return False
        base_tcp.append(self.tcp())
        cam_board.append(corners)
        dist = np.linalg.norm(board_pose_from_corners(corners, self.cam.K, self.cam.dist)[:3, 3])
        self.log(f"   {what}: board at {dist * 100:.0f} cm ({len(base_tcp)} captured)")
        return True

    def _finish_handeye(self, base_tcp, corners):
        if len(base_tcp) < 3:
            raise RuntimeError(f"only {len(base_tcp)} poses - need at least 3 (better 12+), nothing saved")
        K, dist = self.cam.K, self.cam.dist
        cam_board = [board_pose_from_corners(c, K, dist) for c in corners]
        tilt = max(np.degrees(np.linalg.norm(cv2.Rodrigues((inv(A) @ B)[:3, :3])[0]))
                   for A in base_tcp for B in base_tcp)
        self.log(f"{len(base_tcp)} poses, largest rotation between two poses {tilt:.0f} deg (want 12+ poses, 40+ deg)")
        X0 = hand_eye(base_tcp, cam_board)
        self.X, _, px = refine_handeye(base_tcp, corners, X0, K, dist)
        moved = np.linalg.norm(self.X[:3, 3] - X0[:3, 3]) * 1000
        self.log(f"refined with all corners: {px:.2f} px per corner left (camera mount moved {moved:.1f} mm)")
        config.DATA.mkdir(parents=True, exist_ok=True)
        np.save(config.DATA / "handeye.npy", self.X)
        # real-data check: the fixed board should land in the same base position from every pose
        pts = np.array([(B @ self.X @ C)[:3, 3] for B, C in zip(base_tcp, cam_board)])
        spread = np.linalg.norm(pts - pts.mean(0), axis=1) * 1000
        self.log("per pose error (mm): " + " ".join(f"{s:.0f}" for s in spread))  # one big number = one bad capture
        return spread

    # ---- tag-relative targets ----
    def teach(self, name, tag_id, oven=None):
        """Search pose, close view, any number of waypoints on the way in (e.g. 'out a bit', 'oven entrance'),
        the pick/place approach straight above the spot, and the grasp pose. All but the search pose are stored
        relative to the tag, so they follow it. oven: name in config.OVENS whose door opens after the close view
        (the tag may sit on that door: it is only measured while the door is closed)."""
        self.freedrive("SEARCH pose: camera ~50-60 cm from the tag, tag in the middle of the image.")
        q_search = list(self.r.getActualQ())
        self.locate(tag_id)  # check the tag is visible from here
        self.freedrive("CLOSE view: camera ~20-30 cm from the tag, roughly straight on." +
                       (" The door must be closed." if oven else ""))
        T_bt = self.locate(tag_id)
        if oven:
            Oven(oven).open(self.log)
        rel = lambda: (inv(T_bt) @ self.tcp()).tolist()
        tag_view, path = rel(), []
        while self.freedrive(f"WAYPOINT {len(path) + 1} on the way in (e.g. out from the MiR, then the oven entrance). "
                             f"Continue = save this point, Done = no more waypoints (doesn't save).") != "done":
            path.append(rel())
            self.log(f"   waypoint {len(path)} saved")
        self.freedrive("Pick/place APPROACH: straight above the crucible spot, gripper in its final orientation.")
        T_app = self.tcp()
        self.freedrive("GRASP pose: lower straight down onto the crucible spot.")
        T_grasp = self.tcp()
        height = T_app[2, 3] - T_grasp[2, 3]
        sideways = np.linalg.norm(T_app[:2, 3] - T_grasp[:2, 3]) * 1000
        if height < 0.005:
            raise RuntimeError("the approach must be above the grasp pose - nothing saved")
        if sideways > 5:
            self.log(f"   note: approach was {sideways:.0f} mm off to the side - it will go straight down instead")
        targets = load("targets")
        targets[name] = {"tag": tag_id, "oven": oven, "q_search": q_search, "base_tag": T_bt.tolist(),
                         "tag_view": tag_view, "path": path, "approach_height": float(height), "tag_grasp": rel()}
        save("targets", targets)
        self.log(f"   {len(path)} waypoints, approach {height * 100:.1f} cm above the grasp pose")
        if oven:
            self.log(f"   {oven} door left open - jog the arm out before closing it")

    @staticmethod
    def tag(t):
        return t["tag"] if "tag" in t else t["tags"][0]  # targets taught with the old multi-tag version

    def find(self, name):
        """Search from far, refine from the taught close view; returns the tag pose in the base frame."""
        t = load("targets")[name]
        tag = self.tag(t)
        self.move("moveJ", t["q_search"], config.JSPEED, config.JACC, "search pose")
        T_bt = self.locate(tag)                                         # coarse, wide view
        off = np.linalg.norm(T_bt[:3, 3] - np.array(t["base_tag"])[:3, 3])
        if off > config.MAX_OFFSET:
            raise RuntimeError(f"tag {tag} is {off * 100:.0f} cm from where it was taught - not moving")
        self.log(f"   tag {tag} found, {off * 100:.1f} cm from where it was taught")
        self.move("moveL", to_pose(T_bt @ np.array(t["tag_view"])), config.SPEED * 3, config.ACC, "close view")
        return self.locate(tag)                                         # fine, same view as when taught

    def _run(self, name, action):
        """close view -> (open door) -> waypoints -> approach -> straight down -> action -> straight up ->
        waypoints backwards -> (close door)."""
        t = load("targets")[name]
        T_bt = self.find(name)  # with a door: measured while it is still closed
        oven = Oven(t["oven"]) if t.get("oven") else None
        if oven:
            oven.open(self.log)
        path = [T_bt @ np.array(P) for P in t.get("path", [])]
        grasp = T_bt @ np.array(t["tag_grasp"])
        pre = grasp.copy()
        pre[2, 3] += t.get("approach_height", config.APPROACH)  # straight above in the base frame: vertical gripper
        for i, P in enumerate(path):
            self.move("moveL", to_pose(P), config.SPEED * 3, config.ACC, f"waypoint {i + 1}")
        self.move("moveL", to_pose(pre), config.SPEED * 2, config.ACC, "approach")
        self.move("moveL", to_pose(grasp), config.SPEED, config.ACC, "grasp pose")
        action()
        self.move("moveL", to_pose(pre), config.SPEED, config.ACC, "back up")
        for i, P in reversed(list(enumerate(path))):
            self.move("moveL", to_pose(P), config.SPEED * 3, config.ACC, f"waypoint {i + 1}")
        if oven:
            oven.close(self.log)

    def pick(self, name):
        self.gripper(close=False)
        self._run(name, lambda: self.gripper(close=True))

    def place(self, name):
        self._run(name, lambda: self.gripper(close=False))
