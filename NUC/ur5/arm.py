"""UR5 control: named poses, gripper, and tag-relative pick/place."""
import json
import time

import cv2
import numpy as np

import config
from ur5.vision import hand_eye, inv, to_pose, to_T


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
        """Freedrive to ~15 poses looking at the fixed chessboard -> handeye.npy. Returns spread in mm."""
        base_tcp, cam_tag = [], []
        while self.freedrive(f"Hand-eye pose {len(base_tcp) + 1}: whole chessboard in view, 30-60 cm. Vary position AND "
                             f"tilt/roll a lot between poses. Continue = capture, Done = finish (12+ poses).") != "done":
            T_ct = self.cam.detect_board()
            if T_ct is None:
                self.log("chessboard not found, try again (whole board in view, no glare)")
                continue
            base_tcp.append(self.tcp())
            cam_tag.append(T_ct)
            self.log(f"pose {len(base_tcp)}: board at {np.linalg.norm(T_ct[:3, 3]) * 100:.0f} cm")
        if len(base_tcp) < 3:
            raise RuntimeError(f"only {len(base_tcp)} poses - need at least 3 (better 12+), nothing saved")
        tilt = max(np.degrees(np.linalg.norm(cv2.Rodrigues((inv(A) @ B)[:3, :3])[0]))
                   for A in base_tcp for B in base_tcp)
        self.log(f"{len(base_tcp)} poses, largest rotation between two poses {tilt:.0f} deg (want 12+ poses, 40+ deg)")
        self.X = hand_eye(base_tcp, cam_tag)
        config.DATA.mkdir(parents=True, exist_ok=True)
        np.save(config.DATA / "handeye.npy", self.X)
        # real-data check: the fixed board should land in the same base position from every pose
        pts = np.array([(B @ self.X @ C)[:3, 3] for B, C in zip(base_tcp, cam_tag)])
        spread = np.linalg.norm(pts - pts.mean(0), axis=1) * 1000
        self.log("per pose error (mm): " + " ".join(f"{s:.0f}" for s in spread))  # one big number = one bad capture
        return spread

    # ---- tag-relative targets ----
    def teach(self, name, tag_id):
        self.freedrive("1/3 SEARCH pose: camera ~50-60 cm from the tag, tag in the middle of the image.")
        q_search = list(self.r.getActualQ())
        self.locate(tag_id)  # check the tag is visible from here
        self.freedrive("2/3 CLOSE view: camera ~30 cm from the tag, roughly straight on.")
        T_bt = self.locate(tag_id)
        T_tag_view = inv(T_bt) @ self.tcp()
        self.freedrive("3/3 GRASP pose: gripper exactly around the crucible, come down from above.")
        targets = load("targets")
        targets[name] = {"tag": tag_id, "q_search": q_search, "base_tag": T_bt.tolist(),
                         "tag_view": T_tag_view.tolist(), "tag_grasp": (inv(T_bt) @ self.tcp()).tolist()}
        save("targets", targets)

    def find(self, name):
        """Search from far, refine from the taught close view, return T_base_grasp."""
        t = load("targets")[name]
        self.move("moveJ", t["q_search"], config.JSPEED, config.JACC, "search pose")
        T_bt = self.locate(t["tag"])                                    # coarse, wide view
        off = np.linalg.norm(T_bt[:3, 3] - np.array(t["base_tag"])[:3, 3])
        if off > config.MAX_OFFSET:
            raise RuntimeError(f"tag {t['tag']} is {off * 100:.0f} cm from where it was taught - not moving")
        self.log(f"   tag {t['tag']} found, {off * 100:.1f} cm from where it was taught")
        self.move("moveL", to_pose(T_bt @ np.array(t["tag_view"])), config.SPEED * 3, config.ACC, "close view")
        T_bt = self.locate(t["tag"])                                    # fine, same view as when taught
        return T_bt @ np.array(t["tag_grasp"])

    def _in_out(self, T, action):
        pre = T.copy()
        pre[2, 3] += config.APPROACH  # straight above in the robot base frame: the gripper is made to come down vertically
        self.move("moveL", to_pose(pre), config.SPEED * 3, config.ACC, "approach")
        self.move("moveL", to_pose(T), config.SPEED, config.ACC, "grasp pose")
        action()
        self.move("moveL", to_pose(pre), config.SPEED, config.ACC, "retract")

    def pick(self, name):
        self.gripper(close=False)
        self._in_out(self.find(name), lambda: self.gripper(close=True))

    def place(self, name):
        self._in_out(self.find(name), lambda: self.gripper(close=False))
