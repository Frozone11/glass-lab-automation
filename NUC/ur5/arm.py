"""UR5 control: named poses, gripper, and tag-relative pick/place."""
import json
import time

import cv2
import numpy as np

import config
from ur5.vision import hand_eye, inv, shift_z, to_pose, to_T


def load(name):
    f = config.DATA / f"{name}.json"
    return json.loads(f.read_text()) if f.exists() else {}


def save(name, data):
    config.DATA.mkdir(parents=True, exist_ok=True)
    (config.DATA / f"{name}.json").write_text(json.dumps(data, indent=1))


class UR5:
    def __init__(self, c, r, io, cam):
        """c/r/io: ur_rtde control/receive/io interfaces. Use UR5.connect() for the real robot."""
        self.c, self.r, self.io, self.cam = c, r, io, cam
        f = config.DATA / "handeye.npy"
        self.X = np.load(f) if f.exists() else None

    @classmethod
    def connect(cls, cam, ip=config.ROBOT_IP):
        import rtde_control, rtde_io, rtde_receive
        return cls(rtde_control.RTDEControlInterface(ip), rtde_receive.RTDEReceiveInterface(ip),
                   rtde_io.RTDEIOInterface(ip), cam)

    # ---- basics ----
    def move(self, kind, target, speed, acc, what):
        """moveJ/moveL that fails loudly: ur_rtde just returns False when the robot refuses a move."""
        print(f"-> {what}")
        if not getattr(self.c, kind)(list(target), speed, acc):
            raise RuntimeError(f"{kind} to {what} refused - check the pendant: protective stop, popup open, "
                               f"brakes not released, or pose out of reach")

    def tcp(self):
        return to_T(self.r.getActualTCPPose())

    def freedrive(self, msg):
        """Freedrive with a live camera view; returns 'q' if q was pressed."""
        self.c.teachMode()
        key = self.cam.preview(msg)
        self.c.endTeachMode()
        time.sleep(0.5)  # let the arm settle before grabbing images
        return key

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
        print("Chessboard must stay fixed and fully in view. Vary position AND tilt a lot between poses.")
        while self.freedrive(f"[{len(base_tcp)}] whole chessboard in view") != "q":
            T_ct = self.cam.detect_board()
            if T_ct is None:
                print("chessboard not found, try again (whole board in view, no glare)")
                continue
            base_tcp.append(self.tcp())
            cam_tag.append(T_ct)
            print(f"pose {len(base_tcp)}: board at {np.linalg.norm(T_ct[:3, 3]) * 100:.0f} cm")
        tilt = max(np.degrees(np.linalg.norm(cv2.Rodrigues((inv(A) @ B)[:3, :3])[0]))
                   for A in base_tcp for B in base_tcp)
        print(f"{len(base_tcp)} poses, largest rotation between two poses {tilt:.0f} deg (want 12+ poses, 40+ deg)")
        self.X = hand_eye(base_tcp, cam_tag)
        config.DATA.mkdir(parents=True, exist_ok=True)
        np.save(config.DATA / "handeye.npy", self.X)
        # real-data check: the fixed board should land in the same base position from every pose
        pts = np.array([(B @ self.X @ C)[:3, 3] for B, C in zip(base_tcp, cam_tag)])
        spread = np.linalg.norm(pts - pts.mean(0), axis=1) * 1000
        print("per pose error (mm):", " ".join(f"{s:.0f}" for s in spread))  # one big number = one bad capture
        return spread

    # ---- tag-relative targets ----
    def teach(self, name, tag_id):
        self.freedrive("SEARCH pose: camera ~50-60 cm from the tag, tag in the middle of the image")
        q_search = list(self.r.getActualQ())
        self.locate(tag_id)  # check the tag is visible from here
        self.freedrive("CLOSE view: camera ~30 cm from the tag")
        T_bt = self.locate(tag_id)
        T_tag_view = inv(T_bt) @ self.tcp()
        self.freedrive("GRASP pose: gripper exactly around the crucible")
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
        print(f"   tag {t['tag']} found, {off * 100:.1f} cm from where it was taught")
        self.move("moveL", to_pose(T_bt @ np.array(t["tag_view"])), config.SPEED * 3, config.ACC, "close view")
        T_bt = self.locate(t["tag"])                                    # fine, same view as when taught
        return T_bt @ np.array(t["tag_grasp"])

    def _in_out(self, T, action):
        pre = shift_z(T, -config.APPROACH)
        self.move("moveL", to_pose(pre), config.SPEED * 3, config.ACC, "approach")
        self.move("moveL", to_pose(T), config.SPEED, config.ACC, "grasp pose")
        action()
        self.move("moveL", to_pose(pre), config.SPEED, config.ACC, "retract")

    def pick(self, name):
        self.gripper(close=False)
        self._in_out(self.find(name), lambda: self.gripper(close=True))

    def place(self, name):
        self._in_out(self.find(name), lambda: self.gripper(close=False))
