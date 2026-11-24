"""Wrist camera, ArUco tag detection and the transform math."""
import threading
import time

import cv2
import numpy as np

import config

DICT = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, config.TAG_DICT))
# Tag corners in the tag frame, in the order the detector returns them; tag z points out towards the camera.
TAG_OBJ = config.TAG_SIZE / 2 * np.array([[-1, 1, 0], [1, 1, 0], [1, -1, 0], [-1, -1, 0]], np.float32)
inv = np.linalg.inv


def to_T(pose):
    """UR pose [x, y, z, rx, ry, rz] (axis-angle) -> 4x4 matrix."""
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(np.asarray(pose[3:], float))[0]
    T[:3, 3] = pose[:3]
    return T


def to_pose(T):
    return [float(v) for v in T[:3, 3]] + [float(v) for v in cv2.Rodrigues(T[:3, :3])[0].ravel()]


def hand_eye(base_tcp, cam_tag):
    """Solve AX = XB for X = T_tcp_cam (Park & Martin), using all pose pairs.
    Own version because cv2.calibrateHandEye is missing from some OpenCV 5 builds."""
    As, Bs = [], []
    for i in range(len(base_tcp)):
        for j in range(i + 1, len(base_tcp)):
            As.append(inv(base_tcp[j]) @ base_tcp[i])
            Bs.append(cam_tag[j] @ inv(cam_tag[i]))
    M = sum(np.outer(cv2.Rodrigues(B[:3, :3])[0], cv2.Rodrigues(A[:3, :3])[0]) for A, B in zip(As, Bs))
    w, V = np.linalg.eigh(M.T @ M)
    R = V @ np.diag(w ** -0.5) @ V.T @ M.T
    C = np.vstack([A[:3, :3] - np.eye(3) for A in As])
    d = np.hstack([R @ B[:3, 3] - A[:3, 3] for A, B in zip(As, Bs)])
    X = np.eye(4)
    X[:3, :3], X[:3, 3] = R, np.linalg.lstsq(C, d, rcond=None)[0]
    return X


def average(Ts):
    """Average of several 4x4 poses: median translation, rotation = mean matrix projected back onto a rotation.
    (Averaging rotation vectors breaks for a tag facing the camera: it sits at ~180 deg where they flip sign.)"""
    U, _, Vt = np.linalg.svd(sum(T[:3, :3] for T in Ts))
    T = np.eye(4)
    T[:3, :3] = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt
    T[:3, 3] = np.median([T[:3, 3] for T in Ts], axis=0)
    return T


def rot(axis, deg):
    return cv2.Rodrigues(np.asarray(axis, float) / np.linalg.norm(axis) * np.radians(deg))[0]


def orbit(cam0, centre, tilt=25, roll=30):
    """Camera poses (base frame) around a point the camera looks at: same distance, point stays mid-image.
    8 tilts of `tilt` deg in all directions, each with some roll, plus two pure rolls."""
    v = centre - cam0[:3, 3]
    v = v / np.linalg.norm(v)
    u1 = np.cross(v, [0, 0, 1]) if abs(v[2]) < 0.9 else np.cross(v, [1, 0, 0])
    u1 /= np.linalg.norm(u1)
    u2 = np.cross(v, u1)
    rotations = [rot(np.cos(a) * u1 + np.sin(a) * u2, tilt) @ rot(v, roll * (-1) ** k / 2)
                 for k, a in enumerate(np.radians(np.arange(0, 360, 45)))]
    rotations += [rot(v, roll), rot(v, -roll)]
    poses = []
    for R in rotations:
        about = np.eye(4)
        about[:3, :3], about[:3, 3] = R, centre - R @ centre   # rotate about the centre point
        poses.append(about @ cam0)
    return poses


BOARD_OBJ = np.zeros((config.BOARD[0] * config.BOARD[1], 3), np.float32)
BOARD_OBJ[:, :2] = np.mgrid[0:config.BOARD[0], 0:config.BOARD[1]].T.reshape(-1, 2) * config.SQUARE
BOARD_CENTRE = BOARD_OBJ[:, :2].mean(0)


def find_board(gray):
    """Chessboard corners (sub-pixel) or None. SB detector: robust to blur and uneven light."""
    found, corners = cv2.findChessboardCornersSB(gray, config.BOARD, flags=cv2.CALIB_CB_EXHAUSTIVE)
    return corners if found else None


def board_pose(img, K, dist):
    """T_cam_board from one image, or None. Far more accurate in angle than a small tag."""
    corners = find_board(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
    if corners is None:
        return None
    ok, r, t = cv2.solvePnP(BOARD_OBJ, corners, K, dist)
    return to_T(list(t.ravel()) + list(r.ravel())) if ok else None


def tag_pose(img, K, dist, tag_id):
    """T_cam_tag from one image, or None if the tag isn't seen."""
    corners, ids, _ = cv2.aruco.ArucoDetector(DICT).detectMarkers(img)
    if ids is None or tag_id not in ids.ravel():
        return None
    c = corners[list(ids.ravel()).index(tag_id)].reshape(4, 2)
    ok, r, t = cv2.solvePnP(TAG_OBJ, c, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    return to_T(list(t.ravel()) + list(r.ravel())) if ok else None


class Camera:
    """Owns the webcam: a background thread keeps the newest frame, so the web stream, tag detection and
    calibration can all use the camera at the same time."""

    def __init__(self, index=config.CAM):
        self.index, self.K, self.dist, self.calib_res = index, None, None, None
        self.frame, self.stamp = None, 0.0
        self.cond = threading.Condition()
        self._open()
        f = config.DATA / "camera.npz"
        if f.exists():
            d = np.load(f)
            self.K, self.dist = d["K"], d["dist"]
            self.calib_res = tuple(int(x) for x in d["res"]) if "res" in d else None
        threading.Thread(target=self._reader, daemon=True).start()

    def _open(self):
        self.cap = cv2.VideoCapture(self.index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAM_RES[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAM_RES[1])

    @property
    def calib_error(self):
        """None if the camera calibration is usable, else what is wrong."""
        if self.K is None:
            return f"no {config.DATA / 'camera.npz'} - run camera calibration first"
        img = self.latest()
        if img is not None and self.calib_res and img.shape[1::-1] != self.calib_res:
            return f"calibrated at {self.calib_res}, camera now gives {img.shape[1::-1]}: redo camera calibration"
        return None

    def _reader(self):
        fails = 0
        while True:
            ok, img = self.cap.read()
            if not ok:
                fails += 1
                if fails % 10 == 0:  # unplugged, or plugged in after boot: reopen every ~2 s until it works
                    self.cap.release()
                    self._open()
                time.sleep(0.2)
                continue
            fails = 0
            with self.cond:
                self.frame, self.stamp = img, time.time()
                self.cond.notify_all()

    def latest(self):
        """Newest frame (or None if the camera never delivered one)."""
        with self.cond:
            return None if self.frame is None else self.frame.copy()

    def fresh(self, timeout=2.0):
        """First frame captured after this call - use when the robot just stopped moving."""
        t0 = time.time()
        with self.cond:
            if not self.cond.wait_for(lambda: self.stamp > t0, timeout):
                return None
            return self.frame.copy()

    def detect(self, tag_id, frames=10):
        """Average T_cam_tag over several frames, or None. The robot must stand still."""
        return self._average_pose(lambda img: tag_pose(img, self.K, self.dist, tag_id), frames)

    def detect_board(self, frames=5):
        """Average T_cam_board over several frames, or None. The robot must stand still."""
        return self._average_pose(lambda img: board_pose(img, self.K, self.dist), frames)

    def _average_pose(self, pose_fn, frames):
        if err := self.calib_error:
            raise RuntimeError(err)
        self.fresh()  # skip a frame that may have been exposed while the arm was still settling
        Ts = []
        for _ in range(frames * 3):
            img = self.fresh()
            T = pose_fn(img) if img is not None else None
            if T is not None:
                Ts.append(T)
            if len(Ts) == frames:
                break
        return average(Ts) if len(Ts) >= frames // 2 else None

    def calibrate(self, wait, log=print):
        """Chessboard calibration -> camera.npz. wait(msg) returns 'next' (capture) or 'done'."""
        objs, imgs = [], []
        while wait(f"Hold the whole chessboard in view: near/far, tilted, in every corner of the image. "
                   f"{len(objs)} captured, aim for ~20.") == "next":
            img = self.fresh()
            corners = find_board(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)) if img is not None else None
            if corners is None:
                log("chessboard not found, try again (whole board in view, no glare)")
                continue
            objs.append(BOARD_OBJ)
            imgs.append(corners)
            log(f"capture {len(objs)} ok")
        if len(objs) < 5:
            raise RuntimeError(f"only {len(objs)} captures - need at least 5, nothing saved")
        size = img.shape[1::-1]
        err, self.K, self.dist, _, _ = cv2.calibrateCamera(objs, imgs, size, None, None)
        config.DATA.mkdir(parents=True, exist_ok=True)
        np.savez(config.DATA / "camera.npz", K=self.K, dist=self.dist, res=size)
        self.calib_res = tuple(size)
        log(f"{len(objs)} images at {size}, reprojection error {err:.3f} px (want < 0.5)")
