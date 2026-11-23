"""Wrist camera, ArUco tag detection and the transform math."""
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


BOARD_OBJ = np.zeros((config.BOARD[0] * config.BOARD[1], 3), np.float32)
BOARD_OBJ[:, :2] = np.mgrid[0:config.BOARD[0], 0:config.BOARD[1]].T.reshape(-1, 2) * config.SQUARE


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
    def __init__(self, index=config.CAM):
        self.cap = cv2.VideoCapture(index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAM_RES[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAM_RES[1])
        f = config.DATA / "camera.npz"
        if f.exists():
            d = np.load(f)
            self.K, self.dist = d["K"], d["dist"]
            res = (int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
            if "res" in d and tuple(d["res"]) != res:
                raise RuntimeError(f"camera.npz was made at {tuple(d['res'])}, camera now runs at {res}: rerun intrinsics")

    def preview(self, msg):
        """Live view with detected tags outlined, until SPACE or q is pressed (window must have focus). Returns the key."""
        det = cv2.aruco.ArucoDetector(DICT)
        while True:
            ok, img = self.cap.read()
            if ok:
                corners, ids, _ = det.detectMarkers(img)
                cv2.aruco.drawDetectedMarkers(img, corners, ids)
                cv2.putText(img, msg, (20, 40), 0, 0.7, (0, 255, 0), 2)
                cv2.putText(img, "SPACE = continue, q = done", (20, 75), 0, 0.7, (0, 255, 0), 2)
                cv2.imshow("camera", img)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord(" "), ord("q")):
                return chr(k)

    def detect(self, tag_id, frames=10):
        """Average T_cam_tag over several frames, or None. The robot must stand still."""
        return self._average_pose(lambda img: tag_pose(img, self.K, self.dist, tag_id), frames)

    def detect_board(self, frames=5):
        """Average T_cam_board over several frames, or None. The robot must stand still."""
        return self._average_pose(lambda img: board_pose(img, self.K, self.dist), frames)

    def _average_pose(self, pose_fn, frames):
        if not hasattr(self, "K"):
            raise RuntimeError(f"no {config.DATA / 'camera.npz'} - run intrinsics first")
        for _ in range(5):
            self.cap.grab()  # drop stale buffered frames
        Ts = []
        for _ in range(frames * 3):
            ok, img = self.cap.read()
            T = pose_fn(img) if ok else None
            if T is not None:
                Ts.append(T)
            if len(Ts) == frames:
                break
        return average(Ts) if len(Ts) >= frames // 2 else None

    def calibrate(self):
        """Interactive chessboard calibration -> data/camera.npz."""
        objs, imgs = [], []
        print("SPACE = capture (~20, near/far/tilted/corners of the image), q = done")
        while True:
            ok, img = self.cap.read()
            if not ok:
                continue
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            corners = find_board(gray)
            found = corners is not None
            show = img.copy()
            if found:
                cv2.drawChessboardCorners(show, config.BOARD, corners, found)
            cv2.putText(show, f"{len(objs)} captured - board {'FOUND' if found else 'not found'}", (20, 40), 0, 1.0,
                        (0, 255, 0) if found else (0, 0, 255), 2)
            cv2.imshow("calibration", show)
            k = cv2.waitKey(1) & 0xFF
            if k == ord(" ") and found:
                objs.append(BOARD_OBJ)
                imgs.append(corners)
            if k == ord("q"):
                break
        cv2.destroyAllWindows()
        err, self.K, self.dist, _, _ = cv2.calibrateCamera(objs, imgs, gray.shape[::-1], None, None)
        config.DATA.mkdir(parents=True, exist_ok=True)
        np.savez(config.DATA / "camera.npz", K=self.K, dist=self.dist, res=gray.shape[::-1])
        print(f"{len(objs)} images at {gray.shape[::-1]}, reprojection error {err:.3f} px (want < 0.5)")
