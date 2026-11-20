"""Offline tests, no robot or camera needed:  python -m pytest  (from the repo root)."""
import cv2
import numpy as np
import pytest

import config
from ur5.arm import UR5
from ur5.vision import DICT, TAG_OBJ, average, board_pose, hand_eye, inv, tag_pose, to_pose, to_T

rng = np.random.default_rng(0)


def rand_T(scale=0.5):
    return to_T(list(rng.uniform(-scale, scale, 3)) + list(rng.uniform(-1.5, 1.5, 3)))


def yaw(T_xyz, deg):
    return to_T(list(T_xyz) + [0, 0, np.radians(deg)])


# ---------- vision ----------
def test_pose_roundtrip():
    T = rand_T()
    assert np.allclose(to_T(to_pose(T)), T)


def test_average_near_180_deg():
    """A tag facing the camera is ~180 deg rotated, where rotation vectors flip sign between frames."""
    T0 = to_T([0, 0.01, 0.4, np.pi, 0, 0])
    for _ in range(50):
        Ts = [T0 @ to_T(list(rng.normal(0, 0.0005, 3)) + list(rng.normal(0, 0.01, 3))) for _ in range(10)]
        err = np.degrees(np.linalg.norm(cv2.Rodrigues(average(Ts)[:3, :3].T @ T0[:3, :3])[0]))
        assert err < 1.0
        assert np.linalg.norm(average(Ts)[:3, 3] - T0[:3, 3]) < 0.001


def test_tag_pose_from_rendered_image():
    """Render a tag at a known pose and check solvePnP gets it back (catches corner-order mistakes)."""
    K = np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]], float)
    # tag z points out of the tag towards the camera, so a facing tag is ~180 deg about x
    T_ct = to_T([0.02, -0.01, 0.35, np.pi, 0, 0]) @ to_T([0, 0, 0, 0.3, -0.2, 0.1])
    side = 80
    tag = cv2.aruco.generateImageMarker(DICT, 3, side)
    src = np.float32([[0, 0], [side, 0], [side, side], [0, side]])
    dst, _ = cv2.projectPoints(TAG_OBJ, cv2.Rodrigues(T_ct[:3, :3])[0], T_ct[:3, 3], K, None)
    H = cv2.getPerspectiveTransform(src, dst.reshape(4, 2).astype(np.float32))
    img = cv2.warpPerspective(tag, H, (640, 480), dst=np.full((480, 640), 255, np.uint8),
                              borderMode=cv2.BORDER_TRANSPARENT)
    got = tag_pose(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR), K, np.zeros(5), 3)
    assert got is not None
    assert np.linalg.norm(got[:3, 3] - T_ct[:3, 3]) < 0.003
    assert np.allclose(got[:3, :3], T_ct[:3, :3], atol=0.03)


def test_board_pose_same_frame_from_any_roll():
    """Render the chessboard at known poses with the camera rolled 0-270 deg: board_pose must always report the
    same board frame (hand-eye breaks if the corner order flips with camera roll)."""
    K = np.array([[900, 0, 640], [0, 900, 360], [0, 0, 1]], float)
    sq, S = 40, config.SQUARE  # px per square in the flat board image, metres per square
    nx, ny = config.BOARD[0] + 1, config.BOARD[1] + 1
    flat = np.full((ny * sq + 2 * sq, nx * sq + 2 * sq), 255, np.uint8)
    for i in range(nx):
        for j in range(ny):
            if (i + j) % 2 == 0:
                flat[sq + j * sq:sq + (j + 1) * sq, sq + i * sq:sq + (i + 1) * sq] = 0
    src = np.float32([[2 * sq, 2 * sq], [6 * sq, 2 * sq], [2 * sq, 6 * sq], [6 * sq, 6 * sq]])
    pts = np.float32([[0, 0, 0], [4, 0, 0], [0, 4, 0], [4, 4, 0]]) * S
    # OpenCV starts at the opposite corner of this rendering: fixed offset of 8 x 5 squares, turned 180 deg
    D = to_T([8 * S, 5 * S, 0, 0, 0, np.pi])
    for roll in (0, 90, 180, 270):
        T = (to_T([0, 0, 0.5, 0, 0, np.radians(roll)]) @ to_T([0, 0, 0, 0.2, -0.15, 0])
             @ to_T([-4 * S, -2.5 * S, 0, 0, 0, 0]))
        dst, _ = cv2.projectPoints(pts, cv2.Rodrigues(T[:3, :3])[0], T[:3, 3], K, None)
        H = cv2.getPerspectiveTransform(src, dst.reshape(4, 2).astype(np.float32))
        img = cv2.warpPerspective(flat, H, (1280, 720), borderValue=255)
        got = board_pose(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR), K, np.zeros(5))
        assert got is not None
        expected = T @ D
        assert np.linalg.norm(got[:3, 3] - expected[:3, 3]) < 0.002, roll
        assert np.degrees(np.linalg.norm(cv2.Rodrigues(got[:3, :3].T @ expected[:3, :3])[0])) < 0.3, roll


def test_hand_eye():
    X, T_base_tag = rand_T(0.1), rand_T()
    Bs = [rand_T() for _ in range(15)]
    Cs = [inv(X) @ inv(B) @ T_base_tag for B in Bs]
    assert np.allclose(hand_eye(Bs, Cs), X, atol=1e-6)
    noisy = [C @ to_T(list(rng.normal(0, 0.001, 3)) + list(rng.normal(0, 0.005, 3))) for C in Cs]
    assert np.linalg.norm(hand_eye(Bs, noisy)[:3, 3] - X[:3, 3]) < 0.005


# ---------- arm, with a fake robot + camera ----------
class FakeRobot:
    """Stands in for the ur_rtde control/receive/io interfaces. 'Joints' are just the TCP pose."""
    def __init__(self):
        self.T = to_T([0.3, 0, 0.4, np.pi, 0, 0])
        self.moves, self.do = [], {}

    def getActualTCPPose(self): return to_pose(self.T)
    def getActualQ(self): return to_pose(self.T)
    def moveJ(self, q, *a): self.T = to_T(q)
    def moveL(self, p, *a): self.T = to_T(p); self.moves.append(self.T.copy())
    def teachMode(self): pass
    def endTeachMode(self): pass
    def setStandardDigitalOut(self, i, v): self.do[i] = v


class FakeCam:
    def __init__(self, robot, X, tag_in_base):
        self.robot, self.X, self.tag = robot, X, tag_in_base

    def detect(self, tag_id):
        return inv(self.robot.T @ self.X) @ self.tag


@pytest.fixture
def setup(tmp_path, monkeypatch):
    monkeypatch.setattr(config, "DATA", tmp_path)
    monkeypatch.setattr(config, "GRIP_TIME", 0)
    rob = FakeRobot()
    X = to_T([0.0, 0.06, 0.03, 0.05, -0.03, 0.02])          # camera slightly off the tool axis
    cam = FakeCam(rob, X, yaw([0.5, 0.1, 0.0], 0))         # tag flat on the table
    ur = UR5(rob, rob, rob, cam)
    ur.X = X
    search = to_T([0.45, 0.1, 0.6, np.pi, 0, 0])
    view = to_T([0.5, 0.1, 0.3, np.pi, 0, 0])
    grasp = to_T([0.5, 0.15, 0.04, np.pi, 0, 0])
    poses = iter([search, view, grasp])
    ur.freedrive = lambda msg: setattr(rob, "T", next(poses))
    ur.teach("spot", 0)
    return ur, rob, cam, grasp


def test_pick_follows_moved_tag(setup):
    ur, rob, cam, grasp = setup
    M = yaw([0.08, -0.12, 0], 6)                           # MiR came back 14 cm and 6 deg off
    cam.tag = M @ cam.tag
    ur.pick("spot")
    expected = M @ grasp
    pre, at, back = rob.moves[-3:]
    assert np.linalg.norm(at[:3, 3] - expected[:3, 3]) < 1e-6
    assert np.allclose(at[:3, :3], expected[:3, :3], atol=1e-6)
    assert np.isclose(np.linalg.norm(pre[:3, 3] - at[:3, 3]), config.APPROACH)
    assert np.allclose(back, pre)
    assert rob.do == {config.GRIP_OPEN_DO: False, config.GRIP_CLOSE_DO: True}


def test_refuses_far_offset(setup):
    ur, rob, cam, _ = setup
    cam.tag = yaw([0.5, 0, 0], 0) @ cam.tag
    with pytest.raises(RuntimeError, match="from where it was taught"):
        ur.pick("spot")
