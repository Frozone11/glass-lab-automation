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
    def moveJ(self, q, *a): self.T = to_T(q); return True
    def moveL(self, p, *a): self.T = to_T(p); self.moves.append(self.T.copy()); return True
    def isProgramRunning(self): return True
    def stopScript(self): pass
    def setStandardDigitalOut(self, i, v): self.do[i] = v


class FakeCam:
    def __init__(self, robot, X, tag_in_base):
        self.robot, self.X, self.tag = robot, X, tag_in_base

    def detect(self, tag_id):
        return inv(self.robot.T @ self.X) @ self.tag


class FakeOven:
    """Stands in for oven.Oven: records door commands in a shared list."""
    calls = []

    def __init__(self, name):
        self.name = name

    def open(self, log=print):
        FakeOven.calls.append(("open", self.name))

    def close(self, log=print):
        FakeOven.calls.append(("close", self.name))


# taught poses for an oven target: search, close view, clear pose, 2 waypoints, approach (12 cm up, 3 mm off), grasp
SEARCH = to_T([0.45, 0.1, 0.6, np.pi, 0, 0])
VIEW = to_T([0.5, 0.1, 0.3, np.pi, 0, 0])
CLEAR = to_T([0.4, 0.0, 0.4, np.pi, 0, 0])
OUT = to_T([0.55, 0.0, 0.35, np.pi, 0, 0])
ENTRANCE = to_T([0.7, 0.15, 0.2, np.pi, 0, 0])
APPROACH = to_T([0.803, 0.15, 0.16, np.pi, 0, 0])
GRASP = to_T([0.8, 0.15, 0.04, np.pi, 0, 0])


def teach_user(rob, steps):
    """Fake user: for each prompt, jog to the next pose; None = press Done without moving.
    Prompt order in teach: search, close view, (oven: clear pose), waypoints..., Done, approach, grasp."""
    steps = iter(steps)

    def user(msg):
        T = next(steps)
        if T is None:
            return "done"
        rob.T = T
        return "next"
    return user


@pytest.fixture
def setup(tmp_path, monkeypatch):
    monkeypatch.setattr(config, "DATA", tmp_path)
    monkeypatch.setattr(config, "GRIP_TIME", 0)
    monkeypatch.setattr("ur5.arm.time.sleep", lambda s: None)
    rob = FakeRobot()
    X = to_T([0.0, 0.06, 0.03, 0.05, -0.03, 0.02])          # camera slightly off the tool axis
    monkeypatch.setattr("ur5.arm.Oven", FakeOven)
    FakeOven.calls = []
    cam = FakeCam(rob, X, yaw([0.5, 0.1, 0.0], 0))         # tag on the oven door
    ur = UR5(rob, rob, rob, cam, teach_user(rob, [SEARCH, VIEW, CLEAR, OUT, ENTRANCE, None, APPROACH, GRASP]),
             log=lambda line: None)
    ur.X = X
    ur.teach("oven1", 0, oven="oven1")
    assert FakeOven.calls == [("open", "oven1")]           # opened right after the close view, left open
    FakeOven.calls = []
    return ur, rob, cam


def test_pick_follows_moved_tag_through_waypoints(setup):
    ur, rob, cam = setup
    M = yaw([0.08, -0.12, 0], 6)                           # MiR came back 14 cm and 6 deg off
    cam.tag = M @ cam.tag
    rob.moves.clear()
    events = []
    rob.moveL = lambda p, *a: (events.append("move"), rob.moves.append(to_T(p)), setattr(rob, "T", to_T(p)), True)[3]
    FakeOven.calls = events                                 # door commands land in the same timeline
    ur.pick("oven1")
    # close view, clear pose, door opens, in + out, clear pose, door closes
    assert events == ["move", "move", ("open", "oven1")] + ["move"] * 7 + ["move", ("close", "oven1")]
    view, clear, out, entrance, pre, at, back, entrance2, out2, clear2 = rob.moves
    for got, taught in ((view, VIEW), (clear, CLEAR), (out, OUT), (entrance, ENTRANCE), (at, GRASP)):
        assert np.allclose(got, M @ taught, atol=1e-6)
    assert np.allclose(clear2, clear)
    assert np.allclose(pre[:3, 3] - at[:3, 3], [0, 0, 0.12])  # straight above, taught height (side offset dropped)
    assert np.allclose(pre[:3, :3], at[:3, :3])               # same orientation on the way down
    assert np.allclose(back, pre) and np.allclose(entrance2, entrance) and np.allclose(out2, out)
    assert rob.do == {config.GRIP_OPEN_DO: False, config.GRIP_CLOSE_DO: True}


def test_target_without_waypoints(tmp_path, monkeypatch):
    """A simple table spot: Done straight away at the waypoint step."""
    monkeypatch.setattr(config, "DATA", tmp_path)
    monkeypatch.setattr(config, "GRIP_TIME", 0)
    monkeypatch.setattr("ur5.arm.time.sleep", lambda s: None)
    rob = FakeRobot()
    X = to_T([0.0, 0.06, 0.03, 0, 0, 0])
    cam = FakeCam(rob, X, yaw([0.5, 0.1, 0.0], 0))
    ur = UR5(rob, rob, rob, cam, teach_user(rob, [SEARCH, VIEW, None, APPROACH, GRASP]), log=lambda line: None)
    ur.X = X
    ur.teach("spot", 0)
    rob.moves.clear()
    ur.place("spot")
    assert len(rob.moves) == 4                              # close view, approach, grasp, back up
    assert rob.do == {config.GRIP_OPEN_DO: True, config.GRIP_CLOSE_DO: False}


def test_approach_below_grasp_rejected(tmp_path, monkeypatch):
    monkeypatch.setattr(config, "DATA", tmp_path)
    monkeypatch.setattr("ur5.arm.time.sleep", lambda s: None)
    rob = FakeRobot()
    cam = FakeCam(rob, np.eye(4), yaw([0.5, 0.1, 0.0], 0))
    ur = UR5(rob, rob, rob, cam, teach_user(rob, [SEARCH, VIEW, None, GRASP, APPROACH]), log=lambda line: None)
    ur.X = np.eye(4)
    with pytest.raises(RuntimeError, match="above the grasp"):
        ur.teach("bad", 0)
    assert not (tmp_path / "targets.json").exists()


def test_refused_move_raises(setup):
    ur, rob, cam = setup
    rob.moveJ = lambda *a: False  # what ur_rtde does in protective stop etc.
    with pytest.raises(RuntimeError, match="refused"):
        ur.pick("oven1")


def test_refuses_far_offset(setup):
    ur, rob, cam = setup
    cam.tag = yaw([0.5, 0, 0], 0) @ cam.tag
    with pytest.raises(RuntimeError, match="from where it was taught"):
        ur.pick("oven1")


class FakeBoardCam:
    """Chessboard fixed on the MiR deck, seen by a 1280x720 camera with 0.3 px corner noise."""
    K = np.array([[900, 0, 640], [0, 900, 360], [0, 0, 1]], float)
    dist = np.zeros(5)

    def __init__(self, robot, X, board):
        self.robot, self.X, self.board = robot, X, board
        self.rng = np.random.default_rng(5)

    def board_corners(self):
        from ur5.vision import BOARD_OBJ
        C = inv(self.robot.T @ self.X) @ self.board
        if (C[:3, :3] @ BOARD_OBJ.T + C[:3, 3:4])[2].min() <= 0.05:
            return None
        px = cv2.projectPoints(BOARD_OBJ, cv2.Rodrigues(C[:3, :3])[0], C[:3, 3], self.K, self.dist)[0]
        if px[..., 0].min() < 0 or px[..., 0].max() > 1280 or px[..., 1].min() < 0 or px[..., 1].max() > 720:
            return None  # board not fully in the image
        return (px + self.rng.normal(0, 0.3, px.shape)).astype(np.float32)


def test_auto_handeye_recovers_camera_mount(tmp_path, monkeypatch):
    monkeypatch.setattr(config, "DATA", tmp_path)
    monkeypatch.setattr("ur5.arm.time.sleep", lambda s: None)
    rob = FakeRobot()
    rob.isPoseWithinSafetyLimits = lambda pose: True
    X = to_T([0.01, 0.07, 0.04, 0.08, -0.05, 0.6])               # true camera mount, unknown to the code
    board = to_T([0.35, -0.12, 0.0, 0, 0, 0.3])                    # board on the deck
    cam = FakeBoardCam(rob, X, board)
    view = to_T([0.45, -0.05, 0.35, np.pi, 0, 0]) @ inv(X)          # first-time view: camera looking down at it

    def user(msg):
        rob.T = view
        return "next"

    ur = UR5(rob, rob, rob, cam, user, log=lambda line: None)
    spread = ur.auto_handeye()
    assert len(spread) >= 20                                       # start + wiggles + most orbit poses seen
    assert np.linalg.norm(ur.X[:3, 3] - X[:3, 3]) < 0.001
    assert np.degrees(np.linalg.norm(cv2.Rodrigues(ur.X[:3, :3].T @ X[:3, :3])[0])) < 0.1
    assert np.allclose(rob.T, view)                                # back at board_view
    ur.wait = lambda msg: pytest.fail("second run must not ask the user")
    ur.auto_handeye()


def test_refinement_beats_pose_averaging():
    """Same noisy corners: the corner-based refinement must land closer to the true mount than hand_eye alone."""
    from ur5.vision import BOARD_OBJ, board_pose_from_corners, refine_handeye
    rng = np.random.default_rng(2)
    K, dist = FakeBoardCam.K, np.zeros(5)
    X = to_T([0.01, 0.07, 0.04, 0.08, -0.05, 0.6])
    board = to_T([0.35, -0.12, 0.0, 0, 0, 0.3])
    centre = board[:3, 3] + board[:3, :3] @ np.r_[BOARD_OBJ[:, :2].mean(0), 0]
    errs_plain, errs_refined = [], []
    for _ in range(5):
        Bs, corners = [], []
        for _ in range(15):
            # camera looking at the board centre from ~35 cm, random tilt/roll
            cam = to_T(list(centre + [0, 0, 0.35]) + [np.pi, 0, 0])
            T = np.eye(4)
            T[:3, :3] = cv2.Rodrigues(rng.normal(0, 0.3, 3))[0]
            T[:3, 3] = centre - T[:3, :3] @ centre
            cam = T @ cam
            C = inv(cam) @ board
            px = cv2.projectPoints(BOARD_OBJ, cv2.Rodrigues(C[:3, :3])[0], C[:3, 3], K, dist)[0]
            Bs.append(cam @ inv(X))
            corners.append((px + rng.normal(0, 0.3, px.shape)).astype(np.float32))
        X0 = hand_eye(Bs, [board_pose_from_corners(c, K, dist) for c in corners])
        X1, W, rms = refine_handeye(Bs, corners, X0, K, dist)
        errs_plain.append(np.linalg.norm(X0[:3, 3] - X[:3, 3]))
        errs_refined.append(np.linalg.norm(X1[:3, 3] - X[:3, 3]))
        assert np.linalg.norm(W[:3, 3] - board[:3, 3]) < 0.002 and rms < 0.5
    assert np.mean(errs_refined) < np.mean(errs_plain)
