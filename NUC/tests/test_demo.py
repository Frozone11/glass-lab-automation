"""Demo recipes: parsing and running with fake robot, MiR and ovens:  python -m pytest"""
import pytest

import config
import demo
from ur5.arm import save


@pytest.fixture(autouse=True)
def taught(tmp_path, monkeypatch):
    monkeypatch.setattr(config, "DATA", tmp_path)
    monkeypatch.setattr(config, "OVENS", {"oven1": {"host": "x", "key": "k"}})
    save("targets", {"oven1": {"tag": 1}, "spot": {"tag": 0}})
    save("poses", {"home": [0] * 6})


RECIPE = """
mir go to oven1      # drive there
pick oven1
mir go to table
place spot
door close oven1
wait 0.1
home
"""


def test_parse_ok():
    steps = demo.parse(RECIPE, missions=["go to oven1", "go to table"])
    assert [(c, a) for _, c, a in steps] == [("mir", "go to oven1"), ("pick", "oven1"), ("mir", "go to table"),
                                            ("place", "spot"), ("door", ("close", "oven1")), ("wait", 0.1),
                                            ("home", "")]
    assert steps[0][0] == 2                                       # line numbers count blank lines too


@pytest.mark.parametrize("text, error", [
    ("jump oven1", "line 1: unknown command 'jump'"),
    ("pick oven9", "no taught target 'oven9'"),
    ("mir", "needs a mission name"),
    ("mir go to mars", "no MiR mission called 'go to mars'"),
    ("door wiggle oven1", "door open <oven>"),
    ("door open oven9", "door open <oven>"),
    ("wait soon", "number of seconds"),
    ("wait 99999", "0-3600"),
    ("# only a comment", "empty"),
])
def test_parse_errors(text, error):
    with pytest.raises(ValueError, match=error):
        demo.parse(text, missions=["go to oven1"])


def test_missions_not_checked_when_mir_unreachable():
    assert demo.parse("mir anything at all", missions=[])[0][2] == "anything at all"


class Recorder:
    def __init__(self):
        self.calls = []

    def __getattr__(self, name):  # ur.pick(...), ur.go(...), mir.run_mission(...), oven.close(...)
        return lambda *a, **k: self.calls.append((name, a[0] if a and not callable(a[0]) else None))


class FakeJobs:
    stop_requested, step = False, None

    def say(self, line):
        pass


def test_run_in_order_arm_home_before_driving():
    rec = Recorder()
    rec.log = lambda line: None
    jobs = FakeJobs()
    demo.run(rec, demo.parse(RECIPE), jobs, rec, lambda name: rec)
    assert rec.calls == [("go", "home"), ("run_mission", "go to oven1"),
                         ("pick", "oven1"), ("go", "home"),
                         ("go", "home"), ("run_mission", "go to table"),
                         ("place", "spot"), ("go", "home"),
                         ("close", None),
                         ("go", "home")]
    assert jobs.step == 7                                         # all done


def test_stop_between_steps():
    rec = Recorder()
    rec.log = lambda line: None
    jobs = FakeJobs()
    jobs.stop_requested = True
    with pytest.raises(RuntimeError, match="stopped"):
        demo.run(rec, demo.parse("home\nhome"), jobs, rec, lambda name: rec)
    assert rec.calls == []
