"""The console owns the SIM bring-up (start handshake, band release, latch
drain); the session owns the battery machinery. --script joins them so the
SAME battery file that ran on hardware runs against MuJoCo off-robot -- through
the console's bring-up, with NO model and NO API key in the loop, because a
battery is deterministic by contract and home may be offline.
"""
from __future__ import annotations

import importlib.util
import json
import pathlib
import sys

import pytest

REPO = pathlib.Path(__file__).resolve().parents[1]


def _load(name):
    spec = importlib.util.spec_from_file_location(
        name, REPO / "scripts" / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def console():
    return _load("sonic_console")


class _Spy:
    def __init__(self, fail_on=None):
        self.calls = []
        self.fail_on = fail_on

    def execute(self, name, params):
        self.calls.append((name, params))
        if name == self.fail_on:
            raise RuntimeError(f"{name} exploded")
        return "ok"

    def stop_all(self):
        self.calls.append(("stop_all", {}))
        return "ok"


BODY = ("mark battery-start\n"
        "arms arms_up\n"
        "walk 0.1 2\n"
        "mark battery-end\n")


class TestTheConsoleRunsABatteryWithoutAModel:
    def test_verbs_flow_through_the_adapter_and_the_log_lands(
            self, console, tmp_path):
        battery = tmp_path / "b.txt"
        battery.write_text(BODY)
        log = tmp_path / "log.jsonl"
        rc = console._run_battery(_Spy(), None, str(battery), str(log),
                                  sleep_fn=lambda s: None)
        assert rc == 0
        events = [json.loads(l) for l in log.read_text().splitlines()]
        assert events[0]["kind"] == "meta"
        assert events[0]["real"] is False
        assert [e["verb"] for e in events if e["kind"] == "verb"] == \
            ["arms", "walk"]

    def test_an_aborted_battery_exits_nonzero(self, console, tmp_path):
        battery = tmp_path / "b.txt"
        battery.write_text(BODY)
        rc = console._run_battery(_Spy(fail_on="set_arms"), None,
                                  str(battery), str(tmp_path / "log.jsonl"),
                                  sleep_fn=lambda s: None)
        assert rc == 4

    def test_an_invalid_battery_refuses_before_anything_moves(
            self, console, tmp_path):
        battery = tmp_path / "b.txt"
        battery.write_text("walk\n")          # bare motion verb: invalid
        spy = _Spy()
        rc = console._run_battery(spy, None, str(battery),
                                  str(tmp_path / "log.jsonl"),
                                  sleep_fn=lambda s: None)
        assert rc != 0
        assert spy.calls == []
