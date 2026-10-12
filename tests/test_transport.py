"""Tests for physicalagent/robots/transport.py.

Transport is the seam between adapter logic (pure, testable) and roslibpy
(network, hardware). The adapter talks only to a Transport; production uses
RosbridgeTransport, tests use FakeTransport. This file tests the fake — the
contract the adapter will rely on — plus the injectable clock the closed-loop
controllers need.
"""
import pytest

from physicalagent.robots.transport import FakeTransport, ScriptedClock, Transport


class TestFakeTransportPublish:
    def test_records_published_messages_in_order(self):
        t = FakeTransport()
        t.publish("/pug_control/velocity_move", "pug_control/Velocity", {"x": 0.05})
        t.publish("/pug_control/velocity_move", "pug_control/Velocity", {"x": 0.0, "stop": True})
        assert t.published == [
            ("/pug_control/velocity_move", "pug_control/Velocity", {"x": 0.05}),
            ("/pug_control/velocity_move", "pug_control/Velocity", {"x": 0.0, "stop": True}),
        ]

    def test_last_published_helper(self):
        t = FakeTransport()
        t.publish("/a", "T", {"n": 1})
        t.publish("/a", "T", {"n": 2})
        t.publish("/b", "T", {"n": 3})
        assert t.last_published("/a") == {"n": 2}
        assert t.last_published("/b") == {"n": 3}
        assert t.last_published("/never") is None


class TestFakeTransportTelemetry:
    def test_latest_returns_seeded_value(self):
        t = FakeTransport(telemetry={"batt": 11800})
        assert t.latest("batt") == 11800

    def test_latest_missing_key_returns_none(self):
        t = FakeTransport()
        assert t.latest("batt") is None

    def test_set_telemetry_updates_value(self):
        t = FakeTransport()
        t.set_telemetry("dead", "ALIVE")
        assert t.latest("dead") == "ALIVE"


class TestFakeTransportServices:
    def test_call_service_returns_seeded_response(self):
        t = FakeTransport(services={"/pug_control/go_home": {"success": True}})
        assert t.call_service("/pug_control/go_home", "std_srvs/Empty", {}) == {"success": True}

    def test_call_service_records_the_call(self):
        t = FakeTransport(services={"/x": {"ok": 1}})
        t.call_service("/x", "pkg/Srv", {"arg": 5})
        assert t.service_calls == [("/x", "pkg/Srv", {"arg": 5})]

    def test_unknown_service_raises(self):
        t = FakeTransport()
        with pytest.raises(KeyError):
            t.call_service("/missing", "std_srvs/Empty", {})


class TestScriptedTelemetry:
    """The closed-loop turn controller reads odometry that CHANGES over time.
    A scripted value advances each time it is read, simulating motion."""

    def test_scripted_value_advances_on_each_read(self):
        t = FakeTransport()
        t.script_telemetry("odom_yaw", [0.0, 10.0, 25.0, 40.0])
        assert t.latest("odom_yaw") == 0.0
        assert t.latest("odom_yaw") == 10.0
        assert t.latest("odom_yaw") == 25.0
        assert t.latest("odom_yaw") == 40.0

    def test_scripted_value_holds_last_after_exhausted(self):
        t = FakeTransport()
        t.script_telemetry("odom_yaw", [0.0, 40.0])
        t.latest("odom_yaw"); t.latest("odom_yaw")
        assert t.latest("odom_yaw") == 40.0  # holds final value


class TestScriptedClock:
    def test_now_advances_by_sleep(self):
        c = ScriptedClock()
        assert c.now() == 0.0
        c.sleep(1.5)
        assert c.now() == 1.5
        c.sleep(0.5)
        assert c.now() == 2.0

    def test_records_total_slept(self):
        c = ScriptedClock()
        c.sleep(1.0); c.sleep(2.0)
        assert c.total_slept == 3.0


class TestTransportABC:
    def test_cannot_instantiate_abstract_transport(self):
        with pytest.raises(TypeError):
            Transport()
