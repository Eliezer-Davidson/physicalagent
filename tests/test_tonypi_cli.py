"""Tests for the TonyPi CLI wiring.

The research finding: cmd_* hard-coded RospugAdapter, and RobotProfile.adapter_
factory had no call site. These pin the fix — a single build_adapter() seam that
dispatches through the resolved profile's adapter_factory, so `--robot tonypi`
builds a TonyPiAdapter (not a RospugAdapter); a link_for() that picks the link by
kind; and an explicit servo-tier selection defaulting to LOCKED. All offline.
"""
import pytest

from physicalagent.robots.rospug import RospugAdapter
from physicalagent.robots.tonypi import TonyPiAdapter
from physicalagent.robots.tonypi_link import FakeTonyLink
from physicalagent.robots.tonypi_servo import ServoTier
from physicalagent.robots.transport import FakeTransport


class TestBuildAdapterDispatchesThroughTheProfile:
    def test_tonypi_builds_a_tonypi_adapter(self):
        from physicalagent.cli import build_adapter, build_guard
        from physicalagent.profiles import get_profile
        p = get_profile("tonypi")
        a = build_adapter(p, FakeTonyLink(), build_guard(p))
        assert isinstance(a, TonyPiAdapter)

    def test_tonypi_does_not_build_a_rospug_adapter(self):
        """The exact regression the research named: a hard-coded RospugAdapter."""
        from physicalagent.cli import build_adapter, build_guard
        from physicalagent.profiles import get_profile
        p = get_profile("tonypi")
        a = build_adapter(p, FakeTonyLink(), build_guard(p))
        assert not isinstance(a, RospugAdapter)

    def test_rospug_still_builds_a_rospug_adapter(self):
        from physicalagent.cli import build_adapter, build_guard
        from physicalagent.profiles import get_profile
        p = get_profile("rospug")
        t = FakeTransport(telemetry={"batt": 11800, "dead": "ALIVE"})
        a = build_adapter(p, t, build_guard(p), frame_fetcher=lambda: b"x")
        assert isinstance(a, RospugAdapter)

    def test_tier_is_threaded_into_the_tonypi_adapter(self):
        from physicalagent.cli import build_adapter, build_guard
        from physicalagent.profiles import get_profile
        p = get_profile("tonypi")
        a = build_adapter(p, FakeTonyLink(), build_guard(p), tier=ServoTier.FULL)
        assert a._tier is ServoTier.FULL


class TestTierResolution:
    def test_default_is_locked_with_no_flag_and_no_env(self, monkeypatch):
        from physicalagent.cli import tonypi_tier_for
        monkeypatch.delenv("PHYSICALAGENT_TONYPI_TIER", raising=False)
        assert tonypi_tier_for(None) is ServoTier.LOCKED

    def test_env_var_selects_the_tier(self, monkeypatch):
        from physicalagent.cli import tonypi_tier_for
        monkeypatch.setenv("PHYSICALAGENT_TONYPI_TIER", "full")
        assert tonypi_tier_for(None) is ServoTier.FULL

    def test_flag_beats_the_env_var(self, monkeypatch):
        from physicalagent.cli import build_parser, tonypi_tier_for
        monkeypatch.setenv("PHYSICALAGENT_TONYPI_TIER", "full")
        args = build_parser().parse_args(["run", "x", "--tier", "normal"])
        assert tonypi_tier_for(args) is ServoTier.NORMAL

    def test_an_unknown_tier_name_is_rejected(self, monkeypatch):
        from physicalagent.cli import tonypi_tier_for
        monkeypatch.setenv("PHYSICALAGENT_TONYPI_TIER", "ludicrous")
        with pytest.raises(ValueError):
            tonypi_tier_for(None)


class TestParserAcceptsTier:
    def test_run_accepts_tier(self):
        from physicalagent.cli import build_parser
        args = build_parser().parse_args(["run", "x", "--tier", "full"])
        assert args.tier == "full"

    def test_tier_defaults_to_none_when_not_given(self):
        from physicalagent.cli import build_parser
        assert build_parser().parse_args(["run", "x"]).tier is None

    def test_shell_accepts_tier(self):
        from physicalagent.cli import build_parser
        assert build_parser().parse_args(["shell", "--tier", "normal"]).tier == "normal"


class TestLinkForPicksByKind:
    def test_tonypi_link_is_an_http_json_rpc_link(self):
        from physicalagent import config
        from physicalagent.cli import link_for
        from physicalagent.profiles import get_profile
        from physicalagent.robots.tonypi_link import HttpTonyLink
        link = link_for(get_profile("tonypi"))
        assert isinstance(link, HttpTonyLink)
        # points at the profile's host/port — no socket is opened by construction
        assert link._rpc_url == f"http://{config.TONYPI.host}:{config.TONYPI.rpc_port}"

    def test_rosbridge_link_goes_through_connect(self, monkeypatch):
        from physicalagent import cli
        from physicalagent.profiles import get_profile
        sentinel = object()
        monkeypatch.setattr(cli, "_connect", lambda: sentinel)
        assert cli.link_for(get_profile("rospug")) is sentinel
