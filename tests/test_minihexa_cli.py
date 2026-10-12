"""Tests for the MiniHexa CLI wiring — offline, no serial, no BLE.

These pin the same seams the TonyPi CLI tests pin, for the third robot: a single
build_adapter() that dispatches through the profile's adapter_factory so
`--robot minihexa` builds a MiniHexaAdapter; a link_for() that picks the transport
by kind (serial by default, BLE on demand) WITHOUT opening any I/O; an explicit
motion-tier selection defaulting to LOCKED; and a --link flag to choose the
transport. All offline — no link ever opens a port or a radio.
"""
import pytest

from physicalagent.robots.minihexa import MiniHexaAdapter
from physicalagent.robots.minihexa_link import (
    BleMiniHexaLink, FakeMiniHexaLink, SerialMiniHexaLink,
)
from physicalagent.robots.minihexa_motion import MotionTier
from physicalagent.robots.tonypi import TonyPiAdapter


class TestBuildAdapterDispatchesThroughTheProfile:
    def test_minihexa_builds_a_minihexa_adapter(self):
        from physicalagent.cli import build_adapter, build_guard
        from physicalagent.profiles import get_profile
        p = get_profile("minihexa")
        a = build_adapter(p, FakeMiniHexaLink(), build_guard(p))
        assert isinstance(a, MiniHexaAdapter)

    def test_minihexa_does_not_build_a_tonypi_adapter(self):
        from physicalagent.cli import build_adapter, build_guard
        from physicalagent.profiles import get_profile
        p = get_profile("minihexa")
        a = build_adapter(p, FakeMiniHexaLink(), build_guard(p))
        assert not isinstance(a, TonyPiAdapter)

    def test_tier_is_threaded_into_the_minihexa_adapter(self):
        from physicalagent.cli import build_adapter, build_guard
        from physicalagent.profiles import get_profile
        p = get_profile("minihexa")
        a = build_adapter(p, FakeMiniHexaLink(), build_guard(p), tier=MotionTier.FULL)
        assert a._tier is MotionTier.FULL


class TestTierResolution:
    def test_default_is_locked_with_no_flag_and_no_env(self, monkeypatch):
        from physicalagent.cli import minihexa_tier_for
        monkeypatch.delenv("PHYSICALAGENT_MINIHEXA_TIER", raising=False)
        assert minihexa_tier_for(None) is MotionTier.LOCKED

    def test_env_var_selects_the_tier(self, monkeypatch):
        from physicalagent.cli import minihexa_tier_for
        monkeypatch.setenv("PHYSICALAGENT_MINIHEXA_TIER", "full")
        assert minihexa_tier_for(None) is MotionTier.FULL

    def test_flag_beats_the_env_var(self, monkeypatch):
        from physicalagent.cli import build_parser, minihexa_tier_for
        monkeypatch.setenv("PHYSICALAGENT_MINIHEXA_TIER", "full")
        args = build_parser().parse_args(["run", "x", "--tier", "normal"])
        assert minihexa_tier_for(args) is MotionTier.NORMAL

    def test_an_unknown_tier_name_is_rejected(self, monkeypatch):
        from physicalagent.cli import minihexa_tier_for
        monkeypatch.setenv("PHYSICALAGENT_MINIHEXA_TIER", "ludicrous")
        with pytest.raises(ValueError):
            minihexa_tier_for(None)


class TestParserAcceptsLink:
    def test_run_accepts_link(self):
        from physicalagent.cli import build_parser
        args = build_parser().parse_args(["run", "x", "--link", "ble"])
        assert args.link == "ble"

    def test_link_defaults_to_serial(self):
        from physicalagent.cli import build_parser
        assert build_parser().parse_args(["run", "x"]).link == "serial"

    def test_shell_accepts_link(self):
        from physicalagent.cli import build_parser
        assert build_parser().parse_args(["shell", "--link", "ble"]).link == "ble"

    def test_an_invalid_link_is_rejected_by_argparse(self):
        from physicalagent.cli import build_parser
        with pytest.raises(SystemExit):
            build_parser().parse_args(["run", "x", "--link", "usb3"])


class TestMiniHexaLinkKind:
    def test_default_is_serial(self, monkeypatch):
        from physicalagent.cli import minihexa_link_kind_for
        monkeypatch.delenv("PHYSICALAGENT_MINIHEXA_LINK", raising=False)
        assert minihexa_link_kind_for(None) == "serial"

    def test_flag_selects_ble(self):
        from physicalagent.cli import build_parser, minihexa_link_kind_for
        args = build_parser().parse_args(["run", "x", "--link", "ble"])
        assert minihexa_link_kind_for(args) == "ble"


class TestLinkForPicksByKind:
    def test_minihexa_link_is_a_serial_link_by_default(self):
        from physicalagent import config
        from physicalagent.cli import link_for
        from physicalagent.profiles import get_profile
        link = link_for(get_profile("minihexa"))
        assert isinstance(link, SerialMiniHexaLink)
        # points at the profile's device — no port is opened by construction
        assert link._device == config.MINIHEXA.device
        assert link._serial is None   # lazy: nothing opened

    def test_link_kind_override_builds_a_ble_link(self):
        from physicalagent import config
        from physicalagent.cli import link_for
        from physicalagent.profiles import get_profile
        link = link_for(get_profile("minihexa"), kind="ble")
        assert isinstance(link, BleMiniHexaLink)
        assert link._address == config.MINIHEXA.ble_address
        assert link._client is None   # lazy: nothing connected

    def test_an_unknown_kind_raises(self):
        from physicalagent.cli import link_for
        from physicalagent.profiles import get_profile
        p = get_profile("minihexa")
        with pytest.raises(ValueError):
            link_for(p, kind="carrier-pigeon")


class TestExistingRobotsStillWork:
    def test_tonypi_still_builds_a_tonypi_adapter(self):
        from physicalagent.cli import build_adapter, build_guard
        from physicalagent.profiles import get_profile
        from physicalagent.robots.tonypi_link import FakeTonyLink
        p = get_profile("tonypi")
        a = build_adapter(p, FakeTonyLink(), build_guard(p))
        assert isinstance(a, TonyPiAdapter)
