"""The adapter half of fall management: the watchdog that damps, the latch
that marks every result, and the allowlist that gates set_mode.

Offline like every other test here. The watchdog THREAD is not started in
these tests — _fall_poll_once() is driven synchronously, which is the whole
reason the poll logic is a method and the thread is three lines.
"""
from __future__ import annotations

import time

import pytest

from physicalagent.robots import sonic_fall, sonic_modes
from physicalagent.robots.sonic_fall import FallDetector, HARDWARE_DAY_ONE_ALLOWLIST
from physicalagent.robots.sonic_g1 import SonicG1Adapter
from physicalagent.robots.sonic_link import FakeSonicLink, ScriptedClock


class StubGuard:
    def __init__(self, estopped=False):
        self.is_estopped = estopped


class ScriptedPoses:
    """Telemetry stand-in: pose() pops from a script, holding the last entry.

    lowstate() answers healthy by default. The watchdog reads it every beat, so
    a double that omits it is not a smaller double — it is a different one, and
    it hid a crash in the beat for a whole commit.
    """

    def __init__(self, poses, lowstate=None):
        self._poses = list(poses)
        self._lowstate = (lowstate if lowstate is not None
                          else {"active": True, "age_s": 0.0,
                                "source": "ScriptedPoses"})

    def pose(self):
        p = self._poses.pop(0) if len(self._poses) > 1 else self._poses[0]
        return p

    def lowstate(self):
        return self._lowstate


class DampCountingLink(FakeSonicLink):
    def __init__(self):
        super().__init__()
        self.damp_calls = 0

    def emergency_stop(self):
        self.damp_calls += 1
        return {"emergency_stop": True}


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


STANDING = {"available": True, "height_m": 0.786, "tilt_deg": 1.1}
FLAT = {"available": True, "height_m": 0.10, "tilt_deg": 85.0}


def build(poses, **fall_kw):
    link = DampCountingLink()
    a = SonicG1Adapter(link, StubGuard(), clock=ScriptedClock(),
                       telemetry=ScriptedPoses(poses))
    clock = FakeClock()
    clock.t = 100.0
    a._fall = FallDetector(clock=clock, grace_s=0.0, **fall_kw)
    return link, a, clock


class TestWatchdogDamps:
    def test_confirmed_fall_damps_exactly_once_and_latches(self):
        link, a, clock = build([STANDING] * 3 + [FLAT], confirm_samples=5)
        for _ in range(3):
            assert a._fall_poll_once()["fallen"] is False
        for _ in range(5):
            clock.t += 0.1
            v = a._fall_poll_once()
        assert v["fallen"] is True
        assert link.damp_calls == 1
        # Further polls must not re-damp: one fall, one damp, one alarm.
        for _ in range(10):
            a._fall_poll_once()
        assert link.damp_calls == 1

    def test_reset_fall_latch_rearms(self):
        link, a, clock = build([FLAT], confirm_samples=2)
        for _ in range(2):
            clock.t += 0.1
            a._fall_poll_once()
        assert link.damp_calls == 1
        a.reset_fall_latch()
        assert a._fall.fallen is False
        for _ in range(2):
            clock.t += 0.1
            a._fall_poll_once()
        assert link.damp_calls == 2   # a second real fall damps again

    def test_unavailable_telemetry_never_confirms(self):
        link, a, clock = build([{"available": False}], confirm_samples=2)
        for _ in range(20):
            clock.t += 0.1
            a._fall_poll_once()
        assert link.damp_calls == 0

    def test_watchdog_refuses_without_telemetry(self):
        a = SonicG1Adapter(FakeSonicLink(), StubGuard(), clock=ScriptedClock())
        assert a.start_fall_watchdog() is False


class TestModeFollowsCommands:
    def test_set_mode_informs_the_detector(self):
        _, a, _ = build([STANDING])
        a._set_mode("crawl")
        assert a._fall._mode == "crawl"

    def test_stop_all_keeps_the_detector_on_a_held_ground_mode(self):
        # Superseded expectation: stop_all used to note idle unconditionally,
        # but the mode-aware halt HOLDS a ground mode at speed 0 — so the
        # detector must keep judging by the ground mode's envelope, not
        # idle's (which would call the held crawl a fall in 0.5 s).
        _, a, _ = build([STANDING])
        a._set_mode("crawl")
        a.stop_all()
        assert a._fall._mode == "crawl"

    def test_stop_all_notes_idle_when_the_robot_was_walking(self):
        _, a, _ = build([STANDING])
        a._set_mode("slow_walk")
        a.stop_all()
        assert a._fall._mode == "idle"


class TestAllowlist:
    def test_unset_env_allows_every_mode(self):
        _, a, _ = build([STANDING])
        for m in sonic_modes.MODES:
            a._set_mode(m.name)   # must not raise

    def test_allowlist_blocks_and_names_the_allowed_set(self, monkeypatch):
        monkeypatch.setenv("SONIC_MODE_ALLOWLIST", "idle, slow_walk")
        a = SonicG1Adapter(FakeSonicLink(), StubGuard(), clock=ScriptedClock())
        a._set_mode("slow_walk")
        with pytest.raises(ValueError) as e:
            a._set_mode("run")
        assert "allowlist" in str(e.value)
        assert "slow_walk" in str(e.value)

    def test_allowlist_resolves_aliases_at_parse_time(self, monkeypatch):
        # "sneak" is an alias for stealth_walk; the allowlist stores real names
        # so the gate cannot be dodged by synonym.
        monkeypatch.setenv("SONIC_MODE_ALLOWLIST", "sneak")
        a = SonicG1Adapter(FakeSonicLink(), StubGuard(), clock=ScriptedClock())
        a._set_mode("stealth_walk")
        with pytest.raises(ValueError):
            a._set_mode("walk")

    def test_day_one_hardware_list_is_conservative(self):
        for name in HARDWARE_DAY_ONE_ALLOWLIST:
            mode = sonic_modes.resolve(name)
            assert mode.kind != sonic_modes.GROUND
        banned = {"run", "jump_forward", "crawl", "elbow_crawl", "lie_face_down"}
        assert banned.isdisjoint(HARDWARE_DAY_ONE_ALLOWLIST)


class TestLatchReachesTheModel:
    def test_posture_note_carries_the_fall_latch(self):
        link, a, clock = build([FLAT], confirm_samples=1)
        clock.t += 0.1
        a._fall_poll_once()
        assert a._fall.fallen
        body = a._posture()
        assert body is not None
        assert "FALL DETECTED" in body["note"]
        # There IS a get-up now, so the note points at it instead of declaring
        # the robot stranded — but it must not promise that it works, because
        # the route's reliability is measured per attempt, not assumed.
        assert "recover_from_fall" in body["note"]
        assert "operator to reset the robot by hand" in body["note"]

    def test_latch_reported_even_when_telemetry_goes_stale(self):
        # A fallen robot with a frozen stream must still be reported fallen —
        # the latch outranks the pose read.
        link, a, clock = build([FLAT], confirm_samples=1)
        clock.t += 0.1
        a._fall_poll_once()
        a._telemetry = ScriptedPoses([{"available": True, "fresh": False,
                                       "height_m": 0.786, "tilt_deg": 1.0}])
        body = a._posture()
        assert body is not None and "FALL DETECTED" in body["note"]


STALE = {"available": True, "fresh": False, "height_m": 0.10, "tilt_deg": 85.0}


class TestAdversarialReviewFixes:
    """Each of these pins a defect the 2026-08-01 adversarial review found."""

    def test_stale_telemetry_never_accrues_strikes(self):
        # A frozen stream replayed at 10 Hz must not manufacture "5
        # consecutive" bad readings out of one — nor hold "standing" forever.
        link, a, clock = build([STALE], confirm_samples=2)
        for _ in range(20):
            clock.t += 0.1
            a._fall_poll_once()
        assert link.damp_calls == 0
        assert not a._fall.fallen

    def test_walk_informs_the_detector(self):
        # walk() switches the wire to a walking mode; the detector must not
        # keep judging by a stale ground-mode envelope (the lie_face_down ->
        # walk() desync made real falls permanently invisible).
        _, a, _ = build([STANDING])
        a._set_mode("lie_face_down")
        a._move(0.3, 0.0, 0.0, 0.1)
        assert a._fall._mode == "slow_walk"

    def test_walk_respects_the_allowlist(self, monkeypatch):
        monkeypatch.setenv("SONIC_MODE_ALLOWLIST", "idle,squat")
        a = SonicG1Adapter(FakeSonicLink(), StubGuard(), clock=ScriptedClock())
        with pytest.raises(ValueError):
            a._move(0.3, 0.0, 0.0, 0.1)

    def test_set_pose_respects_the_allowlist(self, monkeypatch):
        monkeypatch.setenv("SONIC_MODE_ALLOWLIST", "idle,slow_walk")
        a = SonicG1Adapter(FakeSonicLink(), StubGuard(), clock=ScriptedClock())
        with pytest.raises(ValueError):
            a._set_pose("sit")     # sit -> squat, not on the list

    def test_damp_is_never_allowlist_gated_and_not_a_fall(self, monkeypatch):
        monkeypatch.setenv("SONIC_MODE_ALLOWLIST", "idle")
        link = DampCountingLink()
        a = SonicG1Adapter(link, StubGuard(), clock=ScriptedClock(),
                           telemetry=ScriptedPoses([FLAT]))
        clock = FakeClock()
        a._fall = sonic_fall.FallDetector(clock=clock, grace_s=0.0,
                                          confirm_samples=2)
        a._set_pose("damp")        # must not raise despite the lockdown list
        for _ in range(10):        # robot sinks to the floor BY DESIGN
            clock.t += 0.1
            a._fall_poll_once()
        assert not a._fall.fallen  # a commanded damp is not a fall

    def test_stop_fall_watchdog_stops_the_thread(self):
        _, a, _ = build([STANDING])
        assert a.start_fall_watchdog() is True
        a.stop_fall_watchdog()
        assert not a._watchdog_thread.is_alive()

    def test_posture_uses_the_mode_envelope_not_a_flat_standing_bound(self):
        # stealth_walk_2 walks perfectly at pelvis 0.533 — under the 0.55
        # standing threshold. The note must NOT tell the model the robot is
        # on the ground (§2 recurrence found by review).
        link, a, _ = build([{"available": True, "fresh": True,
                             "height_m": 0.533, "tilt_deg": 12.0}])
        link.select_mode("stealth_walk_2")
        body = a._posture()
        assert body is not None
        assert "NOT STANDING" not in body["note"]
        assert "stealth_walk_2" in body["note"]

    def test_stale_posture_is_not_presented_as_measured(self):
        _, a, _ = build([STALE])
        assert a._posture() is None

    def test_unknown_mode_names_do_not_move_the_detector(self):
        _, a, _ = build([STANDING])
        a._set_mode("crawl")
        a._fall.note_mode("definitely_not_a_mode")
        assert a._fall._mode == "crawl"


class RecordingLink(FakeSonicLink):
    def __init__(self):
        super().__init__()
        self.calls = []

    def select_mode(self, name, **kw):
        self.calls.append(("select_mode", name, kw.get("speed")))
        return super().select_mode(name, **kw)

    def stop(self):
        self.calls.append(("stop", None, None))
        return super().stop()


def build_rec(**env):
    link = RecordingLink()
    a = SonicG1Adapter(link, StubGuard(), clock=ScriptedClock())
    return link, a


class TestModeAwareTaskEndHalt:
    """er.py calls stop_all at the end of EVERY task; idle@height to a robot
    on its hands and knees is a stand-up command it cannot execute (measured
    2026-08-01: it collapses). stop_all must be mode-aware."""

    def test_stop_all_during_a_ground_mode_holds_it_at_speed_zero(self):
        link, a = build_rec()
        a._set_mode("crawl")
        link.calls.clear()
        a.stop_all()
        assert link.calls == [("select_mode", "crawl", 0.0)]
        assert a._fall._mode == "crawl"      # detector follows, no false grace

    def test_stop_all_during_a_static_mode_leaves_the_planner_alone(self):
        link, a = build_rec()
        a._set_mode("kneel_two_legs")
        link.calls.clear()
        a.stop_all()
        assert link.calls == []
        assert a._fall._mode == "kneel_two_legs"

    def test_stop_all_while_standing_still_idles_as_before(self):
        link, a = build_rec()
        a._set_mode("slow_walk")
        link.calls.clear()
        a.stop_all()
        assert ("stop", None, None) in link.calls


class TestStagedCrawlChain:
    """The vendor's beginCrawlingTransition ladder, owned by the adapter —
    Gemini's latency and hop cap broke every manually-paced attempt."""

    def test_entering_elbow_crawl_from_standing_stages_the_full_ladder(self):
        link, a = build_rec()
        a._set_mode("elbow_crawl")
        names = [c[1] for c in link.calls if c[0] == "select_mode"]
        assert names == ["kneel_two_legs", "crawl", "elbow_crawl"]

    def test_exiting_to_idle_reverses_the_ladder(self):
        link, a = build_rec()
        a._set_mode("elbow_crawl")
        link.calls.clear()
        a._set_mode("idle")
        names = [c[1] for c in link.calls if c[0] == "select_mode"]
        assert names == ["crawl", "kneel_two_legs", "idle"]

    def test_adjacent_transition_is_direct(self):
        link, a = build_rec()
        a._set_mode("crawl")
        link.calls.clear()
        a._set_mode("elbow_crawl")
        names = [c[1] for c in link.calls if c[0] == "select_mode"]
        assert names == ["elbow_crawl"]

    def test_result_reports_the_staging_with_outcomes(self):
        _, a = build_rec()
        out = a._set_mode("elbow_crawl")
        # No telemetry attached -> blind fixed-dwell staging, said honestly.
        assert out["staged_through"] == ["kneel_two_legs:blind", "crawl:blind"]

    def test_blocked_intermediate_refuses_before_sending_anything(self, monkeypatch):
        monkeypatch.setenv("SONIC_MODE_ALLOWLIST", "idle,crawl,elbow_crawl")
        link = RecordingLink()
        a = SonicG1Adapter(link, StubGuard(), clock=ScriptedClock())
        with pytest.raises(ValueError):
            a._set_mode("elbow_crawl")   # needs kneel_two_legs, not allowed
        assert [c for c in link.calls if c[0] == "select_mode"] == []


class TestContactAndHeightChannels:
    def test_watchdog_passes_no_contacts_without_a_sim(self):
        _, a, _ = build([STANDING])
        assert a._watchdog_contacts() is None

    def test_set_height_informs_the_detector(self):
        _, a, _ = build([STANDING])
        a._set_height(0.6)
        assert a._fall._commanded_height == pytest.approx(0.6)


class SettablePose:
    """Telemetry stand-in whose reading a test can move mid-run, so the body
    can be made to actually arrive somewhere."""

    def __init__(self, pose):
        self.current = dict(pose)

    def pose(self):
        return dict(self.current)

    def lowstate(self):
        return {"active": True, "age_s": 0.0, "source": "SettablePose"}


class NoLowstateTelemetry:
    """A telemetry source predating lowstate() — exactly what the watchdog met
    when the LOW_STATE_ABSENT check landed unguarded."""

    def pose(self):
        return dict(STANDING)


class ExplodingTelemetry:
    def pose(self):
        raise RuntimeError("DDS reader is gone")

    def lowstate(self):
        raise RuntimeError("DDS reader is gone")


class TestWatchdogSurvivesItsOwnReaders:
    """The watchdog is the safety net. A net that dies quietly is worse than no
    net, because start_fall_watchdog() already told the operator it was up."""

    def test_telemetry_without_lowstate_does_not_break_the_beat(self, capsys):
        _, a, _ = build([STANDING])
        a._telemetry = NoLowstateTelemetry()
        assert a._fall_poll_once() is not None      # the beat still lands
        assert not a._fall.fallen                   # and still reads standing
        out = capsys.readouterr().out
        assert "LOW_STATE_UNKNOWN" in out
        assert "LOW_STATE_ABSENT" not in out        # we do not know, so we do not claim

    def test_a_raising_beat_never_kills_the_watchdog_thread(self, capsys):
        _, a, _ = build([STANDING])
        a._telemetry = ExplodingTelemetry()

        boom = []

        def explode():
            boom.append(1)
            raise RuntimeError("beat exploded")

        a._fall_poll_once = explode
        assert a.start_fall_watchdog() is True
        deadline = time.monotonic() + 2.0
        while len(boom) < 3 and time.monotonic() < deadline:
            time.sleep(0.02)
        thread_alive = a._watchdog_thread.is_alive()
        a.stop_fall_watchdog()
        assert len(boom) >= 3, "the loop stopped beating after the first raise"
        assert thread_alive, "the watchdog thread died on a raising beat"
        assert capsys.readouterr().out.count("FALL WATCHDOG BEAT FAILED") == 1

    def test_absent_lowstate_alarms_once_then_rearms_when_it_returns(self, capsys):
        _, a, _ = build([STANDING])
        a._telemetry.lowstate = lambda: {
            "active": False, "note": "no rt/lowstate has EVER arrived"}
        a._fall_poll_once()
        a._fall_poll_once()
        out = capsys.readouterr().out
        assert out.count("LOW_STATE_ABSENT ALARM") == 1, "alarm must not spam at 10 Hz"

        a._telemetry.lowstate = lambda: {"active": True, "age_s": 0.01}
        a._fall_poll_once()
        assert capsys.readouterr().out == "", "a healthy beat says nothing"
        a._telemetry.lowstate = lambda: {"active": False, "note": "stale again"}
        a._fall_poll_once()
        assert capsys.readouterr().out.count("LOW_STATE_ABSENT ALARM") == 1

    def test_an_unknown_never_suppresses_the_absent_alarm_that_follows(self, capsys):
        """One shared one-shot flag let a transient UNKNOWN (our own reader
        hiccuped) permanently silence LOW_STATE_ABSENT — and ABSENT is the one
        that means SONIC is about to drop control for good."""
        a = build([STANDING])[1]
        a._telemetry.lowstate = lambda: (_ for _ in ()).throw(RuntimeError("hiccup"))
        a._fall_poll_once()
        assert "LOW_STATE_UNKNOWN" in capsys.readouterr().out

        a._telemetry.lowstate = lambda: {"active": False, "note": "never arrived"}
        a._fall_poll_once()
        assert "LOW_STATE_ABSENT ALARM" in capsys.readouterr().out

    def test_a_missing_active_field_is_unknown_not_absent(self):
        """A shape we did not expect is our fault, not evidence the robot
        stopped publishing. Announcing ABSENT sends the operator to the wrong
        fault entirely."""
        a = build([STANDING])[1]
        a._telemetry.lowstate = lambda: {"age_s": 0.01}     # no 'active' key
        a._check_lowstate()
        assert a._lowstate_alarm_fired == "LOW_STATE_UNKNOWN"

    def test_a_recovered_watchdog_rearms_its_report(self, capsys):
        """Never clearing the one-shot meant one transient hiccup silenced
        every later, permanent degradation for the life of the process."""
        a = build([STANDING])[1]
        beats = {"n": 0}

        def flaky():
            beats["n"] += 1
            if beats["n"] == 1:
                raise RuntimeError("first beat explodes")

        a._fall_poll_once = flaky
        a.WATCHDOG_REARM_BEATS = 3
        assert a.start_fall_watchdog() is True
        deadline = time.monotonic() + 3.0
        while beats["n"] < 6 and time.monotonic() < deadline:
            time.sleep(0.02)
        a.stop_fall_watchdog()
        out = capsys.readouterr().out
        assert "FALL WATCHDOG BEAT FAILED" in out
        assert "beating again after a failure" in out

class TestGetUpVerbCommandsTheProvenSequence:
    """The ONLY get-up route with evidence behind it is
    lie_face_down -> crawl -> kneel_two_legs -> idle (probe-measured 2026-08-02,
    n=1: 0.225 -> 0.151 -> 0.431 -> 0.410 -> 0.787 m). Its load-bearing step is
    that CRAWL'S OWN ENTRY TRANSITION lifts a prone body to a ~0.449 m
    quadruped. Any other order is a different, unmeasured experiment — so what
    the verb actually puts on the wire has to be pinned, not assumed.
    """

    @staticmethod
    def _fallen():
        """A latched-fall adapter with a RecordingLink, which is the only state
        the verb agrees to run in."""
        link = RecordingLink()
        a = SonicG1Adapter(link, StubGuard(), clock=ScriptedClock(),
                           telemetry=ScriptedPoses([FLAT]))
        clock = FakeClock()
        clock.t = 100.0
        a._fall = FallDetector(clock=clock, grace_s=0.0, confirm_samples=1)
        for _ in range(3):
            a._fall_poll_once()
        assert a._fall.fallen
        link.calls.clear()
        return link, a

    @staticmethod
    def _fallen_then_stands():
        """Same latched start, but the body arrives standing once idle is
        commanded — the successful-recovery case."""
        tel = SettablePose(FLAT)
        link = RecordingLink()
        a = SonicG1Adapter(link, StubGuard(), clock=ScriptedClock(),
                           telemetry=tel)
        clock = FakeClock()
        clock.t = 100.0
        a._fall = FallDetector(clock=clock, grace_s=0.0, confirm_samples=1)
        for _ in range(3):
            a._fall_poll_once()
        assert a._fall.fallen
        link.calls.clear()
        real = link.select_mode

        def flip(name, **kw):
            if name == "idle":
                tel.current = dict(STANDING)
            return real(name, **kw)

        link.select_mode = flip
        return link, a, tel

    def test_recover_from_fall_puts_the_proven_order_on_the_wire(self):
        link, a = self._fallen()
        a._recover_from_fall()
        sent = [name for kind, name, _ in link.calls if kind == "select_mode"]
        assert sent == ["lie_face_down", "crawl", "kneel_two_legs", "idle"], (
            "the verb promises the proven sequence; _set_mode's crawl ladder "
            "silently inserts kneel_two_legs before crawl, which asks a PRONE "
            "robot to kneel — the transition the probe never ran")

    def test_crawl_is_entered_at_speed_zero(self):
        """Pinning speed=0 through the crawl transition survived 2/2 where
        vendor speeds dropped a shoulder 4 times in 5."""
        link, a = self._fallen()
        a._recover_from_fall()
        crawl = [(name, speed) for kind, name, speed in link.calls
                 if kind == "select_mode" and name == "crawl"]
        assert crawl == [("crawl", 0.0)]

    def test_idle_is_never_commanded_from_prone(self):
        """idle-from-prone thrashed, collapsed to 0.078 m and dragged the robot
        1.99 m past the band's origin guard. It must only follow kneel."""
        link, a = self._fallen()
        a._recover_from_fall()
        sent = [name for kind, name, _ in link.calls if kind == "select_mode"]
        assert sent.count("idle") == 1
        assert sent[sent.index("idle") - 1] == "kneel_two_legs"

    def test_it_refuses_to_lie_a_standing_robot_down(self):
        """The tool description reads as a rescue, so a model will reach for it
        whenever a task looks stuck. On a standing robot the whole route is a
        command to go to the floor."""
        link = RecordingLink()
        a = SonicG1Adapter(link, StubGuard(), clock=ScriptedClock(),
                           telemetry=ScriptedPoses([STANDING]))
        out = a._recover_from_fall()
        assert out["recovered"] is False
        assert out["refused"] == "already standing"
        assert link.calls == [], "it must send NOTHING when it refuses"

    def test_it_refuses_when_it_cannot_see_and_no_fall_is_latched(self):
        link = RecordingLink()
        a = SonicG1Adapter(link, StubGuard(), clock=ScriptedClock())
        out = a._recover_from_fall()
        assert out["refused"] == "posture unknown"
        assert link.calls == []

    def test_a_latched_fall_overrides_a_blind_read(self):
        """Telemetry dying during a fall must not strand the robot: the latch
        is evidence enough to try."""
        link, a = self._fallen()
        a._telemetry = None
        out = a._recover_from_fall()
        assert "refused" not in out
        sent = [name for kind, name, _ in link.calls if kind == "select_mode"]
        assert sent == ["lie_face_down", "crawl", "kneel_two_legs", "idle"]

    def test_the_latch_is_cleared_only_after_prone_is_commanded(self):
        """Clearing it earlier re-trips the alarm on the next beat, because the
        detector is still holding the envelope of the mode it fell in."""
        link, a = self._fallen()
        cleared_after = []
        real_reset = a.reset_fall_latch

        def spy():
            cleared_after.append([n for k, n, _ in link.calls
                                  if k == "select_mode"])
            real_reset()

        a.reset_fall_latch = spy
        a._recover_from_fall()
        assert cleared_after == [["lie_face_down"]]

    def test_a_shoulder_on_the_floor_during_the_route_is_not_a_new_fall(self):
        """MEASURED 2026-08-06, 2/2 prone trials: the route reached every stage
        and ended at 0.79 m upright, and the detector latched anyway on
        'ground contact on right_shoulder_yaw_link' in crawl. A body coming off
        the floor THROUGH crawl has a shoulder on the floor."""
        det = FallDetector(clock=FakeClock(), grace_s=0.0, confirm_samples=1)
        det.note_mode("crawl")
        det.begin_recovery()
        for _ in range(5):
            det.update(0.30, 20.0, contacts=["right_shoulder_yaw_link"])
        assert not det.fallen, "the recovery route tripped the contact guard"

        # ...and the exemption is scoped: a crawl COLLAPSE must still be caught.
        det.end_recovery()
        for _ in range(5):
            det.update(0.30, 20.0, contacts=["right_shoulder_yaw_link"])
        assert det.fallen

    def test_height_and_tilt_still_apply_while_recovering(self):
        """begin_recovery narrows ONE channel. Switching the detector off for
        the duration would be the farmable scalar this project keeps meeting."""
        det = FallDetector(clock=FakeClock(), grace_s=0.0, confirm_samples=1)
        det.note_mode("crawl")
        det.begin_recovery()
        for _ in range(5):
            det.update(0.05, 88.0, contacts=None)
        assert det.fallen

    def test_a_robot_measured_standing_is_reported_recovered(self):
        """The bug this pins: _posture() short-circuits on the latch, so a
        latch set on the way up reported a robot standing at 0.79 m as a failed
        recovery — 2/2 trials, both wrong."""
        link, a, tel = self._fallen_then_stands()
        out = a._recover_from_fall()
        assert out["recovered"] is True
        assert out["posture"]["height_m"] == pytest.approx(0.786)
        assert a._fall.fallen is False, "a stale latch blocks everything after"

    def test_a_robot_still_down_is_never_reported_recovered(self):
        """The other half — the latch clear must follow the MEASUREMENT, not
        the fact that the route ran to the end."""
        link, a = self._fallen()          # telemetry holds FLAT throughout
        out = a._recover_from_fall()
        assert out["recovered"] is False
        assert "NOT standing" in out["note"]

    def test_a_relatch_during_the_route_is_reported_not_hidden(self):
        """It must not quietly clear a latch it caused and call the job done —
        the operator needs to know the guard fired on the recovery itself."""
        link, a, tel = self._fallen_then_stands()
        real = link.select_mode

        def latch_on_crawl(name, **kw):
            if name == "crawl":           # AFTER the route's own latch reset
                a._fall._fallen = True
                a._fall._reason = "in mode 'crawl': ground contact on shoulder"
            if name == "idle":
                tel.current = dict(STANDING)
            return real(name, **kw)

        link.select_mode = latch_on_crawl
        out = a._recover_from_fall()
        assert out["relatched_during_route"] is True
        assert "latched again during the route" in out["note"]
        assert out["recovered"] is True          # the body is the authority
        assert a._fall.fallen is False           # and the stale latch is cleared

    def test_every_stage_outcome_is_reported(self):
        link, a = self._fallen()
        out = a._recover_from_fall()
        assert [s.split(":")[0] for s in out["route"]] == [
            "lie_face_down", "crawl", "kneel_two_legs", "idle"]
        assert all(s.split(":")[1] in ("reached", "timeout", "blind")
                   for s in out["route"])

    def test_a_blocked_mode_stops_the_route_before_anything_is_sent(self, monkeypatch):
        monkeypatch.setenv("SONIC_MODE_ALLOWLIST", "idle,lie_face_down,crawl")
        link, a = self._fallen()
        a2 = SonicG1Adapter(link, StubGuard(), clock=ScriptedClock(),
                            telemetry=ScriptedPoses([FLAT]))
        a2._fall = a._fall            # carry the latch over
        link.calls.clear()
        with pytest.raises(ValueError):
            a2._recover_from_fall()   # kneel_two_legs is not allowed
        assert link.calls == [], "stranding the robot mid-route is the failure"


class TestWatchdogSurvivesItsOwnReadersPartTwo:
    def test_lowstate_failure_never_suppresses_a_real_fall(self, capsys):
        """The safety property that matters: a broken lowstate reader must not
        cost us the fall it exists alongside."""
        link, a, _ = build([STANDING] * 2 + [FLAT], confirm_samples=1)
        a._telemetry.lowstate = lambda: (_ for _ in ()).throw(RuntimeError("gone"))
        for _ in range(4):
            a._fall_poll_once()
        assert a._fall.fallen
        assert link.damp_calls == 1


class TestTheWatchdogsActuatorIsPartOfTheContract:
    """emergency_stop is the fall watchdog's ONLY actuator, and it lived on
    ZmqSonicLink alone — so every other transport armed a watchdog that could
    not damp. The beat's bare except turned that into a printed line, and the
    offline suite emitted 'AUTO-DAMP FAILED' ten times with nothing asserting
    on it. A safety actuator missing from the interface is the same defect
    class as a test double missing a method, one layer down.
    """

    def test_the_interface_declares_the_damp(self):
        from physicalagent.robots.sonic_link import SonicLink
        assert hasattr(SonicLink, "emergency_stop")

    def test_a_confirmed_fall_actually_damps_a_plain_fake_link(self):
        link = FakeSonicLink()
        a = SonicG1Adapter(link, StubGuard(), clock=ScriptedClock(),
                           telemetry=ScriptedPoses([FLAT]))
        clock = FakeClock()
        clock.t = 100.0
        a._fall = FallDetector(clock=clock, grace_s=0.0, confirm_samples=1)
        a._fall_poll_once()
        assert a._fall.fallen
        assert [c for c in link.commands if c["kind"] == "emergency_stop"], (
            "the watchdog confirmed a fall and the motors were never damped")

    def test_a_damp_failure_is_never_silent(self, capsys):
        """If a transport ever loses it again, the operator must hear about it
        — this is the branch that printed ten times unnoticed."""
        class NoDamp(FakeSonicLink):
            def emergency_stop(self):
                raise RuntimeError("transport gone")

        link = NoDamp()
        a = SonicG1Adapter(link, StubGuard(), clock=ScriptedClock(),
                           telemetry=ScriptedPoses([FLAT]))
        clock = FakeClock()
        clock.t = 100.0
        a._fall = FallDetector(clock=clock, grace_s=0.0, confirm_samples=1)
        a._fall_poll_once()
        assert "AUTO-DAMP FAILED" in capsys.readouterr().out
