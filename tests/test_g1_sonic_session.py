"""Guards for the hardware SONIC session driver.

This is the piece that did not exist: SonicG1Adapter and ZmqSonicLink were both
written and tested, and nothing composed them against a real robot. The g1
profile is an unregistered draft whose unlocked_velocity permits 0.5 m/s --
above the 0.3 in the tier table -- so the driver deliberately does NOT
go through the profile registry.

Running this commands a 35 kg biped. Every test here is a refusal test.
"""
from __future__ import annotations

import importlib.util
import pathlib
import sys

import pytest

REPO = pathlib.Path(__file__).resolve().parents[1]
SCRIPTS = REPO / "scripts"


def _load(name):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def sess():
    return _load("g1_sonic_session")


class TestItRefusesBeforeItCommands:
    def test_commanding_requires_an_explicit_estop_acknowledgement(self, sess):
        """SONIC publishes raw rt/lowcmd, which the tier table puts at
        TWO-KEY: gantry mandatory, plus a DISTINCT e-stop holder. A background
        process cannot be that person and must not act as though it is."""
        with pytest.raises(sess.SafetyRefusal) as e:
            sess.check_ready(estop_ack=False, preflight=lambda: (True, "ok"))
        assert "e-stop" in str(e.value).lower()

    def test_a_failed_preflight_refuses_even_with_the_acknowledgement(self, sess):
        with pytest.raises(sess.SafetyRefusal) as e:
            sess.check_ready(estop_ack=True,
                             preflight=lambda: (False, "SONIC_TIME_SCALE is set"))
        assert "SONIC_TIME_SCALE" in str(e.value)

    def test_both_conditions_together_pass(self, sess):
        sess.check_ready(estop_ack=True, preflight=lambda: (True, "ok"))


class TestTheLinkItActuallyBuilds:
    def test_dry_mode_never_builds_a_link_that_can_command(self, sess):
        link = sess.make_link(real=False)
        assert type(link).__name__ == "FakeSonicLink"

    def test_real_mode_builds_the_zmq_link(self, sess, monkeypatch):
        built = {}
        class Spy:
            def __init__(self, **kw):
                built.update(kw)
            def stop(self): pass
            def emergency_stop(self): pass
            def close(self): pass
        monkeypatch.setattr(sess, "_zmq_link_class", lambda: Spy)
        link = sess.make_link(real=True, port=5556)
        assert isinstance(link, Spy)
        assert built["port"] == 5556

    def test_the_driver_does_not_go_through_the_profile_registry(self, sess):
        """profiles/g1.py permits 0.5 m/s, above the signed 0.3. Reaching the
        robot through it would silently raise the ceiling."""
        text = (SCRIPTS / "g1_sonic_session.py").read_text()
        assert "profiles" not in text or "resolve_profile" not in text
        assert "build_adapter" not in text


class TestItAlwaysStops:
    class _Link:
        def __init__(self):
            self.calls = []
        def stop(self):
            self.calls.append("stop")
        def emergency_stop(self):
            self.calls.append("estop")
        def close(self):
            self.calls.append("close")

    class _Adapter:
        def __init__(self, link):
            self._link = link
            self.stopped = 0
        def stop_all(self):
            self.stopped += 1
            self._link.stop()

    def test_a_raising_body_still_stops_the_robot(self, sess):
        link, ad = self._Link(), None
        with pytest.raises(RuntimeError):
            with sess.stopping(self._Adapter(link), link) as a:
                ad = a
                raise RuntimeError("model blew up mid-walk")
        assert ad.stopped == 1
        assert "stop" in link.calls and "close" in link.calls

    def test_a_clean_exit_also_stops(self, sess):
        link = self._Link()
        with sess.stopping(self._Adapter(link), link):
            pass
        assert "stop" in link.calls and "close" in link.calls

    def test_a_keyboard_interrupt_reaches_the_emergency_stop(self, sess):
        """Ctrl-C is what an operator presses when something looks wrong. It
        must escalate past the ordinary stop."""
        link = self._Link()
        with pytest.raises(KeyboardInterrupt):
            with sess.stopping(self._Adapter(link), link):
                raise KeyboardInterrupt
        assert "estop" in link.calls

    def test_a_failure_inside_stop_all_does_not_prevent_the_link_stop(self, sess):
        class Broken:
            def stop_all(self):
                raise RuntimeError("adapter is wedged")
        link = self._Link()
        with sess.stopping(Broken(), link):
            pass
        assert "stop" in link.calls or "estop" in link.calls, (
            "if the adapter cannot stop the robot, the link still must try")


class TestTelemetryIsAttachedButTheDampIsNotTrustedYet:
    """rt/odommodestate is live, so telemetry now reads real pelvis height and
    the fall detector is no longer blind. That does not make it trustworthy:
    its envelopes assume standing heights, and this robot hangs at a 0.731 m
    SUSPENSION height. A false positive damps the robot -- harmless in a
    harness, but it would abort the run and be read as a real fall.

    So the guard is attached and allowed to REPORT before it is allowed to ACT.
    That is the project's own law: validate a guard against the real motion
    before letting it fire."""

    def test_telemetry_is_attached_when_one_is_supplied(self, sess):
        tel = object()
        ad = sess.make_adapter(_Link(), telemetry=tel)
        assert ad._telemetry is tel

    def test_the_automatic_damp_is_off_by_default(self, sess):
        ad = sess.make_adapter(_Link(), telemetry=object())
        assert ad._auto_damp is False, (
            "the fall guard must be observed on a hoisted robot before it is "
            "allowed to damp on its own")

    def test_it_can_be_turned_on_explicitly(self, sess):
        ad = sess.make_adapter(_Link(), telemetry=object(), auto_damp=True)
        assert ad._auto_damp is True

    def test_no_telemetry_still_builds_rather_than_crashing(self, sess):
        ad = sess.make_adapter(_Link(), telemetry=None)
        assert ad._telemetry is None

    def test_a_telemetry_that_cannot_be_built_degrades_instead_of_aborting(self, sess):
        """A robot mid-battery-swap publishes nothing. Losing status readouts
        is acceptable; losing the ability to stop the robot is not."""
        def boom():
            raise RuntimeError("no DDS")
        tel, note = sess.try_telemetry(boom)
        assert tel is None
        assert "no DDS" in note


class _Link:
    def stop(self): pass
    def emergency_stop(self): pass
    def close(self): pass


class TestTheReadOnlyCheck:
    """--check is the command an operator runs right after bring-up to find out
    whether SONIC took over. It must never need the e-stop acknowledgement,
    because demanding a safety assertion for a read teaches people to pass the
    flag reflexively -- and it must never build a link that could command."""

    def test_check_does_not_require_the_estop_acknowledgement(self, sess):
        assert sess.needs_estop(check=True, dry=False, manual=None,
                                task="", real=True) is False

    def test_commanding_still_requires_it(self, sess):
        assert sess.needs_estop(check=False, dry=False, manual="arms",
                                task="", real=True) is True
        assert sess.needs_estop(check=False, dry=False, manual=None,
                                task="wave", real=True) is True

    def test_dry_does_not_require_it_either(self, sess):
        assert sess.needs_estop(check=False, dry=True, manual=None,
                                task="", real=True) is False

    def test_check_renders_the_controller_verdict_verbatim(self, sess):
        state = {"controller": {"active": False, "note": "idle heartbeat"},
                 "pose": {"available": True, "height_m": 0.73, "tilt_deg": 2.9}}
        text = "\n".join(sess.render_check(state))
        assert "idle heartbeat" in text
        assert "0.73" in text

    def test_check_says_so_loudly_when_nothing_is_in_control(self, sess):
        state = {"controller": {"active": False, "note": "x"}, "pose": {}}
        text = "\n".join(sess.render_check(state))
        assert "NOT IN CONTROL" in text

    def test_check_confirms_control_when_gains_are_real(self, sess):
        state = {"controller": {"active": True, "max_kp": 60.0, "max_kd": 1.5,
                                "max_tau": 0.0}, "pose": {}}
        text = "\n".join(sess.render_check(state))
        assert "IN CONTROL" in text and "NOT IN CONTROL" not in text


class TestCheckShowsHeat:
    """The session that stopped on 2026-08-19 stopped on a warning nobody could
    check. --check is what the operator runs between steps, so it is where a
    rising joint has to be visible -- before it trips, not after."""

    def test_the_hottest_joints_appear_in_the_check(self, sess):
        state = {"controller": {"active": True, "max_kp": 60.0, "max_kd": 1.5,
                                "max_tau": 0.0},
                 "pose": {},
                 "thermal": {"available": True, "status": "OK",
                             "hottest": [{"joint": "right_ankle_roll",
                                          "max_c": 62, "winding_c": 62,
                                          "driver_c": 60}],
                             "over_warn": [], "approaching": []}}
        text = "\n".join(sess.render_check(state))
        assert "right_ankle_roll" in text and "62" in text

    def test_an_over_temperature_joint_is_shouted(self, sess):
        state = {"controller": {"active": True}, "pose": {},
                 "thermal": {"available": True, "status": "OVER",
                             "hottest": [{"joint": "right_ankle_roll",
                                          "max_c": 91, "winding_c": 91,
                                          "driver_c": 88}],
                             "over_warn": ["right_ankle_roll"],
                             "approaching": []}}
        text = "\n".join(sess.render_check(state))
        assert "OVER TEMPERATURE" in text.upper()
        assert "right_ankle_roll" in text

    def test_the_watch_band_is_shown_distinctly_from_over(self, sess):
        state = {"controller": {"active": True}, "pose": {},
                 "thermal": {"available": True, "status": "WATCH",
                             "hottest": [{"joint": "right_ankle_roll",
                                          "max_c": 87, "winding_c": 87,
                                          "driver_c": 85}],
                             "over_warn": [], "approaching": ["right_ankle_roll"]}}
        text = "\n".join(sess.render_check(state)).upper()
        assert "WATCH" in text and "OVER TEMPERATURE" not in text

    def test_missing_thermal_does_not_break_the_check(self, sess):
        text = "\n".join(sess.render_check(
            {"controller": {"active": True}, "pose": {}}))
        assert "IN CONTROL" in text


class TestTheInteractiveSession:
    """SONIC in zmq_manager mode has no keyboard: the ']' start key belongs to
    the keyboard interface and does nothing here. Control must be started over
    ZMQ instead.

    And ZmqSonicLink BINDS tcp://*:5556 while SONIC connects to it, so only one
    of our processes can exist at a time -- a second would fail to bind. So one
    long-lived session has to start control and hold it while issuing commands,
    rather than a command per invocation."""

    def test_status_needs_no_arguments(self, sess):
        assert sess.parse_command("status") == ("status", {})

    def test_arms_takes_a_pose(self, sess):
        assert sess.parse_command("arms wave_right") == (
            "arms", {"pose": "wave_right"})

    def test_walk_takes_speed_and_duration(self, sess):
        act, kw = sess.parse_command("walk 0.1 2")
        assert act == "walk"
        assert kw["speed"] == pytest.approx(0.1)
        assert kw["seconds"] == pytest.approx(2.0)

    def test_bare_walk_refuses_rather_than_defaulting(self, sess):
        """SUPERSEDES test_walk_defaults_are_conservative (2026-08-19).

        That test asserted a bare `walk` moved at a conservative 0.1 m/s for
        2 s. Conservative is not the point: those numbers were chosen by the
        code, not the operator, so a truncated input became a motion command.
        A refusal is strictly safer than a small motion nobody asked for."""
        act, kw = sess.parse_command("walk")
        assert act == "error"
        assert "speed" in kw["message"] and "duration" in kw["message"]

    def test_stop_and_quit_are_both_understood(self, sess):
        assert sess.parse_command("stop")[0] == "stop"
        assert sess.parse_command("quit")[0] == "quit"
        assert sess.parse_command("q")[0] == "quit"

    def test_blank_input_is_a_no_op_not_an_error(self, sess):
        assert sess.parse_command("   ")[0] == "noop"

    def test_an_unknown_word_is_refused_rather_than_guessed(self, sess):
        act, kw = sess.parse_command("jump really high")
        assert act == "error"
        assert "jump" in kw["message"]

    def test_a_bad_number_is_refused_rather_than_becoming_zero(self, sess):
        """'walk fast' silently becoming 0.0 m/s would look like a dead robot;
        becoming a default would be a command nobody typed."""
        act, kw = sess.parse_command("walk fast")
        assert act == "error"

    def test_the_help_text_lists_every_command_it_accepts(self, sess):
        text = "\n".join(sess.interactive_help())
        for word in ("status", "arms", "walk", "stop", "quit"):
            assert word in text


class TestManualCommandsUseTheParameterNamesTheToolsActuallyDeclare:
    """2026-08-19, on the physical G1 with SONIC in control: every typed command
    was refused.

        g1> arms arms_forward
          set_arms: ToolResult(ok=False, error="missing required parameter 'pose'")
        g1> walk 0.05 2
          walk: ToolResult(ok=False, error="missing required parameter 'vx'")

    run_manual sent {"name": ...} to a tool whose schema requires "pose", and
    {"speed": ...} to one that requires "vx". The existing tests all passed,
    because every one of them drove a MOCK adapter that accepted any key. They
    proved the session could talk to a fake robot.

    These tests read the required-parameter list off the real ToolSpecs, so a
    rename on either side breaks them."""

    def _required(self, name):
        """Required params straight off the live ToolSpec -- not a copy."""
        from physicalagent.robots.sonic_g1 import SonicG1Adapter

        class NullLink:
            def __getattr__(self, _):
                return lambda *a, **k: None

        for spec in SonicG1Adapter(NullLink()).capabilities():
            if spec.name == name:
                return set(spec.parameters.get("required", []))
        raise AssertionError(f"no ToolSpec named {name!r}")

    def _sent(self, sess, kind, **kw):
        seen = {}

        class Spy:
            def execute(self, name, params):
                seen["name"], seen["params"] = name, params
                return "ok"

            def stop_all(self):
                return "ok"

        sess.run_manual(Spy(), kind, **kw)
        return seen

    def test_arms_sends_the_pose_key_the_tool_requires(self, sess):
        sent = self._sent(sess, "arms", pose="arms_forward")
        assert self._required("set_arms") <= set(sent["params"])
        assert sent["params"]["pose"] == "arms_forward"

    def test_walk_sends_the_vx_key_the_tool_requires(self, sess):
        sent = self._sent(sess, "walk", speed=0.05, seconds=2.0)
        assert self._required("walk") <= set(sent["params"])
        assert sent["params"]["vx"] == 0.05

    def test_walk_still_carries_the_duration(self, sess):
        sent = self._sent(sess, "walk", speed=0.05, seconds=2.0)
        assert sent["params"]["duration_s"] == 2.0

    def test_every_manual_verb_satisfies_its_tools_required_params(self, sess):
        """The general form. Any future verb is covered without a new test."""
        for kind, kw in (("arms", {"pose": "neutral"}),
                         ("walk", {"speed": 0.1, "seconds": 1.0})):
            sent = self._sent(sess, kind, **kw)
            missing = self._required(sent["name"]) - set(sent["params"])
            assert not missing, f"{kind} -> {sent['name']} missing {missing}"


class TestTheThermalWatchStopsTheRobotWithoutBeingAsked:
    """A verdict nobody reads is not a guard. On 2026-08-19 the operator was
    watching, had been told to watch, and still could not have caught motor 22:
    it crossed 90 C 6.5 s after leaving 65 C. The loop has to do it.

    The response is de-escalating on purpose -- stop locomotion, then put the
    arms back to neutral. A held raised-arm pose is what heats the shoulder
    driver, and the capture shows it cooling the moment the arm came down."""

    class Spy:
        def __init__(self):
            self.calls = []

        def execute(self, name, params):
            self.calls.append((name, params))
            return "ok"

        def stop_all(self):
            self.calls.append(("stop_all", {}))
            return "ok"

    def _thermal(self, c):
        return {"available": True,
                "hottest": [{"index": 22, "joint": "right_shoulder_pitch",
                             "winding_c": c - 40, "driver_c": c, "max_c": c}]}

    def test_it_does_nothing_while_the_robot_is_cool(self, sess):
        spy = self.Spy()
        fired = sess.thermal_watch_step(spy, self._thermal(55))
        assert fired is False
        assert spy.calls == []

    def test_it_stops_and_unloads_at_the_limit(self, sess):
        spy = self.Spy()
        fired = sess.thermal_watch_step(spy, self._thermal(91))
        assert fired is True
        names = [n for n, _ in spy.calls]
        assert "stop_all" in names
        assert "set_arms" in names

    def test_it_returns_the_arms_to_neutral_specifically(self, sess):
        spy = self.Spy()
        sess.thermal_watch_step(spy, self._thermal(95))
        arms = [p for n, p in spy.calls if n == "set_arms"]
        assert arms and arms[0]["pose"] == "neutral"

    def test_it_stops_before_it_moves_anything(self, sess):
        """Order matters: halt locomotion first, then unload."""
        spy = self.Spy()
        sess.thermal_watch_step(spy, self._thermal(99))
        names = [n for n, _ in spy.calls]
        assert names.index("stop_all") < names.index("set_arms")

    def test_a_blind_guard_never_moves_the_robot(self, sess):
        """Unknown temperature must not trigger motion nobody asked for."""
        spy = self.Spy()
        fired = sess.thermal_watch_step(spy, {"available": False, "hottest": []})
        assert fired is False
        assert spy.calls == []

    def test_it_survives_the_adapter_refusing(self, sess):
        """If the stop itself fails, the guard must not take the session down
        -- the operator still needs the prompt to hit the real stop."""
        class Broken(self.Spy):
            def stop_all(self):
                raise RuntimeError("link gone")
        fired = sess.thermal_watch_step(Broken(), self._thermal(120))
        assert fired is True


class TestTheTypedVerbSetCoversWhatTheRobotCanActuallyDo:
    """The adapter exposes 23 tools; the typed session exposed 5. Everything
    below already works on the adapter -- this is about reaching it from the
    prompt, which is the only interface that has ever driven this robot.

    Read-only verbs are free and are added without reservation. Motion verbs
    carry the same clamps as walk, because they go through the same adapter."""

    def _sent(self, sess, line):
        seen = []

        class Spy:
            def execute(self, name, params):
                seen.append((name, params))
                return "ok"

            def stop_all(self):
                seen.append(("stop_all", {}))
                return "ok"

        act, kw = sess.parse_command(line)
        assert act not in ("error", "noop"), f"{line!r} -> {kw}"
        if act in ("status", "quit"):
            return act, seen
        sess.run_manual(Spy(), act, **kw)
        return act, seen

    def test_turn_reaches_the_turn_tool_with_a_yaw_rate(self, sess):
        _, seen = self._sent(sess, "turn 0.2 2")
        assert seen[0][0] == "turn"
        assert seen[0][1]["yaw_rate"] == 0.2
        assert seen[0][1]["duration_s"] == 2.0

    def test_pose_reaches_set_pose(self, sess):
        _, seen = self._sent(sess, "pose stand")
        assert seen[0] == ("set_pose", {"name": "stand"})

    def test_height_reaches_set_height(self, sess):
        _, seen = self._sent(sess, "height 0.75")
        assert seen[0] == ("set_height", {"height_m": 0.75})

    def test_joints_is_read_only(self, sess):
        _, seen = self._sent(sess, "joints left_arm")
        assert seen[0] == ("get_joints", {"limb": "left_arm"})

    def test_joints_without_a_limb_asks_for_all_of_them(self, sess):
        _, seen = self._sent(sess, "joints")
        assert seen[0] == ("get_joints", {})

    def test_contacts_is_read_only(self, sess):
        _, seen = self._sent(sess, "contacts")
        assert seen[0] == ("get_contacts", {})

    def test_hands_is_read_only(self, sess):
        _, seen = self._sent(sess, "hands")
        assert seen[0] == ("where_are_my_hands", {})

    def test_release_hands_the_arms_back_to_the_balance_controller(self, sess):
        _, seen = self._sent(sess, "release")
        assert seen[0] == ("release_arms", {})

    def test_estop_is_reachable_by_name(self, sess):
        _, seen = self._sent(sess, "estop")
        assert seen[0][0] == "emergency_stop"

    def test_every_new_verb_satisfies_its_tools_required_params(self, sess):
        from physicalagent.robots.sonic_g1 import SonicG1Adapter

        class NullLink:
            def __getattr__(self, _):
                return lambda *a, **k: None

        req = {s.name: set(s.parameters.get("required", []))
               for s in SonicG1Adapter(NullLink()).capabilities()}
        for line in ("turn 0.2 2", "pose stand", "height 0.75",
                     "joints left_arm", "contacts", "hands", "release",
                     "estop", "arms neutral", "walk 0.1 1"):
            _, seen = self._sent(sess, line)
            for name, params in seen:
                if name in req:
                    missing = req[name] - set(params)
                    assert not missing, f"{line!r} -> {name} missing {missing}"

    def test_a_number_that_is_not_a_number_is_refused_not_defaulted(self, sess):
        for line in ("turn fast", "height tall", "walk quickly"):
            act, kw = sess.parse_command(line)
            assert act == "error", f"{line!r} was accepted as {act} {kw}"

    def test_goto_and_set_mode_stay_out_of_the_typed_surface(self, sess):
        """goto steers to a world point and set_mode is the whole-body hammer.
        Neither belongs on a hoisted robot that cannot actually translate."""
        for line in ("goto 1 0", "mode walk"):
            act, _ = sess.parse_command(line)
            assert act == "error", f"{line!r} should not be typed-reachable"

    def test_help_lists_every_verb_the_parser_accepts(self, sess):
        """Help that omits a verb is how a working feature stays unused."""
        text = " ".join(sess.interactive_help())
        for verb in ("status", "arms", "walk", "turn", "pose", "height",
                     "joints", "contacts", "hands", "release", "stop",
                     "estop", "quit"):
            assert verb in text, f"{verb!r} missing from help"


class TestTheParserRefusesInsteadOfSilentlyDiscarding:
    """Two silent-success holes, found while designing voice input but present
    for typing too:

        'arms wave right'  ->  ('arms', {'pose': 'wave'})   trailing token DROPPED
        'walk'             ->  ('walk', {...0.1, 2.0})      bare verb MOVES

    The first is safe today only by luck: 'wave' is not in POSES so set_arms
    refuses downstream. Rename a pose and it stops being luck.

    The second matters most: a truncated input -- a mis-transcription, a
    fat-fingered Enter -- becomes a motion command with numbers nobody chose."""

    def test_a_trailing_token_is_refused_not_dropped(self, sess):
        act, kw = sess.parse_command("arms wave right")
        assert act == "error", (act, kw)
        assert "wave right" in kw["message"] or "right" in kw["message"]

    def test_the_refusal_lists_what_could_have_been_meant(self, sess):
        _, kw = sess.parse_command("arms wave right")
        assert "wave_right" in kw["message"]

    def test_an_exact_pose_still_works(self, sess):
        assert sess.parse_command("arms wave_right") == (
            "arms", {"pose": "wave_right"})

    def test_bare_walk_no_longer_moves_the_robot(self, sess):
        act, kw = sess.parse_command("walk")
        assert act == "error", f"bare walk produced {act} {kw}"

    def test_bare_turn_no_longer_moves_the_robot(self, sess):
        act, kw = sess.parse_command("turn")
        assert act == "error", f"bare turn produced {act} {kw}"

    def test_walk_with_a_speed_still_works(self, sess):
        assert sess.parse_command("walk 0.1") == (
            "walk", {"speed": 0.1, "seconds": 2.0})

    def test_trailing_punctuation_does_not_break_a_verb(self, sess):
        """A transcriber emits 'Stop.', never 'stop'. Costs nothing for typing
        and is the difference between working and not for voice."""
        for spoken, expect in (("Stop.", "stop"), ("Status.", "status"),
                               ("status", "status"), ("QUIT!", "quit")):
            act, _ = sess.parse_command(spoken)
            assert act == expect, f"{spoken!r} -> {act}"

    def test_extra_words_on_a_read_only_verb_are_still_refused(self, sess):
        act, _ = sess.parse_command("status please")
        assert act == "error"


class TestTheEstopAssertionIsDemandedOnlyWhenSomethingCanMove:
    """needs_estop ignored --real, so rehearsing a task against FakeSonicLink
    -- where no DDS message is published and no motor exists -- refused unless
    the operator asserted that a second human was standing at an e-stop.

    Its own docstring names the hazard: 'Requiring it for a read would train an
    operator to pass it reflexively, which is exactly how a two-key gate
    becomes a habit instead of a check.' Making someone type the assertion to
    run a simulation is that training. The flag must mean one thing, and mean
    it every time it is typed."""

    def test_a_simulated_task_needs_no_estop_holder(self, sess):
        assert sess.needs_estop(check=False, dry=False, manual=None,
                                task="wave", real=False) is False

    def test_a_simulated_manual_command_needs_no_estop_holder(self, sess):
        assert sess.needs_estop(check=False, dry=False, manual="arms",
                                task=None, real=False) is False

    def test_the_real_robot_still_demands_it_for_a_task(self, sess):
        assert sess.needs_estop(check=False, dry=False, manual=None,
                                task="wave", real=True) is True

    def test_the_real_robot_still_demands_it_for_a_manual_command(self, sess):
        assert sess.needs_estop(check=False, dry=False, manual="walk",
                                task=None, real=True) is True

    def test_reads_never_demand_it_on_either_target(self, sess):
        for real in (True, False):
            assert sess.needs_estop(check=True, dry=False, manual=None,
                                    task=None, real=real) is False
            assert sess.needs_estop(check=False, dry=True, manual=None,
                                    task=None, real=real) is False

    def test_doing_nothing_demands_nothing(self, sess):
        assert sess.needs_estop(check=False, dry=False, manual=None,
                                task=None, real=True) is False


class TestTheTaskTraceIsReadableDuringADemo:
    """The first ER-2 run printed ~8 KB of repr() for a 5-call task: every
    joint of every pose, twice, plus the full status dict three times. During a
    demo, with a live robot in a gantry, that is unreadable -- and unreadable
    output is how a wrong action goes unnoticed while it scrolls past.

    One line per hop, with the argument that matters and the outcome."""

    class T:
        def __init__(self, name, args, ok=True, error=None, elapsed_s=0.0,
                     result=None):
            self.name, self.args, self.ok = name, args, ok
            self.error, self.elapsed_s, self.result = error, elapsed_s, result

    def test_one_line_per_hop(self, sess):
        lines = sess.render_trace([
            self.T("get_status", {}),
            self.T("set_arms", {"pose": "wave_right"}, elapsed_s=1.5),
        ])
        assert len(lines) == 2

    def test_the_arguments_are_shown(self, sess):
        line = sess.render_trace([self.T("set_arms", {"pose": "wave_right"})])[0]
        assert "set_arms" in line and "wave_right" in line

    def test_a_failure_is_obvious(self, sess):
        line = sess.render_trace([
            self.T("walk", {"vx": 9.0}, ok=False, error="clamped")])[0]
        assert "FAIL" in line or "!" in line
        assert "clamped" in line

    def test_timing_is_kept_because_it_is_the_demo_metric(self, sess):
        line = sess.render_trace([self.T("set_arms", {"pose": "neutral"},
                                          elapsed_s=1.53)])[0]
        assert "1.5" in line

    def test_no_line_is_longer_than_a_terminal(self, sess):
        big = {"joints": {f"j{i}": i * 1.0 for i in range(40)}}
        for line in sess.render_trace([self.T("set_joints", big)]):
            assert len(line) <= 100, len(line)

    def test_an_empty_trace_renders_nothing(self, sess):
        assert sess.render_trace([]) == []


class TestGeminiCanBePromptedRepeatedlyInsideOneHeldSession:
    """`--task` ran one prompt and exited, which drops control and leaves the
    robot limp between prompts. The operator wants to keep talking to ER-2 with
    the session held.

    Brain already persists its conversation across run() calls (`self._convo`
    is the memory, with an explicit reset()), so follow-ups like "now do that
    again, slower" carry context. One Brain per session, not per prompt.

    The prompt is FREE TEXT and must survive the tokeniser intact: punctuation
    stripping and the trailing-token refusal exist to protect the verb grammar,
    and applying them to a sentence would mangle it."""

    def test_task_takes_a_free_text_prompt(self, sess):
        act, kw = sess.parse_command("task wave your right hand")
        assert act == "task"
        assert kw["prompt"] == "wave your right hand"

    def test_ask_is_an_alias(self, sess):
        act, kw = sess.parse_command("ask what is your height")
        assert act == "task"
        assert kw["prompt"] == "what is your height"

    def test_punctuation_inside_a_prompt_is_preserved(self, sess):
        _, kw = sess.parse_command(
            "task Read your status, then wave. Do not walk.")
        assert kw["prompt"] == "Read your status, then wave. Do not walk."

    def test_case_is_preserved_in_a_prompt(self, sess):
        _, kw = sess.parse_command("task Wave Your RIGHT Hand")
        assert kw["prompt"] == "Wave Your RIGHT Hand"

    def test_an_empty_prompt_is_refused(self, sess):
        act, kw = sess.parse_command("task")
        assert act == "error"
        assert "prompt" in kw["message"].lower()

    def test_reset_clears_the_models_memory(self, sess):
        assert sess.parse_command("reset") == ("reset", {})

    def test_the_verb_grammar_is_unaffected(self, sess):
        """task/ask must not weaken the strict parsing of everything else."""
        assert sess.parse_command("walk")[0] == "error"
        assert sess.parse_command("arms wave right")[0] == "error"
        assert sess.parse_command("Status.")[0] == "status"

    def test_help_mentions_prompting_gemini(self, sess):
        text = " ".join(sess.interactive_help())
        assert "task" in text and "reset" in text


class TestTheBrainIsBuiltOncePerSessionNotOncePerPrompt:
    def test_the_same_brain_is_reused(self, sess):
        made = []

        class FakeBrain:
            def __init__(self):
                made.append(self)

            def run(self, prompt):
                return type("R", (), {"final_text": "ok", "trace": [],
                                      "think_s": 0.0})()

        holder = {}
        for _ in range(3):
            sess.brain_for(holder, lambda: FakeBrain()).run("x")
        assert len(made) == 1, f"built {len(made)} brains"

    def test_reset_forces_a_new_conversation(self, sess):
        made = []

        class FakeBrain:
            def __init__(self):
                made.append(self)
                self.was_reset = False

            def reset(self):
                self.was_reset = True

        holder = {}
        b = sess.brain_for(holder, lambda: FakeBrain())
        sess.reset_brain(holder)
        assert b.was_reset is True


class TestEveryGlobalNameTheScriptsUseActuallyExists:
    """2026-08-19, on the physical robot, AFTER control had been taken:

        [session] start_control() ...
        [session] {'started': True, 'planner_mode': True}
        NameError: name 'T_MOD' is not defined

    A rewrite of an adjacent function removed a module-level import and left
    one reference behind. 1554 tests passed, because not one of them executed
    the --start banner -- the line runs only when a real link has been opened.

    The cost of that gap is specific: the crash landed after start_control()
    succeeded, so SONIC held the robot with no session and no thermal watch,
    while right_ankle_roll sat at 79 C.

    No linter is installed here, so this is the linter: walk the AST of each
    script and assert every global name it loads is defined somewhere it could
    come from. It catches the whole class, not just this instance."""

    import pathlib
    SCRIPTS = ["g1_sonic_session.py", "g1_voice.py", "hw_recorder.py",
               "hw_preflight.py", "hw_timebase_probe.py"]

    def _undefined(self, path):
        import ast, builtins
        tree = ast.parse(path.read_text(), filename=str(path))
        # Module dunders always exist at runtime; they are not names
        # the file has to define.
        defined = set(dir(builtins)) | {
            "__file__", "__name__", "__doc__", "__package__",
            "__spec__", "__loader__", "__builtins__"}
        for node in ast.walk(tree):
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef,
                                 ast.ClassDef)):
                defined.add(node.name)
            elif isinstance(node, ast.Name) and isinstance(node.ctx, ast.Store):
                defined.add(node.id)
            elif isinstance(node, (ast.Import, ast.ImportFrom)):
                for a in node.names:
                    defined.add((a.asname or a.name).split(".")[0])
            elif isinstance(node, ast.arg):
                defined.add(node.arg)
            elif isinstance(node, ast.ExceptHandler) and node.name:
                defined.add(node.name)
            elif isinstance(node, (ast.Global, ast.Nonlocal)):
                defined.update(node.names)
            elif isinstance(node, ast.comprehension):
                for n in ast.walk(node.target):
                    if isinstance(n, ast.Name):
                        defined.add(n.id)
        used = {n.id for n in ast.walk(tree)
                if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load)}
        return sorted(used - defined)

    def test_no_script_references_a_name_it_never_defines(self):
        root = self.pathlib.Path(__file__).resolve().parents[1] / "scripts"
        problems = {}
        for name in self.SCRIPTS:
            path = root / name
            if not path.exists():
                continue
            missing = self._undefined(path)
            if missing:
                problems[name] = missing
        assert not problems, f"undefined global names: {problems}"

    def test_the_checker_would_have_caught_the_real_bug(self, tmp_path):
        """Proof the guard works, using the exact shape of the failure."""
        bad = tmp_path / "bad.py"
        bad.write_text("def f():\n    return T_MOD.TEMP_ACT_C\n")
        assert "T_MOD" in self._undefined(bad)

    def test_the_checker_does_not_cry_wolf_on_a_correct_file(self, tmp_path):
        ok = tmp_path / "ok.py"
        ok.write_text("import os\nX = 1\n\n\ndef f(a):\n"
                      "    return os.path.join(str(a), str(X))\n")
        assert self._undefined(ok) == []


# ---------------------------------------------------------------------------
# The battery: a deterministic verb script whose whole point is that the SAME
# file runs against the hardware today and against the simulator off-robot, and
# the two captures can then be diffed segment by segment. Everything below is
# in service of that comparison being trustworthy.
# ---------------------------------------------------------------------------


class TestTheScriptGrammarIsStricterThanTheKeyboard:
    """A battery line nobody can replay deterministically has no business in a
    battery. `task` goes to a model, `quit`/`reset` are conversation acts, and
    `estop` is a HUMAN act -- scripting it would teach the file to cry wolf."""

    def test_sleep_parses_to_seconds(self, sess):
        assert sess.parse_script_line("sleep 2.5") == ("sleep", {"seconds": 2.5})

    def test_sleep_refuses_a_bare_verb(self, sess):
        act, kw = sess.parse_script_line("sleep")
        assert act == "error"

    def test_sleep_refuses_a_non_number(self, sess):
        act, kw = sess.parse_script_line("sleep soon")
        assert act == "error"

    def test_sleep_refuses_negative(self, sess):
        act, kw = sess.parse_script_line("sleep -1")
        assert act == "error"

    def test_sleep_refuses_a_typo_sized_hold(self, sess):
        # `sleep 400` is a fat-fingered `sleep 4.00`. Refusing beats holding a
        # powered robot in a pose for seven minutes.
        act, kw = sess.parse_script_line("sleep 400")
        assert act == "error"

    def test_mark_parses_a_label(self, sess):
        assert sess.parse_script_line("mark arms-up") == \
            ("mark", {"label": "arms-up"})

    def test_mark_refuses_multiword_labels(self, sess):
        act, kw = sess.parse_script_line("mark arms up")
        assert act == "error"

    def test_mark_refuses_filename_hostile_labels(self, sess):
        # Labels become comparator output filenames (seg_<label>_<joint>.svg).
        act, kw = sess.parse_script_line("mark a/b")
        assert act == "error"

    def test_comments_and_blanks_are_noops(self, sess):
        for line in ("# a comment", "", "   "):
            assert sess.parse_script_line(line) == ("noop", {})

    def test_model_verbs_are_refused(self, sess):
        for line in ("task wave at me", "ask what do you see"):
            act, kw = sess.parse_script_line(line)
            assert act == "error", line
            assert "deterministic" in kw["message"]

    def test_interactive_only_verbs_are_refused(self, sess):
        for line in ("quit", "exit", "q", "reset", "estop"):
            act, kw = sess.parse_script_line(line)
            assert act == "error", line

    def test_motion_verbs_pass_through_the_keyboard_grammar(self, sess):
        assert sess.parse_script_line("walk 0.1 2") == \
            ("walk", {"speed": 0.1, "seconds": 2.0})
        assert sess.parse_script_line("arms wave_right") == \
            ("arms", {"pose": "wave_right"})
        assert sess.parse_script_line("status") == ("status", {})


class TestABatteryIsValidatedWholeBeforeAnythingMoves:
    """A battery that dies on line 9 leaves a robot mid-experiment and a
    capture that can never be compared. Refuse the file, never the step."""

    GOOD = ("# sim-match battery\n"
            "mark battery-start\n"
            "status\n"
            "sleep 1\n"
            "arms arms_up\n"
            "mark battery-end\n")

    def test_a_valid_battery_loads_with_line_numbers(self, sess, tmp_path):
        p = tmp_path / "b.txt"
        p.write_text(self.GOOD)
        script = sess.load_script(str(p))
        assert script["path"] == str(p)
        acts = [(lineno, act) for lineno, raw, act, kw in script["steps"]]
        assert acts == [(2, "mark"), (3, "status"), (4, "sleep"),
                        (5, "arms"), (6, "mark")]

    def test_the_sha_matches_the_bytes(self, sess, tmp_path):
        import hashlib
        p = tmp_path / "b.txt"
        p.write_text(self.GOOD)
        script = sess.load_script(str(p))
        assert script["sha256"] == hashlib.sha256(
            p.read_bytes()).hexdigest()

    def test_one_bad_line_refuses_the_whole_file_naming_the_line(
            self, sess, tmp_path):
        p = tmp_path / "b.txt"
        p.write_text("mark battery-start\nwalk\nmark battery-end\n")
        with pytest.raises(sess.SafetyRefusal) as e:
            sess.load_script(str(p))
        assert "line 2" in str(e.value)
        assert "walk" in str(e.value)

    def test_fewer_than_two_marks_is_refused(self, sess, tmp_path):
        p = tmp_path / "b.txt"
        p.write_text("mark only-one\nstatus\n")
        with pytest.raises(sess.SafetyRefusal):
            sess.load_script(str(p))

    def test_an_empty_battery_is_refused(self, sess, tmp_path):
        p = tmp_path / "b.txt"
        p.write_text("# nothing but comments\n\n")
        with pytest.raises(sess.SafetyRefusal):
            sess.load_script(str(p))

    def test_a_missing_file_is_refused(self, sess, tmp_path):
        with pytest.raises(sess.SafetyRefusal):
            sess.load_script(str(tmp_path / "nope.txt"))


class _ScriptSpy:
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


def _write_battery(tmp_path, body):
    p = tmp_path / "battery.txt"
    p.write_text(body)
    return str(p)


class TestRunScriptWritesTheLogThatAlignsTheCaptures:
    """The JSONL log is the join key between a battery run and its DDS capture
    -- and between the hardware capture and the sim capture. If it lies about
    time or order, every downstream comparison lies with it."""

    BODY = ("mark battery-start\n"
            "arms arms_up\n"
            "sleep 2\n"
            "walk 0.1 2\n"
            "mark battery-end\n")

    def _run(self, sess, tmp_path, **kw):
        script = sess.load_script(_write_battery(tmp_path, self.BODY))
        spy = _ScriptSpy()
        log = tmp_path / "log.jsonl"
        sleeps = []
        summary = sess.run_script(spy, None, script, str(log),
                                  sleep_fn=sleeps.append, **kw)
        import json
        events = [json.loads(l) for l in
                  log.read_text().strip().splitlines()]
        return spy, events, sleeps, summary

    def test_verbs_execute_in_file_order(self, sess, tmp_path):
        spy, events, sleeps, summary = self._run(sess, tmp_path)
        assert [c[0] for c in spy.calls] == ["set_arms", "walk"]
        assert spy.calls[0][1] == {"pose": "arms_up"}
        assert spy.calls[1][1] == {"vx": 0.1, "duration_s": 2.0}

    def test_the_log_is_meta_first_with_sha_and_real(self, sess, tmp_path):
        script = sess.load_script(_write_battery(tmp_path, self.BODY))
        spy, events, sleeps, summary = self._run(sess, tmp_path, real=True)
        assert events[0]["kind"] == "meta"
        assert events[0]["sha256"] == script["sha256"]
        assert events[0]["real"] is True

    def test_sleep_uses_the_injected_clock(self, sess, tmp_path):
        spy, events, sleeps, summary = self._run(sess, tmp_path)
        assert sleeps == [2.0]

    def test_every_event_carries_both_clocks(self, sess, tmp_path):
        spy, events, sleeps, summary = self._run(sess, tmp_path)
        assert all("t_wall" in e and "t_mono" in e for e in events)

    def test_marks_and_verbs_are_logged_in_order(self, sess, tmp_path):
        spy, events, sleeps, summary = self._run(sess, tmp_path)
        kinds = [e["kind"] for e in events]
        assert kinds == ["meta", "mark", "verb", "sleep", "verb",
                         "mark", "end"]
        assert events[1]["label"] == "battery-start"
        assert events[2]["verb"] == "arms"
        assert events[2]["ok"] is True

    def test_the_end_event_reports_completion(self, sess, tmp_path):
        spy, events, sleeps, summary = self._run(sess, tmp_path)
        assert events[-1]["kind"] == "end"
        assert events[-1]["ok"] is True
        assert summary["aborted"] is False


class TestTheScriptStopsAtTheFirstFailure:
    """A battery is an experiment: after a failed step, every later step
    measures a robot in an unknown pose. Fail-stop, log why, hands off."""

    BODY = ("mark battery-start\n"
            "arms arms_up\n"
            "walk 0.1 2\n"
            "mark battery-end\n")

    def test_a_failing_verb_aborts_the_rest(self, sess, tmp_path):
        import json
        script = sess.load_script(_write_battery(tmp_path, self.BODY))
        spy = _ScriptSpy(fail_on="set_arms")
        log = tmp_path / "log.jsonl"
        summary = sess.run_script(spy, None, script, str(log),
                                  sleep_fn=lambda s: None)
        assert summary["aborted"] is True
        assert "exploded" in summary["reason"]
        assert not any(c[0] == "walk" for c in spy.calls)
        events = [json.loads(l) for l in log.read_text().splitlines()]
        assert any(e["kind"] == "abort" for e in events)

    def test_a_tripped_thermal_watch_stops_the_battery(self, sess, tmp_path):
        script = sess.load_script(_write_battery(tmp_path, self.BODY))
        spy = _ScriptSpy()
        summary = sess.run_script(spy, None, script,
                                  str(tmp_path / "log.jsonl"),
                                  sleep_fn=lambda s: None,
                                  abort_flag={"stop": False, "tripped": True})
        assert summary["aborted"] is True
        assert "thermal" in summary["reason"]
        assert not any(c[0] == "set_arms" for c in spy.calls)

    def test_a_hot_reading_between_steps_stops_the_battery(
            self, sess, tmp_path):
        class HotTelemetry:
            def thermal(self):
                return {"available": True,
                        "hottest": [{"index": 22,
                                     "joint": "right_shoulder_pitch",
                                     "winding_c": 55, "driver_c": 95,
                                     "max_c": 95}]}
        script = sess.load_script(_write_battery(tmp_path, self.BODY))
        spy = _ScriptSpy()
        summary = sess.run_script(spy, HotTelemetry(), script,
                                  str(tmp_path / "log.jsonl"),
                                  sleep_fn=lambda s: None)
        assert summary["aborted"] is True
        assert not any(c[0] == "set_arms" for c in spy.calls)


class TestTheThermalActLimitCanOnlyBeLowered:
    """90 C is SONIC's own warn threshold; raising ours above it would mean
    trusting a guard nobody has ever seen fire. Lowering it is how the guard
    gets its FIRST real firing -- at a temperature that costs nothing."""

    def test_lowering_is_accepted(self, sess):
        assert sess.validate_thermal_act_c(55) == 55.0

    def test_raising_is_refused(self, sess):
        with pytest.raises(sess.SafetyRefusal):
            sess.validate_thermal_act_c(95)

    def test_a_limit_below_ambient_is_refused(self, sess):
        # A 20 C limit trips on a motor that was never powered.
        with pytest.raises(sess.SafetyRefusal):
            sess.validate_thermal_act_c(20)

    def test_nan_is_refused(self, sess):
        with pytest.raises(sess.SafetyRefusal):
            sess.validate_thermal_act_c(float("nan"))

    def test_the_watch_honors_a_lowered_limit(self, sess):
        spy = _ScriptSpy()
        thermal = {"available": True,
                   "hottest": [{"index": 22, "joint": "right_shoulder_pitch",
                                "winding_c": 40, "driver_c": 60, "max_c": 60}]}
        assert sess.thermal_watch_step(spy, thermal, act_c=55.0) is True
        assert ("stop_all", {}) in spy.calls

    def test_the_default_limit_is_unchanged_by_the_plumbing(self, sess):
        spy = _ScriptSpy()
        thermal = {"available": True,
                   "hottest": [{"index": 22, "joint": "right_shoulder_pitch",
                                "winding_c": 40, "driver_c": 60, "max_c": 60}]}
        assert sess.thermal_watch_step(spy, thermal) is False
        assert spy.calls == []


class TestAScriptDemandsTheEstopExactlyWhenATaskWould:
    def test_a_real_script_demands_the_assertion(self, sess):
        assert sess.needs_estop(check=False, dry=False, manual=None,
                                task=None, real=True, script=True) is True

    def test_a_simulated_script_does_not(self, sess):
        assert sess.needs_estop(check=False, dry=False, manual=None,
                                task=None, real=False, script=True) is False


class TestTheWatchEscalatesWhenItsFirstActionRemovesNothing:
    """2026-08-20, live: the watch fired at 90 C, ran stop_all + arms
    neutral -- and right_ankle_roll kept climbing to 93 C, because the heat
    source was the LEGS: the balance policy pressing commanded stance into
    the floor at 24-29 N.m while the gantry held the body. The first
    intervention removes an ARM load; when act persists, the only
    load-removing act left is damp. The robot hangs on the gantry, which the
    TWO-KEY tier mandates for exactly this."""

    def test_it_does_not_escalate_before_the_grace_period(self, sess):
        esc = sess.ThermalEscalation(after_s=10.0)
        assert esc.step(True, 100.0) is False
        assert esc.step(True, 105.0) is False

    def test_sustained_act_escalates_exactly_once(self, sess):
        esc = sess.ThermalEscalation(after_s=10.0)
        esc.step(True, 100.0)
        assert esc.step(True, 110.1) is True
        assert esc.step(True, 120.0) is False

    def test_a_clear_reading_resets_the_clock(self, sess):
        esc = sess.ThermalEscalation(after_s=10.0)
        esc.step(True, 100.0)
        assert esc.step(False, 105.0) is False
        assert esc.step(True, 109.0) is False   # new act window starts here
        assert esc.step(True, 118.0) is False   # only 9 s into it
        assert esc.step(True, 119.5) is True

    def test_the_watch_thread_actually_damps(self, sess):
        import threading
        import time as _t

        class Spy:
            def __init__(self):
                self.calls = []

            def execute(self, name, params):
                self.calls.append((name, params))
                return "ok"

            def stop_all(self):
                self.calls.append(("stop_all", {}))
                return "ok"

        class HotTelemetry:
            def thermal(self):
                return {"available": True,
                        "hottest": [{"index": 11, "joint": "right_ankle_roll",
                                     "winding_c": 45, "driver_c": 95,
                                     "max_c": 95}]}

        spy = Spy()
        flag = {"stop": False, "tripped": False}
        th = threading.Thread(
            target=sess.thermal_watch,
            args=(spy, HotTelemetry(), flag),
            kwargs={"period_s": 0.02, "escalate_after_s": 0.15},
            daemon=True)
        th.start()
        deadline = _t.monotonic() + 3.0
        while _t.monotonic() < deadline:
            if any(c[0] == "emergency_stop" for c in spy.calls):
                break
            _t.sleep(0.02)
        flag["stop"] = True
        th.join(timeout=2.0)
        assert any(c[0] == "emergency_stop" for c in spy.calls), \
            "sustained act never escalated to damp"
        assert spy.calls.count(("emergency_stop", {})) == 1
