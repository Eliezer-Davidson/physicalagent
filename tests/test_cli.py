"""Tests for the CLI argument parser (physicalagent/cli.py).

Only the parser is unit-tested; the command bodies do live I/O and are
exercised on hardware. This guards the argument contract.
"""
import pytest

from physicalagent.cli import build_parser


class TestCliParser:
    def test_doctor_subcommand(self):
        args = build_parser().parse_args(["doctor"])
        assert args.cmd == "doctor"
        assert callable(args.func)

    def test_run_requires_task(self):
        with pytest.raises(SystemExit):
            build_parser().parse_args(["run"])

    def test_run_with_task_and_defaults(self):
        args = build_parser().parse_args(["run", "look around"])
        assert args.task == "look around"
        assert args.dry is False
        assert args.force is False
        # raised from 12: a real navigation task was cut off mid-goal at 12 hops
        assert args.max_hops == 60

    def test_run_flags(self):
        args = build_parser().parse_args(["run", "go", "--dry", "--force", "--max-hops", "4"])
        assert args.dry is True
        assert args.force is True
        assert args.max_hops == 4

    def test_unknown_command_errors(self):
        with pytest.raises(SystemExit):
            build_parser().parse_args(["fly"])


class TestCliWiresTheActionWhitelist:
    """REGRESSION: the console built SafetyGuard() with no action_whitelist, so
    it defaulted to () and refused EVERY action group — 'sit', 'stand',
    everything. Two gates sit in series (the adapter's list, then the guard's);
    opening only the adapter's left do_action completely dead in production.
    """

    def test_guard_whitelist_is_not_empty(self):
        from physicalagent.cli import build_guard
        assert build_guard().action_whitelist, "empty whitelist refuses every action"

    def test_guard_allows_the_vendor_actions(self):
        from physicalagent.cli import build_guard
        guard = build_guard()
        for name in ("sit", "stand", "2_legs_stand", "lie_down"):
            assert name in guard.action_whitelist

    def test_adapter_and_guard_agree_so_do_action_actually_runs(self):
        """End-to-end through the real wiring, offline."""
        from physicalagent.cli import build_guard
        from physicalagent.robots.rospug import RospugAdapter
        from physicalagent.robots.transport import FakeTransport, ScriptedClock
        t = FakeTransport(telemetry={"batt": 11800, "dead": "ALIVE"},
                          services={"/pug_control/run_action_group": {"success": True}})
        a = RospugAdapter(t, build_guard(), clock=ScriptedClock(),
                          frame_fetcher=lambda: b"x")
        res = a.execute("do_action", {"name": "sit"})
        assert res.ok, res.error


class TestRobotSelection:
    """Precedence is --robot > $PHYSICALAGENT_ROBOT > "rospug".

    These used to set the env var to "rospug" and assert the result was
    "rospug" -- which is true whichever source won, because only one profile
    is registered. Every one of them passed with the precedence reversed, or
    with one source ignored entirely.

    A second profile is not needed to fix that, only a second NAME: an
    unregistered name is a perfectly good discriminator, because reaching it
    raises and reaching the other source does not. Each test below therefore
    puts a name that CANNOT resolve in the source that must lose, so the
    assertion can only hold if the intended source won.
    """

    def test_default_is_the_rospug_when_no_flag_and_no_env(self, monkeypatch):
        from physicalagent.cli import resolve_profile
        monkeypatch.delenv("PHYSICALAGENT_ROBOT", raising=False)
        assert resolve_profile(None).name == "rospug"

    def test_the_env_var_beats_the_default(self, monkeypatch):
        """Inverted: an unresolvable env value must be USED (and raise), not
        quietly passed over in favour of the rospug default."""
        from physicalagent.cli import resolve_profile
        from physicalagent.profiles import UnknownRobotError
        monkeypatch.setenv("PHYSICALAGENT_ROBOT", "nosuchrobot")
        with pytest.raises(UnknownRobotError):
            resolve_profile(None)

    def test_explicit_flag_beats_the_env_var(self, monkeypatch):
        """The env names a robot that does not exist; the flag names one that
        does. Only flag-wins can return rospug here -- env-wins would raise."""
        from physicalagent.cli import resolve_profile
        monkeypatch.setenv("PHYSICALAGENT_ROBOT", "nosuchrobot")
        assert resolve_profile("rospug").name == "rospug"

    def test_env_var_is_used_when_no_flag(self, monkeypatch):
        """And the mirror image: env names the real robot, so no-flag resolves
        it. Paired with test_the_env_var_beats_the_default this pins that the
        env var is genuinely read rather than coincidentally agreeing with the
        default."""
        from physicalagent.cli import resolve_profile
        monkeypatch.setenv("PHYSICALAGENT_ROBOT", "ROSPug")   # also case-folded
        assert resolve_profile(None).name == "rospug"

    def test_an_empty_env_var_falls_through_to_the_default(self, monkeypatch):
        from physicalagent.cli import resolve_profile
        monkeypatch.setenv("PHYSICALAGENT_ROBOT", "")
        assert resolve_profile(None).name == "rospug"

    def test_unknown_robot_raises_naming_the_alternatives(self, monkeypatch):
        """Discriminating on the MESSAGE: it must list what is available, so a
        typo at the CLI tells you the answer instead of only that you were
        wrong."""
        from physicalagent.cli import resolve_profile
        monkeypatch.delenv("PHYSICALAGENT_ROBOT", raising=False)
        with pytest.raises(KeyError) as excinfo:
            resolve_profile("nosuchrobot")
        assert "nosuchrobot" in excinfo.value.args[0]
        assert "available:" in excinfo.value.args[0]
        for name in ("minihexa", "rospug", "tonypi"):
            assert name in excinfo.value.args[0]

    def test_every_subcommand_accepts_robot(self):
        from physicalagent.cli import build_parser
        p = build_parser()
        for argv in (["doctor"], ["status"], ["run", "x"], ["shell"]):
            assert p.parse_args(argv + ["--robot", "rospug"]).robot == "rospug"

    def test_robot_defaults_to_none_when_not_given(self):
        from physicalagent.cli import build_parser
        assert build_parser().parse_args(["doctor"]).robot is None


class TestRobotWrapperAcceptsALeadingFlag:
    """REGRESSION: the design doc, the plan and the CLI's own --help all show
    `./robot --robot tonypi`, but argparse requires the subcommand first, so it
    died with "invalid choice: 'rospug'". The wrapper's `${@:-shell}` default
    only applied with ZERO arguments, so the documented form was unreachable.

    These shell out to the real wrapper -- the bug lived in the wrapper, not in
    the parser, so testing build_parser() alone would never have caught it.
    Both invocations below exit before any network I/O.
    """

    WRAPPER = __import__("pathlib").Path(__file__).resolve().parent.parent / "robot"

    def _run(self, *argv):
        """Run the real wrapper under the interpreter running this suite.

        PHYSICALAGENT_PYTHON exists so this test does not depend on a `.venv`
        having been created in the checkout -- the wrapper's job under test is
        argument shaping, not interpreter discovery."""
        import os
        import subprocess
        import sys
        env = dict(os.environ, PHYSICALAGENT_PYTHON=sys.executable)
        return subprocess.run([str(self.WRAPPER), *argv], capture_output=True,
                              text=True, timeout=60, env=env)

    def test_bare_robot_flag_reaches_profile_resolution(self):
        """It must fail on the ROBOT NAME, not on the argument parser."""
        r = self._run("--robot", "nosuchrobot")
        assert r.returncode == 2
        assert "unknown robot 'nosuchrobot'" in r.stderr
        assert "invalid choice" not in r.stderr

    def test_subcommand_first_still_works(self):
        r = self._run("doctor", "--robot", "nosuchrobot")
        assert r.returncode == 2
        assert "unknown robot 'nosuchrobot'" in r.stderr

    def test_argument_errors_do_not_need_an_api_key(self):
        """The README promises the whole suite runs with no key in the
        environment. This is the one test that shells out far enough to break
        that promise, so it is the one that pins it: with GEMINI_API_KEY
        removed, an unknown robot must still be reported as an unknown robot."""
        import os
        import subprocess
        import sys
        env = dict(os.environ, PHYSICALAGENT_PYTHON=sys.executable)
        env.pop("GEMINI_API_KEY", None)
        r = subprocess.run([str(self.WRAPPER), "--robot", "nosuchrobot"],
                           capture_output=True, text=True, timeout=60, env=env)
        assert r.returncode == 2
        assert "unknown robot 'nosuchrobot'" in r.stderr
        assert "GEMINI_API_KEY" not in r.stderr


class TestRobotWrapperExemptsHelpFromTheShellInsertion:
    """DELIBERATE BEHAVIOUR CHANGE (N3): this replaces the old
    test_a_leading_flag_implies_the_shell_subcommand, which asserted that
    `./robot --help` printed `"physicalagent shell"` in stdout -- i.e. it
    pinned --help being swallowed into the shell subcommand as correct.

    That was the wrong behaviour to pin: -h/--help used to be swept up by
    the same "insert shell before a leading flag" rule as every other flag
    (see TestRobotWrapperAcceptsALeadingFlag above, e.g. --robot), so
    `./robot --help` became `shell --help` and printed the shell
    subparser's usage instead of the top-level `usage: physicalagent [-h]
    {doctor,status,probe,run,shell} ...`. An operator who forgot the
    subcommand names could not discover them this way.

    -h/--help are now exempted from the insertion so they reach the
    top-level parser; every other leading flag (e.g. --robot) still gets
    `shell` inserted, per TestRobotWrapperAcceptsALeadingFlag.
    """

    WRAPPER = __import__("pathlib").Path(__file__).resolve().parent.parent / "robot"

    def _run(self, *argv):
        """Run the real wrapper under the interpreter running this suite.

        PHYSICALAGENT_PYTHON exists so this test does not depend on a `.venv`
        having been created in the checkout -- the wrapper's job under test is
        argument shaping, not interpreter discovery."""
        import os
        import subprocess
        import sys
        env = dict(os.environ, PHYSICALAGENT_PYTHON=sys.executable)
        return subprocess.run([str(self.WRAPPER), *argv], capture_output=True,
                              text=True, timeout=60, env=env)

    def test_help_shows_the_top_level_usage_not_the_shell_subparsers(self):
        r = self._run("--help")
        assert r.returncode == 0, r.stderr
        assert "invalid choice" not in r.stderr
        assert ("usage: physicalagent [-h] {doctor,status,probe,run,shell}"
                in r.stdout)
        assert "physicalagent shell" not in r.stdout, \
            "--help must not be swallowed into the shell subcommand"

    def test_short_h_also_reaches_the_top_level_parser(self):
        r = self._run("-h")
        assert r.returncode == 0, r.stderr
        assert ("usage: physicalagent [-h] {doctor,status,probe,run,shell}"
                in r.stdout)


class TestGuardComesFromTheProfile:
    def test_build_guard_uses_the_profile_safety(self):
        from physicalagent.cli import build_guard
        from physicalagent.profiles import get_profile
        g = build_guard(get_profile("rospug"))
        assert g.profile is get_profile("rospug").safety

    def test_build_guard_keeps_the_action_whitelist_populated(self):
        """REGRESSION (session 4): an empty whitelist silently killed do_action."""
        from physicalagent.cli import build_guard
        from physicalagent.profiles import get_profile
        assert len(build_guard(get_profile("rospug")).action_whitelist) > 0


class TestUnknownRobotIsHandledCleanly:
    """REGRESSION: --robot rosbug used to either (a) open a real transport,
    subscribe four topics and block for 3s before failing (status/run/shell),
    or (b) blow up with a raw ~10-line KeyError traceback (doctor and
    everything else, via main()). A typo'd robot name is the expected
    failure mode for this flag, so it must fail fast and print one clean
    line instead.
    """

    def test_main_returns_2_with_a_clean_single_line_message(self, capsys):
        from physicalagent.cli import main
        rc = main(["doctor", "--robot", "nosuchrobot"])
        assert rc == 2
        err = capsys.readouterr().err
        assert err.strip().count("\n") == 0, f"expected one clean line, got: {err!r}"
        assert "Traceback" not in err

    def test_main_error_message_names_the_available_robots(self, capsys):
        from physicalagent.cli import main
        rc = main(["doctor", "--robot", "nosuchrobot"])
        assert rc == 2
        err = capsys.readouterr().err
        assert "rospug" in err

    def test_status_resolves_the_robot_before_connecting(self, monkeypatch, capsys):
        from physicalagent import cli

        def _boom():
            raise AssertionError("should not connect")
        monkeypatch.setattr(cli, "_connect", _boom)

        rc = cli.main(["status", "--robot", "nosuchrobot"])
        assert rc == 2
        assert "rospug" in capsys.readouterr().err

    def test_run_resolves_the_robot_before_connecting(self, monkeypatch, capsys):
        from physicalagent import cli

        def _boom():
            raise AssertionError("should not connect")
        monkeypatch.setattr(cli, "_connect", _boom)
        monkeypatch.setenv("GEMINI_API_KEY", "test-key")

        rc = cli.main(["run", "look around", "--robot", "nosuchrobot"])
        assert rc == 2
        assert "rospug" in capsys.readouterr().err

    def test_shell_resolves_the_robot_before_connecting(self, monkeypatch, capsys):
        from physicalagent import cli

        def _boom():
            raise AssertionError("should not connect")
        monkeypatch.setattr(cli, "_connect", _boom)
        monkeypatch.setenv("GEMINI_API_KEY", "test-key")

        rc = cli.main(["shell", "--robot", "nosuchrobot"])
        assert rc == 2
        assert "rospug" in capsys.readouterr().err


class TestBareKeyErrorDuringRealWorkIsNotSwallowed:
    """REGRESSION: `except KeyError` in main() used to wrap the entire
    args.func(args) call, not just profile resolution. So any bare KeyError
    raised while a command was doing real work -- e.g. doctor.py indexing
    scan['ranges'] on a malformed LaserScan message without checking the key
    exists first -- got caught, printed as a one-line "unknown robot"
    message, and its traceback discarded. That's concretely reachable: a bad
    telemetry message from a live robot would be silently mislabelled as a
    typo'd --robot flag, with no traceback to debug the real bug from.

    main() now catches only UnknownRobotError (a KeyError subclass raised
    solely by profile lookup), so a bare KeyError from command work must
    propagate instead of being swallowed into exit code 2.
    """

    def test_keyerror_from_doctor_diagnose_propagates_not_swallowed(self, monkeypatch):
        from physicalagent import cli, doctor

        def _raise_keyerror(*args, **kwargs):
            # Mirrors the real bug: doctor.py's `scan['ranges']` on a scan
            # message missing that key.
            raise KeyError("ranges")

        monkeypatch.setattr(cli, "_connect", lambda: object())
        monkeypatch.setattr(doctor, "diagnose", _raise_keyerror)

        with pytest.raises(KeyError, match="ranges"):
            cli.main(["doctor"])

    def test_keyerror_from_command_body_is_not_printed_as_unknown_robot(self, monkeypatch, capsys):
        """Belt-and-braces: even if some caller swallows the propagated
        KeyError, its text must never masquerade as an unknown-robot error."""
        from physicalagent import cli, doctor

        def _raise_keyerror(*args, **kwargs):
            raise KeyError("ranges")

        monkeypatch.setattr(cli, "_connect", lambda: object())
        monkeypatch.setattr(doctor, "diagnose", _raise_keyerror)

        try:
            cli.main(["doctor"])
        except KeyError:
            pass
        err = capsys.readouterr().err
        assert "unknown robot" not in err


class TestDiagnoseReceivesTheResolvedProfile:
    """REGRESSION GUARD: diagnose() gained a `profile` parameter so a robot on
    a different pack is not judged against the ROSPug's floor. cmd_doctor
    passes profile=profile.safety, but cmd_run and cmd_shell called
    diagnose(t, _frame_fetcher(), settle_s=0) and silently fell back to the
    ROSPUG_SAFETY default. Concretely: register a 2S robot, and `./robot
    doctor --robot x` correctly calls its 7.4V pack healthy, while `./robot
    run "..." --robot x` judges the same pack against the ROSPug's 11100 mV
    floor and refuses to start. All three commands already have the resolved
    `profile` in scope; all three must pass profile.safety to diagnose.
    """

    class _RefusingReport:
        """A Report stand-in with ok=False, so each command under test stops
        right after printing the summary -- before starting any background
        heartbeat thread or touching the Brain/Gemini path."""
        ok = False

        def summary(self):
            return "fake preflight report"

    def _fake_connect(self, monkeypatch, cli_module):
        from physicalagent.robots.transport import FakeTransport
        t = FakeTransport(telemetry={"batt": 11800, "dead": "ALIVE"})
        t.terminate = lambda: None
        monkeypatch.setattr(cli_module, "_connect", lambda: t)
        return t

    def _capture_diagnose(self, monkeypatch):
        from physicalagent import doctor
        captured = {}

        def _fake_diagnose(*args, **kwargs):
            captured["args"] = args
            captured["kwargs"] = kwargs
            return self._RefusingReport()

        monkeypatch.setattr(doctor, "diagnose", _fake_diagnose)
        return captured

    def test_doctor_passes_the_resolved_profile_safety(self, monkeypatch):
        from physicalagent import cli
        from physicalagent.profiles import get_profile

        self._fake_connect(monkeypatch, cli)
        captured = self._capture_diagnose(monkeypatch)

        cli.main(["doctor", "--robot", "rospug"])

        assert captured["kwargs"].get("profile") is get_profile("rospug").safety

    def test_run_passes_the_resolved_profile_safety(self, monkeypatch):
        from physicalagent import cli
        from physicalagent.profiles import get_profile

        monkeypatch.setenv("GEMINI_API_KEY", "test-key")
        monkeypatch.setattr("time.sleep", lambda *a, **k: None)
        self._fake_connect(monkeypatch, cli)
        captured = self._capture_diagnose(monkeypatch)

        rc = cli.main(["run", "look around", "--robot", "rospug"])

        assert rc == 1  # stopped by the fake refusing report, as designed
        assert captured["kwargs"].get("profile") is get_profile("rospug").safety

    def test_shell_passes_the_resolved_profile_safety(self, monkeypatch):
        from physicalagent import cli
        from physicalagent.profiles import get_profile

        monkeypatch.setenv("GEMINI_API_KEY", "test-key")
        monkeypatch.setattr("time.sleep", lambda *a, **k: None)
        self._fake_connect(monkeypatch, cli)
        captured = self._capture_diagnose(monkeypatch)

        rc = cli.main(["shell", "--robot", "rospug"])

        assert rc == 1  # stopped by the fake refusing report, as designed
        assert captured["kwargs"].get("profile") is get_profile("rospug").safety


class TestProbeWiresRosCallIntoRunProbe:
    """REGRESSION GUARD: cmd_probe must actually pass its ros_call through to
    run_probe. The original defect was that the CALLER never called it --
    every capture recorded {"skipped": "no ROS link"} even when rosbridge was
    up. That bug was invisible to the suite: every existing test either
    exercises run_probe(ros_call=...) directly (the callee, never the broken
    part) or stubs cli._rosbridge_link to return (None, None), and the
    existing probe-command tests' _mock_run_probe accepts **kwargs, so it
    silently absorbs a missing (or extra) ros_call instead of rejecting it.

    This test pins the CALLER side: a sentinel from _rosbridge_link must
    arrive at run_probe as its ros_call keyword, captured with an EXPLICIT
    parameter (no **kwargs) so a dropped or renamed argument surfaces as a
    wrong/missing value rather than being swallowed silently.
    """

    def test_ros_call_from_rosbridge_link_reaches_run_probe(self, tmp_path, monkeypatch):
        import json
        from physicalagent import cli, probe

        def _sentinel_ros_call(*a, **kw):
            raise AssertionError("sentinel should never actually be invoked")

        monkeypatch.setattr(cli, "_rosbridge_link",
                            lambda host, port=9090: (_sentinel_ros_call, None))

        captured = {}

        def _capturing_run_probe(host, robot, base_dir, stamp, lsusb_text=None,
                                 serial=None, port_probe=None, ros_call=None,
                                 ros_samples=None):
            captured["ros_call"] = ros_call
            manifest_path = tmp_path / "manifest.json"
            manifest_path.write_text(json.dumps({"conclusions": []}))
            return tmp_path

        monkeypatch.setattr(probe, "run_probe", _capturing_run_probe)

        rc = cli.main(["probe", "--robot", "test_robot"])

        assert rc == 0
        assert captured["ros_call"] is _sentinel_ros_call
        assert captured["ros_call"] is not None


class TestProbeCommandPrintsConclusionsAsReadableSentences:
    """REGRESSION: probe was filtering manifest.json lines looking for
    'conclusion' in the text. Because the JSON is indented, this matched
    only the key line '"conclusions": [' and never the actual conclusion
    sentences inside the array. The fix parses JSON and prints each
    conclusion as a readable sentence, not raw JSON.
    """

    def test_probe_prints_conclusion_sentences_not_json_key(self, tmp_path, monkeypatch, capsys):
        """Probe should print the actual conclusion sentences from the list."""
        import json
        from physicalagent import cli, probe

        # Create a temp manifest with known conclusions
        manifest_data = {
            "robot": "test_robot",
            "host": "10.0.0.1",
            "stamp": "2026-07-22_001122",
            "usb": [],
            "serial_ports": [],
            "ports": [],
            "conclusions": [
                "rosbridge is up — this is a ROS robot; dump the graph next",
                "SSH is open — vendor source can be copied off the robot"
            ],
        }
        manifest_path = tmp_path / "manifest.json"
        manifest_path.write_text(json.dumps(manifest_data, indent=2))

        # Mock run_probe to return our temp directory
        def _mock_run_probe(host, robot, base_dir, stamp, lsusb_text=None,
                            serial=None, port_probe=None, **kwargs):
            return tmp_path

        monkeypatch.setattr(probe, "run_probe", _mock_run_probe)
        # No network from a unit test: cmd_probe now scans for rosbridge and
        # opens a real link when it answers.
        monkeypatch.setattr(cli, "_rosbridge_link", lambda host, port=9090: (None, None))

        # Run the probe command
        rc = cli.main(["probe", "--robot", "test_robot"])
        assert rc == 0

        # Verify the actual conclusion sentences appear in stdout
        out = capsys.readouterr().out
        assert "rosbridge is up — this is a ROS robot; dump the graph next" in out
        assert "SSH is open — vendor source can be copied off the robot" in out

        # Verify the JSON key line does NOT appear
        # (the fix parses JSON and prints sentences, not lines containing "conclusion")
        assert '"conclusions"' not in out

    def test_probe_prints_clear_message_when_no_conclusions(self, tmp_path, monkeypatch, capsys):
        """When there are no conclusions, print a clear message."""
        import json
        from physicalagent import cli, probe

        # Create a temp manifest with no conclusions
        manifest_data = {
            "robot": "test_robot",
            "host": "10.0.0.1",
            "stamp": "2026-07-22_001122",
            "usb": [],
            "serial_ports": [],
            "ports": [],
            "conclusions": [],
        }
        manifest_path = tmp_path / "manifest.json"
        manifest_path.write_text(json.dumps(manifest_data, indent=2))

        # Mock run_probe to return our temp directory
        def _mock_run_probe(host, robot, base_dir, stamp, lsusb_text=None,
                            serial=None, port_probe=None, **kwargs):
            return tmp_path

        monkeypatch.setattr(probe, "run_probe", _mock_run_probe)
        # No network from a unit test: cmd_probe now scans for rosbridge and
        # opens a real link when it answers.
        monkeypatch.setattr(cli, "_rosbridge_link", lambda host, port=9090: (None, None))

        # Run the probe command
        rc = cli.main(["probe", "--robot", "test_robot"])
        assert rc == 0

        # Verify the clear message appears
        out = capsys.readouterr().out
        assert "(no conclusions drawn)" in out
