"""Patch 05 made the Isaac backend a real SONIC backend -- these tests keep it
one. The failure they guard against is not hypothetical: the vendor's
wholebody mode drove the legs from its own policy while SONIC got arms only,
and every Isaac result taken between 2026-08-07 and 08-12 had to be voided. Everything here
reads repo files (the patch, the launcher, the probe) -- no simulator, no
vendor checkout, same stdlib venv as the rest of the suite.
"""
from __future__ import annotations

import ast
import os
import pathlib
import re
import subprocess
import tempfile

import pytest

REPO = pathlib.Path(__file__).resolve().parents[1]
PATCH = REPO / "third_party" / "unitree-sim-isaaclab-local" / "05-sonic-full-body-routing.patch"
LAUNCHER = REPO / "scripts" / "sonic_isaac_live.sh"
PROBE = REPO / "scripts" / "sonic_tracking_probe.py"
# The vendor checkout, from the environment only (physicalagent.paths reads the
# same variable). None on a machine that has no checkout, and the live-tree
# tests below skip rather than guess a location.
_VENDOR_ENV = os.environ.get("UNITREE_SIM_ROOT")
VENDOR = pathlib.Path(_VENDOR_ENV).expanduser() if _VENDOR_ENV else None

# G1 29-DOF hardware order (unitree_hg LowCmd_ motor index), legs + waist.
# Pinned against g1_deploy_onnx_ref/include/policy_parameters.hpp:143-176 and
# the reference MJCF. The arms (hw 15..28) come from the provider's own
# arm_joint_names -- a list the patch text does not contain, so its order is
# pinned against the LIVE vendor file in TestLiveVendorTreeMatchesThePatch.
HW_LEGS_WAIST = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
]


class TestPatch05Content:
    def test_patch_exists(self):
        assert PATCH.exists(), "05-sonic-full-body-routing.patch is missing"

    def test_patch_touches_only_the_wholebody_provider(self):
        """A patch that quietly grows a second file is a different change than
        the one that was reviewed."""
        touched = set(re.findall(r"^\+\+\+ b/(\S+)", PATCH.read_text(), re.M))
        assert touched == {"action_provider/action_provider_wh_dds.py"}, touched

    def test_joint_order_is_the_hw_order(self):
        """The single deadliest failure mode of full-body routing is a joint
        permutation: every position lands on a plausible joint and the robot
        'runs' -- wrongly. The 15 literal names (legs+waist) must appear in
        the patch's ADDED lines in exactly the hardware order, and the arms
        must come from arm_joint_names (already hw-ordered)."""
        added = "\n".join(ln[1:] for ln in PATCH.read_text().splitlines()
                          if ln.startswith("+"))
        positions = []
        for name in HW_LEGS_WAIST:
            pos = added.find(f'"{name}"')
            assert pos != -1, f"{name} not added by the patch"
            positions.append(pos)
        assert positions == sorted(positions), (
            "legs/waist joint names are not in hardware order in the patch")
        assert "list(self.arm_joint_names)" in added

    def test_message_gains_are_written_and_guarded(self):
        added = PATCH.read_text()
        assert "write_joint_stiffness_to_sim" in added
        assert "write_joint_damping_to_sim" in added
        # A zeroed lowcmd must never strip the drives.
        assert "_gains_acceptable" in added

    def test_rejected_gains_also_reject_the_positions(self):
        """The damp/e-stop message is q=0, kp=0, kd=8: 'no position
        authority'. Tracking its q at the previous stiff gains would lunge
        the robot to full extension -- the gains gate must sit in the
        POSITION acceptance condition, not only in the gain writer."""
        added = "\n".join(ln[1:] for ln in PATCH.read_text().splitlines()
                          if ln.startswith("+"))
        assert re.search(r"if complete and fresh and self\._gains_acceptable",
                         added)
        # Refused/stale/absent all hold the last ACCEPTED targets.
        assert "_fb_last_targets" in added

    def test_stale_commands_are_not_obeyed_forever(self):
        """The shm slot replays the last command indefinitely; a SIGKILLed
        controller must not read as a live one."""
        added = PATCH.read_text()
        assert "_timestamp" in added
        assert "stale command" in added

    def test_flag_gates_everything_and_announces_itself(self):
        added = PATCH.read_text()
        assert "SONIC_WB_FULL_BODY" in added
        assert "SONIC FULL-BODY ROUTING ENABLED" in added
        # The old behavior must stay reachable for A/B, and must confess.
        assert "ARMS ONLY" in added

    def test_local_policy_is_not_loaded_in_full_body_mode(self):
        assert "None if self._sonic_full_body else self.load_policy" in \
            PATCH.read_text()


class TestLauncherEnforcesFullBody:
    def test_full_body_is_the_default(self):
        text = LAUNCHER.read_text()
        assert 'WB_FULL_BODY="${SONIC_WB_FULL_BODY:-1}"' in text

    def test_flag_reaches_the_sim_process(self):
        assert re.search(r'SONIC_WB_FULL_BODY="\$WB_FULL_BODY"',
                         LAUNCHER.read_text())

    def test_missing_banner_is_fatal_not_cosmetic(self):
        """If the vendor tree lacks patch 05 the sim silently runs the
        arms-only hybrid -- the exact lie of 2026-08-07. The launcher must
        refuse, not warn -- and the exit must be INSIDE the banner grep's
        own failure group, not merely somewhere later in the file (the
        first version of this test matched any later `exit 1` and passed
        with the gate neutered to a warning)."""
        text = LAUNCHER.read_text()
        gate = re.search(
            r'grep -q "SONIC FULL-BODY ROUTING ENABLED"[^|]*\|\|\s*\{([^}]*)\}',
            text, re.S)
        assert gate, "launcher has no failure group on the banner grep"
        assert "exit 1" in gate.group(1), (
            "the banner-missing group does not exit; a missing patch would "
            "only warn")

    def test_gate_is_scoped_to_wholebody_tasks(self):
        """Non-Wholebody tasks use a different provider and legitimately
        never print the banner; gating them would misdiagnose a fine launch
        as a missing patch."""
        assert re.search(r"case\s+\"\$TASK\"\s+in\s*\n\*Wholebody\*\)",
                         LAUNCHER.read_text())


class TestTrackingProbe:
    def test_probe_exists_and_judges_by_correlation(self):
        text = PROBE.read_text()
        assert "pearson" in text
        assert "kneel_two_legs" in text

    def test_probe_subscribers_cannot_be_garbage_collected(self):
        """The M0 probe's unreferenced ChannelSubscriber was GC'd ~4.8 s in,
        froze its counter, and minted the false '79 Hz lowcmd' figure. Every
        subscriber must be bound to self -- a LOCAL variable is exactly as
        collectable as a bare temporary, so every construction site is
        checked, not just statement-position ones."""
        text = PROBE.read_text()
        sites = [ln for ln in text.splitlines() if "ChannelSubscriber(" in ln
                 and "import" not in ln]
        assert sites, "the probe no longer subscribes at all?"
        for ln in sites:
            assert re.match(r"\s*self\._\w+_sub = ChannelSubscriber\(", ln), (
                f"subscriber not bound to self (GC-able): {ln.strip()!r}")

    def test_probe_refuses_to_judge_an_unswept_command(self):
        """corr on a flat command is noise; the probe must return
        'inconclusive' when the kneel never moved the knees, not a verdict."""
        text = PROBE.read_text()
        assert "CMD_SWEEP_MIN_RAD" in text
        assert "inconclusive" in text

    def test_probe_crash_is_not_a_verdict(self):
        """Exit 1 is reserved for an evidence-backed 'inconclusive'; a crash
        before measuring must exit 2, or a broken bring-up reads as a
        planner problem and gets retried forever."""
        text = PROBE.read_text()
        assert "except KeyboardInterrupt" in text
        assert re.search(r"except Exception:.*?sys\.exit\(2\)", text, re.S)


# HW arm order (lowcmd 15..28), pinned against policy_parameters.hpp:143-176.
HW_ARMS = [
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
    "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]


@pytest.mark.skipif(VENDOR is None or not VENDOR.exists(),
                    reason="UNITREE_SIM_ROOT unset or checkout not on this machine")
class TestLiveVendorTreeMatchesThePatch:
    """Text tests cannot see the live tree drift away from the patch. These
    can -- they run only where the checkout exists (this laptop), and they
    are the difference between 'the patch file says' and 'the sim will run'."""

    def test_patch_stack_reconstructs_the_live_provider_byte_for_byte(self):
        """BASE_COMMIT + 05 + 06 must equal the file the simulator actually
        imports. Reverse-applying a single patch stopped working once two
        patches shared the file (they overlap), and that is precisely when a
        drift guard earns its keep -- so reconstruct the whole stack instead.
        A live-tree edit past the patches fails HERE, not at launch."""
        base = (REPO / "third_party" / "unitree-sim-isaaclab-local"
                / "BASE_COMMIT").read_text().strip()
        rel = "action_provider/action_provider_wh_dds.py"
        with tempfile.TemporaryDirectory() as td:
            work = pathlib.Path(td)
            (work / "action_provider").mkdir()
            src = subprocess.run(
                ["git", "-C", str(VENDOR), "show", f"{base}:{rel}"],
                capture_output=True, text=True, timeout=30)
            assert src.returncode == 0, src.stderr
            (work / rel).write_text(src.stdout)
            for patch in (PATCH, PATCH06, PATCH07):
                r = subprocess.run(["git", "apply", str(patch)], cwd=work,
                                   capture_output=True, text=True, timeout=30)
                assert r.returncode == 0, (
                    f"{patch.name} does not apply onto BASE_COMMIT:\n{r.stderr}")
            assert (work / rel).read_text() == (VENDOR / rel).read_text(), (
                "the live vendor provider is NOT BASE + 05 + 06 -- either the "
                "tree drifted or a patch is stale")

    def test_live_provider_announces_full_body(self):
        live = (VENDOR / "action_provider" /
                "action_provider_wh_dds.py").read_text()
        assert "SONIC FULL-BODY ROUTING ENABLED" in live

    def test_live_arm_joint_order_is_the_hw_order(self):
        """The patch reuses the provider's arm_joint_names for hw 15..28; the
        patch text cannot pin a list it does not contain, so pin the LIVE
        file's list -- a vendor-side reorder would silently permute SONIC's
        arm targets."""
        live = (VENDOR / "action_provider" /
                "action_provider_wh_dds.py").read_text()
        tree = ast.parse(live)
        found = None
        for node in ast.walk(tree):
            if (isinstance(node, ast.Assign)
                    and any(isinstance(t, ast.Attribute)
                            and t.attr == "arm_joint_names"
                            for t in node.targets)):
                found = ast.literal_eval(node.value)
                break
        assert found == HW_ARMS, (
            f"live arm_joint_names is not the hw order: {found}")


ROUTING = REPO / "scripts" / "isaac_routing_probe.py"


class TestOpenLoopRoutingProbe:
    """The open-loop probe is what actually proved patch 05 (2026-08-12): the
    closed-loop tracking probe returned LEGS NOT WIRED for a correctly wired
    path, because it measured correlation inside a diverging control loop.
    These pin the properties that make the open-loop version conclusive."""

    def test_probe_exists(self):
        assert ROUTING.exists()

    def test_signature_magnitudes_are_all_distinct(self):
        """A permuted joint order is only detectable if every commanded
        amplitude is unique -- equal magnitudes would let joint A wear joint
        B's response and still look correct."""
        src = ROUTING.read_text()
        block = re.search(r"SIGNATURE = \{(.*?)\}", src, re.S).group(1)
        mags = [abs(float(m)) for m in
                re.findall(r":\s*([+-]?\d*\.\d+)", block)]
        assert len(mags) >= 6, mags
        assert len(set(mags)) == len(mags), f"duplicate magnitudes: {mags}"

    def test_signature_spans_legs_waist_and_arms(self):
        """Legs are the claim under test; the waist distinguishes full-body
        from the vendor's pinned-waist mode; the arms are the control that
        was wired all along."""
        src = ROUTING.read_text()
        block = re.search(r"SIGNATURE = \{(.*?)\}", src, re.S).group(1)
        idx = [int(m) for m in re.findall(r"(\d+):", block)]
        assert any(i < 12 for i in idx), "no leg joint in the signature"
        assert any(12 <= i < 15 for i in idx), "no waist joint"
        assert any(i >= 15 for i in idx), "no arm joint"

    def test_probe_refuses_a_live_controller_and_requires_the_sidecar(self):
        """It publishes rt/lowcmd, so it must never meet a second controller
        or anything that is not a simulator."""
        src = ROUTING.read_text()
        assert "g1_deploy_onnx_ref" in src and "REFUSING" in src
        assert "SimClient" in src and "sidecar" in src

    def test_probe_subscriber_is_bound_to_self(self):
        for ln in ROUTING.read_text().splitlines():
            if "ChannelSubscriber(" in ln and "import" not in ln:
                assert re.match(r"\s*self\._\w+ = ChannelSubscriber\(", ln), ln


PATCH06 = REPO / "third_party" / "unitree-sim-isaaclab-local" / "06-render-throttle-and-loop-profile.patch"
PATCH07 = REPO / "third_party" / "unitree-sim-isaaclab-local" / "07-pin-real-time-factor.patch"


class TestPatch06RenderThrottle:
    """RTF 0.212 -> 0.431 came from here. The trap this patch exists to
    document: the vendor's --no_render sets render_interval, which a DIRECT
    env.sim.render() call ignores, so the flag measured 0.216 vs 0.216."""

    def test_patch_exists_and_touches_only_the_provider(self):
        assert PATCH06.exists()
        touched = set(re.findall(r"^\+\+\+ b/(\S+)", PATCH06.read_text(), re.M))
        assert touched == {"action_provider/action_provider_wh_dds.py"}, touched

    def test_render_is_throttled_by_env_var_defaulting_to_vendor_behaviour(self):
        added = "\n".join(ln[1:] for ln in PATCH06.read_text().splitlines()
                          if ln.startswith("+"))
        assert 'os.environ.get("SONIC_WB_RENDER_EVERY", "1")' in added
        # N<=1 must keep the unconditional vendor render.
        assert "self._render_every <= 1 or" in added

    def test_profile_line_flushes(self):
        """The vendor's [Performance] block never reached disk because it did
        not flush; a profiler you cannot read is not a profiler."""
        added = "\n".join(ln[1:] for ln in PATCH06.read_text().splitlines()
                          if ln.startswith("+"))
        assert "[wb-prof]" in added and "flush=True" in added

    def test_observation_compute_is_not_throttled(self):
        """obs compute is the ONLY refresher of rt/lowstate content and the
        only publisher of rt/odostate and rt/secondary_imu -- throttling it
        would starve SONIC of state while looking like a speedup."""
        text = PATCH06.read_text()
        # It survives as a CONTEXT line (leading space) -- i.e. still called
        # unconditionally every iteration, outside the throttle branch.
        assert re.search(r"^ +self\.env\.observation_manager\.compute\(\)$",
                         text, re.M), "obs compute is no longer a context line"
        added = "\n".join(ln[1:] for ln in text.splitlines()
                          if ln.startswith("+") and not ln.startswith("+++"))
        assert "self.env.sim.render()" in added, "render not in the added block"
        assert "observation_manager" not in added.split("_render_every")[-1][:200]


TIMESCALE_C = REPO / "tools" / "sonic_timescale.c"


class TestTimescaleShim:
    """tools/sonic_timescale.c interposes a syscall in the CONTROLLER, so its
    refusals matter more than its feature. Verified live 2026-08-12: a 20 ms
    relative timer became 50 ms at scale 2.5, and stayed 20 ms with the env
    var unset."""

    def test_source_exists(self):
        assert TIMESCALE_C.exists()

    def test_refuses_absolute_timers(self):
        """Scaling an absolute deadline is not a slowdown, it is a jump to a
        wrong instant. The shim must pass those through untouched."""
        src = TIMESCALE_C.read_text()
        assert "!(flags & TFD_TIMER_ABSTIME)" in src

    def test_scale_is_bounded_and_opt_in(self):
        """Inert unless SONIC_TIME_SCALE is set above 1.0; a typo must not
        stall the controller for an hour, and speeding it UP is never what a
        slow backend needs."""
        src = TIMESCALE_C.read_text()
        assert 'getenv("SONIC_TIME_SCALE")' in src
        assert "v > 1.0 && v <= 50.0" in src
        assert "scale = 1.0" in src

    def test_carries_the_hardware_prohibition(self):
        """It belongs on the sim-only list with --disable-crc-check and
        SONIC_FALL_RESET_M; a controller deliberately slowed against real
        time is a falling robot."""
        src = TIMESCALE_C.read_text()
        assert "NEVER BE USED WITH A PHYSICAL ROBOT" in src.upper()

    def test_launcher_only_preloads_when_asked(self):
        text = LAUNCHER.read_text()
        assert 'TIME_SCALE="${SONIC_TIME_SCALE:-}"' in text
        assert re.search(r'if \[ -n "\$TIME_SCALE" \]', text)
        # Rebuild on a stale .so -- a shim older than its source is a silent
        # wrong answer.
        assert '-nt "$TIMESCALE_SO"' in text


class TestTrackingProbeGuards:
    """Every guard here was added because its absence produced a confident
    wrong number on 2026-08-12."""

    def test_refuses_to_measure_a_dead_controller(self):
        """A MuJoCo baseline run reported leg RMS, pelvis std and a verdict
        for a robot SONIC had never started driving ('Safety check failed,
        cannot start control'). Silence from the controller looks exactly
        like a backend that cannot balance."""
        text = PROBE.read_text()
        assert "controller_is_live" in text
        assert "rt/lowcmd" in text
        assert re.search(r"if not run\.handshake\(\):\s*\n\s*return 2", text)

    def test_releases_the_mujoco_band_before_measuring(self):
        """MuJoCo hangs the robot from an elastic band during bring-up. The
        first baseline measured pelvis std 0.0037 at 0.96 m -- the band's
        steadiness, not the robot's. Isaac has no band, so comparing the two
        without releasing it compares a hoisted robot to a free one."""
        text = PROBE.read_text()
        assert "release_band" in text
        assert '"band"' in text and "release=True" in text

    def test_standing_is_height_and_tilt_and_contact(self):
        """This project's definition of standing is all three. Pelvis height
        alone scores a robot pitched 40 deg back as standing."""
        text = PROBE.read_text()
        assert "tilt_from_quat" in text
        assert "nonfoot_contacts" in text
        assert "tilt_max_deg" in text

    def test_pelvis_series_is_kept_not_just_its_std(self):
        """std scores a monotonic 0.5 m fall and a violent wobble alike."""
        assert "pelvis_series_m" in PROBE.read_text()


class TestPatch07PinsRealTimeFactor:
    """Four runs on 2026-08-18 separated PERFECTLY on whether the controller's
    time scale matched the sim's RTF: 0.179/1.0 collapse, 0.296/2.5 collapse,
    0.291/3.4 STANDS, 0.205/3.4 collapse. A drifting RTF makes a fixed scale a
    coin flip, so the RTF is pinned the way the MuJoCo reference pins 1.0."""

    def test_patch_exists_and_touches_only_the_provider(self):
        assert PATCH07.exists()
        touched = set(re.findall(r"^\+\+\+ b/(\S+)", PATCH07.read_text(), re.M))
        assert touched == {"action_provider/action_provider_wh_dds.py"}, touched

    def test_pinning_is_opt_in_and_off_by_default(self):
        added = "\n".join(ln[1:] for ln in PATCH07.read_text().splitlines()
                          if ln.startswith("+") and not ln.startswith("+++"))
        assert 'os.environ.get("SONIC_WB_TARGET_RTF", "0")' in added
        assert "self._rtf_target > 0" in added

    def test_overrun_forgives_debt_instead_of_catching_up(self):
        """Chasing a missed deadline makes every later frame sleep-free and
        the pinning silently stops pinning -- the same absolute-deadline
        discipline the MuJoCo reference uses."""
        added = "\n".join(ln[1:] for ln in PATCH07.read_text().splitlines()
                          if ln.startswith("+") and not ln.startswith("+++"))
        assert "max(self._rtf_deadline + period, now)" in added
        assert "_rtf_late" in added

    def test_launcher_pins_below_the_worst_observed_rate(self):
        text = LAUNCHER.read_text()
        assert 'TARGET_RTF="${SONIC_WB_TARGET_RTF:-0.20}"' in text
        assert 'SONIC_WB_TARGET_RTF="$TARGET_RTF"' in text

    def test_probe_warns_when_scale_and_rtf_disagree(self):
        """An unstable run under a mismatched scale is EXPECTED physics, not a
        backend defect -- the probe must say so rather than let the next
        reader draw the 2026-08-07 conclusion again."""
        text = PROBE.read_text()
        assert "scale_mismatch" in text and "outrunning the physics" in text


class TestProbeEvidenceCannotBeOverwritten:
    """On 2026-08-18 the probe ran eight times and wrote every head-camera
    frame to a fixed 'stand.jpg'/'kneel.jpg'. Five of the eight image pairs
    were destroyed in place -- including the MuJoCo baseline row, the only
    genuinely steady stand recorded all day. Evidence a later run can silently
    delete is not evidence."""

    def test_camera_frames_are_run_stamped(self):
        text = PROBE.read_text()
        assert 'name = f"{label}-{self.stamp}.jpg"' in text, (
            "camera_frame must stamp the filename with the run id")
        assert "self.stamp = stamp" in text

    def test_no_fixed_image_filename_survives_at_the_call_sites(self):
        """The bug was at the CALL site, not in the writer -- a stamping
        writer still overwrites if someone passes 'stand.jpg' through it."""
        text = PROBE.read_text()
        assert 'camera_frame("stand.jpg")' not in text
        assert 'camera_frame("kneel.jpg")' not in text
        assert 'run.camera_frame("stand")' in text
        assert 'run.camera_frame("kneel")' in text

    def test_run_configuration_is_recorded_unconditionally(self):
        """SONIC_TIME_SCALE was logged only when it DISAGREED with the
        measured RTF, so the scale of every run had to be reconstructed
        afterwards from 500/cmd_hz. That worked; it must not be needed."""
        text = PROBE.read_text()
        assert 'run.log("run_config"' in text
        for key in ("time_scale", "target_rtf", "full_body", "render_every",
                    "backend", "stamp"):
            assert f"{key}=" in text, f"run_config omits {key}"

    def test_run_config_is_logged_before_any_measurement(self):
        """A config record written after the first phase is missing from
        exactly the runs that crash during that phase."""
        text = PROBE.read_text()
        cfg = text.index('run.log("run_config"')
        first_phase = text.index('run.record_phase("stand"')
        assert cfg < first_phase


class TestProbeReportsTiltInTheUnitItLabels:
    """`tilt_from_quat` returns RADIANS. The probe stored it under
    'tilt_mean_deg' and printed it as 'deg', so a robot lying face-down
    reported 'tilt 1.6 deg' -- small enough to read as a mild lean, which
    bought several minutes of hunting a nonexistent dead-IMU bug. A wrong unit
    is a false measurement, not a cosmetic slip."""

    def test_tilt_is_converted_to_degrees_at_the_call_site(self):
        text = PROBE.read_text()
        assert "math.degrees(tilt_from_quat(" in text, (
            "tilt_from_quat returns radians; convert where it is called")

    def test_no_raw_tilt_from_quat_call_escapes_conversion(self):
        """One converted call site does not help if another is raw."""
        text = PROBE.read_text()
        calls = [m for m in re.finditer(r"tilt_from_quat\(", text)]
        # the import line plus the guard tests below reference the name too,
        # so count only actual invocations that are not preceded by degrees(
        raw = [m for m in calls
               if not text[max(0, m.start() - 14):m.start()].endswith(
                   "math.degrees(")]
        assert len(raw) == 0, (
            f"{len(raw)} unconverted tilt_from_quat call(s) remain")

    def test_math_is_imported(self):
        assert re.search(r"^import math$", PROBE.read_text(), re.M)


class TestTrackingEvidenceIndexIsHonest:
    """The index next to the artifacts is the only place a reader learns that
    five runs' images no longer exist and that the stored tilt fields are
    radians. If it drifts, the raw JSON silently misleads."""

    INDEX = REPO / "artifacts" / "tracking" / "README.md"

    def test_index_exists(self):
        assert self.INDEX.exists()

    def test_index_warns_that_stored_tilt_fields_are_radians(self):
        text = self.INDEX.read_text()
        assert "radians" in text
        assert "tilt_mean_deg" in text

    def test_index_names_the_runs_whose_images_are_gone(self):
        text = self.INDEX.read_text()
        assert "Missing, permanently" in text
        for run in ("10:39", "10:45", "10:48", "10:52", "11:01"):
            assert run in text, f"index does not account for run {run}"

    def test_index_does_not_call_the_isaac_run_a_stand(self):
        """It stayed up; it was never measured upright, and its own frame
        shows a rotated horizon. Calling that a stand is the kind of claim
        this project keeps having to retract."""
        text = self.INDEX.read_text()
        assert '"Stayed up" is not "stood."' in text
