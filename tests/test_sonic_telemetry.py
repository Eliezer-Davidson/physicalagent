"""Telemetry maths and staleness, tested without DDS.

The subscriber itself needs a running simulator; the parts that can be wrong
without one -- quaternion conventions, the tilt definition, freshness -- are
pure functions and are tested here. Getting the quaternion order backwards
produces a plausible attitude that is simply wrong, which is exactly the class
of bug no amount of watching the robot would reveal.
"""
from __future__ import annotations

import math

import pytest

from physicalagent.robots import sonic_telemetry as t


def deg(x):
    return math.degrees(x)


class TestQuaternionConvention:
    def test_identity_is_level(self):
        assert t.quat_to_rpy(1, 0, 0, 0) == (0.0, 0.0, 0.0)
        assert t.tilt_from_quat(1, 0, 0, 0) == 0.0

    def test_unitree_orders_w_first(self):
        """Unitree puts w first; scipy and most of the world put it last.
        Reading a w-first quaternion as w-last yields a confident wrong
        attitude, so the order is pinned by a rotation with a known answer.

        90 degrees about Z is (w, x, y, z) = (cos45, 0, 0, sin45): pure yaw,
        and crucially NOT a tilt. Read w-last it would be a 90-degree roll.
        """
        s = math.sqrt(0.5)
        roll, pitch, yaw = t.quat_to_rpy(s, 0.0, 0.0, s)
        assert deg(yaw) == pytest.approx(90.0, abs=1e-6)
        assert deg(roll) == pytest.approx(0.0, abs=1e-6)
        assert deg(pitch) == pytest.approx(0.0, abs=1e-6)
        assert deg(t.tilt_from_quat(s, 0.0, 0.0, s)) == pytest.approx(0.0, abs=1e-6)

    def test_pitch_forward_reads_as_pitch(self):
        s = math.sqrt(0.5)                      # 90 deg about Y
        _, pitch, _ = t.quat_to_rpy(s, 0.0, s, 0.0)
        assert deg(pitch) == pytest.approx(90.0, abs=1e-6)

    def test_non_unit_quaternion_does_not_raise(self):
        """A slightly denormalised quaternion off the wire must not be able to
        take down a status call."""
        t.quat_to_rpy(1.001, 0.0, 0.0, 0.0)
        t.tilt_from_quat(1.001, 0.0, 0.0, 0.0)


class TestTiltDefinition:
    def test_tilt_is_the_gravity_angle_not_hypot_of_euler(self):
        """Yaw is not tilt. hypot(roll, pitch) happens to agree for small
        rotations and disagrees exactly where it matters -- near the gimbal
        singularity and under combined rotations."""
        s = math.sqrt(0.5)
        # 90 deg yaw: upright, so tilt must be 0 even though Euler terms move
        assert deg(t.tilt_from_quat(s, 0, 0, s)) == pytest.approx(0.0, abs=1e-6)
        # 90 deg roll: lying on its side
        assert deg(t.tilt_from_quat(s, s, 0, 0)) == pytest.approx(90.0, abs=1e-6)
        # 180 deg roll: upside down
        assert deg(t.tilt_from_quat(0, 1, 0, 0)) == pytest.approx(180.0, abs=1e-6)

    def test_tilt_never_goes_nan_at_the_extremes(self):
        for q in [(1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)]:
            assert not math.isnan(t.tilt_from_quat(*q))


class TestJointTable:
    def test_twenty_nine_joints_in_the_wire_order(self):
        assert t.NUM_BODY_MOTORS == 29
        assert t.JOINT_NAMES[0] == "left_hip_pitch"
        assert t.JOINT_NAMES[12] == "waist_yaw"
        assert t.JOINT_NAMES[15] == "left_shoulder_pitch"
        assert t.JOINT_NAMES[28] == "right_wrist_yaw"

    def test_limbs_partition_the_joints_exactly_once(self):
        covered = [i for r in t.LIMBS.values() for i in r]
        assert sorted(covered) == list(range(29)), "limbs overlap or leave a gap"

    def test_limb_names_are_the_ones_the_tool_advertises(self):
        assert set(t.LIMBS) == {"left_leg", "right_leg", "waist",
                                "left_arm", "right_arm"}


class TestStaleness:
    def test_stale_threshold_is_far_above_the_publish_period(self):
        """The simulator publishes every control step (~200 Hz). The threshold
        has to be loose enough not to flap and tight enough that a stopped
        publisher is caught within a fraction of a second."""
        assert 0.1 <= t.STALE_AFTER_S <= 1.0


class TestItReadsTheOdometryTopicThisRobotActuallyPublishes:
    """Measured against the physical G1, 2026-08-19:

        LIVE   rt/odommodestate  497 Hz   SportModeState_ (unitree_go)
        silent rt/odostate         0

    The module docstring said "on the physical G1 the same three topics carry
    the same" -- an assumption, now measured false. It cost more than a missing
    reading: the fall detector takes pelvis height from this stream, so on
    hardware height was always None and the auto-damp guard could never confirm
    a fall. A guard that cannot fire is worse than no guard, because the
    adapter reports 'no fall' forever.

    SportModeState_ is not shaped like OdoState_. It has position[3], but
    velocity is `velocity` not `linear_velocity`, there is no `orientation`
    field at all, and the attitude lives in the nested imu_state.
    """

    def _sport(self):
        """Shaped like SportModeState_ on rt/odommodestate."""
        class I:
            quaternion = [1.0, 0.0, 0.0, 0.0]
            gyroscope = [0.01, -0.02, 0.03]
            accelerometer = [0.0, 0.0, 9.81]
            rpy = [0.0, 0.0, 0.0]
        class S:
            pass
        s = S()
        s.position = [1.5, -0.25, 0.731]
        s.velocity = [0.2, 0.05, -0.01]
        s.yaw_speed = 0.03
        s.imu_state = I()
        s.foot_force = [0, 0, 0, 0]
        return s

    def _odo(self):
        """Shaped like OdoState_ on rt/odostate -- what the simulator sends."""
        class O:
            position = [2.0, 1.0, 0.78]
            orientation = [1.0, 0.0, 0.0, 0.0]
            linear_velocity = [0.1, 0.0, 0.0]
            angular_velocity = [0.0, 0.0, 0.4]
        return O()

    def test_the_subscribed_topic_is_the_live_one(self):
        assert t.ODOM_TOPIC == "rt/odommodestate"

    def test_height_comes_from_position_z(self):
        p = t.pose_from_message(self._sport(), age_s=0.01)
        assert p["available"] is True
        assert p["height_m"] == pytest.approx(0.731)
        assert p["position_m"]["x"] == pytest.approx(1.5)

    def test_attitude_falls_back_to_the_nested_imu_quaternion(self):
        """SportModeState_ has no `orientation` field. Reading it blindly
        would raise, and catching that would silently return 'no pose'."""
        p = t.pose_from_message(self._sport(), age_s=0.01)
        assert p["tilt_deg"] == pytest.approx(0.0, abs=0.01)
        assert p["roll_deg"] == pytest.approx(0.0, abs=0.01)

    def test_the_velocity_field_rename_is_handled(self):
        p = t.pose_from_message(self._sport(), age_s=0.01)
        assert p["linear_velocity_m_s"][:2] == pytest.approx([0.2, 0.05])
        assert p["speed_m_s"] == pytest.approx(0.206, abs=1e-3)

    def test_angular_velocity_uses_yaw_speed_when_there_is_no_vector(self):
        p = t.pose_from_message(self._sport(), age_s=0.01)
        assert p["angular_velocity_rad_s"][2] == pytest.approx(0.03)

    def test_an_odostate_shaped_message_still_works_for_the_simulator(self):
        """The sim publishes OdoState_. One code path must read both, or
        fixing hardware silently breaks every sim comparison."""
        p = t.pose_from_message(self._odo(), age_s=0.01)
        assert p["height_m"] == pytest.approx(0.78)
        assert p["linear_velocity_m_s"][0] == pytest.approx(0.1)
        assert p["angular_velocity_rad_s"][2] == pytest.approx(0.4)

    def test_the_source_string_does_not_claim_a_topic_it_did_not_read(self):
        s = t.pose_from_message(self._sport(), age_s=0.01)["source"]
        assert "rt/odommodestate" in s
        assert "rt/odostate" not in s

    def test_a_missing_sample_says_which_topic_was_silent(self):
        p = t.pose_from_message(None, age_s=None)
        assert p["available"] is False
        assert "rt/odommodestate" in p["note"]


class TestControllerLivenessIsAboutAuthorityNotArrival:
    """Measured on the physical G1, 2026-08-19, with SONIC NOT running:
    rt/lowcmd arrives at 995 Hz, byte-identical, every q/dq/tau/kp/kd zero
    across all 35 slots. controller() reported active=True, source
    "rt/lowcmd from the SONIC policy".

    Its docstring's premise -- "SONIC writes rt/lowcmd while it is controlling
    and goes silent the instant it stops" -- is false on this robot. Something
    publishes an idle heartbeat continuously. Recency therefore proves only
    that a publisher exists, and this is the exact readout an operator uses to
    decide whether the balance controller took over."""

    def _cmd(self, kp=0.0, kd=0.0, tau=0.0, n=35):
        class M:
            pass
        class C:
            pass
        c = C()
        c.motor_cmd = []
        for _ in range(n):
            m = M()
            m.q = m.dq = 0.0
            m.tau, m.kp, m.kd = tau, kp, kd
            c.motor_cmd.append(m)
        return c

    def test_an_all_zero_heartbeat_is_not_a_live_controller(self):
        r = t.controller_from_message(self._cmd(), age_s=0.001)
        assert r["active"] is False
        assert "zero" in r["note"].lower() or "heartbeat" in r["note"].lower()

    def test_real_gains_are_a_live_controller(self):
        r = t.controller_from_message(self._cmd(kp=60.0, kd=1.5), age_s=0.001)
        assert r["active"] is True
        assert r["max_kp"] == pytest.approx(60.0)

    def test_feedforward_torque_alone_counts(self):
        assert t.controller_from_message(self._cmd(tau=2.0),
                                         age_s=0.001)["active"] is True

    def test_nothing_ever_arriving_is_still_reported(self):
        r = t.controller_from_message(None, age_s=None)
        assert r["active"] is False
        assert "ever arrived" in r["note"].lower()

    def test_a_stale_command_is_still_the_loud_failure(self):
        r = t.controller_from_message(self._cmd(kp=60.0), age_s=99.0)
        assert r["active"] is False
        assert "STOPPED" in r["note"]

    def test_the_note_does_not_name_sonic_when_sonic_may_not_be_running(self):
        """Attributing an anonymous heartbeat to 'the SONIC policy' is how an
        operator concludes bring-up succeeded when nothing started."""
        r = t.controller_from_message(self._cmd(), age_s=0.001)
        assert "SONIC" not in r.get("source", "")


class TestThermalReadout:
    """2026-08-19: SONIC warned that the right ankle roll was overheating and
    the operator stopped the session rather than gamble. Nothing on this
    machine exposed a temperature -- not the telemetry, not the recorder -- so
    the warning could be neither confirmed nor dismissed, and the answer is
    gone with the robot.

    SONIC's own rule (g1_deploy_onnx_ref.cpp:256) is hysteretic: warn at
    max(winding, driver) >= 90 C, clear below 85. Those numbers are mirrored
    here so a reading can be judged against the same bar that raised the alarm,
    and so a rising joint is visible BEFORE it trips."""

    def _low(self, temps):
        class M:
            pass
        class LS:
            pass
        ls = LS()
        ls.motor_state = []
        for w, d in temps:
            m = M()
            m.temperature = [w, d]
            ls.motor_state.append(m)
        return ls

    def test_it_reports_the_hottest_motors_by_name(self):
        temps = [(30, 30)] * 29
        temps[11] = (71, 68)
        temps[5] = (55, 54)
        out = t.thermal_from_message(self._low(temps))
        assert out["hottest"][0]["joint"] == "right_ankle_roll"
        assert out["hottest"][0]["max_c"] == 71
        assert out["hottest"][1]["joint"] == "left_ankle_roll"

    def test_it_uses_the_max_of_winding_and_driver_like_sonic_does(self):
        out = t.thermal_from_message(self._low([(60, 88)] + [(30, 30)] * 28))
        assert out["hottest"][0]["max_c"] == 88

    def test_a_motor_at_the_warn_threshold_is_flagged(self):
        temps = [(30, 30)] * 29
        temps[11] = (90, 70)
        out = t.thermal_from_message(self._low(temps))
        assert out["over_warn"] == ["right_ankle_roll"]
        assert out["status"] == "OVER"

    def test_the_approach_band_is_reported_before_it_trips(self):
        """85-89 C is the band SONIC has not yet warned about. Seeing a joint
        climb through it is the difference between a decision and a surprise."""
        temps = [(30, 30)] * 29
        temps[11] = (86, 70)
        out = t.thermal_from_message(self._low(temps))
        assert out["approaching"] == ["right_ankle_roll"]
        assert out["status"] == "WATCH"
        assert out["over_warn"] == []

    def test_a_cool_robot_is_reported_as_ok(self):
        out = t.thermal_from_message(self._low([(40, 39)] * 29))
        assert out["status"] == "OK"

    def test_the_thresholds_match_sonics_own(self):
        assert t.TEMP_WARN_C == 90 and t.TEMP_CLEAR_C == 85

    def test_no_message_says_so_rather_than_reporting_zero(self):
        out = t.thermal_from_message(None)
        assert out["available"] is False
        assert out["status"] == "UNKNOWN"


class TestControllerReadsTheMessageAndNotItsWrapper:
    """2026-08-19, physical G1, SONIC demonstrably in control -- the robot was
    balancing, walking and holding arm poses. `status` reported:

        SONIC IS NOT IN CONTROL
        rt/lowcmd IS arriving, and every kp, kd and feedforward torque in it
        is zero -- an idle heartbeat. NOTHING is holding the robot up.

    The capture proves the opposite: 100% of rt/lowcmd records in that window
    carried kp up to 99. controller() passed the _Sample WRAPPER to
    controller_from_message, while thermal() and pose() correctly pass .msg.
    command_authority reads getattr(msg, "motor_cmd", None) or [] -- a wrapper
    has no motor_cmd, so the defensive default returned an empty list and every
    gain read as zero. The defence is what hid it: nothing ever raised.

    This is the readout an operator uses to decide whether the robot is being
    held up. It failed toward "not in control" while the robot was walking."""

    class FakeMotor:
        def __init__(self, kp=0.0, kd=0.0, tau=0.0):
            self.kp, self.kd, self.tau = kp, kd, tau

    class FakeCmd:
        def __init__(self, motors):
            self.motor_cmd = motors

    def _live(self):
        import time
        cmd = self.FakeCmd([self.FakeMotor(kp=99.0, kd=6.3)])
        return t._Sample(cmd, time.monotonic())   # fresh, as on hardware

    def test_authority_is_seen_through_the_wrapper(self):
        """The exact hardware case: real gains must read as IN CONTROL."""
        tel = t.UnitreeTelemetry.__new__(t.UnitreeTelemetry)
        import threading
        tel._lock = threading.Lock()
        tel._cmd = self._live()
        got = tel.controller()
        assert got["active"] is True, got

    def test_a_wrapper_is_never_passed_to_command_authority(self):
        """Directly: the wrapper reads as zero, which is why this went unseen."""
        cmd = self.FakeCmd([self.FakeMotor(kp=99.0)])
        assert t.command_authority(cmd) == (99.0, 0.0, 0.0)
        assert t.command_authority(t._Sample(cmd, 0.0)) == (0.0, 0.0, 0.0)

    def test_every_accessor_unwraps_its_sample(self):
        """The general form. controller() was the only one that forgot."""
        import inspect
        src = inspect.getsource(t.UnitreeTelemetry)
        for line in src.splitlines():
            s = line.strip()
            if s.startswith("return ") and "_from_message(" in s:
                head = s.split("_from_message(")[1]
                arg = head.split(",")[0].strip()
                if arg in ("None", ""):
                    continue
                assert ".msg" in arg or "None" in arg, \
                    f"passes a wrapper, not a message: {s}"


class TestTheThermalGuardActsFasterThanAnOperatorCan:
    """Measured on the physical G1, 2026-08-19, right_shoulder_pitch (motor 22)
    while holding a raised-arm pose:

        t+1321.4s   65 C
        t+1327.9s   91 C     <- crossed the 90 C threshold
        t+1332.9s  106 C     <- peak

    215 C/min on the DRIVER sensor. The winding sensor moved 53 -> 56 C over the
    same window, because a driver stage has almost no thermal mass and the
    copper has a lot. The driver is what limits, and it is what warns.

    From 70 C there are 5.6 seconds of margin. The advice this session first
    gave -- 'watch it on every status command' -- was generalised from the
    ankle, which rises at 4 C/min. It cannot work here: nobody types that fast.
    So the decision has to be made by something that is always looking.

    verdict() is pure so the policy can be tested without a robot."""

    def _t(self, *pairs):
        rows = [{"index": i, "joint": j, "winding_c": c - 3,
                 "driver_c": c, "max_c": c} for i, (j, c) in enumerate(pairs)]
        rows.sort(key=lambda r: r["max_c"], reverse=True)
        return {"available": True, "hottest": rows}

    def test_a_cool_robot_is_left_alone(self):
        v = t.thermal_verdict(self._t(("right_shoulder_pitch", 51)))
        assert v["level"] == "ok"
        assert v["act"] is False

    def test_it_warns_before_it_acts(self):
        v = t.thermal_verdict(self._t(("right_shoulder_pitch", 82)))
        assert v["level"] == "warn"
        assert v["act"] is False
        assert "right_shoulder_pitch" in v["message"]

    def test_the_real_excursion_triggers_the_stop(self):
        """The exact reading at t+1327.9s."""
        v = t.thermal_verdict(self._t(("right_shoulder_pitch", 91)))
        assert v["level"] == "act"
        assert v["act"] is True
        assert "91" in v["message"]

    def test_it_names_the_joint_so_the_operator_knows_what_to_unload(self):
        v = t.thermal_verdict(self._t(("left_ankle_roll", 60),
                                      ("right_shoulder_pitch", 106)))
        assert v["hottest"]["joint"] == "right_shoulder_pitch"
        assert v["hottest"]["max_c"] == 106

    def test_unknown_temperature_is_not_treated_as_cool(self):
        """No reading must never read as safe -- that is the failure that lets
        a guard sleep through the event it exists for."""
        v = t.thermal_verdict({"available": False, "hottest": []})
        assert v["level"] == "unknown"
        assert v["act"] is False
        assert "no temperature" in v["message"].lower()

    def test_thresholds_are_below_the_measured_peak(self):
        """A threshold above what the hardware actually reached is decoration."""
        assert t.TEMP_ACT_C <= 106
        assert t.TEMP_GUARD_WARN_C < t.TEMP_ACT_C
