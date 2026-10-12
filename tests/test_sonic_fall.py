"""The fall detector, held to the project's own law: every guard validated
against every mode's real behaviour, never against "standing" alone.

The in-mode postures below are the MEASURED ones from
artifacts/mode-test-2026-08-01.md wherever a measurement exists, and the
vendor operating point otherwise. If the detector flags any of them, it has
re-committed the §2 bug (a standing-calibrated bound applied to a mode that
is not standing) and must not ship.
"""
from __future__ import annotations

import unittest

from physicalagent.robots import sonic_modes
from physicalagent.robots.sonic_fall import ENVELOPES, FallDetector


class FakeClock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def detector(**kw):
    clock = FakeClock()
    d = FallDetector(clock=clock, **kw)
    return d, clock


def feed(d, clock, height, tilt, n, dt=0.1):
    out = None
    for _ in range(n):
        clock.t += dt
        out = d.update(height, tilt)
    return out


# (height_m, tilt_deg) the mode legitimately holds. (m) = measured 2026-08-01,
# (v) = vendor operating point, (~) = plausible in-kind reading, unmeasured.
IN_MODE_POSTURE = {
    "idle": (0.786, 1.1),          # (m) baseline
    "slow_walk": (0.760, 5.0),     # (m) min pelvis over the bout
    "walk": (0.740, 8.0),          # (~) between slow_walk and stealth
    "run": (0.650, 20.0),          # (~) depth unmeasured; envelope must allow it
    "squat": (0.400, 5.0),         # (v) height 0.4 hold
    "kneel_two_legs": (0.407, 5.0),   # (m) settled kneel
    "kneel": (0.407, 10.0),        # (v/m) one knee, same commanded height
    "lie_face_down": (0.108, 83.0),   # (m) settled lie
    "crawl": (0.382, 60.0),        # (m) clean crawl min, quadruped tilt
    "boxing_stance": (0.750, 10.0),   # (~) standing guard
    "boxing_walk": (0.720, 10.0),
    "punch_left": (0.750, 12.0),
    "punch_right": (0.750, 12.0),
    "punch_random": (0.750, 12.0),
    "elbow_crawl": (0.150, 58.0),  # (m) lowest clean bout, measured tilt
    "hook_left": (0.750, 12.0),
    "hook_right": (0.750, 12.0),
    "jump_forward": (0.653, 25.0),    # (m) deepest dip in the skip
    "stealth_walk": (0.701, 10.0),    # (m)
    "injured_walk": (0.646, 12.0),    # (m)
    "ledge_walk": (0.740, 8.0),    # (m)
    "carry": (0.678, 10.0),        # (m)
    "stealth_walk_2": (0.533, 12.0),  # (m) the lowest standing gait
    "dance_walk": (0.693, 12.0),   # (m)
    "zombie_walk": (0.721, 10.0),  # (m)
    "gun_walk": (0.648, 10.0),     # (m) 2nd lowest crouch
    "scare_walk": (0.717, 10.0),   # (m)
}

# A robot flat on the floor: pelvis 0.10, tilt 85. Measured falls tonight read
# 0.088-0.169 m at 58-83 deg with shoulders down; this is squarely a fall in
# any mode that is not itself a floor mode.
FLAT = (0.10, 85.0)

# Modes whose own envelope legitimately contains FLAT — the detector abstains
# there, and says so in its envelope source. Only the lie mode qualifies.
ABSTAINS_ON_FLAT = {"lie_face_down"}


class TestEveryModeHasADeliberateEnvelope(unittest.TestCase):
    def test_envelopes_cover_exactly_the_27_modes(self):
        self.assertEqual({m.name for m in sonic_modes.MODES}, set(ENVELOPES))

    def test_every_envelope_cites_a_source(self):
        for name, env in ENVELOPES.items():
            self.assertTrue(env.source.strip(), f"{name} has no source citation")

    def test_posture_fixture_covers_every_mode(self):
        self.assertEqual({m.name for m in sonic_modes.MODES}, set(IN_MODE_POSTURE))


class TestInModePosturesAreNotFalls(unittest.TestCase):
    """The §2 regression test: the detector must accept every mode's own
    legitimate posture, sustained indefinitely, past the grace window."""

    def test_no_mode_flags_its_own_posture(self):
        for name, (height, tilt) in IN_MODE_POSTURE.items():
            d, clock = detector()
            d.note_mode(name)
            clock.t += 10.0                      # far past grace
            out = feed(d, clock, height, tilt, 50)
            self.assertFalse(out["fallen"],
                             f"{name} at its own measured posture "
                             f"({height} m, {tilt} deg) was called a fall: "
                             f"{out['reason']}")


class TestRealFallsAreCaught(unittest.TestCase):
    def test_flat_on_the_floor_is_a_fall_in_every_non_floor_mode(self):
        for name in IN_MODE_POSTURE:
            if name in ABSTAINS_ON_FLAT:
                continue
            d, clock = detector()
            d.note_mode(name)
            clock.t += 10.0
            out = feed(d, clock, *FLAT, n=10)
            self.assertTrue(out["fallen"],
                            f"flat on the floor in mode {name} was not caught")
            self.assertIn(name, out["reason"])

    def test_lie_face_down_abstains_and_that_is_the_documented_contract(self):
        d, clock = detector()
        d.note_mode("lie_face_down")
        clock.t += 10.0
        out = feed(d, clock, *FLAT, n=50)
        self.assertFalse(out["fallen"])
        self.assertIn("abstains", ENVELOPES["lie_face_down"].source)

    def test_measured_elbow_crawl_collapse_is_a_fall(self):
        # The 2026-08-01 collapse: pelvis 0.097, tilt 83, torso down.
        d, clock = detector()
        d.note_mode("elbow_crawl")
        clock.t += 10.0
        out = feed(d, clock, 0.097, 83.0, n=10)
        self.assertTrue(out["fallen"])


class TestDebounceGraceAndLatch(unittest.TestCase):
    def test_four_bad_samples_then_recovery_is_not_a_fall(self):
        d, clock = detector(confirm_samples=5)
        d.note_mode("slow_walk")
        clock.t += 10.0
        feed(d, clock, *FLAT, n=4)
        out = feed(d, clock, 0.760, 5.0, n=1)
        self.assertFalse(out["fallen"])
        self.assertEqual(out["strikes"], 0)

    def test_fifth_consecutive_bad_sample_confirms_and_latches(self):
        d, clock = detector(confirm_samples=5)
        d.note_mode("slow_walk")
        clock.t += 10.0
        out = feed(d, clock, *FLAT, n=5)
        self.assertTrue(out["fallen"])
        # Latched: a good reading afterwards does NOT clear it. SONIC never
        # gets up on its own — recovery is an operator/agent-run mode sequence
        # (artifacts/getup-2026-08-02.md), so a healthy reading after a
        # confirmed fall means somebody intervened, and the latch must survive
        # until reset() acknowledges that on purpose.
        out = feed(d, clock, 0.786, 1.0, n=20)
        self.assertTrue(out["fallen"])
        d.reset()
        self.assertFalse(d.fallen)

    def test_mode_change_opens_grace_and_grace_abstains(self):
        # Entering a ground mode passes through fall-shaped postures for
        # seconds (measured: kneel ~4 s, lie ~6 s). Inside the window the
        # detector must abstain and say it is abstaining.
        d, clock = detector(grace_s=4.0)
        d.note_mode("slow_walk")
        clock.t += 10.0
        d.note_mode("crawl")                      # transition begins
        out = feed(d, clock, 0.79, 2.0, n=5, dt=0.1)   # descending, still high
        self.assertTrue(out["in_grace"])
        self.assertFalse(out["fallen"])
        clock.t += 4.0                            # grace over
        out = feed(d, clock, 0.40, 60.0, n=10)    # clean crawl posture
        self.assertFalse(out["fallen"])

    def test_missing_telemetry_abstains_rather_than_inventing_a_posture(self):
        d, clock = detector()
        d.note_mode("slow_walk")
        clock.t += 10.0
        out = feed(d, clock, None, None, n=50)
        self.assertFalse(out["fallen"])


def feed_contacts(d, clock, height, tilt, contacts, n, dt=0.1):
    out = None
    for _ in range(n):
        clock.t += dt
        out = d.update(height, tilt, contacts=contacts)
    return out


class TestContactEvidence(unittest.TestCase):
    """Item 4: toppled-body contact is the measured discriminator for
    entry-transition collapses (3 of 4 elbow_crawl falls happened inside the
    grace window), so it counts DURING grace."""

    def test_shoulder_contact_during_grace_confirms_a_fall(self):
        d, clock = detector(grace_s=4.0, confirm_samples=5)
        d.note_mode("elbow_crawl")           # transition just began — in grace
        out = feed_contacts(d, clock, 0.25, 60.0,
                            ["left_shoulder_yaw_link", "left_ankle_roll_link"],
                            n=5)
        self.assertTrue(out["fallen"])
        self.assertIn("shoulder", out["reason"])

    def test_clean_crawl_contacts_never_strike(self):
        d, clock = detector(grace_s=0.0)
        d.note_mode("crawl")
        clock.t += 10.0
        out = feed_contacts(d, clock, 0.40, 60.0,
                            ["L_hand_base_link", "left_knee_link",
                             "right_ankle_roll_link"], n=50)
        self.assertFalse(out["fallen"])

    def test_lie_face_down_is_contact_exempt(self):
        d, clock = detector(grace_s=0.0)
        d.note_mode("lie_face_down")
        clock.t += 10.0
        out = feed_contacts(d, clock, 0.10, 83.0,
                            ["torso_link", "right_shoulder_yaw_link"], n=50)
        self.assertFalse(out["fallen"])

    def test_contact_evidence_stands_alone_when_pose_is_stale(self):
        # A frozen odostate must not blind the detector when MuJoCo's own
        # contact list still says a shoulder is on the floor.
        d, clock = detector(grace_s=0.0, confirm_samples=3)
        d.note_mode("crawl")
        clock.t += 10.0
        out = feed_contacts(d, clock, None, None, ["torso_link"], n=3)
        self.assertTrue(out["fallen"])


class TestCommandedHeight(unittest.TestCase):
    """Item 5: envelope floors follow a commanded set_height instead of
    assuming the mode's default — a lawful crouch must not read as a fall."""

    def test_commanded_low_height_lowers_the_floor(self):
        d, clock = detector(grace_s=0.0)
        d.note_mode("stealth_walk_2")
        d.note_height(0.5)                   # rides up to ~0.28 below command
        clock.t += 10.0
        out = feed(d, clock, 0.30, 12.0, n=50)
        self.assertFalse(out["fallen"],
                         f"lawful crouch under commanded height flagged: "
                         f"{out['reason']}")

    def test_flat_fall_still_caught_under_a_commanded_height(self):
        d, clock = detector(grace_s=0.0)
        d.note_mode("stealth_walk_2")
        d.note_height(0.5)
        clock.t += 10.0
        out = feed(d, clock, 0.09, 85.0, n=10)
        self.assertTrue(out["fallen"])

    def test_clearing_the_command_restores_the_mode_floor(self):
        d, clock = detector(grace_s=0.0, confirm_samples=5)
        d.note_mode("slow_walk")
        d.note_height(0.5)
        d.note_height(None)
        clock.t += 10.0
        out = feed(d, clock, 0.30, 5.0, n=10)   # under slow_walk's 0.45 floor
        self.assertTrue(out["fallen"])


if __name__ == "__main__":
    unittest.main()
