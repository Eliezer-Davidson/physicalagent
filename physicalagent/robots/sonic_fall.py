"""Mode-aware fall detection for the SONIC G1. Pure stdlib.

WHY THIS EXISTS
    SONIC has no fall management at all: no detector, no reflex, no get-up
    mode or policy (verified 2026-08-01 by reading the whole deploy binary and
    enumerating every ONNX on disk). The only fall check in the system lives
    in the SIMULATOR's episode reset, which does not exist on hardware. On the
    real robot, nothing notices a fall — the policy keeps fighting the floor.
    (2026-08-02: a get-up does exist as a COMPOSED mode sequence —
    lie_face_down -> crawl -> kneel_two_legs -> idle stood the robot up from a
    real shoulder-drop fall in sim, n=1, artifacts/getup-2026-08-02.md. That
    is a recovery RECIPE the operator layer can run after reset; SONIC itself
    still notices nothing, so nothing in this module changes.)

    This module is step 1 of the safety build order (detect, reflex, recover):
    a detector that runs on signals the PHYSICAL robot publishes — height from
    rt/odostate, tilt from the IMU quaternion — so the same code works in sim
    and on hardware.

THE LAW THIS MODULE EXISTS TO OBEY
    A bound calibrated on a standing robot, applied to a mode that is not
    standing, was THE recurring bug of this project: six times in one day, in
    four different layers. A naive height-or-tilt guard calls
    stealth_walk_2 (pelvis 0.533) a fall, calls kneel (0.407) a fall, calls
    crawl (hands and knees) a fall, and makes lie_face_down impossible.

    So every mode carries its own envelope, sourced from the vendor's
    operating points and from the measured fingerprints in
    artifacts/mode-test-2026-08-01.md (27/27 modes observed 2026-08-01).
    test_sonic_fall.py refuses to pass if any mode in sonic_modes.MODES lacks
    an envelope — adding mode 28 without deciding its envelope is a test
    failure, not a silent default.

WHAT A VERDICT MEANS
    - Falls are confirmed only after `confirm_samples` consecutive
      out-of-envelope readings. A single frame outside the envelope is
      transition chatter, not a fall — the same reasoning as the flight
      detector's 5-sample rule.
    - Mode changes open a grace window (`grace_s`). Measured 2026-08-01:
      entering kneel takes ~4 s, entering lie_face_down ~6 s, and both pass
      through postures a static envelope must call a fall. During grace the
      detector reports in_grace and abstains. A real fall during a
      transition INTO a ground mode is genuinely indistinguishable from the
      transition by posture alone; the abstention is honest, not lazy.
    - lie_face_down's envelope accepts everything. Lying on the floor on
      purpose and having fallen are the same posture; only the commanded mode
      distinguishes them, and this detector already knows the mode. What it
      can still catch in that mode is nothing — so it says so.
    - A confirmed fall LATCHES. Un-latching is reset(), an operator act,
      matching the posture annotation's "ask the operator to reset" contract.

KNOWN LIMITS OF THE GRACE WINDOW (adversarial review, 2026-08-01 — real,
deferred, and to be designed against contact sensing rather than papered over):
    - Ground-mode ENTRY falls land inside the window: 3 of 4 measured
      elbow_crawl collapses happened within 4 s of the transition. Height and
      tilt cannot separate a collapsing entry from a clean one; the measured
      discriminator is a shoulder in ground_contact, which the sim sidecar
      reports and hardware foot/torso sensing would have to replace.
    - Every mode change reopens the window and zeroes strikes, so alternating
      mode commands < grace_s apart (a model retrying at a fallen robot) can
      defer confirmation indefinitely, and a task-end stop_all during a
      3-strike topple restarts the count.
    A future revision should scope grace to transitions that NEED it (into
    static/ground modes) and rate-limit reopening. Until then the watchdog is
    a mid-bout and settled-state detector, not an entry-transition detector.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass

from physicalagent.robots import sonic_modes


@dataclass(frozen=True)
class Envelope:
    """What posture this mode is ALLOWED to hold without being a fall.

    min_height_m is set below the lowest pelvis height the mode legitimately
    reaches; max_tilt_deg above its steepest legitimate pelvis tilt. Sources
    are cited per entry: (v) = vendor operating point, (m) = measured on this
    robot 2026-08-01, (d) = default for the kind, unmeasured depth.
    """
    min_height_m: float
    max_tilt_deg: float
    source: str


# Standing gaits: lowest measured standing pelvis is stealth_walk_2 at 0.533
# (m); the envelope floor sits a margin below it. Tilt during styled walks
# stays shallow; 45 deg is far outside any measured gait and far inside a
# real topple (measured falls read 58-83 deg).
_STANDING = Envelope(0.45, 45.0, "(m) stealth_walk_2 min 0.533; falls read >58 deg")

# idle can legitimately be commanded down to height 0.4 by the console
# (observed on the wire 2026-08-01 after every static-mode task), so its
# floor sits below that, not at standing height.
_IDLE = Envelope(0.30, 40.0, "(m) idle@0.4 is a legitimate console command")

_KNEEL = Envelope(0.28, 55.0, "(v) height 0.4; (m) kneel_two_legs 0.407-0.471, upright")

ENVELOPES: dict[str, Envelope] = {
    "idle": _IDLE,
    "slow_walk": _STANDING,
    "walk": _STANDING,
    "run": Envelope(0.40, 50.0, "(d) run depth unmeasured; slightly wider than walks"),
    "squat": Envelope(0.25, 45.0, "(v) height 0.4 hold"),
    "kneel_two_legs": _KNEEL,
    "kneel": _KNEEL,
    "lie_face_down": Envelope(0.0, 180.0,
                              "(m) on the floor by design (0.08-0.25, tilt to 83); "
                              "indistinguishable from a fall by posture — abstains"),
    "crawl": Envelope(0.26, 80.0, "(m) clean crawl 0.382-0.502; collapses read 0.169-0.238"),
    "boxing_stance": _STANDING,
    "boxing_walk": _STANDING,
    "punch_left": _STANDING,
    "punch_right": _STANDING,
    "punch_random": _STANDING,
    "elbow_crawl": Envelope(0.12, 75.0,
                            "(m) clean bouts 0.150-0.322 at tilt ~58; collapses read "
                            "0.088-0.097 at tilt ~83 — the margins are THIN, n=4"),
    "hook_left": _STANDING,
    "hook_right": _STANDING,
    "jump_forward": Envelope(0.40, 55.0, "(m) pelvis dips to 0.653 in the skip; margin below"),
    "stealth_walk": _STANDING,
    "injured_walk": _STANDING,
    "ledge_walk": _STANDING,
    "carry": _STANDING,
    "stealth_walk_2": _STANDING,
    "dance_walk": _STANDING,
    "zombie_walk": _STANDING,
    "gun_walk": _STANDING,
    "scare_walk": _STANDING,
}


# Contact evidence (2026-08-01 evening review, item 4). Height and tilt cannot
# separate a collapsing ground-mode ENTRY from a clean one — 3 of 4 measured
# elbow_crawl collapses happened inside the transition grace window. The
# measured discriminator is WHICH BODY touches the floor: every observed
# collapse put a shoulder or the torso down; no mode legitimately does that
# except lie_face_down (and a commanded damp). Contact strikes therefore
# BYPASS grace — that is their whole point — but still need the same
# consecutive-sample confirmation. In sim the names come from MuJoCo's own
# contact list via the sidecar; on hardware nothing provides them yet, so the
# channel is dormant (None) until foot/torso sensing exists.
TOPPLED_CONTACT_MARKERS = ("shoulder", "head", "torso", "waist", "pelvis")
CONTACT_EXEMPT_MODES = frozenset({"lie_face_down"})

# Commanded height (item 5): envelope floors assume each mode's DEFAULT
# height. A legitimate set_height() moves the whole operating point — the
# stealth family rides up to ~0.256 m below its commanded height (measured:
# commanded 0.7887, pelvis min 0.533) — so the floor must follow the command
# or a lawful crouch reads as a fall. 0.28 = that measured ride + margin;
# 0.10 is the absolute floor below which nothing upright remains.
RIDE_BELOW_COMMANDED_M = 0.28
ABSOLUTE_FLOOR_M = 0.10


def toppled_contacts(contacts) -> list[str]:
    """The subset of contact body names that mean 'toppled over'."""
    if not contacts:
        return []
    return sorted({c for c in contacts
                   if any(m in str(c).lower() for m in TOPPLED_CONTACT_MARKERS)})


# Day-one hardware allowlist (§4e step 4). The G1 tier table governs
# hardware; this is the starting point it gates: postural modes plus the slow
# walk, nothing dynamic, nothing that ends on the floor. run has a measured
# flight phase and jump_forward is the most aggressive mode in the table. The
# ground modes: a stand-up SEQUENCE from the floor is now demonstrated in sim
# (n=1, 2026-08-02, artifacts/getup-2026-08-02.md), but it is sim-only
# evidence and the wrong sequence (idle from prone) thrashes violently — so
# ground modes stay off the list until the recovery recipe is proven ON
# HARDWARE. Wire via $SONIC_MODE_ALLOWLIST.
HARDWARE_DAY_ONE_ALLOWLIST = ("idle", "slow_walk", "squat")

# The §2 law as a tripwire: a mode without a deliberate envelope decision must
# fail at IMPORT, not fall back to some default calibrated on a different
# posture. If this fires you added (or renamed) a mode — go measure it, or at
# minimum decide its envelope on the vendor's operating point, then cite that.
_missing = [m.name for m in sonic_modes.MODES if m.name not in ENVELOPES]
_extra = [name for name in ENVELOPES if name not in sonic_modes.BY_NAME]
if _missing or _extra:
    raise RuntimeError(
        f"sonic_fall envelopes out of sync with sonic_modes: "
        f"missing={_missing} extra={_extra}")


class FallDetector:
    """Feed it (height, tilt) readings; it says whether the robot has fallen,
    judged against what the CURRENT mode is allowed to look like.

    Deliberately free of I/O and threads: the caller owns the sampling loop
    (and the tests own the clock), this owns only the judgement.
    """

    # A commanded damp (set_pose('damp') / emergency_stop) sinks the robot to
    # the floor BY DESIGN. Without this state the detector confirms a "fall"
    # on the very posture the operator asked for, alarms, and re-damps — a
    # false alarm on the safety pose itself. Not in ENVELOPES because it is
    # not a locomotion mode; the tripwire below checks against MODES only.
    DAMPED = "__damped__"

    def __init__(self, *, confirm_samples: int = 5, grace_s: float = 4.0,
                 clock=time.monotonic):
        self._confirm = max(1, int(confirm_samples))
        self._grace_s = float(grace_s)
        self._clock = clock
        self._mode = "idle"
        self._mode_changed_at = clock()
        self._strikes = 0
        self._fallen = False
        self._reason: str | None = None
        self._commanded_height: float | None = None
        self._recovering = False
        # note_mode/reset run on the console thread while update runs on the
        # watchdog thread. The state is tiny; one lock removes every torn
        # interleaving (mode vs grace timestamp, reason vs latch) outright.
        self._lock = threading.Lock()

    # -- inputs ----------------------------------------------------------
    def begin_recovery(self) -> None:
        """A deliberate, measured get-up is starting: stop counting ground
        contact as evidence of a fall until it ends.

        MEASURED 2026-08-06, 2/2 prone trials: the recovery route reached every
        stage and the robot ended at 0.79 m and <2 deg tilt — and the detector
        latched anyway, 'in mode crawl: ground contact on
        right_shoulder_yaw_link'. A body coming off the floor THROUGH crawl has
        a shoulder on the floor; that is the route working, not failing.

        Contact strikes deliberately bypass the grace window (elbow_crawl
        collapses happened inside it), and that rule was validated against
        DESCENDING motion only. This project's own law is to validate every
        guard against the reference motion itself — the ascent was never in the
        reference set. Scoped to the recovery rather than exempting crawl
        outright, because a crawl COLLAPSE must still be caught.

        Height and tilt still apply throughout: this narrows one channel, it
        does not switch the detector off.
        """
        with self._lock:
            self._recovering = True
            self._strikes = 0

    def end_recovery(self) -> None:
        with self._lock:
            self._recovering = False
            self._strikes = 0

    @property
    def recovering(self) -> bool:
        return self._recovering

    def note_mode(self, mode_name: str) -> None:
        """Call on every mode change; opens the transition grace window.

        Unknown names are IGNORED, keeping the current envelope: silently
        adopting the idle envelope for a string nobody vetted would be the
        standing-calibrated default this module exists to forbid.
        """
        name = str(mode_name).strip().lower()
        with self._lock:
            if name == self._mode:
                return
            if name != self.DAMPED and name not in ENVELOPES:
                return
            self._mode = name
            self._mode_changed_at = self._clock()
            self._strikes = 0

    def note_damp(self) -> None:
        """A commanded damp: the robot is going to the floor on purpose."""
        self.note_mode(self.DAMPED)

    def note_height(self, height_m: float | None) -> None:
        """A commanded body height (set_height). It survives across mode
        changes on the wire, so it survives here too until changed."""
        with self._lock:
            self._commanded_height = (None if height_m is None or height_m < 0
                                      else float(height_m))

    def update(self, height_m: float | None, tilt_deg: float | None,
               contacts=None) -> dict:
        """One posture sample in, one verdict out. None readings abstain —
        a fall detector must never invent a posture from missing telemetry.
        The CALLER must also pass None for stale readings: a frozen stream
        replayed at 10 Hz would otherwise manufacture five 'consecutive'
        strikes out of one bad sample, or hold 'standing' forever.

        `contacts` is an iterable of ground-contact body names (sim sidecar /
        future hardware sensing) or None when unavailable. Toppled-body
        contact is the one signal that counts DURING grace."""
        with self._lock:
            if self._fallen:
                return self._verdict(in_grace=False)
            if self._mode == self.DAMPED:
                return self._verdict(in_grace=False)

            toppled = ([] if (self._mode in CONTACT_EXEMPT_MODES
                              or self._recovering)
                       else toppled_contacts(contacts))
            if height_m is None or tilt_deg is None:
                if not toppled:
                    return self._verdict(in_grace=False)
                low = tipped = False       # contact evidence stands alone
                in_grace = (self._clock() - self._mode_changed_at) < self._grace_s
            else:
                in_grace = (self._clock() - self._mode_changed_at) < self._grace_s
                if in_grace and not toppled:
                    self._strikes = 0
                    return self._verdict(in_grace=True)
                env = ENVELOPES[self._mode]
                floor = env.min_height_m
                if self._commanded_height is not None:
                    floor = min(floor, max(ABSOLUTE_FLOOR_M,
                                           self._commanded_height
                                           - RIDE_BELOW_COMMANDED_M))
                low = (not in_grace) and height_m < floor
                tipped = (not in_grace) and tilt_deg > env.max_tilt_deg

            if toppled or low or tipped:
                self._strikes += 1
                if self._strikes >= self._confirm:
                    parts = []
                    if toppled:
                        parts.append("ground contact on " + ", ".join(toppled))
                    if low:
                        parts.append(f"pelvis {height_m:.2f} m is below the "
                                     f"{floor:.2f} m floor")
                    if tipped:
                        parts.append(f"tilt {tilt_deg:.0f} deg exceeds "
                                     f"{ENVELOPES[self._mode].max_tilt_deg:.0f} deg")
                    self._reason = f"in mode '{self._mode}': " + " and ".join(parts)
                    self._fallen = True
            else:
                self._strikes = 0
            return self._verdict(in_grace=in_grace)

    def reset(self) -> None:
        """Operator act: clears the latch after the robot has been recovered."""
        with self._lock:
            self._fallen = False
            self._reason = None
            self._strikes = 0
            self._mode_changed_at = self._clock()

    # -- outputs ---------------------------------------------------------
    @property
    def fallen(self) -> bool:
        return self._fallen

    @property
    def reason(self) -> str | None:
        return self._reason

    def _verdict(self, *, in_grace: bool) -> dict:
        return {"fallen": self._fallen, "reason": self._reason,
                "in_grace": in_grace, "mode": self._mode,
                "strikes": self._strikes}
