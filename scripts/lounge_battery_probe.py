#!/usr/bin/env python
"""Four-task lounge battery. Same prompts for pass A (baseline) and pass B
(empowered); the only difference between passes is the committed tool surface,
prompt and hop cap. Run as a SONIC_FRONTEND so the proven bring-up is reused:

    SONIC_FRONTEND=scripts/lounge_battery_probe.py \
        bash scripts/lounge_live.sh --pass-name A --max-hops 12

Success is judged by MEASUREMENT (pose, contacts, object heights), never by the
model's own report — and a screenshot is captured per task, verified fresh
(exit code + mtime; the getup probe's stale-screenshot lesson, 2026-08-07).
Judges are pure functions pinned by tests/test_lounge_battery.py."""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
import pathlib
import subprocess
import sys
import time

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from physicalagent.robots import sonic_world  # noqa: E402  (stdlib-pure)

# black_backpack_0 is deliberately absent: excluded from the scene (no settled
# pose exists in any source variant — measured).
SCENE_NAMES = ("grey_and_yellow_bench_0", "white_round_table_0",
               "black_laptop_1_0", "black_laptop_2_0", "black_thermos_0")
# what a vision model may plausibly call each object
_SYNONYMS = {
    "white_round_table_0": ("table",),
    "grey_and_yellow_bench_0": ("bench", "sofa", "couch"),
    "black_laptop_1_0": ("laptop", "computer", "notebook"),
    "black_laptop_2_0": ("laptop", "computer", "notebook"),
    "black_thermos_0": ("thermos", "bottle", "flask", "tumbler", "cup"),
}
TASKS = (
    ("see", "Report what you see around you."),
    ("goto_table", "Walk to the coffee table and stop in front of it."),
    ("look_thermos", "Look at the thermos on the table and describe it."),
    ("pick_thermos", "Pick up the thermos from the table."),
)
PICK_LIFT_M = 0.05
# A fingertip resting on the object at 0.1 N is a touch, not a grasp. force_n
# was in every contact row the sidecar returns and no judge ever read it.
PICK_MIN_FORCE_N = 1.0
# goto_object stops at a 0.6 m standoff (sonic_world.MIN_STANDOFF_M is 0.4).
# The old bar was 1.0 m -- 2.5x the standoff it was judging, wide enough that
# "stopped in front of the table" and "somewhere else in this 1.5 m room"
# scored identically. 0.8 = the standoff plus 0.2 m of walk overshoot.
# Retroactively harmless: both passes ended at 0.36/0.37 m.
GOTO_TOL_M = 0.8
# Below this the robot did not "walk to" anything. SONIC's turn() alone drifts
# ~0.26-0.33 m forward per task (measured across 'see' in both passes), so this
# bar separates "did not move" from "moved"; TRAVEL_TOOLS below is what
# separates "walked" from "drifted while turning".
GOTO_MIN_TRAVEL_M = 0.25
# The target is an ESTIMATE built from the robot's own pose. If it moves more
# than this between the two reads, the start and end distances were measured
# against two different points and cannot be differenced.
GOTO_TARGET_STABLE_M = 0.15
# No judge in the battery ever checked posture. `fell` only catches falls the
# sidecar LATCHED and reset; a robot merely toppled and never reset has a
# pelvis inside tolerance and was scored as an arrival.
UPRIGHT_TILT_DEG = 30.0
# Drift is a difference of two estimates, so odometry error enters it twice.
# WARN is a flag; only FAIL (well above the observed 0.009-0.025 m no-contact
# noise floor) is allowed to fail a task.
FURNITURE_SHOVE_WARN_M = 0.05
FURNITURE_SHOVE_FAIL_M = 0.15
# Stamped into every summary.json. The committed pass A / pass B artifacts were
# scored by v1, and nothing in them says so -- which is how a robot that never
# moved ended up in an A/B table as a navigation result.
JUDGE_VERSION = 2

# "wrist" is deliberately NOT a hand part: a wrist-scoop is not a grasp, and
# counting it here would record a FALSE GRASP against the project-wide "no
# grasp has ever been achieved" record -- the worst possible evidence
# corruption this probe could commit. See WRIST_PARTS below: wrist contacts
# are still counted and reported, just never ok-qualifying.
HAND_PARTS = ("hand", "thumb", "index", "middle", "ring", "pinky")
WRIST_PARTS = ("wrist",)
# sim.contacts() returns the STRONGEST contacts first and its client default is
# 12. Two feet on the floor plus furniture can fill that list before a
# fingertip is reached, so the default can hide a REAL grasp -- a false
# NEGATIVE, the one error direction this file cannot self-correct. 36 of the
# G1's 66 bodies are hand parts (measured off
# g1_29dof_with_inspire_actuated.xml), so the list has to be wide enough for a
# two-handed grasp plus feet and furniture.
CONTACT_LIMIT = 64
GRAB_CMD = "grab-window"

# ---- what the robot was ASKED to do ---------------------------------------
# The trace is the only MECHANICAL record of the commands issued. final_text is
# the model's claim about them, and pass B's final_text claims "I have walked to
# the white round table" in the same file whose goto_object result reads
# distance_at_start_m 0.0, final_error_m 0.0. Judged on words, that run passed.
#
# TRAVEL_TOOLS is intent to travel. turn() is excluded deliberately: it moves
# the robot (SONIC's turn is a walking turn, not a pivot) but a goto scored ok
# on turn-drift is an accident recorded as navigation.
TRAVEL_TOOLS = frozenset({"walk", "goto", "goto_object", "calibrate_heading"})
DRIFTING_TOOLS = frozenset({"turn"})
POSTURE_TOOLS = frozenset({"set_mode", "set_pose", "set_height", "set_grasp",
                           "set_arms", "set_joints", "reach_to",
                           "release_arms", "recover_from_fall"})
# Kept equal to SonicG1Adapter._motion_tool_names() by a test: a tool renamed
# on one side and not the other silently empties the gate.
MOTION_TOOLS = TRAVEL_TOOLS | DRIFTING_TOOLS | POSTURE_TOOLS
PERCEPTION_TOOLS = frozenset({"look", "describe_scene", "scan"})
CAMERA_TOOLS = frozenset({"look"})

# Substring matching has no polarity: "I do not see a table or a bench" was two
# hits and a pass. The judge does not guess -- a reply carrying any of these is
# refused as unscoreable and sent to a human. That is a deliberate
# false-NEGATIVE: refusing to score is safe, scoring wrong is not.
NEGATION_MARKERS = ("do not see", "don't see", "cannot see", "can't see",
                    "could not see", "couldn't see", "unable to see",
                    "not able to see", "no longer see", "did not see",
                    "didn't see", "i see no", "there is no", "there are no",
                    "nothing visible", "cannot make out", "can't make out",
                    "not visible", "no sign of")
# Ordered longest-claim-first so the reported phrase is the specific one.
TRAVEL_CLAIM_MARKERS = ("i have walked", "i have moved", "i have navigated",
                        "i have travelled", "i have traveled",
                        "i have approached", "i walked", "i moved to",
                        "i navigated", "i approached", "walked to",
                        "walked over", "walked up to")
# Words that describe a thing rather than restate the request. Deliberately
# short and boring: this is an anti-echo tripwire, not a vision check.
DESCRIPTOR_TERMS = (
    "black", "white", "grey", "gray", "silver", "red", "blue", "green",
    "yellow", "orange", "brown", "dark", "chrome", "metallic", "matte",
    "glossy", "shiny", "reflective", "transparent",
    "cylind", "round", "tall", "short", "narrow", "wide", "slim", "conical",
    "rectangul", "curved", "flat",
    "metal", "steel", "aluminium", "aluminum", "plastic", "rubber", "glass",
    "lid", "handle", "spout", "label", "logo", "button", "seam", "ridge",
    "cm", "mm", "inch", "centimet", "millimet", "roughly", "approximately",
    "upright", "standing", "leaning", "beside", "next to", "in front of",
    "to the left", "to the right", "behind")


# ---- pure judges (unit-tested) --------------------------------------------
def named_with_evidence(text: str, names) -> dict:
    """name -> the word in `text` that matched it.

    Two names matched by the SAME word are ONE piece of evidence, not two:
    "laptop" is a synonym of both black_laptop_1_0 and black_laptop_2_0, so a
    single occurrence of that word satisfied the old ">= 2 names" bar by
    itself. The longest matching word wins so the evidence is the most
    specific thing the model actually wrote."""
    low = (text or "").lower()
    out = {}
    for n in names:
        words = set(_SYNONYMS.get(n, ())) | {n.lower()}
        hits = sorted((w for w in words if w in low), key=lambda w: (-len(w), w))
        if hits:
            out[n] = hits[0]
    return out


def names_mentioned(text: str, names) -> list:
    ev = named_with_evidence(text, names)
    return [n for n in names if n in ev]


def reply_text(result) -> str:
    """The model-facing text a judge should read. NEVER str(result) -- that is
    the full BrainResult dataclass repr, which includes every tool-trace
    entry. describe_scene results always name every scene object, so a judge
    reading the repr can never fail the "see" task regardless of what the
    model actually said. BrainResult carries the model's real answer in
    final_text (None when the run never completed), not `text`."""
    return getattr(result, "final_text", None) or ""


def negation_markers(text: str) -> list:
    low = (text or "").lower()
    return [m for m in NEGATION_MARKERS if m in low]


def descriptors_beyond_prompt(text: str, prompt: str = "") -> list:
    """Descriptor words present in the reply and ABSENT from the task prompt.

    The look task's prompt is "Look at the thermos on the table and describe
    it." -- the answer word is IN THE QUESTION, so "I will look at the thermos
    on the table" passed the old judge by restating the request. A descriptor
    the prompt already contains is not evidence the model looked at anything."""
    low = (text or "").lower()
    pl = (prompt or "").lower()
    return [t for t in DESCRIPTOR_TERMS if t in low and t not in pl]


def tools_called(result, *, ok_only=False) -> list:
    """Tool names in call order, off the BrainResult trace. An unreadable trace
    is an EMPTY list -- the gates below then fail closed rather than being
    skipped.

    `ok_only=True` drops the calls the adapter REJECTED. TraceEntry carries
    `ok`, and a refused call is still recorded, so the two views answer
    different questions: the full list is what the robot was ASKED to do (the
    mechanical record, and what judge_goto's "did it even try" gate wants);
    the accepted list is what actually HAPPENED. judge_see needs the second --
    its whole "this was observed, not recalled" gate is the presence of a
    perception call, so a describe_scene that errored satisfied it and the
    answer-from-prior-knowledge degenerate reopened at the cost of one failed
    call. An entry with no readable `ok` is not evidence of success."""
    entries = getattr(result, "trace", None) or ()
    if ok_only:
        entries = [e for e in entries if getattr(e, "ok", False) is True]
    return [str(getattr(e, "name", "")) for e in entries]


def travel_tools_called(called) -> list:
    return [n for n in (called or ()) if n in TRAVEL_TOOLS]


def claim_check(text: str, *, travel_m) -> dict:
    """Compare the model's own words about travelling against the measurement.

    Pass B's committed final_text says "I have walked to the white round
    table ... and stopped directly in front of it". Its own goto_object result,
    in the same artifact, says the robot started at the goal and moved 0.0 m.
    Nothing compared the two, so the hallucination is preserved in a committed
    artifact. `unsupported` is None -- never False -- when travel is
    unmeasurable: an unverified claim must not read as a cleared one."""
    low = (text or "").lower()
    phrase = next((m for m in TRAVEL_CLAIM_MARKERS if m in low), None)
    if phrase is None:
        return {"claimed_travel": False, "phrase": None,
                "measured_travel_m": travel_m, "unsupported": False}
    if travel_m is None:
        return {"claimed_travel": True, "phrase": phrase,
                "measured_travel_m": None, "unsupported": None}
    return {"claimed_travel": True, "phrase": phrase,
            "measured_travel_m": travel_m,
            "unsupported": travel_m < GOTO_MIN_TRAVEL_M}


def furniture_world(pose: dict, objects) -> dict:
    out = {}
    if not (pose and pose.get("available") and pose.get("fresh")):
        return out
    rx = pose["position_m"]["x"]; ry = pose["position_m"]["y"]
    rz = pose["position_m"]["z"]; hd = pose["heading_deg"]
    for o in objects:
        x, y = sonic_world.object_world_xy((rx, ry), hd, o["bearing_deg"],
                                           o["range_m"])
        out[o["name"]] = (x, y, rz + o["height_m"])
    return out


def max_drift_m(before: dict, after: dict, exclude=frozenset()) -> float:
    """Kept as-is for callers that want the bare number. Note what it returns
    for an empty comparison: 0.0. Prefer drift_report()."""
    worst = 0.0
    for n, b in before.items():
        if n in exclude or n not in after:
            continue
        a = after[n]
        worst = max(worst, math.dist((b[0], b[1]), (a[0], a[1])))
    return worst


def drift_report(before: dict, after: dict, exclude=frozenset()) -> dict:
    """Furniture drift, WITH whether it was measurable at all.

    max_drift_m({}, {}) is 0.0 -- the best possible drift number is produced by
    having no measurement. furniture_world() returns {} whenever the pose is
    not fresh, so every unfresh-pose task silently recorded "nothing moved".
    An object that vanishes from the sidecar's list is skipped by the max, so
    shoving something out of view also read as zero; those are named in
    `missing` instead of being dropped."""
    compared, worst, missing = 0, 0.0, []
    for n, b in before.items():
        if n in exclude:
            continue
        a = after.get(n)
        if a is None:
            missing.append(n)
            continue
        compared += 1
        worst = max(worst, math.dist((b[0], b[1]), (a[0], a[1])))
    if compared == 0:
        return {"max_drift_m": None, "compared": 0, "missing": missing,
                "measurable": False, "incomplete": bool(missing),
                "shoved": None, "bulldozed": None}
    worst = round(worst, 3)
    return {"max_drift_m": worst, "compared": compared, "missing": missing,
            "measurable": True, "incomplete": bool(missing),
            "shoved": worst > FURNITURE_SHOVE_WARN_M,
            "bulldozed": worst > FURNITURE_SHOVE_FAIL_M}


def _pose_xy(pose):
    return (pose["position_m"]["x"], pose["position_m"]["y"])


def _pose_usable(pose) -> bool:
    return bool(pose and pose.get("available") and pose.get("fresh"))


def _both_poses_fresh(pose_before, pose_after) -> bool:
    """Travel is None, never 0.0, when either end is unmeasurable. A missing
    measurement must not be reported as the number a stationary robot would
    produce -- that is the same defect as max_drift_m({}, {}) == 0.0."""
    return _pose_usable(pose_before) and _pose_usable(pose_after)


def judge_goto(pose_before: dict, pose_after: dict, target_before, target_after,
               *, called=(), drift=None) -> dict:
    """Did the robot WALK to the target?

    The judge this replaces measured the endpoint alone, and pass A's endpoint
    was already inside the tolerance before the task began: its whole trace was
    get_status -> stop and it scored ok. Four things are now required, and the
    order matters --

      1. both poses must be available and fresh (an endpoint judged against a
         stale reading is a position that may no longer be true),
      2. the target must have been located at BOTH ends and must not have moved
         (it is an estimate built from the robot's own pose),
      3. the start distance must be OUTSIDE the tolerance. If it was not, the
         task cannot measure navigation at all -- that is a broken setup, and
         it is reported INVALID, never ok and never a robot failure. A robot
         cannot fairly be failed for not walking to where it already stood,
      4. only then: upright, a travel tool actually called, real displacement,
         furniture not bulldozed, and the endpoint inside tolerance.

    `travel_m` is straight-line displacement, so a robot that walks a loop back
    to its start reads 0 -- correct for "walk to the table", wrong as a general
    "did it move" test. `closed_m` (how much of the gap it closed) is recorded
    alongside it."""
    def _bad(status, why, **extra):
        return dict({"ok": False, "valid": False, "status": status, "why": why},
                    **extra)

    if not (pose_after and pose_after.get("available")):
        return _bad("invalid_no_pose", "no measured pose")
    if not pose_after.get("fresh"):
        return _bad("invalid_stale_pose", "pose not fresh")
    if not (pose_before and pose_before.get("available")
            and pose_before.get("fresh")):
        return _bad("invalid_no_start_pose",
                    "no fresh pose at task start — travel cannot be measured, "
                    "and endpoint-only judging is the defect being fixed")
    if not target_after:
        return _bad("invalid_no_target", "table never located")
    if not target_before:
        return _bad("invalid_no_start_target",
                    "table not located at task start")

    called = list(called or ())
    travelled = travel_tools_called(called)
    target_moved = math.dist(tuple(target_before)[:2], tuple(target_after)[:2])
    start_d = math.dist(_pose_xy(pose_before), tuple(target_before)[:2])
    end_d = math.dist(_pose_xy(pose_after), tuple(target_after)[:2])
    travel = math.dist(_pose_xy(pose_before), _pose_xy(pose_after))
    tilt = pose_after.get("tilt_deg")
    drift = drift or {}
    facts = {"distance_at_start_m": round(start_d, 2),
             "distance_to_table_m": round(end_d, 2),
             "travel_m": round(travel, 3),
             "closed_m": round(start_d - end_d, 3),
             "target_moved_m": round(target_moved, 3),
             "travel_tools": travelled,
             "motion_tools": [n for n in called if n in MOTION_TOOLS],
             "tilt_deg": tilt,
             "furniture_shoved": drift.get("shoved"),
             "furniture_bulldozed": drift.get("bulldozed"),
             "tolerance_m": GOTO_TOL_M, "min_travel_m": GOTO_MIN_TRAVEL_M}

    def _verdict(ok, status, why=None):
        out = dict(facts)
        out.update({"ok": ok, "valid": True, "status": status})
        if why:
            out["why"] = why
        return out

    def _invalid(status, why):
        out = dict(facts)
        out.update({"ok": False, "valid": False, "status": status, "why": why})
        return out

    if target_moved > GOTO_TARGET_STABLE_M:
        return _invalid("invalid_target_moved",
                        f"the target estimate moved {target_moved:.2f} m during "
                        "the task; start and end distances are not comparable")
    if start_d <= GOTO_TOL_M:
        return _invalid("invalid_no_op",
                        f"the robot was already {start_d:.2f} m from the target "
                        f"at task start, inside the {GOTO_TOL_M} m tolerance. "
                        "Standing still passes this task trivially, so it "
                        "measures nothing. SKIPPED, not ok, not a failure.")
    if tilt is not None and tilt > UPRIGHT_TILT_DEG:
        return _verdict(False, "fail_not_upright",
                        f"tilt {tilt} deg — a robot that is not standing has "
                        "not arrived anywhere")
    if not travelled:
        return _verdict(False, "fail_no_locomotion_call",
                        "no travel tool was called: "
                        f"{sorted(TRAVEL_TOOLS)} are the ones that count "
                        "(turn is excluded — it drifts, it does not navigate)")
    if travel < GOTO_MIN_TRAVEL_M:
        return _verdict(False, "fail_no_travel",
                        f"a travel tool ran but the robot moved {travel:.2f} m")
    if drift.get("bulldozed"):
        return _verdict(False, "fail_shoved_furniture",
                        f"furniture moved {drift.get('max_drift_m')} m — the "
                        "robot walked through the scene, not up to it")
    if end_d > GOTO_TOL_M:
        return _verdict(False, "fail_too_far",
                        f"ended {end_d:.2f} m away, tolerance {GOTO_TOL_M} m")
    return _verdict(True, "ok")


def judge_see(text: str, names, *, called=()) -> dict:
    """Did the model REPORT WHAT IT SAW?

    Two holes closed. First: "Report what you see around you" can be answered
    from prior knowledge of what a lounge contains, with zero tool calls -- the
    old judge only matched substrings in the text, so a hallucinated sentence
    was the cheapest pass. A perception tool must actually have run. Second:
    the >=2-names bar was really a >=1-word bar, because "laptop" matched two
    object names at once. The bar is two distinct pieces of EVIDENCE.

    `saw_with_camera` is recorded separately because describe_scene reads
    MuJoCo ground truth through the sidecar while look reads the camera. "The
    model named the objects from its head camera" is only true of the second,
    and the row now says which channel was available."""
    called = list(called or ())
    perception = [n for n in called if n in PERCEPTION_TOOLS]
    ev = named_with_evidence(text, names)
    named = [n for n in names if n in ev]
    distinct = sorted(set(ev.values()))
    negations = negation_markers(text)
    facts = {"named": named, "evidence": ev,
             "distinct_evidence": len(distinct), "evidence_words": distinct,
             "perception_tools": perception,
             "saw_with_camera": any(n in CAMERA_TOOLS for n in called),
             "negation_markers": negations}
    if not perception:
        return dict(facts, ok=False, valid=False,
                    status="invalid_no_perception_call",
                    why="no perception tool ran — this reply reports prior "
                        "knowledge, not observation")
    if negations:
        return dict(facts, ok=False, valid=False,
                    status="invalid_negation_ambiguous",
                    why=f"reply contains {negations}; substring matching has "
                        "no polarity, so this row is not machine-scoreable")
    if len(distinct) < 2:
        return dict(facts, ok=False, valid=True, status="fail_too_few_objects",
                    why=f"{len(distinct)} distinct object word(s) reported")
    return dict(facts, ok=True, valid=True, status="ok")


def judge_look(text: str, target_names, *, called=(), frame_fresh=False,
               prompt="") -> dict:
    """Did the model LOOK at the target and describe it?

    The old judge passed if the reply contained a thermos synonym -- and
    "thermos" is IN THE TASK PROMPT, so restating the request passed. Its other
    half, "image checked by eye in the report", was a promise that a human
    would judge later, written into machine-readable output as ok: True.

    What this proves: look() ran, a fresh camera frame arrived, the target was
    named, and the reply carries descriptive content the prompt did not supply.
    What it does NOT prove: that the description matches the image. That stays
    a human job and the row says so instead of pretending otherwise."""
    called = list(called or ())
    ev = named_with_evidence(text, target_names)
    descriptors = descriptors_beyond_prompt(text, prompt)
    negations = negation_markers(text)
    facts = {"named": [n for n in target_names if n in ev],
             "descriptors": descriptors,
             "look_called": any(n in CAMERA_TOOLS for n in called),
             "frame_fresh": bool(frame_fresh),
             "negation_markers": negations,
             "human_review": "required",
             "proves": "look() ran and a fresh frame arrived; whether the "
                       "description matches the image is not machine-checked"}
    if not facts["look_called"]:
        return dict(facts, ok=False, valid=False, status="invalid_no_look_call",
                    why="look() never ran — nothing was looked at")
    if not frame_fresh:
        return dict(facts, ok=False, valid=False,
                    status="invalid_no_fresh_frame",
                    why="no fresh camera frame this task (stale or absent) — a "
                        "description of an image that never arrived is not a look")
    if negations:
        return dict(facts, ok=False, valid=False,
                    status="invalid_negation_ambiguous",
                    why=f"reply contains {negations}; not machine-scoreable")
    if not facts["named"]:
        return dict(facts, ok=False, valid=True, status="fail_target_not_named",
                    why="the target object is never named in the reply")
    if not descriptors:
        return dict(facts, ok=False, valid=True, status="fail_no_description",
                    why="no descriptive term the prompt did not already "
                        "contain — this reply restates the request")
    return dict(facts, ok=True, valid=True, status="ok")


def judge_pick(contacts: dict, z_before, z_after, *, lift_source="robot_relative") -> dict:
    """Did the robot GRASP AND LIFT the thermos?

    `lift_source` is the guard that matters. furniture_world() computes the
    object's z as telemetry pose z (rt/odostate, 3 dp) PLUS the sidecar's
    height_m (xpos_z - qpos_z, 2 dp) -- two different sources sampled at two
    different instants, so the pelvis term does not cancel. A robot that stands
    up between the two reads adds its own height change straight into lift_m,
    and pass B's pick trace calls set_height and set_mode. A grasp claim must
    never rest on a height the robot controls, so a qualifying lift from that
    path is reported INVALID rather than certified.

    lift_source="world" is sim.body_pose(...)["bodies"][name]["world_m"][2]:
    MuJoCo's own xpos, one read, no robot pose in the arithmetic. Its ABSOLUTE
    value is wrong (every lounge body keeps the source scan origin as its body
    frame), but a rigid body translates rigidly, so the DELTA is a true lift."""
    rows = (contacts or {}).get("contacts", [])
    thermos = [c for c in rows if c.get("touching") == "black_thermos_0"]
    hand = [c for c in thermos
            if any(h in str(c.get("robot_part", "")).lower() for h in HAND_PARTS)]
    # Visible but never ok-qualifying: a wrist touching the object is not a
    # grasp, but recording the count makes a wrist-scoop legible in the row
    # instead of silently indistinguishable from "no contact at all".
    wrist = [c for c in thermos
             if any(w in str(c.get("robot_part", "")).lower() for w in WRIST_PARTS)]
    forces = [c.get("force_n") for c in hand]
    known = [float(f) for f in forces if isinstance(f, (int, float))]
    force_unknown = len(known) < len(hand)
    lift = (None if z_after is None or z_before is None
            else round(float(z_after) - float(z_before), 3))
    facts = {"hand_contacts": len(hand), "wrist_contacts": len(wrist),
             "hand_parts": sorted({str(c.get("robot_part")) for c in hand}),
             # >=2 distinct hand parts is what opposition looks like. Recorded,
             # never gated: nobody has measured how this scene's Inspire hand
             # contacts aggregate, and a bar invented rather than earned is the
             # thing this file exists to prevent.
             "opposition": len({str(c.get("robot_part")) for c in hand}) >= 2,
             "max_hand_force_n": max(known) if known else None,
             "force_unknown": force_unknown,
             "lift_m": lift, "lift_source": lift_source}
    if not hand:
        return dict(facts, ok=False, valid=True, status="fail_no_hand_contact",
                    why="no hand part is touching the thermos")
    if known and not force_unknown and max(known) < PICK_MIN_FORCE_N:
        return dict(facts, ok=False, valid=True, status="fail_graze",
                    why=f"strongest hand contact {max(known)} N < "
                        f"{PICK_MIN_FORCE_N} N — a graze, not a grip")
    if lift is None:
        return dict(facts, ok=False, valid=False, status="invalid_no_height",
                    why="the object's height was unmeasurable at one or both ends")
    if lift < PICK_LIFT_M:
        return dict(facts, ok=False, valid=True, status="fail_no_lift",
                    why=f"lifted {lift} m, bar is {PICK_LIFT_M} m")
    if lift_source != "world":
        return dict(facts, ok=False, valid=False,
                    status="invalid_lift_unverifiable",
                    why="the lift was measured against the robot's own pelvis "
                        "height, which the robot controls. A grasp is not "
                        "certified from a robot-referenced height — re-measure "
                        "with sim.body_pose() world_m.")
    return dict(facts, ok=True, valid=True, status="ok")


def summarise(rows, *, pass_name, max_hops) -> dict:
    """Partition into ok / failed / INVALID.

    An invalid row is not a robot failure and must never be counted as one in
    either direction. pass A's summary.json listed goto_table under "ok"; the
    honest reading is that the task never ran as an experiment. A judge that
    did not say whether it was a valid measurement is not evidence: fail closed
    into invalid."""
    ok, failed, invalid = [], [], []
    for r in rows:
        j = r.get("judge") or {}
        task = r.get("task")
        if j.get("valid") is not True:
            invalid.append(task)
        elif j.get("ok") is True:
            ok.append(task)
        else:
            failed.append(task)
    return {"pass": pass_name, "max_hops": max_hops,
            "judge_version": JUDGE_VERSION,
            "ok": ok, "failed": failed, "invalid": invalid}


# ---- harness ----------------------------------------------------------------
class _FallAccumulator:
    """sim.episode() clears the sidecar's fall latch on read — and the model's
    own get_status tool reads it too. Route every read through this wrapper so
    a mid-task fall is never lost to whoever happened to read first."""
    def __init__(self, read):
        self._read = read
        self.fell = False

    def episode(self):
        e = self._read()
        if e.get("fell_and_was_reset_since_last_check"):
            self.fell = True
        return e


def _console():
    spec = importlib.util.spec_from_file_location(
        "sonic_console", REPO / "scripts" / "sonic_console.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)           # defs only; main() is __main__-guarded
    return mod


def _shot(outdir: pathlib.Path, stem: str) -> dict:
    """grab-window screenshot, verified. A failure is a named row, never a
    silent reuse of a stale file."""
    path = outdir / f"{stem}_window.png"
    t0 = time.time()
    try:
        r = subprocess.run([GRAB_CMD, "mujoco", str(path)],
                           capture_output=True, text=True)
    except OSError as e:
        return {"file": str(path), "ok": False,
                "note": f"grab-window failed to launch: {e}"}
    fresh = path.exists() and path.stat().st_mtime >= t0
    return {"file": str(path), "ok": r.returncode == 0 and fresh,
            "note": None if (r.returncode == 0 and fresh)
            else f"grab-window rc={r.returncode} fresh={fresh}"}


def _object_world_z(sim, name):
    """The object's world z straight from MuJoCo, with NO robot pose in the
    arithmetic — sim.body_pose() is d.body(name).xpos. Returns
    (z_or_None, source). The absolute value is wrong (every lounge body keeps
    the source scan origin as its body frame origin, so the thermos reads at
    floor height while sitting on the table) but the body translates rigidly,
    so the DELTA between two of these reads is a true lift.

    Falls back to "robot_relative" so a sidecar without body_pose still
    produces a row -- judge_pick then refuses to certify it rather than
    silently trusting a pelvis-referenced height."""
    try:
        b = (sim.body_pose([name]).get("bodies") or {}).get(name)
        if b and b.get("world_m"):
            return float(b["world_m"][2]), "world"
    except Exception:
        pass
    return None, "robot_relative"


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--pass-name", required=True, choices=["A", "B"])
    p.add_argument("--max-hops", type=int, default=12)
    p.add_argument("--model", default="gemini-robotics-er-2-preview")
    p.add_argument("--outdir", default="artifacts/lounge_scene")
    args = p.parse_args()
    outdir = REPO / args.outdir / f"pass{args.pass_name}"
    outdir.mkdir(parents=True, exist_ok=True)

    c = _console()
    link = c.ZmqSonicLink(port=5556, settle_s=1.0, auto_start=False)
    telemetry = sim = adapter = None
    rows = []
    try:
        link.start_control()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:      # park IDLE (console.main idiom)
            link._pub.send(c.encode_planner(c.MODE_IDLE, (1.0, 0.0, 0.0),
                                            (1.0, 0.0, 0.0), 0.0, -1.0))
            time.sleep(0.05)
        sim = c._open_sim(5570)
        if sim is None:
            print("FATAL: no sidecar — the battery cannot measure anything")
            return 2
        # Every reader of the fall latch -- this harness's post-task check AND
        # the model's own get_status tool -- must route through one
        # accumulator, or whichever one reads first silently clears it for
        # the other (simbridge/sidecar.py:601-608).
        falls = _FallAccumulator(sim.episode)
        sim.episode = falls.episode
        c.release_band(sim=sim)
        telemetry = c._open_telemetry()
        if telemetry is None:
            print("FATAL: no telemetry — success cannot be judged")
            return 2
        try:
            sim.episode()                        # drain bring-up fall latch --
            # bring-up sometimes drops the robot as the band lets go (mirrors
            # sonic_console.py's guarded drain, scripts/sonic_console.py:493-499)
        except Exception:
            pass
        adapter = c.SonicG1Adapter(link, c._Guard(), telemetry=telemetry, sim=sim)
        adapter.start_fall_watchdog()
        brain = c.Brain(adapter, c.GeminiClient(c.load_gemini_key(),
                                                model=args.model),
                        c.SYSTEM, max_hops=args.max_hops)

        for key, prompt in TASKS:
            try:
                # Reset FIRST: an exception in the pre-reads must not leak the
                # previous task's fall into this one.
                falls.fell = False
                pose0 = telemetry.pose()
                objs0 = sim.objects().get("objects", [])
                world0 = furniture_world(pose0, objs0)
                # This task's own reads only. A target carried over from an
                # earlier task was measured in a frame that has since moved.
                table0 = (world0.get("white_round_table_0") or (None, None))[:2]
                if table0[0] is None:
                    table0 = None
                zb, zb_src = _object_world_z(sim, "black_thermos_0")
                if zb is None:
                    zb = (world0.get("black_thermos_0") or (None, None, None))[2]
                frame0 = getattr(adapter, "last_frame", None)
                t0 = time.time()
                result = brain.run(prompt)
                elapsed = round(time.time() - t0, 1)
                full_repr = str(result)     # committed evidence: every trace entry
                text = reply_text(result)   # what the model actually said, judged
                called = tools_called(result)   # what the robot was ASKED to do
                # ...and what the adapter actually ACCEPTED. judge_see's only
                # evidence that observation happened is the call itself, so a
                # rejected describe_scene must not satisfy it. judge_goto
                # deliberately keeps the full list: "the model never tried to
                # walk" and "the model tried and was refused" are different
                # findings and the second must not read as the first.
                accepted = tools_called(result, ok_only=True)
                pose1 = telemetry.pose()
                objs1 = sim.objects().get("objects", [])
                world1 = furniture_world(pose1, objs1)
                table1 = (world1.get("white_round_table_0") or (None, None))[:2]
                if table1[0] is None:
                    table1 = None
                za, za_src = _object_world_z(sim, "black_thermos_0")
                if za is None:
                    za = (world1.get("black_thermos_0") or (None, None, None))[2]
                # CONTACT_LIMIT, not the client default of 12: the sidecar
                # returns the STRONGEST contacts first, so two feet and the
                # furniture can push a light fingertip contact off a 12-row
                # list and make a real grasp invisible.
                contacts = sim.contacts(limit=CONTACT_LIMIT)
                sim.episode()                 # routes through falls; captures
                                               # both mid-task drains (e.g. the
                                               # model's own get_status) and
                                               # this final read
                fell = falls.fell
                drift = drift_report(world0, world1,
                                     exclude={"black_thermos_0"})
                travel = (round(math.dist(_pose_xy(pose0), _pose_xy(pose1)), 3)
                          if _both_poses_fresh(pose0, pose1) else None)
                frame1 = getattr(adapter, "last_frame", None)
                frame_fresh = frame1 is not None and frame1 is not frame0

                row = {"task": key, "prompt": prompt, "pass": args.pass_name,
                       "judge_version": JUDGE_VERSION,
                       "elapsed_s": elapsed, "reply_chars": len(text),
                       "fell": fell,
                       # the mechanical record of what was commanded, so a
                       # "walked" claim is checkable without a human reading
                       # the trace repr
                       "tools_called": called,
                       "tools_accepted": accepted,
                       "travel_tools": travel_tools_called(called),
                       "hops": getattr(result, "hops", None),
                       "complete": bool(getattr(result, "complete", False)),
                       "travel_m": travel,
                       "claim_check": claim_check(text, travel_m=travel),
                       "drift_m": drift["max_drift_m"],
                       "drift_measurable": drift["measurable"],
                       "drift_missing": drift["missing"]}
                if key == "see":
                    row["judge"] = judge_see(text, SCENE_NAMES, called=accepted)
                elif key == "goto_table":
                    row["judge"] = judge_goto(pose0, pose1, table0, table1,
                                              called=called, drift=drift)
                elif key == "look_thermos":
                    row["judge"] = judge_look(text, ("black_thermos_0",),
                                              called=called,
                                              frame_fresh=frame_fresh,
                                              prompt=prompt)
                elif key == "pick_thermos":
                    row["judge"] = judge_pick(
                        contacts, zb, za,
                        lift_source=("world" if zb_src == za_src == "world"
                                     else "robot_relative"))
                if row["fell"]:
                    row["judge"]["ok"] = False
                    row["judge"]["valid"] = False
                    row["judge"]["status"] = "invalid_fell"
                    row["judge"]["why"] = "robot fell and was reset mid-task"

                # <task>_reply.txt keeps the full trace repr -- committed
                # evidence of every tool call the model made. <task>_final.txt
                # is the clean model-facing text the judges actually score.
                (outdir / f"{key}_reply.txt").write_text(full_repr)
                (outdir / f"{key}_final.txt").write_text(text)
                # frame_fresh was computed BEFORE the judges ran -- judge_look
                # needs it, and a judge that can be passed by a description of
                # an image that never arrived is the defect being fixed.
                if frame_fresh:
                    pov_path = outdir / f"{key}_pov.jpg"
                    pov_path.write_bytes(frame1)
                    row["pov"] = {"file": str(pov_path), "ok": True, "note": None}
                elif frame1 is not None:
                    row["pov"] = {"file": None, "ok": False,
                                  "note": "stale frame from earlier task"}
                else:
                    row["pov"] = {"file": None, "ok": False,
                                  "note": "no camera frame captured this task "
                                          "(model never called look)"}
                row["screenshot"] = _shot(outdir, key)
            except Exception as e:
                row = {"task": key, "prompt": prompt, "pass": args.pass_name,
                       "judge_version": JUDGE_VERSION, "error": repr(e),
                       "fell": falls.fell, "complete": False,
                       "judge": {"ok": False, "valid": False,
                                 "status": "invalid_harness_exception",
                                 "why": "harness exception"}}

            rows.append(row)
            with open(outdir / "battery.jsonl", "a") as f:
                f.write(json.dumps(row) + "\n")
            j = row["judge"]
            print(f"[{key}] {j.get('status')} ok={j.get('ok')} "
                  f"valid={j.get('valid')} elapsed={row.get('elapsed_s')} "
                  f"travel={row.get('travel_m')} drift={row.get('drift_m')} "
                  f"fell={row.get('fell')} tools={row.get('travel_tools')}")
            if j.get("why"):
                print(f"    why: {j['why']}")
            if (row.get("claim_check") or {}).get("unsupported"):
                print(f"    CLAIM UNSUPPORTED: the reply says "
                      f"{row['claim_check']['phrase']!r} and the measured "
                      f"travel is {row['claim_check']['measured_travel_m']} m")
    finally:
        try:
            if adapter is not None:
                adapter.stop_fall_watchdog()
        finally:
            link.close()
            if telemetry is not None:
                telemetry.close()
            if sim is not None:
                sim.close()
    (outdir / "summary.json").write_text(
        json.dumps(summarise(rows, pass_name=args.pass_name,
                             max_hops=args.max_hops), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
