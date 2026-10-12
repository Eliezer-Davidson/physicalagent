"""The battery's judges are pure functions: what counts as success is pinned
here, not decided at runtime. Loaded via importlib because scripts/ is not a
package (same idiom the suite already uses for probe scripts)."""
import importlib.util
import pathlib

import pytest

REPO = pathlib.Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "lounge_battery_probe", REPO / "scripts" / "lounge_battery_probe.py")
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


def test_names_mentioned_is_substring_and_synonym_aware():
    text = "I can see a round table with a black bottle on it, and a bench."
    hit = probe.names_mentioned(text, probe.SCENE_NAMES)
    assert "white_round_table_0" in hit and "grey_and_yellow_bench_0" in hit
    assert "black_thermos_0" in hit          # bottle/flask/thermos all count


def test_furniture_world_uses_pose_plus_relative():
    pose = {"available": True, "fresh": True, "heading_deg": 0.0,
            "position_m": {"x": 1.0, "y": 0.0, "z": 0.79}}
    objects = [{"name": "black_thermos_0", "range_m": 2.0, "bearing_deg": 0.0,
                "height_m": -0.30}]
    w = probe.furniture_world(pose, objects)
    assert w["black_thermos_0"] == pytest.approx((3.0, 0.0, 0.49))


def test_max_drift_ignores_the_target_object():
    before = {"a": (0, 0, 0), "black_thermos_0": (1, 1, 0.5)}
    after = {"a": (0.03, 0, 0), "black_thermos_0": (2, 2, 0.9)}
    assert probe.max_drift_m(before, after, exclude={"black_thermos_0"}) == \
        pytest.approx(0.03)



def test_reply_text_extracts_final_text_not_the_trace_repr():
    # str(BrainResult) is the full dataclass repr -- every tool-trace entry,
    # including describe_scene results that always name every scene object.
    # A judge reading that repr can never fail the "see" task. reply_text()
    # must extract only the model's actual final_text.
    class _FakeIncompleteResult:
        """Shaped like a BrainResult that timed out mid-tool-loop: no final
        answer, but its repr leaks every scene name via the trace -- exactly
        what str(BrainResult) does for a real incomplete run."""
        final_text = None

        def __repr__(self):
            return ("BrainResult(final_text=None, complete=False, hops=40, "
                    "trace=[TraceEntry(name='describe_scene', args={}, ok=True, "
                    "error=None, result={'objects': ["
                    "{'name': 'grey_and_yellow_bench_0'}, "
                    "{'name': 'white_round_table_0'}, "
                    "{'name': 'black_laptop_1_0'}, "
                    "{'name': 'black_laptop_2_0'}, "
                    "{'name': 'black_thermos_0'}]})])")

    fake = _FakeIncompleteResult()
    # the repr alone would let the see judge "pass" -- pin that it does, so
    # the fixture is proven to reproduce the bug before checking the fix.
    assert probe.names_mentioned(str(fake), probe.SCENE_NAMES) == list(probe.SCENE_NAMES)

    extracted = probe.reply_text(fake)
    assert extracted == ""
    assert probe.names_mentioned(extracted, probe.SCENE_NAMES) == []


def test_reply_text_returns_final_text_when_present():
    class _FakeCompleteResult:
        final_text = "I see a table and a bench."

        def __repr__(self):
            return "BrainResult(final_text='I see a table and a bench.', ...)"

    assert probe.reply_text(_FakeCompleteResult()) == "I see a table and a bench."


def test_fall_accumulator_latches_across_multiple_reads():
    # sim.episode() clears the sidecar's fall latch on read, and the model's
    # own get_status tool reads the same latch mid-task -- the accumulator
    # must not lose a fall to whichever reader happened to see it first.
    payloads = iter([
        {"fell_and_was_reset_since_last_check": False},   # bring-up drain
        {"fell_and_was_reset_since_last_check": True},    # mid-task fall (get_status)
        {"fell_and_was_reset_since_last_check": False},   # post-task read: latch
                                                            # already cleared by
                                                            # the mid-task reader
    ])
    falls = probe._FallAccumulator(lambda: next(payloads))
    assert falls.fell is False

    falls.episode()                      # bring-up: no fall
    assert falls.fell is False

    falls.episode()                      # mid-task: fall happens
    assert falls.fell is True

    falls.episode()                      # post-task: someone already drained it
    assert falls.fell is True            # still latched -- not lost


def test_fall_accumulator_reset_then_no_fall_payload_stays_false():
    falls = probe._FallAccumulator(
        lambda: {"fell_and_was_reset_since_last_check": True})
    falls.episode()
    assert falls.fell is True

    falls.fell = False                   # harness resets before the next task
    falls._read = lambda: {"fell_and_was_reset_since_last_check": False}
    falls.episode()
    assert falls.fell is False


def test_shot_handles_missing_grab_window_binary(tmp_path, monkeypatch):
    monkeypatch.setattr(probe, "GRAB_CMD", "definitely-not-a-real-binary-xyz123")
    result = probe._shot(tmp_path, "task")
    assert result["ok"] is False
    assert result["note"]


# ===========================================================================
# JUDGE HARDENING (2026-08-18)
#
# The defect these tests exist for: judge_goto returned {"ok": True} for pass A,
# whose entire trace was get_status -> stop. The robot never moved. The judge
# measured the ENDPOINT only, and the endpoint was already inside tolerance
# before the task began -- so the cheapest way to pass "walk to the table" was
# to stand still. Per the project stoplist: "a scalar that cannot distinguish
# success from failure will be optimised into failure."
#
# Each test below names the specific degenerate behaviour it forecloses.
# ===========================================================================

# ---- trace reading: what the robot was ASKED to do -------------------------

class _FakeTrace:
    """Shaped like a BrainResult: final_text plus an ordered tool trace.

    An entry may be a bare name (assumed accepted) or a (name, ok) pair --
    the real TraceEntry carries ok, and a call the adapter REJECTED is still
    recorded, which is the distinction tools_called(ok_only=True) exists for.
    """
    def __init__(self, names, final_text=None, complete=True):
        self.final_text = final_text
        self.complete = complete
        self.trace = []
        for n in names:
            name, ok = n if isinstance(n, tuple) else (n, True)
            self.trace.append(
                type("E", (), {"name": name, "args": {}, "ok": ok})())


def test_tools_called_reads_the_trace_in_order():
    r = _FakeTrace(["get_status", "stop"])
    assert probe.tools_called(r) == ["get_status", "stop"]


def test_tools_called_survives_a_result_with_no_trace():
    # A harness exception or an SDK shape change must not KeyError the judge
    # into silence -- an unreadable trace is an EMPTY call list, which fails
    # the locomotion gate closed rather than skipping it.
    assert probe.tools_called(object()) == []
    assert probe.tools_called(None) == []


def test_travel_tools_excludes_turn():
    # SONIC's turn() is a walking turn that drifts ~0.3 m forward per task
    # (measured across the 'see' task in both passes). It moves the robot, so
    # it is a MOTION tool -- but it is never evidence of intent to travel, and
    # a goto judged ok on turn-drift alone would be an accident scored as
    # navigation.
    assert "turn" not in probe.TRAVEL_TOOLS
    assert "turn" in probe.MOTION_TOOLS
    assert probe.travel_tools_called(["describe_scene", "turn", "look"]) == []
    assert probe.travel_tools_called(["turn", "goto_object"]) == ["goto_object"]


def test_motion_tool_names_stay_in_sync_with_the_adapter():
    # The gate is a name list. If a tool is renamed in the adapter and not
    # here, the gate silently stops seeing locomotion and every goto becomes
    # "no locomotion call" -- or worse, a new travel tool appears that the
    # gate does not count. Pin the two lists together.
    sonic_g1 = pytest.importorskip("physicalagent.robots.sonic_g1")
    adapter_motion = sonic_g1.SonicG1Adapter._motion_tool_names(None)
    assert probe.MOTION_TOOLS <= adapter_motion, (
        f"probe knows motion tools the adapter does not: "
        f"{sorted(probe.MOTION_TOOLS - adapter_motion)}")
    assert adapter_motion <= probe.MOTION_TOOLS, (
        f"adapter has motion tools the probe's gate cannot see: "
        f"{sorted(adapter_motion - probe.MOTION_TOOLS)}")
    assert probe.TRAVEL_TOOLS <= adapter_motion


def test_perception_tools_stay_in_sync_with_the_adapter():
    sonic_g1 = pytest.importorskip("physicalagent.robots.sonic_g1")
    allowed = sonic_g1.SonicG1Adapter._always_allowed_tools(None)
    assert probe.PERCEPTION_TOOLS <= allowed


# ---- judge_goto: the no-op-passes defect ----------------------------------

def _pose(x, y, *, fresh=True, available=True, tilt=2.4, z=0.78):
    return {"available": available, "fresh": fresh, "heading_deg": 0.0,
            "tilt_deg": tilt, "position_m": {"x": x, "y": y, "z": z}}


def test_judge_goto_regression_pass_a_no_op_must_not_return_plain_ok():
    """THE regression test. Reproduces pass A's goto_table exactly:

      artifacts/lounge_scene/passA/battery.jsonl:
        {"task":"goto_table", ..., "judge":{"ok":true,"distance_to_table_m":0.36}}
      artifacts/lounge_scene/passA/goto_table_reply.txt:
        trace=[TraceEntry(name='get_status'...), TraceEntry(name='stop'...)]
        final_text='I have stopped and am already standing directly in front of
                    the white round table ... just 0.35 meters ahead of me.'

    Robot started 0.35 m from the table, called ZERO locomotion tools, and
    ended 0.36 m away. The old judge scored that ok. The new one must not.
    """
    table = (0.01, -0.67)
    before = _pose(0.01, -0.32)       # 0.35 m from the table
    after = _pose(0.01, -0.31)        # 0.36 m from the table, 0.01 m travelled
    v = probe.judge_goto(before, after, table, table,
                         called=["get_status", "stop"])
    assert v["ok"] is False
    assert v["valid"] is False                       # not a measured failure --
    assert v["status"] == "invalid_no_op"            # an invalid experiment
    assert v["distance_at_start_m"] == pytest.approx(0.35, abs=0.01)
    assert v["distance_to_table_m"] == pytest.approx(0.36, abs=0.01)
    assert v["travel_m"] == pytest.approx(0.01, abs=0.005)
    assert v["travel_tools"] == []


def test_judge_goto_regression_pass_b_zero_travel_goto_object_call():
    """Pass B's goto_table: goto_object WAS called, and its own committed
    result reads distance_at_start_m: 0.0, start == goal == end, final_error
    0.0 -- standoff_point() correctly returned robot_xy because the robot was
    already inside the 0.6 m standoff. A locomotion CALL is not locomotion.
    """
    table = (0.01, -0.67)
    before = _pose(0.24, -0.39)       # 0.28 m from the table
    after = _pose(0.24, -0.39)        # did not move at all
    v = probe.judge_goto(before, after, table, table,
                         called=["goto_object", "describe_scene", "look"])
    assert v["ok"] is False
    assert v["status"] == "invalid_no_op"
    assert v["travel_tools"] == ["goto_object"]      # called, and it did nothing
    assert v["travel_m"] == pytest.approx(0.0, abs=1e-9)


def test_judge_goto_fails_a_run_that_called_nothing_and_did_not_travel():
    # Start OUTSIDE tolerance so the experiment is valid, then do nothing.
    # This is the honest failure the no-op case would have been if the setup
    # had not already been inside the bar.
    table = (0.0, -0.67)
    v = probe.judge_goto(_pose(0.0, 1.5), _pose(0.0, 1.49), table, table,
                         called=["get_status", "stop"])
    assert v["ok"] is False
    assert v["valid"] is True                        # a real, measured failure
    assert v["status"] == "fail_no_locomotion_call"


def test_judge_goto_fails_when_a_locomotion_tool_ran_but_the_robot_stayed_put():
    # goto_object called, refused/failed/timed out, robot did not travel.
    table = (0.0, -0.67)
    v = probe.judge_goto(_pose(0.0, 1.5), _pose(0.05, 1.48), table, table,
                         called=["goto_object"])
    assert v["ok"] is False
    assert v["status"] == "fail_no_travel"
    assert v["travel_m"] < probe.GOTO_MIN_TRAVEL_M


def test_judge_goto_passes_a_real_walk():
    table = (0.0, -0.67)
    v = probe.judge_goto(_pose(0.0, 1.5), _pose(0.0, 0.1), table, table,
                         called=["describe_scene", "goto_object", "get_status"])
    assert v["ok"] is True
    assert v["valid"] is True
    assert v["status"] == "ok"
    assert v["travel_m"] == pytest.approx(1.4, abs=0.01)
    assert v["closed_m"] == pytest.approx(1.4, abs=0.01)


def test_judge_goto_fails_a_walk_that_ends_too_far_away():
    table = (0.0, -0.67)
    v = probe.judge_goto(_pose(0.0, 3.0), _pose(0.0, 1.2), table, table,
                         called=["walk"])
    assert v["ok"] is False
    assert v["status"] == "fail_too_far"


def test_judge_goto_tolerance_is_tighter_than_the_standoff_it_judges():
    # goto_object stops at a 0.6 m standoff (MIN_STANDOFF_M 0.4). A tolerance
    # of 1.0 m was 2.5x that: it could not tell "stopped in front of the table"
    # from "somewhere else in a 1.5 m room". The bar must be within reach of
    # the standoff it is judging.
    sonic_world = pytest.importorskip("physicalagent.robots.sonic_world")
    assert probe.GOTO_TOL_M < 1.0
    assert probe.GOTO_TOL_M >= sonic_world.MIN_STANDOFF_M


def test_judge_goto_refuses_a_stale_pose():
    # PRESERVED from the pre-hardening suite (the exact-dict assertion was
    # loosened only because the judge now reports more facts; the REFUSAL is
    # unchanged and must never regress). A pose that is "available" but not
    # "fresh" is a stale reading -- judging goto against it would pass on a
    # position that may no longer be true.
    stale = _pose(0.2, -1.0, fresh=False)
    v = probe.judge_goto(_pose(0.2, 2.0), stale, (0.2, -1.8), (0.2, -1.8),
                         called=["goto"])
    assert v["ok"] is False
    assert v["valid"] is False
    assert v["status"] == "invalid_stale_pose"
    assert v["why"] == "pose not fresh"


def test_judge_goto_refuses_an_unavailable_pose():
    v = probe.judge_goto(_pose(0.2, 2.0), {"available": False},
                         (0.2, -1.8), (0.2, -1.8), called=["goto"])
    assert v["ok"] is False
    assert v["status"] == "invalid_no_pose"
    assert v["why"] == "no measured pose"


def test_judge_goto_refuses_a_stale_START_pose():
    # Without a trustworthy pose at t0 there is no start distance and no
    # displacement -- the travel requirement cannot be evaluated at all, and
    # the endpoint-only judgement is exactly the defect being fixed.
    v = probe.judge_goto(_pose(0.2, 2.0, fresh=False), _pose(0.2, -1.0),
                         (0.2, -1.8), (0.2, -1.8), called=["goto"])
    assert v["ok"] is False
    assert v["valid"] is False
    assert v["status"] == "invalid_no_start_pose"


def test_judge_goto_refuses_when_the_target_was_never_located():
    v = probe.judge_goto(_pose(0.0, 2.0), _pose(0.0, 0.1), None, None,
                         called=["goto"])
    assert v["ok"] is False
    assert v["status"] == "invalid_no_target"
    assert v["why"] == "table never located"


def test_judge_goto_refuses_when_the_target_moved_during_the_task():
    # The target is an ESTIMATE derived from the robot's own pose. If it moves
    # (odometry drift, or the robot shoving the furniture), the start and end
    # distances are measured against two different points and are not
    # comparable. Refuse rather than difference them.
    v = probe.judge_goto(_pose(0.0, 2.0), _pose(0.0, 0.1),
                         (0.0, -0.67), (0.0, -0.20), called=["goto"])
    assert v["ok"] is False
    assert v["valid"] is False
    assert v["status"] == "invalid_target_moved"
    assert v["target_moved_m"] == pytest.approx(0.47, abs=0.01)


def test_judge_goto_refuses_a_robot_that_is_not_upright():
    # No judge in the battery checked posture. A robot lying on the floor
    # 0.5 m from the table has a pelvis inside tolerance. `fell` only catches
    # falls the sidecar LATCHED and reset; a robot that is merely toppled and
    # not reset was scored as an arrival.
    v = probe.judge_goto(_pose(0.0, 2.0), _pose(0.0, 0.1, tilt=78.0),
                         (0.0, -0.67), (0.0, -0.67), called=["goto"])
    assert v["ok"] is False
    assert v["status"] == "fail_not_upright"
    assert v["tilt_deg"] == 78.0


def test_judge_goto_fails_a_robot_that_bulldozed_the_furniture():
    # "Walk to the table and stop in front of it" is not satisfied by shoving
    # the table across the room. drift was measured and recorded on every row
    # and gated nothing.
    drift = probe.drift_report({"a": (0.0, 0.0, 0.0)}, {"a": (0.4, 0.0, 0.0)})
    v = probe.judge_goto(_pose(0.0, 2.0), _pose(0.0, 0.1),
                         (0.0, -0.67), (0.0, -0.67), called=["goto"], drift=drift)
    assert v["ok"] is False
    assert v["status"] == "fail_shoved_furniture"


def test_judge_goto_only_warns_on_small_furniture_drift():
    # Small drift is dominated by odometry error in an estimate-of-an-estimate,
    # so it must warn, never fail. The observed range across both passes was
    # 0.009-0.025 m on tasks where nothing was touched.
    drift = probe.drift_report({"a": (0.0, 0.0, 0.0)}, {"a": (0.08, 0.0, 0.0)})
    v = probe.judge_goto(_pose(0.0, 2.0), _pose(0.0, 0.1),
                         (0.0, -0.67), (0.0, -0.67), called=["goto"], drift=drift)
    assert v["ok"] is True
    assert v["furniture_shoved"] is True
    assert v["furniture_bulldozed"] is False


def test_judge_goto_reports_no_op_before_it_reports_no_travel():
    # Ordering is a fairness claim: a robot cannot be FAILED for not walking
    # when it was already standing at the goal. That is a broken setup, and
    # calling it a robot failure would corrupt the record in the other
    # direction.
    table = (0.0, -0.67)
    v = probe.judge_goto(_pose(0.0, 0.0), _pose(0.0, 0.0), table, table,
                         called=[])
    assert v["status"] == "invalid_no_op"      # not "fail_no_locomotion_call"


# ---- the walk claim vs the measured trace ---------------------------------

def test_claim_check_catches_the_pass_b_hallucination_verbatim():
    """Pass B's committed final_text, verbatim from
    artifacts/lounge_scene/passB/goto_table_reply.txt. Its own goto_object
    result in the same file reads distance_at_start_m 0.0, final_error_m 0.0.
    The claim is false and it is preserved in a committed artifact because
    nothing mechanically compared the words to the measurement."""
    text = ("I have walked to the white round table (coffee table) and stopped "
            "directly in front of it. It is now about 0.37 meters away, well "
            "within reach.")
    c = probe.claim_check(text, travel_m=0.0)
    assert c["claimed_travel"] is True
    assert c["unsupported"] is True
    assert c["phrase"] == "i have walked"


def test_claim_check_clears_pass_a_which_did_not_claim_a_walk():
    # Pass A's model was HONEST -- it said "already standing", not "walked".
    # The check must not manufacture a hallucination where there was none.
    text = ("I have stopped and am already standing directly in front of the "
            "white round table (coffee table), which is positioned just 0.35 "
            "meters ahead of me.")
    c = probe.claim_check(text, travel_m=0.01)
    assert c["claimed_travel"] is False
    assert c["unsupported"] is False


def test_claim_check_clears_a_claim_the_measurement_supports():
    c = probe.claim_check("I have walked to the table.", travel_m=1.4)
    assert c["claimed_travel"] is True
    assert c["unsupported"] is False


def test_claim_check_treats_an_unmeasurable_travel_as_unverified_not_ok():
    c = probe.claim_check("I have walked to the table.", travel_m=None)
    assert c["claimed_travel"] is True
    assert c["unsupported"] is None       # unknown, never silently "fine"


# ---- judge_see: naming without looking ------------------------------------

def test_judge_see_rejects_a_reply_that_never_called_a_perception_tool():
    # DEGENERATE: "Report what you see around you" can be answered from prior
    # knowledge of a lounge with zero tool calls. The old judge only read
    # substrings out of the text.
    v = probe.judge_see("There is a table and a bench here.", probe.SCENE_NAMES,
                        called=["get_status"])
    assert v["ok"] is False
    assert v["valid"] is False
    assert v["status"] == "invalid_no_perception_call"


def test_judge_see_accepts_a_reply_backed_by_a_perception_call():
    v = probe.judge_see("I see a round table and a bench.", probe.SCENE_NAMES,
                        called=["describe_scene", "look"])
    assert v["ok"] is True
    assert v["status"] == "ok"


def test_judge_see_records_which_perception_channel_was_used():
    # describe_scene reads MuJoCo ground truth through the sidecar; look reads
    # the camera. "The model named the objects from its head camera" is only
    # true of the second. The row must say which was available so the claim
    # can be checked instead of assumed.
    v = probe.judge_see("A table and a bench.", probe.SCENE_NAMES,
                        called=["describe_scene"])
    assert v["perception_tools"] == ["describe_scene"]
    assert v["saw_with_camera"] is False
    v2 = probe.judge_see("A table and a bench.", probe.SCENE_NAMES,
                         called=["describe_scene", "look"])
    assert v2["saw_with_camera"] is True


def test_judge_see_one_synonym_cannot_satisfy_two_objects():
    # DEGENERATE: "laptop" is a synonym of BOTH black_laptop_1_0 and
    # black_laptop_2_0, so the single word "laptop" produced TWO names and met
    # the >=2 bar by itself. The bar is two pieces of EVIDENCE, not two names.
    v = probe.judge_see("I can see a laptop.", probe.SCENE_NAMES,
                        called=["look"])
    assert v["named"] == ["black_laptop_1_0", "black_laptop_2_0"]
    assert v["distinct_evidence"] == 1
    assert v["ok"] is False
    assert v["status"] == "fail_too_few_objects"


def test_judge_see_two_distinct_words_pass():
    v = probe.judge_see("I can see a laptop on a table.", probe.SCENE_NAMES,
                        called=["look"])
    assert v["distinct_evidence"] == 2
    assert v["ok"] is True


def test_judge_see_refuses_to_score_a_negated_report():
    # DEGENERATE: "I do not see a table or a bench" is two substring hits and
    # scored ok. Substring matching has no polarity. The judge must not guess
    # -- it refuses to score and sends the row to a human.
    v = probe.judge_see("I do not see a table or a bench anywhere.",
                        probe.SCENE_NAMES, called=["look"])
    assert v["ok"] is False
    assert v["valid"] is False
    assert v["status"] == "invalid_negation_ambiguous"
    assert v["negation_markers"] == ["do not see"]


def test_judge_see_negation_guard_is_conservative_not_clever():
    # A mixed reply ("I see X; I cannot see Y") is also refused. That is a
    # deliberate false-NEGATIVE: refusing to score is safe, scoring wrong is
    # not. Pinned so nobody "improves" it into a polarity parser by accident.
    v = probe.judge_see("I see a table and a bench, but I cannot see the laptop.",
                        probe.SCENE_NAMES, called=["look"])
    assert v["status"] == "invalid_negation_ambiguous"


def test_judge_see_real_pass_a_reply_still_passes():
    # Guard against over-tightening: the honest evidence is that the model DID
    # name all five objects. That result must survive the hardening.
    text = ("Directly in front of me is a white round table. On it are two "
            "black laptops and a black thermos. Behind the table is a grey "
            "and yellow bench.")
    v = probe.judge_see(text, probe.SCENE_NAMES, called=["describe_scene", "look"])
    assert v["ok"] is True
    assert set(v["named"]) == set(probe.SCENE_NAMES)


# ---- judge_look: the prompt contains the answer ---------------------------

LOOK_PROMPT = "Look at the thermos on the table and describe it."


def test_judge_look_rejects_a_reply_that_only_echoes_the_prompt():
    # DEGENERATE AND CHEAPEST IN THE FILE: the old judge passed if the reply
    # contained a thermos synonym -- and "thermos" is IN THE TASK PROMPT. "I
    # will look at the thermos on the table." passes by restating the request.
    v = probe.judge_look("I will look at the thermos on the table.",
                         ("black_thermos_0",), called=["look"],
                         frame_fresh=True, prompt=LOOK_PROMPT)
    assert v["ok"] is False
    assert v["status"] == "fail_no_description"


def test_judge_look_requires_the_look_tool_to_have_run():
    v = probe.judge_look("It is a tall silver cylinder with a black lid.",
                         ("black_thermos_0",), called=["describe_scene"],
                         frame_fresh=True, prompt=LOOK_PROMPT)
    assert v["ok"] is False
    assert v["valid"] is False
    assert v["status"] == "invalid_no_look_call"


def test_judge_look_requires_a_fresh_camera_frame():
    # The probe already measured this and threw it away: row["pov"] recorded
    # "stale frame from earlier task" / "no camera frame captured this task"
    # while row["judge"]["ok"] stayed True. A description of an image that
    # never arrived is not a look.
    v = probe.judge_look("It is a tall silver cylinder with a black lid.",
                         ("black_thermos_0",), called=["look"],
                         frame_fresh=False, prompt=LOOK_PROMPT)
    assert v["ok"] is False
    assert v["valid"] is False
    assert v["status"] == "invalid_no_fresh_frame"


def test_judge_look_passes_a_real_description():
    v = probe.judge_look(
        "The thermos is a tall silver cylinder, roughly 25 cm high, with a "
        "dark lid, standing to the right of the laptop.",
        ("black_thermos_0",), called=["look"], frame_fresh=True,
        prompt=LOOK_PROMPT)
    assert v["ok"] is True
    assert v["status"] == "ok"
    assert v["descriptors"]


def test_judge_look_never_claims_the_description_is_true():
    # The judge proves that look ran and a fresh frame arrived. It cannot
    # prove the words match the image. That limit is written into the row so
    # "ok" is never read as "the model saw it correctly".
    v = probe.judge_look(
        "The thermos is a tall silver cylinder with a dark lid.",
        ("black_thermos_0",), called=["look"], frame_fresh=True,
        prompt=LOOK_PROMPT)
    assert v["ok"] is True
    assert v["human_review"] == "required"
    assert "not machine-checked" in v["proves"]


def test_judge_look_fails_when_the_target_is_never_named():
    v = probe.judge_look("It is a tall silver cylinder with a dark lid.",
                         ("black_thermos_0",), called=["look"],
                         frame_fresh=True, prompt=LOOK_PROMPT)
    assert v["ok"] is False
    assert v["status"] == "fail_target_not_named"


def test_judge_look_descriptors_must_be_absent_from_the_prompt():
    # The anti-echo rule: a descriptor that appears in the prompt is not
    # evidence the model looked at anything.
    assert probe.descriptors_beyond_prompt("a table", LOOK_PROMPT) == []
    assert "silver" in probe.descriptors_beyond_prompt(
        "a silver flask", LOOK_PROMPT)


# ---- drift: a missing measurement must not read as zero -------------------

def test_drift_report_marks_an_unmeasurable_comparison():
    # DEGENERATE: furniture_world() returns {} when the pose is not fresh, and
    # max_drift_m({}, {}) returns 0.0 -- the best possible drift number is
    # produced by having NO measurement at all. Every unfresh-pose task
    # silently recorded "nothing moved".
    r = probe.drift_report({}, {})
    assert r["measurable"] is False
    assert r["max_drift_m"] is None
    assert r["shoved"] is None
    assert r["bulldozed"] is None
    assert probe.max_drift_m({}, {}) == 0.0      # the old function, unchanged


def test_drift_report_lists_objects_that_vanished_from_the_scene():
    # An object that drops out of the sidecar's list is skipped by the max, so
    # shoving something out of view read as zero drift.
    r = probe.drift_report({"a": (0, 0, 0), "b": (1, 1, 0)},
                           {"a": (0.02, 0, 0)})
    assert r["max_drift_m"] == pytest.approx(0.02)
    assert r["missing"] == ["b"]
    assert r["incomplete"] is True


def test_drift_report_honours_the_exclude_set():
    r = probe.drift_report({"a": (0, 0, 0), "black_thermos_0": (1, 1, 0.5)},
                           {"a": (0.03, 0, 0), "black_thermos_0": (2, 2, 0.9)},
                           exclude={"black_thermos_0"})
    assert r["max_drift_m"] == pytest.approx(0.03)
    assert r["compared"] == 1
    assert r["incomplete"] is False


# ---- judge_pick: never let a false grasp through --------------------------

def test_judge_pick_refuses_a_lift_measured_against_the_robots_own_pelvis():
    """THE false-grasp guard. furniture_world() computes the object's z as
    telemetry pose z (rt/odostate, 3 dp) + sidecar height_m (xpos_z - qpos_z,
    2 dp) -- two different sources sampled at two different instants. The
    pelvis term does not cancel, so a robot that STANDS UP between the two
    reads adds its own height change straight into lift_m. pass B's pick trace
    calls set_height and set_mode, so this is not hypothetical.

    A grasp claim must never rest on a height the robot controls. When the
    contacts and the lift both qualify but the lift came from the
    pelvis-referenced path, the judge refuses to certify it.
    """
    contacts = {"contacts": [{"robot_part": "right_hand_index_1",
                              "touching": "black_thermos_0", "force_n": 4.0}]}
    v = probe.judge_pick(contacts, 0.49, 0.56, lift_source="robot_relative")
    assert v["ok"] is False
    assert v["valid"] is False
    assert v["status"] == "invalid_lift_unverifiable"
    assert v["hand_contacts"] == 1
    assert v["lift_m"] == pytest.approx(0.07)


def test_judge_pick_certifies_a_lift_measured_in_the_world_frame():
    # sim.body_pose(["black_thermos_0"])["bodies"][...]["world_m"][2] is
    # MuJoCo's own xpos: no robot pose in the arithmetic, one read, 4 dp.
    # The body origin is at the scan origin (so the ABSOLUTE z is wrong), but
    # the body translates rigidly, so the DELTA is a true lift.
    contacts = {"contacts": [{"robot_part": "right_hand_index_1",
                              "touching": "black_thermos_0", "force_n": 4.0}]}
    v = probe.judge_pick(contacts, 0.0026, 0.0726, lift_source="world")
    assert v["ok"] is True
    assert v["status"] == "ok"


def test_judge_pick_fails_a_grazing_contact():
    # DEGENERATE: one fingertip resting on the object at 0.1 N is a touch, not
    # a grasp. force_n was in every contact row and no judge read it.
    contacts = {"contacts": [{"robot_part": "right_hand_index_1",
                              "touching": "black_thermos_0", "force_n": 0.1}]}
    v = probe.judge_pick(contacts, 0.0, 0.07, lift_source="world")
    assert v["ok"] is False
    assert v["status"] == "fail_graze"
    assert v["max_hand_force_n"] == pytest.approx(0.1)


def test_judge_pick_does_not_gate_on_force_it_cannot_read():
    # A contact row without force_n must not be failed for a number that was
    # never reported -- record the gap instead of inventing a verdict.
    contacts = {"contacts": [{"robot_part": "right_hand_index_1",
                              "touching": "black_thermos_0"}]}
    v = probe.judge_pick(contacts, 0.0, 0.07, lift_source="world")
    assert v["force_unknown"] is True
    assert v["ok"] is True


def test_judge_pick_records_opposition_without_gating_on_it():
    # Two distinct hand parts touching is what a grasp looks like, but nobody
    # has measured how the Inspire hand's contacts aggregate in this scene, so
    # gating on it would be a bar invented rather than earned.
    one = {"contacts": [{"robot_part": "right_hand_palm",
                         "touching": "black_thermos_0", "force_n": 4.0}]}
    two = {"contacts": [{"robot_part": "right_hand_palm",
                         "touching": "black_thermos_0", "force_n": 4.0},
                        {"robot_part": "right_hand_thumb_2",
                         "touching": "black_thermos_0", "force_n": 3.0}]}
    assert probe.judge_pick(one, 0.0, 0.07, lift_source="world")["opposition"] is False
    assert probe.judge_pick(two, 0.0, 0.07, lift_source="world")["opposition"] is True
    assert probe.judge_pick(one, 0.0, 0.07, lift_source="world")["ok"] is True


def test_judge_pick_regression_the_measured_pick_failure_still_fails():
    # The finding that stands: 0 hand contacts, mm-scale lift, in both passes.
    # Hardening must not soften it and must not accidentally turn it into an
    # INVALID row -- it is a real, measured failure.
    contacts = {"contacts": [{"robot_part": "left_ankle_roll_link",
                              "touching": "floor", "force_n": 300.0}]}
    v = probe.judge_pick(contacts, 0.0, 0.001, lift_source="world")
    assert v["ok"] is False
    assert v["valid"] is True
    assert v["status"] == "fail_no_hand_contact"
    assert v["hand_contacts"] == 0


def test_judge_pick_wrist_contact_alone_is_still_not_a_grasp():
    # PRESERVED: a wrist-scoop with 5cm+ lift must never read as a grasp.
    wrist_only = {"contacts": [{"robot_part": "right_wrist_pitch_joint",
                                "touching": "black_thermos_0", "force_n": 5.0}]}
    v = probe.judge_pick(wrist_only, 0.0, 0.11, lift_source="world")
    assert v["ok"] is False
    assert v["hand_contacts"] == 0
    assert v["wrist_contacts"] == 1
    assert v["status"] == "fail_no_hand_contact"


def test_judge_pick_needs_hand_contact_and_lift():
    # PRESERVED from the pre-hardening suite, re-expressed against the world
    # frame source.
    contacts = {"contacts": [{"robot_part": "right_hand_index_1",
                              "touching": "black_thermos_0", "force_n": 2.1}]}
    assert probe.judge_pick(contacts, 0.49, 0.56, lift_source="world")["ok"] is True
    assert probe.judge_pick(contacts, 0.49, 0.51, lift_source="world")["ok"] is False
    none = {"contacts": [{"robot_part": "left_ankle", "touching": "floor",
                          "force_n": 300.0}]}
    assert probe.judge_pick(none, 0.49, 0.60, lift_source="world")["ok"] is False
    non_hand_on_thermos = {"contacts": [{"robot_part": "left_ankle",
                                         "touching": "black_thermos_0",
                                         "force_n": 300.0}]}
    assert probe.judge_pick(non_hand_on_thermos, 0.49, 0.60,
                            lift_source="world")["ok"] is False


def test_judge_pick_reports_a_missing_height_as_unmeasurable():
    contacts = {"contacts": [{"robot_part": "right_hand_index_1",
                              "touching": "black_thermos_0", "force_n": 4.0}]}
    v = probe.judge_pick(contacts, None, 0.07, lift_source="world")
    assert v["ok"] is False
    assert v["valid"] is False
    assert v["status"] == "invalid_no_height"


# ---- summary partitioning -------------------------------------------------

def test_summarise_separates_invalid_rows_from_measured_failures():
    # An INVALID row is not a robot failure and must never be counted as one.
    # pass A's summary.json listed goto_table under "ok"; the honest reading is
    # that the task never ran as an experiment.
    rows = [
        {"task": "see", "judge": {"ok": True, "valid": True}},
        {"task": "goto_table", "judge": {"ok": False, "valid": False,
                                         "status": "invalid_no_op"}},
        {"task": "pick_thermos", "judge": {"ok": False, "valid": True,
                                           "status": "fail_no_hand_contact"}},
    ]
    s = probe.summarise(rows, pass_name="A", max_hops=12)
    assert s["ok"] == ["see"]
    assert s["failed"] == ["pick_thermos"]
    assert s["invalid"] == ["goto_table"]
    assert s["judge_version"] == probe.JUDGE_VERSION


def test_summarise_treats_a_row_with_no_valid_key_as_invalid():
    # A judge that forgot to say whether it was a valid measurement is not
    # evidence. Fail closed.
    rows = [{"task": "x", "judge": {"ok": False, "why": "harness exception"}}]
    s = probe.summarise(rows, pass_name="A", max_hops=12)
    assert s["invalid"] == ["x"]
    assert s["failed"] == []


def test_judge_version_is_stamped_so_old_artifacts_cannot_be_confused():
    # The committed pass A / pass B artifacts were scored by the v1 judges.
    # Nothing in them says so, which is how a no-op ended up in an A/B table.
    assert probe.JUDGE_VERSION >= 2


# ===========================================================================
# CALL-SITE HARDENING (2026-08-18, second pass)
#
# Three holes the judge-hardening pass left open. All three live where the
# suite cannot reach: main() needs a live sidecar, so the ONLY protection a
# call-site fix has is a test that pins it deliberately.
#
#   1. _object_world_z() decides whether a grasp can ever be certified
#      (lift_source "world" vs "robot_relative") and had zero coverage.
#   2. sim.contacts(limit=64) is the fix for a false NEGATIVE -- the sidecar
#      sorts strongest-first, so two feet can push a fingertip off a 12-row
#      list -- and nothing stopped a later edit from dropping back to the
#      client default.
#   3. tools_called() reports every trace entry INCLUDING the ones the adapter
#      rejected. judge_see's entire "this was observed, not recalled" gate is
#      the presence of a perception call, so a describe_scene that ERRORED
#      satisfied it and the D-S1 degenerate reopened at the cost of one
#      failed call.
# ===========================================================================

class _FakeSim:
    """Only the body_pose op _object_world_z uses."""
    def __init__(self, payload=None, *, raises=None):
        self._payload = payload
        self._raises = raises
        self.asked = []

    def body_pose(self, bodies):
        self.asked.append(list(bodies))
        if self._raises is not None:
            raise self._raises
        return self._payload


def test_object_world_z_reads_mujocos_own_world_frame():
    # world_m is d.body(name).xpos: one read, no robot pose in the arithmetic.
    # The ABSOLUTE value is wrong (every lounge body keeps the source scan
    # origin as its body frame), which is exactly why only the DELTA is used.
    sim = _FakeSim({"bodies": {"black_thermos_0": {
        "world_m": [0.0057, -0.6743, 0.4312], "pelvis_m": [0.2, -0.1, -0.35]}}})
    z, src = probe._object_world_z(sim, "black_thermos_0")
    assert z == pytest.approx(0.4312)
    assert src == "world"
    assert sim.asked == [["black_thermos_0"]]


def test_object_world_z_falls_back_when_the_sidecar_cannot_answer():
    # An older sidecar has no body_pose op. The probe must still produce a row
    # -- and must SAY the height came from the pelvis-referenced path, because
    # judge_pick refuses to certify a grasp from that source.
    sim = _FakeSim(raises=RuntimeError("unknown op 'body_pose'"))
    assert probe._object_world_z(sim, "black_thermos_0") == (None, "robot_relative")


def test_object_world_z_falls_back_when_the_body_is_not_in_the_model():
    # _op_body_pose returns None for a name MuJoCo does not know, rather than
    # raising. A None that reached judge_pick as a number would be a lift
    # measured against nothing.
    assert probe._object_world_z(
        _FakeSim({"bodies": {"black_thermos_0": None}}),
        "black_thermos_0") == (None, "robot_relative")
    assert probe._object_world_z(
        _FakeSim({"bodies": {}}), "black_thermos_0") == (None, "robot_relative")
    assert probe._object_world_z(
        _FakeSim({}), "black_thermos_0") == (None, "robot_relative")


def test_the_probe_asks_for_more_contacts_than_the_sidecar_client_default():
    # sim.contacts() returns the STRONGEST contacts first and defaults to 12.
    # Standing on two feet next to furniture can fill that list before a
    # fingertip contact is reached, so the default can hide a REAL grasp --
    # a false negative, the one error direction this file cannot self-correct.
    client = pytest.importorskip("physicalagent.robots.sonic_sim_client")
    import inspect as _inspect
    default = _inspect.signature(
        client.SimClient.contacts).parameters["limit"].default
    assert default == 12, "client default changed — re-check CONTACT_LIMIT"
    assert probe.CONTACT_LIMIT > default
    # 36 of the G1's 66 bodies are hand parts (measured off
    # g1_29dof_with_inspire_actuated.xml), so a full two-handed grasp plus
    # feet and furniture has to fit.
    assert probe.CONTACT_LIMIT >= 64


def test_tools_called_can_exclude_the_calls_the_adapter_rejected():
    r = _FakeTrace([("describe_scene", False), ("look", True), ("stop", True)])
    # the mechanical record is every call, failures included
    assert probe.tools_called(r) == ["describe_scene", "look", "stop"]
    # ...and the accepted-only view is what an "it was observed" gate needs
    assert probe.tools_called(r, ok_only=True) == ["look", "stop"]


def test_tools_called_ok_only_is_conservative_about_an_unreadable_entry():
    # A trace entry with no ok field is not evidence the call succeeded.
    r = _FakeTrace(["look"])
    r.trace = [type("E", (), {"name": "look", "args": {}})()]
    assert probe.tools_called(r) == ["look"]
    assert probe.tools_called(r, ok_only=True) == []


def test_judge_see_is_not_satisfied_by_a_perception_call_that_failed():
    # DEGENERATE (D-S1, reopened): the whole "this was observed" gate is the
    # presence of a perception call. A describe_scene the adapter REJECTED is
    # still in the trace, so hallucinating the room after one failed call
    # passed. The call site must hand judge_see the accepted calls only.
    r = _FakeTrace([("describe_scene", False)],
                   final_text="There is a round table and a bench here.")
    v = probe.judge_see(probe.reply_text(r), probe.SCENE_NAMES,
                        called=probe.tools_called(r, ok_only=True))
    assert v["ok"] is False
    assert v["valid"] is False
    assert v["status"] == "invalid_no_perception_call"


# ---- the two fixes that live in main(), which the suite cannot run --------

def test_main_hands_judge_see_only_the_tool_calls_that_succeeded():
    # main() needs a live sidecar, a live model and a running simulator, so
    # this invariant has no runtime test anywhere. Pin the source instead:
    # without it, a failed describe_scene satisfies judge_see's only gate.
    import inspect as _inspect
    src = _inspect.getsource(probe.main)
    assert "tools_called(result, ok_only=True)" in src, (
        "judge_see must be dispatched with the ACCEPTED perception calls")
    assert "judge_see(text, SCENE_NAMES, called=accepted)" in src


def test_main_requests_the_wide_contact_list():
    import inspect as _inspect
    src = _inspect.getsource(probe.main)
    assert "sim.contacts(limit=CONTACT_LIMIT)" in src, (
        "the default limit of 12 can hide a real fingertip contact")
