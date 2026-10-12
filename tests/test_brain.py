"""Tests for physicalagent/brain/er.py — the Gemini agent loop.

The loop (sense -> think -> act -> observe, repeat) is the brain's value, so it
is tested here in full, offline. The model is abstracted behind a Conversation
interface; a FakeConversation replays scripted model turns. The real
GeminiConversation (google-genai wiring) is verified on hardware, not here.
"""
import pytest

from physicalagent.brain.er import (
    Brain, Conversation, ModelResponse, ToolCall,
)
from physicalagent.robots.base import RobotAdapter, ToolSpec


# ---- a tiny adapter that records executed tools ----------------------------
class RecordingAdapter(RobotAdapter):
    def __init__(self, fail=(), frame=b"\xff\xd8JPEG"):
        super().__init__(guard=None)
        self.executed = []
        self._fail = set(fail)
        self.last_frame = None
        self._frame = frame

    def capabilities(self):
        return (ToolSpec("get_status", "d", {"type": "object", "properties": {}}),
                ToolSpec("look", "d", {"type": "object", "properties": {}}),
                ToolSpec("walk", "d", {"type": "object",
                         "properties": {"distance_cm": {"type": "number"}},
                         "required": ["distance_cm"]}))

    def _handlers(self):
        return {"get_status": lambda: {"battery_mv": 11800},
                "look": self._look,
                "walk": lambda distance_cm: {"measured_cm": distance_cm}}

    def _look(self):
        self.last_frame = self._frame
        return {"frame_captured": True}

    def execute(self, name, args):
        self.executed.append((name, args))
        if name in self._fail:
            from physicalagent.robots.base import ToolResult
            return ToolResult.failure(f"{name} refused")
        return super().execute(name, args)

    def get_frame(self): return self._frame
    def telemetry(self): return {}
    def stop_all(self): pass


# ---- a scripted conversation ----------------------------------------------
class FakeConversation(Conversation):
    def __init__(self, scripted):
        self._scripted = list(scripted)
        self.user_texts = []
        self.tool_result_batches = []

    def user_text(self, text):
        self.user_texts.append(text)

    def tool_results(self, results, images=()):
        self.tool_result_batches.append((results, list(images)))

    def respond(self):
        return self._scripted.pop(0)


def brain_with(scripted, adapter=None):
    adapter = adapter or RecordingAdapter()
    convo = FakeConversation(scripted)
    b = Brain(adapter, client=_FakeClient(convo), system="be a robot", max_hops=8)
    return b, adapter, convo


class _FakeClient:
    def __init__(self, convo): self._convo = convo
    def new_conversation(self, system, tool_decls): return self._convo


# ---------------------------------------------------------------------------
class TestBrainBasicLoop:
    def test_immediate_final_answer_no_tools(self):
        b, adapter, convo = brain_with([ModelResponse(text="all done", calls=[])])
        res = b.run("do nothing")
        assert res.complete is True
        assert res.final_text == "all done"
        assert adapter.executed == []
        assert convo.user_texts == ["do nothing"]

    def test_single_tool_then_answer(self):
        b, adapter, convo = brain_with([
            ModelResponse(text=None, calls=[ToolCall("get_status", {})]),
            ModelResponse(text="battery is fine", calls=[]),
        ])
        res = b.run("check status")
        assert res.complete is True
        assert res.final_text == "battery is fine"
        assert adapter.executed == [("get_status", {})]
        # the tool result was fed back to the model
        assert len(convo.tool_result_batches) == 1
        results, _ = convo.tool_result_batches[0]
        assert results[0][0] == "get_status"
        assert results[0][1]["ok"] is True

    def test_parallel_tool_calls_in_one_hop(self):
        b, adapter, convo = brain_with([
            ModelResponse(text=None, calls=[ToolCall("get_status", {}),
                                            ToolCall("walk", {"distance_cm": 10})]),
            ModelResponse(text="did both", calls=[]),
        ])
        res = b.run("go")
        assert [n for n, _ in adapter.executed] == ["get_status", "walk"]
        results, _ = convo.tool_result_batches[0]
        assert len(results) == 2


class TestBrainLookAttachesImage:
    def test_look_result_attaches_frame_as_image(self):
        b, adapter, convo = brain_with([
            ModelResponse(text=None, calls=[ToolCall("look", {})]),
            ModelResponse(text="I see a wall", calls=[]),
        ])
        b.run("look around")
        results, images = convo.tool_result_batches[0]
        assert images == [b"\xff\xd8JPEG"]     # the captured frame was attached

    def test_non_look_tool_attaches_no_image(self):
        b, adapter, convo = brain_with([
            ModelResponse(text=None, calls=[ToolCall("get_status", {})]),
            ModelResponse(text="ok", calls=[]),
        ])
        b.run("status")
        _, images = convo.tool_result_batches[0]
        assert images == []


class TestBrainHandlesFailures:
    def test_failed_tool_is_reported_and_loop_continues(self):
        adapter = RecordingAdapter(fail={"walk"})
        b, _, convo = brain_with([
            ModelResponse(text=None, calls=[ToolCall("walk", {"distance_cm": 10})]),
            ModelResponse(text="could not walk, explaining", calls=[]),
        ], adapter=adapter)
        res = b.run("walk")
        assert res.complete is True
        results, _ = convo.tool_result_batches[0]
        assert results[0][1]["ok"] is False        # model sees the refusal
        assert "refused" in results[0][1]["error"]

    def test_trace_records_every_tool_call(self):
        b, adapter, convo = brain_with([
            ModelResponse(text=None, calls=[ToolCall("get_status", {})]),
            ModelResponse(text=None, calls=[ToolCall("walk", {"distance_cm": 5})]),
            ModelResponse(text="done", calls=[]),
        ])
        res = b.run("go")
        assert [e.name for e in res.trace] == ["get_status", "walk"]
        assert all(e.ok for e in res.trace)


class TestBrainSafetyRails:
    def test_stops_at_max_hops_without_final_answer(self):
        # model keeps calling tools forever
        scripted = [ModelResponse(text=None, calls=[ToolCall("get_status", {})]) for _ in range(20)]
        b, adapter, convo = brain_with(scripted)
        b._max_hops = 5
        res = b.run("loop forever")
        assert res.complete is False
        assert res.hops == 5
        assert len(adapter.executed) == 5  # did not run away past the cap

    def test_stop_all_called_on_completion(self):
        calls = {"n": 0}
        class A(RecordingAdapter):
            def stop_all(s): calls["n"] += 1
        b, adapter, convo = brain_with([ModelResponse(text="done", calls=[])], adapter=A())
        b.run("x")
        assert calls["n"] >= 1  # brain leaves the robot halted when it finishes


class TestTraceCarriesToolResults:
    """The trace must record what each tool RETURNED, not just that it ran.

    Without this the operator sees `turn({'angle_deg': 90})` and has no idea
    whether the closed-loop controller achieved 90 degrees or 45 — exactly the
    number that matters when evaluating motion on carpet.
    """

    def test_trace_entry_records_result_payload(self):
        brain, adapter, _ = brain_with([
            ModelResponse(text=None, calls=[ToolCall("walk", {"distance_cm": 20})]),
            ModelResponse(text="done", calls=[]),
        ])
        res = brain.run("walk a bit")
        entry = next(e for e in res.trace if e.name == "walk")
        assert entry.result == {"measured_cm": 20}


class TestBrainLiveStepCallback:
    """An interactive operator needs to see tools fire as they happen, not only
    in the summary after the whole task finishes."""

    def test_on_step_called_per_tool_with_name_args_and_result(self):
        seen = []
        brain, _, _ = brain_with([
            ModelResponse(text=None, calls=[ToolCall("walk", {"distance_cm": 20})]),
            ModelResponse(text="done", calls=[]),
        ])
        brain.run("go", on_step=lambda n, a, ok, err, res, took=None:
                  seen.append((n, a, ok, err, res, took)))
        (name, args, ok, err, res, took) = seen[0]
        assert (name, args, ok, err, res) == ("walk", {"distance_cm": 20}, True, None,
                                              {"measured_cm": 20})
        assert took is not None and took >= 0      # timing is reported live too

    def test_run_works_without_a_callback(self):
        brain, _, _ = brain_with([
            ModelResponse(text=None, calls=[ToolCall("get_status", {})]),
            ModelResponse(text="ok", calls=[]),
        ])
        assert brain.run("status").complete is True


class _CountingClient:
    """Records how many conversations were opened."""
    def __init__(self, convo):
        self._convo = convo
        self.opened = 0

    def new_conversation(self, system, tool_decls):
        self.opened += 1
        return self._convo


class TestBrainMemory:
    """The model said, verbatim: 'I don't have the details of our previous task
    in my current memory.' Each task opened a FRESH conversation, so the robot
    forgot everything the instant a task ended — including a task interrupted by
    a network error, which is exactly when continuity matters most.
    """

    def test_second_task_continues_the_same_conversation(self):
        convo = FakeConversation([
            ModelResponse(text="one", calls=[]),
            ModelResponse(text="two", calls=[]),
        ])
        client = _CountingClient(convo)
        b = Brain(RecordingAdapter(), client=client, system="s")
        b.run("first task")
        b.run("second task")
        assert client.opened == 1                       # one continuous dialogue
        assert convo.user_texts == ["first task", "second task"]

    def test_reset_starts_a_fresh_conversation(self):
        convo = FakeConversation([
            ModelResponse(text="one", calls=[]),
            ModelResponse(text="two", calls=[]),
        ])
        client = _CountingClient(convo)
        b = Brain(RecordingAdapter(), client=client, system="s")
        b.run("first")
        b.reset()
        b.run("second")
        assert client.opened == 2


class TestBrainApiResilience:
    """SSL EOF, 'Server disconnected', and 429 RESOURCE_EXHAUSTED each destroyed
    an in-flight task in the real session. They are transient; the correct
    response is to wait and retry, not to throw away the operator's task.
    """

    class _FlakyConversation(FakeConversation):
        def __init__(self, scripted, fail_times, exc):
            super().__init__(scripted)
            self._fail = fail_times
            self._exc = exc

        def respond(self):
            if self._fail > 0:
                self._fail -= 1
                raise self._exc
            return super().respond()

    def _brain(self, convo, slept):
        return Brain(RecordingAdapter(), client=_FakeClient(convo), system="s",
                     sleep=slept.append)

    def test_retries_a_429_and_completes(self):
        convo = self._FlakyConversation(
            [ModelResponse(text="done", calls=[])], 2,
            RuntimeError("429 RESOURCE_EXHAUSTED quota exceeded"))
        slept = []
        res = self._brain(convo, slept).run("go")
        assert res.complete is True and res.final_text == "done"
        assert len(slept) == 2                      # backed off between attempts

    def test_backoff_grows(self):
        convo = self._FlakyConversation(
            [ModelResponse(text="done", calls=[])], 2,
            RuntimeError("429 RESOURCE_EXHAUSTED"))
        slept = []
        self._brain(convo, slept).run("go")
        assert slept[1] > slept[0]

    def test_retries_transient_ssl_and_disconnect(self):
        for msg in ("[SSL: UNEXPECTED_EOF_WHILE_READING] unexpected eof",
                    "Server disconnected without sending a response."):
            convo = self._FlakyConversation(
                [ModelResponse(text="ok", calls=[])], 1, RuntimeError(msg))
            slept = []
            assert self._brain(convo, slept).run("go").complete is True

    def test_gives_up_after_max_retries_and_reports(self):
        convo = self._FlakyConversation(
            [ModelResponse(text="never", calls=[])], 99,
            RuntimeError("429 RESOURCE_EXHAUSTED"))
        slept = []
        b = Brain(RecordingAdapter(), client=_FakeClient(convo), system="s",
                  sleep=slept.append, max_retries=3)
        with pytest.raises(Exception):
            b.run("go")

    def test_non_transient_error_is_not_retried(self):
        convo = self._FlakyConversation(
            [ModelResponse(text="x", calls=[])], 99, ValueError("bad api key"))
        slept = []
        with pytest.raises(ValueError):
            self._brain(convo, slept).run("go")
        assert slept == []                          # failed fast, no pointless waiting


class TestBrainContinuousOperation:
    def test_default_hop_cap_allows_long_tasks(self):
        """12 hops cut off a real navigation task mid-goal."""
        b = Brain(RecordingAdapter(), client=_FakeClient(FakeConversation([])), system="s")
        assert b._max_hops >= 50


class TestBrainTimesEachStep:
    """The operator reported 'long waits between actions, not sure why'.

    Several candidates exist (a 10 s walk, model inference, a camera fetch
    timing out at 8 s, retry backoff). Rather than guess, time every tool call
    and every model turn so the next run shows where the seconds actually go.
    """

    def test_trace_entries_carry_elapsed_time(self):
        brain, _, _ = brain_with([
            ModelResponse(text=None, calls=[ToolCall("walk", {"distance_cm": 5})]),
            ModelResponse(text="done", calls=[]),
        ])
        res = brain.run("go")
        assert res.trace[0].elapsed_s is not None
        assert res.trace[0].elapsed_s >= 0

    def test_result_reports_time_spent_thinking(self):
        brain, _, _ = brain_with([
            ModelResponse(text=None, calls=[ToolCall("get_status", {})]),
            ModelResponse(text="done", calls=[]),
        ])
        res = brain.run("go")
        assert res.think_s is not None and res.think_s >= 0


class TestDepletedCreditsAreNotRetried:
    """Verified live 2026-08-19 against the project's own key:

        models.list()      OK, 50 models, 0.40 s   (metadata is free)
        generate_content   429 RESOURCE_EXHAUSTED in 0.39 s
                           'Your prepayment credits are depleted.'

    _TRANSIENT matches "429" and "resource_exhausted", so this climbs the whole
    ladder -- 2+4+8+16+32 = 62 s of blocking -- and then fails anyway, because
    no amount of waiting restores a depleted balance. It is not a rate limit.

    62 silent seconds is a safety problem at a gantry: the operator has spoken
    or typed a command, the robot is live, and nothing on screen says whether
    it is coming. Fail in 0.4 s and say why."""

    def test_depleted_credits_is_not_transient(self):
        from physicalagent.brain import er
        exc = RuntimeError(
            "429 RESOURCE_EXHAUSTED. {'error': {'code': 429, 'message': "
            "'Your prepayment credits are depleted. Please go to AI Studio'}}")
        assert er._is_transient(exc) is False

    def test_an_ordinary_rate_limit_is_still_transient(self):
        """The distinction has to survive: real 429s DO clear on their own."""
        from physicalagent.brain import er
        exc = RuntimeError("429 RESOURCE_EXHAUSTED: rate limit exceeded, "
                           "retry in 12s")
        assert er._is_transient(exc) is True

    def test_the_other_transient_markers_are_untouched(self):
        from physicalagent.brain import er
        for msg in ("503 unavailable", "connection reset by peer",
                    "deadline exceeded", "SSL: UNEXPECTED_EOF"):
            assert er._is_transient(RuntimeError(msg)) is True, msg


class TestTheRoboticsModelIsER2AndIsSelectable:
    """The point of this project is Gemini Robotics ER in the loop with SONIC
    holding balance underneath. The client defaulted to ER-1.6.

    Verified against the live key 2026-08-19 (models.list() is free even with
    the billing 429):

        models/gemini-robotics-er-1.6-preview
        models/gemini-robotics-er-2-preview            <- now available
        models/gemini-robotics-er-2-streaming-preview

    NOT verified: that ER-2 answers with the same tool-calling contract as
    ER-1.6. Credits are depleted, so no generate_content call has ever been
    made against it from this machine. The first funded run is the test."""

    def test_the_default_is_er2(self):
        import inspect
        from physicalagent.brain.er import GeminiClient
        default = inspect.signature(GeminiClient).parameters["model"].default
        assert default == "gemini-robotics-er-2-preview", default

    def test_the_model_can_still_be_overridden_per_call(self):
        from physicalagent.brain.er import GeminiClient
        c = GeminiClient.__new__(GeminiClient)
        assert "model" in __import__("inspect").signature(
            GeminiClient.__init__).parameters

    def test_an_env_var_pins_the_model_without_editing_code(self, monkeypatch):
        """So a bad ER-2 run can be rolled back to 1.6 at the gantry, without
        an edit, by someone whose hands are busy."""
        from physicalagent.brain import er
        monkeypatch.setenv("PHYSICALAGENT_ER_MODEL", "gemini-robotics-er-1.6-preview")
        assert er.default_model() == "gemini-robotics-er-1.6-preview"

    def test_without_the_env_var_it_is_er2(self, monkeypatch):
        from physicalagent.brain import er
        monkeypatch.delenv("PHYSICALAGENT_ER_MODEL", raising=False)
        assert er.default_model() == "gemini-robotics-er-2-preview"
