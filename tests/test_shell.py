"""Tests for physicalagent/shell.py — the interactive operator console.

The console is what a human actually drives, so its safety behaviour matters as
much as the adapter's: an instant stop that never waits on model inference, a
guaranteed halt when the session ends, and a loop that survives a bad task
without dropping the operator out of control.

I/O is injected (read_line / write), so the whole REPL runs offline.
"""
import pytest

from physicalagent.shell import run_shell


class FakeBrain:
    """Records tasks; optionally raises for a given task to test resilience."""

    def __init__(self, raise_on=(), text="did it"):
        self.tasks = []
        self._raise_on = set(raise_on)
        self._text = text

    def run(self, task, on_step=None):
        self.tasks.append(task)
        if task in self._raise_on:
            raise RuntimeError("model exploded")
        from physicalagent.brain.er import BrainResult, TraceEntry
        return BrainResult(final_text=self._text, complete=True, hops=1,
                           trace=[TraceEntry("look", {}, True, None, {"bytes": 5})])


class FakeAdapter:
    def __init__(self):
        self.stops = 0

    def stop_all(self):
        self.stops += 1


def drive(lines, brain=None, adapter=None):
    """Run the shell over a scripted list of input lines."""
    brain = brain or FakeBrain()
    adapter = adapter or FakeAdapter()
    it = iter(lines)
    out = []

    def read_line(_prompt):
        try:
            return next(it)
        except StopIteration:
            raise EOFError

    run_shell(brain, adapter, read_line=read_line, write=out.append)
    return brain, adapter, out


class TestShellRunsTasks:
    def test_a_typed_line_becomes_a_task(self):
        brain, _, _ = drive(["walk to the door"])
        assert brain.tasks == ["walk to the door"]

    def test_prints_the_models_final_answer(self):
        _, _, out = drive(["look around"])
        assert any("did it" in line for line in out)

    def test_runs_several_tasks_in_one_session(self):
        brain, _, _ = drive(["first", "second", "third"])
        assert brain.tasks == ["first", "second", "third"]

    def test_blank_lines_are_ignored(self):
        brain, _, _ = drive(["", "   ", "real task"])
        assert brain.tasks == ["real task"]


class TestShellInstantStop:
    """`stop` must halt the robot directly — never via the model.

    Routing an emergency stop through Gemini would add seconds of inference
    latency to the one command that must never wait, and could be refused or
    reinterpreted by the model. It bypasses the brain entirely.
    """

    def test_stop_halts_without_calling_the_model(self):
        brain, adapter, _ = drive(["stop"])
        assert brain.tasks == []          # model never consulted
        assert adapter.stops >= 1         # robot halted anyway

    def test_stop_is_case_insensitive_and_ignores_whitespace(self):
        brain, adapter, _ = drive(["  STOP  "])
        assert brain.tasks == []
        assert adapter.stops >= 1

    def test_shell_continues_after_a_stop(self):
        brain, _, _ = drive(["stop", "now look around"])
        assert brain.tasks == ["now look around"]


class TestShellExitAlwaysHalts:
    def test_quit_exits_and_halts(self):
        brain, adapter, _ = drive(["quit", "never reached"])
        assert brain.tasks == []
        assert adapter.stops >= 1

    def test_eof_exits_and_halts(self):
        _, adapter, _ = drive([])          # immediate EOF
        assert adapter.stops >= 1

    def test_halts_even_if_a_task_raises(self):
        brain = FakeBrain(raise_on={"boom"})
        _, adapter, _ = drive(["boom"], brain=brain)
        assert adapter.stops >= 1


class TestShellSurvivesFailure:
    def test_a_failing_task_does_not_end_the_session(self):
        brain = FakeBrain(raise_on={"boom"})
        brain, _, out = drive(["boom", "recover"], brain=brain)
        assert brain.tasks == ["boom", "recover"]
        assert any("exploded" in line or "error" in line.lower() for line in out)

    def test_keyboard_interrupt_during_a_task_halts_but_keeps_session(self):
        class Interrupting(FakeBrain):
            def run(self, task, on_step=None):
                if task == "long":
                    self.tasks.append(task)
                    raise KeyboardInterrupt
                return super().run(task, on_step)   # parent records the task

        brain = Interrupting()
        brain, adapter, _ = drive(["long", "after"], brain=brain)
        assert brain.tasks == ["long", "after"]   # Ctrl-C aborted the task, not the shell
        assert adapter.stops >= 1


class TestShellLiveTrace:
    def test_tool_calls_are_shown_as_they_happen(self):
        """The operator must see each tool fire, not just a summary at the end."""
        class Stepping(FakeBrain):
            def run(self, task, on_step=None):
                self.tasks.append(task)
                if on_step:
                    on_step("turn", {"angle_deg": 90},
                            True, None, {"achieved_deg": 89.9})
                return super().run(task, on_step)

        _, _, out = drive(["turn"], brain=Stepping())
        joined = "\n".join(out)
        assert "turn" in joined
        assert "89.9" in joined


class TestShellPromptInterrupt:
    """Ctrl-C at the PROMPT must cancel the half-typed line, not end the session.

    From the real session log: the operator was editing a long task, hit Ctrl-C
    to clear it, and the whole console exited. Losing the session over a typo is
    hostile — and Ctrl-D / `quit` already exist for deliberate exit.
    """

    def test_ctrl_c_at_prompt_cancels_line_and_keeps_session(self):
        calls = {"n": 0}

        def read_line(_prompt):
            calls["n"] += 1
            if calls["n"] == 1:
                raise KeyboardInterrupt      # operator abandons a typed line
            if calls["n"] == 2:
                return "look around"
            raise EOFError

        brain, adapter, out = FakeBrain(), FakeAdapter(), []
        run_shell(brain, adapter, read_line=read_line, write=out.append)
        assert brain.tasks == ["look around"]   # session survived the Ctrl-C

    def test_eof_still_exits(self):
        _, adapter, _ = drive([])
        assert adapter.stops >= 1


class TestShellMemoryControl:
    """Memory persists by default, but the operator needs a way to clear it —
    a stale goal carried into an unrelated task is its own hazard."""

    def test_forget_resets_the_brain_without_calling_the_model(self):
        class Resettable(FakeBrain):
            def __init__(self):
                super().__init__()
                self.resets = 0

            def reset(self):
                self.resets += 1

        brain = Resettable()
        brain, _, out = drive(["forget", "new task"], brain=brain)
        assert brain.resets == 1
        assert brain.tasks == ["new task"]          # forget is not itself a task

    def test_forget_is_safe_when_the_brain_has_no_reset(self):
        brain, _, _ = drive(["forget", "carry on"])   # FakeBrain has no reset()
        assert brain.tasks == ["carry on"]
