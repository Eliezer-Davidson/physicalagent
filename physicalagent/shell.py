"""Interactive operator console — type a task, watch the robot do it.

This is the surface a human actually drives, so it carries its own safety
obligations beyond the adapter's:

  * `stop` halts the robot IMMEDIATELY and never consults the model. Routing an
    emergency stop through Gemini would put seconds of inference latency in
    front of the one command that must not wait — and the model could refuse or
    reinterpret it. It calls adapter.stop_all() directly.
  * the robot is halted when the session ends, by ANY path: quit, EOF (Ctrl-D),
    an exception inside a task, or Ctrl-C.
  * a failed task never drops the operator out of the console — losing the
    prompt would mean losing the ability to type `stop`.

I/O is injected (read_line / write) so the whole loop is tested offline.
"""
from __future__ import annotations

import os


def enable_line_editing(history_path: str | None = None) -> None:
    """Turn on readline so arrow keys edit the line instead of emitting ^[[D.

    Robot tasks are long sentences that get retyped with small variations, so
    cursor movement and history recall are the difference between a usable
    console and a frustrating one. Best-effort: readline is absent on some
    platforms and its lack must never stop the operator getting a prompt.
    """
    try:
        import readline
    except ImportError:
        return
    path = history_path or os.path.expanduser("~/.physicalagent_history")
    try:
        readline.read_history_file(path)
    except (OSError, PermissionError):
        pass
    try:
        readline.set_history_length(1000)
        import atexit
        atexit.register(_save_history, readline, path)
    except Exception:
        pass


def _save_history(readline, path) -> None:
    try:
        readline.write_history_file(path)
    except (OSError, PermissionError):
        pass


STOP_WORDS = {"stop", "halt", "e-stop", "estop"}
QUIT_WORDS = {"quit", "exit", "q"}
FORGET_WORDS = {"forget", "reset", "new"}

BANNER = """\
PhysicalAgent console — Gemini has control of the ROSPug.
  <task>   plain English, e.g. 'turn left and tell me what you see'
  stop     halt the robot IMMEDIATELY (does not consult the model)
  forget   clear the robot's memory of this session's earlier tasks
  quit     end the session (always halts first)
"""


def _fmt_step(name, args, ok, error, result, elapsed_s=None) -> str:
    mark = "ok " if ok else "ERR"
    tail = f" -> {result}" if ok and result else ("" if ok else f" -> {error or ''}")
    took = f" [{elapsed_s}s]" if elapsed_s is not None else ""
    return f"    [{mark}] {name}({args}){took}{tail}"


def run_shell(brain, adapter, read_line=input, write=print, prompt="\nrobot> ",
              banner=None) -> int:
    """Run the interactive console until quit/EOF. Always leaves the robot halted.

    `banner` overrides the default header; the console is shared between robots
    (the hexapod and the simulated G1) and telling the operator which machine
    they are about to command is not decoration.
    """
    write(banner if banner is not None else BANNER)
    try:
        while True:
            try:
                raw = read_line(prompt)
            except EOFError:
                write("\nending session.")
                return 0
            except KeyboardInterrupt:
                # Ctrl-C at the prompt abandons the half-typed line only. Ending
                # the whole session over a typo is hostile, and Ctrl-D / `quit`
                # already cover deliberate exit.
                write("  (cancelled — type 'quit' or press Ctrl-D to exit)")
                continue

            task = (raw or "").strip()
            if not task:
                continue

            low = task.lower()
            if low in QUIT_WORDS:
                write("ending session.")
                return 0
            if low in STOP_WORDS:
                # direct, unmediated halt — no model in the path
                try:
                    adapter.stop_all()
                    write("*** STOPPED (robot halted directly, model not consulted) ***")
                except Exception as exc:
                    write(f"*** STOP FAILED: {exc} — power the robot down manually ***")
                continue

            if low == "reset-fall":
                # Operator-only latch clear, deliberately NOT routed through
                # the model: it is the "ask the operator to reset" contract's
                # other half. Distinct from FORGET_WORDS' "reset", which
                # clears the model's memory, not the safety latch.
                reset_latch = getattr(adapter, "reset_fall_latch", None)
                if callable(reset_latch):
                    reset_latch()
                    write("fall latch cleared — the watchdog is re-armed.")
                else:
                    write("this robot has no fall latch.")
                continue

            if low in FORGET_WORDS:
                # the robot remembers earlier tasks by default; this is how the
                # operator draws a line before an unrelated goal
                reset = getattr(brain, "reset", None)
                if callable(reset):
                    reset()
                write("memory cleared — the next task starts fresh.")
                continue

            try:
                result = brain.run(task, on_step=lambda *a: write(_fmt_step(*a)))
            except KeyboardInterrupt:
                # Ctrl-C aborts the TASK, not the session: halt and give the
                # prompt back so the operator stays in control.
                try:
                    adapter.stop_all()
                except Exception:
                    pass
                write("\n*** task interrupted — robot halted ***")
                continue
            except Exception as exc:
                try:
                    adapter.stop_all()
                except Exception:
                    pass
                write(f"*** error: {exc} — robot halted, session continues ***")
                continue

            if getattr(result, "think_s", None) is not None:
                write(f"    (model thinking time this task: {result.think_s}s)")
            if result.final_text:
                write(f"\n{result.final_text}")
            if not result.complete:
                write(f"\n(stopped at hop cap after {result.hops} hops)")
    finally:
        try:
            adapter.stop_all()
        except Exception:
            pass
