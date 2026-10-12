"""ErBrain — the Gemini Robotics-ER agent loop.

sense -> think -> act -> observe, repeated until the model gives a final answer
or a hop cap is hit. The model is abstracted behind a Conversation so the loop
is fully testable; GeminiConversation wires the real google-genai client.

Safety posture:
  * the loop is bounded by max_hops — the model cannot drive the robot forever.
  * every tool result (success OR refusal) is fed back so the model can adapt;
    the adapter's execute() never raises, so one bad call never breaks the loop.
  * when the loop ends (answer, cap, or exception), the robot is left halted via
    adapter.stop_all(). Finishing a task must not leave it mid-motion.
  * a `look` result attaches the freshly captured frame as an image on the next
    turn, closing the perception loop.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any


@dataclass
class ToolCall:
    name: str
    args: dict


@dataclass
class ModelResponse:
    text: str | None
    calls: list[ToolCall] = field(default_factory=list)


@dataclass
class TraceEntry:
    name: str
    args: dict
    ok: bool
    error: str | None = None
    # what the tool actually returned (achieved angle, measured distance,
    # clearances). Without it the operator only sees what was REQUESTED, which
    # is the wrong half when evaluating motion accuracy on a slippery surface.
    result: dict = field(default_factory=dict)
    elapsed_s: float | None = None      # wall time this tool took


@dataclass
class BrainResult:
    final_text: str | None
    complete: bool
    hops: int
    trace: list[TraceEntry] = field(default_factory=list)
    think_s: float | None = None        # wall time spent waiting on the model


class Conversation(ABC):
    """One model dialogue. Owns the message history; the brain drives it."""

    @abstractmethod
    def user_text(self, text: str) -> None: ...

    @abstractmethod
    def tool_results(self, results: list[tuple[str, dict]], images: list[bytes] = ()) -> None: ...

    @abstractmethod
    def respond(self) -> ModelResponse: ...


# ---------------------------------------------------------------------------
# Real Gemini wiring (google-genai). Not unit-tested — it wraps SDK types and
# is verified against the live model. The loop logic above is what carries tests.
# ---------------------------------------------------------------------------
class GeminiConversation(Conversation):
    def __init__(self, client, model, system, tool_decls, temperature=0.0):
        from google.genai import types
        self._types = types
        self._client = client
        self._model = model
        self._contents: list = []
        self._tools = [types.Tool(function_declarations=[
            types.FunctionDeclaration(name=d["name"], description=d["description"],
                                      parameters_json_schema=d["parameters"])
            for d in tool_decls])]
        self._cfg = types.GenerateContentConfig(
            tools=self._tools, temperature=temperature,
            system_instruction=system if system else None)

    def user_text(self, text):
        t = self._types
        self._contents.append(t.Content(role="user", parts=[t.Part.from_text(text=text)]))

    def tool_results(self, results, images=()):
        t = self._types
        parts = [t.Part.from_function_response(name=name, response=resp)
                 for name, resp in results]
        for img in images:
            parts.append(t.Part.from_bytes(data=img, mime_type="image/jpeg"))
        self._contents.append(t.Content(role="user", parts=parts))

    def respond(self):
        resp = self._client.models.generate_content(
            model=self._model, contents=self._contents, config=self._cfg)
        # A blocked/empty response is a normal model outcome (safety filter, no
        # candidates); surface it as a final answer rather than crashing the loop
        # (which would stop the heartbeat and pause the robot).
        if not getattr(resp, "candidates", None):
            fb = getattr(resp, "prompt_feedback", None)
            return ModelResponse(text=f"model returned no candidates (feedback: {fb})", calls=[])
        cand = resp.candidates[0]
        if not getattr(cand, "content", None) or not getattr(cand.content, "parts", None):
            return ModelResponse(text=f"model returned empty content (finish: "
                                      f"{getattr(cand, 'finish_reason', None)})", calls=[])
        self._contents.append(cand.content)  # keep the model's turn in history
        calls, text = [], None
        for p in cand.content.parts:
            if getattr(p, "function_call", None):
                calls.append(ToolCall(p.function_call.name, dict(p.function_call.args or {})))
            elif getattr(p, "text", None):
                text = (text or "") + p.text
        return ModelResponse(text=text, calls=calls)


# Gemini Robotics-ER is the point of this project: it reasons about the scene
# and calls the robot's tools, while SONIC holds balance underneath it.
#
# ER-2 confirmed available against this project's key on 2026-08-19 (a
# models.list() call, which stays free while generate_content 429s on billing).
# NOT confirmed: that ER-2 returns the same tool-calling shape as ER-1.6 --
# no generate_content call has ever reached it from this machine. Set
# PHYSICALAGENT_ER_MODEL to fall back to 1.6 without editing code, which is
# what you want available when the robot is live and your hands are busy.
ER_MODEL = "gemini-robotics-er-2-preview"
ER_MODEL_FALLBACK = "gemini-robotics-er-1.6-preview"


def default_model() -> str:
    """The ER model to use, overridable by env for a gantry-side rollback."""
    import os as _os
    return _os.environ.get("PHYSICALAGENT_ER_MODEL", "").strip() or ER_MODEL


class GeminiClient:
    """Factory the Brain uses to open conversations against Gemini Robotics-ER."""

    def __init__(self, api_key: str, model: str = ER_MODEL,
                 temperature: float = 0.0):
        from google import genai
        self._client = genai.Client(api_key=api_key)
        self._model = model
        self._temperature = temperature

    def new_conversation(self, system, tool_decls) -> Conversation:
        return GeminiConversation(self._client, self._model, system, tool_decls,
                                  self._temperature)


# Substrings identifying failures that are worth waiting out rather than
# throwing away the operator's task. All observed live in one session.
_TRANSIENT = (
    "429", "resource_exhausted", "quota",
    "ssl", "unexpected_eof", "eof occurred",
    "server disconnected", "connection reset", "connection aborted",
    "timed out", "timeout", "503", "502", "500", "unavailable",
    "deadline exceeded",
)


# A 429 that will NEVER clear on its own. Verified live 2026-08-19: the API
# returns "prepayment credits are depleted" in 0.39 s, and _TRANSIENT's "429"
# then bought 62 s of blocking (2+4+8+16+32) before failing anyway. Waiting
# does not buy credits. At a gantry, 62 silent seconds with a live robot is a
# safety problem, so this must fail fast and say what is actually wrong.
_PERMANENT = (
    "credits are depleted", "prepayment credits", "billing",
    "api key not valid", "permission_denied",
)


def _is_transient(exc: Exception) -> bool:
    """True for errors that a later identical request would likely survive.

    Deliberately conservative: an unrecognised error is NOT retried, because
    silently repeating a request that failed for a real reason (bad key, bad
    argument) wastes quota and hides the true fault from the operator.
    """
    text = f"{type(exc).__name__}: {exc}".lower()
    if any(marker in text for marker in _PERMANENT):
        return False
    return any(marker in text for marker in _TRANSIENT)


class Brain:
    def __init__(self, adapter, client, system: str, max_hops: int = 60,
                 sleep=None, max_retries: int = 5, retry_base_s: float = 2.0):
        self._adapter = adapter
        self._client = client
        self._system = system
        self._max_hops = max_hops
        self._convo = None            # persists across run() — this is the memory
        self._max_retries = max_retries
        self._retry_base_s = retry_base_s
        if sleep is None:
            import time as _t
            sleep = _t.sleep
        self._sleep = sleep

    def reset(self) -> None:
        """Forget the dialogue so far; the next task starts a fresh conversation."""
        self._convo = None

    def _conversation(self):
        """One dialogue per session, so the robot remembers earlier tasks.

        A fresh conversation per task meant the model could not answer
        'continue what you were doing' — and a task killed by a transient
        network error lost all its context, which is precisely when continuity
        matters most.
        """
        if self._convo is None:
            self._convo = self._client.new_conversation(
                self._system, self._adapter.gemini_declarations())
        return self._convo

    def _respond(self, convo) -> ModelResponse:
        """convo.respond() with backoff on transient API failures."""
        delay = self._retry_base_s
        for attempt in range(self._max_retries + 1):
            try:
                return convo.respond()
            except Exception as exc:
                if attempt >= self._max_retries or not _is_transient(exc):
                    raise
                self._sleep(delay)
                delay *= 2

    def run(self, task: str, on_step=None) -> BrainResult:
        """Drive the task to completion.

        on_step(name, args, ok, error, result) is invoked as each tool returns,
        so an interactive console can show work as it happens instead of only
        after the whole task finishes.
        """
        import time as _time
        convo = self._conversation()
        convo.user_text(task)
        trace: list[TraceEntry] = []
        think_s = 0.0
        try:
            for hop in range(self._max_hops):
                _t0 = _time.monotonic()
                resp = self._respond(convo)
                think_s += _time.monotonic() - _t0
                if not resp.calls:
                    return BrainResult(final_text=resp.text, complete=True,
                                       hops=hop + 1, trace=trace,
                                       think_s=round(think_s, 1))
                results: list[tuple[str, dict]] = []
                images: list[bytes] = []
                for call in resp.calls:
                    _c0 = _time.monotonic()
                    r = self._adapter.execute(call.name, call.args)
                    elapsed = round(_time.monotonic() - _c0, 1)
                    payload = dict(r.payload or {})
                    trace.append(TraceEntry(call.name, call.args, r.ok, r.error,
                                            result=payload, elapsed_s=elapsed))
                    if on_step is not None:
                        # a broken display callback must never abort a task that
                        # is physically underway
                        try:
                            on_step(call.name, call.args, r.ok, r.error,
                                    payload, elapsed)
                        except Exception:
                            pass
                    results.append((call.name, r.to_gemini()))
                    if call.name == "look" and r.ok:
                        frame = getattr(self._adapter, "last_frame", None)
                        if frame:
                            images.append(frame)
                convo.tool_results(results, images)
            return BrainResult(final_text=None, complete=False,
                               hops=self._max_hops, trace=trace,
                               think_s=round(think_s, 1))
        finally:
            try:
                self._adapter.stop_all()
            except Exception:
                pass
