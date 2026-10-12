#!/usr/bin/env python3
"""Speak commands to the G1 instead of typing them.

The simplest thing that can work, on purpose:

    press Enter -> speak -> press Enter -> see what was heard -> press y

Gemini TRANSCRIBES. It decides nothing. It never receives the tool surface,
never chooses a verb and never chooses a number. The transcript is handed to
g1_sonic_session.parse_command(), which already refuses rather than guesses, so
the worst a mis-hear can do is produce a string that fails to parse. That is a
deliberate choice over letting the model emit a function call directly: a
plausible-but-wrong function call looks exactly like a correct one, whereas a
wrong transcript mostly fails to parse.

WHAT THIS IS NOT
----------------
This is not an emergency stop and must never be treated as one. A spoken stop
needs the network, so it fails intermittently -- and a stop that works most of
the time is worse than none, because it trains you to rely on it. The G1 has no
hardware e-stop at all: the de-facto stop engages
damping and the robot then slowly falls. The real stops are the gantry, the
second person, and Ctrl-C. `stop` and `estop` are reachable by voice as a
convenience and execute without confirmation, but the banner says plainly that
they are best-effort.

TWO FACTS MEASURED ON THIS MACHINE, 2026-08-19
----------------------------------------------
* There is NO microphone. `arecord` exits 0, writes a valid 16 kHz WAV, and
  every sample is zero, because the default source is auto_null.monitor. So the
  mic gate asserts SIGNAL, never exit status -- silence that looks like success
  is the failure this is designed against. Fix with a USB headset; the onboard
  audio is broken at the firmware level and is not worth chasing.
* The API is out of credits: generate_content 429s in 0.39 s. Voice cannot arm
  until that is topped up, and it says so rather than hanging.

Capture and transcription are both injectable so the whole path is testable
with neither a microphone nor an API key. See tests/test_g1_voice.py.
"""
from __future__ import annotations

import array
import os
import subprocess
import sys
import wave

# 16 kHz mono 16-bit: what every speech model wants, and small to upload.
RATE = 16000
CHANNELS = 1
SAMPLE_BYTES = 2

# This gate answers ONE question: is anything actually connected and listening?
# Measured on the workstation, 2026-08-19:
#
#   dead source (auto_null.monitor)   peak    0     <- what we must reject
#   live BT mic, quiet room           peak  179
#   live BT mic, some room noise      peak  657
#   live BT mic, handling noise       peak 6856
#
# This started at 200 and therefore called a LIVE microphone dead the moment
# the room went quiet -- found only because real earbuds were connected and
# nobody happened to be talking. A dead source reads exactly 0, so the floor
# belongs near the bottom, not near speech. Room tone that transcribes to
# nothing costs one cheap call and reports "heard nothing"; rejecting a working
# microphone costs the whole feature.
SIGNAL_FLOOR = 40

# Capture through PulseAudio, not raw ALSA. A Bluetooth headset is NOT an ALSA
# card -- `arecord -l` cannot see one, however well it is working -- but
# PulseAudio exposes it as an ordinary source. Going through `pulse` covers USB
# headsets, Bluetooth headsets and the internal audio with one code path;
# going through raw ALSA covers only the one this laptop does not have.
# Verified on the workstation 2026-08-19: `arecord -D pulse` returns 32000 bytes, and
# module-bluez5-discover / module-bluetooth-{discover,policy} are all loaded.
CAPTURE_DEVICE = "pulse"

# One try, one short deadline. The Brain's retry ladder is 62 s (2+4+8+16+32),
# which is intolerable with a live robot in front of you -- see
# TestDepletedCreditsAreNotRetried in tests/test_brain.py.
TRANSCRIBE_TIMEOUT_S = 3.0

TRANSCRIBE_PROMPT = (
    "Transcribe this audio to plain text, exactly as spoken. "
    "Output ONLY the words. No punctuation, no explanation, no formatting. "
    "If there is no intelligible speech, output nothing at all."
)

# Verbs that move the robot need a keypress. Reads are free, and stops must
# never be gated -- making someone confirm a stop is how a stop gets missed.
MOTION_VERBS = {"walk", "turn", "arms", "pose", "height", "release", "quit"}


def has_signal(pcm: bytes, floor: int = SIGNAL_FLOOR) -> bool:
    """True if this capture contains anything a transcriber could use.

    Asserts the CAPABILITY, not the transport. arecord returning 0 proves only
    that arecord ran; on this laptop it returns 0 while writing pure silence.
    """
    if not pcm or len(pcm) < SAMPLE_BYTES:
        return False
    samples = array.array("h")
    samples.frombytes(pcm[: len(pcm) - (len(pcm) % SAMPLE_BYTES)])
    if not samples:
        return False
    return max(abs(int(s)) for s in samples) >= floor


def _session():
    """The session module, loaded by path.

    scripts/ is not a package and is not importable by name from the test
    runner's cwd, so resolve it relative to this file rather than relying on
    however the caller happens to have set sys.path.
    """
    import importlib.util
    if "g1_sonic_session" in sys.modules:
        return sys.modules["g1_sonic_session"]
    here = os.path.dirname(os.path.abspath(__file__))
    spec = importlib.util.spec_from_file_location(
        "g1_sonic_session", os.path.join(here, "g1_sonic_session.py"))
    mod = importlib.util.module_from_spec(spec)
    sys.modules["g1_sonic_session"] = mod
    spec.loader.exec_module(mod)
    return mod


# Named speeds instead of spoken decimals. A misheard decimal is a 10x error
# ("zero point one" -> "zero point one two"), and while the adapter clamps
# magnitude, 0.1 heard as 0.3 is legal and three times too fast. These three
# words are a CLOSED set: a mis-hear lands outside it and is refused, rather
# than quietly becoming a different speed. The values are the signed table's
# own ceilings, so `fast` is the clamp and never above it.
SPOKEN_SPEEDS = {"slow": 0.1, "medium": 0.2, "fast": 0.3}
SPOKEN_TURNS = {"left": 0.2, "right": -0.2}
SPOKEN_INTS = {"one": 1, "two": 2, "three": 3, "four": 4, "five": 5}

# Words a person says around a command. Dropped before parsing; none of them
# is a verb or an argument, so removing them cannot change a command's meaning.
FILLER = {"please", "robot", "now", "the", "a", "go", "and", "then", "for",
          "seconds", "second", "at", "to", "hey", "ok", "okay"}


def normalize(transcript: str) -> str:
    """Rewrite spoken English into canonical command text.

    Deterministic and table-driven -- no model, no fuzzy matching. Anything it
    cannot map is left EXACTLY as heard, so the strict parser refuses it rather
    than this function inventing a command. That asymmetry is the whole design:
    normalisation may only ever turn a phrase into a command the speaker
    plainly meant, never into a different one.
    """
    words = [w.strip(".,!?;:").lower()
             for w in (transcript or "").strip().split()]
    words = [w for w in words if w]
    if not words:
        return ""
    verb = words[0]
    rest = [w for w in words[1:] if w not in FILLER]
    if verb in FILLER and rest:                 # "please walk slow", "robot status"
        verb, rest = rest[0], rest[1:]

    if verb == "arms" and rest:
        joined = "_".join(rest)
        # Only if it is a REAL pose. Otherwise leave it alone to be refused.
        if joined in _poses():
            return f"arms {joined}"
        return " ".join([verb] + rest)

    if verb in ("walk", "turn"):
        table = SPOKEN_SPEEDS if verb == "walk" else SPOKEN_TURNS
        rate = seconds = None
        for w in rest:
            if w in table and rate is None:
                rate = table[w]
            elif w in SPOKEN_INTS and seconds is None:
                seconds = SPOKEN_INTS[w]
            elif seconds is None and w.isdigit():
                seconds = int(w)
        if rate is not None:
            return f"{verb} {rate} {seconds if seconds is not None else 2}"
        return " ".join([verb] + rest)

    return " ".join([verb] + rest)


def _poses():
    try:
        return set(_session()._pose_names())
    except Exception:
        return {"neutral", "arms_down", "arms_up", "arms_forward", "arms_out",
                "carry", "reach_left", "reach_right", "wave_right"}


def transcript_to_command(transcript: str):
    """Hand a transcript to the SAME parser the typed prompt uses.

    Deliberately not a second grammar: two parsers drift, and the one that
    drifts is the one nobody is watching.
    """
    return _session().parse_command(transcript or "")


def needs_confirmation(verb: str) -> bool:
    return verb in MOTION_VERBS


def confirmed(key: str) -> bool:
    """Only a bare y. Anything else -- including empty, Enter, or 'yes please'
    -- discards. Ambiguity here is motion nobody chose."""
    return (key or "").strip().lower() == "y"


def readback(verb: str, params: dict) -> str:
    """What will ACTUALLY be sent, so the operator confirms the real command.

    Clamping happens downstream in the adapter; this shows the requested value
    and the prompt says so. Never claim a number was accepted unchanged.
    """
    if not params:
        return verb
    body = "  ".join(f"{k}={v}" for k, v in params.items())
    return f"{verb}  {body}"


def find_hfp_card(pactl_cards_text: str):
    """Find a Bluetooth card whose HFP profile carries a microphone.

    A headset connects in A2DP, which is playback only (`sources: 0`). The mic
    exists only in the handsfree profile. Parsing `pactl list cards` is how we
    learn both the card name and whether a switch is needed -- doing it blind
    would fight a USB headset that needs no switch at all.

    Returns None when there is no such card, which is the normal, fine case.
    """
    card = active = hfp = None
    for raw in (pactl_cards_text or "").splitlines():
        line = raw.strip()
        if line.startswith("Name: "):
            name = line[6:].strip()
            if name.startswith("bluez_card."):
                card, active, hfp = name, None, None
            elif card and hfp:
                break                       # next card; keep the one we have
            else:
                card = None
        elif card and line.startswith("Active Profile:"):
            # MUST come first: "Active Profile: handsfree_head_unit" contains
            # "handsfree" and would otherwise be read as a profile listing.
            active = line.split(":", 1)[1].strip()
        elif card and ("handsfree" in line or "headset_head_unit" in line):
            prof = line.split(":", 1)[0].strip()
            # Only a profile that actually exposes a capture source.
            if "sources: 0" not in line and prof and " " not in prof:
                hfp = prof
    if not (card and hfp):
        return None
    return {"card": card, "active": active, "hfp": hfp,
            "needs_switch": active != hfp}


def set_card_profile(card: str, profile: str) -> bool:
    try:
        r = subprocess.run(["pactl", "set-card-profile", card, profile],
                           capture_output=True, timeout=10)
        return r.returncode == 0
    except Exception:
        return False


def ensure_microphone_profile():
    """Put a paired Bluetooth headset into its microphone profile.

    Returns (restore_card, restore_profile) so the caller can put it back --
    HFP is 16 kHz mono, which is right for speech and poor for music, and
    silently leaving someone's earbuds degraded is rude.
    """
    try:
        text = subprocess.run(["pactl", "list", "cards"], capture_output=True,
                              text=True, timeout=10).stdout
    except Exception:
        return None
    found = find_hfp_card(text)
    if not found or not found["needs_switch"]:
        return None
    if set_card_profile(found["card"], found["hfp"]):
        print(f"  [voice] switched {found['card']} to {found['hfp']} "
              f"(its microphone lives there; {found['active']} has none)")
        return (found["card"], found["active"])
    print(f"  [voice] could NOT switch {found['card']} to {found['hfp']} -- "
          f"its microphone will not be available")
    return None


def capture_argv(seconds: float = 4.0, device: str = "") -> list:
    """The arecord command line, built separately so it can be asserted.

    `-d` takes an INTEGER: `arecord -d 1.0` exits 1 with "invalid duration
    argument". str(4.0) gave "4.0" and every capture failed -- while the error
    surfaced as "arecord produced no data (is it installed?)", blaming the
    machine for our own argv. Round UP, never down: a 0.5 s request must not
    become `-d 0`, which records without limit.
    """
    import math
    secs = max(1, int(math.ceil(float(seconds))))
    return ["arecord", "-q", "-d", str(secs), "-f", "S16_LE",
            "-r", str(RATE), "-c", str(CHANNELS),
            "-D", device or CAPTURE_DEVICE, "-t", "raw"]


def capture_wav(seconds: float = 4.0, device: str = "") -> bytes:
    """Record from the default source with arecord. Returns raw PCM.

    Uses a subprocess rather than a Python audio package on purpose: the
    interpreter that drives the robot is NVIDIA's Isaac venv, and installing
    packages into it to get a microphone is a bad trade.
    """
    cmd = capture_argv(seconds, device)
    try:
        out = subprocess.run(cmd, capture_output=True,
                             timeout=seconds + 5.0)
    except FileNotFoundError:
        return b""
    except subprocess.TimeoutExpired:
        return b""
    return out.stdout or b""


def _pcm_to_wav(pcm: bytes) -> bytes:
    import io
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(CHANNELS)
        w.setsampwidth(SAMPLE_BYTES)
        w.setframerate(RATE)
        w.writeframes(pcm)
    return buf.getvalue()


def _api_key() -> str:
    """The project's shared loader: env first, then .env, no dotenv import."""
    import importlib.util
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    spec = importlib.util.spec_from_file_location(
        "physicalagent.apikey", os.path.join(root, "physicalagent", "apikey.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    try:
        return mod.load_gemini_key()
    except SystemExit:
        return ""


def gemini_transcribe(pcm: bytes, api_key: str = "", model: str = "") -> str:
    """One call, one short deadline, no retry ladder.

    Model choice is separate from the ER model on purpose: transcription is not
    robotics reasoning, and pinning them together means a robotics model change
    silently changes how words are heard.
    """
    from google import genai
    from google.genai import types

    key = api_key or _api_key()
    if not key:
        raise RuntimeError("GEMINI_API_KEY not set (put it in .env)")
    model = model or os.environ.get("PHYSICALAGENT_STT_MODEL",
                                    "gemini-3.5-flash")
    client = genai.Client(api_key=key)
    resp = client.models.generate_content(
        model=model,
        contents=[
            TRANSCRIBE_PROMPT,
            types.Part.from_bytes(data=_pcm_to_wav(pcm), mime_type="audio/wav"),
        ],
        config=types.GenerateContentConfig(temperature=0.0),
    )
    return (resp.text or "").strip()


def one_utterance(capture=None, transcribe=None):
    """Capture -> gate on signal -> transcribe -> parse. Never raises.

    Returns a dict: ok, transcript, command, message. `command` is set only
    when a real command parsed; a failure carries the transcript anyway, so the
    operator can see what the channel actually heard rather than guessing.
    """
    capture = capture or capture_wav
    transcribe = transcribe or gemini_transcribe

    try:
        pcm = capture()
    except Exception as exc:
        return {"ok": False, "transcript": None, "command": None,
                "message": f"could not record: {exc!r}"}

    if not has_signal(pcm):
        return {"ok": False, "transcript": None, "command": None,
                "message": ("no audio captured -- the microphone is silent "
                            "or absent. Nothing was sent, and no API call was "
                            "spent. Check `pactl list sources short` for a "
                            "live source: a USB or Bluetooth headset shows up "
                            "there, and a Bluetooth one never appears in "
                            "`arecord -l` at all. If the only source is "
                            "auto_null.monitor, nothing is listening.")}

    try:
        transcript = (transcribe(pcm) or "").strip()
    except Exception as exc:
        text = str(exc)
        if "credit" in text.lower() or "429" in text:
            msg = ("voice unavailable: Gemini API credits are depleted. "
                   "This will not clear by waiting.")
        else:
            msg = f"transcription failed, utterance discarded: {text[:160]}"
        return {"ok": False, "transcript": None, "command": None,
                "message": msg}

    if not transcript:
        return {"ok": False, "transcript": "", "command": None,
                "message": "heard nothing"}

    act, kw = transcript_to_command(normalize(transcript))
    if act in ("error", "noop"):
        return {"ok": False, "transcript": transcript, "command": None,
                "message": kw.get("message", "nothing to do")}
    return {"ok": True, "transcript": transcript, "command": (act, kw),
            "message": readback(act, kw)}


def banner() -> str:
    return "\n".join([
        "=" * 74,
        "  VOICE INPUT -- Gemini transcribes, it does NOT decide",
        "=" * 74,
        "    Enter        start recording, then Enter again to stop",
        "    y            confirm a motion command (anything else discards)",
        "    reads/stops  run immediately, no confirmation",
        "",
        "  THIS IS NOT AN EMERGENCY STOP. A spoken stop needs the network and",
        "  fails intermittently. The real stops are the gantry, the second",
        "  person, and Ctrl-C. Do not let voice change where your hands are.",
        "=" * 74,
    ])


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--check", action="store_true",
                    help="test the microphone and the API, command nothing")
    ap.add_argument("--seconds", type=float, default=4.0)
    ap.add_argument("--keep-profile", action="store_true",
                    help="leave a Bluetooth headset in its microphone profile "
                         "(HFP is 16 kHz mono: right for speech, poor for music)")
    a = ap.parse_args()

    if a.check:
        print("microphone:")
        restore = ensure_microphone_profile()
        pcm = capture_wav(seconds=1.0)
        if not pcm:
            print("  [FAIL] arecord produced no data (is it installed?)")
            return 1
        ok = has_signal(pcm)
        peak = 0
        if pcm:
            s = array.array("h")
            s.frombytes(pcm[: len(pcm) - (len(pcm) % 2)])
            peak = max((abs(int(x)) for x in s), default=0)
        print(f"  {'[PASS]' if ok else '[FAIL]'} peak amplitude {peak} "
              f"(need >= {SIGNAL_FLOOR})")
        if not ok:
            print("         Recorded silence. That is this laptop's normal "
                  "state -- it has no working capture device.")
            print("         Sources PulseAudio can see:")
            try:
                out = subprocess.run(["pactl", "list", "sources", "short"],
                                     capture_output=True, text=True,
                                     timeout=5).stdout.strip()
                for row in (out.splitlines() or ["(none)"]):
                    print(f"           {row}")
            except Exception as exc:
                print(f"           (pactl unavailable: {exc!r})")
            print("         A USB or Bluetooth headset appears in that list. "
                  "A Bluetooth one never appears in `arecord -l`.")
        if restore and not a.keep_profile:
            set_card_profile(*restore)
            print(f"  [voice] put {restore[0]} back to {restore[1]}")
        print("\nAPI:")
        try:
            got = gemini_transcribe(b"\x00\x00" * RATE)
            print(f"  [PASS] transcription answered: {got[:60]!r}")
        except Exception as exc:
            print(f"  [FAIL] {str(exc)[:160]}")
            return 1
        return 0 if ok else 1

    print(banner())
    print("\nThis module is the voice PATH. To drive the robot with it, run "
          "the session:\n  g1_sonic_session.py --real --i-am-at-the-estop "
          "--start --voice")
    return 0


if __name__ == "__main__":
    sys.exit(main())
