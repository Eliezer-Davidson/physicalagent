"""Voice input for the G1 session.

Deliberately the simplest thing that can work, because that is what the
session needed: press Enter, speak, press Enter, see what was heard, press y.

Gemini TRANSCRIBES and decides nothing. It never sees the tool surface and
never picks a verb or a number. The transcript meets parse_command(), which
already refuses rather than guesses -- so the worst a mis-hear can do is
produce a string that fails to parse.

Two facts from this machine, 2026-08-19, that shape every test below:

  * There is NO microphone. arecord exits 0, writes a valid 16 kHz WAV, and
    every sample is zero, because the default source is auto_null.monitor.
    Silence that looks like success is the failure mode to design against.
  * The API is out of credits: generate_content 429s in 0.39 s.

So capture and transcription are both injected, and the mic gate asserts
SIGNAL, never exit status.
"""
from __future__ import annotations

import importlib.util
import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]


def _load(name):
    spec = importlib.util.spec_from_file_location(
        name, ROOT / "scripts" / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def voice():
    return _load("g1_voice")


class TestTheMicGateAssertsSignalNotExitStatus:
    def test_pure_silence_is_rejected(self, voice):
        assert voice.has_signal(b"\x00\x00" * 16000) is False

    def test_real_audio_passes(self, voice):
        import struct
        pcm = b"".join(struct.pack("<h", (i % 400) * 40 - 8000)
                       for i in range(16000))
        assert voice.has_signal(pcm) is True

    def test_a_dead_source_reads_exactly_zero_and_is_rejected(self, voice):
        import struct
        pcm = b"".join(struct.pack("<h", (i % 3) - 1) for i in range(16000))
        assert voice.has_signal(pcm) is False

    def test_a_live_but_quiet_microphone_is_ACCEPTED(self, voice):
        """Calibration, from three captures on the workstation 2026-08-19:

            dead source (auto_null)          peak    0
            live BT mic, quiet room          peak  179
            live BT mic, some room noise     peak  657
            live BT mic, handling noise      peak 6856

        The floor started at 200 and so called a working microphone dead --
        caught only because the operator connected real earbuds and the room
        happened to be quiet. The job of this gate is to tell a DEAD source
        from a LIVE one, and dead reads exactly 0. Room tone that yields an
        empty transcript costs one cheap call and reports 'heard nothing';
        rejecting a live mic costs the whole feature."""
        import struct
        pcm = b"".join(struct.pack("<h", 179 if i % 2 else -179)
                       for i in range(16000))
        assert voice.has_signal(pcm) is True

    def test_the_floor_sits_between_dead_and_a_quiet_room(self, voice):
        assert 0 < voice.SIGNAL_FLOOR < 179

    def test_an_empty_capture_is_rejected(self, voice):
        assert voice.has_signal(b"") is False


class TestTranscriptsBecomeCommandsOnlyThroughTheExistingParser:
    def test_a_clean_transcript_parses(self, voice):
        act, kw = voice.transcript_to_command("walk 0.1 2")
        assert act == "walk" and kw["speed"] == 0.1

    def test_spoken_punctuation_and_case_survive(self, voice):
        act, _ = voice.transcript_to_command("Status.")
        assert act == "status"

    def test_a_garbled_transcript_is_refused_not_guessed(self, voice):
        act, _ = voice.transcript_to_command("wark zero point one")
        assert act == "error"

    def test_a_truncated_motion_verb_cannot_move_the_robot(self, voice):
        """The partial-transcript hazard, which is the whole reason for the
        parser change: a transcript cut to its first word must not be motion."""
        act, _ = voice.transcript_to_command("walk")
        assert act == "error"

    def test_an_empty_transcript_does_nothing(self, voice):
        act, _ = voice.transcript_to_command("")
        assert act == "noop"


class TestNothingMovesWithoutATypedConfirmation:
    """Voice proposes; a keypress disposes. The confirmation is on a different
    channel from the one that produced the command, so a mis-hear cannot also
    approve itself."""

    def test_motion_requires_confirmation(self, voice):
        assert voice.needs_confirmation("walk") is True
        assert voice.needs_confirmation("arms") is True
        assert voice.needs_confirmation("turn") is True
        assert voice.needs_confirmation("pose") is True
        assert voice.needs_confirmation("height") is True

    def test_reads_and_stops_do_not(self, voice):
        """Making the operator confirm a stop is how a stop gets missed."""
        for verb in ("status", "joints", "contacts", "hands",
                     "stop", "estop"):
            assert voice.needs_confirmation(verb) is False, verb

    def test_only_y_confirms(self, voice):
        assert voice.confirmed("y") is True
        assert voice.confirmed("Y") is True
        for other in ("", "n", "yes please", "\n", "q", " "):
            assert voice.confirmed(other) is False, repr(other)

    def test_the_readback_shows_what_will_actually_be_sent(self, voice):
        line = voice.readback("walk", {"speed": 0.1, "seconds": 2.0})
        assert "walk" in line and "0.1" in line and "2" in line


class TestFailuresAreLoudAndCostNothing:
    def test_a_silent_capture_never_reaches_the_api(self, voice):
        calls = []
        out = voice.one_utterance(
            capture=lambda: b"\x00\x00" * 16000,
            transcribe=lambda pcm: calls.append(pcm) or "walk 0.1 2")
        assert calls == [], "spent an API call on silence"
        assert out["ok"] is False
        assert "no audio" in out["message"].lower()

    def test_a_transcription_failure_discards_the_utterance(self, voice):
        def boom(pcm):
            raise RuntimeError("429 RESOURCE_EXHAUSTED credits are depleted")
        out = voice.one_utterance(capture=lambda: _tone(), transcribe=boom)
        assert out["ok"] is False
        assert out.get("command") is None
        assert "credit" in out["message"].lower() or "429" in out["message"]

    def test_a_good_utterance_returns_a_parsed_command(self, voice):
        out = voice.one_utterance(capture=lambda: _tone(),
                                  transcribe=lambda pcm: "arms neutral")
        assert out["ok"] is True
        assert out["command"] == ("arms", {"pose": "neutral"})
        assert out["transcript"] == "arms neutral"

    def test_the_transcript_is_always_shown_even_when_it_fails(self, voice):
        """So the operator learns what the channel actually hears."""
        out = voice.one_utterance(capture=lambda: _tone(),
                                  transcribe=lambda pcm: "wark forward")
        assert out["transcript"] == "wark forward"
        assert out["ok"] is False


def _tone():
    import struct
    return b"".join(struct.pack("<h", (i % 400) * 40 - 8000)
                    for i in range(16000))


class TestTheCaptureCommandIsOneArecordWillAccept:
    """`arecord -d` takes an INTEGER number of seconds:

        $ arecord -q -d 1.0 -f S16_LE -r 16000 -c 1 -t raw
        arecord: main:656: invalid duration argument '1.0'
        exit=1, 0 bytes

    capture_wav built its argv with str(seconds), so the default float 4.0
    became '4.0' and every capture failed -- and the failure surfaced as
    'arecord produced no data (is it installed?)', blaming the machine for the
    caller's argv. It was caught by running --check on real hardware, not by
    any unit test, which is the point of running it."""

    def test_the_duration_is_an_integer_string(self, voice):
        argv = voice.capture_argv(seconds=1.0)
        assert "-d" in argv
        assert argv[argv.index("-d") + 1] == "1"

    def test_a_fractional_duration_rounds_up_rather_than_to_zero(self, voice):
        """0.5 s must not become '0', which records nothing forever."""
        assert voice.capture_argv(seconds=0.5)[
            voice.capture_argv(seconds=0.5).index("-d") + 1] == "1"

    def test_the_normal_case_is_unchanged(self, voice):
        argv = voice.capture_argv(seconds=4.0)
        assert argv[argv.index("-d") + 1] == "4"

    def test_the_format_is_what_the_transcriber_expects(self, voice):
        argv = voice.capture_argv(seconds=4.0)
        assert argv[argv.index("-r") + 1] == str(voice.RATE)
        assert argv[argv.index("-c") + 1] == str(voice.CHANNELS)
        assert "S16_LE" in argv and "raw" in argv

    def test_a_device_is_passed_through_when_given(self, voice):
        argv = voice.capture_argv(seconds=2.0, device="plughw:1,0")
        assert argv[argv.index("-D") + 1] == "plughw:1,0"


class TestCaptureGoesThroughPulseAudioSoBluetoothWorks:
    """Bluetooth earbuds do not appear as an ALSA card. `arecord -l` lists
    hardware devices only, so a paired headset is invisible to it -- while
    PulseAudio exposes it as a source like any other.

    Measured on the workstation, 2026-08-19:
        arecord -L            ->  default, pulse
        arecord -D pulse ...  ->  exit 0, 32000 bytes
        hci0 UP RUNNING, UUIDs include Handsfree Audio Gateway
        pactl modules: module-bluez5-discover, module-bluetooth-discover,
                       module-bluetooth-policy   (all loaded)

    Capturing through `pulse` covers USB headsets, Bluetooth headsets and the
    (currently broken) internal audio with one code path. Capturing from raw
    ALSA covers only the one that does not exist on this machine."""

    def test_capture_goes_through_pulse_by_default(self, voice):
        argv = voice.capture_argv(seconds=2.0)
        assert "-D" in argv
        assert argv[argv.index("-D") + 1] == "pulse"

    def test_an_explicit_device_still_wins(self, voice):
        """So a specific card can be forced when pulse picks the wrong source."""
        argv = voice.capture_argv(seconds=2.0, device="plughw:2,0")
        assert argv[argv.index("-D") + 1] == "plughw:2,0"

    def test_the_duration_fix_survives(self, voice):
        assert voice.capture_argv(seconds=1.0)[
            voice.capture_argv(seconds=1.0).index("-d") + 1] == "1"

    def test_the_no_audio_message_does_not_send_people_to_arecord_dash_l(self, voice):
        """`arecord -l` cannot see a Bluetooth headset. Telling an operator to
        check there would have them conclude a working headset is broken."""
        out = voice.one_utterance(capture=lambda: b"\x00\x00" * 16000,
                                  transcribe=lambda p: "walk 0.1 2")
        msg = out["message"].lower()
        assert "pactl" in msg, "must point at the tool that can see a BT mic"
        # arecord -l may be MENTIONED, but only to say a BT mic is not there.
        if "arecord -l" in msg:
            assert "never appears in" in msg or "not" in msg


class TestTheBluetoothHeadsetIsPutIntoTheProfileThatHasAMicrophone:
    """A Bluetooth headset connects in A2DP by default, which is playback ONLY
    -- `sources: 0`. Its microphone exists only in the HFP profile. Measured on
    the workstation, 2026-08-19 with a Soundcore Space A40 already paired:

        a2dp_sink:           (sinks: 1, sources: 0)   <- connected like this
        handsfree_head_unit: (sinks: 1, sources: 1)   <- the mic lives here

    After switching, the source appears as s16le 1ch 16000Hz -- exactly the
    format the transcriber wants -- and capture read peak=657, rms=36.

    So voice has to make the switch itself. Leaving it to the operator means
    the first attempt records silence from a headset that is working fine."""

    PACTL = """Card #0
	Name: alsa_card.pci-0000_80_1f.3
	Profiles:
		output:analog-stereo: Analog Stereo Output (sinks: 1, sources: 0, priority: 6500, available: yes)
	Active Profile: off
Card #1
	Name: bluez_card.AA_BB_CC_11_22_33
	Profiles:
		a2dp_sink: High Fidelity Playback (A2DP Sink) (sinks: 1, sources: 0, priority: 40, available: yes)
		handsfree_head_unit: Handsfree Head Unit (HFP) (sinks: 1, sources: 1, priority: 30, available: yes)
		off: Off (sinks: 0, sources: 0, priority: 0, available: yes)
	Active Profile: a2dp_sink
"""

    def test_it_finds_the_bluetooth_card_and_its_current_profile(self, voice):
        got = voice.find_hfp_card(self.PACTL)
        assert got is not None
        assert got["card"] == "bluez_card.AA_BB_CC_11_22_33"
        assert got["active"] == "a2dp_sink"
        assert got["hfp"] == "handsfree_head_unit"

    def test_it_ignores_cards_with_no_microphone_profile(self, voice):
        only_alsa = self.PACTL.split("Card #1")[0]
        assert voice.find_hfp_card(only_alsa) is None

    def test_it_does_not_switch_when_hfp_is_already_active(self, voice):
        already = self.PACTL.replace("Active Profile: a2dp_sink",
                                     "Active Profile: handsfree_head_unit")
        got = voice.find_hfp_card(already)
        assert got["needs_switch"] is False

    def test_it_switches_when_the_card_is_in_a2dp(self, voice):
        assert voice.find_hfp_card(self.PACTL)["needs_switch"] is True

    def test_no_bluetooth_card_is_not_an_error(self, voice):
        """A USB headset needs no profile switch at all."""
        assert voice.find_hfp_card("") is None


class TestSpokenEnglishIsNormalisedBeforeTheStrictParserSeesIt:
    """Round-tripped through Gemini TTS then transcription, 2026-08-19:

        said 'status'                  heard 'Status'                -> status   OK
        said 'stop'                    heard 'stop'                  -> stop     OK
        said 'arms wave right'         heard 'arms wave right'       -> ERROR
        said 'walk zero point one two' heard 'walk zero point one two' -> ERROR

    Both failures are real: 7 of the 9 pose names are underscore-joined and
    cannot be spoken, and spoken decimals never parse.

    The fix belongs HERE, not in parse_command. That parser's strictness is a
    safety property of the typed prompt and must not be loosened. Voice instead
    rewrites spoken English into canonical command text -- deterministically,
    from a closed table -- and then hands it to the same strict parser. A
    normaliser that cannot map a phrase leaves it alone, so it gets refused
    rather than guessed.

    Speeds are an ENUM, not spoken decimals. A misheard decimal is a 10x error;
    'slow/medium/fast' are closed, and a mis-hear lands outside the set and is
    refused rather than becoming a different speed."""

    def test_a_spoken_pose_becomes_the_underscored_name(self, voice):
        assert voice.normalize("arms wave right") == "arms wave_right"

    def test_every_real_pose_name_is_reachable_by_speech(self, voice):
        for pose in ("arms_down", "arms_up", "arms_forward", "arms_out",
                     "reach_left", "reach_right", "wave_right"):
            spoken = "arms " + pose.replace("_", " ")
            assert voice.normalize(spoken) == f"arms {pose}", spoken

    def test_single_word_poses_are_untouched(self, voice):
        assert voice.normalize("arms neutral") == "arms neutral"
        assert voice.normalize("arms carry") == "arms carry"

    def test_an_unknown_pose_phrase_is_left_alone_to_be_refused(self, voice):
        """Never invent a pose. 'arms wobble left' must reach the parser as-is."""
        assert voice.normalize("arms wobble left") == "arms wobble left"

    def test_named_speeds_become_the_table_values(self, voice):
        assert voice.normalize("walk slow") == "walk 0.1 2"
        assert voice.normalize("walk medium") == "walk 0.2 2"
        assert voice.normalize("walk fast") == "walk 0.3 2"

    def test_the_named_speeds_are_exactly_the_signed_table(self, voice):
        """fast must be the clamp, never above it."""
        assert max(voice.SPOKEN_SPEEDS.values()) == 0.3

    def test_a_spoken_duration_is_honoured(self, voice):
        assert voice.normalize("walk slow for three seconds") == "walk 0.1 3"

    def test_turn_takes_named_rates_too(self, voice):
        assert voice.normalize("turn left") == "turn 0.2 2"
        assert voice.normalize("turn right") == "turn -0.2 2"

    def test_filler_words_are_dropped(self, voice):
        assert voice.normalize("please walk slow") == "walk 0.1 2"
        assert voice.normalize("robot status") == "status"

    def test_spoken_decimals_are_NOT_invented(self, voice):
        """'zero point one two' stays unparseable on purpose: a spoken decimal
        is where a 10x error hides, and the enum exists to avoid it."""
        act, _ = voice.transcript_to_command(
            voice.normalize("walk zero point one two"))
        assert act == "error"

    def test_normalisation_then_parsing_gives_a_real_command(self, voice):
        act, kw = voice.transcript_to_command(voice.normalize("arms wave right"))
        assert act == "arms" and kw["pose"] == "wave_right"

    def test_walk_slow_survives_the_whole_path(self, voice):
        act, kw = voice.transcript_to_command(voice.normalize("walk slow"))
        assert act == "walk" and kw["speed"] == 0.1 and kw["seconds"] == 2.0
