# G1 hardware run — 2026-08-19

> **Read this first.** The robot was harness-held (hoisted) for both control
> windows. The only locomotion figure in this file is a net pelvis displacement
> of 0.19 m over 632 s of commanded authority, and that figure includes gantry
> pendulum motion; no clean locomotion-speed measurement exists. Whether the
> feet bore weight at any point is not recorded. The capture every number below
> is read from, `artifacts/hw/20260819-153703-day0-run3/`, is archived offline
> and is not part of this repository.

First session in which a **physical Unitree G1, hoisted, stepped and held
commanded arm poses under SONIC**, driven from this repo's typed session.
Everything below is read out of the offline capture, not from memory.

## What actually happened

Two windows of control in one 1341 s recording:

| window | duration | what |
|---|---|---|
| t+0 .. t+209 s | 209 s | zero authority — SONIC up, not commanding |
| t+209 .. t+628 s | 419 s | **COMMANDING** — first control window |
| t+628 .. t+1128 s | 500 s | zero authority — cool-down between attempts |
| t+1128 .. t+1341 s | 213 s | **COMMANDING** — walk + poses |

Authority is measured as any non-zero `kp` in `rt/lowcmd`: peak `kp` 99, `kd` 6.3
across all 35 motors. Feed-forward torque stayed 0 — this is a pure position
controller.

Locomotion, from `rt/odommodestate` pelvis position:

```
  t+1240s .. t+1243s   2.1s   peak 0.42 m/s   travelled 0.16 m
  t+1319s .. t+1320s   1.4s   peak 0.31 m/s   travelled 0.05 m
  net displacement over the session: 0.19 m
```

The robot is **hoisted**, so pelvis velocity includes gantry pendulum motion and
is not a clean gait measurement. Peak 0.42 m/s exceeds the 0.3 m/s commanded
ceiling for that reason — it is not evidence the clamp failed, and it is not
evidence the clamp held either. **Open: no clean locomotion speed measurement
exists yet.** It needs an unloaded run or a pelvis-velocity measurement that
subtracts the suspension.

## The thermal event — the important result

`right_shoulder_pitch` (motor 22), **driver** sensor, while holding a raised-arm
pose:

```
  t+1321.4s   65 C
  t+1327.9s   91 C     <- crossed 90 C
  t+1332.9s  106 C     <- peak
  t+1345.0s   79 C     (falling, arm returned to neutral)
```

**215 °C/min.** Over the same window the *winding* sensor moved 53 → 56 °C. A
driver stage has almost no thermal mass and the copper has a lot, so the driver
is both the first thing to get hot and the only one that can be read in time.
`motorstate` stayed 0 throughout and bus voltage held 46.0–46.5 V, so this was
not a fault code — it was real dissipation.

Fastest sustained rise, per motor:

| motor | °C/min | peak | margin 70 → 90 °C |
|---|---|---|---|
| 22 right_shoulder_pitch | 215.6 | 106 | **5.6 s** |
| 14 | 167.7 | 84 | 7.2 s |
| 15 left_shoulder_pitch | 137.8 | 88 | 8.7 s |
| 11 right_ankle_roll | 137.7 | 85 | 8.7 s |

### Consequence

The guidance given earlier in the session — *"watch the temperature on every
`status`"* — was **wrong**, and was generalised from the ankle's ~4 °C/min in an
earlier window. At 215 °C/min there are under 6 seconds of margin. No operator
types that fast. This is why `thermal_watch` now runs on its own thread and acts
without being asked.

The earlier `right_ankle_roll` warning was separately confirmed real: 42 → 71 °C
over 7 minutes of control, plateauing rather than running away, against 40 → 44 °C
on its mirror. A genuine, repeatable left/right asymmetry. **Open: unexplained.**

## Bugs found and fixed

1. **`status` reported "SONIC IS NOT IN CONTROL" while the robot was walking.**
   `controller()` passed the `_Sample` wrapper to `controller_from_message`
   instead of `.msg`. `command_authority` reads
   `getattr(msg, "motor_cmd", None) or []`, so the wrapper hit the defensive
   default, read an empty motor list, and reported zero gains. `thermal()` and
   `pose()` unwrapped correctly, which is why heat and pose were accurate in the
   same readout. **The defensive `getattr` default is what hid it — nothing ever
   raised.** Regression test asserts every `*_from_message` call site passes a
   message.

2. **Every typed motion command was refused on hardware.** `run_manual` sent
   `{"name": ...}` to a tool requiring `pose`, and `{"speed": ...}` to one
   requiring `vx`. All prior tests drove a **mock** adapter that accepted any
   key — they proved the session could talk to a fake robot. Tests now read
   required parameters off the live `ToolSpec`s.

3. **A recorder started while the link was down bound DDS to Wi-Fi** and wrote
   six zero-byte files for 34 minutes; CycloneDDS picks its interface once, at
   startup, so it would never have recovered. `hw_recorder` now refuses without
   a `192.168.123.x` interface, and `--iface` does not bypass the check.

## What Gemini did

**Nothing.** `--start` runs `parse_command` → `run_manual` → `adapter.execute`
directly. The Gemini path is `run_task()`, reached only via `--task`, which was
not used. This run demonstrates the *tool surface* Gemini would drive, operated
by hand. Gemini-in-the-loop on hardware remains **undemonstrated**.

## Still open

- Clean locomotion speed measurement (suspension confounds pelvis velocity).
- Why the right ankle heats ~7× faster than the left.
- `thermal_watch` has been tested against captured numbers, **never fired on
  hardware**. Its first real trip is itself an experiment.

---

# Session 2 — Gemini Robotics-ER 2 in the loop

Same day, after credits were topped up. This is the session in which ER-2 was
first put in the loop, against `FakeSonicLink`; the hardware run came the next day
([hardware-2026-08-20-er2-driving.md](hardware-2026-08-20-er2-driving.md)).

## ER-2 drives the robot's tools (against `FakeSonicLink`)

Default model moved from `gemini-robotics-er-1.6-preview` to
**`gemini-robotics-er-2-preview`**, confirmed available via `models.list()`, with
`PHYSICALAGENT_ER_MODEL` as a gantry-side rollback that needs no code edit.

Verified against `FakeSonicLink` before any hardware:

```
get_status -> set_arms(pose=wave_right) -> get_status -> set_arms(pose=neutral)
"...raised the right hand in a wave, and returned the arms to neutral
 without walking."
```

It chose `wave_right` unprompted from "raise your right hand in a wave", and
honoured "do not walk".

**Conversation persists across prompts.** One Brain per session, not per prompt:

```
>>> Raise your right arm into a wave.        4 calls, 12.6 s
>>> Good. Now put that same arm back to neutral.
                                             1 call,   5.1 s
```

The follow-up resolved "that same arm" with no restatement. `task <text>` and
`ask <text>` reach it from the held prompt; `reset` clears the model's memory
without touching the robot.

The measured thermal numbers are now in ER-2's system prompt, and it acts on
them unprompted — in the first test it returned the arm to neutral on its own
"to prevent the sh[oulder overheating]".

## The `status` fix, verified in BOTH directions on hardware

```
SONIC IS IN CONTROL
  gains seen: kp 99.0984  kd 6.3088  tau 0.0
```

Session 1 recorded that this readout said "NOT IN CONTROL" while the robot was
walking. The negative case (SONIC down) and now the positive case (SONIC
commanding) both read correctly on the physical robot.

## A crash AFTER control had been taken

```
[session] start_control() ...
[session] {'started': True, 'planner_mode': True}
NameError: name 'T_MOD' is not defined
```

A rewrite of an adjacent function removed a module-level import and left one
reference behind. **1554 tests passed**, because none of them executes the
`--start` banner — that line runs only after a real link is opened.

The cost was specific: the crash landed *after* `start_control()` succeeded, so
SONIC held the robot with no session and no thermal watch, while
`right_ankle_roll` sat at 79 °C. A crash *before* control would have been
harmless.

Fixed by reading the limit from its single definition at call time. Guarded by
`TestEveryGlobalNameTheScriptsUseActuallyExists`, which walks the AST of all
five scripts and asserts every global name they load is defined — no linter is
installed here, so the test is the linter. Two of its own cases prove it
catches the exact failure and does not cry wolf.

## Voice input works, on hardware already owned

No purchase needed. A paired **Soundcore Space A40** connects in `a2dp_sink`,
which is playback only (`sources: 0`); the microphone exists solely in the
`handsfree_head_unit` (HFP) profile, which the code now switches into and back
out of automatically.

```
bluez_source.AA_BB_CC_11_22_33.handsfree_head_unit   s16le 1ch 16000Hz
capture:  arecord -D pulse -f S16_LE -r 16000 -c 1     peak 22820
```

`arecord -l` can never see a Bluetooth microphone — it lists ALSA hardware
only. Capture therefore goes through `pulse`, which covers USB, Bluetooth and
internal audio in one path.

TTS round-trip through the full voice path:

```
said 'arms wave right'           -> ('arms', {'pose': 'wave_right'})
said 'walk slow for two seconds' -> ('walk', {'speed': 0.1, 'seconds': 2.0})
said 'turn left'                 -> ('turn', {'yaw_rate': 0.2, 'seconds': 2.0})
```

Gemini transcribes only; it never sees the tool surface and never picks a verb
or a number. Transcripts are normalised deterministically from a closed table,
then handed to the same strict parser the typed prompt uses — anything the
table cannot map passes through untouched and is refused rather than guessed.

Silence transcribed to the word `'of'` — a hallucination from nothing. The
signal gate stops that reaching the API and the parser refuses it regardless,
which is the clearest argument for keeping the model out of verb selection.

## Calibration corrected by real hardware

The microphone signal floor started at 200. Measured: dead source **0**, live
mic in a quiet room **179**, with noise **657**, speaking **22820**. The floor
sat *above* a working-but-quiet microphone and called it dead. Now 40 — the
gate's job is telling dead from live, and dead reads exactly zero.

## Still open at end of day

- `right_ankle_roll` reached 79 °C again and still heats far faster than its
  mirror. **Unexplained.**
- The thermal watch has still **never fired on hardware**.
- Locomotion speed remains unmeasurable while the robot is hoisted.
