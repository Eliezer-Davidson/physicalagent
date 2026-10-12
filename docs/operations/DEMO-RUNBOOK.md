# G1 demo runbook — ER-2 in the loop

Verified 2026-08-19. Everything below was run; nothing is aspirational.

Snapshot as of 2026-08-19. The thermal watch fired live the next day (2026-08-20,
12:02) — see [hardware-2026-08-20-er2-driving.md](../evidence/hardware-2026-08-20-er2-driving.md).

## Pre-flight (do these in order, each must pass)

Every command below uses full paths on purpose. Shell variables do NOT cross
terminals, and a runbook whose first line is `export` breaks the moment you
open a second window -- as it did on 2026-08-19, giving
`bash: scripts/g1_sonic_session.py: Permission denied`, which reads like a
robot problem and is not one.

```bash
cd "$PA_ROOT"

ip -br addr show enp132s0          # must read UP + 192.168.123.222
"$WBC_PY" scripts/hw_preflight.py         # must read PREFLIGHT OK
PYTHONPATH=. "$WBC_PY" \
  scripts/g1_sonic_session.py --check   # cold temps, robot alive
```

`--check` is read-only and constructs no link. Note the hottest motor before
starting; `right_shoulder_pitch` is the one to watch.

## Terminal 1 — recorder (start BEFORE SONIC)

```bash
"$WBC_PY" scripts/hw_recorder.py --label demo --seconds 0
```

`--seconds 0` runs until Ctrl-C — use it for a demo of unknown length;
a fixed 3600 stops mid-session with no warning. Refuses to start if the
robot link is down, so it cannot silently record nothing.

## Terminal 2 — SONIC (whole-body control)

```bash
cd "$WBC_ROOT"/gear_sonic_deploy && ./deploy.sh --input-type zmq_manager real
```

Wait for `Init Done`. Zero gains until control is taken — nothing moves.

## Terminal 3 — the session

**Typed:**
```bash
PYTHONPATH=. "$WBC_PY" \
  scripts/g1_sonic_session.py --real --i-am-at-the-estop --start
```

**Voice:** add `--voice`. It switches the Bluetooth headset into its microphone
profile and back out on exit.

**ER-2 driving, interactively — this is the one to demo.** Use `--start` and
then type `task <what you want>` at the prompt. Control is HELD the whole time,
so the robot stays up between prompts, and the model keeps its conversation:

```
g1> task wave your right hand, then return to neutral
g1> task good, now do that again but slower
g1> ask how tall are you standing right now?
g1> reset                 # clear the model's memory; robot untouched
g1> status
```

`--task "…"` still exists for a single scripted prompt, but it exits when done,
which drops control and leaves the robot limp between runs.

```bash
PYTHONPATH=. "$WBC_PY" \
  scripts/g1_sonic_session.py --real --i-am-at-the-estop \
  --task "Read your status, then wave your right hand, then return to neutral. Do not walk." \
  --max-hops 8
```

## Verified before the robot

| thing | status | evidence |
|---|---|---|
| ER-2 reachable | PASS | `generate_content` 1.38 s |
| ER-2 tool-calling | PASS | 5 calls, correct sequence, on FakeSonicLink |
| ER-2 picks pose names unprompted | PASS | chose `wave_right` from "wave your right hand" |
| Voice mic | PASS | Soundcore Space A40 via HFP, peak 22820 |
| Voice transcription | PASS | TTS round-trip: "status"->status, "stop"->stop |
| Thermal watch | armed | tested against captured numbers, **never fired on hardware** |
| `status` authority readout | fixed | negative case verified live; **positive case unverified on hardware** |

## Watch during the demo

- **`right_shoulder_pitch`.** It reached 106 °C on 2026-08-19 holding a raised
  arm, rising 215 °C/min — under 6 s from 70 to 90 °C. The thermal watch polls
  5x/s and will stop and return the arms to neutral on its own at 90 °C. Do not
  rely on reading temperatures yourself; you cannot type fast enough.
- **Keep the robot hoisted.** The fall guard is report-only and its envelopes
  assume a standing robot.
- **`quit` releases control and the robot goes limp.** `stop` keeps control.

## One thing still unproven

1. The thermal watch has never fired on hardware. Its first trip is an
   experiment, not a routine — see HW-DAY2-PLAN.md P1-A for the deliberate
   live-fire at a lowered `--thermal-act-c`.

(`status` saying "SONIC IS IN CONTROL" WAS verified live in both directions
on 2026-08-19 session 2 — "kp 99.1 across 35 motors" while commanding, and
NOT-IN-CONTROL before control was taken. This line previously listed it as
unproven; [hardware-2026-08-19-g1-sonic.md](../evidence/hardware-2026-08-19-g1-sonic.md) is the record.)
