# Gemini Robotics-ER 2 issuing tool calls to a physical Unitree G1

**2026-08-20, hardware session.** The operator typed natural-language
tasks into `scripts/g1_sonic_session.py`; `gemini-robotics-er-2-preview` planned
and issued the tool calls; NVIDIA SONIC executed them on the real robot.

**Session totals, from the recording: 7 typed inputs, 18 tool calls, round
trips 10.1 s to 31.3 s.** Eleven of the eighteen calls are transcribed
call-by-call in the table below; the remainder sit behind the rows marked
*not transcribed*, and are not broken out because the recording was not legible
enough to break them out honestly.

## Why this file exists, and what it is not

The raw DDS capture behind this session is ~14 GB and is archived offline, not in
this repository. The session itself was screen-recorded, and that recording is
**deliberately not published**: it shows a hostname, developer home paths, a
LAN address whose link-local IPv6 encodes a NIC MAC, process ids and an unrelated
application pane. Publishing frames of it would leak all of that.

So the lines below are **re-typeset by hand from that recording**. They are a
transcription, not a machine-exported artifact, and they are labelled as one.
Treat this as the weakest evidence in the repository — every other number in the
README traces to a committed file. Rows marked *not transcribed* are calls that
are part of the 18-call total but whose exact arguments or timing were not
legible enough to quote; they are left blank rather than reconstructed.

## The session banner

```
TARGET: PHYSICAL G1 via SONIC (raw rt/lowcmd)

  velocity clamp     |vx| <= 0.3   |vy| <= 0.2   |wz| <= 0.5
  command expiry     <= 5.0 s, no latching
  stop_all           on every exit path; Ctrl-C escalates to emergency_stop
  telemetry          rt/odommodestate measured at 497 Hz

FALL GUARD: REPORTS ONLY. The robot hangs at ~0.731 m. KEEP HOISTED.

[session] thermal watch ARMED (stop + arms neutral at 90 C, polled 5x/s).
          It acts on its own: motor 22 went 65 -> 91 C in 6.5 s.
[session] Gemini Robotics-ER model: gemini-robotics-er-2-preview
          SONIC holds balance; ER only calls the tools above it.
```

## The tasks, as transcribed

| typed input | tool calls ER-2 issued | round trip |
|---|---|---|
| `task walk 2 steps foward` | `get_status`, `walk(duration_s=1.5, vx=0.2)`, `get_status` | 14.3 s |
| `task take 5 steps forward` | `walk(duration_s=3.5, vx=0.2)` *(bracketing calls not transcribed)* | *not transcribed* |
| `walk foward 1 meter` | **none — refused before any model call** by the argument-count guard | — |
| `task walk forward 1 meter` | *not transcribed* | *not transcribed* |
| `task wave hi` | `set_arms(pose=wave_right)`, `set_arms(pose=neutral)` | 16.9 s |
| `task move your fingers` | `set_grasp(hand=both, grasp=index_point, strength=firm)` → **FAILED** | 10.1 s |
| `task change through the poses you know` | `set_arms(arms_out)`, `set_arms(arms_up)`, `set_arms(carry)`, `set_arms(neutral)` | 31.3 s |

The 10.1–31.3 s spread is the cost of a cloud model in the loop, and the reason
SONIC — not the model — is what keeps the robot upright. Nothing in this design
lets a model latency spike become a fall: commands expire after 5 s without
latching, and `stop_all` runs on every exit path.

## The two results that are failures, stated plainly

**No grasp occurred.**

```
> task move your fingers
[er] set_grasp(hand=both, grasp=index_point, strength=firm)
[tool] ERROR: finger grasp control is currently unavailable
```

**No walking occurred.** The robot stayed in its gantry harness for the whole
session. Its legs stepped in the air; it did not translate forward. The model
said so itself, in its own summary of the first task:

> "its legs moved in the air to simulate the steps, but it did not translate
> forward in space"

**A malformed input was refused, not guessed at.** `walk foward 1 meter` — no
`task` prefix — was rejected by the argument-count guard before any model call.
The parser refuses what it cannot map; it never asks the model to fill a gap.

## What this does and does not establish

Established: a cloud embodied-reasoning model (Gemini Robotics-ER 2) chose tools, filled their
arguments, and those calls reached the motors of a physical humanoid through a
whole-body controller, under a typed safety layer, with an operator at the stop.

Not established: locomotion, grasping, autonomy without an operator at the
keyboard, or any behaviour off the harness.

Corroborating write-up, with the session's measured joint and thermal data:
[`hardware-2026-08-20-er2-driving.md`](hardware-2026-08-20-er2-driving.md).
