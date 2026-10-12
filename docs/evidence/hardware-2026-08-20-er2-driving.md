# G1 hardware — day 2 (2026-08-20)

Second hardware day. The 14 GB of raw DDS captures behind
this write-up are archived offline and are not part of this repository; every
number below is quoted from them.

---

## Headline: Gemini Robotics-ER 2 drove the physical G1

The 16:12 run is the first run in this project in which a Gemini model commanded the
physical G1 (on 2026-08-19 the Gemini path was exercised only against `FakeSonicLink`).
For ~6 minutes of continuous authority the G1 held commanded arm poses and
stepped, **entirely through `task` and `ask`** — no typed verbs.

> **Correction, made against the session recording (2026-09-22).** This
> section originally said the robot "stood on the floor bearing its own weight
> (not hoisted), and walked". The recording shows otherwise: the robot remained
> in its gantry harness for the whole session, its legs stepped **in the air**,
> and it did **not** translate forward. The model's own narration in the run
> says so — "its legs moved in the air to simulate the steps, but it did not
> translate forward in space." The claim has been removed rather than softened.
> `set_grasp(hand=both, grasp=index_point, strength=firm)` also failed on this
> run — *"finger grasp control is currently unavailable"* — so **no grasp
> occurred**. See `docs/evidence/er2-hardware-toolcalls-2026-08-20.md`.

This closes the gap left open on 2026-08-19, where ER-2's tool-calling was
proven only against `FakeSonicLink`. The chain
*natural language → ER-2 → tool call → SONIC → 29 motors* now has hardware
evidence: `artifacts/hw/20260820-161210-day1-rehang-check` (463 MB, 356 s of
authority; archived offline, not in this repository).

Not yet demonstrated: voice input on hardware. `scripts/g1_voice.py` works
(Bluetooth HFP path proven 08-19) but was never wired into a real session —
the day ended first.

---

## Run table

| Start | Label | Authority | Size | What it was |
|---|---|---|---|---|
| 12:00 | `day1-battery1` | 121 s | 176 MB | Battery `sim_match_v1`, hoisted. **Aborted t+20 s** on thermal trip (idle baseline). |
| 15:23 | `day1-rehang-check` | 248 s | 326 MB | After re-hang (feet clear of floor). Ankle pitch overheated **again**. |
| 16:12 | `day1-rehang-check` | 356 s | 463 MB | **ER-2 driving via `task`/`ask`.** Arm poses + in-air stepping, on the harness. Went well. |

All three captured 6 topics: `rt/lowstate` (~1052 Hz), `rt/lowcmd` (~505 Hz),
`rt/odommodestate`, `rt/lf_lowstate`, `rt/secondary_imu`, plus the 2 Hz
thermal re-subscription. Zero write errors; 58 records dropped after close in
the 16:12 run (end-of-file only, harmless).

---

## The ankle question, settled: no hardware fault

**Verdict: the motors are healthy. The controller is commanding ankle pitch
past the joint's mechanical limit, and the resulting standoff is the heat.**

Evidence, from the captures:

| joint (16:12 run) | mean tracking error | corr | mean \|τ\| |
|---|---|---|---|
| every hip, knee, waist, shoulder | **0.002 – 0.056 rad** | 0.66–0.98 | 0.3–6.4 N·m |
| L_ankle_**roll** | −0.016 rad | 0.82 | 0.73 N·m |
| R_ankle_**roll** | +0.022 rad | 0.83 | 0.53 N·m |
| L_ankle_**pitch** | **+0.339 rad** | 0.14 | **9.60 N·m** |
| R_ankle_**pitch** | **+0.513 rad** | 0.59 | **14.49 N·m** |

- Commanded ankle pitch reaches **+1.35 / +1.38 rad**. The joint's limit is
  **+0.5236 rad** (`gear_sonic_deploy/g1/g1_29dof.xml:72,111`) — the command
  is **2.6× the mechanical limit**. Measured pitch never exceeds +0.17 rad.
- A persistent 0.3–0.8 rad gap at kp 28.5 is 10–15 N·m of continuous torque
  producing **no motion** — pure resistive heating.
- **No motor fault word was ever nonzero**, any motor, any capture, either day.
- It is **not new and not today's rig**: the 2026-08-19 commanded-walk run (hoisted) shows the
  same signature (L +0.152 rad / 4.34 N·m, R +0.307 rad / 8.67 N·m).
- Right consistently worse than left (0.51 vs 0.34 today; 0.31 vs 0.15 on
  day 0) — this, and nothing wrong with the motor, is the whole of the
  "right ankle heats ~7× faster" mystery from day 0.

### Correction: my 2026-08-19/20 "hang geometry" explanation was WRONG

The earlier claim — that the ankles heated because the hoist held the body
while the balance policy pressed the commanded stance into the floor — is
**refuted by this day's own data**:

- the 15:23 run had the feet **clear of the floor** and the ankles heated anyway;
- the 16:12 run showed the same ankle-pitch standoff (mean error 0.51 rad,
  14.5 N·m) under a completely different command stream;
- the signature is present in day 0's commanded-walk run (hoisted), and in every capture on both
  days.

Hoisting modulates severity. It is not the cause. The cause is an ankle-pitch
target the joint cannot reach, present in every session ever recorded.

> **Note on this argument after the 2026-09-22 correction from the unpublished session recording.** As
> originally written, the middle bullet leaned on the 16:12 run having been
> weight-bearing on the floor. The recording says it was not. What survives
> unchanged is the measurement: the standoff is present with the feet clear of
> the floor (15:23), in the day-0 run, and in the 16:12 run, i.e. across every
> rig configuration recorded — and the commanded target is 2.6x the joint's
> mechanical limit regardless of how the robot is supported. The conclusion
> stands on the command range, not on the posture.

### Why this matters for sim-matching

This is a **sim-real mismatch expressing itself as heat**. SONIC emits ankle
pitch targets valid in some model but not on this robot. Highest-value check
away from the robot: does the MuJoCo ankle pitch joint carry the real limit
**[−0.87267, +0.5236] rad**, and is the policy's action ever clamped? A sim
that lets the ankle reach +1.38 rad teaches a policy to use travel the
hardware does not have.

`mode_pr` is **not** the culprit: the deploy hard-codes `mode_pr = 0` (series
pitch/roll) and never reassigns it
(`g1_deploy_onnx_ref.cpp:2170,2663`), and ankle **roll** — same mode, same
linkage, same firmware path — tracks to 0.02 rad.

---

## Why the steps were short (and went nowhere)

`walk` is **velocity × duration**, not a step count:
`_walk(vx, vy, duration_s)` → `set_velocity(...)` then sleep
(`physicalagent/robots/sonic_g1.py:809-828`). "Two steps forward" has no
representation in the tool surface, so ER-2 can only approximate it with a
short, slow velocity hold — which on a biped is a weight-shift wiggle rather
than two steps.

Two things to fix, and they may be the same bug:
1. **Tool surface**: there is no `step(n)` verb. Either add one, or document
   that distances come from `goto`/duration and teach the system prompt to
   convert "N steps" into a duration.
2. **The ankle standoff above.** The ankle is how a biped pushes off. A pitch
   joint that cannot reach its commanded dorsiflexion will produce exactly
   this: short, shuffling, in-place gait. **Fixing the ankle command range is
   the first thing to try for stride length** — do not tune gait parameters
   before ruling it out.

---

## HW-DAY2-PLAN outcome

| Item | State |
|---|---|
| Thermal watch fires for real | **DONE** — live at 90 °C, 12:02 |
| Watch escalation to damp | **DONE** — written and tested (1650 green) after the watch's first action failed to remove a leg load |
| ER-2 on physical hardware | **DONE** — 6 min of ER-2-driven arm poses and in-air stepping, on the harness |
| Ankle asymmetry explained | **DONE** — mechanism found, hardware exonerated |
| Battery + comparator built | **DONE** — `batteries/`, `scripts/sim_match_compare.py` |
| **Paired hw/sim battery capture** | **NOT DONE** — see below |
| Provenance photos | **NOT DONE** — `photos/` still empty |
| `foot_force` ground truth | **PARTIAL** — captured in the 16:12 run, never analysed. (The robot was harnessed throughout, so this is not the clean weight-bearing data the plan wanted.) |
| Clean locomotion speed | **NOT DONE** |
| Voice on hardware | **NOT DONE** |

### The one real miss: no complete battery capture

Neither day-2 battery run produced a comparable capture.

- **12:00** aborted at step 4 of `sim_match_v1` (thermal trip during the idle
  baseline) — only two marks, a 4 ms segment.
- **16:12** ran long and well, but the **recorder was started 20 s after the
  battery's last logged mark**, so the verb log and the capture do not
  overlap. `sim_match_compare.py` correctly refuses with exit 2:
  `"rt_lowstate has no records inside any common segment"`.

So there is **no hardware side for a hardware-vs-sim comparison of the
battery**. The comparator itself is proven (self-compare of battery1 renders
the full per-joint RMSE/corr report and SVG overlays) — it simply has nothing
to compare yet. The 14 GB of raw captures remain fully usable for
trajectory-level and per-joint analysis; it is the *segment-aligned battery
protocol* that did not land.

**Lesson for the runbook**: start the recorder first, then verify overlap
*before* the run, not after. A recorder that starts late fails silently — both
processes report success.

---

## Away from the robot

```bash
# terminal 1 — recorder against the sim's loopback DDS (--allow-any-net is REQUIRED)
cd "$PA_ROOT"
python scripts/hw_recorder.py --label sim-battery1 --seconds 0 --allow-any-net

# terminal 2 — same battery file the hardware ran
./scripts/sonic_live.sh --script batteries/sim_match_v1.txt

# then compare (needs a hardware side with overlapping segments — see miss above)
PYTHONPATH=. python scripts/sim_match_compare.py \
  --a <hw_capture_dir>  --a-log artifacts/sessions/<ts>-script.jsonl --a-label hw \
  --b <sim_capture_dir> --b-log artifacts/sessions/<ts>-script.jsonl --b-label sim \
  --out artifacts/compare/first
```

Calibration targets the captures can already supply, no robot needed:
ankle pitch/roll limits and the command range actually used; per-joint kp/kd
as flown; the 1052 Hz lowstate / 505 Hz lowcmd rate pair; IMU at rest;
thermal rise-per-N·m per motor.

---

## Open questions

1. **Where does the out-of-range ankle-pitch target come from** — policy
   output, the PR-mode linkage IK, or a joint-zero convention mismatch? Answer
   this in sim; it is the highest-value thread left.
2. Does clamping ankle-pitch commands to the real limit lengthen the stride?
3. `foot_force` in the 16:12 run — captured, still unanalysed. A clean
   weight-bearing measurement needs a run off the harness, which never happened.
4. Voice → ER-2 → hardware, never run end to end.
