# `recover_from_fall` trial logs — what each file is

MuJoCo only (SONIC policy in the vendor's own simulator). One JSONL line per
event; the lines that matter are `"msg"` lines of the form
`prone trial N: verb says X, body says Y` — the verb's own return value against
an independent check of the body (pelvis ≥ 0.70 m, only feet in contact, feet on
the floor, sustained 3 s).

| File | What it records |
|---|---|
| `trials-run1-prone10-realfall2.jsonl` | **The prone tally: 10/10**, verb and body agreeing on every trial, plus 2 induced real-fall recoveries. Ends when the robot drifts 11.17 m from the band anchor and the harness correctly refuses to re-engage. |
| `trials-run2-realfall.jsonl` | The **third** shoulder-drop recovery, 1/1. Stops for the same harness-geometry reason. |
| `trials-run3-afterdamp.jsonl` | **0/1 after auto-damp**, and 0/1 again after a `start_control` re-arm (the single retry recorded in the same file) — the verb's one failure, and the reason the auto-damp interaction is called out rather than buried. Console transcript in `run3-afterdamp.log`. |
| `trials-BEFORE-fix.jsonl` | The run **before** the standing criterion was fixed: `verb says False, body says True <-- THEY DISAGREE`. Kept because it is what made the disagreement visible. |
| `trials.jsonl` | A **later, aborted 2026-08-07 run**. It is not part of any tally: the simulator's band recovery failed and the run stopped after 2/10 with *"could not reset the robot to standing"*. Kept for the harness failure mode, not for an outcome. |
| `isaac-prone-20260807.log`, `zerodrop-validation.log` | Side runs: the Isaac-side prone attempt, and the zero-drop validation. |

The tally quoted in the front-page README — **prone 10/10, shoulder-drop 3/3,
0/1 after auto-damp** — comes from the first three files only.
