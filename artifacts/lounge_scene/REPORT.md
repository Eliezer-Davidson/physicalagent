# Lounge scene A/B report — run 2026-08-12, corrected 2026-08-13, re-verified 2026-08-18

G1 + SONIC in a photo-reconstructed furnished-lounge scene produced by a separate
reconstruction tool (5 furniture bodies, actuated Inspire hands). Same four
prompts in both passes, run by `scripts/lounge_battery_probe.py`.

Commit hashes quoted below refer to the project's private history and do not
resolve in this repository; they date a code version, nothing more.

Every number below was re-read from the committed artifacts or re-measured on
2026-08-18. Numbers that no artifact backs are marked **session-observed** and
are collected in "Measurement provenance" at the end; nothing in this report is
quoted from memory.

## Correction notice — read this before the table

This report previously presented a four-row A/B table and drew two conclusions
from it. A re-audit against the committed traces and the live scene XML found
that **two of the four rows do not measure what their column headings say**, and
that both conclusions were unsupported. Nothing measured has been deleted; what
changed is what the numbers are allowed to mean.

1. **`describe_scene` did not report where objects are.** It reported each
   body's *origin*, and in this scene every furniture body keeps the source
   scan's origin as its frame — the mesh carries the offset. The white table's
   body origin is at `(0.004, -0.670, 0.000)`; the table's collision geometry is
   centred at `(-0.151, -1.649, 0.227)` with its top at `z = 0.453`. The origin
   is **0.99 m** horizontally from the table it names. For the thermos it is
   **0.91 m** away horizontally and **0.51 m** below. For the bench, **1.72 m**.
   For `black_laptop_2_0`, **1.78 m**. Every range, bearing, height and
   `world_xy` the model was given for these objects pointed at a phantom floor
   point, not at the object.
2. **Consequently the `goto_table` row measured the distance to that phantom
   point.** Judged "0.36 m from the table" (A) and "0.37 m" (B); the robots were
   actually **1.04 m** and **1.01 m** from the nearest vertex of the table's
   collision mesh. And neither robot walked: pass A made zero locomotion calls,
   pass B's one `goto_object` call returned `distance_at_start_m: 0.0`. The row
   is not a navigation result.
3. **Consequently the pick failure is a placement failure, not a grasp
   failure.** The thermos was 1.22–1.24 m away and its top was at `z = 0.543`;
   the arm reaches 0.587 m from the shoulder (`where_are_my_hands`). Zero hand
   contacts was geometrically guaranteed before the first `reach_to`.

The measurements themselves stand and are unedited in `battery.jsonl`. What
follows re-reads them.

## Root cause: body origin is not object position

Measured on 2026-08-18 from the live scene XML
(`$WBC_ROOT/gear_sonic/data/robot_model/model_data/g1/scene_29dof_inspire_lounge.xml`)
with a headless `MjModel.from_xml_path` + `mj_forward` on CPU. "Geometry" below
is the axis-aligned box of the body's **collision** mesh vertices at scene load;
the offset column is horizontal (x,y) only.

| body | XML `pos` (body origin) | collision-geometry centre | geometry top `z` | origin → geometry (horiz.) |
|---|---|---|---|---|
| `grey_and_yellow_bench_0` | `0.000  -0.670  -0.003` | `+0.046  -2.391  +0.330` | 0.660 | **1.72 m** |
| `white_round_table_0` | `0.004  -0.670  +0.000` | `-0.151  -1.649  +0.227` | 0.453 | **0.99 m** |
| `black_laptop_1_0` | `0.005  -0.655  +0.013` | `-0.134  -1.764  +0.561` | 0.680 | **1.12 m** |
| `black_laptop_2_0` | `-0.101  -0.526  +0.033` | `+0.369  -2.247  +0.399` | 0.486 | **1.78 m** |
| `black_thermos_0` | `0.006  -0.674  -0.017` | `-0.188  -1.565  +0.492` | 0.543 | **0.91 m** |

The version of `simbridge/sidecar.py::_op_objects` that served these runs
(the version that ran on 2026-08-12) reports `range_m`, `bearing_deg` and
`height_m` from `d.xpos[b]` — the body origin — relative to the pelvis
(`d.qpos[0:3]`). Four of the five origins sit within ~2 cm of each other at
`z ≈ 0`, so `describe_scene` told the model that the bench, the table, laptop 1
and the thermos were all **at the same place, on the floor**, roughly a metre
short of where they really are. The physics is correct: the thermos collision
mesh spans `z = 0.442 … 0.543`, sitting on a table top at `z = 0.453`. Only the
reporting channel was wrong.

This is the single defect behind both broken rows. It is a scene-merge/reporting
bug, not a controller or model bug.

## Results

The raw rows, unedited, with an honest reading of each:

| task | A ok | B ok | A elapsed | B elapsed | A drift | B drift | what the row actually measures |
|---|---|---|---|---|---|---|---|
| see | ✅ | ✅ | 29.6 s | 30.4 s | 0.012 m | 0.025 m | Real. All 5 objects named from the head camera, both passes, with their spatial relations right. |
| goto_table | ✅ | ✅ | 15.2 s | 19.4 s | 0.014 m | 0.015 m | **Not a navigation result.** Two non-navigations judged ok against a phantom point and a 1.0 m tolerance. See below. |
| look_thermos | ✅ | ✅ | 12.0 s | 26.4 s | 0.009 m | 0.023 m | Real, and weak: the judge only requires the word "thermos". The image check is done by eye (it passes). |
| pick_thermos | ❌ | ❌ | 69.1 s | 279.7 s | 0.056 m | 0.227 m | Real failure, wrong attribution. Object was ~1.2 m away and ~0.5 m up. Both passes ran out of hops. |

Verbatim judges, exactly as committed:

```json
passA: {"see": {"ok": true, "named": ["grey_and_yellow_bench_0", "white_round_table_0", "black_laptop_1_0", "black_laptop_2_0", "black_thermos_0"]},
        "goto_table": {"ok": true, "distance_to_table_m": 0.36, "furniture_shoved": false},
        "look_thermos": {"ok": true, "note": "image checked by eye in the report"},
        "pick_thermos": {"ok": false, "hand_contacts": 0, "lift_m": 0.001}}
passB: {"see": {"ok": true, "named": ["grey_and_yellow_bench_0", "white_round_table_0", "black_laptop_1_0", "black_laptop_2_0", "black_thermos_0"]},
        "goto_table": {"ok": true, "distance_to_table_m": 0.37, "furniture_shoved": false},
        "look_thermos": {"ok": true, "note": "image checked by eye in the report"},
        "pick_thermos": {"ok": false, "hand_contacts": 0, "lift_m": 0.004}}
```

Evidence per task: `passA/` and `passB/` hold `<task>_reply.txt` (full trace
repr), `<task>_pov.jpg` where the model looked, `battery.jsonl`, `summary.json`.
`<task>_window.png` exists for all four pass B tasks and for **none** of pass A —
pass A's four window grabs all failed and each failure is recorded in its own row
(`"screenshot": {"ok": false, "note": "grab-window rc=1 fresh=False"}`, 4/4).
Pass A has no `goto_table_pov.jpg` and pass B has no `pick_thermos_pov.jpg`; in
both cases the model called no `look` during that task (verified against the
trace call lists) and the harness refused to reuse a stale frame.

## What the goto row actually measured

Both rows are non-navigations, and the judge could not tell.

**Pass A** (`passA/goto_table_reply.txt`), the entire trace:

```
get_status → stop
```

Zero locomotion calls. The robot was already at `(0.088, -0.321)` when the task
began, having drifted there during the preceding `see` task. Its own final text
is honest about this — *"I have stopped and am **already** standing directly in
front of the white round table (coffee table), which is positioned just 0.35
meters ahead of me."*

**Pass B** (`passB/goto_table_reply.txt`) called `goto_object` once. Its own
committed result:

```
'reason': 'arrived', 'distance_at_start_m': 0.0,
'start_m': {'x': 0.24, 'y': -0.39}, 'goal_m': {'x': 0.24, 'y': -0.39},
'end_m':   {'x': 0.24, 'y': -0.39}, 'final_error_m': 0.0, 'cycles': 6
```

Start = goal = end. Zero travel. `standoff_point()` behaved exactly as written:
the robot was 0.37 m from the *phantom* table point, inside the 0.6 m standoff,
so it correctly held position and only turned to face
(`final_heading_error_deg: -10.8`). Pass B's final text nonetheless claims
*"I have walked to the white round table (coffee table) and stopped directly in
front of it"* — a hallucination, preserved here because it is committed evidence
of a failure mode we must catch, not hide.

How far each robot really was from the table, measured three ways from its own
committed end pose against the load-time scene geometry:

| pose | to the body origin (what the judge used) | to the nearest collision-mesh vertex | to the nearest point of the collision-mesh AABB |
|---|---|---|---|
| pass A `(0.088, -0.321)` | 0.36 m | **1.04 m** | 1.02 m |
| pass B `(0.24, -0.39)` | 0.37 m | **1.01 m** | 0.95 m |

The AABB column is a lower bound (a square envelope around a round table pokes
out at the corners, so the true nearest-surface distance sits between the two
columns); the vertex column is the honest headline. Either way the judged number
is wrong by about 0.65 m, in the flattering direction.

Three separate things had to line up for both rows to be judged `ok`:

- `GOTO_TOL_M = 1.0` in the probe version that ran
  (the probe version that ran on 2026-08-12, line 48), against start
  distances of 0.36 m (A) and 0.37 m (B) to the point being judged. The
  tolerance was larger than the distance to be closed, so the task was passed
  before it began.
- That judge was endpoint-only: `judge_goto(pose, table_xy)` reads the final
  pose and never asks whether the robot moved, or whether it was already there.
- The target it measures against is the phantom body origin, 0.99 m from the
  table.

`passB/goto_table_window.png` is the visual proof, and it has been viewed for
this report: after this row was judged "0.37 m from the table", the robot is
standing in open floor with only the extreme edge of the table clipping the
bottom-left corner of the frame.

Also visible in that trace and never acted on:

```
'warning': 'frame offset was assumed to be zero, not measured. If the robot
            walked off at an angle, that is why — call calibrate_heading() once
            per session and try again.'
'frame_offset_measured': False
```

`calibrate_heading()` appears in no trace in either pass. The tool warned
correctly and nothing consumed the warning.

## The pick outcome, honestly

**No grasp was achieved in either pass.** Measured: `hand_contacts: 0,
lift_m: 0.001` (A) and `hand_contacts: 0, lift_m: 0.004` (B). Millimetre-scale
`lift_m` is settling noise, not lift. Pass B called `get_contacts` four times
and every returned row is a floor contact — `left_ankle_roll_link`,
`right_ankle_roll_link`, `left_knee_link`, `right_knee_link`, all
`touching: 'world'`, 37–148 N — with no furniture contact of any kind, at any
sample. That measurement stands.

What this failure is **not** evidence of is a grasp-control gap. At pass B's last
measured pick pose the thermos was 1.08 m away at its nearest collision-mesh
vertex (1.24 m at the pose it started the task from), with its top at
`z = 0.543`; the arm reaches 0.587 m from the shoulder. The object was never
inside the workspace. This report previously wrote that the pass B robot
*"engaged the scene, couldn't close the hand on the object"*. It did not engage
the object; it engaged a phantom on the floor.

The trace shows the phantom driving the behaviour step by step:

- In pass A's pick task `describe_scene` reports the thermos at
  `height_m: -0.79` with the pelvis at 0.785 m — i.e. **world z ≈ -0.005, the
  floor**. Pass B's first pick-task `describe_scene` reports `height_m: -0.53`
  with the pelvis already down at 0.526 m — the same world z, the same floor.
- The scene-aware SYSTEM block added in `957a93c` instructs: *"reach_to() just
  above it"*. Pass B obeyed the channel it was told to trust.
- It called `set_height(0.5)`, then `set_mode('squat')`, then
  `set_mode('kneel_two_legs')` — dropping the pelvis from 0.785 m to
  0.395–0.411 m to get down to the reported height. `kneel_two_legs`' own reply
  carries `'tested_on_this_robot': False` (`squat`'s carries `True`).
- Its **first** `reach_to` target was `z = -0.45` pelvis-relative. Across all 16
  `reach_to` calls it never once aimed above `z = -0.20` (the highest solved
  palm position was `z = -0.199`). With the pelvis at ~0.40 m that is world
  `z ≈ -0.05 … +0.20`. The thermos is at `z = 0.44 … 0.54`. It was reaching a
  quarter to half a metre below the object, a metre short of it.
- **Eight of the sixteen** `reach_to` calls were refused by the reachability
  check (`'nothing_was_commanded': True`, *"cannot reach that point … the arm
  reaches about 0.59 m from the shoulder, so walk closer"*); eight executed. The
  refusals were correct.

Pass A, on the pre-empowerment surface, did **not** obey the reported height: it
stayed standing (pelvis 0.785–0.787 m) and its five `reach_to` targets
(`z = -0.25 … -0.16`, i.e. world `z = 0.535 … 0.627`) were at roughly the right
*height* and only wrong in *range*. Three of the five were refused, two
executed. Its last act before the hop cap cut it off was
`walk(vx=0.1, duration_s=0.8)` — it had read the "walk closer" refusals and was
starting to close the distance. This is n=1 per arm and must not be over-read,
but the traces are unambiguous about what each pass did.

Both passes ended `complete=False, final_text=None`: pass A at `hops=12`
(`think_s=63.8` of `elapsed_s=69.1`), pass B at `hops=40` (`think_s=262.6` of
`279.7`). Neither model gave up; both ran out of hops.

Reported joint-tracking gaps, re-grepped from `passB/pick_thermos_reply.txt`
(`"Largest gap: N deg"`, in call order): 22.9, 22.3, 22.5, 23.8, 24.0, **46.3**,
24.2, 20.9 — so 20.9° to 46.3°, eight values. Pass A's two are 13.1° and 15.6°,
taken while standing. The larger pass B gaps are not a like-for-like comparison:
they were measured in `kneel_two_legs`, where the balance controller is fighting
the arm reference much harder.

Whatever the arm-tracking story turns out to be, it cannot be read out of this
run — the object was never reachable, so nothing here tests the grasp.

## `drift_m` cannot tell a shove from a settle

Pass B's pick row records `drift_m: 0.227`, and this report previously read that
as *"it moved 0.227 m of furniture"*. There is no evidence for that reading.

`drift_m` is `max_drift_m` over the world positions of the furniture **body
origins**, each derived from the robot's own odometry plus the phantom
`range_m`/`bearing_deg`, taken once before and once after the task. Only the
maximum survives into `battery.jsonl` — **no per-body breakdown is committed**,
so which body produced the 0.227 m cannot be read off the artifacts at all.

What can be read off the artifacts is the same derived quantity as the *model*
saw it, across the four `describe_scene` calls inside the pick task:

| body | describe_scene #1 | #2 | #3 | #4 | max pairwise |
|---|---|---|---|---|---|
| `white_round_table_0` | (0.00, -0.67) | (0.01, -0.67) | (0.01, -0.67) | (0.00, -0.67) | 0.01 m |
| `grey_and_yellow_bench_0` | (0.00, -0.67) | (-0.00, -0.67) | (0.00, -0.67) | (-0.00, -0.67) | 0.00 m |
| `black_laptop_1_0` | (0.00, -0.66) | (0.00, -0.65) | (-0.00, -0.66) | (0.00, -0.65) | 0.01 m |
| `black_thermos_0` | (0.06, -0.70) | (0.12, -0.73) | (0.06, -0.71) | (0.05, -0.71) | 0.07 m |
| `black_laptop_2_0` | (0.01, -0.50) | (0.04, -0.49) | (0.06, -0.49) | **(0.15, -0.47)** | 0.14 m |

Three of the five bodies hold to ±0.01 m; `black_laptop_2_0` moves 0.14 m
monotonically and the thermos wanders 0.07 m. The probe's own before/after
bracket is wider than these four samples, so 0.227 m is consistent with
`black_laptop_2_0` continuing to drift — but that attribution is an inference
from the trace, not a committed measurement, and it is recorded here as one.

Two things make the inference physically plausible and the metric useless
either way:

- `black_laptop_2_0`'s real geometry was **1.69 m** (nearest collision-mesh
  vertex) to **1.86 m** (collision-geometry centre) from the robot at the pick
  pose, and no
  `get_contacts` sample in either pass shows contact with it or with any other
  furniture body.
- Its body origin sits **1.78 m** from its own mesh. On that lever arm a
  rotation of ~7° swings the origin 0.22 m while the laptop travels nowhere.
  (Arithmetic, not a measurement: 0.22 / 1.78 = 0.12 rad.)

That makes `furniture_shoved = drift_m > 0.05` a scalar that cannot distinguish
"the robot shoved the furniture" from "a body a metre and a half away rotated a
few degrees while settling" — and one that cannot even say which body it is
talking about. It is the stoplist law in miniature, inside our own probe. It has
to be rebuilt on MuJoCo's own world poses (the sidecar's `body_pose` op) and
report per-body displacement.

## Was this an A/B at all?

No. It cannot answer the question it was set up to answer, for four independent
reasons:

1. **Three variables moved at once.** Pass B added `goto_object` + `world_xy` in
   `describe_scene` (`fe506a4`, which touched `sonic_g1.py` + a new test), and
   a scene-aware SYSTEM block **and** a hop-cap default of 40 vs pass A's 12
   (both in `957a93c`, a single 14-line commit to `sonic_console.py`). The
   project's own law is one component change at a time against a golden
   baseline.
2. **The hop caps differ by 3.3×,** and pass A hit its cap mid-recovery on the
   only task that failed. The "richer failure evidence" in pass B is what 28
   extra hops buy; it is not attributable to the frame bridge.
3. **n = 1 per arm.**
4. **Two of the four rows measure a phantom**, so even a clean design would have
   produced two meaningless rows.

The previous verdict — *"the empowerment moved the failure from 'couldn't engage
the scene' to 'engaged the scene, couldn't close the hand'"* — is withdrawn. It
is contradicted by pass A's own trace (which opened the hand, ran five `reach_to`
calls, and was walking closer when the cap hit) and by the geometry (neither
pass ever had the object in reach).

The claim that pass B *"consumes `world_xy` and `goto_object` directly instead of
reasoning about bearings"* is half true and worth stating precisely: pass B did
call `goto_object('white_round_table_0')` — name resolution worked, and the tool
executed live. But what it consumed was a phantom coordinate, and in the `see`
task the model's answer references no `world_xy` at all; it describes the scene
from the camera image, exactly as pass A did. A tool being used is not a tool
adding value.

## What IS proven, and stays proven

None of this is retracted:

- **The merged scene loads and is physically sound.** `nu = 53`, `nq = 95`,
  `nv = 89`, `nbody = 72`; robot spawn `(0, 0, 0.793)`; nearest furniture
  collision-mesh vertex 1.34 m away (the white table; its collision-mesh AABB
  footprint is the same 1.34 m); all five bodies carry `<freejoint/>`; the thermos rests
  on the table top (`z = 0.442…0.543` on a top at `z = 0.453`).
- **The robot stood and operated through a full four-task battery without
  falling, twice** (`"fell": false` on all 8 rows of `passA` + `passB`).
- **The model named all five objects from its head camera in both passes**, and
  got their *relations* right — thermos and laptop 1 on the table, laptop 2 on
  the bench — which is information the numeric channel did not contain and
  could not have supplied. Re-verified against the real `final_text`, not the
  trace repr. This is the strongest result in the run, and the phantom channel
  makes it stronger, not weaker.
- **`object_world_xy`'s trigonometry is correct**, and was validated in flight:
  across pass B's `see` task the range to the table went 0.69 → 0.55 → 0.36 m
  and the bearing -96.6° → -45.4° → -18.8°, while the recovered `world_xy` held
  at (0.01,-0.67) / (0.00,-0.67) / (0.00,-0.67) — constant to ±0.01 m. The
  bridge is right; its input was wrong.
- **`goto_object` resolved a name to an object and executed live.** The
  resolution half of the tool works.
- **The pick produced no grasp.** 0 hand contacts, mm-scale lift, no thermos
  contact anywhere in either trace.
- **The `grab-window` fix works**: 0/4 captures before, 4/4 after.
- **The void run was correctly excluded** as not-an-A/B-datum.
- **The judge defect was found and disclosed** rather than quietly patched.

## Operational discovery: the robot moves when nothing told it to

**`turn` translates.** During the `see` task, both robots moved forward while
issuing nothing but `turn` calls. Pass A:
`describe_scene → look → turn → look → describe_scene → turn → look →
describe_scene`, range to the table 0.61 → 0.51 → 0.35 m. Pass B:
`describe_scene → look → turn → describe_scene → look → turn → describe_scene →
look`, range 0.69 → 0.55 → 0.36 m. No `walk`, `goto` or `goto_object` call
appears in either trace, and the turns were only
`yaw_rate=-1.0, 1.5 s` and `yaw_rate=-0.6…-0.8, 1.0 s`.

Because the object is static and its recovered `world_xy` held constant, the
range collapse is a lower bound on how far the *robot* moved: **at least 0.26 m
(A) and 0.33 m (B)**. The robots' own odometry agrees and puts it higher — the
first pose read of the next task is `(0.088, -0.321)` for A and `(0.24, -0.39)`
for B, i.e. 0.33 m and 0.46 m from the session origin.

**Mode changes translate too.** Pass B's pick task contains no locomotion call
of any kind, yet its two `get_status` reads bracket a move from `(0.272, -0.384)`
to `(0.278, -0.562)` — **0.18 m**, across the `squat` → `kneel_two_legs`
transition.

Three consequences, all real:

- Any "turn to look around" behaviour silently repositions the robot. This is
  precisely what pre-closed the `goto_table` distance in both passes.
- Posture changes are locomotion as far as the world frame is concerned.
- A probe that wants a known start pose must re-measure it after any `turn` or
  `set_mode`, never assume it.

## Judge-defect caveat

The battery rows above were produced by a judge that scored `str(BrainResult)` —
the full trace repr, which includes every tool call the model made — instead of
the model's actual `final_text`. Because `describe_scene` results always name
every scene object, a repr-based judge cannot fail the `see` task regardless of
what the model said.

The `see` and `look_thermos` verdicts in both passes have been independently
re-verified against the models' real `final_text`, parsed straight out of the
committed `*_reply.txt` traces, and all four stand unchanged (`ok: true`): each
`see` answer names the table, both laptops, the bench and the thermos, and each
`look_thermos` answer says "thermos". `goto_table` and `pick_thermos` never
depended on model text.

Three residual weaknesses in that channel, all visible in the committed traces:

- **`look_thermos`'s judge only requires the substring.** It cannot fail a model
  that names the thermos without looking at it. The image check is the real
  evidence and it is done by eye: `passB/look_thermos_pov.jpg` — viewed for this
  report — shows the thermos standing on the white table beside the open laptop,
  with the bench behind. Check passes; but a human eye is not a pinned judge.
- **`final_text` is contaminated.** **Five of the six** completed replies begin
  mid-word — `"0Based on…"`, `"ating the scene:…"`, `"se the robot has
  arrived…"`, `"es.\nThe thermos…"`, `"'s a grey/black…"` — and only pass A's
  `goto_table` is clean. Four of the six also concatenate raw deliberation onto
  the user-facing answer (`"…I will report this to the user.I have walked to the
  white round table…"`). The extraction is dropping a leading fragment and not
  separating thinking from answer. Now that judges read `final_text`, that is on
  the scoring path.
- **The phantom leaked into the answers.** Pass A's `see` reply tells the user
  the bench is *"about 0.35 meters away"* and pass B's `goto_table` reply says
  the table is *"about 0.37 meters away, well within reach"*. Both are the
  phantom number, restated to a human as a measurement. A model that faithfully
  reports a wrong instrument produces a confident wrong answer, and no judge in
  this battery could catch it.

Going forward the harness writes the clean `final_text` to `<task>_final.txt`
alongside the full trace in `<task>_reply.txt`. No committed run has exercised
that yet — see "Status of the shipped code".

## Void run

`passB_void_gpu_contention/` (not in this repository; source-history commit `f00b5c9`) is a first pass B attempt that is
**not** an A/B datum: it was run while a co-tenant GPU load (Isaac Sim RTX
actively rendering) was believed to be starving the SONIC control loop, and the
robot fell on every task — 0/4, all `"fell": true` in its `battery.jsonl` (not
in this repository, so here this is **session-observed**), with 13 `*** FALL DETECTED ***` lines in
`artifacts/live/passB_void_console.log` (not in this repository; see *Measurement
provenance* below). Its own `README.md`, also not shipped, records that the
`file` paths inside its own `battery.jsonl` are pre-move and now resolve into
`../passB/`; the correct files are that directory's own siblings.

**The fall outcome is evidenced. The contention diagnosis is not.** The
`LowState age` figures (28–56 ms starved vs 2–8 ms healthy), the planner
inference time (17.7 ms/tick) and the 1 % GPU utilization at rerun are all
**session-observed** and appear in no committed artifact — see "Measurement
provenance" below, which also records what the one surviving loop-timing log
actually says, and why it does not support the "healthy 2–8 ms" half of that
claim.

What *is* on disk and does support the operational lesson: the void run's own
gate line, `GATES OK: gpu_free=8154MiB ram_avail=12092MiB no rival stack`. The
launch gates checked free VRAM and RAM but not GPU *utilization* — a compute
co-tenant can starve a 500 Hz loop while leaving 8 GB of VRAM free. That gate
has since been written (`7e3518a`): a CUDA compute process holding >1500 MiB, or
utilization ≥60 % in **all** of 5 samples, now fails `scripts/lounge_live.sh`.
It has never gated a live run.

## Evidence-channel note

Pass A's window screenshots all failed (`grab-window rc=1`, recorded per row):
mutter exposes pixel-less reparenting frames, and the old tool `xwd`'d the frame.
Fixed in `~/.local/bin/grab-window` (child-first, honest needle-miss failure)
mid-session; pass B captured all four. An XWayland window only holds pixels while
displayed — the window must be frontmost during evidence runs. Note that
`grab-window` lives outside this repo and is not versioned with it.

## Status of the shipped code

**Nothing that fixes anything in this report has ever produced a live run.**

Committed and post-dating both passes:

- `ca36f97` — judges read `final_text` not the trace repr; wrist contacts
  counted but never grasp-qualifying; stale poses refused; full-precision
  control path.
- `7e3518a` — the GPU co-tenant gate.

The cheapest artifact-only proof that no committed row was scored by any of
this: `judge_pick` at `HEAD` emits a `wrist_contacts` field, and that field is
absent from every row in both `battery.jsonl` files.

Uncommitted in the working tree at the time of writing, and likewise never run
live: the geometry fix itself. `simbridge/sidecar.py` now carries
`body_extent()` and `footprint_range_m()`, and `_op_objects` reports
`range_m`/`bearing_deg`/`height_m` to the **collision-box centre** plus `top_m`,
`size_m`, `nearest_edge_m` and a `measured_from` field that names the fallback
when a body has no bounding geometry. `scripts/lounge_battery_probe.py` carries
a rebuilt `judge_goto` (`JUDGE_VERSION = 2`) that requires fresh poses at both
ends, a stable target, a start distance *outside* tolerance (else `invalid_no_op`
— reported SKIPPED, never ok and never a robot failure), an upright endpoint, a
travel tool actually called, and ≥ `GOTO_MIN_TRAVEL_M = 0.25` of real
displacement; `GOTO_TOL_M` is now 0.8. Tests: `tests/test_lounge_battery.py`
72 passed, `tests/test_sidecar_geometry.py` (new, untracked) 15 passed,
107 passed across those plus `test_sonic_world.py` and `test_goto_object.py` in
0.13 s, CPU only.

Those tests prove the arithmetic. They do not prove the model does anything
better with a correct scene. Nothing does yet.

## What would actually prove the thing

The A/B above cannot be repaired by re-reading it. It has to be re-run. Items
marked **[landed, unvalidated]** exist in the working tree and have never served
a live reply; the rest are open.

**Fix first, then measure.**

1. **[landed, unvalidated] Report object geometry, not body origins.**
   `_op_objects` now derives range/bearing/height from the body's collision-box
   centre, and reports `top_m` and `nearest_edge_m` alongside. Pinned by
   `tests/test_sidecar_geometry.py`. Until a live run serves this reply, no
   scene task in this scene means anything.
2. **Rebuild `drift_m` on `sim.body_pose()`** — MuJoCo's own world poses — and
   report per-body displacement rather than a single max. A shove and a distant
   settle must produce different numbers, and the row must name the body. Still
   open: the probe's drift path still goes through odometry-derived origins.
3. **Make `goto_object` refuse a stale frame offset.** It warned that the frame
   offset was assumed, not measured, and the warning went nowhere. Either call
   `calibrate_heading()` automatically once per session or fail closed. Still
   open: `calibrate_heading` exists as a tool and nothing forces it.

**Then the navigation test, designed so it can fail:**

4. **Start pose requiring ≥ 2 m of travel** to the goal standoff. The current
   spawn is 1.34 m from the nearest furniture geometry, and both robots began
   the goto row already inside the standoff radius — having closed the distance
   during `see` without being asked to.
5. **[landed, unvalidated] A tolerance below the start distance, and a travel
   assertion.** `judge_goto` v2 refuses to score a row whose start distance is
   inside tolerance (`invalid_no_op`), requires a travel tool from
   `TRAVEL_TOOLS` (`turn` deliberately excluded), and requires
   ≥ 0.25 m of displacement. `GOTO_TOL_M` is 0.8. Against a ≥ 2 m start that is
   a real bar; against a start 1.0 m from the table it is not, so item 4 is not
   optional.
6. **Distance measured to the object's geometry**, not to any origin, and
   cross-checked once by hand against the XML — as the table in this report was.
7. **Matched hop caps** across arms. If the cap is a variable, it gets its own
   A/B.
8. **One variable at a time.** `goto_object` + `world_xy` is one arm. The
   scene-aware SYSTEM block is a second. The hop cap is a third. Three arms, not
   one bundle.
9. **n ≥ 3 per arm**, reporting every run including the falls, with the void
   criteria fixed in advance rather than after seeing the outcome — and with the
   loop-timing log preserved per run (see below), so a void call is evidenced
   rather than remembered.

**And for the pick, before any grasp claim:**

10. **A reachability precondition asserted by the harness, not the model**: log
    the distance from each shoulder to the object's nearest mesh point at task
    start, and mark the task `not-attemptable` when it exceeds the 0.587 m arm
    reach. A pick task the robot could not physically attempt must never be
    scored as a manipulation failure again.

## Measurement provenance

**From the committed artifacts** (`passA/`, `passB/`; the void run's
`passB_void_gpu_contention/` is not in this repository, so its figures are
session-observed here): every `elapsed_s`, `drift_m`, `fell`, judge
verdict, screenshot/pov status, tool call, tool argument and tool result quoted
above; both `summary.json` hop caps (12 / 40); every `final_text`.

**Re-measured on 2026-08-18, headless and CPU-only** (`MjModel.from_xml_path` +
`mj_forward` on the live scene XML; no viewer, no GPU, no simulator launch):
`nu/nq/nv/nbody`, the spawn pose, every body origin and collision-geometry
extent in the root-cause table, every distance in the goto and pick sections,
and the spawn clearance. Method is recorded inline where each number appears.
These are load-time (authored, CEM-settled) poses; the bodies drift during a run
by the amounts the `drift_m` section records.

**Not committed, on disk, and volatile** — `artifacts/live/` is untracked:
`passA_console.log`, `passB_console.log`, `passB_void_console.log`, `policy.log`
and three `pose.csv.*`. `policy.log` is opened with `>` by `scripts/sonic_live.sh`
and therefore holds only the **last** launch — the successful pass B rerun, not
the void run. Its 364 sampled `Loop timing` lines give `LowState age` min
0.67 ms, median 14.3 ms, p90 30.0 ms, p99 46.7 ms, max 67.2 ms.

**Session-observed, and now contradicted in part.** The void-run diagnostics —
`LowState age` 28–56 ms starved vs 2–8 ms healthy, planner inference
17.7 ms/tick, 1 % GPU utilization at rerun — were read live off the console and
`nvidia-smi` during the session and appear in no artifact, committed or
otherwise. The one surviving loop-timing log is the *rerun's*, and a median of
14.3 ms with a tail to 67 ms is not a 2–8 ms healthy baseline. Either the
healthy figure was misremembered, or `LowState age` is not the discriminator the
void call assumed it was. **The void run's fall outcome was read from its
`battery.jsonl` (0/4, all `fell: true`; not in this repository, so
session-observed here); its attribution to GPU contention is
not, and this report no longer treats it as established.** Preserving the
per-run policy log is now item 9's rider.
