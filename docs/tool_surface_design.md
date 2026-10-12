I have all four grounding files. Here is the synthesized final recommendation.

---

# ROSPug Tool Surface — Final Recommendation

## KEY CROSS-PANEL DECISIONS (resolved disagreements)

| Disagreement | Decision | One-line reason |
|---|---|---|
| Units: m/rad vs cm/deg | **cm + degrees at the boundary; adapter converts to SI before the SafetyGuard** | LLMs reason reliably about "30 cm / 45°", poorly about "0.30 m / 0.785 rad"; the clamp still runs in verified SI. |
| Signed single param vs direction words | **Direction-word enums everywhere** (`walk` dir ∈ {forward,backward,left,right}; `turn` dir ∈ {left,right}) with a positive magnitude | The project was repeatedly burned by yaw/axis **sign flips** (§J, §E); a word-enum removes the entire sign-error class at the model boundary. |
| Separate `walk_forward`/`walk_backward`/`strafe` vs one `walk` | **One `walk` with a direction enum; adapter applies per-direction caps** | Same `velocity_move` path for all four; one tool is tighter, and the rear/side blind-spot warning + smaller caps live in the adapter, not extra tools. |
| Height convention (−15…−8 vs 8…15) | **Positive ride-height cm, ~8 crouch … 13 normal … 15 tall**; adapter maps `height_m = −ride_cm/100` | "Taller = bigger number" is far less error-prone; the safety-panel's `−15=crouch` was also **kinematically inverted** (more leg extension = taller, so −0.15 m is the *tall* stance, default −0.13 m). |
| Expose roll in pose | **No** — pitch + height only, roll held at 0, yaw forced to 0 | Roll has no nav/perception use, its hardware sign is unverified, and omitting it removes a footgun; pitch is load-bearing (only way to aim the fixed camera). |
| Full 28 action groups vs none | **Curated in-place safe subset only** (postures + gestures) | Preserves expressive/HRI capability while cutting every dynamic tip/collision trajectory (boxing, stairs, kick, 2-leg, spacewalk); locomotion is done with `walk`/`turn`, not `forward_little`/`multi_turn`. |
| Expose odometry pose | **Yes, as RELATIVE `get_odometry` (drifts ~0.15°/s, resettable, "not a compass")** | `turn` already depends on rf2o being up, so the plumbing exists; relative dead-reckoning genuinely enables "explore and return," and the framing manages the drift risk. No absolute heading is ever exposed. |
| AprilTag detector | **Include `detect_apriltags`** | Only vendor detector giving hard 6-DoF metric ground truth, and unlike object_tracking/visual_patrol it does **not** drive `/cmd_vel` — zero motion-contention risk. |
| Model-latching e-stop vs recoverable stop | **One non-latching `stop` for the model; the latching hard e-stop stays operator/watchdog-only** | Reactive navigation needs a recoverable halt; irreversible latching is a human/deadman responsibility, so a confused model can always halt danger but can never brick its own session. |
| Raw velocity / cmd_vel / move_base / heartbeat / e-stop-reset tools | **All excluded** | The `velocity_move` path is **unclamped** (config.py:231-236) so the adapter must own speed; move_base bypasses the guard and its launch kills the camera (§K); the heartbeat is orchestrator-only; the latch is operator-only. |

Surface size: **12 tools** (5 read-only, 5 motion, 1 zero-motion output, 1 abort). If it must shrink, cut in this order: `signal` → `detect_apriltags` → `reset_stance` → `get_odometry`.

---

## RECOMMENDED TOOL SURFACE

### 1. `get_robot_status` — read-only
**moves_robot:** false **closed_loop:** false

> Read the robot's current safety and telemetry state. Call this BEFORE any motion and whenever you need to decide whether it is safe to move. Returns: `battery_mv` and `battery_state` ('ok' at ≥11100 mV, 'warn' at 10500–11099 mV, 'critical' below 10500 mV) — the robot REFUSES all motion below 11100 mV no matter what you request, and voltage SAGS under load and recovers at rest, so check this first and pace yourself; `deadman` ('ALIVE' or 'PAUSED') — motion only runs while ALIVE, and PAUSED means the supervision link dropped and the robot has already stopped itself; `estop` (true/false) — if true a human or the safety system has latched an emergency stop and NO motion runs until a human re-arms the robot, which you cannot do; `roll_deg` and `pitch_deg` (body tilt against gravity, trustworthy); `foot_height_cm` (real body ride-height from leg kinematics, ~8 crouched to ~15 tall); `max_servo_temp_c` (hottest of the 12 servos — watch for overheat during sustained motion). Heading/yaw is intentionally NOT reported because the IMU yaw drifts ~1.4°/s and is unusable. This never moves the robot.

```json
{"type":"object","properties":{},"required":[],"additionalProperties":false}
```

### 2. `capture_image` — read-only
**moves_robot:** false **closed_loop:** false

> Capture one still image from the robot's forward head camera (640×360) and return it for you to look at. Use it to perceive the scene, identify and point to objects, and judge where it is safe to move before committing. The camera is FIXED and looks straight ahead at the current body pitch — it cannot pan or tilt on its own. To aim it up or down, use `set_body_pose` pitch; to aim it left or right, use `turn` to rotate the body. The call has an 8-second timeout and returns an error instead of ever hanging if no frame is available. This does not move the robot.

```json
{"type":"object","properties":{},"required":[],"additionalProperties":false}
```

### 3. `scan_surroundings` — read-only
**moves_robot:** false **closed_loop:** false

> Get a 360° obstacle summary from the lidar. Returns the nearest obstacle distance in centimetres for each of 8 body-relative sectors (`front`, `front_left`, `left`, `back_left`, `back`, `back_right`, `right`, `front_right`, each a 45° wedge) plus the single closest obstacle as `{direction, distance_cm}`. Returns under ~12 cm are the robot's own body and are removed. Lidar range is ~3 cm to 12 m. IMPORTANT: the lidar sees one thin horizontal slice at body height, so low objects (cables, thresholds, stair edges/drop-offs) and overhangs may not appear, and glass is invisible — when unsure, also take a photo. Use this to check clearance BEFORE walking, backing up, or strafing; note that `walk` also refuses on its own if something is closer than your requested distance plus a margin in the travel direction. This does not move the robot.

```json
{"type":"object","properties":{},"required":[],"additionalProperties":false}
```

### 4. `get_odometry` — read-only
**moves_robot:** false **closed_loop:** false

> Report the robot's estimated position (`x_cm`, `y_cm`) and `heading_deg` RELATIVE to an origin you set, from lidar scan-matching odometry. This is drift-slow (~0.15°/s, scan-anchored) and is the only position estimate worth trusting — but it is RELATIVE and accumulates small error over many moves, so treat it as "roughly how far and which way I've moved since I reset," NEVER as an absolute compass or map coordinate. Use it to track exploration progress and to find your way back toward where you started. Optionally pass `reset_origin: true` once at the start of a task to zero position and heading at the current spot. Returns an error if the scan-matching odometry is not running (the same source `turn` needs). Does not move the robot.

```json
{"type":"object","properties":{"reset_origin":{"type":"boolean","description":"If true, zero x/y/heading at the current spot before reading. Do this once at the start of a navigation task."}},"required":[],"additionalProperties":false}
```

### 5. `detect_apriltags` — read-only
**moves_robot:** false **closed_loop:** false

> Look for AprilTag fiducial markers in the current camera view and return each visible tag's `id` and 6-DoF pose (position x/y/z in metres and orientation, relative to the camera). Tags give exact metric ground truth — use them for precise docking/alignment or to verify a spatial estimate from the image. Returns an empty list if no tags are visible or if the camera has no frame. Does not move the robot.

```json
{"type":"object","properties":{},"required":[],"additionalProperties":false}
```

### 6. `walk` — motion
**moves_robot:** true **closed_loop:** false (measures and reports achieved distance; the agent closes the loop)

> Walk a short, bounded distance in one of four body-relative directions, then stop. This is the primary way to translate the robot. `direction` is 'forward', 'backward', 'left', or 'right' — left/right are lateral SIDE-STEPS, not turns (use `turn` to change heading). `distance_cm` is how far to travel; it is CLAMPED to the safe per-call maximum (about 30 cm forward; backward and lateral are capped SHORTER, ~15 cm, because the camera faces forward and cannot see behind or to the sides — check `back`/`left`/`right` clearance with `scan_surroundings` first). To go farther, call again — this per-call cap is intentional so you re-check the scene between moves. The robot walks at a fixed safe speed you cannot set. SAFETY: the move is REFUSED with no motion if the battery is below 11100 mV, the deadman is not ALIVE, an emergency stop is latched, or the lidar shows an obstacle closer than your distance plus a margin in the travel direction. On success it measures how far it actually moved (lidar wall-fit) and returns `requested_cm`, `achieved_cm`, and `achieved_valid` — expect slip, especially on carpet, and when `achieved_valid` is false (no surface to range against) treat `achieved_cm` as a rough estimate. Read `achieved_cm` and issue a correction if you need precision.

```json
{"type":"object","properties":{"direction":{"type":"string","enum":["forward","backward","left","right"],"description":"body-relative travel direction; left/right are lateral side-steps, not turns"},"distance_cm":{"type":"number","minimum":1,"maximum":30,"description":"distance to travel in cm; clamped to the safe per-call maximum (~30 forward, ~15 backward/lateral)"}},"required":["direction","distance_cm"],"additionalProperties":false}
```

### 7. `turn` — motion
**moves_robot:** true **closed_loop:** true

> Turn the robot in place to change heading, using closed-loop feedback. `direction` is 'left' (counter-clockwise) or 'right' (clockwise); `angle_deg` is a POSITIVE magnitude, 5–90° per call — chain calls for larger turns. WHY CLOSED-LOOP: open-loop this robot OVER-rotates by ~22% and its IMU heading is too drifty to trust, so this tool measures real rotation with lidar scan-matching odometry and stops when the measured angle reaches your target — do NOT pre-compensate. It returns `requested_deg`, `achieved_deg`, and `residual_deg` (usually within a few degrees; re-issue the residual if you need precision). Turns are always RELATIVE to the current facing — there is no compass, so never express a turn as an absolute heading or "face the door." SAFETY: REFUSED with no motion if the battery is below 11100 mV, the deadman is not ALIVE, an emergency stop is latched, OR the scan-matching odometry is unavailable — in that last case it will NOT fall back to an inaccurate open-loop turn; the odometry must be fixed first. Turning scrubs the feet sideways and is less accurate on carpet.

```json
{"type":"object","properties":{"direction":{"type":"string","enum":["left","right"],"description":"left = counter-clockwise, right = clockwise"},"angle_deg":{"type":"number","minimum":5,"maximum":90,"description":"positive magnitude in degrees, 5-90 per call; the tool measures and corrects the real rotation"}},"required":["direction","angle_deg"],"additionalProperties":false}
```

### 8. `set_body_pose` — motion
**moves_robot:** true **closed_loop:** false (reports achieved height; does not iterate)

> Adjust the robot's standing posture WITHOUT walking — the feet stay planted, so this is the safest motion. Use it to aim the head camera up/down and to change ride height. Both parameters are optional; an omitted axis holds its current value. `pitch_deg`: nose/camera UP (positive) or DOWN (negative), clamped to ±11°. `height_cm`: body ride height, ~8 (lowest crouch) to ~15 (tallest stand), default ~13 (normal) — lower to peek under something or for stability, higher to see over a low edge. Roll and yaw are not adjustable here (the body is held level in those axes). SAFETY: refused if the battery is below 11100 mV, the deadman is not ALIVE, or an emergency stop is latched. After moving it reads the real leg kinematics and returns `achieved_height_cm` so you can confirm the pose took effect.

```json
{"type":"object","properties":{"pitch_deg":{"type":"number","minimum":-11,"maximum":11,"description":"nose/camera up(+)/down(-); omit to hold current"},"height_cm":{"type":"number","minimum":8,"maximum":15,"description":"ride height in cm: ~8 crouch, ~13 normal, ~15 tall; omit to hold current"}},"required":[],"additionalProperties":false}
```

### 9. `reset_stance` — motion
**moves_robot:** true **closed_loop:** false

> Return the robot to its neutral, level four-legged standing pose (the vendor 'home' reset). Use it to recover a known-good stance after any posture change or gesture, or as a safe default when unsure of the current stance. Takes no parameters and does not translate the robot or change which way it faces. SAFETY: refused if the battery is below 11100 mV, the deadman is not ALIVE, or an emergency stop is latched.

```json
{"type":"object","properties":{},"required":[],"additionalProperties":false}
```

### 10. `perform_action` — motion
**moves_robot:** true **closed_loop:** false

> Run one of the robot's built-in, pre-recorded IN-PLACE routines by name, then settle. These are safe, low-displacement postures and gestures for expression and acknowledgement — NOT navigation (use `walk`/`turn` to move). Postures: `stand`, `sit`, `lie_down`, `up`, `stretch`. Gestures: `nod`, `shake_head`, `wave`, `shake_hands`, `shake_hands1`. Each runs to completion and can only be interrupted with `stop`. SAFETY: refused if the battery is below 11100 mV, the deadman is not ALIVE, or an emergency stop is latched. (Dynamic/vigorous routines — stairs, kicks, boxing, two-leg stand, spacewalk — are deliberately not available because they can tip or collide the robot with no closed-loop check.)

```json
{"type":"object","properties":{"action_name":{"type":"string","enum":["stand","sit","lie_down","up","stretch","nod","shake_head","wave","shake_hands","shake_hands1"],"description":"which in-place posture or gesture to run"}},"required":["action_name"],"additionalProperties":false}
```

### 11. `signal` — zero-motion output
**moves_robot:** false **closed_loop:** false

> Communicate with nearby people WITHOUT moving the body: show a short line of text on the robot's OLED display and/or emit a brief buzzer beep. Use it to acknowledge an instruction, announce what you are about to do, or get someone's attention. Both fields optional; never required to complete a task.

```json
{"type":"object","properties":{"oled_text":{"type":"string","description":"short text for the OLED, ~<=20 characters per line; optional"},"beep_ms":{"type":"integer","minimum":0,"maximum":500,"description":"buzzer beep length in ms (0 = silent, max 500); optional"}},"required":[],"additionalProperties":false}
```

### 12. `stop` — abort
**moves_robot:** true (to a halt) **closed_loop:** false

> IMMEDIATELY halt all motion and hold position. Always available — it works during any other motion and even if the battery is low or an emergency stop is engaged, and it never depends on another tool succeeding first. Call it the instant anything looks unsafe: an unexpected obstacle, a person too close, the robot tilting or moving the wrong way. This is NON-latching: it stops velocity and any running action but does not lock you out, so you can issue new commands once it is safe. (A separate HARD emergency stop is held by the human operator and the safety system; if it engages, every motion tool is refused until a human re-arms the robot — you cannot engage or clear that latch yourself.)

```json
{"type":"object","properties":{},"required":[],"additionalProperties":false}
```

---

## PER-TOOL EDGE-CASE CHECKLIST (for TDD)

Conventions used below: "refuse" = returns `ToolResult.failure(<reason>)` and **publishes nothing to the robot**; "never raises" = `execute()` returns a failure result, the process does not throw. Numeric schema bounds are **advisory only** — `_validate_args` checks required/unexpected keys, NOT ranges — so every numeric clamp must be enforced in the handler and proven by test.

### Cross-cutting (assert for EVERY motion tool: walk, turn, set_body_pose, reset_stance, perform_action; and `stop` where noted)
- Battery = 11099 mV → refuses with the exact floor message; **11100 mV exactly → permitted** (gate is `< min`, assert the off-by-one boundary).
- Battery telemetry not yet arrived (subscription < 3 s old, §G) → treat as "unknown", refuse motion with a "battery unreadable" reason, do NOT assume OK.
- Deadman = PAUSED → refuses; deadman = ALIVE required to proceed; deadman state absent/stale → refuse.
- SafetyGuard `is_estopped` → `execute()` short-circuits before the handler; assert the tool is refused and **no handler side effect** occurs (except tools in `_always_allowed_tools`).
- Watchdog `expired()` at call time → adapter stops the robot and refuses new motion (unsupervised).
- RateLimiter `allow()` returns False (>10 Hz) → call is deferred/dropped without publishing a second command; assert no command storm reaches `velocity_move`.
- Handler raises internally (roslibpy publish error, service timeout, malformed telemetry) → `execute()` catches it and returns failure; **never propagates** (would kill the loop → heartbeat stop → deadman pause).
- Unknown tool name → `ToolResult.failure("unknown tool ...")`.
- Missing required param → failure naming the param; unexpected param → failure naming the param (both from `_validate_args`, before the handler).
- Wrong-type arg that passes key-validation (e.g. `distance_cm: "far"`, `angle_deg: null`) reaches the handler → handler coerces or fails gracefully; assert it returns failure and **never raises** and never publishes a NaN/None to the SI clamp.
- SI conversion is applied BEFORE the SafetyGuard (cm→m /100, deg→rad ·π/180) and the guard clamps in SI — assert the guard sees SI, and that a value the model schema "allows" but that exceeds an SI limit is still clamped (schema is not the guarantee).
- Every failure `reason` string is model-readable and states which condition fired (battery/deadman/estop/clearance/odometry/unmeasured), so the model can re-plan.

### 1. `get_robot_status`
- Never moves the robot under any input; callable while e-stopped and on low battery (read-only, should be in `_always_allowed_tools`).
- Battery classification boundaries: 10499→'critical', 10500→'warn', 11099→'warn', 11100→'ok' (assert each edge).
- IMU roll/pitch: the documented axis swap (§E: command roll appears on IMU pitch) is applied so reported `roll_deg`/`pitch_deg` correspond to the physical body axes; assert a known tilt maps to the correct field.
- Yaw/heading is **never** present in the payload (assert key absent), even though `/imu` carries a yaw value.
- `foot_height_cm` comes from `get_leg_position` (real), never from `/joint_states` or TF (fake zeros); if `get_leg_position` service fails, field is reported as `null`/"unavailable", the whole call still succeeds.
- `max_servo_temp_c` is the max over 12 servos from `bus_servo/get_state`; assert the request **never sets `get_voltage=1`** (vendor bug crashes the whole call); if temperature readback fails, field degrades to null without failing the call.
- Battery topic publisher healthy but no message in the window → `battery_mv` reported as unknown, not 0.
- Deadman status read from the latched `/physicalagent/deadman_status` topic, not from `pgrep` (§I false-positive gotcha).

### 2. `capture_image`
- Camera topic advertised but **no publisher** (the open CSI fault, §C/K) → returns failure within the 8 s `frame_timeout_s`, **never hangs** the loop; assert elapsed < timeout+ε.
- Snapshot HTTP returns 0 bytes / HTTP 200 with empty body → treated as failure, not a valid frame.
- Frame returned → decoded as **rgb8** and the RGB/BGR order is pinned so "blue" stays blue (§K: vendor nodes do RGB2BGR; a swap makes the model call red↔blue). Assert against a known-colour fixture.
- Image dimensions asserted 640×360 (not 480, §K correction).
- Never moves the robot; callable while e-stopped/low-battery.
- Concurrent calls / lazy image_transport first-frame lag → still bounded by the timeout.

### 3. `scan_surroundings`
- Reads `/scan_raw` (full 360, ~96% valid), **not** `/scan` (rear blanked [2.0,4.28] rad); assert the source topic and that rear sectors are populated.
- Self-return (~8 cm, the chassis) is filtered out; assert no sector reports the body as an obstacle and nothing < ~12 cm survives.
- All 8 named sectors present even when some are empty; an empty/all-invalid sector → that sector reports `null`/"clear" (max range), not NaN, not a crash.
- `nearest` = the true minimum over valid returns with its sector label; ties resolved deterministically.
- No `/scan_raw` messages in the window (lidar node down) → failure with a clear reason, not an all-zeros scan.
- cm conversion from metres correct (assert 0.45 m → 45 cm).
- Never moves the robot.

### 4. `get_odometry`
- rf2o `/odom_raw` not running (not in boot stack, §H/J) → failure "odometry unavailable", never a fabricated pose.
- `reset_origin: true` → subsequent reads are relative to the reset point (x≈0,y≈0,heading≈0 immediately after); assert the offset is applied.
- `reset_origin` omitted/false → origin unchanged across calls.
- Values are RELATIVE; assert no claim of absolute/map frame; heading returned in degrees, wrapped to (−180,180].
- Drift is not corrected here (documented ~0.15°/s); assert the tool does not silently "snap" or fuse IMU yaw (IMU yaw unusable).
- Never moves the robot; wrong-type `reset_origin` (non-bool) → failure, never raises.

### 5. `detect_apriltags`
- No tags visible → returns an **empty list**, `ok: true` (absence is not an error).
- Camera has no frame (open fault) → empty list or clear failure, never hangs (shares the camera timeout).
- Enabling `/apriltag_detect` via `set_running` must NOT publish `/cmd_vel` (assert no motion side effect — this is why it's chosen over object_tracking).
- Tag pose returned in the camera frame with documented axes; assert `id` and 6 pose numbers per tag, no partial/garbage entries.
- Multiple tags → all returned; duplicate IDs handled.
- Never moves the robot.

### 6. `walk`
- `distance_cm` beyond schema max (e.g. 200) → NOT rejected by validation; handler **clamps** to the reachable per-call cap and returns `requested_cm=200, achieved_cm≈cap` so intent-masking is visible (§ clamp-not-reject risk).
- Per-direction cap enforced: forward ≤ ~30, backward/left/right ≤ ~15; assert a backward request of 25 clamps to 15 and reports both.
- `distance_cm` ≤ 0 or negative → clamped to 0 → **no motion**, returns achieved 0 (safe direction), never a reversed move.
- Locked vs unlocked speed/duration: with the locked cap (0.05 m/s × 1 s), a `walk(forward,30)` achieves only ~5 cm → assert `achieved_cm` reflects reality and the result signals it was capped, so the model calls again.
- Clearance pre-check: obstacle in the travel-direction sector closer than `distance+margin` (from `/scan_raw`) → refuse with the clearance reason, no motion.
- Direction not in enum → validation/handler failure, no motion.
- `direction` present, `distance_cm` missing (or vice-versa) → required-key failure.
- Achieved-distance measurement uses the **yaw-invariant wall-perp line-fit**, never median-range (§G median-range inflated 33%); establish the stationary noise floor first; if no flat surface in-sector → `achieved_valid=false` and `achieved_cm` = commanded estimate (never a crash, never a fabricated precise number).
- Battery permitted at start but sags below floor mid-burst → the burst is duration-bounded and self-terminates; next `walk` refuses (assert the model sees intermittent refusals as low battery, not tool failure).
- Velocity command derived by the adapter and clamped by `clamp_velocity`; model never supplies m/s; assert the published `Velocity.x/y` never exceeds the active limit even for an over-range `distance_cm`.
- Lateral (left/right) drives `Velocity.y` (real on ROSPug, `ALLOW_LATERAL_Y`); assert y is clamped to `max_abs_y` and forward/back drive x only.
- Stop path at end of burst uses the clean stop (`Velocity.stop=true` and/or `/ros_robot_controller/robot/stop`), not just zero velocity; robot ends halted.

### 7. `turn`
- rf2o `/odom_raw` unavailable → **refuse** (never fall back to the ~122% open-loop over-turn); assert no motion and a clear "odometry required" reason.
- Over-turn handling: commanded 60° must converge to ~60° measured (not 73–75°); assert the loop measures on `/odom_raw` and corrects rather than applying a fixed scale.
- Sign/direction: `left` produces CCW (positive) yaw on `/odom_raw`, `right` CW; the sign is pinned against a known motion (§J flip lesson) — assert direction is not inverted.
- `angle_deg` beyond 90 → clamped to 90, returns `requested_deg` vs `achieved_deg`; `angle_deg` < 5 → clamped up to the min or reported as too-small/no-op (define and test), never a runaway tiny command.
- Convergence cap: on high-friction carpet the loop may not reach tolerance → the tool caps iterations/time and returns `residual_deg` (the un-achieved remainder), never spins indefinitely or drains the battery unbounded.
- Timeout / odometry stops mid-turn → robot is halted and the partial `achieved_deg` reported, never left moving.
- Target already reached (residual within tolerance at entry) → returns immediately with ~0 motion.
- Missing `direction` or `angle_deg` → required-key failure; direction not in enum → failure.
- Feet-scrub battery cost: repeated correction still bounded by the iteration/time cap (assert bounded burst count).

### 8. `set_body_pose`
- Both params omitted → holds current pose (no-op that still returns current `achieved_height_cm`), does not reset to default.
- `pitch_deg` beyond ±11 → clamped (SI ±0.2 rad) and reported; `height_cm` beyond 8–15 → clamped to config `HEIGHT_MIN_M=-0.15 … HEIGHT_MAX_M=-0.08` and reported.
- Height convention correctness: `height_cm=15` maps to `height_m≈-0.15` (tall), `height_cm=8` → `-0.08` (crouch); assert the mapping is not inverted (this is the resolved cross-panel bug).
- Roll and yaw are never sent by the model and are forced to 0 by `clamp_pose`; assert the published `Pose.yaw==0` and roll is at the safe default regardless of any injected arg.
- The verified SDK roll/pitch transposition is **left intact** (do NOT "fix" it, §E/G); assert the adapter does not swap the command-side packing.
- `achieved_height_cm` read from `get_leg_position` (real), converted back to positive ride-cm; if that service fails, pose still commands but `achieved_height_cm` is null with a note.
- Non-numeric pitch/height passing key-validation → handler failure, never raises, never publishes NaN.
- Feet stay planted (zero translation) — assert no `Velocity` message is published by this tool.

### 9. `reset_stance`
- Maps to `/pug_control/go_home` (std_srvs/Empty), the verified-safe init (§E); assert the correct service is called.
- Refused under battery/deadman/e-stop like any motion tool (it drives all 12 servos).
- Service call fails/times out → returns failure, never raises; robot left in prior stance.
- Idempotent: calling twice in a row is safe and does not translate the robot or change heading.
- No parameters accepted → any arg → unexpected-param failure.

### 10. `perform_action`
- `action_name` not in the exposed enum → refused (also double-guarded by `check_action_allowed` against the whitelist); assert scratch files (`111`,`123`,`temp`,`test`,`0_050`) and excluded dynamic actions (`boxing`,`2_legs_stand`,`kick_ball_*`,`up_stair_*`,`spacewalk`,etc.) are **rejected** even if named.
- Name passed to the service WITHOUT `.d6a` extension (vendor appends it, config.py); assert the wire value is e.g. `"wave"` not `"wave.d6a"`.
- Refused under battery/deadman/e-stop.
- Action runs to completion (blocking service) without stalling the heartbeat loop — assert it is threaded/async so `execute()` returns and the orchestrator keeps kicking the deadman.
- `stop` can interrupt a running action (routes to `/ros_robot_controller/robot/stop`); assert an in-progress action halts.
- Missing `action_name` → required-key failure.

### 11. `signal`
- Both fields omitted → no-op success (nothing displayed/beeped), never a failure.
- `beep_ms` beyond 500 → clamped to 500; negative → clamped to 0 (silent); assert `/set_buzzer` never gets an out-of-range value.
- `oled_text` longer than the line width → truncated/wrapped, never overflows or crashes `/set_oled`; empty string is valid (clears/no text).
- Non-string `oled_text` / non-int `beep_ms` passing key-validation → handler coerces or fails gracefully, never raises.
- Never moves the robot; callable while e-stopped and on low battery (zero-motion output — safe to allow always).
- OLED/buzzer topic publish failure → returns failure, does not affect motion state.

### 12. `stop`
- Always allowed: works while e-stopped, on low battery, deadman PAUSED (must be in `_always_allowed_tools` and routed through `stop_all()`, OUTSIDE `execute()`'s guard — assert it is NOT blocked by the e-stop short-circuit).
- Publishes BOTH the velocity stop (`Velocity.stop=true`) and `/ros_robot_controller/robot/stop`; assert both are sent so an in-flight burst and a running action both halt.
- **Non-latching**: after `stop`, a subsequent valid motion call is accepted (assert it does not set the SafetyGuard latch); contrast with the operator/watchdog e-stop which does latch.
- Interrupts a running `perform_action` and an in-progress `walk`/`turn` burst.
- Idempotent: repeated `stop` calls are safe and keep the robot halted.
- Publish failure (link down) → returns failure but the robot-side deadman remains the backstop (assert `stop` failing does not mask that the deadman will still pause on heartbeat loss).
- Takes no params → any arg → unexpected-param failure, but the halt intent should still be honored defensively (decide + test: prefer halting even on a malformed stop call).