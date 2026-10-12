# ROSPug field run — 2026-07-21 (session 4: limits opened up, field-tested)

This is the record of session 4 (2026-07-21). Everything below is either a verified fact or
explicitly flagged as unverified.

---

## 1. State at the end of session 4

**Gemini has driven this robot for real, repeatedly, in an interactive console.
Perception, turning, walking, posture and memory all work live. Session 4 removed the
early-caution limits deliberately, once the field run showed they prevented no harm, and fixed
what the field run exposed.**

Suite: `.venv/bin/python -m pytest -q` → **244 passed**, all offline.
Launch: `./robot` (console) · `./robot doctor` · `./robot run "<task>"`

### Proven ON HARDWARE (not just tested)
- Gemini choosing its own tools, fusing lidar + camera, and reporting honestly.
- Closed-loop turn at −89.9° on a 90° request (early session, when rf2o was healthy).
- Walking under AI control, navigating toward a named object (distance not recorded; the only measured walk is §1b, 28.9 cm at 0.10 m/s).
- **Memory across tasks** — "what did I ask you before?" answered correctly.
- **Battery sag abort firing in the field**: `battery sagged to 11053 mV mid-move`.
- Instant `stop` halting the robot without consulting the model.

### THE BIG REVERSAL OF SESSION 4 — read before touching turns
`turn` is **open-loop** now (`open_loop_turn=True`, default). rf2o odometry
**under-reports rotation**, so closing the loop on it made the robot *keep turning* —
the operator watched it "spin and spin" past the target — and made it *refuse valid
turns* with "odometry stalled mid-turn" while it was rotating perfectly well.
The physical behaviour was confirmed by eye: **it spun far past what was asked**, so the
GAIT is accurate and only the SENSOR is wrong. Turns now drive a duration calibrated
for the measured ~122% over-turn and need no odometry at all. Odometry survives only
as a gross-overshoot monitor — safe in that role *because* it under-reports.
Closed-loop is still available via `open_loop_turn=False`.

⚠️ **Do not "fix" turns by reinstating the odometry loop.** That is the bug.

### Limits after session 4 opened them up
| Limit | Now | Note |
|---|---|---|
| motion budget | **removed** (`max_moves=None`) | prevented no harm |
| walk distance | 200 cm cap (~150 cm/call reachable) | was 30 cm |
| walk duration | 15 s | was 3 s |
| forward speed | **0.10 m/s — UNCHANGED** | this IS the vendor ceiling; "faster" is only reachable as "longer" |
| yaw / lateral | 0.4 rad/s · 0.1 m/s | vendor cmd_vel values |
| hop cap | 60 | was 12 |
| action groups | **all 28** | was a curated 10 |
| pose pitch/roll | ~46° | was 11.5°; vendor clamps nothing |
| pose height | −0.22 .. −0.04 m | was −0.15 .. −0.08 |
| pose roll + x_shift | **exposed** | roll had been suppressed |

**The one hard ceiling that must never move:** `VENDOR_MAX_*` in config.py. Invariant
tests in `tests/test_safety.py::TestVendorCeilingsAreNeverExceeded` enforce it.
Exceeding it damages servos rather than being merely bold.

### Bugs fixed in session 4 (all were live failures)
1. **`do_action` was dead in production.** Two gates in series; the adapter's list was
   opened to all 28 actions while `cli.py` built `SafetyGuard()` with no whitelist,
   defaulting to `()` → refused *everything*, including `sit`. Now one `build_guard()`
   used by every entry point, with an end-to-end test.
2. **Turn could spin ~550° blind.** Its timeout was `max_duration_s + 4`; raising the
   *walk* cap 8→20 silently raised the turn timeout 12s→24s. Budget now derives from
   the angle. Also aborts when rotation heads the wrong way (live: commanded +45,
   odometry reported −27.8, controller kept driving).
3. **`measured_cm` was a lie.** See §M of hardware_notes. Replaced by `expected_cm`,
   a labelled `odom_estimate_cm`, and `front_clearance_m` as the real progress signal.
4. **No memory** — Brain opened a fresh conversation per task.
5. **Transient API errors destroyed tasks** — now retried with backoff.
6. **Console**: no line editing; Ctrl-C killed the session.

### Still open / not done
- **Per-joint servo control** — wanted; NOT built. Requires reading the
  vendor source for real joint limits first. Inventing bounds risks stripping a servo.
- **Why rf2o degrades mid-session** — unexplained. Jetson load average was 11 during the
  run, so CPU starvation of the scan matcher is an untested hypothesis.
- **Deadman flapped to PAUSED** for a single call, then back to ALIVE. Cause unknown.
- **Latency** — settle cut 1.0s→0.3s and per-step timing now printed (`elapsed_s`,
  `think_s`). The next run should show where the dead air actually is, rather than guessing.
- A rigorous straight-on **wall calibration** for translation distance.
- `tonypi.py` adapter (no hardware).

### Open items, in order
1. Robot will have rebooted → **relaunch rf2o and the deadman** (§G of hardware_notes);
   `./robot doctor` shows them failing if not.
2. Best first tests: `turn 90 degrees left` (does it land near 90 without spinning?) and
   `sit back on your rear legs` (exercises the whitelist fix + new pose range; has
   never once worked).

---

## 1b. Hardware status as of 2026-07-20 session 3

- **✅ WALKING WORKS.** At the vendor max 0.10 m/s the robot travelled **28.9 cm against a
  30 cm prediction (96%)**, noise floor 0.0 cm. See hardware_notes §G.
- **✅ Front-Right leg repair VALIDATED.** Static pose sequence (roll ±0.12, pitch, height)
  produced matching IMU responses, symmetric between the repaired and good sides, with
  **0.28° roll / 0.41° pitch total drift** across the whole sequence. A bad rod length shows
  hysteresis under repeated loading; this did not.
- **✅ The old "walking does not translate" finding is VOID.** It was entirely an artifact of
  the failing push-rod that was coming loose *during* those tests. Gait params, command
  paths and carpet friction were never the problem — do not go back and debug them.
- **Deadman deployed and PROVEN** (halts an actively-gaiting robot in 1.40 s, auto-resumes).
  ⚠️ It does **not** auto-start on boot — relaunch after every robot reboot (command in §G).
- **⚠️ IMU yaw is unusable** — drifts ~1.4°/s even while stationary. Roll/pitch are fine
  (gravity reference); yaw is not. Any heading control must use lidar scan-matching.
- **⚠️ Lidar odometry must be yaw-invariant** — use the wall line-fit perpendicular distance,
  never median forward range (which inflates travel via heading change). §G explains.
- **✅ TURNING, LATERAL, SUSTAINED WALK all confirmed** (session 3b). Turn ~45° of a commanded
  60° (75%) — carpet scrub; lateral (y) works, confirming `ALLOW_LATERAL_Y`; an 8 s continuous
  walk held with no fault.
- **✅ WIRELESS WORKS — 2.4 GHz phone hotspot.** Robot + PC both join it ⇒ wireless control AND
  PC internet at once. Robot at −55 dBm / 144 Mbit/s. **0% packet loss, telemetry rates identical
  to ethernet.** See §H for the `hw_wifi.yaml` STA recipe (it auto-falls-back to AP on failure).
- **❌ An 802.1X (EAP) WLAN is impossible** — robot radio locked to CN regdomain (their 5 GHz
  is out-of-band or DFS-hidden), signal −86 dBm, and `hw_wifi.py` has **no 802.1X/EAP support**.
  Credentials cannot fix any of it. §H has all four reasons.
- **🔑 ARCHITECTURAL: use TOPICS, not services, in the control loop.** A rosbridge service call
  costs **~95 ms even on a 2 ms network** (rosbridge overhead, not the link). WiFi adds only
  ~14 ms. Topics stream losslessly over WiFi. §H.
- **🔍 `/scan` is FILTERED** — the rear ~130° is blanked by a `laser_filters` chain. Use
  **`/scan_raw`** for perception: 432 valid returns vs 269 (60% more data). §H.
- **🗺️ Full navigation stack EXISTS but is not running**: `move_base` + `/move_base_simple/goal`
  for autonomous goal-seeking, `rf2o_laser_odometry` → `/odom_raw` (⭐ real scan-matching
  odometry — use this instead of hand-rolled wall fitting), EKF `/odom`, AMCL, gmapping. §H.
- **👁️ Vision pipelines already available** on MJPEG port 8080: AprilTag, face, hand-gesture,
  object-tracking. ⭐ AprilTag gives exact 6-DoF ground truth for checking Gemini's claims. §H.
- **⚠️ `lidar_app` publishes to `/cmd_vel` at boot** — contention hazard. Keep using
  `/pug_control/velocity_move`, which it does not touch. §H.
- **🔴 BATTERY 10.97 V — BELOW the 11.10 V floor. CHARGE BEFORE ANY MOTION.** Session drained
  12.05 → 10.97 V; the gate correctly refused the last test. Budget ~100 mV per motion test.
- **Still untested:** anything on a hard floor (all results are carpet); the deadman over a real
  wireless drop (proven over cable only); navigation stack; vision pipelines.

## 1c. Exact state at the end of session 3 (2026-07-20)

- **Robot is powered down / charging.** Both network paths were down at sign-off (expected).
- **⚠️ The robot is still configured for STA mode pointing at SSID `<PHONE_HOTSPOT>`.** On next boot,
  if that hotspot is not up, it will try for ~45 s, fail, and **auto-fall-back to its own AP**
  (`HW-XXXXXXXX` (the serial-derived SSID printed on the robot), password `<AP_PASSWORD>` (HiWonder factory default), robot at `192.168.149.1`). This is self-healing — just be
  aware of the 45 s delay before WiFi appears. To force factory AP immediately:
  `cp ~/hiwonder_wifi/config/hw_wifi.yaml.bak-physicalagent ~/hiwonder_wifi/config/hw_wifi.yaml && sudo systemctl restart hw_wifi.service`
- **Ethernet always works as the lifeline**: robot `<ROBOT_IP>` (DHCP lease), PC `10.42.0.1` (NetworkManager shared-connection default).
  ⚠️ The PC's ethernet must be on the **`robot-wired`** NM profile, not `g1-wired`:
  `nmcli con up robot-wired ifname <dev>`
- **SSH is now key-based** (no password): `ssh hiwonder@<ROBOT_IP>`.
- **The deadman must be relaunched after every robot boot** — it does not auto-start. See §G.
- **PC was returned to the site network** at sign-off (the phone hotspot costs cellular data).
  To go wireless again: bring the hotspot up, then `nmcli dev wifi connect "<PHONE_HOTSPOT>" password <psk>`.
- Working tree clean; test suite 59 passed.

### Open items at the end of session 3
1. **Confirm the battery is ≥ 11.10 V** before anything (`/ros_robot_controller/battery`, and note
   telemetry needs a >3 s subscription window before the first message arrives).
2. Relaunch the deadman.
3. Highest-value untested items, in order:
   - **Deadman over a real wireless drop** — it has only ever been proven over cable, and that is
     the exact scenario it exists for.
   - **Motion on a hard floor** — every result so far is carpet-damped (turn was only 75%).
   - **`rf2o_laser_odometry`** — launch it for real scan-matching odometry instead of the
     hand-rolled wall fitting (`/odom_raw`, and the EKF `/odom`).
   - The 4 capability-survey domains that never completed: camera, control-surface,
     examples-demos, hazards.

## 2. Situation

- **ROSPug is BACK** (HiWonder finished setup, 2026-07-20). TonyPi not yet brought up.
- **Reconnect is flaky right now due to POWER, not networking.** On 2026-07-20 the Pug
  answered at `rospug.local` (~2 ms), then the link died mid-SSH — a battery brown-out on a
  nearly-empty pack under boot load. **The pack over-discharged; the charger initially sat
  green (refused), then accepted a recovery charge (red light).** Do NOT boot it for real
  work until it charges to ≥11.1 V (ideally 12.6 V / green). `config.BATTERY_MOTION_MIN_MV`
  already gates motion on this.
- **No `GEMINI_API_KEY` yet.** Only `.env.example` exists. Copy it to `.env` and add a key
  from https://aistudio.google.com/apikey. Nothing that calls Gemini can run until then.
- The safety layer was built during this power-wait — it needs neither the robot nor a key.

## 3. The two things that matter most

### 3a. The quadruped is a **ROSPug**, not a PuppyPi
Jetson Nano, 12× HTS-30HS serial-bus servos, Oradar MS200 lidar, IMX219 CSI camera.
**The pre-bring-up (PuppyPi) assumptions are WRONG for this robot** — its namespace is `pug_control`,
not `puppy_control`. Every quadruped topic name assumed before bring-up would have failed. The real, live-verified API is
in `config.py` and `docs/hardware_notes.md` §C. Treat the git-ignored `reference/PuppyPi-*` vendor clones (not in this repository) as analogy
only, never as truth.

### 3b. ✅ RESOLVED: units are SI — METRES and SECONDS
Settled 2026-07-20 from the vendor source (`grab_everything.sh` ran first thing and secured
`/home/hiwonder/pug/src`). The SDK's own comments: `move(...) # 移动, 单位m` (unit m),
`stop(duration) # 停止, 运行时间s` (seconds), `z_clearance: 步高m` (metres).

**How close this was:** our "safe under either unit" guess was 0.2, but the vendor's own
clamp is **0.1 m/s** — we were 2× the vendor maximum. `MOTION_UNLOCKED=False` is the only
reason nothing was ever sent.

🔴 **AND THE CRITICAL ONE: the vendor does NOT clamp.** `pug_node.move_callback()` and
`pose_callback()` pass values straight through with no limiting; only `/cmd_vel` is clamped.
**On the primary interface our safety layer is the only guard.** See hardware_notes §D.

## 4. Verified facts (hard-won; don't re-derive)

**Connection (proven to work):**
```bash
nmcli connection up robot-wired    # profile exists on this PC; enp132s0, ipv4.method=shared
                                   # PC=10.42.0.1, robot DHCPs. autoconnect=no so it never
                                   # fights the existing g1-wired (Unitree) profile.
```
Robot answers at **`rospug.local`** (mDNS works — don't pin the DHCP IP). PC keeps Wi-Fi
internet, so the Gemini API and the robot work simultaneously.

- **SSH: `hiwonder` / `<FACTORY_DEFAULT>`** ✅ verified (the vendor factory password is deliberately not reproduced here)
- Ubuntu 18.04, kernel `4.9.253-tegra` → **ROS1 Melodic**, `ROS_MASTER_URI=http://localhost:11311`
- Ports: 22 SSH · **4000 NoMachine** (the GUI path; no VNC) · 8080 web_video_server · 9090 rosbridge
- Workspace `/home/hiwonder/pug`; bringup `pug_bringup/launch/app_bringup.launch`

**Landmines we already stepped on — do not rediscover:**
1. **`web_video_server` blocks FOREVER** when a topic is advertised but unpublished. The
   `:8080` index still returns 200 in 10ms so it *looks* healthy. **Always use a timeout**
   (`config.ROSPUG.frame_timeout_s`) or the agent loop deadlocks silently.
2. **`gscam` silently self-exits** — it published ~79s then cleanly quit while the rest of
   ROS ran on. A camera liveness watchdog is mandatory.
3. **`/joint_states` is FAKE — never use it.** `pug_control.launch:7` runs the stock
   `joint_state_publisher` with no `source_list`, so it emits URDF placeholders for RViz.
   It is not wired to the servos. **TF leg transforms are fake for the same reason.**
   Real feedback: `get_leg_position` service (Cartesian feet, metres — tracks commands
   exactly) and `bus_servo/get_state` (position + temperature + id).
   ⚠️ **NEVER set `get_voltage=1`** on that service — vendor bug fails the WHOLE call.
4. **Clock is ~3 years off** (Jetson has no RTC). Never trust robot-side timestamps;
   stamp PC-side. `config.TRUST_ROBOT_TIMESTAMPS = False`.
5. **Don't relaunch `csi_camera.launch`** — it restarts `image_proc` too and the name
   collision kills the running one. Start `gscam` alone if the camera needs reviving.
6. **`Velocity.stop=true`** routes to `stop_pub` in `pug_node` — the clean halt path
   (all-zero velocity + `stop=True`). Used by the deadman and verified working.
7. **Lateral `y` IS implemented here** (unlike PuppyPi): ±0.05 m/s on the app path.
8. **Action-group names take NO extension** — pass `"stand"`, not `"stand.d6a"`; the
   controller appends it. 28 verified names are in `config.ROSPUG.action_groups`.
9. **A servo readback CANNOT validate mechanical geometry** — it and `get_leg_position`
   both compute from assumed kinematics. After any mechanical repair use the physical
   rock/paper test, then a **static pose sequence watching for IMU drift from baseline**
   (the sensitive test — a bad linkage shows hysteresis under repeated loading).
10. **IMU yaw drifts ~1.4°/s and is unusable for heading.** Roll/pitch are trustworthy
    (gravity gives an absolute reference); yaw has no magnetometer to correct it. Never
    conclude "the robot is curving" from IMU yaw without a stationary drift baseline.
11. **Lidar odometry: never use median forward range.** Body-mounted lidar + heading change
    at ~5 m range = large fake travel. Fit a line to the wall and use the perpendicular
    distance (yaw-invariant). Always measure the noise floor with two stationary reads first.
12. **The robot's login shell is zsh** — `ssh host 'source ...setup.bash'` fails. Wrap
    remote ROS commands in `bash -c "..."`.
13. **Ethernet: the Pug needs the `robot-wired` NM profile** (10.42.0.1/24 shared), not
    `g1-wired` (192.168.123.x, the Go2/G1). Wrong profile = silent ping failure.

## 5. Next steps, in order

1. ~~`physicalagent/safety.py` + tests (TDD)~~ ✅ **DONE** — 55 tests green. See §1.
2. **`robots/base.py`** — `RobotAdapter` ABC + `ToolSpec`. ← **next**. Fully buildable now;
   define `get_frame()`, `capabilities()`, `execute()`, `stop_all()`, `telemetry()`. The
   adapter composes the safety pieces: hold a `SafetyGuard`, tick a `RateLimiter`/`Watchdog`,
   gate on `check_battery_ok_for_motion`, and `stop_all()` = send physical stop + `guard.estop()`.
3. **`robots/rospug.py`** — roslibpy adapter against the verified API in `config.py`. Test
   against a fake rosbridge / the captured fixtures (`docs/alternatives.md` §2b).
   `robots/tonypi.py` — JSON-RPC; **not blind**, the full vendor source was held in the git-ignored `reference/TonyPi` (not in this repository).
4. **`brain/er.py`** — ER 1.6 client + `[y,x]` denormalisation + overlay rendering.
   *Buildable now; runnable once the key lands.* A real 640×360 ROSPug frame (not published)
   was used to prove the perception pipeline end-to-end on genuine robot output with no hardware.
5. **Hardware contract tests** (planned; not in this repository) — `@pytest.mark.hardware`, <60s, read-only. Asserts every
   `config.py` assumption against a live robot. Makes the next hardware window
   confirmatory instead of exploratory.
6. **On hardware's return (charged pack!):** `scripts/grab_everything.sh` FIRST → resolve
   units/limits → populate `action_groups` → contract tests → only then `MOTION_UNLOCKED=1`
   and smoke test.

## 6. Map of the repo

| Path | What it is |
|---|---|
| `docs/evidence/hardware-2026-07-21-rospug-field-run.md` | **this file** |
| `physicalagent/config.py` | single source of truth; every constant marked VERIFIED / INFERRED / UNVERIFIED |
| `docs/hardware_notes.md` | §A/§B datasheet+source facts · **§C = live-verified ROSPug truth** |
| `docs/rospug_ros_graph.txt` | raw dump: 105 topics, 158 services, 26 nodes |
| `docs/alternatives.md` | every parked option (Gazebo, fake rosbridge, MCP, networking, camera) + why |
| `docs/design/physicalagent-design.md` | the adopted design |
| a 640×360 ROSPug camera frame | **real** frame off the robot's camera; not published |
| `tests/fixtures/rospug_observed_2026-07-17.json` | real observed messages; check `_fidelity` on each |
| `scripts/grab_everything.sh` | **run first** on next hardware contact |
| `reference/` | vendor source (git-ignored, ~700MB; not in this repository). TonyPi = authoritative. PuppyPi-* = analogy only |

## 7. Decisions recorded in session 4

- Quadruped scope: **parity-first** with TonyPi; lidar/SLAM `navigate_to` deferred to Phase 6.
- Sequencing: **software-first**, hardware bring-up when reachable.
- Topology: **plain Python loop**; MCP wrapper deferred to Phase 6.
- No arm fitted on the quadruped.
- Camera: use **`image_raw`** (skip `image_proc`, it proved fragile).
- Networking: **wired, PC-as-router**. A router/switch is needed for the dual-robot demo.

## 8. Open questions for HiWonder (ask while they hold the robots)

1. **The `pug_control` source or its docs — specifically the UNITS and LIMITS of
   `Velocity.x` / `yaw_rate`.** ← unblocks everything
2. The valid action-group names for `/pug_control/run_action_group`.
3. Is the camera fault known? (`gscam` self-exiting; and whether the CSI camera is meant to
   autostart in `app_bringup.launch` at all.)
4. Is lateral `y` velocity implemented on the ROSPug? (It wasn't on PuppyPi.)
5. Confirm whether `enable_reception` is required for real `/joint_states` values.
