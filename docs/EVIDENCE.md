# Evidence index

Every claim in [`README.md`](../README.md), and the file in this repository that
carries it. If a row has no file, it says so.

Conventions used throughout: a result is recorded at the **highest rung
demonstrated with evidence**, never the highest attempted; simulation results are
labelled as simulation in the same sentence as the number; and numbers that no
artifact backs are marked as such in the document that states them.

---

## Hardware — Unitree G1 under NVIDIA SONIC

| Claim | Evidence |
|---|---|
| Gemini Robotics-ER 2 planned and issued the tool calls that SONIC executed on a physical G1, 2026-08-20 | `docs/evidence/er2-hardware-toolcalls-2026-08-20.md` (hand transcription — see the file's own caveat) + `docs/evidence/hardware-2026-08-20-er2-driving.md` |
| 7 typed inputs and 18 tool calls in the session, round trips 10.1–31.3 s; **11 of the 18 calls transcribed call-by-call**, the rest marked *not transcribed* | `docs/evidence/er2-hardware-toolcalls-2026-08-20.md` (hand transcription; the ~14 GB DDS capture is offline and not in this repository) |
| `task wave hi` → `set_arms(wave_right)`, `set_arms(neutral)`, 16.9 s | same |
| `task change through the poses you know` → 4 `set_arms` calls, 31.3 s | same |
| A bare `walk foward 1 meter` was refused by the argument-count guard | same |
| **`set_grasp` failed** — "finger grasp control is currently unavailable"; no grasp occurred | same |
| **The robot was harnessed throughout; legs stepped in the air, no forward translation** | same, and the correction notice at the top of `docs/evidence/hardware-2026-08-20-er2-driving.md` |
| 2026-08-19: the hoisted robot stepped under commanded walks; net pelvis displacement 0.19 m across 632 s (419 s + 213 s) of commanded authority, confounded by pendulum motion — not evidenced walking | `docs/evidence/hardware-2026-08-19-g1-sonic.md` |
| Peak kp 99, kd 6.3 across 35 motors; feed-forward torque 0 throughout | same |
| "No clean locomotion speed measurement exists yet" while hoisted | same (stated in the document) |
| Motor 22 driver 65 → 91 → 106 °C in 11.5 s; winding 53 → 56 °C over the same window | same |
| 215.6 °C/min; 5.6 s of margin from 70 °C to the 90 °C limit | same |
| No motor fault word ever non-zero; bus 46.0–46.5 V | same |
| `thermal_watch` runs on its own thread at 0.2 s and trips on max(winding, driver) per motor, so the fast-moving driver sensor is the one that reaches the limit first | `scripts/g1_sonic_session.py` (`thermal_watch`, `thermal_watch_step`, `_thermal_limit`) + `physicalagent/robots/sonic_telemetry.py` (`thermal_verdict`, `max_c`) |
| The guard **fired for real** on 2026-08-20: live at 90 °C at 12:02, aborting the 12:00 battery run 20 s in | `docs/evidence/hardware-2026-08-20-er2-driving.md` (run table; "Thermal watch fires for real — DONE, live at 90 °C, 12:02") |
| Its action (stop + arms neutral) did not remove the leg load that tripped it; an escalation to damp was written and tested the same day | same (HW-DAY2-PLAN outcome: "Watch escalation to damp — DONE ... after the watch's first action failed to remove a leg load") |
| Raised-arm holds capped at 8 s | `batteries/sim_match_v1.txt` |
| Commanded ankle pitch reaches +1.35/+1.38 rad against a +0.5236 rad mechanical limit | `docs/evidence/hardware-2026-08-20-er2-driving.md` |
| 0.3–0.8 rad standoff at kp 28.5 = 10–15 N·m producing no motion | same |
| Per-joint tracking: hips/knees/waist/shoulders 0.002–0.056 rad; ankle pitch +0.339 / +0.513 rad at 9.60 / 14.49 N·m | same |
| `foot_force` captured in the 16:12 run, unanalysed | same |
| No segment-aligned hardware-vs-sim battery capture; `sim_match_compare.py` refuses with exit 2 | same |
| The plotted thermal figure | `docs/media/thermal-near-miss.png`, generated from the four logged readings in `docs/evidence/hardware-2026-08-19-g1-sonic.md`; its t = 0 is the first logged reading (t+1321.4 s), not the start of the hold; the 215.6 °C/min annotation is the sustained-rise figure from the offline capture, not the slope between plotted points |
| The round-trip timeline figure | `docs/media/er2-roundtrips.png`, drawn from the four transcribed round trips (14.3, 16.9, 10.1, 31.3 s) and the refused input in `docs/evidence/er2-hardware-toolcalls-2026-08-20.md`; the refused input is marked refused (no model call) and the two untimed inputs are shown as not transcribed |
| No photograph or video of the physical robot is published | The hero figure `docs/media/hero_sim.png` is composed from simulation frames only: `artifacts/getup/realfall_{0_down,1_end}.png`, `artifacts/lounge_scene/passB/goto_table_pov.jpg`, `artifacts/lounge_scene/passB/goto_table_window.png` and `artifacts/isaac_m1/cam_head_via_sidecar.jpg`. The hardware sessions are evidenced by the write-ups and telemetry in this table |

## Hardware — HiWonder robots

| Claim | Evidence |
|---|---|
| Gemini driving a physical ROSPug end to end: tool choice, lidar+camera fusion, navigation to a named object, memory across tasks, instant stop | `docs/evidence/hardware-2026-07-21-rospug-field-run.md` |
| Battery-sag abort fired in the field at 11 053 mV mid-move | same |
| Closed-loop turn hit −89.9° on a 90° request | same |
| Turns reverted to open loop; ~122 % measured over-turn; odometry kept only as a gross-overshoot monitor | same + `docs/hardware_notes.md` §J + `physicalagent/robots/turn_control.py` |
| TonyPi: the `look()` → describe → `do_action()` chain is the adapter's design; what is evidenced **on hardware** is the camera path below | `physicalagent/robots/tonypi.py` (the chain), `docs/design/physicalagent-design.md` (the design) |
| `?action=snapshot` 10/10 byte-distinct JPEGs vs `?action=stream` 1/10, measured live on the physical robot; the same docstring records that no two frames differing in content have yet been observed from this robot, so TonyPi vision is still unproven — what is evidenced is which endpoint is stale | `physicalagent/robots/tonypi_link.py` (`snapshot()` docstring) and `tools/tonypi_view.py` |
| Frames arrive 640×960 with duplicated halves; `dedupe_frame()` crops | `physicalagent/robots/tonypi.py`, `tests/test_tonypi_dedupe.py` |
| Four TonyPi RPC dead ends, including one retraction | `docs/design/hiwonder-dead-ends.md` (D1–D4) |
| A live BLE capture off the real MiniHexa: `$8127$65535$` on notify char `ffe2` | `physicalagent/robots/minihexa_codec.py` (`parse_ble_battery`, line 141), `tests/test_minihexa_codec.py` (`TestBleTelemetryDecode`) |
| The BLE decoder fails closed; `0xFFFF` must never read as a healthy 65 V pack | `tests/test_minihexa_codec.py` |
| A BLE disconnect does not stop the robot, so the profile is `OPERATOR_BOUNDED` | `physicalagent/profiles/minihexa.py`, `physicalagent/profiles/base.py` |

## Simulation — MuJoCo

| Claim | Evidence |
|---|---|
| Standing defined before the reported runs 2–4 as pelvis ≥ 0.70 m AND only feet in contact AND feet on the floor, sustained 3 s; the foot-contact clause was added after run 1 was voided | `artifacts/getup-2026-08-02.md` (criterion, and the harness-bug note) |
| Kneel ladder: 0.154 → 0.416 → 0.786 m | same + `artifacts/getup/ladder_{0_down,1_end}.png` |
| Crawl route: 0.152 → 0.449 → 0.421 → 0.787 m | same + `artifacts/getup/crawl_route_1_end.png` |
| Genuine induced fall recovered, n=1: 0.225 → 0.151 → 0.431 → 0.410 → 0.787 m | same + `artifacts/getup/realfall_{0_down,1_end}.png` |
| `idle` from prone thrashed to 0.550 m, collapsed to 0.078 m, dragged the robot 1.99 m | same + `artifacts/getup/direct_1_end.png` |
| `jump_forward`: 0 of 438 samples contact-free over 8 s | same + `artifacts/getup/jump_end.png`, `artifacts/getup/run3_getup_log.jsonl` |
| `recover_from_fall`: prone 10/10, shoulder-drop 3/3 of the three falls that could be induced (harness-limited: the other eight trials either failed to drop a shoulder or could not re-engage the band), 0/1 after auto-damp and 0/1 again after a `start_control` re-arm (the single retry in the same trial file) | `docs/evidence/getup-verb-2026-08-06.png` + `artifacts/getup_verb/trials-run1-prone10-realfall2.jsonl`, `trials-run2-realfall.jsonl`, `trials-run3-afterdamp.jsonl` (what every other file in that directory is: `artifacts/getup_verb/README.md`) |
| 23 of 27 modes never previously observed; four defects found | `artifacts/mode-test-2026-08-01.md` + `artifacts/mode*.png`; figure `docs/media/modes-2up.png` |
| `select_mode` sent speed 0.0 when none was named; planner reads 0.0 as stationary, −1.0 as trained speed | same |
| The vendor gamepad cannot produce speed 0 on a standing mode | same (cites the vendor header) |
| `HEIGHT_MIN_M = 0.50` silently raised squat/kneel/crawl; floor now per-kind | same |
| "sneak forward quietly" → `walk(vx=0.3)`; `stealth_walk` was never commanded | same |
| Lounge A/B: body origin is not object position — 0.91 / 0.99 / 1.12 / 1.72 / 1.78 m offsets | `artifacts/lounge_scene/REPORT.md` |
| `goto_table` judged 0.36/0.37 m, measured 1.04/1.01 m; neither robot translated (pass B turned in place, `final_heading_error_deg: -10.8`) | same + `artifacts/lounge_scene/pass{A,B}/battery.jsonl` |
| The pick was a placement failure: object 1.08–1.24 m away, top z = 0.543, reach 0.587 m | same |
| Pass B: 8 of 16 `reach_to` calls refused by the reachability check (pass A: 3 of 5) | same |
| **No grasp**: `hand_contacts` 0, `lift_m` 0.001 and 0.004 | `artifacts/lounge_scene/pass{A,B}/battery.jsonl` |
| The model's false "walked to the white round table" text is kept in the trace | `artifacts/lounge_scene/pass{A,B}/*_reply.txt` |
| The head-camera / third-person contrast figure | `docs/media/lounge-pov-vs-truth.png`, from `artifacts/lounge_scene/passB/goto_table_pov.jpg` (the `look` frame at the end of pass B's `goto_table`) and `passB/goto_table_window.png` |

## Simulation — Isaac Sim

| Claim | Evidence |
|---|---|
| The vendor `dds_wholebody` mode drove legs from its own `policy.onnx`; external `rt/lowcmd` reached only 14 arm joints; `kp`/`kd` parsed and discarded | `third_party/unitree-sim-isaaclab-local/README.md` (patch 05 entry) |
| The vendor mode was believed to be driving the legs from the first Isaac get-up attempt (2026-08-07) until the wiggle test of 2026-08-12; every Isaac result in that window, including the get-up A/B, was voided rather than reported | `artifacts/getup_verb/isaac-prone-20260807.log` (the 08-07 attempt: 0/1) + the patch 05 entry |
| Patch 05 routes all 29 body joints by name, order pinned against `policy_parameters.hpp` | `05-sonic-full-body-routing.patch` |
| BASE_COMMIT + patches 05–07 is rebuilt and compared byte-for-byte with the live vendor provider, where `UNITREE_SIM_ROOT` points at the checkout (3 of the 8 skips in the offline suite) | `tests/test_full_body_patch.py` (`TestLiveVendorTreeMatchesThePatch`) |
| The eight-joint wiggle-test table (arms track in both modes; legs and waist only in full-body, two of them moving against the command) | read directly out of `artifacts/routing/routing-20260818-101450-fullbody.jsonl` and `routing-20260818-101547-vendor.jsonl` |
| 8-run time-scale table: 0.291 @ ~3.4 stayed up; 0.179, 0.296, 0.205, 0.191 collapsed | `artifacts/tracking/README.md` + `artifacts/tracking/*.jsonl` |
| "Stayed up" is not "stood": pelvis std 0.0499 vs the MuJoCo baseline's 0.0002; legs 4.074 vs 0.008 rad/s | `artifacts/tracking/README.md` |
| Run 11:13 came within 5 % of the needed scale and still collapsed (takeover z 0.155 vs 0.752) | same |
| The tracking figure and the scale-vs-needed chart | `docs/media/isaac-tracking-3up.png`, from `artifacts/tracking/stand-*.jpg`; `docs/media/isaac-scale-vs-needed.png`, plotted from the 8-run table in `artifacts/tracking/README.md` with the 10:48 row (SONIC never started, no valid needed scale) excluded |
| DDS domain mismatch: upstream domain 1 on the default interface, stack on domain 0 loopback; ~80 Hz published, zero received | `third_party/unitree-sim-isaaclab-local/README.md`, patch 01 entry |
| IMU quaternion order fixed: `[0.0039, −0.0224, −0.0415, 0.9989]` → `[0.9988, 0.0035, −0.0235, −0.0438]` | `third_party/unitree-sim-isaaclab-local/README.md`, patch 02 entry |
| A separate measurement: nothing talks across the CycloneDDS 0.11.0 / 11.0.1 boundary, and after the pin the two interpreters see each other | `artifacts/isaac_m0/dds-interop-matrix.txt` and `dds-interop-matrix-after-fix.txt` (a 4-row publisher→subscriber matrix; it measures the version split, not the domain fix) |
| Camera frames served through the ZMQ sidecar | `artifacts/isaac_m1/cam_head_via_sidecar.jpg`, `artifacts/isaac_m0/cam_{head,left,right}.jpg`; shown in the README as `docs/media/isaac-sidecar-cam.jpg` |

## The codebase itself — measured on 2026-09-22

| Claim | How it was measured |
|---|---|
| 1,753 tests pass, 8 skip, in seconds (5.3–5.6 s on the machine recorded here; 7–15 s on others), fully offline **with no API key set at all** | `env -u GEMINI_API_KEY python -m pytest -m "not hardware" -q -p no:cacheprovider`, run inside this export on Python 3.12: `1753 passed, 8 skipped in 5.3–5.6 s` across three runs |
| 64 test files, 17,166 lines of test, 1,687 `def test` functions (re-measured 2026-10-12) | `ls tests/*.py \| wc -l`; `find tests -name '*.py' \| xargs wc -l`; `grep -c 'def test' tests/*.py` |
| 11,168 lines in `physicalagent/` (re-measured 2026-10-12) | `find physicalagent -name '*.py' \| xargs wc -l` |
| Hardware tests are deselected by default | `pyproject.toml`, `[tool.pytest.ini_options] addopts` |
| Every transport has a `Fake` variant | `physicalagent/robots/{transport,tonypi_link,minihexa_link,aihand_link,sonic_link}.py` |
| A profile with no supervision story raises at construction | `physicalagent/profiles/base.py`, `tests/test_minihexa_profile.py` |
| `SupervisionClass` and `SafeStatePolicy` enums, as described | `physicalagent/profiles/base.py` (`SupervisionClass`, `SafeStatePolicy`) |
| Three robots registered; the G1 deliberately not | `physicalagent/profiles/__init__.py` (the registry, with the reason in a comment) |
| The AST test walks five operator scripts | `tests/test_g1_sonic_session.py`, `TestEveryGlobalNameTheScriptsUseActuallyExists` |
| 1,554 tests passed when the `NameError` fired on hardware | `docs/evidence/hardware-2026-08-19-g1-sonic.md` (Session 2) |
| A recorder wrote six zero-byte files for 34 minutes; `hw_recorder` now refuses | `tests/test_hw_capture.py` (`TestTheRecorderRefusesToRecordOffTheRobotsNetwork`), `scripts/hw_recorder.py`, `docs/evidence/hardware-2026-08-19-g1-sonic.md` (bug 3) |
| One module holds machine layout; a test greps the tree for home directories | `physicalagent/paths.py`, `tests/test_paths.py::test_no_developer_home_left_anywhere_in_the_tree` |
| The G1 tier table, and the test that pins it in both directions | `docs/design/g1-safety-tiers.md`, `tests/test_g1_locomotion_plane.py` |

---

## Claims this repository does NOT make

Stated explicitly, because absence of a claim is easy to miss:

- **No grasp and no lift** has been achieved here, in simulation or on hardware,
  on any task.
- The unpublished 2026-08-20 session recording does **not** show walking. The robot was harnessed.
- Voice was **never** wired into a hardware session.
- Nothing here trained a policy. SONIC is NVIDIA's, used as shipped.
- No success flag in any config or registry is treated as evidence of anything.
  A flag is asserted, not earned.
