# Hardware day 2 plan — 2026-08-20 — capture what the simulator needs

The second hardware session with the G1. Between hardware sessions, development continues
off-robot against the MuJoCo SONIC sim only — so **every minute of robot time today buys
data that calibrates the sim against reality**. The corpus is complete only
with respect to the questions it will be asked (learned 2026-08-19: 5 GB/h
was recorded and the one field that could adjudicate the thermal alarm was
not). The questions, and what answers them, are below.

Number provenance is tagged inline: **[measured]** = we measured it,
**[assumed]** = nobody has. Derived figures move when their assumption does.

## Budgets (check, don't trust)

- Disk: 84 GB free **[measured 2026-08-20, `df -h "$HOME"`]**. Capture
  costs ~5 GB/h all-streams **[measured 2026-08-19]** → ~16 h of headroom.
  Run `df -h "$HOME"` between sessions anyway; exhaustion silently
  truncates an irreplaceable capture.
- Thermal: raised-arm hold heats motor 22's driver ~215 C/min; from 70 C
  there are ~5.6 s to the 90 C limit **[measured 2026-08-19]**. Read
  `temp_driver`, never `temp_winding` (they differ 20 C+ under load).
- lowstate arrives ~1052 Hz, lowcmd ~995 Hz, odommodestate ~498.6 Hz
  **[measured 2026-08-19]** — not the commonly quoted 500 Hz.

## Setup — same as DEMO-RUNBOOK.md, plus one rule

Full paths on purpose; shell variables do not cross terminals.

```bash
cd "$PA_ROOT"
ip -br addr show enp132s0          # must read UP + 192.168.123.222
"$WBC_PY" scripts/hw_preflight.py
PYTHONPATH=. "$WBC_PY" \
  scripts/g1_sonic_session.py --check
```

**The one rule: recorder FIRST (link up), THEN `deploy.sh`, THEN the
session.** That order is not just the zero-byte-capture guard — it also makes
every SONIC bring-up a free INIT-window capture (P2-A below): the gain
handover from zero-torque idle to SONIC's first lowcmd is exactly the window
MuJoCo's elastic band papers over, and the sim needs its real shape.

```bash
# Terminal 1 — recorder, one per experiment, label = experiment name
"$WBC_PY" scripts/hw_recorder.py \
  --label day1-battery1 --seconds 0
# Terminal 2 — SONIC
cd "$WBC_ROOT"/gear_sonic_deploy && ./deploy.sh --input-type zmq_manager real
# Terminal 3 — the session (per experiment, below)
```

## Priorities

| # | Experiment | Robot time | Why it cannot wait |
|---|---|---|---|
| P0-A | 9 provenance photos | 0 (needs a camera, not the robot) | `photos/` is EMPTY; the checklist in `artifacts/hw/20260819-112932-day0-live-robot-provenance/PHOTOS.md` (offline capture, not in this repository) was never shot |
| P0-B | `sim_match_v1` battery ×2–3 | ~2 min each | THE sim-calibration dataset; first run on a COOL robot |
| P1-A | Thermal-watch live fire at 55 C | ~5 min | The guard has never fired on hardware |
| P1-B | `ankle_asym_v1` battery | ~5 min | The 7× right/left asymmetry is real, repeatable, unexplained |
| P1-C | ER-2 driving the physical robot | 15–30 min | The whole point of the project; capture + verb traces |
| P2-A | INIT-window captures | free | Comes with the startup order rule |
| P2-B | auto-damp → re-arm probe | ~5 min | Sim says recovery is impossible by design; hardware never asked |
| P3 | Ground contact (decided at the session) | 30+ min | foot_force is zero in EVERY capture that exists |

### P0-B — the battery (the centerpiece)

```bash
PYTHONPATH=. "$WBC_PY" \
  scripts/g1_sonic_session.py --real --i-am-at-the-estop \
  --script batteries/sim_match_v1.txt
```

- Recorder running with a matching label (`--label day1-battery1`).
- The session prints the JSONL log path (`artifacts/sessions/<ts>-script.jsonl`).
  **That log is half the dataset** — it is the join key between the verb
  stream and the DDS capture. Note which capture dir pairs with which log.
- Run 1 on a cool robot, run 2 mid-day, run 3 warm/end of day — the same
  trajectory at three thermal states separates gains from temperature.
- Segments map to open questions: `idle-baseline` → balance micro-corrections;
  `arms-*` → upper-body tracking + shoulder thermal; `height-*` → pelvis
  calibration; `walk-*` → speed staircase (still pendulum-confounded while
  hoisted); `turn-*` → ankle loads.

Off-robot, the same file runs against the MuJoCo sim **through the console**
(the session without `--real` is FakeSonicLink — a stub, not the sim; the
console owns the real sim bring-up: start handshake, band release, latch
drain). `sonic_live.sh` forwards its arguments to the console:

```bash
# terminal A — recorder on loopback DDS. This exact path is [assumed]:
#   verify with a 10 s test capture before trusting a long run. The sim
#   publishes rt/odostate, NOT rt/odommodestate, so the odom file being
#   zero bytes in sim captures is expected.
"$WBC_PY" scripts/hw_recorder.py \
  --allow-any-net --label sim-battery1
# terminal B — full sim bring-up, then the battery, no model, no API key:
./scripts/sonic_live.sh --script batteries/sim_match_v1.txt
```

Then compare:

```bash
python3 scripts/sim_match_compare.py \
  --a artifacts/hw/<hw-dir> --a-log <hw-script.jsonl> --a-label hw \
  --b artifacts/hw/<sim-dir> --b-log <sim-script.jsonl> --b-label sim \
  --out artifacts/sim_match/<date>
```

Per-segment, per-joint RMSE/correlation plus overlay plots — the number that
guides everything off-robot.

### P0-C — FIRST (added 12:30 after battery run 1): re-hang so the feet clear the floor

Battery run 1 aborted at t+20 s: the moment SONIC took authority, BOTH
ankle-pitch joints sat 0.86/1.04 rad from their commanded targets at kp 28.5
— a sustained 24–29 N·m pressing the commanded stance into the floor while
the gantry held the body. All four ankle drivers heated together (m11 43→89 C
in 17 s); the coupled two-motor ankle linkage puts that pitch fight through
the "roll" motor's sensor, which is ALSO the mechanism behind yesterday's
"unexplained" asymmetry. The readings are real (voltage dipped at authority,
four sensors in lockstep, winding lagging driver exactly as physics says).

**Fix: raise the hoist until both soles clearly clear the floor.** Then a
30-second verification BEFORE any battery:

```bash
cd "$PA_ROOT" && PYTHONPATH=. \
  "$WBC_PY" \
  scripts/g1_sonic_session.py --real --i-am-at-the-estop --start
g1> joints left_leg     # ankle torque_nm must be ~<3 N·m (was 24–29)
g1> joints right_leg
g1> status              # temps flat over ~30 s
g1> quit
```

If ankle torques are still tens of N·m with feet in the air, STOP — the
model is wrong and we investigate before heating anything.

Honest limitation this creates: off-robot the sim robot STANDS (band-release
lowers it onto the floor); hoisted-feet-free hardware is a different plant
for leg segments. Arm/height segments compare cleanly; leg segments compare
plant-to-plant only if P3 (real ground contact) happens.

### P1-A — ~~fire the thermal watch~~ DONE, live, at the real 90 C

Battery run 1 fired it for real: trip at 90 C, stop + arms-neutral ran, log
and capture prove it. Found a gap the same minute: the ankle load persisted
after intervention (93 C peak AFTER stop), so the watch now ESCALATES to
damp if act persists >10 s after the first intervention (robot hangs on the
gantry — that is what the gantry is for). No need to re-test at 55 C.

```bash
PYTHONPATH=. "$WBC_PY" \
  scripts/g1_sonic_session.py --real --i-am-at-the-estop --start \
  --thermal-act-c 55
```

Then `arms arms_up` and wait. Some motor passes 55 C within a minute of a
raised hold **[measured 2026-08-19: 65→91 C in 6.5 s under way]**; the watch
must print `THERMAL STOP`, stop locomotion, and return the arms to neutral —
on its own. That is the guard's first real firing. The limit can only be
LOWERED (35–90 C); raising is refused in code.

### P1-B — ankle asymmetry

```bash
PYTHONPATH=. "$WBC_PY" \
  scripts/g1_sonic_session.py --real --i-am-at-the-estop \
  --script batteries/ankle_asym_v1.txt
```

While it runs, check what the data cannot see: harness pulling one leg, foot
hanging angle, cable tension on the right ankle. Compare BOTH ankle motors on
BOTH sides afterwards (`temp_driver`; the ankle is a coupled pitch/roll
linkage, so a roll-index torque and a physical motor temperature are in
different frames — don't let one "explain" the other without the pitch data).

### P1-C — ER-2 on the physical robot

```bash
PYTHONPATH=. "$WBC_PY" \
  scripts/g1_sonic_session.py --real --i-am-at-the-estop --start
```

Recorder on (`--label day1-er2`). Suggested ladder, one `task` each, simple →
compound (each ER-2 response prints its tool-call trace — that trace beside
the DDS capture is the dataset "what does the model do with real telemetry"):

1. `task read your status and tell me the hottest motor`
2. `task wave your right hand, then return to neutral`
3. `task walk forward slowly for two seconds, then stop`
4. `task look at your joint readings and tell me if anything seems asymmetric between your left and right legs` ← the model looking at the live ankle data
5. `task turn slowly toward your left, then face forward again`

Voice: add `--voice` (earbuds must be connected; profile switches automatically).

### P2-B — the auto-damp question (end of day, deliberate)

In sim, auto-damp sends `command{start:0, stop:1}` and `start_control()`
does NOT recover — get-up after damp is impossible by design 0/1
**[sim-measured]**. Hardware has never been asked. At end of day, hoisted,
hands off: `estop` (damp), then `quit`, restart the session, `--check`, take
control again. Record whether SONIC re-arms cleanly after its own damp — one
bit that decides whether any in-session fall recovery can ever exist.

### P3 — ground contact: decided at the session, not by the plan

Every byte captured so far has `foot_force == [0,0,0,0]` because the robot
has never borne weight. If (and only if) the setup is judged, at the session, to allow it:
robot lowered until feet take load, **gantry stays attached with slack —
never removed**, e-stop holder present, and re-run `sim_match_v1` with label
`day1-battery-ground`. That single run would be the only contact ground
truth and the only clean locomotion speed measurement in existence. If it
doesn't happen, that's fine — everything else still lands.

## What NOT to spend robot time on

- The 1052 Hz vs 999.94 ticks/s ambiguity — offline analysis of existing captures.
- `jump_forward` reference question — sim work.
- Anything requiring raised limits. Clamps are fixed by the tier
  table; discrepancies are fixed by lowering code, never raising tables.
- Isaac. MuJoCo is the off-robot sim; Isaac's gaps are known and documented.

## End of day

1. `df -h "$HOME"` — confirm nothing truncated.
2. Every capture dir has its paired script-log noted (a one-line README in
   the capture dir is enough).
3. `artifacts/hw` is now the **only copy** of everything today produced, and the
   workstation is not a permanent store: copy the 13 GB (growing today)
   elsewhere before the session ends.
