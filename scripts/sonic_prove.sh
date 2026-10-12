#!/usr/bin/env bash
# Prove -- or disprove -- that a typed prompt moves the G1 in MuJoCo.
#
#   ./scripts/sonic_prove.sh ["walk forward about two metres, then stop"]
#
# WHY AN A/B AND NOT JUST A VIDEO
#   The viewer camera defaults to mjCAMERA_TRACKING on the pelvis, so the robot
#   stays centred in frame whether it walks or marches in place -- a recording
#   alone cannot tell the difference. And even a video of the robot moving does
#   not show the MOVEMENT WAS CAUSED BY THE PROMPT; the policy might do that
#   anyway. So this runs two phases against the same policy, same session:
#
#     CONTROL  no commands published at all      -- expect ~no translation
#     GEMINI   Gemini Robotics-ER 2 drives it    -- expect translation
#
#   Ground-truth pelvis position is logged by the simulator itself
#   (SONIC_POSE_LOG, base_sim.py local addition) because nothing on the DDS
#   side carries world position -- rt/lowstate is IMU only.
#
# Camera is switched to mjCAMERA_FREE (key 'v') so the recording actually shows
# ground-plane motion instead of a robot glued to the centre of the frame.
set -uo pipefail

PROMPT="${1:-walk forward for about five seconds, then stop and stand still}"
WBC="${WBC_ROOT:-$HOME/GR00T-WholeBodyControl}"
DEPLOY=$WBC/gear_sonic_deploy
PY=$WBC/.venv_sim/bin/python
YAML=$WBC/gear_sonic/utils/mujoco_sim/wbc_configs/g1_29dof_sonic_model12.yaml
# NOT /tmp: a run's artifacts were wiped out from under it there on 2026-07-31,
# which destroys the evidence the run exists to produce.
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(dirname "$HERE")"
OUT=${OUT:-${PA_ARTIFACTS:-$REPO/artifacts}/sonic_prove}
CONTROL_S=${CONTROL_S:-20}

rm -rf "$OUT"; mkdir -p "$OUT"
unset CYCLONEDDS_URI WAYLAND_DISPLAY || true
export PYGLFW_LIBRARY_VARIANT=x11   # the Wayland libglfw cannot open an X window
export XDG_SESSION_TYPE=x11
export DISPLAY="${DISPLAY:-:0}"
export MUJOCO_GL=glfw
export SONIC_POSE_LOG="$OUT/pose.csv"

grep -q '^ENABLE_ELASTIC_BAND: True' "$YAML" || {
    echo "ERROR: ENABLE_ELASTIC_BAND must be True -- nothing holds the robot"
    echo "       during the ~35 s the policy spends loading TensorRT."; exit 1; }

pkill -f "[r]un_sim_loop" 2>/dev/null; pkill -f "[g]1_deploy_onnx_ref" 2>/dev/null
sleep 2

mark() { echo "$1,$(date +%s.%N)" >> "$OUT/marks.csv"; echo "[mark] $1"; }

echo "[1/6] simulator (windowed, pose logging) -> $OUT/sim.log"
( cd "$WBC" && exec "$PY" gear_sonic/scripts/run_sim_loop.py --enable-onscreen ) \
    > "$OUT/sim.log" 2>&1 &
SIM=$!
sleep 12
kill -0 $SIM 2>/dev/null || { echo "simulator died -- see $OUT/sim.log"; exit 1; }
# the sim suffixes its pid onto SONIC_POSE_LOG so a stray second writer is visible
ls "$SONIC_POSE_LOG".* >/dev/null 2>&1 || {
    echo "no pose log -- the base_sim addition did not run"; kill $SIM; exit 1; }

echo "[2/6] SONIC policy -> $OUT/policy.log   (TensorRT load, ~35 s)"
( cd "$DEPLOY" && exec ./target/release/g1_deploy_onnx_ref lo \
      policy/release/model_decoder.onnx reference/example/ \
      --obs-config   policy/release/observation_config.yaml \
      --encoder-file policy/release/model_encoder.onnx \
      --planner-file planner/target_vel/V2/planner_sonic.onnx \
      --input-type   zmq_manager --output-type all \
      --zmq-host localhost --zmq-port 5556 \
      --disable-crc-check ) > "$OUT/policy.log" 2>&1 &
POL=$!
for i in $(seq 1 30); do
    grep -q "Init Done" "$OUT/policy.log" 2>/dev/null && break
    kill -0 $POL 2>/dev/null || { echo "policy died -- see $OUT/policy.log"; kill $SIM; exit 1; }
    sleep 2
done
grep -q "Init Done" "$OUT/policy.log" || { echo "policy never reached Init Done"; kill $SIM $POL; exit 1; }

echo "[3/6] start + planner mode over ZMQ"
"$PY" "$HERE/sonic_start.py" >> "$OUT/policy_start.log" 2>&1 || {
    echo "start command failed"; kill $SIM $POL; exit 1; }

echo "[4/6] releasing band (9) and switching camera off TRACKING (v)"
# Verified release, not a blind keystroke: XTEST focus on an XWayland surface is
# flaky and a silent failure leaves the robot hanging in mid-air, where every
# command does nothing and the run looks like a dead command path.
( cd "$REPO" && "$PY" -c "
import sys; sys.path.insert(0, 'scripts')
from sonic_console import release_band
raise SystemExit(0 if release_band() else 1)
" ) 2>&1 | tee "$OUT/keys.log"
[ "${PIPESTATUS[0]}" -eq 0 ] || { echo "band never released -- aborting, the run would be meaningless"; kill $SIM $POL; exit 1; }
"$PY" "$HERE/sonic_key.py" v 2>&1 | tee -a "$OUT/keys.log"
sleep 3

WIN=$("$PY" - <<'EOF'
from Xlib import display
d = display.Display(); r = d.screen().root; rg = r.get_geometry()
best, area = None, 0
for w in r.query_tree().children:
    try: g = w.get_geometry()
    except Exception: continue
    a = g.width * g.height
    if 400_000 < a < 0.95 * rg.width * rg.height and a > area: best, area = w, a
print(hex(best.id) if best else "")
EOF
)
echo "[rec] viewer window = ${WIN:-<none>}"
[ -n "$WIN" ] || { echo "no window -- cannot record"; kill $SIM $POL; exit 1; }
nohup "$HERE/sonic_record.sh" "$WIN" $((CONTROL_S + 70)) "$OUT/rec" > "$OUT/rec.log" 2>&1 &
REC=$!
sleep 3
[ "$(ls "$OUT/rec/frames"/*.xwd 2>/dev/null | wc -l)" -gt 0 ] || {
    echo "recorder captured nothing in 3 s -- xwd is failing on $WIN"; cat "$OUT/rec.log"; }

echo "[5/6] CONTROL phase: ${CONTROL_S}s with NO commands published"
mark control_start
sleep "$CONTROL_S"
mark control_end

echo "[6/6] GEMINI phase: \"$PROMPT\""
mark gemini_start
( cd "$REPO" && "$PY" scripts/sonic_sim_demo.py --link sonic --no-camera \
    --max-hops 8 "$PROMPT" ) 2>&1 | tee "$OUT/gemini.log"
mark gemini_end
sleep 3

echo
# NOT grep "has fallen" on sim.log: the simulator's stdout is block-buffered to a
# file and is lost when the process is killed, so that grep returns 0 whether or
# not the robot fell. Falls are judged from the pose log instead -- minimum
# pelvis height and RESET rows -- which is line-buffered and survives.
"$PY" "$HERE/sonic_analyze.py" "$OUT" | tee "$OUT/VERDICT.txt"

sleep 8   # let the recorder finish encoding
kill $SIM $POL 2>/dev/null
echo "artifacts in $OUT"
