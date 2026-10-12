#!/usr/bin/env bash
# Bring up SONIC + MuJoCo with a LIVE WINDOW you can watch.
#
#   ./scripts/sonic_sim_up.sh
#
# Then: click the MuJoCo window, press 9 to drop the robot off its band.
# Then, in another terminal, drive it with a typed prompt:
#   .venv_sim/bin/python scripts/sonic_sim_demo.py --link sonic "walk forward"
#
# ---------------------------------------------------------------------------
# WHY THE ROBOT USED TO FALL OVER, and the three things that actually fix it.
# Each cost hours; none of them is what it first looked like.
#
#  1. THE WINDOW. `PYGLFW_LIBRARY_VARIANT=x11` is THE fix.
#     glfw/library.py:219-225 picks which libglfw.so to load:
#         if PYGLFW_LIBRARY_VARIANT in (wayland, x11) -> use it
#         elif XDG_SESSION_TYPE == 'wayland'          -> use the WAYLAND build
#         else                                        -> use the X11 build
#     This session is XDG_SESSION_TYPE=wayland, so the Wayland-only build
#     loaded — and it CANNOT open an X window. mujoco.viewer.launch_passive()
#     then returned a live Handle, is_running() was True, nothing was logged,
#     and it rendered into the void. Unsetting WAYLAND_DISPLAY does NOT change
#     this choice; it only silences the louder earlier failure
#     ("Wayland: Failed to create XKB compose table" -> OpenGL error 0x502).
#     No window meant the sim degraded, stopped publishing rt/lowstate, and the
#     policy correctly refused to start.
#
#  2. THE START COMMAND. The policy publishes nothing useful until it receives
#     command{start:1, planner:1} (keyboard ']' or ZMQ). Loading its TensorRT
#     engine takes ~35 s, and until then NOTHING holds the robot up. That is
#     what the elastic band is for, so leave ENABLE_ELASTIC_BAND: True and drop
#     the robot only once the policy is ready.
#
#  3. DO NOT SET CYCLONEDDS_URI. It was never needed; loopback DDS works out of
#     the box. "selected interface lo is not multicast-capable" is BENIGN, and
#     so is "[ChannelFactory] create domain error" (init is called twice on
#     purpose and NVIDIA wraps the second in try/except).
# ---------------------------------------------------------------------------
set -euo pipefail

WBC="${WBC_ROOT:-$HOME/GR00T-WholeBodyControl}"
DEPLOY=$WBC/gear_sonic_deploy
PY=$WBC/.venv_sim/bin/python
YAML=$WBC/gear_sonic/utils/mujoco_sim/wbc_configs/g1_29dof_sonic_model12.yaml
LOGS=${LOGS:-/tmp/sonic_sim}
HERE="$(cd "$(dirname "$0")" && pwd)"

mkdir -p "$LOGS"
unset CYCLONEDDS_URI WAYLAND_DISPLAY || true
export PYGLFW_LIBRARY_VARIANT=x11      # (1) — the one that matters
export XDG_SESSION_TYPE=x11
export DISPLAY="${DISPLAY:-:0}"
export MUJOCO_GL=glfw

grep -q '^ENABLE_ELASTIC_BAND: True' "$YAML" || {
    echo "ERROR: set ENABLE_ELASTIC_BAND: True in $YAML — without it nothing"
    echo "       holds the robot during the ~35 s the policy takes to load."; exit 1; }

pkill -f "[r]un_sim_loop" 2>/dev/null || true
pkill -f "[g]1_deploy_onnx_ref" 2>/dev/null || true
sleep 2

echo "[1/3] MuJoCo simulator (windowed) -> $LOGS/sim.log"
( cd "$WBC" && exec "$PY" gear_sonic/scripts/run_sim_loop.py --enable-onscreen ) \
    > "$LOGS/sim.log" 2>&1 &
SIM=$!
sleep 12
kill -0 $SIM 2>/dev/null || { echo "simulator died — see $LOGS/sim.log"; exit 1; }
grep -qE "0x502|Failed to create XKB" "$LOGS/sim.log" && {
    echo "GL failed — the window will be missing. Check PYGLFW_LIBRARY_VARIANT."; }

echo "[2/3] SONIC policy -> $LOGS/policy.log   (TensorRT load, ~35 s)"
( cd "$DEPLOY" && exec ./target/release/g1_deploy_onnx_ref lo \
      policy/release/model_decoder.onnx reference/example/ \
      --obs-config   policy/release/observation_config.yaml \
      --encoder-file policy/release/model_encoder.onnx \
      --planner-file planner/target_vel/V2/planner_sonic.onnx \
      --input-type   zmq_manager --output-type all \
      --zmq-host localhost --zmq-port 5556 \
      --disable-crc-check ) > "$LOGS/policy.log" 2>&1 &
POL=$!

for i in $(seq 1 30); do
    grep -q "Init Done" "$LOGS/policy.log" 2>/dev/null && break
    kill -0 $POL 2>/dev/null || { echo "policy died — see $LOGS/policy.log"; kill $SIM; exit 1; }
    sleep 2
done
grep -q "Init Done" "$LOGS/policy.log" || { echo "policy never reached 'Init Done'"; exit 1; }
grep -q "Safety check failed" "$LOGS/policy.log" && {
    echo "policy refused to start — it is not seeing rt/lowstate."; kill $SIM $POL; exit 1; }

echo "[3/3] sending start + planner mode over ZMQ"
"$PY" "$HERE/sonic_start.py" || { echo "start command failed"; kill $SIM $POL; exit 1; }

cat <<EOF

  simulator pid $SIM   policy pid $POL
  logs: $LOGS/sim.log  $LOGS/policy.log

  >>> NOW: click the MuJoCo window and press  9  to release the elastic band. <<<
      The robot drops and the policy should catch and hold it.

  Check the plumbing any time (expect lowstate ~1800, lowcmd ~5000 per 10 s):
      $PY $HERE/sonic_check.py

  Drive it with a prompt:
      cd $(dirname "$HERE")
      $PY scripts/sonic_sim_demo.py --link sonic "walk forward, then turn left"

  stop:  kill $SIM $POL
EOF
wait
