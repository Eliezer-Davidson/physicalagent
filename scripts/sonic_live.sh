#!/usr/bin/env bash
# ONE COMMAND: live MuJoCo window + a prompt you can type into.
#
#   ./scripts/sonic_live.sh
#
# Brings up the MuJoCo simulator (with a real on-screen window), the SONIC
# policy, performs the start handshake, releases the elastic band, and hands you
# a `g1>` prompt. Type plain English; Gemini Robotics-ER 2 drives the robot and
# you watch it happen.
#
#   g1> walk forward three metres then turn left
#   g1> stop            <- immediate, never consults the model
#   g1> quit            <- halts the robot and shuts everything down
#
# ---------------------------------------------------------------------------
# THE THREE THINGS THAT MAKE THIS WORK. Each cost hours; none is obvious.
#
#  1. PYGLFW_LIBRARY_VARIANT=x11 is THE fix for the missing window.
#     glfw/library.py:219-225 picks which libglfw.so to load: if
#     XDG_SESSION_TYPE == 'wayland' it loads the WAYLAND build, which cannot
#     open an X window at all. mujoco.viewer.launch_passive() then returns a
#     live Handle, is_running() is True, nothing is logged, and it renders into
#     the void -- a success-shaped failure. Unsetting WAYLAND_DISPLAY does NOT
#     change this choice.
#
#  2. ORDER: policy ready -> start command -> THEN release the band. The policy
#     publishes nothing that holds the robot until it gets
#     command{start:1, planner:1}, and its TensorRT engine takes ~35 s to load.
#     Drop the robot before that and it just falls. ENABLE_ELASTIC_BAND must
#     stay True for exactly this reason.
#
#  3. DO NOT SET CYCLONEDDS_URI. Loopback DDS works out of the box. Both
#     "selected interface lo is not multicast-capable" and "[ChannelFactory]
#     create domain error" are benign -- init is called twice on purpose and
#     NVIDIA wraps the second in try/except.
# ---------------------------------------------------------------------------
set -uo pipefail

WBC="${WBC_ROOT:-$HOME/GR00T-WholeBodyControl}"
DEPLOY=$WBC/gear_sonic_deploy
PY=$WBC/.venv_sim/bin/python
CFGDIR=$WBC/gear_sonic/utils/mujoco_sim/wbc_configs
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(dirname "$HERE")"
LOGS=${LOGS:-${PA_ARTIFACTS:-$REPO/artifacts}/live}

YAML=$CFGDIR/g1_29dof_sonic_model12.yaml

# SONIC_HANDS=actuated swaps in the scene whose Inspire fingers have motors, so
# set_grasp() can do something. Default is the rigid-hand scene the walking runs
# were proven on -- a proof is only worth something if the thing it proved is
# still what runs by default.
if [ -n "${SONIC_CUSTOM_SCENE:-}" ]; then
    export SONIC_ROBOT_SCENE="${SONIC_CUSTOM_SCENE}"
else
    case "${SONIC_HANDS:-rigid}" in
        actuated) export SONIC_ROBOT_SCENE="gear_sonic/data/robot_model/model_data/g1/scene_29dof_inspire_actuated.xml" ;;
        rigid)    unset SONIC_ROBOT_SCENE ;;
        *) echo "SONIC_HANDS must be 'rigid' or 'actuated'"; exit 1 ;;
    esac
fi

mkdir -p "$LOGS"
unset CYCLONEDDS_URI WAYLAND_DISPLAY || true
export PYGLFW_LIBRARY_VARIANT=x11
export XDG_SESSION_TYPE=x11
export DISPLAY="${DISPLAY:-:0}"
export MUJOCO_GL=glfw
export SONIC_POSE_LOG="$LOGS/pose.csv"   # ground truth, so any claim is checkable
# The simulator drops the elastic band when this file appears. Deterministic,
# unlike synthesising the '9' keypress through XTEST, which fails intermittently
# on XWayland -- and fails SILENTLY, leaving the robot hanging in mid-air where
# every command looks like it went nowhere.
export SONIC_BAND_RELEASE_FILE="$LOGS/release_band"
rm -f "$SONIC_BAND_RELEASE_FILE"
# The sidecar answers camera / lidar / contact / finger requests from inside the
# simulation loop. Off unless this port is set, so nothing changes for anyone
# running the vendor script directly.
export SONIC_SIDECAR_PORT="${SONIC_SIDECAR_PORT:-5570}"
export SONIC_SIDECAR_PATH="$REPO"

grep -q '^ENABLE_ELASTIC_BAND: True' "$YAML" || {
    echo "ERROR: set ENABLE_ELASTIC_BAND: True in $YAML -- without it nothing"
    echo "       holds the robot while the policy loads."; exit 1; }

cleanup() {
    echo
    echo "[live] shutting down"
    kill "${SIM:-0}" "${POL:-0}" 2>/dev/null
    wait "${SIM:-0}" "${POL:-0}" 2>/dev/null
}
trap cleanup EXIT INT TERM

# Clear out anything left from a previous run. The console is included because
# it BINDS 5556: a stale one from a session that ended badly takes the port and
# the next launch dies at the very last step, after paying the 35 s TensorRT
# load. The [x]-class patterns stop pkill matching its own command line.
pkill -f "[r]un_sim_loop" 2>/dev/null
pkill -f "[g]1_deploy_onnx_ref" 2>/dev/null
pkill -f "[s]onic_console.py" 2>/dev/null
sleep 2
for port in 5556 "$SONIC_SIDECAR_PORT"; do
    if ss -lnt "sport = :$port" 2>/dev/null | grep -q LISTEN; then
        echo "ERROR: port $port is still held by another process:"
        ss -lntp "sport = :$port" 2>/dev/null | tail -n +2
        exit 1
    fi
done

echo "[1/4] MuJoCo simulator (windowed, hands=${SONIC_HANDS:-rigid}) -> $LOGS/sim.log"
# `trap '' INT` before exec: an IGNORED signal disposition survives exec, so
# terminal Ctrl-C (which SIGINTs the whole foreground process group) can no
# longer kill the simulator out from under the console — that used to cost a
# full ~90 s restart per accident (§4e-6). cleanup's SIGTERM still works and
# the PIDs stay honest, which setsid would have broken.
( trap '' INT; cd "$WBC" && exec "$PY" gear_sonic/scripts/run_sim_loop.py --enable-onscreen ) \
    > "$LOGS/sim.log" 2>&1 &
SIM=$!
sleep 12
kill -0 $SIM 2>/dev/null || { echo "simulator died -- see $LOGS/sim.log"; exit 1; }
grep -qE "0x502|Failed to create XKB" "$LOGS/sim.log" && \
    echo "  WARNING: GL error in the log -- the window may be missing."

echo "[2/4] SONIC policy -> $LOGS/policy.log   (TensorRT load, ~35 s)"
( trap '' INT; cd "$DEPLOY" && exec ./target/release/g1_deploy_onnx_ref lo \
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
    kill -0 $POL 2>/dev/null || { echo "policy died -- see $LOGS/policy.log"; exit 1; }
    sleep 2
done
grep -q "Init Done" "$LOGS/policy.log" || { echo "policy never reached Init Done"; exit 1; }
grep -q "Safety check failed" "$LOGS/policy.log" && {
    echo "policy refused to start -- it is not seeing rt/lowstate."; exit 1; }

# SONIC_FRONTEND lets a scripted check reuse this entire bring-up instead of
# reimplementing the start handshake and the band release, which are the two
# steps most easily got subtly wrong. Default is the interactive console.
FRONTEND="${SONIC_FRONTEND:-scripts/sonic_console.py}"

echo "[3/4] policy ready -- handing over to $FRONTEND"
echo "[4/4] it will send start, release the band, then take over"
echo

cd "$REPO" && "$PY" "$FRONTEND" "$@"
