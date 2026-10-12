#!/usr/bin/env bash
# ONE COMMAND: Isaac Sim + SONIC + a prompt you can type into.
#
# Ordering is copied from sonic_live.sh because it is the ordering that works:
# sim up -> policy "Init Done" -> frontend. One deliberate difference: there is
# NO physics gate and NO elastic band. The vendor controller holds the default
# pose until the policy takes over -- measured 2026-08-07: the robot stood for
# 3+ minutes before SONIC attached and the takeover cost 4.3 deg of tilt, max.
# MuJoCo needs the band because its loop steps a bare robot; Isaac does not.
#
# SONIC_FRONTEND lets a scripted probe reuse this entire bring-up instead of
# reimplementing the start handshake, which is the step most easily got subtly
# wrong. Default is the interactive console.
set -uo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(dirname "$HERE")"
# External checkouts, all env-configurable -- see physicalagent/paths.py.
# There are no machine-specific literals here; the script refuses rather than
# guessing, because a wrong guess launches a simulator against the wrong tree.
WBC="${WBC_ROOT:-$HOME/GR00T-WholeBodyControl}"
DEPLOY=$WBC/gear_sonic_deploy
VENDOR="${UNITREE_SIM_ROOT:-}"
ISAAC_PY="${SONIC_ISAAC_PYTHON:-${ISAAC_PY:-}}"
for v in VENDOR ISAAC_PY; do
  if [ -z "${!v}" ]; then
    echo "$v is unset. Set UNITREE_SIM_ROOT (a unitree_sim_isaaclab checkout)" >&2
    echo "and ISAAC_PY (its interpreter) before running the Isaac stack." >&2
    exit 2
  fi
done
LOGS=${LOGS:-$REPO/artifacts/isaac_live}
TASK="${SONIC_ISAAC_TASK:-Isaac-Move-Cylinder-G129-Inspire-Wholebody}"

# LOAD-BEARING for every DDS-touching process under unitree_sim_env, and for
# nothing else. Without CYCLONEDDS_HOME the python binding's loader resolves a
# SECOND C core out of the ldconfig cache (find_library ignores
# LD_LIBRARY_PATH), and with two cores in one process topic creation dies
# PRECONDITION_NOT_MET -- the sim then runs and publishes nothing. Full story:
# scripts/dds_interop_probe.py, "FIXED 2026-08-07".
# The SONIC binary carries its own DDS and is launched WITHOUT these.
CDDS="${ISAAC_CYCLONEDDS_HOME:-}"
if [ -z "$CDDS" ]; then
  echo "ISAAC_CYCLONEDDS_HOME is unset -- point it at the source-built" >&2
  echo "CycloneDDS install prefix, or the sim will publish into nothing." >&2
  exit 2
fi
ISAAC_DDS_ENV=(CYCLONEDDS_HOME="$CDDS" LD_LIBRARY_PATH="$CDDS/lib")

# FULL-BODY ROUTING (patch 05), default ON. Without it the vendor's wholebody
# mode is an XR-teleop hybrid: legs = the vendor's own policy.onnx, waist =
# pinned defaults, SONIC drives ARMS ONLY at the asset's gains -- the robot
# that "balanced" through M0 was never under SONIC's control (found
# 2026-08-12). Set SONIC_WB_FULL_BODY=0 only for a deliberate A/B against the
# old hybrid behavior.
WB_FULL_BODY="${SONIC_WB_FULL_BODY:-1}"

# REAL-TIME PACING (patch 06 + tools/sonic_timescale.so), measured 2026-08-12.
# The loop cost 94.6 ms per 20 ms of sim time (RTF 0.212), of which 41.0 ms
# was rendering that the vendor's --no_render CANNOT disable on this path --
# the provider calls env.sim.render() directly, while that flag only gates
# env.step(), which this mode never calls. Rendering one iteration in
# RENDER_EVERY drops it to ~10 ms at N=4 (RTF ~0.40) for a viewport that
# still moves; N=8 buys ~0.43 with a choppier view.
RENDER_EVERY="${SONIC_WB_RENDER_EVERY:-4}"
# Two of the scene's five cameras are never read by anything; excluding them
# costs nothing. The write interval trades camera freshness for loop time --
# keep it well under the sidecar's 2 s staleness gate or every camera answer
# becomes a refusal.
CAM_EXCLUDE="${SONIC_ISAAC_CAM_EXCLUDE:-world_camera,robot_camera}"
CAM_INTERVAL="${SONIC_ISAAC_CAM_INTERVAL:-10}"
# SONIC paces itself on the WALL clock, so a backend at RTF<1 receives a
# controller whose motions outrun the physics -- at RTF 0.18 the legs churned
# at 7.65 rad/s while merely standing. SONIC_TIME_SCALE stretches its thread
# periods to match (roughly 1/RTF), leaving its internal dt untouched so all
# its own math stays right in SIM time. SIM ONLY -- never with a real robot.
# Pinning the RTF is what makes TIME_SCALE deterministic: a drifting RTF
# means a fixed scale is sometimes right and sometimes not, and the four runs
# of 2026-08-18 separated perfectly on exactly that. Target BELOW the worst
# observed rate (0.205 under load) so the loop can always hold it; then a
# fixed scale of 1/target is always correct. 0 disables.
TARGET_RTF="${SONIC_WB_TARGET_RTF:-0.20}"
TIME_SCALE="${SONIC_TIME_SCALE:-}"
TIMESCALE_SO="$REPO/tools/sonic_timescale.so"

# Pre-flight must exercise the SAME loader configuration runtime uses: an
# import check without the env vars validates the find_library fallback path,
# which is precisely the broken one.
[ -e "$CDDS/lib/libddsc.so" ] || {
    echo "ERROR: load-bearing cyclonedds install missing: $CDDS/lib/libddsc.so"
    echo "  (see the ISAAC_DDS_ENV comment in this script)"; exit 1; }
[ -x "$ISAAC_PY" ] && env "${ISAAC_DDS_ENV[@]}" "$ISAAC_PY" \
        -c "import unitree_sdk2py, cyclonedds" 2>/dev/null || {
    echo "ERROR: $ISAAC_PY cannot import unitree_sdk2py + cyclonedds."
    echo "  The env with Isaac Sim AND the Unitree SDK is the conda env"
    echo "  'unitree_sim_env'. Override with SONIC_ISAAC_PYTHON."
    echo "  Use ./scripts/sonic_live.sh (MuJoCo) meanwhile -- the measured path."
    exit 1; }
[ -f "$VENDOR/sim_main.py" ] || { echo "ERROR: no vendor sim at $VENDOR"; exit 1; }

mkdir -p "$LOGS"
unset CYCLONEDDS_URI || true          # loopback DDS works; setting this breaks it
export DISPLAY="${DISPLAY:-:0}"
export SONIC_SIDECAR_PORT="${SONIC_SIDECAR_PORT:-5570}"

# Guarded kills only: `kill "${SIM:-0}"` with SIM unset signals pid 0 -- the
# WHOLE process group, including whatever shell sourced us -- on any early exit.
cleanup() { echo; echo "[isaac] shutting down"
            [ -n "${SIM:-}" ] && kill "$SIM" 2>/dev/null
            [ -n "${POL:-}" ] && kill "$POL" 2>/dev/null
            wait "${SIM:-}" "${POL:-}" 2>/dev/null; }
trap cleanup EXIT INT TERM

# Port check FIRST: a LISTENing 5556 is a live console (MuJoCo session or
# otherwise) and this launch must refuse WITHOUT having already SIGTERMed that
# session's policy -- the deploy binary is the same file on both paths.
for port in 5556 "$SONIC_SIDECAR_PORT"; do
    if ss -lnt "sport = :$port" 2>/dev/null | grep -q LISTEN; then
        echo "ERROR: port $port is held -- a live session is running:";
        ss -lntp "sport = :$port" | tail -n +2
        echo "  Quit it first; refusing to kill a session this script did not start."
        exit 1
    fi
done
# Now clear genuinely stale processes (they hold DDS participants, and a stale
# console binds 5556 only while alive-and-listening, which the check above
# already excluded). [x]-class patterns so pkill cannot match its own command
# line. pkill matches process COMMAND LINES, not file contents, so the launch
# strings later in this file are safe -- what is NOT safe is putting a pkill
# and a literal target string on one shared `bash -c` line; that trap has
# fired twice in this project.
pkill -f "[s]im_main.py" 2>/dev/null
pkill -f "[g]1_deploy_onnx_ref" 2>/dev/null
pkill -f "[s]onic_console.py" 2>/dev/null
sleep 2

# Fresh logs, ALWAYS: the readiness gates below grep for success strings, and
# evidence copies of previous runs' logs live in git at this very path -- a
# stale file would satisfy the gate before the process it watches has printed
# anything (or after it crashed).
: > "$LOGS/sim.log"
: > "$LOGS/policy.log"

echo "[1/3] Isaac Sim ($TASK) -> $LOGS/sim.log   (cold start can take ~2 min)"
( trap '' INT; cd "$VENDOR" && exec env "${ISAAC_DDS_ENV[@]}" \
    SONIC_WB_FULL_BODY="$WB_FULL_BODY" SONIC_WB_RENDER_EVERY="$RENDER_EVERY" \
    SONIC_WB_TARGET_RTF="$TARGET_RTF" \
    "$ISAAC_PY" sim_main.py \
    --enable_cameras --task "$TASK" --robot_type g129 --enable_inspire_dds \
    --camera_exclude "$CAM_EXCLUDE" --camera_write_interval "$CAM_INTERVAL" \
    ) > "$LOGS/sim.log" 2>&1 &
SIM=$!
# "[odostate] publisher initialized" is the LAST DDS init line the sim prints;
# waiting for it means lowstate + odostate are both live, not merely that the
# process exists.
for i in $(seq 1 150); do
    grep -q "\[odostate\] publisher initialized" "$LOGS/sim.log" 2>/dev/null && break
    kill -0 $SIM 2>/dev/null || { echo "Isaac died -- see $LOGS/sim.log"; exit 1; }
    sleep 2
done
grep -q "\[odostate\] publisher initialized" "$LOGS/sim.log" || {
    echo "Isaac never finished DDS bring-up -- see $LOGS/sim.log"; exit 1; }
# The full-body banner prints when the action provider is constructed, which
# is BEFORE the odostate line above -- so on a Wholebody task its absence
# means the vendor tree is missing patch 05 and the sim is silently running
# the arms-only teleop hybrid. That is the exact lie this project already
# fell for once; refuse to hand a console to it. Non-Wholebody tasks use a
# different provider entirely (sim_main flips on the task NAME), so the
# banner legitimately never prints there and the gate would misdiagnose.
case "$TASK" in
*Wholebody*)
    if [ "$WB_FULL_BODY" = "1" ]; then
        grep -q "SONIC FULL-BODY ROUTING ENABLED" "$LOGS/sim.log" || {
            echo "ERROR: SONIC_WB_FULL_BODY=1 but the sim never announced full-body"
            echo "  routing. The vendor tree at $VENDOR"
            echo "  is missing 05-sonic-full-body-routing.patch -- without it SONIC"
            echo "  drives the ARMS ONLY and the legs are the vendor's own policy."
            exit 1; }
    else
        echo "WARNING: SONIC_WB_FULL_BODY=0 -- teleop hybrid mode: SONIC drives ARMS ONLY."
    fi
    ;;
*)
    echo "NOTE: task '$TASK' is not a Wholebody task; full-body routing does not apply."
    ;;
esac

echo "[2/3] SONIC policy -> $LOGS/policy.log   (TensorRT load, ~35 s)"
# Build the pacing shim on demand; a stale .so beside a newer .c is a silent
# wrong answer, so rebuild whenever the source is newer.
POLICY_ENV=()
if [ -n "$TIME_SCALE" ]; then
    if [ ! -f "$TIMESCALE_SO" ] || [ "$REPO/tools/sonic_timescale.c" -nt "$TIMESCALE_SO" ]; then
        cc -shared -fPIC -O2 -o "$TIMESCALE_SO" "$REPO/tools/sonic_timescale.c" -ldl || {
            echo "ERROR: could not build $TIMESCALE_SO"; exit 1; }
    fi
    POLICY_ENV=(LD_PRELOAD="$TIMESCALE_SO" SONIC_TIME_SCALE="$TIME_SCALE")
    echo "      pacing: SONIC slowed x$TIME_SCALE to match a sim below real time (SIM ONLY)"
fi
( trap '' INT; cd "$DEPLOY" && exec env "${POLICY_ENV[@]}" ./target/release/g1_deploy_onnx_ref lo \
      policy/release/model_decoder.onnx reference/example/ \
      --obs-config   policy/release/observation_config.yaml \
      --encoder-file policy/release/model_encoder.onnx \
      --planner-file planner/target_vel/V2/planner_sonic.onnx \
      --input-type   zmq_manager --output-type all \
      --zmq-host localhost --zmq-port 5556 \
      --disable-crc-check ) > "$LOGS/policy.log" 2>&1 &
POL=$!
for i in $(seq 1 60); do
    grep -q "Init Done" "$LOGS/policy.log" 2>/dev/null && break
    kill -0 $POL 2>/dev/null || { echo "policy died -- see $LOGS/policy.log"; exit 1; }
    sleep 2
done
grep -q "Init Done" "$LOGS/policy.log" || { echo "policy never reached Init Done"; exit 1; }
# "Init Done" only means the engines loaded; the safety check that refuses to
# start (no rt/lowstate, bad state) prints AFTER it. Same guard as sonic_live.sh.
sleep 2
grep -q "Safety check failed" "$LOGS/policy.log" && {
    echo "policy refused to start -- it is not seeing rt/lowstate. See $LOGS/policy.log"
    exit 1; }

# The frontend runs under .venv_sim exactly as it does on the MuJoCo path: it
# needs google-genai, zmq and unitree_sdk2py, none of which is Isaac's job --
# and .venv_sim is natively in SONIC's DDS world, so no env vars either.
# The default console gets --no-band-release: there is no band on this backend,
# and the band-release fallback would inject an XTEST '9' keystroke into
# whatever window looks likely -- the Isaac viewport, here.
FRONTEND="${SONIC_FRONTEND:-}"
FRONTEND_PY="${SONIC_FRONTEND_PYTHON:-$WBC/.venv_sim/bin/python}"
echo "[3/3] policy ready -- handing over to ${FRONTEND:-scripts/sonic_console.py}"
echo
cd "$REPO"
if [ -z "$FRONTEND" ]; then
    "$FRONTEND_PY" scripts/sonic_console.py --no-band-release "$@"
else
    "$FRONTEND_PY" "$FRONTEND" "$@"
fi
