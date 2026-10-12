#!/usr/bin/env bash
# Launch the lounge scene through the standard bring-up, with resource gates.
#
#   bash scripts/lounge_live.sh --check     # gates only (GPU, RAM, other sims)
#   bash scripts/lounge_live.sh             # gates, then sonic_live.sh + console
#   SONIC_FRONTEND=scripts/lounge_battery_probe.py \
#       bash scripts/lounge_live.sh --pass-name A
#
# The gates exist because a PARALLEL Isaac session shares this machine's GPU and
# the DDS world on lo domain 0 — and because sonic_live.sh pkills stale
# g1_deploy_onnx_ref processes, which would kill THAT session's policy if it is
# live. We refuse and report; we never kill.
set -uo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(dirname "$HERE")"
SCENE_REL="gear_sonic/data/robot_model/model_data/g1/scene_29dof_inspire_lounge.xml"
WBC="${WBC_ROOT:-$HOME/GR00T-WholeBodyControl}"
SCENE_ABS="$WBC/$SCENE_REL"

CHECK_ONLY=0
if [ "${1:-}" = "--check" ]; then CHECK_ONLY=1; shift; fi

fail=0
running=$(pgrep -af "[r]un_sim_loop|[g]1_deploy_onnx_ref|[s]im_main.py|[s]onic_console" || true)
if [ -n "$running" ]; then
    echo "GATE FAIL: another sim/policy stack is alive. NOT killing it —"
    echo "coordinate with the other session, then retry:"
    echo "$running"
    fail=1
fi
gpu_free=$(nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits 2>/dev/null | head -1)
if ! [[ "${gpu_free:-}" =~ ^[0-9]+$ ]]; then
    echo "GATE FAIL: nvidia-smi did not return a number (got: '${gpu_free:-}')"; fail=1
elif [ "$gpu_free" -lt 2500 ]; then
    echo "GATE FAIL: only ${gpu_free} MiB GPU free (<2500). Another GPU job is running."
    fail=1
fi
# GPU COMPUTE CO-TENANT gate — free VRAM alone is NOT enough: a rendering CUDA
# job starves the 500 Hz control loop while leaving memory plentiful (voided
# pass B, 2026-08-12: LowState age 28-56 ms under a concurrent Isaac RTX run;
# the robot fell on every task). Two probes, both fail-closed:
#  1) any CUDA compute process holding >1500 MiB (Isaac ran at 2.4-3.6 GB;
#     the desktop compositor stays ~0.5 GB, so 1500 separates them cleanly);
#  2) utilization pinned high across 5 samples — MINIMUM over the window, so
#     bursty desktop compositing (spiky, dips between frames) passes while a
#     sustained render/compute job (high in every sample) fails.
# Rows with non-numeric used_memory ("[N/A]") are skipped DELIBERATELY: a
# process nvidia-smi cannot size either isn't holding meaningful VRAM (the
# unchanged memory.free gate backstops that) or is compute-idle (probe 2
# catches active ones). Reviewed 2026-08-13: acceptable on this single-GPU box.
hogs=$(nvidia-smi --query-compute-apps=pid,used_memory --format=csv,noheader,nounits 2>/dev/null \
    | awk -F', *' '$2 ~ /^[0-9]+$/ && $2 > 1500 {print "  pid "$1" holding "$2" MiB"}')
if [ -n "$hogs" ]; then
    echo "GATE FAIL: CUDA compute co-tenant(s) big enough to starve the control loop —"
    echo "$hogs"
    echo "coordinate with the session that owns it, then retry. NOT killing it."
    fail=1
fi
util_min=""
for _ in 1 2 3 4 5; do
    u=$(nvidia-smi --query-gpu=utilization.gpu --format=csv,noheader,nounits 2>/dev/null | head -1)
    if ! [[ "${u:-}" =~ ^[0-9]+$ ]]; then util_min="probe-error:'${u:-}'"; break; fi
    if ! [[ "$util_min" =~ ^[0-9]+$ ]] || [ "$u" -lt "$util_min" ]; then util_min=$u; fi
    sleep 1
done
if ! [[ "${util_min:-}" =~ ^[0-9]+$ ]]; then
    echo "GATE FAIL: GPU utilization probe did not return numbers (got: '${util_min:-}')"; fail=1
elif [ "$util_min" -ge 60 ]; then
    echo "GATE FAIL: GPU utilization >=60% in ALL of 5 samples (min ${util_min}%) —"
    echo "a sustained render/compute job is active. Coordinate, then retry."
    fail=1
fi
ram_avail=$(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo)
if ! [[ "${ram_avail:-}" =~ ^[0-9]+$ ]]; then
    echo "GATE FAIL: /proc/meminfo MemAvailable unreadable (got: '${ram_avail:-}')"; fail=1
elif [ "$ram_avail" -lt 4096 ]; then
    echo "GATE FAIL: only ${ram_avail} MiB RAM available (<4096)"; fail=1
fi
if [ ! -f "$SCENE_ABS" ]; then
    echo "GATE FAIL: $SCENE_ABS missing. Generate it:"
    echo "  $WBC/.venv_sim/bin/python simbridge/make_lounge_scene.py"
    fail=1
fi
[ "$fail" -ne 0 ] && exit 1
echo "GATES OK: gpu_free=${gpu_free}MiB util_min=${util_min}% ram_avail=${ram_avail}MiB no rival stack"
[ "$CHECK_ONLY" -eq 1 ] && exit 0

export SONIC_CUSTOM_SCENE="$SCENE_REL"
export SONIC_HANDS=actuated              # informational; the scene includes the
                                         # actuated robot regardless
export LOGS="$REPO/artifacts/live"       # keep runtime state inside THIS tree
exec "$HERE/sonic_live.sh" "$@"
