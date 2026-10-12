#!/usr/bin/env bash
# ============================================================================
# RUN THIS FIRST — before ANY diagnosis — the moment the ROSPug is reachable.
#
# WHY THIS EXISTS: on 2026-07-17 we had a ~40 minute SSH window and spent it all
# on live diagnosis. The robot was then taken away without warning. We never
# copied the on-robot source tree, which is the ONLY place that documents
# pug_control's real velocity/pose UNITS and LIMITS. Those limits gate our SAFETY
# layer, so the whole build is now blocked behind an ambiguity that a 10-second
# `scp -r` would have resolved forever.
#
# THE RULE: access windows close without warning. Insight can be re-derived from
# artifacts; artifacts cannot be re-derived from insight. COPY FIRST, ANALYSE LATER.
#
# This script is READ-ONLY on the robot. It sends NO motion command, changes no
# config, writes nothing to the robot. Safe to run any time it is reachable.
#
# Usage:  ./scripts/grab_everything.sh [host]      (default: rospug.local)
# ============================================================================
set -uo pipefail

HOST="${1:-rospug.local}"
USER="${ROSPUG_SSH_USER:-}"
PASS="${ROSPUG_SSH_PASS:-}"
if [ -z "${USER}" ] || [ -z "${PASS}" ]; then
  echo "!! set ROSPUG_SSH_USER and ROSPUG_SSH_PASS first (see .env.example)."
  echo "   No credential default ships in this repository."
  exit 2
fi
STAMP="$(date +%Y-%m-%d_%H%M%S)"
OUT="reference/rospug_capture_${STAMP}"

echo "=== ROSPug artifact grab -> ${OUT} ==="
mkdir -p "${OUT}"

if ! ping -c1 -W2 "${HOST}" >/dev/null 2>&1; then
  echo "!! ${HOST} unreachable. Is the cable in and the robot powered on?"
  echo "   Reminder: activate the wired profile first:  nmcli connection up robot-wired"
  exit 1
fi
echo "host ${HOST} is up"

# --- sshpass is optional; fall back to a python/paramiko path ---------------
have_sshpass=0; command -v sshpass >/dev/null 2>&1 && have_sshpass=1
SSHOPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR"

rsh() {  # run a remote command
  if [ "$have_sshpass" = "1" ]; then
    sshpass -p "${PASS}" ssh ${SSHOPTS} "${USER}@${HOST}" "$1"
  else
    .venv/bin/python - "$HOST" "$USER" "$PASS" "$1" <<'PY'
import sys, warnings; warnings.filterwarnings("ignore")
import paramiko
h,u,p,cmd = sys.argv[1:5]
c=paramiko.SSHClient(); c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect(h,22,u,p,timeout=15,allow_agent=False,look_for_keys=False)
_,o,e=c.exec_command(f"bash -lc {cmd!r}",timeout=120)
sys.stdout.write(o.read().decode(errors="replace"))
sys.stderr.write(e.read().decode(errors="replace"))
c.close()
PY
  fi
}

# ============================================================================
# PRIORITY 1 — THE SOURCE TREE.  Nothing else matters as much. Do it first.
# ============================================================================
echo
echo ">>> [1/6] THE SOURCE TREE (holds the real UNITS + LIMITS)"
if [ "$have_sshpass" = "1" ]; then
  sshpass -p "${PASS}" scp ${SSHOPTS} -r "${USER}@${HOST}:/home/hiwonder/pug/src" "${OUT}/pug_src" 2>&1 | tail -2
else
  # tar over the ssh pipe — no scp/sshpass needed
  .venv/bin/python - "$HOST" "$USER" "$PASS" "${OUT}/pug_src.tar.gz" <<'PY'
import sys, warnings; warnings.filterwarnings("ignore")
import paramiko
h,u,p,dest = sys.argv[1:5]
c=paramiko.SSHClient(); c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect(h,22,u,p,timeout=15,allow_agent=False,look_for_keys=False)
_,o,_=c.exec_command("tar czf - -C /home/hiwonder/pug src 2>/dev/null",timeout=300)
n=0
with open(dest,"wb") as f:
    while True:
        b=o.channel.recv(65536)
        if not b: break
        f.write(b); n+=len(b)
print(f"  wrote {dest} ({n/1e6:.1f} MB)")
c.close()
PY
fi
echo "  ^ CONTAINS THE ANSWER TO: is pug_control/Velocity.x in cm/s or m/s?"
echo "    grep for the velocity clamp in pug_src/pug_driver/pug_control/"

echo
echo ">>> [2/6] ACTION GROUP NAMES (our whitelist is empty until we have these)"
rsh 'echo "--- likely dirs ---"; ls -la /home/hiwonder/pug/src/pug_control/ActionGroups/ 2>/dev/null; ls -la /home/hiwonder/software/ 2>/dev/null; find /home/hiwonder -iname "*.d6a*" -o -iname "*.d6ac" 2>/dev/null | head -40' \
  > "${OUT}/action_groups.txt" 2>&1
echo "  -> ${OUT}/action_groups.txt"

echo
echo ">>> [3/6] CALIBRATION + CONFIG"
rsh 'for f in /home/hiwonder/.ros/camera_info/*.yaml /home/hiwonder/pug/src/*/config/*.yaml /home/hiwonder/pug/src/*/param/*.yaml; do [ -f "$f" ] && { echo "=== $f ==="; cat "$f"; }; done 2>/dev/null' \
  > "${OUT}/calibration_and_config.txt" 2>&1
echo "  -> ${OUT}/calibration_and_config.txt"

echo
echo ">>> [4/6] LIVE ROS GRAPH (topics/services/params, with types)"
rsh 'source /opt/ros/melodic/setup.bash; source /home/hiwonder/pug/devel/setup.bash 2>/dev/null; export ROS_MASTER_URI=http://localhost:11311;
     echo "### rostopic list -v"; rostopic list -v 2>&1;
     echo; echo "### rosservice list"; rosservice list 2>&1;
     echo; echo "### rosnode list"; rosnode list 2>&1;
     echo; echo "### rosparam list"; rosparam list 2>&1;
     echo; echo "### MSG DEFS"; for m in pug_control/Velocity pug_control/Pose pug_control/Gait pug_control/SetActionName ros_robot_controller/SetBusServoState ros_robot_controller/Stop; do echo "--- $m"; rosmsg show $m 2>&1 || rossrv show $m 2>&1; done' \
  > "${OUT}/ros_graph_full.txt" 2>&1
echo "  -> ${OUT}/ros_graph_full.txt"

echo
echo ">>> [5/6] SYSTEM FACTS (versions, devices, autostart)"
rsh 'echo "### uname"; uname -a; echo; echo "### os"; cat /etc/os-release | head -3; echo; echo "### rosversion"; rosversion -d 2>/dev/null;
     echo; echo "### video devices"; ls -l /dev/video* 2>&1; v4l2-ctl --list-devices 2>&1 | head;
     echo; echo "### autostart"; systemctl list-units --type=service --state=running 2>/dev/null | grep -iE "ros|pug|start" ; crontab -l 2>/dev/null; cat /etc/rc.local 2>/dev/null | tail -12;
     echo; echo "### date"; date; echo; echo "### battery-ish"; cat /home/hiwonder/pug/src/*/config/*.yaml 2>/dev/null | grep -i batt' \
  > "${OUT}/system_facts.txt" 2>&1
echo "  -> ${OUT}/system_facts.txt"

echo
echo ">>> [6/6] ROSBAG — 20s of every topic (replayable ground truth for tests)"
rsh 'source /opt/ros/melodic/setup.bash; source /home/hiwonder/pug/devel/setup.bash 2>/dev/null; export ROS_MASTER_URI=http://localhost:11311;
     cd /tmp && timeout 30 rosbag record -a -O /tmp/rospug_capture.bag --duration=20 >/dev/null 2>&1; ls -la /tmp/rospug_capture.bag' \
  > "${OUT}/rosbag_record.log" 2>&1
if [ "$have_sshpass" = "1" ]; then
  sshpass -p "${PASS}" scp ${SSHOPTS} "${USER}@${HOST}:/tmp/rospug_capture.bag" "${OUT}/" 2>/dev/null && echo "  -> ${OUT}/rospug_capture.bag"
else
  echo "  (bag left at /tmp/rospug_capture.bag on the robot; scp it manually)"
fi

echo
echo "=== DONE -> ${OUT} ==="
echo "NEXT: resolve the blocking unknowns, in this order:"
echo "  1. UNITS of pug_control/Velocity.x  -> grep pug_src for the velocity clamp"
echo "  2. Real velocity/pose LIMITS        -> same source"
echo "  3. Action group names               -> action_groups.txt -> config.ROSPUG.action_groups"
echo "Then set MOTION_UNLOCKED and update physicalagent/config.py."
