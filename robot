#!/usr/bin/env bash
# Launch the PhysicalAgent operator console against the ROSPug.
#
#   ./robot                     interactive console (default)
#   ./robot doctor              preflight only
#   ./robot status              telemetry dump
#   ./robot run "..."           one-shot task
#   ./robot --robot tonypi      console against another robot in the registry
#   ./robot doctor --robot ...  any subcommand takes --robot too
#
# Override the robot's address with ROSPUG_HOST=... ./robot
set -euo pipefail
cd "$(dirname "$0")"

export ROSPUG_HOST="${ROSPUG_HOST:-rospug.local}"
# Set to 0 for the timid 0.05 m/s caps. Note this can only ever make motion
# MORE conservative: whether a robot is unlocked at all is now a per-robot
# decision carried by its RobotProfile (profile.motion_unlocked), because this
# line runs before anyone knows which robot is being launched.
export PHYSICALAGENT_MOTION_UNLOCKED="${PHYSICALAGENT_MOTION_UNLOCKED:-1}"

# argparse requires the subcommand first, so a bare `./robot --robot rospug`
# used to die with "invalid choice: 'rospug'" even though the design doc, the
# plan and the CLI's own --help all show exactly that form. The old
# "${@:-shell}" default only applied with ZERO arguments. Insert the implied
# `shell` whenever the first argument is a flag -- EXCEPT -h/--help, which
# must reach the top-level parser so `./robot --help` shows
# `usage: physicalagent [-h] {doctor,status,probe,run,shell} ...` and not
# the shell subparser's own usage. An operator who forgot the subcommand
# names is exactly who needs to see that list.
args=("$@")
if [ ${#args[@]} -eq 0 ]; then
    args=(shell)
elif [[ ${args[0]} == "-h" || ${args[0]} == "--help" ]]; then
    : # leave args as-is -- falls through to the top-level parser
elif [[ ${args[0]} == -* ]]; then
    args=(shell "${args[@]}")
fi

# Interpreter, in precedence order: an explicit override (used by the test that
# exercises this wrapper's argument shaping), the project venv, then whatever
# python3 is on PATH -- so a fresh clone runs before `python -m venv .venv`.
PY="${PHYSICALAGENT_PYTHON:-}"
[ -n "$PY" ] || PY=.venv/bin/python
[ -x "$PY" ] || PY="$(command -v python3)"
exec "$PY" -m physicalagent.cli "${args[@]}"
