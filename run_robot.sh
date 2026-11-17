#!/usr/bin/env bash
# Run follow_trajectory.py on the real robot from a local virtual environment.
# The first run creates .venv-robot/ in this folder; nothing is installed system-wide.
#   ./run_robot.sh --port /dev/ttyACM0 --speed 0.3 --loops 1
#   ./run_robot.sh --dry-run
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
VENV=.venv-robot

if [[ ! -f third_party/trlc-dk1/trlc_dk1_control/robot.py ]]; then
  git submodule update --init third_party/trlc-dk1
fi

if [[ ! -x $VENV/bin/python ]] || ! cmp -s requirements-robot.txt "$VENV/.installed"; then
  echo "Setting up $VENV (first run only)..."
  python3 -m venv "$VENV"
  "$VENV/bin/pip" install -q --upgrade pip
  "$VENV/bin/pip" install -q -r requirements-robot.txt
  cp requirements-robot.txt "$VENV/.installed"
fi

port=/dev/ttyACM0
dry=0
args=("$@")
for i in "${!args[@]}"; do
  case "${args[$i]}" in
    --port) port="${args[$((i + 1))]}" ;;
    --port=*) port="${args[$i]#--port=}" ;;
    --dry-run) dry=1 ;;
  esac
done
if [[ $dry == 0 && ! -e $port ]]; then
  echo "Robot port $port not found. Plug in the arm or pass --port (see: ls /dev/ttyACM* /dev/ttyUSB*)." >&2
  exit 1
fi
if [[ $dry == 0 && ! -w $port ]]; then
  echo "No permission for $port. Run: sudo usermod -aG dialout \$USER  (then log out and back in)." >&2
  exit 1
fi

PYTHONPATH="$PWD/third_party/trlc-dk1${PYTHONPATH:+:$PYTHONPATH}" exec "$VENV/bin/python" follow_trajectory.py "$@"
