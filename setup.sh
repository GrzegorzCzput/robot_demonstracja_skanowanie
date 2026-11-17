#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
CUROBO_SOURCE="${CUROBO_SOURCE:-/home/jan/Documents/grip/curobo}"
if [[ ! -f "$CUROBO_SOURCE/curobo/motion_planner.py" ]]; then
  echo 'Set CUROBO_SOURCE to a cuRobo 0.8 checkout exposing curobo.motion_planner.' >&2
  exit 1
fi
if [[ ! -x .venv/bin/python ]]; then
  uv venv --python 3.13 .venv
fi
uv pip install --python .venv/bin/python -r requirements.txt --extra-index-url https://download.pytorch.org/whl/cu130
uv pip install --python .venv/bin/python --no-deps -e "$CUROBO_SOURCE"
# Official robot model and driver, pinned as a git submodule.
git submodule update --init third_party/trlc-dk1
mkdir -p outputs
.venv/bin/python build_robot.py
