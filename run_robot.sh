#!/usr/bin/env bash
# Build (first time only) and run follow_trajectory.py in Docker.
#   ./run_robot.sh --port /dev/ttyACM0 --speed 0.3 --loops 1
#   ./run_robot.sh --dry-run
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
IMAGE=trlc-dk1-follow

docker image inspect "$IMAGE" >/dev/null 2>&1 || docker build -t "$IMAGE" .

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

device=()
if [[ $dry == 0 ]]; then
  if [[ ! -e $port ]]; then
    echo "Robot port $port not found. Plug in the arm or pass --port (see: ls /dev/ttyACM* /dev/ttyUSB*)." >&2
    exit 1
  fi
  device=(--device "$port")
fi

exec docker run --rm -it "${device[@]}" -v "$PWD:/app:ro" "$IMAGE" "$@"
