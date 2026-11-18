#!/usr/bin/env bash
# Run the TRLC-DK1 scanning demo in Docker. See README.md.
#   ./run_robot_docker.sh            # combo with photo stops (default)
#   ./run_robot_docker.sh --dry-run
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
IMAGE=trlc-dk1-demo

# Build the image the first time (takes a minute or two).
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "Building the Docker image (first run only)..."
  docker build -t "$IMAGE" - < Dockerfile
fi

# Find the robot's USB port from --port, or use /dev/ttyACM0.
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
    echo "Robot not found at $port." >&2
    echo "Plug in the arm, find its port with:  ls /dev/ttyACM* /dev/ttyUSB*" >&2
    echo "then run:  $0 --port /dev/ttyACM1 $*" >&2
    exit 1
  fi
  device=(--device "$port")
fi

tty=(-i); [[ -t 0 ]] && tty=(-it)   # Esc needs a terminal
exec docker run --rm "${tty[@]}" "${device[@]}" -v "$PWD:/app:ro" "$IMAGE" "$@"
