#!/usr/bin/env bash
set -Ee -o pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set +u
source /opt/ros/jazzy/setup.bash
source "${SCRIPT_DIR}/../install/setup.bash"
set -u
exec python3 "${SCRIPT_DIR}/j14_demo_launcher.py" --mode feedforward --hardware-mode active "$@"
