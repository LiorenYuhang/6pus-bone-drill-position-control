#!/usr/bin/env bash
set -Ee -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${WS_ROOT}/.." && pwd)"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
SETUP_FILE="${WS_ROOT}/install/setup.bash"
LAUNCHER="${REPO_ROOT}/bone_ws/src/h30_imu/scripts/j11_staged_launcher.py"
RUNNER="${REPO_ROOT}/bone_ws/src/h30_imu/scripts/j11_base_feedforward_runner.py"
HOME_RUNNER="${REPO_ROOT}/bone_ws/scripts/run_j7_servo_home_park.py"
SOURCE_CONFIG="${REPO_ROOT}/bone_ws/src/h30_imu/config/j11_feedforward_params.yaml"
INSTALL_CONFIG="${REPO_ROOT}/bone_ws/install/h30_imu/share/h30_imu/config/j11_feedforward_params.yaml"
ACTIVE_MARKER="/tmp/j11_active_signaled"
DEMO_DURATION_S="${J11_DEMO_DURATION_S:-86400}"

bootstrap_environment() {
  if [[ -n "${ROS_DISTRO:-}" && "${ROS_DISTRO}" != "jazzy" ]]; then
    echo "ERROR: incompatible ROS_DISTRO detected: ${ROS_DISTRO}" >&2
    echo "Expected: jazzy" >&2
    return 1
  fi

  if [[ ! -r "${ROS_SETUP}" ]]; then
    echo "ERROR: ROS 2 Jazzy setup not found:" >&2
    echo "${ROS_SETUP}" >&2
    return 1
  fi

  if [[ ! -r "${SETUP_FILE}" ]]; then
    echo "ERROR: workspace install/setup.bash not found." >&2
    echo "Build ${WS_ROOT} before running this demo." >&2
    return 1
  fi

  # Colcon-generated setup files may read unset variables such as COLCON_TRACE.
  # Source both environments before enabling nounset in this wrapper.
  set +u
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"
  # shellcheck disable=SC1090
  source "${SETUP_FILE}"
  set -u

  if [[ "${ROS_DISTRO:-}" != "jazzy" ]]; then
    echo "ERROR: ROS 2 Jazzy environment bootstrap failed." >&2
    return 1
  fi
}

software_check() {
  local package path
  for path in "${LAUNCHER}" "${RUNNER}" "${HOME_RUNNER}" "${SOURCE_CONFIG}" "${INSTALL_CONFIG}"; do
    if [[ ! -r "${path}" ]]; then
      echo "[SOFTWARE CHECK FAIL] Missing or unreadable: ${path}" >&2
      return 1
    fi
  done
  if ! cmp -s "${SOURCE_CONFIG}" "${INSTALL_CONFIG}"; then
    echo "[SOFTWARE CHECK FAIL] Source/install feedforward configs differ." >&2
    return 1
  fi
  if ! command -v ros2 >/dev/null 2>&1; then
    echo "[SOFTWARE CHECK FAIL] ros2 is not available after environment bootstrap." >&2
    return 1
  fi
  for package in h30_imu pus6_motor_driver pus6_control; do
    if ! ros2 pkg prefix "${package}" >/dev/null 2>&1; then
      echo "[SOFTWARE CHECK FAIL] Workspace package is not visible: ${package}" >&2
      return 1
    fi
  done
  echo "[SOFTWARE CHECK PASS] J11 feedforward demo paths and configs are ready."
  echo "Repo root: ${REPO_ROOT}"
  echo "Workspace root: ${WS_ROOT}"
  echo "ROS setup: PASS"
  echo "ROS_DISTRO: ${ROS_DISTRO}"
  echo "Workspace setup: PASS"
  echo "Demo mode: software-check"
  echo "Hardware writes: 0"
  echo "hardware_write_count=0"
}

bootstrap_environment

if [[ "${1:-}" == "--software-check" ]]; then
  software_check
  exit $?
fi

if pgrep -f 'j11_staged_launcher.py|j11_base_feedforward_runner.py|j11_m2_staged_launcher.py|j11_m2_pid_residual_runner.py|pus6_system.launch.py|/pus6_motor_driver/motor_driver_node|h30_imu_node' >/dev/null; then
  echo "[DEMO ABORT] A J11 or required ROS hardware process is already active." >&2
  exit 2
fi

for marker in /tmp/j11_active_signaled /tmp/j11_ready_for_start /tmp/j11_stop_requested /tmp/j11_start_signal; do
  [[ ! -e "${marker}" ]] || unlink "${marker}"
done

cd "${REPO_ROOT}"

LAUNCHER_PID=""
ACTIVE_SEEN=0

cleanup() {
  local original_status="${1:-0}"
  local home_status=0
  trap - EXIT INT TERM
  set +e

  if [[ -n "${LAUNCHER_PID}" ]] && kill -0 "${LAUNCHER_PID}" 2>/dev/null; then
    : > /tmp/j11_stop_requested
    kill -INT "${LAUNCHER_PID}" 2>/dev/null
    wait "${LAUNCHER_PID}" 2>/dev/null
  fi

  for marker in /tmp/j11_active_signaled /tmp/j11_ready_for_start /tmp/j11_stop_requested /tmp/j11_start_signal; do
    [[ ! -e "${marker}" ]] || unlink "${marker}"
  done

  if [[ "${ACTIVE_SEEN}" -eq 1 ]]; then
    echo "[DEMO CLEANUP] Returning all six axes to OPERATING_HOME..."
    python3 "${HOME_RUNNER}" \
      --action home \
      --real-hardware \
      --confirm-motion
    home_status=$?
  fi

  echo "J11 Feedforward Demo stopped."
  if [[ "${ACTIVE_SEEN}" -eq 1 && "${home_status}" -eq 0 ]]; then
    echo "OPERATING_HOME restored."
  elif [[ "${ACTIVE_SEEN}" -eq 1 ]]; then
    echo "OPERATING_HOME recovery FAILED; inspect the HOME runner output before any further motion." >&2
  else
    echo "CONTROL ACTIVE was never reached; no Demo motion recovery was requested."
  fi
  echo "Hardware output stopped."

  if [[ "${home_status}" -ne 0 ]]; then
    return "${home_status}"
  fi
  return "${original_status}"
}

trap 'status=$?; cleanup "${status}"; status=$?; exit "${status}"' EXIT
trap 'exit 130' INT TERM

echo "[J11 DEMO] Starting frozen Base-IMU SO(3) Feedforward path."
python3 "${LAUNCHER}" \
  --duration "${DEMO_DURATION_S}" \
  --stage j11_demo_feedforward \
  --config-file "${SOURCE_CONFIG}" \
  --execute-motion \
  "$@" &
LAUNCHER_PID=$!

while kill -0 "${LAUNCHER_PID}" 2>/dev/null; do
  if [[ "${ACTIVE_SEEN}" -eq 0 && -e "${ACTIVE_MARKER}" ]]; then
    ACTIVE_SEEN=1
    echo "CONTROL ACTIVE"
    echo "Base disturbance may begin only under the Task 2 operator gate."
  fi
  sleep 0.1
done

set +e
wait "${LAUNCHER_PID}"
status=$?
set -e
LAUNCHER_PID=""
exit "${status}"
