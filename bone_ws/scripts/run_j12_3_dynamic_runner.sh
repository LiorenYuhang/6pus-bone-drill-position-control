#!/usr/bin/env bash
set -Ee -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${WS_ROOT}/.." && pwd)"

TIMESTAMP="${J12_3_TIMESTAMP:-$(date +%Y%m%d_%H%M%S)}"
OUTPUT_DIR="${REPO_ROOT}/results/j12/j12_3_dynamic_performance/${TIMESTAMP}"
DURATION="${1:-300}"

echo "======================================================================"
echo "J12-3 Dynamic Performance Characterization Runner"
echo "Host Overcurrent Gate: 1500 mA (Production safety ceiling)"
echo "Actuator Internal 0x20: 1500 mA"
echo "Controller: J11 Frozen Baseline (Roll P, Pitch PD, Yaw PD, residual ±0.5°)"
echo "Output Directory: ${OUTPUT_DIR}"
echo "Duration: ${DURATION}s"
echo "======================================================================"

mkdir -p "${OUTPUT_DIR}"

exec "${REPO_ROOT}/bone_ws/scripts/run_j11_demo_feedforward_pd.sh" \
  --stage j12_3_dynamic_screening \
  --output-dir "${OUTPUT_DIR}" \
  --duration "${DURATION}"
