#!/usr/bin/env bash
# shellcheck disable=SC1090,SC1091
set -eo pipefail

SCRIPT_DIR="$(
    cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
    pwd -P
)"

WS_ROOT="$(
    cd -- "${SCRIPT_DIR}/.."
    pwd -P
)"

export UAV_USV_WS="${WS_ROOT}"

LAB_BASELINE=false
LAB_NO_BUILD=false
for lab_option in "$@"; do
    case "${lab_option}" in
        --baseline) LAB_BASELINE=true ;;
        --no-build) LAB_NO_BUILD=true ;;
        -h|--help)
            echo "Usage: $0 [--no-build] [--baseline]"
            echo "Default: isolated P7 capture; 1-5 select only, X starts motion/FOLLOW, Y capture."
            echo "--no-build: use the existing workspace build."
            echo "--baseline: preserve the previous baseline/custom-launch backend."
            exit 0
            ;;
        *) echo "Unknown option: ${lab_option}" >&2; exit 2 ;;
    esac
done

echo "=================================================="
echo "UAV-USV one-click launcher"
echo "Workspace:"
echo "  ${UAV_USV_WS}"

if git -C "${UAV_USV_WS}" rev-parse \
    --is-inside-work-tree >/dev/null 2>&1; then
    echo "Branch:"
    echo "  $(git -C "${UAV_USV_WS}" branch --show-current)"
    echo "Commit:"
    echo "  $(git -C "${UAV_USV_WS}" rev-parse --short HEAD)"
fi

echo "=================================================="

LAB_BACKEND_ARGS=()
if ${LAB_NO_BUILD}; then
    LAB_BACKEND_ARGS+=(--no-build)
fi
# Existing experiment drivers set their launch explicitly and call this
# wrapper again. Preserve that route so P7 cannot recursively start itself.
if ${LAB_BASELINE} || [[ -n "${UAV_USV_EXPERIMENT_LAUNCH:-}" ]]; then
    exec "${WS_ROOT}/scripts/start_px4_ros2.sh" "${LAB_BACKEND_ARGS[@]}"
fi

source /opt/ros/humble/setup.bash
if ! ${LAB_NO_BUILD}; then
    "${WS_ROOT}/scripts/build_workspace.sh"
fi
if [[ ! -f "${WS_ROOT}/install/setup.bash" ]]; then
    echo "Workspace has not been built; omit --no-build for the first run." >&2
    exit 1
fi
source "${WS_ROOT}/install/setup.bash"

mkdir -p "${WS_ROOT}/data/experiments"
LAB_EXPERIMENT_DIR="${WS_ROOT}/data/experiments/p7_manual_$(date +%Y%m%d_%H%M%S_%N)_${BASHPID}"
echo "Starting P7 capture; 1-5 only select USV motion, the USV stays still until X."
echo "Experiment: ${LAB_EXPERIMENT_DIR}"
echo "Wait for FLIGHT READY, then X; send Y after stable MINCO_FOLLOW."
exec python3 "${WS_ROOT}/scripts/p7_follow_sitl.py" \
    "${LAB_EXPERIMENT_DIR}" --manual --capture
