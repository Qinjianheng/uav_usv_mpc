#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(
    cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
    pwd -P
)"
DEFAULT_WS_ROOT="$(
    cd -- "${SCRIPT_DIR}/.."
    pwd -P
)"
WS_ROOT="${UAV_USV_WS:-${DEFAULT_WS_ROOT}}"

if [[ ! -d "${WS_ROOT}/src" ]]; then
    echo "Workspace source directory not found: ${WS_ROOT}/src" >&2
    exit 1
fi

source /opt/ros/humble/setup.bash
cd "${WS_ROOT}"

echo "Building ROS 2 workspace: ${WS_ROOT}"
colcon build --base-paths "${WS_ROOT}/src" --symlink-install "$@"

echo
echo "Build complete. To use this shell, run:"
echo "source ${WS_ROOT}/install/setup.bash"
