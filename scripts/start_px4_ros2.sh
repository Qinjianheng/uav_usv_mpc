#!/usr/bin/env bash
# shellcheck disable=SC1090,SC1091
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
PX4_ROOT="${PX4_ROOT:-/home/qin/Projects/PX4-Autopilot}"
OCEAN_WORLD="${WS_ROOT}/src/uav_usv_bringup/worlds/ocean.sdf"
PX4_GZ_ENV="${PX4_ROOT}/build/px4_sitl_default/rootfs/gz_env.sh"
CUSTOM_GZ_MODELS="${WS_ROOT}/src/uav_usv_bringup/models"
QGC_APPIMAGE="${QGC_APPIMAGE:-/home/qin/桌面/QGroundControl-x86_64.AppImage}"
BUILD_WORKSPACE=true
CAMERA_STARTUP_TIMEOUT="${CAMERA_STARTUP_TIMEOUT:-180}"
FLIGHT_READY_TIMEOUT="${FLIGHT_READY_TIMEOUT:-60}"
PX4_VERTICAL_SPEED_LIMIT="${PX4_VERTICAL_SPEED_LIMIT:-4.0}"
EXPERIMENT_LAUNCH="${UAV_USV_EXPERIMENT_LAUNCH:-modular_intercept.launch.py}"
EXPERIMENT_CONFIG_FILE="${UAV_USV_EXPERIMENT_CONFIG_FILE:-}"
EXPERIMENT_CONFIG_ARG=""
EXPERIMENT_ENABLE_EVALUATOR="${UAV_USV_ENABLE_EVALUATOR:-true}"
EXPERIMENT_ENABLE_SHADOW="${UAV_USV_ENABLE_SHADOW_PERCEPTION:-true}"
EXPERIMENT_ENABLE_DOWN="${UAV_USV_ENABLE_DOWN_CAMERA:-false}"
EXPERIMENT_FRONT_DEPTH_MODEL="${UAV_USV_FRONT_DEPTH_MODEL:-tof}"

if [[ "${1:-}" == "--no-build" ]]; then
    BUILD_WORKSPACE=false
elif [[ $# -gt 0 ]]; then
    echo "Usage: $0 [--no-build]" >&2
    exit 2
fi

if [[ ! -d "${WS_ROOT}/src" ]]; then
    echo "Workspace not found: ${WS_ROOT}" >&2
    exit 1
fi

if [[ ! -d "${PX4_ROOT}" ]]; then
    echo "PX4 source directory not found: ${PX4_ROOT}" >&2
    exit 1
fi

if [[ ! -f "${OCEAN_WORLD}" ]]; then
    echo "Ocean world not found: ${OCEAN_WORLD}" >&2
    exit 1
fi

if ! [[ "${CAMERA_STARTUP_TIMEOUT}" =~ ^[1-9][0-9]*$ ]]; then
    echo "CAMERA_STARTUP_TIMEOUT must be a positive integer." >&2
    exit 2
fi
if ! [[ "${FLIGHT_READY_TIMEOUT}" =~ ^[1-9][0-9]*$ ]]; then
    echo "FLIGHT_READY_TIMEOUT must be a positive integer." >&2
    exit 2
fi
if ! [[ "${PX4_VERTICAL_SPEED_LIMIT}" =~ ^([1-9][0-9]*([.][0-9]+)?|0[.][0-9]*[1-9][0-9]*)$ ]]; then
    echo "PX4_VERTICAL_SPEED_LIMIT must be a positive number." >&2
    exit 2
fi
if ! [[ "${EXPERIMENT_LAUNCH}" =~ ^[A-Za-z0-9_.-]+[.]launch[.]py$ ]]; then
    echo "UAV_USV_EXPERIMENT_LAUNCH must be a launch filename." >&2
    exit 2
fi
for experiment_boolean in "${EXPERIMENT_ENABLE_EVALUATOR}" \
    "${EXPERIMENT_ENABLE_SHADOW}" "${EXPERIMENT_ENABLE_DOWN}"; do
    if [[ "${experiment_boolean}" != "true" && "${experiment_boolean}" != "false" ]]; then
        echo "Evaluator, shadow and down-camera switches must be true or false." >&2
        exit 2
    fi
done
if [[ "${EXPERIMENT_FRONT_DEPTH_MODEL}" != "tof" ]] \
    && [[ "${EXPERIMENT_FRONT_DEPTH_MODEL}" != "ideal" ]]; then
    echo "UAV_USV_FRONT_DEPTH_MODEL must be tof or ideal." >&2
    exit 2
fi
if [[ -n "${EXPERIMENT_CONFIG_FILE}" ]]; then
    if [[ ! -f "${EXPERIMENT_CONFIG_FILE}" ]]; then
        echo "Experiment config file not found: ${EXPERIMENT_CONFIG_FILE}" >&2
        exit 2
    fi
    printf -v EXPERIMENT_CONFIG_ARG ' config_file:=%q' \
        "${EXPERIMENT_CONFIG_FILE}"
fi

for required_command in gnome-terminal gz MicroXRCEAgent timeout rg; do
    if ! command -v "${required_command}" >/dev/null; then
        echo "Required command not found: ${required_command}" >&2
        exit 1
    fi
done

if ! pgrep -f '[Q]GroundControl' >/dev/null; then
    if [[ ! -x "${QGC_APPIMAGE}" ]]; then
        echo "QGroundControl is not running and its AppImage was not found:" >&2
        echo "  ${QGC_APPIMAGE}" >&2
        echo "Set QGC_APPIMAGE to the executable path and retry." >&2
        exit 1
    fi

    echo "Starting QGroundControl..."
    qgc_log="${TMPDIR:-/tmp}/uav_usv_qgroundcontrol.log"
    nohup "${QGC_APPIMAGE}" >"${qgc_log}" 2>&1 &

    qgc_ready=false
    for _ in {1..15}; do
        if pgrep -f '[Q]GroundControl' >/dev/null; then
            qgc_ready=true
            break
        fi
        sleep 1
    done

    if ! ${qgc_ready}; then
        echo "QGroundControl did not start. See ${qgc_log}" >&2
        exit 1
    fi
fi

if timeout 3s gz topic -e -t /world/default/clock -n 1 \
    >/dev/null 2>&1; then
    echo "A Gazebo world is already running." >&2
    echo "Close the existing Gazebo/PX4 session, then retry." >&2
    exit 1
fi

# Gazebo Transport can retain a stopped world's topic discovery records for a
# short time.  If no live clock is present, a PX4 process left by an aborted
# launch is orphaned and must not attach to the next Gazebo instance.
PX4_EXECUTABLE="$(readlink -f \
    "${PX4_ROOT}/build/px4_sitl_default/bin/px4" 2>/dev/null || true)"
if [[ -n "${PX4_EXECUTABLE}" ]]; then
    mapfile -t stale_px4_pids < <(
        pgrep -f "^${PX4_EXECUTABLE}([[:space:]]|$)" || true
    )
    if [[ ${#stale_px4_pids[@]} -gt 0 ]]; then
        echo "Stopping orphaned PX4 SITL from an earlier launch..."
        kill -INT "${stale_px4_pids[@]}" 2>/dev/null || true
        for _ in {1..10}; do
            px4_still_running=false
            for stale_pid in "${stale_px4_pids[@]}"; do
                if kill -0 "${stale_pid}" 2>/dev/null; then
                    px4_still_running=true
                    break
                fi
            done
            if ! ${px4_still_running}; then
                break
            fi
            sleep 0.2
        done
        if ${px4_still_running}; then
            echo "Orphaned PX4 SITL did not stop cleanly." >&2
            echo "Close its PX4 SITL terminal, then retry." >&2
            exit 1
        fi
    fi
fi

source /opt/ros/humble/setup.bash

if ${BUILD_WORKSPACE}; then
    "${WS_ROOT}/scripts/build_workspace.sh"
elif [[ ! -f "${WS_ROOT}/install/setup.bash" ]]; then
    echo "Workspace has not been built; omit --no-build for the first run." >&2
    exit 1
fi

source "${WS_ROOT}/install/setup.bash"
export UAV_USV_WS="${WS_ROOT}"

if [[ ! -f "${PX4_GZ_ENV}" ]]; then
    echo "Preparing PX4 SITL build and Gazebo environment..."
    (
        cd "${PX4_ROOT}"
        make px4_sitl_default
    )
fi

if [[ ! -f "${PX4_GZ_ENV}" ]]; then
    echo "PX4 Gazebo environment not found: ${PX4_GZ_ENV}" >&2
    exit 1
fi

source "${PX4_GZ_ENV}"
LAB_SESSION_DIR="$(mktemp -d "${TMPDIR:-/tmp}/uav_usv_lab.XXXXXX")"
LAB_RESTART_MARKER="${LAB_SESSION_DIR}/restarting"
LAB_GZ_SERVER_CONFIG="${LAB_SESSION_DIR}/magnetometer_enu.config"
LAB_CAMERA_MODELS="${LAB_SESSION_DIR}/models"
python3 "${WS_ROOT}/scripts/prepare_gz_camera_model.py" \
    --source "${CUSTOM_GZ_MODELS}/x500_mono_cam" \
    --output "${LAB_CAMERA_MODELS}/x500_mono_cam" \
    --enable-down-camera "${EXPERIMENT_ENABLE_DOWN}"
export GZ_SIM_RESOURCE_PATH="${LAB_CAMERA_MODELS}:${CUSTOM_GZ_MODELS}:${GZ_SIM_RESOURCE_PATH:-}"
export PX4_GZ_MODELS="${LAB_CAMERA_MODELS}"
echo "Down camera enabled: ${EXPERIMENT_ENABLE_DOWN}"
echo "Front depth model: ${EXPERIMENT_FRONT_DEPTH_MODEL}"

# The native bridge and Gazebo field must select the same coordinates. Fail
# before launching either component if only one half of the fix is installed.
if ! LC_ALL=C grep -aFq 'PX4_GZ_MAGNETOMETER_ENU' \
    "${PX4_ROOT}/build/px4_sitl_default/bin/px4"; then
    echo "PX4 needs the workspace ENU magnetometer bridge patch and rebuild." >&2
    echo "See patches/px4/gz_magnetometer_enu.patch and the follow repair report." >&2
    exit 1
fi
python3 "${WS_ROOT}/scripts/prepare_gz_magnetometer_config.py" \
    --source "${PX4_ROOT}/src/modules/simulation/gz_bridge/server.config" \
    --output "${LAB_GZ_SERVER_CONFIG}"

PX4_ENU_INIT="${PX4_ROOT}/build/px4_sitl_default/rootfs/etc/init.d-posix/px4-rc.gzmag_enu"
if ! cmp -s "${WS_ROOT}/patches/px4/px4-rc.gzmag_enu" "${PX4_ENU_INIT}" \
    || ! grep -Fq '. px4-rc.gzmag_enu || exit 1' \
        "${PX4_ROOT}/build/px4_sitl_default/rootfs/etc/init.d-posix/rcS"; then
    echo "PX4 needs the ENU zero-bias simulated magnetometer startup hook." >&2
    echo "See patches/px4/gz_magnetic_initialization.patch and the repair report." >&2
    exit 1
fi
PX4_GNSS_INIT="${PX4_ROOT}/build/px4_sitl_default/rootfs/etc/init.d-posix/px4-rc.gzgnss"
if ! cmp -s "${WS_ROOT}/patches/px4/px4-rc.gzgnss" "${PX4_GNSS_INIT}" \
    || ! grep -Fq '. px4-rc.gzgnss || exit 1' \
        "${PX4_ROOT}/build/px4_sitl_default/rootfs/etc/init.d-posix/rcS"; then
    echo "PX4 needs the Gazebo no-delay GNSS startup hook and rebuild." >&2
    echo "See patches/px4/gz_gnss_initialization.patch and px4-rc.gzgnss." >&2
    exit 1
fi
# Preserve imported parameters before the selected simulator profile
# initializes magnetic offsets and GNSS delay. Do not reset the parameter DB.
PX4_PARAMETER_BACKUP_ROOT="${WS_ROOT}/data/experiments/px4_parameter_backups"
mkdir -p "${PX4_PARAMETER_BACKUP_ROOT}"
PX4_PARAMETER_BACKUP_DIR="$(mktemp -d "${PX4_PARAMETER_BACKUP_ROOT}/enu_XXXXXX")"
for parameter_file in parameters.bson parameters_backup.bson; do
    if [[ -f "${PX4_ROOT}/build/px4_sitl_default/rootfs/${parameter_file}" ]]; then
        cp -- "${PX4_ROOT}/build/px4_sitl_default/rootfs/${parameter_file}" \
            "${PX4_PARAMETER_BACKUP_DIR}/${parameter_file}"
    fi
done
echo "PX4 parameter backup: ${PX4_PARAMETER_BACKUP_DIR}"

stop_lab_component()
{
    local component_name="$1"
    local pid_file="${LAB_SESSION_DIR}/${component_name}.pid"
    local component_pid=""
    local component_pgid=""
    local command_console_pgid=""

    if [[ ! -r "${pid_file}" ]]; then
        echo "Cannot restart: missing ${component_name} session PID." >&2
        return 1
    fi
    read -r component_pid <"${pid_file}"
    if ! [[ "${component_pid}" =~ ^[1-9][0-9]*$ ]]; then
        echo "Cannot restart: invalid ${component_name} session PID." >&2
        return 1
    fi
    if ! kill -0 "${component_pid}" 2>/dev/null; then
        return 0
    fi

    component_pgid="$(
        ps -o pgid= -p "${component_pid}" | tr -d '[:space:]'
    )"
    command_console_pgid="$(
        ps -o pgid= -p "${BASHPID}" | tr -d '[:space:]'
    )"
    if ! [[ "${component_pgid}" =~ ^[1-9][0-9]*$ ]] \
        || [[ "${component_pgid}" == "${command_console_pgid}" ]]; then
        echo "Cannot safely stop ${component_name} process group." >&2
        return 1
    fi

    echo "Stopping ${component_name}..."
    kill -INT -- "-${component_pgid}" 2>/dev/null
}

lab_components_stopped()
{
    local component_name=""
    local component_pid=""
    local pid_file=""

    for component_name in experiment dds px4 gazebo; do
        pid_file="${LAB_SESSION_DIR}/${component_name}.pid"
        if [[ -r "${pid_file}" ]]; then
            read -r component_pid <"${pid_file}"
            if [[ "${component_pid}" =~ ^[1-9][0-9]*$ ]] \
                && kill -0 "${component_pid}" 2>/dev/null; then
                return 1
            fi
        fi
    done
    return 0
}

restart_lab()
{
    local component_name=""
    local restart_ready=false

    : >"${LAB_RESTART_MARKER}"
    for component_name in experiment dds px4 gazebo; do
        if ! stop_lab_component "${component_name}"; then
            echo "Restart aborted; close the experiment windows manually." >&2
            rm -f "${LAB_RESTART_MARKER}"
            return 1
        fi
    done

    for _ in {1..50}; do
        if lab_components_stopped \
            && ! timeout 1s gz topic -e -t /world/default/clock -n 1 \
                >/dev/null 2>&1; then
            restart_ready=true
            break
        fi
        sleep 0.2
    done
    if ! ${restart_ready}; then
        echo "Restart aborted: a Gazebo/PX4 component is still running." >&2
        echo "Close the remaining simulation windows, then rerun uav_lab.sh." >&2
        rm -f "${LAB_RESTART_MARKER}"
        return 1
    fi

    rm -f \
        "${LAB_RESTART_MARKER}" \
        "${LAB_GZ_SERVER_CONFIG}" \
        "${LAB_SESSION_DIR}/experiment.pid" \
        "${LAB_SESSION_DIR}/dds.pid" \
        "${LAB_SESSION_DIR}/px4.pid" \
        "${LAB_SESSION_DIR}/gazebo.pid"
    rm -rf -- "${LAB_CAMERA_MODELS}"
    rmdir "${LAB_SESSION_DIR}" 2>/dev/null || true
    echo "Previous simulation stopped. Starting a clean session..."
    exec "${BASH_SOURCE[0]}" --no-build
}

echo "Starting Gazebo ocean world..."
gnome-terminal --title="Gazebo Ocean" -- bash -lc "
printf '%s\n' \"\${BASHPID}\" > '${LAB_SESSION_DIR}/gazebo.pid' &&
source '${PX4_GZ_ENV}' &&
export GZ_SIM_SERVER_CONFIG_PATH='${LAB_GZ_SERVER_CONFIG}' &&
export GZ_SIM_RESOURCE_PATH='${LAB_CAMERA_MODELS}':'${CUSTOM_GZ_MODELS}':\${GZ_SIM_RESOURCE_PATH:-} &&
gz sim -r '${OCEAN_WORLD}';
component_status=\$?;
if [[ ! -f '${LAB_RESTART_MARKER}' ]]; then exec bash; fi;
exit \${component_status}"

echo "Waiting for Gazebo ocean world..."
gazebo_ready=false
for _ in {1..30}; do
    gazebo_services="$(gz service -l 2>/dev/null || true)"
    if grep -Fxq '/world/default/create' <<< "${gazebo_services}"; then
        gazebo_ready=true
        break
    fi
    sleep 1
done

if ! ${gazebo_ready}; then
    echo "Gazebo ocean world did not become ready within 30 seconds." >&2
    exit 1
fi

echo "Starting PX4 SITL in standalone Gazebo mode..."
gnome-terminal --title="PX4 SITL" -- bash -lc "
printf '%s\n' \"\${BASHPID}\" > '${LAB_SESSION_DIR}/px4.pid' &&
source '${PX4_GZ_ENV}' &&
export GZ_SIM_SERVER_CONFIG_PATH='${LAB_GZ_SERVER_CONFIG}' &&
export PX4_GZ_MAGNETOMETER_ENU=1 &&
export PX4_GZ_GNSS_NO_DELAY=1 &&
export GZ_SIM_RESOURCE_PATH='${LAB_CAMERA_MODELS}':'${CUSTOM_GZ_MODELS}':\${GZ_SIM_RESOURCE_PATH:-} &&
export PX4_GZ_MODELS='${LAB_CAMERA_MODELS}' &&
export PX4_GZ_STANDALONE=1 &&
export PX4_GZ_WORLD=default &&
# Face Gazebo +Y (local NED north), where the USV starts 8 m away.
export PX4_GZ_MODEL_POSE='0,0,0,0,0,1.57079632679' &&
cd '${PX4_ROOT}' &&
make px4_sitl gz_x500_mono_cam;
component_status=\$?;
if [[ ! -f '${LAB_RESTART_MARKER}' ]]; then exec bash; fi;
exit \${component_status}"

echo "Waiting up to ${CAMERA_STARTUP_TIMEOUT} seconds for PX4 and camera streams..."
camera_topics_ready=false
gazebo_topics=""
for ((elapsed = 0; elapsed < CAMERA_STARTUP_TIMEOUT; elapsed++)); do
    gazebo_topics="$(timeout 3s gz topic -l 2>/dev/null || true)"
    if grep -Fxq '/uav/camera/front/image' <<< "${gazebo_topics}" \
        && grep -Fxq '/uav/camera/front/depth_image' \
            <<< "${gazebo_topics}"; then
        if [[ "${EXPERIMENT_ENABLE_DOWN}" == "false" ]] \
            || { grep -Fxq '/uav/camera/down/image' <<< "${gazebo_topics}" \
                && grep -Fxq '/uav/camera/down/depth_image' <<< "${gazebo_topics}"; }; then
            camera_topics_ready=true
            break
        fi
    fi
    if ((elapsed > 0 && elapsed % 15 == 0)); then
        echo "Still waiting for PX4/cameras (${elapsed}s)..."
    fi
    sleep 1
done

if ! ${camera_topics_ready}; then
    echo "Required camera topics were not available within" \
        "${CAMERA_STARTUP_TIMEOUT} seconds." >&2
    echo "Expected front image/depth, plus down streams only when enabled." >&2
    echo "Available camera-related topics:" >&2
    rg -i 'camera|image|depth' <<< "${gazebo_topics}" >&2 || true
    echo "Check the PX4 SITL terminal for model-spawn errors." >&2
    exit 1
fi

echo "Required RGB/depth camera streams are available."

# PX4's stock x500 descent limit is 1.5 m/s. Keep its velocity controller
# aligned with the ROS terminal envelope so a 4 m/s by 4 m/s flight path can
# reach 45 degrees. Takeoff remains independently limited by baseline.yaml.
PX4_PARAM_TOOL="${PX4_ROOT}/build/px4_sitl_default/bin/px4-param"
if [[ ! -x "${PX4_PARAM_TOOL}" ]]; then
    echo "PX4 parameter tool not found: ${PX4_PARAM_TOOL}" >&2
    exit 1
fi
echo "Configuring PX4 vertical speed envelope to" \
    "${PX4_VERTICAL_SPEED_LIMIT} m/s..."
"${PX4_PARAM_TOOL}" --instance 0 set \
    MPC_Z_VEL_MAX_UP "${PX4_VERTICAL_SPEED_LIMIT}"
"${PX4_PARAM_TOOL}" --instance 0 set \
    MPC_Z_VEL_MAX_DN "${PX4_VERTICAL_SPEED_LIMIT}"

echo "Starting Micro XRCE-DDS Agent..."
gnome-terminal --title="Micro XRCE-DDS Agent" -- bash -lc "
printf '%s\n' \"\${BASHPID}\" > '${LAB_SESSION_DIR}/dds.pid' &&
MicroXRCEAgent udp4 -p 8888;
component_status=\$?;
if [[ ! -f '${LAB_RESTART_MARKER}' ]]; then exec bash; fi;
exit \${component_status}"

echo "Waiting 5 seconds for DDS connection..."
sleep 5

echo "Starting UAV-USV experiment..."
echo "Experiment launch: ${EXPERIMENT_LAUNCH}"
if [[ -n "${EXPERIMENT_CONFIG_FILE}" ]]; then
    echo "Experiment config: ${EXPERIMENT_CONFIG_FILE}"
fi
gnome-terminal --title="UAV-USV experiment" -- bash -lc "
printf '%s\n' \"\${BASHPID}\" > '${LAB_SESSION_DIR}/experiment.pid' &&
export UAV_USV_WS='${WS_ROOT}' &&
source /opt/ros/humble/setup.bash &&
source '${WS_ROOT}/install/setup.bash' &&
cd '${WS_ROOT}' &&
ros2 launch uav_usv_bringup '${EXPERIMENT_LAUNCH}' \
    enable_evaluator:=${EXPERIMENT_ENABLE_EVALUATOR} \
    enable_shadow_perception:=${EXPERIMENT_ENABLE_SHADOW} \
    front_depth_model:=${EXPERIMENT_FRONT_DEPTH_MODEL} \
    enable_down_camera:=${EXPERIMENT_ENABLE_DOWN}${EXPERIMENT_CONFIG_ARG};
component_status=\$?;
if [[ ! -f '${LAB_RESTART_MARKER}' ]]; then exec bash; fi;
exit \${component_status}"

echo "Waiting up to ${FLIGHT_READY_TIMEOUT} seconds for OFFBOARD ground hold..."
flight_ready=false
flight_ready_start_seconds=${SECONDS}
next_flight_ready_status_seconds=10
while ((SECONDS - flight_ready_start_seconds < FLIGHT_READY_TIMEOUT)); do
    ready_sample="$(
        # Do not use the long-lived ROS 2 CLI daemon here.  A daemon whose
        # rclpy context was invalidated returns "!rclpy.ok()" for every graph
        # query, making a healthy controller look permanently unready.
        timeout 3s ros2 topic echo --once --no-daemon --spin-time 1 \
            /simulation/impact/flight_ready \
            std_msgs/msg/Bool 2>/dev/null || true
    )"
    if rg -q 'data: true' <<< "${ready_sample}"; then
        flight_ready=true
        break
    fi
    flight_ready_elapsed=$((SECONDS - flight_ready_start_seconds))
    if ((flight_ready_elapsed >= next_flight_ready_status_seconds)); then
        echo "Still preparing PX4 OFFBOARD/arming" \
            "(${flight_ready_elapsed}s)..."
        next_flight_ready_status_seconds=$((
            next_flight_ready_status_seconds + 10
        ))
    fi
    sleep 1
done

if ! ${flight_ready}; then
    echo "UAV did not report flight readiness within" \
        "${FLIGHT_READY_TIMEOUT} seconds." >&2
    echo "Inspect the UAV-USV experiment and PX4 terminals." >&2
    exit 1
fi

publish_command()
{
    # Both moving_target and mission_manager_node must receive X.  The ROS 2
    # CLI otherwise publishes as soon as it discovers the first subscriber,
    # so a short-lived publisher can start the UAV while the target misses the
    # same command.  Wait for both subscribers and repeat the reliable sample
    # before letting the CLI publisher disappear.  Y is harmlessly ignored by
    # moving_target, but the same gate guarantees it reaches the controller.
    if ! timeout 8s ros2 topic pub \
        --times 3 \
        --rate 10 \
        --print 3 \
        --wait-matching-subscriptions 2 \
        --keep-alive 0.5 \
        /simulation/impact/command \
        std_msgs/msg/String \
        "{data: '$1'}"; then
        echo "Command $1 was not delivered to both experiment nodes." >&2
        echo "Inspect the UAV-USV experiment terminal, then retry." >&2
        return 1
    fi
}

echo
echo "Two-stage control is ready; PX4 is disarmed in OFFBOARD ground hold."
echo "  X: start UAV takeoff and USV motion simultaneously"
echo "  Y: start interception from visually locked FOLLOW"
echo "  R: stop this simulation and restart a clean session"
echo "  Q: leave this command console"

while true; do
    read -r -p "Command [X/Y/R/Q]: " command
    command=${command^^}

    case "${command}" in
        X)
            publish_command X
            ;;
        Y)
            publish_command Y
            ;;
        R)
            restart_lab
            ;;
        Q)
            echo "Command console closed; experiment terminals remain open."
            break
            ;;
        *)
            echo "Please enter X, Y, R, or Q."
            ;;
    esac
done
