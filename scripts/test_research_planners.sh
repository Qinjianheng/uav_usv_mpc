#!/usr/bin/env bash
# Run from the package so generated build trees are never collected.
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
TASK_WS="${UAV_USV_WS:-$(cd -- "${SCRIPT_DIR}/.." && pwd -P)}"
MODE="${1:-algorithm}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
cd "${TASK_WS}/src/uav_control"
START_NS="$(date +%s%N)"
case "${MODE}" in
    algorithm)
        python3 -m pytest -q test/test_camera_visibility.py test/test_planned_attitude.py \
            test/test_minco_trajectory.py test/test_maneuvering_target_predictor.py \
            test/test_follow_research_algorithms.py test/test_fast_follow_minco.py \
            test/test_realtime_follow.py test/test_short_follow.py test/test_prediction_batch.py \
            test/test_progress_follow.py test/test_follow_contract.py \
            test/test_hypothetical_follow.py test/test_polynomial_bounds.py \
            test/test_p4_session_safety.py \
            test/test_follow_minco_experiment.py \
            -k 'not existing_rgbd_forward_inverse_and_intrinsics_agree'
        ;;
    related|full)
        source /opt/ros/humble/setup.bash
        source "${TASK_WS}/install/setup.bash"
        if [[ "${MODE}" == related ]]; then
            python3 -m pytest -q test/test_camera_visibility.py test/test_planned_attitude.py \
                test/test_minco_trajectory.py test/test_fast_minco_planner.py \
                test/test_maneuvering_target_predictor.py test/test_target_predictor_node.py \
                test/test_follow_mpc_seed.py test/test_mpc_shadow_inputs.py \
                test/test_follow_mpc_shadow_node.py test/test_follow_research_algorithms.py \
                test/test_follow_research_runtime.py test/test_fast_follow_minco.py \
                test/test_realtime_follow.py test/test_short_follow.py \
                test/test_prediction_batch.py test/test_target_prediction_engine.py \
                test/test_progress_follow.py test/test_follow_contract.py \
                test/test_hypothetical_follow.py test/test_polynomial_bounds.py \
                test/test_p4_receiver_node.py test/test_p4_session_safety.py \
                test/test_follow_minco_experiment.py
        else
            python3 -m pytest -q
        fi
        ;;
    *) echo 'Usage: test_research_planners.sh {algorithm|related|full}' >&2; exit 2 ;;
esac
END_NS="$(date +%s%N)"
echo "${MODE} wall_ms=$(( (END_NS - START_NS) / 1000000 ))"
