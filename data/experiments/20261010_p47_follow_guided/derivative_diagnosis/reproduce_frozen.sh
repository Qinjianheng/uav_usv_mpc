#!/usr/bin/env bash
set -eo pipefail
artifact_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
frozen_code_dir="$(mktemp -d /tmp/p47_frozen_code.XXXXXX)"
reproduced_output="${1:-$(mktemp -d /tmp/p47_reproduced.XXXXXX)}"
if [[ $# -gt 0 ]]; then shift; fi
tar -xzf "$artifact_dir/code_baseline_353c3e1.tar.gz" -C "$frozen_code_dir"
ln -s /home/qin/data/uav_usv_mpc/data "$frozen_code_dir/data"
cp "$artifact_dir/analysis_source_used.py" "$frozen_code_dir/scripts/p47_derivative_diagnosis.py"
cd "$frozen_code_dir"
export PYTHONPATH="$frozen_code_dir/src/uav_control:$frozen_code_dir/scripts"
export OPENBLAS_NUM_THREADS=1
python3 scripts/p47_derivative_diagnosis.py --output "$reproduced_output" "$@"
printf 'Frozen code: %s\nReproduction: %s\n' "$frozen_code_dir" "$reproduced_output"
