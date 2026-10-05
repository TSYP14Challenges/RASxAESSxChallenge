#!/usr/bin/env bash
set -euo pipefail
factory_project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
factory_python=/usr/bin/python3

usage() {
  cat <<'USAGE'
Usage: bash run.sh [demo|check|plan] [options]
  demo              Run the writer, then the gray robot (default).
  check             Run offline checks and native checks when Gazebo is installed.
  check --offline   Run checks without Gazebo.
  check --native    Require Gazebo and include native message/SDF checks.
  plan              Rebuild the routes and scene after changing configuration.
  --help            Show this help.
Demo options: --headless --writer-seconds 90 --seconds 60 --range 10
USAGE
}

[[ -x "$factory_python" ]] || { echo 'The system interpreter /usr/bin/python3 is required.' >&2; exit 1; }
if ! command -v gz >/dev/null 2>&1 && [[ -f /opt/ros/jazzy/setup.bash ]]; then
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
fi
export GZ_SIM_RESOURCE_PATH="$factory_project_dir/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export PYTHONDONTWRITEBYTECODE=1
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python

case "${1:-demo}" in
  -h|--help) usage; exit 0 ;;
  plan)
    (( $# == 1 )) || { usage >&2; exit 2; }
    "$factory_python" "$factory_project_dir/scripts/build_step1.py"
    "$factory_python" "$factory_project_dir/scripts/build_step2.py"
    "$factory_python" "$factory_project_dir/scripts/build_step3.py"
    "$factory_python" "$factory_project_dir/scripts/check_step1_preview.py" --structural-only
    "$factory_python" "$factory_project_dir/scripts/check_step2.py" --structural-only
    "$factory_python" "$factory_project_dir/scripts/check_step3.py" --structural-only
    exit 0
    ;;
  check)
    shift
    factory_native=auto
    case "${1:-}" in
      '') ;;
      --offline) factory_native=no; shift ;;
      --native) factory_native=yes; shift ;;
      *) usage >&2; exit 2 ;;
    esac
    (( $# == 0 )) || { usage >&2; exit 2; }
    if [[ "$factory_native" == yes ]] && ! command -v gz >/dev/null 2>&1; then
      echo 'Native checks need Gazebo Harmonic (gz sim 8). See README.md.' >&2
      exit 1
    fi
    for factory_check in check_step1_feedback check_step1 check_step1_preview check_step1_pose_service check_step2 check_step2_network check_step2_messages check_step2_transport check_step3; do
      "$factory_python" "$factory_project_dir/scripts/$factory_check.py"
    done
    if [[ "$factory_native" != no ]] && command -v gz >/dev/null 2>&1; then
      "$factory_python" "$factory_project_dir/scripts/check_step2_messages.py" --native
      "$factory_python" "$factory_project_dir/scripts/check_step3.py" --native
      for factory_sdf in "$factory_project_dir"/worlds/*.sdf "$factory_project_dir/models/aisle_robot/model.sdf" "$factory_project_dir/step2/entrance_blue_beacon.sdf" "$factory_project_dir"/step3/*.sdf; do
        gz sdf -k "$factory_sdf"
      done
    elif [[ "$factory_native" == auto ]]; then
      echo 'Offline checks passed. Install Gazebo to include native message and SDF checks.'
    fi
    exit 0
    ;;
  demo)
    if (( $# )); then shift; fi
    ;;
  *) usage >&2; exit 2 ;;
esac
command -v gz >/dev/null 2>&1 || { echo 'Gazebo Harmonic (gz sim 8) was not found. See README.md.' >&2; exit 1; }
export GZ_PARTITION="factory_environment_simulation_${USER:-user}_$$"
exec "$factory_python" "$factory_project_dir/scripts/launch_step3.py" "$@"
