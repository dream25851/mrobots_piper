#!/bin/bash
set -euo pipefail

APP_ROOT=${APP_ROOT:-/home/alpha/mrobot/piper_dual_bringup}
RUNTIME_ROOT=${RUNTIME_ROOT:-/home/alpha/physical_ai_runtime}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-1}
CYCLONEDDS_URI=${CYCLONEDDS_URI:-file://${RUNTIME_ROOT}/.config/cyclonedds_default.xml}

export ROS_DOMAIN_ID
cd "${RUNTIME_ROOT}"
set +u
source "${RUNTIME_ROOT}/install/setup.bash"
set -u

duration=0
relay_args=()
while [ "$#" -gt 0 ]; do
  case "$1" in
    --duration)
      [ "$#" -ge 2 ] || { echo "ERROR: --duration requires a value" >&2; exit 2; }
      duration=$2
      shift 2
      ;;
    --duration=*)
      duration=${1#--duration=}
      shift
      ;;
    *)
      relay_args+=("$1")
      shift
      ;;
  esac
done

relay_pid=
engaged=0

set_leader_preempt()
{
  enabled=$1
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  CYCLONEDDS_URI="${CYCLONEDDS_URI}" \
  PYTHONPATH="${APP_ROOT}:${PYTHONPATH:-}" \
    timeout 25 python "${APP_ROOT}/leader_preempt_control.py" --state "${enabled}"
}

cleanup()
{
  trap - INT TERM EXIT
  if [ "${engaged}" = "1" ]; then
    set +e
    set_leader_preempt false
    set -e
  fi
  if [ -n "${relay_pid}" ]; then
    kill -INT "${relay_pid}" 2>/dev/null || true
    wait "${relay_pid}" 2>/dev/null || true
  fi
}
trap cleanup INT TERM EXIT

RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
PYTHONPATH="${APP_ROOT}:${PYTHONPATH:-}" \
  python "${APP_ROOT}/leader_remote_teleop.py" "${relay_args[@]}" &
relay_pid=$!

# Let the Fast DDS relay discover both board bridges before the Leaders begin
# streaming.  If prerequisites are missing, the relay fails instead of
# engaging 0-G mode without a command path.
sleep 2
if ! kill -0 "${relay_pid}" 2>/dev/null; then
  wait "${relay_pid}"
  exit $?
fi

engaged=1
set_leader_preempt true
if [ "${duration}" = "0" ] || [ "${duration}" = "0.0" ]; then
  wait "${relay_pid}"
else
  sleep "${duration}"
  kill -INT "${relay_pid}" 2>/dev/null || true
  wait "${relay_pid}" 2>/dev/null || true
fi
