#!/bin/bash
set -euo pipefail

HDC=${HDC:-/home/alpha/mrobot/toolchains/hdc}
HDC_TARGET=${HDC_TARGET:-192.168.1.19:8710}
BOARD_ROOT=${BOARD_ROOT:-/data/local/piper_dual_bringup}
PIPER_ENABLE_ACTUATION=${PIPER_ENABLE_ACTUATION:-1}
APP_ROOT=${APP_ROOT:-/home/alpha/mrobot/piper_dual_bringup}
RUNTIME_ROOT=${RUNTIME_ROOT:-/home/alpha/physical_ai_runtime}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-1}
CYCLONEDDS_URI=${CYCLONEDDS_URI:-file://${RUNTIME_ROOT}/.config/cyclonedds_default.xml}
COMPAT_PID_FILE=${COMPAT_PID_FILE:-${APP_ROOT}/.controller_manager_compat.pid}
COMPAT_LOG=${COMPAT_LOG:-${APP_ROOT}/controller_manager_compat.log}
PIPER_CONTROLLER_MANAGER=${PIPER_CONTROLLER_MANAGER:-/rk3588_piper/controller_manager}

if [ ! -x "${HDC}" ]; then
  echo "ERROR: HDC executable not found: ${HDC}" >&2
  exit 2
fi

board_shell()
{
  "${HDC}" -t "${HDC_TARGET}" shell "$1"
}

board_path_state=$(board_shell "if [ -x '${BOARD_ROOT}/board_start_dual.sh' ]; then echo READY; else echo MISSING; fi" | tr -d '\r[:space:]')
if [ "${board_path_state}" != "READY" ]; then
  echo "ERROR: board deployment is missing: ${BOARD_ROOT}/board_start_dual.sh" >&2
  exit 3
fi

running_processes=$(board_shell "ps -ef | grep -E '[b]oard_start_dual.sh|[p]iper_bridge.py' || true")
if [ -n "${running_processes//[$'\r\n\t ']/}" ]; then
  echo "Dual-arm board service is already running on ${HDC_TARGET}."
  printf '%s\n' "${running_processes}"
else
  echo "Preparing serial-bound piper1/piper0 interfaces on ${HDC_TARGET}..."
  board_shell "REQUIRE_BOTH=1 '${BOARD_ROOT}/board_prepare_piper_can.sh'"

  echo "Starting dual-arm board service with actuation=${PIPER_ENABLE_ACTUATION}..."
  board_shell "cd '${BOARD_ROOT}'; setsid sh -c 'PIPER_ENABLE_ACTUATION=${PIPER_ENABLE_ACTUATION} exec ./board_start_dual.sh > dual.log 2>&1' < /dev/null > /dev/null 2>&1 & echo \$! > dual.pid"
  sleep 5

  running_processes=$(board_shell "ps -ef | grep -E '[b]oard_start_dual.sh|[p]iper_bridge.py' || true")
  if [ -z "${running_processes//[$'\r\n\t ']/}" ]; then
    echo "ERROR: dual-arm board service exited during startup" >&2
    board_shell "tail -n 100 '${BOARD_ROOT}/dual.log'" || true
    exit 4
  fi
fi

bridge_count=$(board_shell "ps -ef | grep '[p]iper_bridge.py' | wc -l" | tr -d '\r[:space:]')
if [ "${bridge_count}" -ne 2 ]; then
  echo "ERROR: expected two piper bridges, found ${bridge_count}" >&2
  board_shell "ps -ef | grep -E '[b]oard_start_dual.sh|[p]iper_bridge.py'" || true
  board_shell "tail -n 100 '${BOARD_ROOT}/dual.log'" || true
  exit 5
fi

echo "Dual-arm board service is ready: left=piper1, right=piper0."
board_shell "ps -ef | grep -E '[b]oard_start_dual.sh|[p]iper_bridge.py'"

compat_pid=
if [ -f "${COMPAT_PID_FILE}" ]; then
  compat_pid=$(tr -cd '0-9' < "${COMPAT_PID_FILE}")
fi
if [ -n "${compat_pid}" ] && kill -0 "${compat_pid}" 2>/dev/null \
  && ps -p "${compat_pid}" -o args= | grep -q '[c]ontroller_manager_compat.py'; then
  echo "Controller-manager compatibility service is already running (pid=${compat_pid})."
else
  if [ ! -x "${RUNTIME_ROOT}/.pixi/envs/runtime/bin/python" ]; then
    echo "ERROR: physical_ai_runtime environment is missing under ${RUNTIME_ROOT}" >&2
    exit 6
  fi
  echo "Starting local controller-manager compatibility service..."
  setsid bash -c \
    "source '${RUNTIME_ROOT}/install/setup.bash'; exec env ROS_DOMAIN_ID='${ROS_DOMAIN_ID}' RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI='${CYCLONEDDS_URI}' PIPER_CONTROLLER_MANAGER='${PIPER_CONTROLLER_MANAGER}' '${RUNTIME_ROOT}/.pixi/envs/runtime/bin/python' '${APP_ROOT}/controller_manager_compat.py'" \
    > "${COMPAT_LOG}" 2>&1 < /dev/null &
  compat_pid=$!
  printf '%s\n' "${compat_pid}" > "${COMPAT_PID_FILE}"
  sleep 3
  if ! kill -0 "${compat_pid}" 2>/dev/null; then
    echo "ERROR: controller-manager compatibility service exited" >&2
    tail -n 100 "${COMPAT_LOG}" >&2 || true
    exit 7
  fi
  echo "Controller-manager compatibility service is ready at ${PIPER_CONTROLLER_MANAGER} (pid=${compat_pid})."
fi
