#!/bin/bash
set -euo pipefail

HDC=${HDC:-/home/alpha/mrobot/toolchains/hdc}
HDC_TARGET=${HDC_TARGET:-192.168.1.19:8710}
BOARD_ROOT=${BOARD_ROOT:-/data/local/piper_dual_bringup}
APP_ROOT=${APP_ROOT:-/home/alpha/mrobot/piper_dual_bringup}
COMPAT_PID_FILE=${COMPAT_PID_FILE:-${APP_ROOT}/.controller_manager_compat.pid}

compat_pid=
if [ -f "${COMPAT_PID_FILE}" ]; then
  compat_pid=$(tr -cd '0-9' < "${COMPAT_PID_FILE}")
fi
if [ -n "${compat_pid}" ] && kill -0 "${compat_pid}" 2>/dev/null \
  && ps -p "${compat_pid}" -o args= | grep -q '[c]ontroller_manager_compat.py'; then
  echo "Stopping RK3588S controller-manager compatibility service (pid=${compat_pid})..."
  kill -INT "${compat_pid}"
  for _ in $(seq 1 30); do
    kill -0 "${compat_pid}" 2>/dev/null || break
    sleep 0.2
  done
  if kill -0 "${compat_pid}" 2>/dev/null; then
    echo "ERROR: compatibility service did not stop" >&2
    exit 2
  fi
else
  echo "RK3588S controller-manager compatibility service is not running."
fi

if [ ! -x "${HDC}" ]; then
  echo "ERROR: HDC executable not found: ${HDC}" >&2
  exit 3
fi

"${HDC}" -t "${HDC_TARGET}" shell \
  "pid=\$(cat '${BOARD_ROOT}/dual.pid' 2>/dev/null || true); if [ -n \"\${pid}\" ] && kill -0 \"\${pid}\" 2>/dev/null; then kill -INT \"\${pid}\"; i=0; while kill -0 \"\${pid}\" 2>/dev/null; do i=\$((i+1)); [ \"\${i}\" -lt 30 ] || exit 9; sleep 1; done; echo 'Dual-arm board service stopped.'; else echo 'Dual-arm board service is not running.'; fi"

echo "RK3588S RT backend stopped; it is now safe to start the NUC RT backend."
