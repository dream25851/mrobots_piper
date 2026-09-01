#!/system/bin/sh
set -eu

APP_ROOT=${APP_ROOT:-/data/local/piper_dual_bringup}
STARTUP_STAGGER_S=${STARTUP_STAGGER_S:-2}

for interface in piper1 piper0; do
  if [ ! -d "/sys/class/net/${interface}" ]; then
    echo "ERROR: ${interface} is missing; dual-arm start aborted" >&2
    exit 30
  fi
done

left_pid=
right_pid=

cleanup()
{
  trap - INT TERM EXIT
  [ -z "${left_pid}" ] || kill -INT "${left_pid}" 2>/dev/null || true
  [ -z "${right_pid}" ] || kill -INT "${right_pid}" 2>/dev/null || true
  [ -z "${left_pid}" ] || wait "${left_pid}" 2>/dev/null || true
  [ -z "${right_pid}" ] || wait "${right_pid}" 2>/dev/null || true
}
trap cleanup INT TERM EXIT

SIDE=left CAN_INTERFACE=piper1 APP_ROOT="${APP_ROOT}" \
  "${APP_ROOT}/board_start_piper.sh" &
left_pid=$!

# This board's ROS 2 userspace occasionally fails allocator initialization if
# two independent rclpy/Fast DDS processes are created in the same instant.
# Stagger startup, and ensure the first bridge survived before opening piper0.
sleep "${STARTUP_STAGGER_S}"
if ! kill -0 "${left_pid}" 2>/dev/null; then
  set +e
  wait "${left_pid}"
  child_rc=$?
  set -e
  [ "${child_rc}" -ne 0 ] || child_rc=31
  echo "ERROR: piper1 bridge exited during startup (rc=${child_rc})" >&2
  exit "${child_rc}"
fi

SIDE=right CAN_INTERFACE=piper0 APP_ROOT="${APP_ROOT}" \
  "${APP_ROOT}/board_start_piper.sh" &
right_pid=$!

# Do not silently continue with one arm if either bridge exits.
while true; do
  if ! kill -0 "${left_pid}" 2>/dev/null; then
    set +e
    wait "${left_pid}"
    child_rc=$?
    set -e
    [ "${child_rc}" -ne 0 ] || child_rc=32
    echo "ERROR: piper1 bridge exited (rc=${child_rc}); stopping piper0" >&2
    exit "${child_rc}"
  fi
  if ! kill -0 "${right_pid}" 2>/dev/null; then
    set +e
    wait "${right_pid}"
    child_rc=$?
    set -e
    [ "${child_rc}" -ne 0 ] || child_rc=33
    echo "ERROR: piper0 bridge exited (rc=${child_rc}); stopping piper1" >&2
    exit "${child_rc}"
  fi
  sleep 1
done
