#!/system/bin/sh
set -eu

APP_ROOT=${APP_ROOT:-/data/local/piper_dual_bringup}
SIDE=${SIDE:-left}
CAN_INTERFACE=${CAN_INTERFACE:-piper1}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-1}
RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
CYCLONEDDS_URI=${CYCLONEDDS_URI:-file://${APP_ROOT}/config/cyclonedds.xml}
PYTHONPATH=${APP_ROOT}/compat:${APP_ROOT}/vendor:${APP_ROOT}:${PYTHONPATH:-}

export APP_ROOT SIDE CAN_INTERFACE ROS_DOMAIN_ID RMW_IMPLEMENTATION CYCLONEDDS_URI PYTHONPATH

case "${SIDE}:${CAN_INTERFACE}" in
  left:piper1|right:piper0) ;;
  *)
    echo "ERROR: validated mapping is left:piper1 or right:piper0" >&2
    exit 19
    ;;
esac

if [ ! -d "/sys/class/net/${CAN_INTERFACE}" ]; then
  echo "ERROR: ${CAN_INTERFACE} does not exist; run board_prepare_piper_can.sh first" >&2
  exit 20
fi

can_details=$(/bin/run ip -details link show "${CAN_INTERFACE}")
echo "${can_details}" | grep -q "bitrate 1000000" || {
  echo "ERROR: ${CAN_INTERFACE} is not configured for Piper's 1000000 bit/s" >&2
  exit 21
}
echo "${can_details}" | grep -q "<.*UP.*>" || {
  echo "ERROR: ${CAN_INTERFACE} is not UP" >&2
  exit 22
}

actuation_flag=
if [ "${PIPER_ENABLE_ACTUATION:-0}" = "1" ]; then
  actuation_flag=--actuate
fi

exec /bin/run python3 "${APP_ROOT}/piper_bridge.py" \
  --side "${SIDE}" \
  --can-interface "${CAN_INTERFACE}" \
  --speed-percent 10 \
  ${actuation_flag}
