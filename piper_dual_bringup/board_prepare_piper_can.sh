#!/system/bin/sh
set -eu

APP_ROOT=${APP_ROOT:-/data/local/piper_dual_bringup}
KERNEL_RELEASE=$(uname -r)
KERNEL_DIR=${KERNEL_DIR:-${APP_ROOT}/kernel/artifacts/${KERNEL_RELEASE}}
SERIAL_CONFIG=${SERIAL_CONFIG:-${APP_ROOT}/config/piper_can_serials.env}
REQUIRE_BOTH=${REQUIRE_BOTH:-0}

. "${SERIAL_CONFIG}"
: "${PIPER1_USB_SERIAL:?PIPER1_USB_SERIAL must be configured}"
PIPER0_USB_SERIAL=${PIPER0_USB_SERIAL:-}

for module in can_rx_offload_shim.ko gs_usb.ko; do
  if [ ! -f "${KERNEL_DIR}/${module}" ]; then
    echo "ERROR: missing ${KERNEL_DIR}/${module} for kernel ${KERNEL_RELEASE}" >&2
    exit 9
  fi
done

if ! grep -q '^can_rx_offload_shim ' /proc/modules; then
  insmod "${KERNEL_DIR}/can_rx_offload_shim.ko"
fi
if ! grep -q '^can_rx_offload_shim ' /proc/modules; then
  echo "ERROR: can_rx_offload_shim was rejected; inspect dmesg" >&2
  exit 10
fi

if ! grep -q '^gs_usb ' /proc/modules; then
  insmod "${KERNEL_DIR}/gs_usb.ko"
fi
if ! grep -q '^gs_usb ' /proc/modules; then
  echo "ERROR: gs_usb was rejected; inspect dmesg" >&2
  exit 11
fi

interface_serial()
{
  cat "/sys/class/net/$1/device/../serial" 2>/dev/null || true
}

find_interface_by_serial()
{
  wanted=$1
  for path in /sys/class/net/*; do
    [ -e "${path}" ] || continue
    name=${path##*/}
    actual=$(interface_serial "${name}")
    if [ -n "${actual}" ] && [ "${actual}" = "${wanted}" ]; then
      echo "${name}"
      return 0
    fi
  done
  return 1
}

report_usb_can_inventory()
{
  echo "Detected candleLight CAN interfaces:" >&2
  found=0
  for path in /sys/class/net/*; do
    [ -e "${path}" ] || continue
    name=${path##*/}
    serial=$(interface_serial "${name}")
    [ -n "${serial}" ] || continue
    product=$(cat "${path}/device/../product" 2>/dev/null || true)
    echo "  ${name} serial=${serial} product=${product}" >&2
    found=1
  done
  if [ "${found}" = "0" ]; then
    echo "  none" >&2
  fi

  echo "USB device topology:" >&2
  for device in /sys/bus/usb/devices/*; do
    [ -f "${device}/idVendor" ] || continue
    vendor=$(cat "${device}/idVendor")
    product_id=$(cat "${device}/idProduct")
    product=$(cat "${device}/product" 2>/dev/null || true)
    serial=$(cat "${device}/serial" 2>/dev/null || true)
    echo "  ${device##*/} ${vendor}:${product_id} serial=${serial} product=${product}" >&2
  done
}

prepare_one()
{
  logical_name=$1
  expected_serial=$2

  if [ -z "${expected_serial}" ]; then
    if [ "${REQUIRE_BOTH}" = "1" ]; then
      echo "ERROR: ${logical_name} serial is not configured" >&2
      report_usb_can_inventory
      exit 20
    fi
    echo "INFO: ${logical_name} is not configured yet; skipping"
    return 0
  fi

  if [ -e "/sys/class/net/${logical_name}" ]; then
    current_serial=$(interface_serial "${logical_name}")
    if [ "${current_serial}" != "${expected_serial}" ]; then
      echo "ERROR: ${logical_name} belongs to unexpected serial ${current_serial}" >&2
      exit 21
    fi
  else
    source_name=$(find_interface_by_serial "${expected_serial}") || {
      echo "ERROR: no candleLight interface with serial ${expected_serial} for ${logical_name}" >&2
      report_usb_can_inventory
      exit 22
    }
    /bin/run ip link set "${source_name}" down
    /bin/run ip link set "${source_name}" name "${logical_name}"
  fi

  details=$(/bin/run ip -details link show "${logical_name}")
  if ! echo "${details}" | grep -q 'bitrate 1000000'; then
    /bin/run ip link set "${logical_name}" down 2>/dev/null || true
    /bin/run ip link set "${logical_name}" type can bitrate 1000000
  fi
  /bin/run ip link set "${logical_name}" up
  echo "READY ${logical_name} serial=${expected_serial}"
  /bin/run ip -details -statistics link show "${logical_name}"
}

# physical_ai_runtime site mapping: left=piper1, right=piper0.
prepare_one piper1 "${PIPER1_USB_SERIAL}"
prepare_one piper0 "${PIPER0_USB_SERIAL}"

# The SoC-native can0 is deliberately not used as a Piper interface.
if [ -e /sys/class/net/can0 ]; then
  /bin/run ip link set can0 down 2>/dev/null || true
fi
