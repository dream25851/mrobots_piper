#!/system/bin/sh
set -eu

ETH_INTERFACE=eth0
BOARD_CIDR=192.168.1.19/24

if [ ! -d "/sys/class/net/${ETH_INTERFACE}" ]; then
  echo "ERROR: ${ETH_INTERFACE} does not exist" >&2
  exit 30
fi

/bin/run ip link set "${ETH_INTERFACE}" up

if ! /bin/run ip -4 address show dev "${ETH_INTERFACE}" | grep -q '192\.168\.1\.19/24'; then
  /bin/run ip address add "${BOARD_CIDR}" dev "${ETH_INTERFACE}"
fi

/bin/run ip -br address show dev "${ETH_INTERFACE}"
/bin/run ip route show dev "${ETH_INTERFACE}"
