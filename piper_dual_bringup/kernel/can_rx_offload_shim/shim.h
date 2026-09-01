// SPDX-License-Identifier: GPL-2.0
#ifndef CAN_RX_OFFLOAD_SHIM_H
#define CAN_RX_OFFLOAD_SHIM_H

#include <linux/netdevice.h>
#include <linux/skbuff.h>

struct sk_buff *
can_rx_offload_shim_get_echo_skb(struct net_device *dev, unsigned int idx,
				 unsigned int *len_ptr,
				 unsigned int *frame_len_ptr);

#endif
