// SPDX-License-Identifier: GPL-2.0
#include <linux/can/dev.h>
#include <linux/can/skb.h>
#include <linux/module.h>

/*
 * drivers/net/can/dev/rx-offload.c calls __can_get_echo_skb(), but that
 * helper is intentionally private to the built-in CAN device core.  The
 * board kernel was built without CAN_RX_OFFLOAD, so a standalone copy of
 * rx-offload.c cannot link against the private helper.
 *
 * Keep this implementation in lockstep with Linux 6.6.101
 * drivers/net/can/dev/skb.c::__can_get_echo_skb().  The copied
 * rx-offload.c is patched at build time to call this namespaced helper.
 */
struct sk_buff *
can_rx_offload_shim_get_echo_skb(struct net_device *dev, unsigned int idx,
				 unsigned int *len_ptr,
				 unsigned int *frame_len_ptr)
{
	struct can_priv *priv = netdev_priv(dev);

	if (idx >= priv->echo_skb_max) {
		netdev_err(dev,
			   "%s: echo_skb index out of bounds (%u/max %u)\n",
			   __func__, idx, priv->echo_skb_max);
		return NULL;
	}

	if (priv->echo_skb[idx]) {
		struct sk_buff *skb = priv->echo_skb[idx];
		struct can_skb_priv *can_skb_priv = can_skb_prv(skb);

		if (skb_shinfo(skb)->tx_flags & SKBTX_IN_PROGRESS)
			skb_tstamp_tx(skb, skb_hwtstamps(skb));

		*len_ptr = can_skb_get_data_len(skb);
		if (frame_len_ptr)
			*frame_len_ptr = can_skb_priv->frame_len;

		priv->echo_skb[idx] = NULL;

		if (skb->pkt_type == PACKET_LOOPBACK) {
			skb->pkt_type = PACKET_BROADCAST;
		} else {
			dev_consume_skb_any(skb);
			return NULL;
		}

		return skb;
	}

	return NULL;
}

static int __init can_rx_offload_shim_init(void)
{
	return 0;
}

static void __exit can_rx_offload_shim_exit(void)
{
}

module_init(can_rx_offload_shim_init);
module_exit(can_rx_offload_shim_exit);

MODULE_DESCRIPTION("Standalone CAN RX offload prerequisite for gs_usb");
MODULE_LICENSE("GPL");
