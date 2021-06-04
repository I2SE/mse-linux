/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright 2021 in-tech smart charging GmbH
 */

#ifndef __MSE102X_H__
#define __MSE102X_H__

struct mse102x_rxctrl {
	u16	mchash[4];
	u16	rxcr1;
	u16	rxcr2;
};

union mse102x_tx_hdr {
	u8	txb[6];
	__le16	txw[3];
};

struct mse102x_net {
	struct net_device	*netdev;
	spinlock_t		statelock;

	union mse102x_tx_hdr	txh ____cacheline_aligned;
	u8			rxd[8];
	u8			txd[8];

	u32			msg_enable ____cacheline_aligned;
	u16			tx_space;
	u8			fid;

	struct sk_buff_head	txq;

	void			(*lock)(struct mse102x_net *mse,
					unsigned long *flags);
	void			(*unlock)(struct mse102x_net *mse,
					  unsigned long *flags);
	unsigned int		(*rdreg16)(struct mse102x_net *mse,
					   unsigned int reg);
	void			(*wrreg16)(struct mse102x_net *mse,
					   unsigned int reg, unsigned int val);
	void			(*rdfifo)(struct mse102x_net *mse, u8 *buff,
					  unsigned int len);
	void			(*wrfifo)(struct mse102x_net *mse,
					  struct sk_buff *txp, bool irq);
	netdev_tx_t		(*start_xmit)(struct sk_buff *skb,
					      struct net_device *dev);
	void			(*rx_skb)(struct mse102x_net *mse,
					  struct sk_buff *skb);
	void			(*flush_tx_work)(struct mse102x_net *mse);
};

int mse102x_probe_common(struct net_device *netdev, struct device *dev,
			int msg_en);
int mse102x_remove_common(struct device *dev);
int mse102x_suspend(struct device *dev);
int mse102x_resume(struct device *dev);

static __maybe_unused SIMPLE_DEV_PM_OPS(mse102x_pm_ops,
					mse102x_suspend, mse102x_resume);

static void __maybe_unused mse102x_done_tx(struct mse102x_net *mse,
					  struct sk_buff *txb)
{
	struct net_device *dev = mse->netdev;

	dev->stats.tx_bytes += txb->len;
	dev->stats.tx_packets++;

	dev_kfree_skb(txb);
}

#endif /* __MSE102X_H__ */
