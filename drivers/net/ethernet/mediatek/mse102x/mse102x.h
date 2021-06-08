/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright 2021 in-tech smart charging GmbH
 */

#ifndef __MSE102X_H__
#define __MSE102X_H__

union mse102x_tx_hdr {
	u8	txb[6];
	__le16	txw[3];
};

struct mse102x_net {
	struct net_device	*netdev;

	union mse102x_tx_hdr	txh ____cacheline_aligned;
	u8			rxd[8];
	u8			txd[8];

	u32			msg_enable ____cacheline_aligned;
	u8			fid;

	struct sk_buff_head	txq;
};

int mse102x_probe_common(struct net_device *netdev, struct device *dev,
			int msg_en);
int mse102x_remove_common(struct device *dev);
int mse102x_suspend(struct device *dev);
int mse102x_resume(struct device *dev);

void mse102x_lock_spi(struct mse102x_net *mse, unsigned long *flags);
void mse102x_unlock_spi(struct mse102x_net *mse, unsigned long *flags);
void mse102x_rx_pkts_spi(struct mse102x_net *mse);
void mse102x_flush_tx_work_spi(struct mse102x_net *mse);
netdev_tx_t mse102x_start_xmit_spi(struct sk_buff *skb, struct net_device *dev);

static __maybe_unused SIMPLE_DEV_PM_OPS(mse102x_pm_ops,
					mse102x_suspend, mse102x_resume);

#endif /* __MSE102X_H__ */
