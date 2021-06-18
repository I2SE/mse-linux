// SPDX-License-Identifier: GPL-2.0-only
/* Copyright 2021 in-tech smart charging GmbH
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#define DEBUG

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/cache.h>

#include <linux/spi/spi.h>
#include <linux/of_net.h>

#include "mse102x.h"

static int msg_enable;

struct mse102x_net_spi {
	struct mse102x_net	mse102x;
	struct mutex		lock;
	struct work_struct	tx_work;
	struct spi_device	*spidev;
	struct spi_message	spi_msg1;
	struct spi_message	spi_msg2;
	struct spi_transfer	spi_xfer1;
	struct spi_transfer	spi_xfer2[2];
};

#define to_mse102x_spi(mse) container_of((mse), struct mse102x_net_spi, mse102x)

#define DET_CMD		0x0001
#define DET_SOF		0x0002
#define DET_DFT		0x55AA

#define CMD_SHIFT	12
#define CMD_RTS		(0x1 << CMD_SHIFT)
#define CMD_CTR		(0x2 << CMD_SHIFT)

#define CMD_MASK	GENMASK(15, CMD_SHIFT)
#define LEN_MASK	GENMASK(CMD_SHIFT - 1, 0)

#define	DET_CMD_LEN	4
#define	DET_SOF_LEN	2
#define	DET_DFT_LEN	2

void mse102x_lock_spi(struct mse102x_net *mse, unsigned long *flags)
{
	struct mse102x_net_spi *mses = to_mse102x_spi(mse);

	mutex_lock(&mses->lock);
}

void mse102x_unlock_spi(struct mse102x_net *mse, unsigned long *flags)
{
	struct mse102x_net_spi *mses = to_mse102x_spi(mse);

	mutex_unlock(&mses->lock);
}

/* SPI register read/write calls.
 *
 * All these calls issue SPI transactions to access the chip's registers. They
 * all require that the necessary lock is held to prevent accesses when the
 * chip is busy transferring packet data.
 */

static void mse102x_tx_cmd_spi(struct mse102x_net *mse, u16 cmd)
{
	struct mse102x_net_spi *mses = to_mse102x_spi(mse);
	struct spi_transfer *xfer = &mses->spi_xfer1;
	struct spi_message *msg = &mses->spi_msg1;
	__be16 txb[2];
	int ret;

	txb[0] = cpu_to_be16(DET_CMD);
	txb[1] = cpu_to_be16(cmd);

	xfer->tx_buf = txb;
	xfer->rx_buf = NULL;
	xfer->len = 4;

	ret = spi_sync(mses->spidev, msg);
	if (ret < 0)
		netdev_err(mse->netdev, "spi_sync() failed\n");
}

static int mse102x_rx_cmd_spi(struct mse102x_net *mse, u8 *rxb)
{
	struct mse102x_net_spi *mses = to_mse102x_spi(mse);
	struct spi_transfer *xfer = &mses->spi_xfer1;
	struct spi_message *msg = &mses->spi_msg1;
	__be16 *txb = (__be16 *)mse->txd;
	__be16 *cmd = (__be16 *)mse->rxd;
	u8 *trx = mse->rxd;
	int ret;

	txb[0] = 0;
	txb[1] = 0;

	xfer->tx_buf = txb;
	xfer->rx_buf = trx;
	xfer->len = 4;

	ret = spi_sync(mses->spidev, msg);
	if (ret < 0) {
		netdev_err(mse->netdev, "read: spi_sync() failed\n");
	} else if (*cmd != cpu_to_be16(DET_CMD)) {
		netdev_warn(mse->netdev, "%s: Unexpected response (0x%04x)\n",
					 __func__, *cmd);
		ret = -EIO;
	} else {
		memcpy(rxb, trx + 2, 2);
	}

	return ret;
}

static int mse102x_tx_frame_spi(struct mse102x_net *mse, struct sk_buff *txp)
{
	struct mse102x_net_spi *mses = to_mse102x_spi(mse);
	struct spi_transfer *xfer = &mses->spi_xfer1;
	struct spi_message *msg = &mses->spi_msg1;
	struct sk_buff *tskb;
	u8 pad_len = 0;
	u8 *ptmp;
	int ret;

	netif_dbg(mse, tx_queued, mse->netdev, "%s: skb %p, %d@%p\n",
		  __func__, txp, txp->len, txp->data);

	if (txp->len < 60)
		pad_len = 60 - txp->len;

	if ((skb_headroom(txp) < DET_SOF_LEN) ||
	    (skb_tailroom(txp) < DET_DFT_LEN + pad_len)) {
		tskb = skb_copy_expand(txp, DET_SOF_LEN, DET_DFT_LEN + pad_len, GFP_KERNEL);
		if (!tskb)
			return -ENOMEM;

		dev_kfree_skb(txp);
		txp = tskb;
	}

	ptmp = skb_push(txp, DET_SOF_LEN);
	*ptmp = (DET_SOF >> 8) & 0xFF;
	ptmp++;
	*ptmp = DET_SOF & 0xFF;

	if (pad_len) {
		ptmp = skb_put_zero(txp, pad_len);
	}

	ptmp = skb_put(txp, DET_DFT_LEN);
	*ptmp = (DET_DFT >> 8) & 0xFF;
	ptmp++;
	*ptmp = DET_DFT & 0xFF;

	xfer->tx_buf = txp->data;
	xfer->rx_buf = NULL;
	xfer->len = txp->len;

	ret = spi_sync(mses->spidev, msg);
	if (ret < 0)
		netdev_err(mse->netdev, "%s: spi_sync() failed\n", __func__);

	return ret;
}

static int mse102x_rx_frame_spi(struct mse102x_net *mse, u8 *buff, unsigned int len)
{
	struct mse102x_net_spi *mses = to_mse102x_spi(mse);
	struct spi_transfer *xfer = &mses->spi_xfer1;
	struct spi_message *msg = &mses->spi_msg1;
	int ret;

	xfer->rx_buf = buff;
	xfer->tx_buf = NULL;
	xfer->len = len;

	ret = spi_sync(mses->spidev, msg);
	if (ret < 0)
		netdev_err(mse->netdev, "%s: spi_sync() failed\n", __func__);

	return ret;
}

static void mse102x_dbg_dumpkkt(struct mse102x_net *mse, u8 *rxpkt)
{
	netdev_dbg(mse->netdev,
		   "pkt %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x\n",
		   rxpkt[4], rxpkt[5], rxpkt[6], rxpkt[7],
		   rxpkt[8], rxpkt[9], rxpkt[10], rxpkt[11],
		   rxpkt[12], rxpkt[13], rxpkt[14], rxpkt[15]);
}

void mse102x_rx_pkts_spi(struct mse102x_net *mse)
{
	struct sk_buff *skb;
	unsigned long flags;
	unsigned int rxalign;
	unsigned int rxlen;
	u8 *rxpkt;
	__be16 rx = 0;
	u16 cmd_resp;
	int ret;

	mse102x_lock_spi(mse, &flags);

	mse102x_tx_cmd_spi(mse, CMD_CTR);
	ret = mse102x_rx_cmd_spi(mse, (u8 *)&rx);
	cmd_resp = be16_to_cpu(rx);

	if (ret || ((cmd_resp & CMD_MASK) != CMD_RTS)) {
		usleep_range(50, 100);

		mse102x_tx_cmd_spi(mse, CMD_CTR);
		ret = mse102x_rx_cmd_spi(mse, (u8 *)&rx);
		cmd_resp = be16_to_cpu(rx);
		if (ret) {
			netdev_err(mse->netdev, "%s: Failed to receive (%d)\n",
						__func__, ret);
			goto unlock_spi;
		} else if ((cmd_resp & CMD_MASK) != CMD_RTS) {
			netdev_err(mse->netdev, "%s: Unexpected response (0x%04x)\n",
						__func__, cmd_resp);
			goto unlock_spi;
		} else {
			netdev_warn(mse->netdev, "%s: Unexpected response to first CMD\n",
						 __func__);
		}
	}

	rxlen = cmd_resp & LEN_MASK;
	if (!rxlen) {
		netdev_warn(mse->netdev, "%s: No frame length defined\n",
					 __func__);
		goto unlock_spi;
	}

	rxalign = ALIGN(rxlen + DET_SOF_LEN + DET_DFT_LEN, 4);
	skb = netdev_alloc_skb_ip_align(mse->netdev, rxalign);
	if (!skb)
		goto unlock_spi;

	/* 2 bytes Start of frame (before ethernet header)
	 * 2 bytes Data frame tail (after ethernet frame)
	 * They are copied, but ignored.
	 */
	rxpkt = skb_put(skb, rxlen) - DET_SOF_LEN;
	mse102x_rx_frame_spi(mse, rxpkt, rxalign);

	if (netif_msg_pktdata(mse))
		mse102x_dbg_dumpkkt(mse, rxpkt);

	skb->protocol = eth_type_trans(skb, mse->netdev);
	skb->ip_summed = CHECKSUM_UNNECESSARY;
	netif_rx_ni(skb);

	mse->netdev->stats.rx_packets++;
	mse->netdev->stats.rx_bytes += rxlen;

unlock_spi:
	mse102x_unlock_spi(mse, &flags);
}

static void mse102x_tx_work(struct work_struct *work)
{
	struct mse102x_net_spi *mses;
	struct mse102x_net *mse;
	struct device *dev;
	unsigned long flags;
	struct sk_buff *txb;
	unsigned int pad_len;
	__be16 rx = 0;
	u16 cmd_resp;
	int ret;

	mses = container_of(work, struct mse102x_net_spi, tx_work);
	mse = &mses->mse102x;
	dev = &mses->spidev->dev;

	mse102x_lock_spi(mse, &flags);

	txb = skb_dequeue(&mse->txq);
	if (!txb)
		goto unlock_spi;

	pad_len = max_t(unsigned int, txb->len, 60);

	mse102x_tx_cmd_spi(mse, CMD_RTS | pad_len);
	ret = mse102x_rx_cmd_spi(mse, (u8 *)&rx);
	cmd_resp = be16_to_cpu(rx);

	if (ret || (cmd_resp != CMD_CTR)) {
		usleep_range(50, 100);

		/* Retransmit CMD_RTS */
		mse102x_tx_cmd_spi(mse, CMD_RTS | pad_len);
		ret = mse102x_rx_cmd_spi(mse, (u8 *)&rx);
		cmd_resp = be16_to_cpu(rx);
		if (ret) {
			netdev_err(mse->netdev, "%s: Failed to receive (%d), drop frame\n",
						__func__, ret);
			mse->netdev->stats.tx_dropped++;
			goto free_skb;
		} else if (cmd_resp != CMD_CTR) {
			netdev_err(mse->netdev, "%s: Unexpected response (0x%04x), drop frame\n",
						__func__, cmd_resp);
			mse->netdev->stats.tx_dropped++;
			goto free_skb;
		} else {
			netdev_warn(mse->netdev, "%s: Unexpected response to first CMD\n",
						 __func__);
		}
	}

	if (mse102x_tx_frame_spi(mse, txb)) {
		netdev_err(mse->netdev, "%s: Failed to send, drop frame (%u)\n", __func__, txb->len);
		mse->netdev->stats.tx_dropped++;
	} else {
		mse->netdev->stats.tx_bytes += txb->len;
		mse->netdev->stats.tx_packets++;
	}

free_skb:
	dev_kfree_skb(txb);

unlock_spi:
	mse102x_unlock_spi(mse, &flags);

	netif_wake_queue(mse->netdev);
}

void mse102x_flush_tx_work_spi(struct mse102x_net *mse)
{
	struct mse102x_net_spi *mses = to_mse102x_spi(mse);

	flush_work(&mses->tx_work);
}

netdev_tx_t mse102x_start_xmit_spi(struct sk_buff *skb,
				   struct net_device *dev)
{
	struct mse102x_net *mse = netdev_priv(dev);
	struct mse102x_net_spi *mses = to_mse102x_spi(mse);

	/* FIXME this needs proper TX flow control */

	netif_stop_queue(dev);

	netif_dbg(mse, tx_queued, mse->netdev,
		  "%s: skb %p, %d@%p\n", __func__, skb, skb->len, skb->data);

	skb_queue_tail(&mse->txq, skb);

	schedule_work(&mses->tx_work);

	return NETDEV_TX_OK;
}

static int mse102x_probe_spi(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct mse102x_net_spi *mses;
	struct net_device *netdev;
	struct mse102x_net *mse;
	int ret;

	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_3;
	ret = spi_setup(spi);
	if (ret < 0) {
		dev_err(&spi->dev, "Unable to setup SPI device: %d\n", ret);
		return ret;
	}

	netdev = devm_alloc_etherdev(dev, sizeof(struct mse102x_net_spi));
	if (!netdev)
		return -ENOMEM;

	netdev->priv_flags &= ~IFF_TX_SKB_SHARING;
	netdev->tx_queue_len = 100;

	dev_info(dev, "max_speed_hz=%d, half_duplex=%d\n",
		 spi->max_speed_hz,
		 (spi->master->flags & SPI_MASTER_HALF_DUPLEX) ? 1 : 0);

	mse = netdev_priv(netdev);
	mses = to_mse102x_spi(mse);

	mses->spidev = spi;
	mutex_init(&mses->lock);
	INIT_WORK(&mses->tx_work, mse102x_tx_work);

	/* initialise pre-made spi transfer messages */
	spi_message_init(&mses->spi_msg1);
	spi_message_add_tail(&mses->spi_xfer1, &mses->spi_msg1);

	spi_message_init(&mses->spi_msg2);
	spi_message_add_tail(&mses->spi_xfer2[0], &mses->spi_msg2);
	spi_message_add_tail(&mses->spi_xfer2[1], &mses->spi_msg2);

	netdev->irq = spi->irq;

	return mse102x_probe_common(netdev, dev, msg_enable);
}

static int mse102x_remove_spi(struct spi_device *spi)
{
	return mse102x_remove_common(&spi->dev);
}

static const struct of_device_id mse102x_match_table[] = {
	{ .compatible = "mediatek,mse1021" },
	{ .compatible = "mediatek,mse1022" },
	{ }
};
MODULE_DEVICE_TABLE(of, mse102x_match_table);

static struct spi_driver mse102x_driver = {
	.driver = {
		.name = "mse102x",
		.of_match_table = mse102x_match_table,
		.pm = &mse102x_pm_ops,
	},
	.probe = mse102x_probe_spi,
	.remove = mse102x_remove_spi,
};
module_spi_driver(mse102x_driver);

MODULE_DESCRIPTION("MSE102X Network driver");
MODULE_AUTHOR("Stefan Wahren <stefan.wahren@in-tech.com>");
MODULE_LICENSE("GPL");

module_param_named(message, msg_enable, int, 0);
MODULE_PARM_DESC(message, "Message verbosity level (0=none, 31=all)");
MODULE_ALIAS("spi:mse102x");
