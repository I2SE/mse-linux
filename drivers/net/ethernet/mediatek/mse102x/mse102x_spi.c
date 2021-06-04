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
#include <linux/crc32.h>

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

#define LEN_MASK	GENMASK(CMD_SHIFT - 1, 0)

#define	DET_CMD_LEN	4
#define	DET_SOF_LEN	2
#define	DET_DFT_LEN	2

/* SPI frame opcodes */
#define KS_SPIOP_RD	0x00
#define KS_SPIOP_WR	0x40
#define KS_SPIOP_RXFIFO	0x80
#define KS_SPIOP_TXFIFO	0xC0

/* shift for byte-enable data */
#define BYTE_EN(_x)	((_x) << 2)

/* turn register number and byte-enable mask into data for start of packet */
#define MK_OP(_byteen, _reg)	\
	(BYTE_EN(_byteen) | (_reg) << (8 + 2) | (_reg) >> 6)

static void mse102x_lock_spi(struct mse102x_net *mse, unsigned long *flags)
{
	struct mse102x_net_spi *mses = to_mse102x_spi(mse);

	mutex_lock(&mses->lock);
}

static void mse102x_unlock_spi(struct mse102x_net *mse, unsigned long *flags)
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
	__le16 txb[2];
	int ret;

	txb[0] = cpu_to_le16(DET_CMD);
	txb[1] = cpu_to_le16(cmd);

	xfer->tx_buf = txb;
	xfer->rx_buf = NULL;
	xfer->len = 4;

	ret = spi_sync(mses->spidev, msg);
	if (ret < 0)
		netdev_err(mse->netdev, "spi_sync() failed\n");
}

static void mse102x_rx_cmd_spi(struct mse102x_net *mse, u8 *rxb)
{
	struct mse102x_net_spi *mses = to_mse102x_spi(mse);
	struct spi_transfer *xfer;
	struct spi_message *msg;
	__le16 *txb = (__le16 *)mse->txd;
	u8 *trx = mse->rxd;
	int ret;

	txb[0] = 0;
	txb[1] = 0;

	if (mses->spidev->master->flags & SPI_MASTER_HALF_DUPLEX) {
		msg = &mses->spi_msg2;
		xfer = mses->spi_xfer2;

		xfer->tx_buf = txb;
		xfer->rx_buf = NULL;
		xfer->len = 2;

		xfer++;
		xfer->tx_buf = NULL;
		xfer->rx_buf = trx;
		xfer->len = 2;
	} else {
		msg = &mses->spi_msg1;
		xfer = &mses->spi_xfer1;

		xfer->tx_buf = txb;
		xfer->rx_buf = trx;
		xfer->len = 4;
	}

	ret = spi_sync(mses->spidev, msg);
	if (ret < 0)
		netdev_err(mse->netdev, "read: spi_sync() failed\n");
	else if (mses->spidev->master->flags & SPI_MASTER_HALF_DUPLEX)
		memcpy(rxb, trx, 2);
	else
		memcpy(rxb, trx + 2, 2);
}

static void mse102x_tx_frame_spi(struct mse102x_net *mse, struct sk_buff *txp)
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
		tskb = skb_copy_expand(txp, DET_SOF_LEN, DET_DFT_LEN + pad_len, GFP_ATOMIC);
		if (!tskb)
			return;

		dev_kfree_skb(txp);
		txp = tskb;
	}

	ptmp = skb_push(txp, DET_SOF_LEN);
	*ptmp = (DET_SOF >> 8) && 0xFF;
	ptmp++;
	*ptmp = DET_SOF && 0xFF;

	if (pad_len) {
		ptmp = skb_put_zero(txp, pad_len);
	}

	ptmp = skb_put(txp, DET_DFT_LEN);
	*ptmp = (DET_DFT >> 8) && 0xFF;
	ptmp++;
	*ptmp = DET_DFT && 0xFF;

	xfer->tx_buf = txp->data;
	xfer->rx_buf = NULL;
	xfer->len = txp->len;

	ret = spi_sync(mses->spidev, msg);
	if (ret < 0)
		netdev_err(mse->netdev, "%s: spi_sync() failed\n", __func__);
}

static void mse102x_wrreg16_spi(struct mse102x_net *mse, unsigned int reg,
			       unsigned int val)
{
	struct mse102x_net_spi *mses = to_mse102x_spi(mse);
	struct spi_transfer *xfer = &mses->spi_xfer1;
	struct spi_message *msg = &mses->spi_msg1;
	__le16 txb[2];
	int ret;

	txb[0] = cpu_to_le16(MK_OP(reg & 2 ? 0xC : 0x03, reg) | KS_SPIOP_WR);
	txb[1] = cpu_to_le16(val);

	xfer->tx_buf = txb;
	xfer->rx_buf = NULL;
	xfer->len = 4;

	ret = spi_sync(mses->spidev, msg);
	if (ret < 0)
		netdev_err(mse->netdev, "spi_sync() failed\n");
}

static void mse102x_rdreg(struct mse102x_net *mse, unsigned int op,
			 u8 *rxb, unsigned int rxl)
{
	struct mse102x_net_spi *mses = to_mse102x_spi(mse);
	struct spi_transfer *xfer;
	struct spi_message *msg;
	__le16 *txb = (__le16 *)mse->txd;
	u8 *trx = mse->rxd;
	int ret;

	txb[0] = cpu_to_le16(op | KS_SPIOP_RD);

	if (mses->spidev->master->flags & SPI_MASTER_HALF_DUPLEX) {
		msg = &mses->spi_msg2;
		xfer = mses->spi_xfer2;

		xfer->tx_buf = txb;
		xfer->rx_buf = NULL;
		xfer->len = 2;

		xfer++;
		xfer->tx_buf = NULL;
		xfer->rx_buf = trx;
		xfer->len = rxl;
	} else {
		msg = &mses->spi_msg1;
		xfer = &mses->spi_xfer1;

		xfer->tx_buf = txb;
		xfer->rx_buf = trx;
		xfer->len = rxl + 2;
	}

	ret = spi_sync(mses->spidev, msg);
	if (ret < 0)
		netdev_err(mse->netdev, "read: spi_sync() failed\n");
	else if (mses->spidev->master->flags & SPI_MASTER_HALF_DUPLEX)
		memcpy(rxb, trx, rxl);
	else
		memcpy(rxb, trx + 2, rxl);
}

static unsigned int mse102x_rdreg16_spi(struct mse102x_net *mse, unsigned int reg)
{
	__le16 rx = 0;

	mse102x_rdreg(mse, MK_OP(reg & 2 ? 0xC : 0x3, reg), (u8 *)&rx, 2);
	return le16_to_cpu(rx);
}

static void mse102x_rdfifo_spi(struct mse102x_net *mse, u8 *buff, unsigned int len)
{
	struct mse102x_net_spi *mses = to_mse102x_spi(mse);
	struct spi_transfer *xfer = mses->spi_xfer2;
	struct spi_message *msg = &mses->spi_msg2;
	u8 txb[1];
	int ret;

	netif_dbg(mse, rx_status, mse->netdev,
		  "%s: %d@%p\n", __func__, len, buff);

	/* set the operation we're issuing */
	txb[0] = KS_SPIOP_RXFIFO;

	xfer->tx_buf = txb;
	xfer->rx_buf = NULL;
	xfer->len = 1;

	xfer++;
	xfer->rx_buf = buff;
	xfer->tx_buf = NULL;
	xfer->len = len;

	ret = spi_sync(mses->spidev, msg);
	if (ret < 0)
		netdev_err(mse->netdev, "%s: spi_sync() failed\n", __func__);
}

static void mse102x_wrfifo_spi(struct mse102x_net *mse, struct sk_buff *txp,
			      bool irq)
{
	struct mse102x_net_spi *mses = to_mse102x_spi(mse);
	struct spi_transfer *xfer = mses->spi_xfer2;
	struct spi_message *msg = &mses->spi_msg2;
	unsigned int fid = 0;
	int ret;

	netif_dbg(mse, tx_queued, mse->netdev, "%s: skb %p, %d@%p, irq %d\n",
		  __func__, txp, txp->len, txp->data, irq);

	fid = mse->fid++;

	/* start header at txb[1] to align txw entries */
	mse->txh.txb[1] = KS_SPIOP_TXFIFO;
	mse->txh.txw[1] = cpu_to_le16(fid);
	mse->txh.txw[2] = cpu_to_le16(txp->len);

	xfer->tx_buf = &mse->txh.txb[1];
	xfer->rx_buf = NULL;
	xfer->len = 5;

	xfer++;
	xfer->tx_buf = txp->data;
	xfer->rx_buf = NULL;
	xfer->len = ALIGN(txp->len, 4);

	ret = spi_sync(mses->spidev, msg);
	if (ret < 0)
		netdev_err(mse->netdev, "%s: spi_sync() failed\n", __func__);
}

static void mse102x_rx_skb_spi(struct mse102x_net *mse, struct sk_buff *skb)
{
	netif_rx_ni(skb);
}

static void mse102x_tx_work(struct work_struct *work)
{
	struct mse102x_net_spi *mses;
	struct mse102x_net *mse;
	unsigned long flags;
	struct sk_buff *txb;
	bool last;

	mses = container_of(work, struct mse102x_net_spi, tx_work);
	mse = &mses->mse102x;
	last = skb_queue_empty(&mse->txq);

	mse102x_lock_spi(mse, &flags);

	while (!last) {
		txb = skb_dequeue(&mse->txq);
		last = skb_queue_empty(&mse->txq);

		if (txb) {
			mse102x_wrfifo_spi(mse, txb, last);

			mse102x_done_tx(mse, txb);
		}
	}

	mse102x_unlock_spi(mse, &flags);
}

static void mse102x_flush_tx_work_spi(struct mse102x_net *mse)
{
	struct mse102x_net_spi *mses = to_mse102x_spi(mse);

	flush_work(&mses->tx_work);
}

static netdev_tx_t mse102x_start_xmit_spi(struct sk_buff *skb,
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

	netdev = devm_alloc_etherdev(dev, sizeof(struct mse102x_net_spi));
	if (!netdev)
		return -ENOMEM;

	spi->bits_per_word = 8;

	mse = netdev_priv(netdev);

	mse->lock = mse102x_lock_spi;
	mse->unlock = mse102x_unlock_spi;
	mse->rdreg16 = mse102x_rdreg16_spi;
	mse->wrreg16 = mse102x_wrreg16_spi;
	mse->rdfifo = mse102x_rdfifo_spi;
	mse->wrfifo = mse102x_wrfifo_spi;
	mse->start_xmit = mse102x_start_xmit_spi;
	mse->rx_skb = mse102x_rx_skb_spi;
	mse->flush_tx_work = mse102x_flush_tx_work_spi;

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
	{ .compatible = "mediatek,mse102x" },
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
