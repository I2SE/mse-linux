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
#include <linux/regulator/consumer.h>

#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
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

#define to_mse102x_spi(ks) container_of((ks), struct mse102x_net_spi, mse102x)

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

static void mse102x_lock_spi(struct mse102x_net *ks, unsigned long *flags)
{
	struct mse102x_net_spi *kss = to_mse102x_spi(ks);

	mutex_lock(&kss->lock);
}

static void mse102x_unlock_spi(struct mse102x_net *ks, unsigned long *flags)
{
	struct mse102x_net_spi *kss = to_mse102x_spi(ks);

	mutex_unlock(&kss->lock);
}

/* SPI register read/write calls.
 *
 * All these calls issue SPI transactions to access the chip's registers. They
 * all require that the necessary lock is held to prevent accesses when the
 * chip is busy transferring packet data (RX/TX FIFO accesses).
 */

static void mse102x_wrreg16_spi(struct mse102x_net *ks, unsigned int reg,
			       unsigned int val)
{
	struct mse102x_net_spi *kss = to_mse102x_spi(ks);
	struct spi_transfer *xfer = &kss->spi_xfer1;
	struct spi_message *msg = &kss->spi_msg1;
	__le16 txb[2];
	int ret;

	txb[0] = cpu_to_le16(MK_OP(reg & 2 ? 0xC : 0x03, reg) | KS_SPIOP_WR);
	txb[1] = cpu_to_le16(val);

	xfer->tx_buf = txb;
	xfer->rx_buf = NULL;
	xfer->len = 4;

	ret = spi_sync(kss->spidev, msg);
	if (ret < 0)
		netdev_err(ks->netdev, "spi_sync() failed\n");
}

static void mse102x_rdreg(struct mse102x_net *ks, unsigned int op,
			 u8 *rxb, unsigned int rxl)
{
	struct mse102x_net_spi *kss = to_mse102x_spi(ks);
	struct spi_transfer *xfer;
	struct spi_message *msg;
	__le16 *txb = (__le16 *)ks->txd;
	u8 *trx = ks->rxd;
	int ret;

	txb[0] = cpu_to_le16(op | KS_SPIOP_RD);

	if (kss->spidev->master->flags & SPI_MASTER_HALF_DUPLEX) {
		msg = &kss->spi_msg2;
		xfer = kss->spi_xfer2;

		xfer->tx_buf = txb;
		xfer->rx_buf = NULL;
		xfer->len = 2;

		xfer++;
		xfer->tx_buf = NULL;
		xfer->rx_buf = trx;
		xfer->len = rxl;
	} else {
		msg = &kss->spi_msg1;
		xfer = &kss->spi_xfer1;

		xfer->tx_buf = txb;
		xfer->rx_buf = trx;
		xfer->len = rxl + 2;
	}

	ret = spi_sync(kss->spidev, msg);
	if (ret < 0)
		netdev_err(ks->netdev, "read: spi_sync() failed\n");
	else if (kss->spidev->master->flags & SPI_MASTER_HALF_DUPLEX)
		memcpy(rxb, trx, rxl);
	else
		memcpy(rxb, trx + 2, rxl);
}

static unsigned int mse102x_rdreg16_spi(struct mse102x_net *ks, unsigned int reg)
{
	__le16 rx = 0;

	mse102x_rdreg(ks, MK_OP(reg & 2 ? 0xC : 0x3, reg), (u8 *)&rx, 2);
	return le16_to_cpu(rx);
}

static void mse102x_rdfifo_spi(struct mse102x_net *ks, u8 *buff, unsigned int len)
{
	struct mse102x_net_spi *kss = to_mse102x_spi(ks);
	struct spi_transfer *xfer = kss->spi_xfer2;
	struct spi_message *msg = &kss->spi_msg2;
	u8 txb[1];
	int ret;

	netif_dbg(ks, rx_status, ks->netdev,
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

	ret = spi_sync(kss->spidev, msg);
	if (ret < 0)
		netdev_err(ks->netdev, "%s: spi_sync() failed\n", __func__);
}

static void mse102x_wrfifo_spi(struct mse102x_net *ks, struct sk_buff *txp,
			      bool irq)
{
	struct mse102x_net_spi *kss = to_mse102x_spi(ks);
	struct spi_transfer *xfer = kss->spi_xfer2;
	struct spi_message *msg = &kss->spi_msg2;
	unsigned int fid = 0;
	int ret;

	netif_dbg(ks, tx_queued, ks->netdev, "%s: skb %p, %d@%p, irq %d\n",
		  __func__, txp, txp->len, txp->data, irq);

	fid = ks->fid++;
	fid &= TXFR_TXFID_MASK;

	if (irq)
		fid |= TXFR_TXIC;	/* irq on completion */

	/* start header at txb[1] to align txw entries */
	ks->txh.txb[1] = KS_SPIOP_TXFIFO;
	ks->txh.txw[1] = cpu_to_le16(fid);
	ks->txh.txw[2] = cpu_to_le16(txp->len);

	xfer->tx_buf = &ks->txh.txb[1];
	xfer->rx_buf = NULL;
	xfer->len = 5;

	xfer++;
	xfer->tx_buf = txp->data;
	xfer->rx_buf = NULL;
	xfer->len = ALIGN(txp->len, 4);

	ret = spi_sync(kss->spidev, msg);
	if (ret < 0)
		netdev_err(ks->netdev, "%s: spi_sync() failed\n", __func__);
}

static void mse102x_rx_skb_spi(struct mse102x_net *ks, struct sk_buff *skb)
{
	netif_rx_ni(skb);
}

static void mse102x_tx_work(struct work_struct *work)
{
	struct mse102x_net_spi *kss;
	struct mse102x_net *ks;
	unsigned long flags;
	struct sk_buff *txb;
	bool last;

	kss = container_of(work, struct mse102x_net_spi, tx_work);
	ks = &kss->mse102x;
	last = skb_queue_empty(&ks->txq);

	mse102x_lock_spi(ks, &flags);

	while (!last) {
		txb = skb_dequeue(&ks->txq);
		last = skb_queue_empty(&ks->txq);

		if (txb) {
			mse102x_wrreg16_spi(ks, KS_RXQCR,
					   ks->rc_rxqcr | RXQCR_SDA);
			mse102x_wrfifo_spi(ks, txb, last);
			mse102x_wrreg16_spi(ks, KS_RXQCR, ks->rc_rxqcr);
			mse102x_wrreg16_spi(ks, KS_TXQCR, TXQCR_METFE);

			mse102x_done_tx(ks, txb);
		}
	}

	mse102x_unlock_spi(ks, &flags);
}

static void mse102x_flush_tx_work_spi(struct mse102x_net *ks)
{
	struct mse102x_net_spi *kss = to_mse102x_spi(ks);

	flush_work(&kss->tx_work);
}

static unsigned int calc_txlen(unsigned int len)
{
	return ALIGN(len + 4, 4);
}

static netdev_tx_t mse102x_start_xmit_spi(struct sk_buff *skb,
					 struct net_device *dev)
{
	unsigned int needed = calc_txlen(skb->len);
	struct mse102x_net *ks = netdev_priv(dev);
	netdev_tx_t ret = NETDEV_TX_OK;
	struct mse102x_net_spi *kss;

	kss = to_mse102x_spi(ks);

	netif_dbg(ks, tx_queued, ks->netdev,
		  "%s: skb %p, %d@%p\n", __func__, skb, skb->len, skb->data);

	spin_lock(&ks->statelock);

	if (needed > ks->tx_space) {
		netif_stop_queue(dev);
		ret = NETDEV_TX_BUSY;
	} else {
		ks->tx_space -= needed;
		skb_queue_tail(&ks->txq, skb);
	}

	spin_unlock(&ks->statelock);
	schedule_work(&kss->tx_work);

	return ret;
}

static int mse102x_probe_spi(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct mse102x_net_spi *kss;
	struct net_device *netdev;
	struct mse102x_net *ks;

	netdev = devm_alloc_etherdev(dev, sizeof(struct mse102x_net_spi));
	if (!netdev)
		return -ENOMEM;

	spi->bits_per_word = 8;

	ks = netdev_priv(netdev);

	ks->lock = mse102x_lock_spi;
	ks->unlock = mse102x_unlock_spi;
	ks->rdreg16 = mse102x_rdreg16_spi;
	ks->wrreg16 = mse102x_wrreg16_spi;
	ks->rdfifo = mse102x_rdfifo_spi;
	ks->wrfifo = mse102x_wrfifo_spi;
	ks->start_xmit = mse102x_start_xmit_spi;
	ks->rx_skb = mse102x_rx_skb_spi;
	ks->flush_tx_work = mse102x_flush_tx_work_spi;

#define STD_IRQ (IRQ_LCI |	/* Link Change */	\
		 IRQ_TXI |	/* TX done */		\
		 IRQ_RXI |	/* RX done */		\
		 IRQ_SPIBEI |	/* SPI bus error */	\
		 IRQ_TXPSI |	/* TX process stop */	\
		 IRQ_RXPSI)	/* RX process stop */
	ks->rc_ier = STD_IRQ;

	kss = to_mse102x_spi(ks);

	kss->spidev = spi;
	mutex_init(&kss->lock);
	INIT_WORK(&kss->tx_work, mse102x_tx_work);

	/* initialise pre-made spi transfer messages */
	spi_message_init(&kss->spi_msg1);
	spi_message_add_tail(&kss->spi_xfer1, &kss->spi_msg1);

	spi_message_init(&kss->spi_msg2);
	spi_message_add_tail(&kss->spi_xfer2[0], &kss->spi_msg2);
	spi_message_add_tail(&kss->spi_xfer2[1], &kss->spi_msg2);

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
