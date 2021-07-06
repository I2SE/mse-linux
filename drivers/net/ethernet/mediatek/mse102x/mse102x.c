// SPDX-License-Identifier: GPL-2.0-only
/* Copyright 2021 in-tech smart charging GmbH
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/cache.h>

#include <linux/spi/spi.h>
#include <linux/of_net.h>

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

struct mse102x_net {
	struct net_device	*netdev;

	u8			rxd[8];
	u8			txd[8];

	u32			msg_enable ____cacheline_aligned;
	u8			fid;

	struct sk_buff_head	txq;
};

struct mse102x_net_spi {
	struct mse102x_net	mse102x;
	struct mutex		lock;		/* Protect SPI frame transfer */
	struct work_struct	tx_work;
	struct spi_device	*spidev;
	struct spi_message	spi_msg;
	struct spi_transfer	spi_xfer;
};

#define to_mse102x_spi(mse) container_of((mse), struct mse102x_net_spi, mse102x)

static int msg_enable;
module_param_named(message, msg_enable, int, 0);
MODULE_PARM_DESC(message, "Message verbosity level (0=none, 31=all)");

/* SPI register read/write calls.
 *
 * All these calls issue SPI transactions to access the chip's registers. They
 * all require that the necessary lock is held to prevent accesses when the
 * chip is busy transferring packet data.
 */

static void mse102x_tx_cmd_spi(struct mse102x_net *mse, u16 cmd)
{
	struct mse102x_net_spi *mses = to_mse102x_spi(mse);
	struct spi_transfer *xfer = &mses->spi_xfer;
	struct spi_message *msg = &mses->spi_msg;
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
	struct spi_transfer *xfer = &mses->spi_xfer;
	struct spi_message *msg = &mses->spi_msg;
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
	if (ret < 0)
		netdev_err(mse->netdev, "read: spi_sync() failed\n");
	else if (*cmd != cpu_to_be16(DET_CMD))
		ret = -EIO;
	else
		memcpy(rxb, trx + 2, 2);

	return ret;
}

static int mse102x_tx_frame_spi(struct mse102x_net *mse, struct sk_buff *txp)
{
	struct mse102x_net_spi *mses = to_mse102x_spi(mse);
	struct spi_transfer *xfer = &mses->spi_xfer;
	struct spi_message *msg = &mses->spi_msg;
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

	/* TODO handle CPU endianess */
	ptmp = skb_push(txp, DET_SOF_LEN);
	*ptmp = (DET_SOF >> 8) & 0xFF;
	ptmp++;
	*ptmp = DET_SOF & 0xFF;

	if (pad_len)
		ptmp = skb_put_zero(txp, pad_len);

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

static int mse102x_rx_frame_spi(struct mse102x_net *mse, u8 *buff, unsigned int frame_len)
{
	struct mse102x_net_spi *mses = to_mse102x_spi(mse);
	struct spi_transfer *xfer = &mses->spi_xfer;
	struct spi_message *msg = &mses->spi_msg;
	__be16 *sof = (__be16 *)buff;
	__be16 *dft = (__be16 *)(buff + DET_SOF_LEN + frame_len);
	int ret;

	xfer->rx_buf = buff;
	xfer->tx_buf = NULL;
	xfer->len = DET_SOF_LEN + frame_len + DET_DFT_LEN;

	ret = spi_sync(mses->spidev, msg);
	if (ret < 0) {
		netdev_err(mse->netdev, "%s: spi_sync() failed\n", __func__);
	} else if (*sof != cpu_to_be16(DET_SOF)) {
		netdev_err(mse->netdev, "%s: SPI start of frame is invalid (0x%04x)\n",
			   __func__, *sof);
		ret = -EIO;
	} else if (*dft != cpu_to_be16(DET_DFT)) {
		netdev_err(mse->netdev, "%s: SPI frame tail is invalid (0x%04x)\n",
			   __func__, *dft);
		ret = -EIO;
	}

	return ret;
}

static void mse102x_dump_packet(const char *msg, int len, const char *data)
{
	printk(KERN_DEBUG ": %s - packet len:%d\n", msg, len);
	print_hex_dump(KERN_DEBUG, "pk data: ", DUMP_PREFIX_OFFSET, 16, 1,
		       data, len, true);
}

void mse102x_rx_pkts_spi(struct mse102x_net *mse)
{
	struct mse102x_net_spi *mses = to_mse102x_spi(mse);
	struct sk_buff *skb;
	unsigned int rxalign;
	unsigned int rxlen;
	u8 *rxpkt;
	__be16 rx = 0;
	u16 cmd_resp;
	int ret;

	mutex_lock(&mses->lock);

	mse102x_tx_cmd_spi(mse, CMD_CTR);
	ret = mse102x_rx_cmd_spi(mse, (u8 *)&rx);
	cmd_resp = be16_to_cpu(rx);

	if (ret || ((cmd_resp & CMD_MASK) != CMD_RTS)) {
		usleep_range(50, 100);

		mse102x_tx_cmd_spi(mse, CMD_CTR);
		ret = mse102x_rx_cmd_spi(mse, (u8 *)&rx);
		cmd_resp = be16_to_cpu(rx);
		if (ret) {
			net_err_ratelimited("%s: Failed to receive (%d)\n",
					    __func__, ret);
			goto unlock_spi;
		} else if ((cmd_resp & CMD_MASK) != CMD_RTS) {
			net_err_ratelimited("%s: Unexpected response (0x%04x)\n",
					    __func__, cmd_resp);
			goto unlock_spi;
		} else {
			net_warn_ratelimited("%s: Unexpected response to first CMD\n",
					     __func__);
		}
	}

	rxlen = cmd_resp & LEN_MASK;
	if (!rxlen) {
		net_warn_ratelimited("%s: No frame length defined\n",
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
	if (mse102x_rx_frame_spi(mse, rxpkt, rxlen)) {
		mse->netdev->stats.rx_errors++;
		dev_kfree_skb(skb);
		goto unlock_spi;
	}

	if (netif_msg_pktdata(mse))
		mse102x_dump_packet(__func__, skb->len, skb->data);

	skb->protocol = eth_type_trans(skb, mse->netdev);
	skb->ip_summed = CHECKSUM_UNNECESSARY;
	netif_rx_ni(skb);

	mse->netdev->stats.rx_packets++;
	mse->netdev->stats.rx_bytes += rxlen;

unlock_spi:
	mutex_unlock(&mses->lock);
}

static void mse102x_tx_work(struct work_struct *work)
{
	struct mse102x_net_spi *mses;
	struct mse102x_net *mse;
	struct device *dev;
	struct sk_buff *txb;
	unsigned int pad_len;
	__be16 rx = 0;
	u16 cmd_resp;
	int ret;

	mses = container_of(work, struct mse102x_net_spi, tx_work);
	mse = &mses->mse102x;
	dev = &mses->spidev->dev;

	mutex_lock(&mses->lock);

	txb = skb_dequeue(&mse->txq);
	if (!txb)
		goto unlock_spi;

	/* TODO: move padding out of mse102x_tx_frame_spi and place it HERE */
	pad_len = max_t(unsigned int, txb->len, 60);

	mse102x_tx_cmd_spi(mse, CMD_RTS | pad_len);
	ret = mse102x_rx_cmd_spi(mse, (u8 *)&rx);
	cmd_resp = be16_to_cpu(rx);

	if (ret || cmd_resp != CMD_CTR) {
		usleep_range(50, 100);

		/* Retransmit CMD_RTS */
		mse102x_tx_cmd_spi(mse, CMD_RTS | pad_len);
		ret = mse102x_rx_cmd_spi(mse, (u8 *)&rx);
		cmd_resp = be16_to_cpu(rx);
		if (ret) {
			net_err_ratelimited("%s: Failed to receive (%d), drop frame\n",
					    __func__, ret);
			mse->netdev->stats.tx_dropped++;
			goto free_skb;
		} else if (cmd_resp != CMD_CTR) {
			net_err_ratelimited("%s: Unexpected response (0x%04x), drop frame\n",
					    __func__, cmd_resp);
			mse->netdev->stats.tx_dropped++;
			goto free_skb;
		} else {
			net_warn_ratelimited("%s: Unexpected response to first CMD\n",
					     __func__);
		}
	}

	if (mse102x_tx_frame_spi(mse, txb)) {
		net_err_ratelimited("%s: Failed to send, drop frame (%u)\n",
				    __func__, txb->len);
		mse->netdev->stats.tx_dropped++;
	} else {
		mse->netdev->stats.tx_bytes += txb->len;
		mse->netdev->stats.tx_packets++;
	}

free_skb:
	dev_kfree_skb(txb);

unlock_spi:
	mutex_unlock(&mses->lock);

	netif_wake_queue(mse->netdev);
}

netdev_tx_t mse102x_start_xmit_spi(struct sk_buff *skb,
				   struct net_device *dev)
{
	struct mse102x_net *mse = netdev_priv(dev);
	struct mse102x_net_spi *mses = to_mse102x_spi(mse);

	/* Since the chip accepts only one packet at once, stop the tx queue
	 * now and wake it after spi transfer.
	 */
	netif_stop_queue(dev);

	netif_dbg(mse, tx_queued, mse->netdev,
		  "%s: skb %p, %d@%p\n", __func__, skb, skb->len, skb->data);

	skb_queue_tail(&mse->txq, skb);

	schedule_work(&mses->tx_work);

	return NETDEV_TX_OK;
}


static void mse102x_init_mac(struct mse102x_net *mse, struct device_node *np)
{
	struct net_device *dev = mse->netdev;
	const u8 *mac_addr;

	mac_addr = of_get_mac_address(np);
	if (!IS_ERR(mac_addr)) {
		ether_addr_copy(dev->dev_addr, mac_addr);
		return;
	}

	eth_hw_addr_random(dev);
}

static irqreturn_t mse102x_irq(int irq, void *_mse)
{
	struct mse102x_net *mse = _mse;

	mse102x_rx_pkts_spi(mse);

	return IRQ_HANDLED;
}

static int mse102x_net_open(struct net_device *dev)
{
	struct mse102x_net *mse = netdev_priv(dev);
	struct mse102x_net_spi *mses = to_mse102x_spi(mse);
	int ret;

	ret = request_threaded_irq(dev->irq, NULL, mse102x_irq, IRQF_ONESHOT,
				   dev->name, mse);
	if (ret < 0) {
		netdev_err(dev, "failed to get irq\n");
		return ret;
	}

	/* lock the card, even if we may not actually be doing anything
	 * else at the moment
	 */
	mutex_lock(&mses->lock);

	netif_dbg(mse, ifup, mse->netdev, "opening\n");

	netif_start_queue(mse->netdev);

	netif_carrier_on(mse->netdev);

	netif_dbg(mse, ifup, mse->netdev, "network device up\n");

	mutex_unlock(&mses->lock);

	return 0;
}

static int mse102x_net_stop(struct net_device *dev)
{
	struct mse102x_net *mse = netdev_priv(dev);
	struct mse102x_net_spi *mses = to_mse102x_spi(mse);

	netif_info(mse, ifdown, dev, "shutting down\n");

	netif_stop_queue(dev);

	/* stop any outstanding work */
	flush_work(&mses->tx_work);

	/* ensure any queued tx buffers are dumped */
	while (!skb_queue_empty(&mse->txq)) {
		struct sk_buff *txb = skb_dequeue(&mse->txq);

		netif_dbg(mse, ifdown, mse->netdev,
			  "%s: freeing txb %p\n", __func__, txb);

		dev_kfree_skb(txb);
	}

	free_irq(dev->irq, mse);

	return 0;
}

static const struct net_device_ops mse102x_netdev_ops = {
	.ndo_open		= mse102x_net_open,
	.ndo_stop		= mse102x_net_stop,
	.ndo_start_xmit		= mse102x_start_xmit_spi,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
};

/* ethtool support */

static void mse102x_get_drvinfo(struct net_device *dev,
				struct ethtool_drvinfo *di)
{
	strlcpy(di->driver, "mse102x", sizeof(di->driver));
	strlcpy(di->version, "1.00", sizeof(di->version));
	strlcpy(di->bus_info, dev_name(dev->dev.parent), sizeof(di->bus_info));
}

static u32 mse102x_get_msglevel(struct net_device *dev)
{
	struct mse102x_net *mse = netdev_priv(dev);

	return mse->msg_enable;
}

static void mse102x_set_msglevel(struct net_device *dev, u32 to)
{
	struct mse102x_net *mse = netdev_priv(dev);

	mse->msg_enable = to;
}

static const struct ethtool_ops mse102x_ethtool_ops = {
	.get_drvinfo	= mse102x_get_drvinfo,
	.get_link	= ethtool_op_get_link,
	.get_msglevel	= mse102x_get_msglevel,
	.set_msglevel	= mse102x_set_msglevel,
};

/* driver bus management functions */

#ifdef CONFIG_PM_SLEEP

int mse102x_suspend(struct device *dev)
{
	struct mse102x_net *mse = dev_get_drvdata(dev);
	struct net_device *netdev = mse->netdev;

	if (netif_running(netdev)) {
		netif_device_detach(netdev);
		mse102x_net_stop(netdev);
	}

	return 0;
}

int mse102x_resume(struct device *dev)
{
	struct mse102x_net *mse = dev_get_drvdata(dev);
	struct net_device *netdev = mse->netdev;

	if (netif_running(netdev)) {
		mse102x_net_open(netdev);
		netif_device_attach(netdev);
	}

	return 0;
}
#endif

static __maybe_unused SIMPLE_DEV_PM_OPS(mse102x_pm_ops,
					mse102x_suspend, mse102x_resume);

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
	spi_message_init(&mses->spi_msg);
	spi_message_add_tail(&mses->spi_xfer, &mses->spi_msg);

	netdev->irq = spi->irq;
	mse->netdev = netdev;

	dev_info(dev, "message enable is %d\n", msg_enable);

	/* set the default message enable */
	mse->msg_enable = netif_msg_init(msg_enable, NETIF_MSG_DRV |
					 NETIF_MSG_PROBE | NETIF_MSG_LINK);

	skb_queue_head_init(&mse->txq);

	netdev->ethtool_ops = &mse102x_ethtool_ops;
	SET_NETDEV_DEV(netdev, dev);

	dev_set_drvdata(dev, mse);

	netif_carrier_off(mse->netdev);
	netdev->if_port = IF_PORT_100BASET;
	netdev->netdev_ops = &mse102x_netdev_ops;

	mse102x_init_mac(mse, dev->of_node);

	ret = register_netdev(netdev);
	if (ret) {
		dev_err(dev, "failed to register network device: %d\n", ret);
		return ret;
	}

	dev_info(dev, "%s: Success\n", __func__);

	return 0;
}

static int mse102x_remove_spi(struct spi_device *spi)
{
	struct mse102x_net *priv = dev_get_drvdata(&spi->dev);

	if (netif_msg_drv(priv))
		dev_info(&spi->dev, "remove\n");

	unregister_netdev(priv->netdev);

	return 0;
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
MODULE_ALIAS("spi:mse102x");
