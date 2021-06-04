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

#include <linux/of_net.h>

#include "mse102x.h"

static void mse102x_lock(struct mse102x_net *mse, unsigned long *flags)
{
	mse->lock(mse, flags);
}

static void mse102x_unlock(struct mse102x_net *mse, unsigned long *flags)
{
	mse->unlock(mse, flags);
}

static void mse102x_wrreg16(struct mse102x_net *mse, unsigned int reg,
			   unsigned int val)
{
	mse->wrreg16(mse, reg, val);
}

static unsigned int mse102x_rdreg16(struct mse102x_net *mse,
				   unsigned int reg)
{
	return mse->rdreg16(mse, reg);
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

static void mse102x_dbg_dumpkkt(struct mse102x_net *mse, u8 *rxpkt)
{
	netdev_dbg(mse->netdev,
		   "pkt %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x\n",
		   rxpkt[4], rxpkt[5], rxpkt[6], rxpkt[7],
		   rxpkt[8], rxpkt[9], rxpkt[10], rxpkt[11],
		   rxpkt[12], rxpkt[13], rxpkt[14], rxpkt[15]);
}

static void mse102x_rx_skb(struct mse102x_net *mse, struct sk_buff *skb)
{
	mse->rx_skb(mse, skb);
}

static void mse102x_rx_pkts(struct mse102x_net *mse)
{
	struct sk_buff *skb;
	unsigned rxfc;
	unsigned rxlen;
	unsigned rxstat;
	u8 *rxpkt;

	rxfc = (mse102x_rdreg16(mse, KS_RXFCTR) >> 8) & 0xff;

	netif_dbg(mse, rx_status, mse->netdev,
		  "%s: %d packets\n", __func__, rxfc);

	/* Currently we're issuing a read per packet, but we could possibly
	 * improve the code by issuing a single read, getting the receive
	 * header, allocating the packet and then reading the packet data
	 * out in one go.
	 *
	 * This form of operation would require us to hold the SPI bus'
	 * chipselect low during the entie transaction to avoid any
	 * reset to the data stream coming from the chip.
	 */

	for (; rxfc != 0; rxfc--) {
		rxstat = mse102x_rdreg16(mse, KS_RXFHSR);
		rxlen = mse102x_rdreg16(mse, KS_RXFHBCR) & RXFHBCR_CNT_MASK;

		netif_dbg(mse, rx_status, mse->netdev,
			  "rx: stat 0x%04x, len 0x%04x\n", rxstat, rxlen);

		/* the length of the packet includes the 32bit CRC */

		/* set dma read address */
		mse102x_wrreg16(mse, KS_RXFDPR, RXFDPR_RXFPAI | 0x00);

		/* start DMA access */
		mse102x_wrreg16(mse, KS_RXQCR, mse->rc_rxqcr | RXQCR_SDA);

		if (rxlen > 4) {
			unsigned int rxalign;

			rxlen -= 4;
			rxalign = ALIGN(rxlen, 4);
			skb = netdev_alloc_skb_ip_align(mse->netdev, rxalign);
			if (skb) {

				/* 4 bytes of status header + 4 bytes of
				 * garbage: we put them before ethernet
				 * header, so that they are copied,
				 * but ignored.
				 */

				rxpkt = skb_put(skb, rxlen) - 8;

				mse->rdfifo(mse, rxpkt, rxalign + 8);

				if (netif_msg_pktdata(mse))
					mse102x_dbg_dumpkkt(mse, rxpkt);

				skb->protocol = eth_type_trans(skb, mse->netdev);
				mse102x_rx_skb(mse, skb);

				mse->netdev->stats.rx_packets++;
				mse->netdev->stats.rx_bytes += rxlen;
			}
		}

		/* end DMA access and dequeue packet */
		mse102x_wrreg16(mse, KS_RXQCR, mse->rc_rxqcr | RXQCR_RRXEF);
	}
}

static irqreturn_t mse102x_irq(int irq, void *_mse)
{
	struct mse102x_net *mse = _mse;
	unsigned long flags;

	mse102x_lock(mse, &flags);

	mse102x_rx_pkts(mse);

	mse102x_unlock(mse, &flags);

	return IRQ_HANDLED;
}

static void mse102x_flush_tx_work(struct mse102x_net *mse)
{
	if (mse->flush_tx_work)
		mse->flush_tx_work(mse);
}

static int mse102x_net_open(struct net_device *dev)
{
	struct mse102x_net *mse = netdev_priv(dev);
	unsigned long flags;
	int ret;

	ret = request_threaded_irq(dev->irq, NULL, mse102x_irq,
				   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				   dev->name, mse);
	if (ret < 0) {
		netdev_err(dev, "failed to get irq\n");
		return ret;
	}

	/* lock the card, even if we may not actually be doing anything
	 * else at the moment */
	mse102x_lock(mse, &flags);

	netif_dbg(mse, ifup, mse->netdev, "opening\n");

	/* setup transmission parameters */

	mse102x_wrreg16(mse, KS_TXCR, (TXCR_TXE | /* enable transmit process */
				     TXCR_TXPE | /* pad to min length */
				     TXCR_TXCRC | /* add CRC */
				     TXCR_TXFCE)); /* enable flow control */

	/* auto-increment tx data, reset tx pointer */
	mse102x_wrreg16(mse, KS_TXFDPR, TXFDPR_TXFPAI);

	/* setup receiver control */

	mse102x_wrreg16(mse, KS_RXCR1, (RXCR1_RXPAFMA | /*  from mac filter */
				      RXCR1_RXFCE | /* enable flow control */
				      RXCR1_RXBE | /* broadcast enable */
				      RXCR1_RXUE | /* unicast enable */
				      RXCR1_RXE)); /* enable rx block */

	/* transfer entire frames out in one go */
	mse102x_wrreg16(mse, KS_RXCR2, RXCR2_SRDBL_FRAME);

	/* set receive counter timeouts */
	mse102x_wrreg16(mse, KS_RXDTTR, 1000); /* 1ms after first frame to IRQ */
	mse102x_wrreg16(mse, KS_RXDBCTR, 4096); /* >4Kbytes in buffer to IRQ */
	mse102x_wrreg16(mse, KS_RXFCTR, 10);  /* 10 frames to IRQ */

	mse->rc_rxqcr = (RXQCR_RXFCTE |  /* IRQ on frame count exceeded */
			RXQCR_RXDBCTE | /* IRQ on byte count exceeded */
			RXQCR_RXDTTE);  /* IRQ on time exceeded */

	mse102x_wrreg16(mse, KS_RXQCR, mse->rc_rxqcr);


	netif_start_queue(mse->netdev);

	netif_dbg(mse, ifup, mse->netdev, "network device up\n");

	mse102x_unlock(mse, &flags);

	return 0;
}

static int mse102x_net_stop(struct net_device *dev)
{
	struct mse102x_net *mse = netdev_priv(dev);
	unsigned long flags;

	netif_info(mse, ifdown, dev, "shutting down\n");

	netif_stop_queue(dev);

	/* stop any outstanding work */
	mse102x_flush_tx_work(mse);
	flush_work(&mse->rxctrl_work);

	mse102x_lock(mse, &flags);
	/* shutdown RX process */
	mse102x_wrreg16(mse, KS_RXCR1, 0x0000);

	/* shutdown TX process */
	mse102x_wrreg16(mse, KS_TXCR, 0x0000);

	mse102x_unlock(mse, &flags);

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

static netdev_tx_t mse102x_start_xmit(struct sk_buff *skb,
				     struct net_device *dev)
{
	struct mse102x_net *mse = netdev_priv(dev);

	return mse->start_xmit(skb, dev);
}

static void mse102x_rxctrl_work(struct work_struct *work)
{
	struct mse102x_net *mse = container_of(work, struct mse102x_net, rxctrl_work);
	unsigned long flags;

	mse102x_lock(mse, &flags);

	/* need to shutdown RXQ before modifying filter parameters */
	mse102x_wrreg16(mse, KS_RXCR1, 0x00);

	mse102x_unlock(mse, &flags);
}

static void mse102x_set_rx_mode(struct net_device *dev)
{
	struct mse102x_net *mse = netdev_priv(dev);
	struct mse102x_rxctrl rxctrl;

	memset(&rxctrl, 0, sizeof(rxctrl));

	if (dev->flags & IFF_PROMISC) {
		/* interface to receive everything */

		rxctrl.rxcr1 = RXCR1_RXAE | RXCR1_RXINVF;
	} else if (dev->flags & IFF_ALLMULTI) {
		/* accept all multicast packets */

		rxctrl.rxcr1 = (RXCR1_RXME | RXCR1_RXAE |
				RXCR1_RXPAFMA | RXCR1_RXMAFMA);
	} else if (dev->flags & IFF_MULTICAST && !netdev_mc_empty(dev)) {
		struct netdev_hw_addr *ha;
		u32 crc;

		/* accept some multicast */

		netdev_for_each_mc_addr(ha, dev) {
			crc = ether_crc(ETH_ALEN, ha->addr);
			crc >>= (32 - 6);  /* get top six bits */

			rxctrl.mchash[crc >> 4] |= (1 << (crc & 0xf));
		}

		rxctrl.rxcr1 = RXCR1_RXME | RXCR1_RXPAFMA;
	} else {
		/* just accept broadcast / unicast */
		rxctrl.rxcr1 = RXCR1_RXPAFMA;
	}

	rxctrl.rxcr1 |= (RXCR1_RXUE | /* unicast enable */
			 RXCR1_RXBE | /* broadcast enable */
			 RXCR1_RXE | /* RX process enable */
			 RXCR1_RXFCE); /* enable flow control */

	rxctrl.rxcr2 |= RXCR2_SRDBL_FRAME;

	/* schedule work to do the actual set of the data if needed */

	spin_lock(&mse->statelock);

	if (memcmp(&rxctrl, &mse->rxctrl, sizeof(rxctrl)) != 0) {
		memcpy(&mse->rxctrl, &rxctrl, sizeof(mse->rxctrl));
		schedule_work(&mse->rxctrl_work);
	}

	spin_unlock(&mse->statelock);
}

static const struct net_device_ops mse102x_netdev_ops = {
	.ndo_open		= mse102x_net_open,
	.ndo_stop		= mse102x_net_stop,
	.ndo_start_xmit		= mse102x_start_xmit,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_set_rx_mode	= mse102x_set_rx_mode,
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

int mse102x_probe_common(struct net_device *netdev, struct device *dev,
			int msg_en)
{
	struct mse102x_net *mse = netdev_priv(netdev);
	unsigned cider;
	int ret;

	mse->netdev = netdev;
	mse->tx_space = 6144;

	spin_lock_init(&mse->statelock);

	INIT_WORK(&mse->rxctrl_work, mse102x_rxctrl_work);

	dev_info(dev, "message enable is %d\n", msg_en);

	/* set the default message enable */
	mse->msg_enable = netif_msg_init(msg_en, NETIF_MSG_DRV |
						NETIF_MSG_PROBE |
						NETIF_MSG_LINK);

	skb_queue_head_init(&mse->txq);

	netdev->ethtool_ops = &mse102x_ethtool_ops;
	SET_NETDEV_DEV(netdev, dev);

	dev_set_drvdata(dev, mse);

	netif_carrier_off(mse->netdev);
	netdev->if_port = IF_PORT_100BASET;
	netdev->netdev_ops = &mse102x_netdev_ops;

	/* simple check for a valid chip being connected to the bus */
	cider = mse102x_rdreg16(mse, KS_CIDER);
	if ((cider & ~CIDER_REV_MASK) != CIDER_ID) {
		dev_err(dev, "failed to read device ID\n");
		return -ENODEV;
	}

	mse102x_init_mac(mse, dev->of_node);

	ret = register_netdev(netdev);
	if (ret) {
		dev_err(dev, "failed to register network device\n");
		return ret;
	}

	return 0;
}

int mse102x_remove_common(struct device *dev)
{
	struct mse102x_net *priv = dev_get_drvdata(dev);

	if (netif_msg_drv(priv))
		dev_info(dev, "remove\n");

	unregister_netdev(priv->netdev);

	return 0;
}
