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

#include <linux/of_net.h>

#include "mse102x.h"

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
	mse102x_lock_spi(mse, &flags);

	netif_dbg(mse, ifup, mse->netdev, "opening\n");

	netif_start_queue(mse->netdev);

	netif_carrier_on(mse->netdev);

	netif_dbg(mse, ifup, mse->netdev, "network device up\n");

	mse102x_unlock_spi(mse, &flags);

	return 0;
}

static int mse102x_net_stop(struct net_device *dev)
{
	struct mse102x_net *mse = netdev_priv(dev);

	netif_info(mse, ifdown, dev, "shutting down\n");

	netif_stop_queue(dev);

	/* stop any outstanding work */
	mse102x_flush_tx_work_spi(mse);

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

int mse102x_probe_common(struct net_device *netdev, struct device *dev,
			int msg_en)
{
	struct mse102x_net *mse = netdev_priv(netdev);
	int ret;

	mse->netdev = netdev;

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

	mse102x_init_mac(mse, dev->of_node);

	ret = register_netdev(netdev);
	if (ret) {
		dev_err(dev, "failed to register network device: %d\n", ret);
		return ret;
	}

	dev_info(dev, "%s: Success\n", __func__);

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
