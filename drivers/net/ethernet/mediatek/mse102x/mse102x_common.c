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
#include <linux/mii.h>
#include <linux/regulator/consumer.h>

#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_net.h>

#include "mse102x.h"

static void mse102x_lock(struct mse102x_net *ks, unsigned long *flags)
{
	ks->lock(ks, flags);
}

static void mse102x_unlock(struct mse102x_net *ks, unsigned long *flags)
{
	ks->unlock(ks, flags);
}

static void mse102x_wrreg16(struct mse102x_net *ks, unsigned int reg,
			   unsigned int val)
{
	ks->wrreg16(ks, reg, val);
}

static unsigned int mse102x_rdreg16(struct mse102x_net *ks,
				   unsigned int reg)
{
	return ks->rdreg16(ks, reg);
}

static void mse102x_soft_reset(struct mse102x_net *ks, unsigned op)
{
	mse102x_wrreg16(ks, KS_GRR, op);
	mdelay(1);	/* wait a short time to effect reset */
	mse102x_wrreg16(ks, KS_GRR, 0);
	mdelay(1);	/* wait for condition to clear */
}

static void mse102x_set_powermode(struct mse102x_net *ks, unsigned pwrmode)
{
	unsigned pmecr;

	netif_dbg(ks, hw, ks->netdev, "setting power mode %d\n", pwrmode);

	pmecr = mse102x_rdreg16(ks, KS_PMECR);
	pmecr &= ~PMECR_PM_MASK;
	pmecr |= pwrmode;

	mse102x_wrreg16(ks, KS_PMECR, pmecr);
}

static int mse102x_write_mac_addr(struct net_device *dev)
{
	struct mse102x_net *ks = netdev_priv(dev);
	unsigned long flags;
	u16 val;
	int i;

	mse102x_lock(ks, &flags);

	/*
	 * Wake up chip in case it was powered off when stopped; otherwise,
	 * the first write to the MAC address does not take effect.
	 */
	mse102x_set_powermode(ks, PMECR_PM_NORMAL);

	for (i = 0; i < ETH_ALEN; i += 2) {
		val = (dev->dev_addr[i] << 8) | dev->dev_addr[i + 1];
		mse102x_wrreg16(ks, KS_MAR(i), val);
	}

	if (!netif_running(dev))
		mse102x_set_powermode(ks, PMECR_PM_SOFTDOWN);

	mse102x_unlock(ks, &flags);

	return 0;
}

static void mse102x_read_mac_addr(struct net_device *dev)
{
	struct mse102x_net *ks = netdev_priv(dev);
	unsigned long flags;
	u16 reg;
	int i;

	mse102x_lock(ks, &flags);

	for (i = 0; i < ETH_ALEN; i += 2) {
		reg = mse102x_rdreg16(ks, KS_MAR(i));
		dev->dev_addr[i] = reg >> 8;
		dev->dev_addr[i + 1] = reg & 0xff;
	}

	mse102x_unlock(ks, &flags);
}

static void mse102x_init_mac(struct mse102x_net *ks, struct device_node *np)
{
	struct net_device *dev = ks->netdev;
	const u8 *mac_addr;

	mac_addr = of_get_mac_address(np);
	if (!IS_ERR(mac_addr)) {
		ether_addr_copy(dev->dev_addr, mac_addr);
		mse102x_write_mac_addr(dev);
		return;
	}

	if (ks->rc_ccr & CCR_EEPROM) {
		mse102x_read_mac_addr(dev);
		if (is_valid_ether_addr(dev->dev_addr))
			return;

		netdev_err(ks->netdev, "invalid mac address read %pM\n",
				dev->dev_addr);
	}

	eth_hw_addr_random(dev);
	mse102x_write_mac_addr(dev);
}

static void mse102x_dbg_dumpkkt(struct mse102x_net *ks, u8 *rxpkt)
{
	netdev_dbg(ks->netdev,
		   "pkt %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x\n",
		   rxpkt[4], rxpkt[5], rxpkt[6], rxpkt[7],
		   rxpkt[8], rxpkt[9], rxpkt[10], rxpkt[11],
		   rxpkt[12], rxpkt[13], rxpkt[14], rxpkt[15]);
}

static void mse102x_rx_skb(struct mse102x_net *ks, struct sk_buff *skb)
{
	ks->rx_skb(ks, skb);
}

static void mse102x_rx_pkts(struct mse102x_net *ks)
{
	struct sk_buff *skb;
	unsigned rxfc;
	unsigned rxlen;
	unsigned rxstat;
	u8 *rxpkt;

	rxfc = (mse102x_rdreg16(ks, KS_RXFCTR) >> 8) & 0xff;

	netif_dbg(ks, rx_status, ks->netdev,
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
		rxstat = mse102x_rdreg16(ks, KS_RXFHSR);
		rxlen = mse102x_rdreg16(ks, KS_RXFHBCR) & RXFHBCR_CNT_MASK;

		netif_dbg(ks, rx_status, ks->netdev,
			  "rx: stat 0x%04x, len 0x%04x\n", rxstat, rxlen);

		/* the length of the packet includes the 32bit CRC */

		/* set dma read address */
		mse102x_wrreg16(ks, KS_RXFDPR, RXFDPR_RXFPAI | 0x00);

		/* start DMA access */
		mse102x_wrreg16(ks, KS_RXQCR, ks->rc_rxqcr | RXQCR_SDA);

		if (rxlen > 4) {
			unsigned int rxalign;

			rxlen -= 4;
			rxalign = ALIGN(rxlen, 4);
			skb = netdev_alloc_skb_ip_align(ks->netdev, rxalign);
			if (skb) {

				/* 4 bytes of status header + 4 bytes of
				 * garbage: we put them before ethernet
				 * header, so that they are copied,
				 * but ignored.
				 */

				rxpkt = skb_put(skb, rxlen) - 8;

				ks->rdfifo(ks, rxpkt, rxalign + 8);

				if (netif_msg_pktdata(ks))
					mse102x_dbg_dumpkkt(ks, rxpkt);

				skb->protocol = eth_type_trans(skb, ks->netdev);
				mse102x_rx_skb(ks, skb);

				ks->netdev->stats.rx_packets++;
				ks->netdev->stats.rx_bytes += rxlen;
			}
		}

		/* end DMA access and dequeue packet */
		mse102x_wrreg16(ks, KS_RXQCR, ks->rc_rxqcr | RXQCR_RRXEF);
	}
}

static irqreturn_t mse102x_irq(int irq, void *_ks)
{
	struct mse102x_net *ks = _ks;
	unsigned handled = 0;
	unsigned long flags;
	unsigned int status;

	mse102x_lock(ks, &flags);

	status = mse102x_rdreg16(ks, KS_ISR);

	netif_dbg(ks, intr, ks->netdev,
		  "%s: status 0x%04x\n", __func__, status);

	if (status & IRQ_LCI)
		handled |= IRQ_LCI;

	if (status & IRQ_LDI) {
		u16 pmecr = mse102x_rdreg16(ks, KS_PMECR);
		pmecr &= ~PMECR_WKEVT_MASK;
		mse102x_wrreg16(ks, KS_PMECR, pmecr | PMECR_WKEVT_LINK);

		handled |= IRQ_LDI;
	}

	if (status & IRQ_RXPSI)
		handled |= IRQ_RXPSI;

	if (status & IRQ_TXI) {
		handled |= IRQ_TXI;

		/* no lock here, tx queue should have been stopped */

		/* update our idea of how much tx space is available to the
		 * system */
		ks->tx_space = mse102x_rdreg16(ks, KS_TXMIR);

		netif_dbg(ks, intr, ks->netdev,
			  "%s: txspace %d\n", __func__, ks->tx_space);
	}

	if (status & IRQ_RXI)
		handled |= IRQ_RXI;

	if (status & IRQ_SPIBEI) {
		netdev_err(ks->netdev, "%s: spi bus error\n", __func__);
		handled |= IRQ_SPIBEI;
	}

	mse102x_wrreg16(ks, KS_ISR, handled);

	if (status & IRQ_RXI) {
		/* the datasheet says to disable the rx interrupt during
		 * packet read-out, however we're masking the interrupt
		 * from the device so do not bother masking just the RX
		 * from the device. */

		mse102x_rx_pkts(ks);
	}

	/* if something stopped the rx process, probably due to wanting
	 * to change the rx settings, then do something about restarting
	 * it. */
	if (status & IRQ_RXPSI) {
		struct mse102x_rxctrl *rxc = &ks->rxctrl;

		/* update the multicast hash table */
		mse102x_wrreg16(ks, KS_MAHTR0, rxc->mchash[0]);
		mse102x_wrreg16(ks, KS_MAHTR1, rxc->mchash[1]);
		mse102x_wrreg16(ks, KS_MAHTR2, rxc->mchash[2]);
		mse102x_wrreg16(ks, KS_MAHTR3, rxc->mchash[3]);

		mse102x_wrreg16(ks, KS_RXCR2, rxc->rxcr2);
		mse102x_wrreg16(ks, KS_RXCR1, rxc->rxcr1);
	}

	mse102x_unlock(ks, &flags);

	if (status & IRQ_LCI)
		mii_check_link(&ks->mii);

	if (status & IRQ_TXI)
		netif_wake_queue(ks->netdev);

	return IRQ_HANDLED;
}

static void mse102x_flush_tx_work(struct mse102x_net *ks)
{
	if (ks->flush_tx_work)
		ks->flush_tx_work(ks);
}

static int mse102x_net_open(struct net_device *dev)
{
	struct mse102x_net *ks = netdev_priv(dev);
	unsigned long flags;
	int ret;

	ret = request_threaded_irq(dev->irq, NULL, mse102x_irq,
				   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				   dev->name, ks);
	if (ret < 0) {
		netdev_err(dev, "failed to get irq\n");
		return ret;
	}

	/* lock the card, even if we may not actually be doing anything
	 * else at the moment */
	mse102x_lock(ks, &flags);

	netif_dbg(ks, ifup, ks->netdev, "opening\n");

	/* bring chip out of any power saving mode it was in */
	mse102x_set_powermode(ks, PMECR_PM_NORMAL);

	/* issue a soft reset to the RX/TX QMU to put it into a known
	 * state. */
	mse102x_soft_reset(ks, GRR_QMU);

	/* setup transmission parameters */

	mse102x_wrreg16(ks, KS_TXCR, (TXCR_TXE | /* enable transmit process */
				     TXCR_TXPE | /* pad to min length */
				     TXCR_TXCRC | /* add CRC */
				     TXCR_TXFCE)); /* enable flow control */

	/* auto-increment tx data, reset tx pointer */
	mse102x_wrreg16(ks, KS_TXFDPR, TXFDPR_TXFPAI);

	/* setup receiver control */

	mse102x_wrreg16(ks, KS_RXCR1, (RXCR1_RXPAFMA | /*  from mac filter */
				      RXCR1_RXFCE | /* enable flow control */
				      RXCR1_RXBE | /* broadcast enable */
				      RXCR1_RXUE | /* unicast enable */
				      RXCR1_RXE)); /* enable rx block */

	/* transfer entire frames out in one go */
	mse102x_wrreg16(ks, KS_RXCR2, RXCR2_SRDBL_FRAME);

	/* set receive counter timeouts */
	mse102x_wrreg16(ks, KS_RXDTTR, 1000); /* 1ms after first frame to IRQ */
	mse102x_wrreg16(ks, KS_RXDBCTR, 4096); /* >4Kbytes in buffer to IRQ */
	mse102x_wrreg16(ks, KS_RXFCTR, 10);  /* 10 frames to IRQ */

	ks->rc_rxqcr = (RXQCR_RXFCTE |  /* IRQ on frame count exceeded */
			RXQCR_RXDBCTE | /* IRQ on byte count exceeded */
			RXQCR_RXDTTE);  /* IRQ on time exceeded */

	mse102x_wrreg16(ks, KS_RXQCR, ks->rc_rxqcr);

	/* clear then enable interrupts */
	mse102x_wrreg16(ks, KS_ISR, ks->rc_ier);
	mse102x_wrreg16(ks, KS_IER, ks->rc_ier);

	netif_start_queue(ks->netdev);

	netif_dbg(ks, ifup, ks->netdev, "network device up\n");

	mse102x_unlock(ks, &flags);
	mii_check_link(&ks->mii);
	return 0;
}

static int mse102x_net_stop(struct net_device *dev)
{
	struct mse102x_net *ks = netdev_priv(dev);
	unsigned long flags;

	netif_info(ks, ifdown, dev, "shutting down\n");

	netif_stop_queue(dev);

	mse102x_lock(ks, &flags);
	/* turn off the IRQs and ack any outstanding */
	mse102x_wrreg16(ks, KS_IER, 0x0000);
	mse102x_wrreg16(ks, KS_ISR, 0xffff);
	mse102x_unlock(ks, &flags);

	/* stop any outstanding work */
	mse102x_flush_tx_work(ks);
	flush_work(&ks->rxctrl_work);

	mse102x_lock(ks, &flags);
	/* shutdown RX process */
	mse102x_wrreg16(ks, KS_RXCR1, 0x0000);

	/* shutdown TX process */
	mse102x_wrreg16(ks, KS_TXCR, 0x0000);

	/* set powermode to soft power down to save power */
	mse102x_set_powermode(ks, PMECR_PM_SOFTDOWN);
	mse102x_unlock(ks, &flags);

	/* ensure any queued tx buffers are dumped */
	while (!skb_queue_empty(&ks->txq)) {
		struct sk_buff *txb = skb_dequeue(&ks->txq);

		netif_dbg(ks, ifdown, ks->netdev,
			  "%s: freeing txb %p\n", __func__, txb);

		dev_kfree_skb(txb);
	}

	free_irq(dev->irq, ks);

	return 0;
}

static netdev_tx_t mse102x_start_xmit(struct sk_buff *skb,
				     struct net_device *dev)
{
	struct mse102x_net *ks = netdev_priv(dev);

	return ks->start_xmit(skb, dev);
}

static void mse102x_rxctrl_work(struct work_struct *work)
{
	struct mse102x_net *ks = container_of(work, struct mse102x_net, rxctrl_work);
	unsigned long flags;

	mse102x_lock(ks, &flags);

	/* need to shutdown RXQ before modifying filter parameters */
	mse102x_wrreg16(ks, KS_RXCR1, 0x00);

	mse102x_unlock(ks, &flags);
}

static void mse102x_set_rx_mode(struct net_device *dev)
{
	struct mse102x_net *ks = netdev_priv(dev);
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

	spin_lock(&ks->statelock);

	if (memcmp(&rxctrl, &ks->rxctrl, sizeof(rxctrl)) != 0) {
		memcpy(&ks->rxctrl, &rxctrl, sizeof(ks->rxctrl));
		schedule_work(&ks->rxctrl_work);
	}

	spin_unlock(&ks->statelock);
}

static int mse102x_set_mac_address(struct net_device *dev, void *addr)
{
	struct sockaddr *sa = addr;

	if (netif_running(dev))
		return -EBUSY;

	if (!is_valid_ether_addr(sa->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(dev->dev_addr, sa->sa_data, ETH_ALEN);
	return mse102x_write_mac_addr(dev);
}

static int mse102x_net_ioctl(struct net_device *dev, struct ifreq *req, int cmd)
{
	struct mse102x_net *ks = netdev_priv(dev);

	if (!netif_running(dev))
		return -EINVAL;

	return generic_mii_ioctl(&ks->mii, if_mii(req), cmd, NULL);
}

static const struct net_device_ops mse102x_netdev_ops = {
	.ndo_open		= mse102x_net_open,
	.ndo_stop		= mse102x_net_stop,
	.ndo_do_ioctl		= mse102x_net_ioctl,
	.ndo_start_xmit		= mse102x_start_xmit,
	.ndo_set_mac_address	= mse102x_set_mac_address,
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
	struct mse102x_net *ks = netdev_priv(dev);
	return ks->msg_enable;
}

static void mse102x_set_msglevel(struct net_device *dev, u32 to)
{
	struct mse102x_net *ks = netdev_priv(dev);
	ks->msg_enable = to;
}

static int mse102x_get_link_ksettings(struct net_device *dev,
				     struct ethtool_link_ksettings *cmd)
{
	struct mse102x_net *ks = netdev_priv(dev);

	mii_ethtool_get_link_ksettings(&ks->mii, cmd);

	return 0;
}

static int mse102x_set_link_ksettings(struct net_device *dev,
				     const struct ethtool_link_ksettings *cmd)
{
	struct mse102x_net *ks = netdev_priv(dev);
	return mii_ethtool_set_link_ksettings(&ks->mii, cmd);
}

static u32 mse102x_get_link(struct net_device *dev)
{
	struct mse102x_net *ks = netdev_priv(dev);
	return mii_link_ok(&ks->mii);
}

static int mse102x_nway_reset(struct net_device *dev)
{
	struct mse102x_net *ks = netdev_priv(dev);
	return mii_nway_restart(&ks->mii);
}

/* EEPROM support */

static void mse102x_eeprom_regread(struct eeprom_93cx6 *ee)
{
	struct mse102x_net *ks = ee->data;
	unsigned val;

	val = mse102x_rdreg16(ks, KS_EEPCR);

	ee->reg_data_out = (val & EEPCR_EESB) ? 1 : 0;
	ee->reg_data_clock = (val & EEPCR_EESCK) ? 1 : 0;
	ee->reg_chip_select = (val & EEPCR_EECS) ? 1 : 0;
}

static void mse102x_eeprom_regwrite(struct eeprom_93cx6 *ee)
{
	struct mse102x_net *ks = ee->data;
	unsigned val = EEPCR_EESA;	/* default - eeprom access on */

	if (ee->drive_data)
		val |= EEPCR_EESRWA;
	if (ee->reg_data_in)
		val |= EEPCR_EEDO;
	if (ee->reg_data_clock)
		val |= EEPCR_EESCK;
	if (ee->reg_chip_select)
		val |= EEPCR_EECS;

	mse102x_wrreg16(ks, KS_EEPCR, val);
}

static int mse102x_eeprom_claim(struct mse102x_net *ks)
{
	/* start with clock low, cs high */
	mse102x_wrreg16(ks, KS_EEPCR, EEPCR_EESA | EEPCR_EECS);
	return 0;
}

static void mse102x_eeprom_release(struct mse102x_net *ks)
{
	unsigned val = mse102x_rdreg16(ks, KS_EEPCR);

	mse102x_wrreg16(ks, KS_EEPCR, val & ~EEPCR_EESA);
}

#define KS_EEPROM_MAGIC (0x00008851)

static int mse102x_set_eeprom(struct net_device *dev,
			     struct ethtool_eeprom *ee, u8 *data)
{
	struct mse102x_net *ks = netdev_priv(dev);
	int offset = ee->offset;
	unsigned long flags;
	int len = ee->len;
	u16 tmp;

	/* currently only support byte writing */
	if (len != 1)
		return -EINVAL;

	if (ee->magic != KS_EEPROM_MAGIC)
		return -EINVAL;

	if (!(ks->rc_ccr & CCR_EEPROM))
		return -ENOENT;

	mse102x_lock(ks, &flags);

	mse102x_eeprom_claim(ks);

	eeprom_93cx6_wren(&ks->eeprom, true);

	/* ethtool currently only supports writing bytes, which means
	 * we have to read/modify/write our 16bit EEPROMs */

	eeprom_93cx6_read(&ks->eeprom, offset/2, &tmp);

	if (offset & 1) {
		tmp &= 0xff;
		tmp |= *data << 8;
	} else {
		tmp &= 0xff00;
		tmp |= *data;
	}

	eeprom_93cx6_write(&ks->eeprom, offset/2, tmp);
	eeprom_93cx6_wren(&ks->eeprom, false);

	mse102x_eeprom_release(ks);
	mse102x_unlock(ks, &flags);

	return 0;
}

static int mse102x_get_eeprom(struct net_device *dev,
			     struct ethtool_eeprom *ee, u8 *data)
{
	struct mse102x_net *ks = netdev_priv(dev);
	int offset = ee->offset;
	unsigned long flags;
	int len = ee->len;

	/* must be 2 byte aligned */
	if (len & 1 || offset & 1)
		return -EINVAL;

	if (!(ks->rc_ccr & CCR_EEPROM))
		return -ENOENT;

	mse102x_lock(ks, &flags);

	mse102x_eeprom_claim(ks);

	ee->magic = KS_EEPROM_MAGIC;

	eeprom_93cx6_multiread(&ks->eeprom, offset/2, (__le16 *)data, len/2);
	mse102x_eeprom_release(ks);
	mse102x_unlock(ks, &flags);

	return 0;
}

static int mse102x_get_eeprom_len(struct net_device *dev)
{
	struct mse102x_net *ks = netdev_priv(dev);

	/* currently, we assume it is an 93C46 attached, so return 128 */
	return ks->rc_ccr & CCR_EEPROM ? 128 : 0;
}

static const struct ethtool_ops mse102x_ethtool_ops = {
	.get_drvinfo	= mse102x_get_drvinfo,
	.get_msglevel	= mse102x_get_msglevel,
	.set_msglevel	= mse102x_set_msglevel,
	.get_link	= mse102x_get_link,
	.nway_reset	= mse102x_nway_reset,
	.get_eeprom_len	= mse102x_get_eeprom_len,
	.get_eeprom	= mse102x_get_eeprom,
	.set_eeprom	= mse102x_set_eeprom,
	.get_link_ksettings = mse102x_get_link_ksettings,
	.set_link_ksettings = mse102x_set_link_ksettings,
};

/* MII interface controls */

static int mse102x_phy_reg(int reg)
{
	switch (reg) {
	case MII_BMCR:
		return KS_P1MBCR;
	case MII_BMSR:
		return KS_P1MBSR;
	case MII_PHYSID1:
		return KS_PHY1ILR;
	case MII_PHYSID2:
		return KS_PHY1IHR;
	case MII_ADVERTISE:
		return KS_P1ANAR;
	case MII_LPA:
		return KS_P1ANLPR;
	}

	return 0x0;
}

static int mse102x_phy_read(struct net_device *dev, int phy_addr, int reg)
{
	struct mse102x_net *ks = netdev_priv(dev);
	unsigned long flags;
	int ksreg;
	int result;

	ksreg = mse102x_phy_reg(reg);
	if (!ksreg)
		return 0x0;	/* no error return allowed, so use zero */

	mse102x_lock(ks, &flags);
	result = mse102x_rdreg16(ks, ksreg);
	mse102x_unlock(ks, &flags);

	return result;
}

static void mse102x_phy_write(struct net_device *dev,
			     int phy, int reg, int value)
{
	struct mse102x_net *ks = netdev_priv(dev);
	unsigned long flags;
	int ksreg;

	ksreg = mse102x_phy_reg(reg);
	if (ksreg) {
		mse102x_lock(ks, &flags);
		mse102x_wrreg16(ks, ksreg, value);
		mse102x_unlock(ks, &flags);
	}
}

static int mse102x_read_selftest(struct mse102x_net *ks)
{
	unsigned both_done = MBIR_TXMBF | MBIR_RXMBF;
	int ret = 0;
	unsigned rd;

	rd = mse102x_rdreg16(ks, KS_MBIR);

	if ((rd & both_done) != both_done) {
		netdev_warn(ks->netdev, "Memory selftest not finished\n");
		return 0;
	}

	if (rd & MBIR_TXMBFA) {
		netdev_err(ks->netdev, "TX memory selftest fail\n");
		ret |= 1;
	}

	if (rd & MBIR_RXMBFA) {
		netdev_err(ks->netdev, "RX memory selftest fail\n");
		ret |= 2;
	}

	return 0;
}

/* driver bus management functions */

#ifdef CONFIG_PM_SLEEP

int mse102x_suspend(struct device *dev)
{
	struct mse102x_net *ks = dev_get_drvdata(dev);
	struct net_device *netdev = ks->netdev;

	if (netif_running(netdev)) {
		netif_device_detach(netdev);
		mse102x_net_stop(netdev);
	}

	return 0;
}

int mse102x_resume(struct device *dev)
{
	struct mse102x_net *ks = dev_get_drvdata(dev);
	struct net_device *netdev = ks->netdev;

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
	struct mse102x_net *ks = netdev_priv(netdev);
	unsigned cider;
	int gpio;
	int ret;

	ks->netdev = netdev;
	ks->tx_space = 6144;

	gpio = of_get_named_gpio_flags(dev->of_node, "reset-gpios", 0, NULL);
	if (gpio == -EPROBE_DEFER)
		return gpio;

	ks->gpio = gpio;
	if (gpio_is_valid(gpio)) {
		ret = devm_gpio_request_one(dev, gpio,
					    GPIOF_OUT_INIT_LOW, "mse102x_rst_n");
		if (ret) {
			dev_err(dev, "reset gpio request failed\n");
			return ret;
		}
	}

	ks->vdd_io = devm_regulator_get(dev, "vdd-io");
	if (IS_ERR(ks->vdd_io)) {
		ret = PTR_ERR(ks->vdd_io);
		goto err_reg_io;
	}

	ret = regulator_enable(ks->vdd_io);
	if (ret) {
		dev_err(dev, "regulator vdd_io enable fail: %d\n", ret);
		goto err_reg_io;
	}

	ks->vdd_reg = devm_regulator_get(dev, "vdd");
	if (IS_ERR(ks->vdd_reg)) {
		ret = PTR_ERR(ks->vdd_reg);
		goto err_reg;
	}

	ret = regulator_enable(ks->vdd_reg);
	if (ret) {
		dev_err(dev, "regulator vdd enable fail: %d\n", ret);
		goto err_reg;
	}

	if (gpio_is_valid(gpio)) {
		usleep_range(10000, 11000);
		gpio_set_value(gpio, 1);
	}

	spin_lock_init(&ks->statelock);

	INIT_WORK(&ks->rxctrl_work, mse102x_rxctrl_work);

	/* setup EEPROM state */
	ks->eeprom.data = ks;
	ks->eeprom.width = PCI_EEPROM_WIDTH_93C46;
	ks->eeprom.register_read = mse102x_eeprom_regread;
	ks->eeprom.register_write = mse102x_eeprom_regwrite;

	/* setup mii state */
	ks->mii.dev		= netdev;
	ks->mii.phy_id		= 1,
	ks->mii.phy_id_mask	= 1;
	ks->mii.reg_num_mask	= 0xf;
	ks->mii.mdio_read	= mse102x_phy_read;
	ks->mii.mdio_write	= mse102x_phy_write;

	dev_info(dev, "message enable is %d\n", msg_en);

	/* set the default message enable */
	ks->msg_enable = netif_msg_init(msg_en, NETIF_MSG_DRV |
						NETIF_MSG_PROBE |
						NETIF_MSG_LINK);

	skb_queue_head_init(&ks->txq);

	netdev->ethtool_ops = &mse102x_ethtool_ops;
	SET_NETDEV_DEV(netdev, dev);

	dev_set_drvdata(dev, ks);

	netif_carrier_off(ks->netdev);
	netdev->if_port = IF_PORT_100BASET;
	netdev->netdev_ops = &mse102x_netdev_ops;

	/* issue a global soft reset to reset the device. */
	mse102x_soft_reset(ks, GRR_GSR);

	/* simple check for a valid chip being connected to the bus */
	cider = mse102x_rdreg16(ks, KS_CIDER);
	if ((cider & ~CIDER_REV_MASK) != CIDER_ID) {
		dev_err(dev, "failed to read device ID\n");
		ret = -ENODEV;
		goto err_id;
	}

	/* cache the contents of the CCR register for EEPROM, etc. */
	ks->rc_ccr = mse102x_rdreg16(ks, KS_CCR);

	mse102x_read_selftest(ks);
	mse102x_init_mac(ks, dev->of_node);

	ret = register_netdev(netdev);
	if (ret) {
		dev_err(dev, "failed to register network device\n");
		goto err_netdev;
	}

	netdev_info(netdev, "revision %d, MAC %pM, IRQ %d, %s EEPROM\n",
		    CIDER_REV_GET(cider), netdev->dev_addr, netdev->irq,
		    ks->rc_ccr & CCR_EEPROM ? "has" : "no");

	return 0;

err_netdev:
err_id:
	if (gpio_is_valid(gpio))
		gpio_set_value(gpio, 0);
	regulator_disable(ks->vdd_reg);
err_reg:
	regulator_disable(ks->vdd_io);
err_reg_io:
	return ret;
}

int mse102x_remove_common(struct device *dev)
{
	struct mse102x_net *priv = dev_get_drvdata(dev);

	if (netif_msg_drv(priv))
		dev_info(dev, "remove\n");

	unregister_netdev(priv->netdev);
	if (gpio_is_valid(priv->gpio))
		gpio_set_value(priv->gpio, 0);
	regulator_disable(priv->vdd_reg);
	regulator_disable(priv->vdd_io);

	return 0;
}
