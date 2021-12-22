// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021 Davicom Semiconductor,Inc.
 * Davicom DM9051 SPI Fast Ethernet Linux driver
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/phy.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/cache.h>
#include <linux/crc32.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/iopoll.h>
#include <linux/of_gpio.h>
#include <linux/spi/spi.h>

#include "dm9051.h"

/* spi low level code */
static inline int dm9051_xfer(struct board_info *db, u8 cmd, u8 *txb, u8 *rxb, unsigned int len)
{
	struct device *dev = &db->spidev->dev;
	int ret;

	db->cmd[0] = cmd;
	db->spi_xfer2[0].tx_buf = &db->cmd[0];
	db->spi_xfer2[0].rx_buf = NULL;
	db->spi_xfer2[0].len = 1;
	if (!rxb) {
		db->spi_xfer2[1].tx_buf = txb;
		db->spi_xfer2[1].rx_buf = NULL;
		db->spi_xfer2[1].len = len;
	} else {
		db->spi_xfer2[1].tx_buf = txb;
		db->spi_xfer2[1].rx_buf = rxb;
		db->spi_xfer2[1].len = len;
	}
	ret = spi_sync(db->spidev, &db->spi_msg2);
	if (ret < 0)
		dev_err(dev, "spi burst cmd 0x%02x, ret=%d\n", cmd, ret);
	return ret;
}

static u8 dm9051_ior(struct board_info *db, unsigned int reg)
{
	int ret;
	u8 rxb[1];

	ret = dm9051_xfer(db, DM_SPI_RD | reg, NULL, rxb, 1);
	if (ret < 0)
		return 0xff;
	return rxb[0];
}

static void dm9051_iow(struct board_info *db, unsigned int reg, unsigned int val)
{
	u8 txb[1];

	txb[0] = val;
	dm9051_xfer(db, DM_SPI_WR | reg, txb, NULL, 1);
}

static int dm9051_inblk(struct board_info *db, u8 *buff, unsigned int len)
{
	int ret;
	u8 txb[1];

	ret = dm9051_xfer(db, DM_SPI_RD | DM_SPI_MRCMD, txb, buff, len);
	return ret;
}

static void dm9051_dumpblk(struct board_info *db, unsigned int len)
{
	while (len--)
		dm9051_ior(db, DM_SPI_MRCMD);
}

static int dm9051_outblk(struct board_info *db, u8 *buff, unsigned int len)
{
	int ret;

	ret = dm9051_xfer(db, DM_SPI_WR | DM_SPI_MWCMD, buff, NULL, len);
	return ret;
}

static int dm9051_phy_read(struct board_info *db, int reg, int *pvalue)
{
	int ret;
	u8 check_val;

	dm9051_iow(db, DM9051_EPAR, DM9051_PHY | reg);
	dm9051_iow(db, DM9051_EPCR, EPCR_ERPRR | EPCR_EPOS);
	ret = read_poll_timeout(dm9051_ior, check_val, !(check_val & EPCR_ERRE), 100, 10000,
				true, db, DM9051_EPCR);
	dm9051_iow(db, DM9051_EPCR, 0x0);
	if (ret) {
		netdev_err(db->ndev, "timeout read phy register\n");
		return -ETIMEDOUT;
	}
	*pvalue = (dm9051_ior(db, DM9051_EPDRH) << 8) | dm9051_ior(db, DM9051_EPDRL);
	return ret;
}

static int dm9051_phy_write(struct board_info *db, int reg, int value)
{
	int ret;
	u8 check_val;

	if (reg == MII_BMCR && !(value & 0x0800))
		dev_info(&db->spidev->dev, "[phy_write/inline delay..] %d %04x\n", MII_BMCR, value);

	dm9051_iow(db, DM9051_EPAR, DM9051_PHY | reg);
	dm9051_iow(db, DM9051_EPDRL, value);
	dm9051_iow(db, DM9051_EPDRH, value >> 8);
	dm9051_iow(db, DM9051_EPCR, EPCR_EPOS | EPCR_ERPRW);
	ret = read_poll_timeout(dm9051_ior, check_val, !(check_val & EPCR_ERRE), 100, 10000,
				true, db, DM9051_EPCR);
	dm9051_iow(db, DM9051_EPCR, 0x0);

	if (reg == MII_BMCR && !(value & 0x0800))
		mdelay(1); /* need for activate phyxcer */

	if (ret) {
		netdev_err(db->ndev, "timeout write phy register\n");
		return -ETIMEDOUT;
	}
	return ret;
}

/* Read a word data from SROM
 */
static void dm9051_read_eeprom(struct board_info *db, int offset, u8 *to)
{
	int ret;
	u8 check_val;

	mutex_lock(&db->addr_lock);
	dm9051_iow(db, DM9051_EPAR, offset);
	dm9051_iow(db, DM9051_EPCR, EPCR_ERPRR);
	ret = read_poll_timeout(dm9051_ior, check_val, !(check_val & EPCR_ERRE), 100, 10000,
				true, db, DM9051_EPCR);
	dm9051_iow(db, DM9051_EPCR, 0x0);
	if (!ret) {
		to[0] = dm9051_ior(db, DM9051_EPDRL);
		to[1] = dm9051_ior(db, DM9051_EPDRH);
	} else {
		to[0] = DM9051_EEPROM_NULLVALUE & 0xff;
		to[1] = DM9051_EEPROM_NULLVALUE >> 8;
		netdev_err(db->ndev, "timeout read eeprom\n");
	}
	mutex_unlock(&db->addr_lock);
}

/* Write a word data to SROM
 */
static void dm9051_write_eeprom(struct board_info *db, int offset, u8 *data)
{
	int ret;
	u8 check_val;

	mutex_lock(&db->addr_lock);
	dm9051_iow(db, DM9051_EPAR, offset);
	dm9051_iow(db, DM9051_EPDRH, data[1]);
	dm9051_iow(db, DM9051_EPDRL, data[0]);
	dm9051_iow(db, DM9051_EPCR, EPCR_WEP | EPCR_ERPRW);
	ret = read_poll_timeout(dm9051_ior, check_val, !(check_val & EPCR_ERRE), 100, 10000,
				true, db, DM9051_EPCR);
	dm9051_iow(db, DM9051_EPCR, 0);
	if (ret)
		netdev_err(db->ndev, "timeout write eeprom\n");
	mutex_unlock(&db->addr_lock);
}

static int dm9051_mdio_read(struct mii_bus *mdiobus, int phy_id, int reg)
{
	struct board_info *db = mdiobus->priv;
	int val, ret;

	if (phy_id == DM9051_PHY_ID) {
		mutex_lock(&db->addr_lock);
		ret = dm9051_phy_read(db, reg, &val);
		mutex_unlock(&db->addr_lock);
		if (ret)
			return ret;
		return val;
	}

	return 0xffff;
}

static int dm9051_mdio_write(struct mii_bus *mdiobus, int phy_id, int reg, u16 val)
{
	struct board_info *db = mdiobus->priv;
	int ret;

	if (phy_id == DM9051_PHY_ID) {

		dev_info(&db->spidev->dev, "_mdio_write %d %04x(mdiobus)\n", reg, val);

		mutex_lock(&db->addr_lock);
		ret = dm9051_phy_write(db, reg, val);
		mutex_unlock(&db->addr_lock);
		if (ret)
			return ret;
		return 0;
	}

	return -ENODEV;
}

static unsigned int dm9051_chipid(struct board_info *db)
{
	struct device *dev = &db->spidev->dev;
	unsigned int id;

	id = (unsigned int)dm9051_ior(db, DM9051_PIDH) << 8;
	id |= dm9051_ior(db, DM9051_PIDL);
	if (id == DM9051_ID) {
		dev_info(dev, "chip %04x found !\n", id);
		return id;
	}

	dev_info(dev, "chipid error as %04x !\n", id);
	return id;
}

static void dm9051_reset(struct board_info *db)
{
	mdelay(2); /* need before NCR_RST */
	dm9051_iow(db, DM9051_NCR, NCR_RST); /* NCR reset */
	mdelay(1);
	dm9051_iow(db, DM9051_MBNDRY, MBNDRY_BYTE); /* MemBound */
	mdelay(1);

	// work around for if ever phy power down
#if 0
	if (1) {
		//int ret = 
		dm9051_phy_write(db, 0, 0x8000);
	}
#endif
}

static void dm9051_restart_fifo_rst(struct board_info *db)
{
	u8 fcr = 0;

	db->bc.DO_FIFO_RST_counter++;

	if (db->fl.fc_rx)
		fcr |= FCR_BKPM | FCR_FLCE;
	if (db->fl.fc_tx)
		fcr |= FCR_TXPEN;

//	printk("init fcr %02x\n", fcr);
//.	dm9051_iow(db, DM9051_FCR, fcr); /* init and/or save pause param FlowCtrl */
	dm9051_iow(db, DM9051_PPCR, PPCR_PAUSE_COUNT); /* Pause Pkt Count */
	dm9051_iow(db, DM9051_LMCR, db->lcr_all); /* LEDMode1 */
	dm9051_iow(db, DM9051_INTCR, INTCR_POL_LOW); /* INTCR */
}

#if 0
static void dm9051_phy_restart(struct board_info *db)
{
	/* configure phy advertised pause support */
	phy_set_asym_pause(db->phydev, db->fl.fc_rx, db->fl.fc_tx);
	phy_start(db->phydev);
}
#endif

static void dm9051_handle_link_change(struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);
	u8 fcr = 0;
	int lcl_adv, rmt_adv;

	phy_print_status(ndev->phydev);

	/* only write pause settings to mac. since mac and phy are integrated
	 * together, such as link state, speed and duplex are sync already
	 */
	if (ndev->phydev->link) {
		//int ret;
		//mutex_lock(&db->addr_lock);
		//ret = dm9051_phy_read(db, MII_ADVERTISE, &lcl_adv);
		//if (!ret)
		//	ret = dm9051_phy_read(db, MII_LPA, &rmt_adv);
		//mutex_unlock(&db->addr_lock);
		//if (ret)
		//	return;

		//printk("link change lcladv & remoteadv, %04x & %04x\n", lcl_adv, rmt_adv);

		lcl_adv = linkmode_adv_to_mii_adv_t(db->phydev->advertising);
		rmt_adv = linkmode_adv_to_mii_adv_t(db->phydev->lp_advertising);
		printk("link change lcladv & remoteadv, %04x & %04x, from phydev\n", lcl_adv, rmt_adv);

		if (lcl_adv & rmt_adv & ADVERTISE_PAUSE_CAP) {
			db->fl.fc_rx = true;
			db->fl.fc_tx = true;
			fcr = FCR_FLOW_ENABLE;
		}

		mutex_lock(&db->addr_lock);
		if (dm9051_ior(db, DM9051_FCR) != fcr) {
			printk("write link change fcr %02x\n", fcr);
			dm9051_iow(db, DM9051_FCR, fcr); /* link change, used saved pause param FlowCtrl */
		}
		printk("link change fcr %02x\n", fcr);
		mutex_unlock(&db->addr_lock);
	}
}

static int dm9051_phy_connect(struct board_info *db)
{
	char phy_id[MII_BUS_ID_SIZE + 3];

	snprintf(phy_id, MII_BUS_ID_SIZE + 3, PHY_ID_FMT,
		 db->mdiobus->id, DM9051_PHY_ID);
	db->phydev = phy_connect(db->ndev, phy_id, dm9051_handle_link_change,
				 PHY_INTERFACE_MODE_MII);

	if (IS_ERR(db->phydev))
		return PTR_ERR(db->phydev);

	return 0;
}

/* ESSENTIAL, ensure rxFifoPoint control, disable/enable the interrupt mask
 */
static void dm9051_imr_disable_lock_essential(struct board_info *db)
{
	mutex_lock(&db->addr_lock);
	dm9051_iow(db, DM9051_IMR, IMR_PAR); /* Disabe All */
	mutex_unlock(&db->addr_lock);
}

static void dm9051_imr_enable_lock_essential(struct board_info *db)
{
	mutex_lock(&db->addr_lock);
	dm9051_iow(db, DM9051_IMR, db->imr_all); /* rxp to 0xc00 */
	mutex_unlock(&db->addr_lock);
}

/* mac address is major from EEPROM
 */
static void dm9051_init_mac_addr(struct net_device *ndev, struct board_info *db)
{
	u8 addr[ETH_ALEN];
	int i;

	for (i = 0; i < ETH_ALEN; i++)
		addr[i] = dm9051_ior(db, DM9051_PAR + i);

	if (is_valid_ether_addr(addr)) {
		memcpy(ndev->dev_addr, addr, 6); //eth_hw_addr_set(ndev, addr);
		return;
	}

	eth_hw_addr_random(ndev);
	dev_dbg(&db->spidev->dev, "Use random MAC address\n");
}

/* set mac permanently
 */
static void dm9051_write_mac_lock(struct board_info *db)
{
	struct net_device *ndev = db->ndev;
	int i, oft;

	netdev_dbg(ndev, "set_mac_address %pM\n", ndev->dev_addr);

	/* write to net device and chip */
	mutex_lock(&db->addr_lock);
	for (i = 0, oft = DM9051_PAR; i < ETH_ALEN; i++, oft++)
		dm9051_iow(db, oft, ndev->dev_addr[i]);
	mutex_unlock(&db->addr_lock);

	/* write to EEPROM */
	for (i = 0; i < ETH_ALEN; i += 2)
		dm9051_write_eeprom(db, i / 2, (u8 *)&ndev->dev_addr[i]);
}

/* ethtool-ops
 */
static void
dm9051_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *info)
{
	strscpy(info->driver, DRVNAME_9051, sizeof(info->driver));
}

static void dm9051_set_msglevel(struct net_device *ndev, u32 value)
{
	struct board_info *db = to_dm9051_board(ndev);

	db->msg_enable = value;
}

static u32 dm9051_get_msglevel(struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);

	return db->msg_enable;
}

static int dm9051_get_eeprom_len(struct net_device *dev)
{
	return 128;
}

static int dm9051_get_eeprom(struct net_device *ndev,
			     struct ethtool_eeprom *ee, u8 *data)
{
	struct board_info *db = to_dm9051_board(ndev);
	int offset = ee->offset;
	int len = ee->len;
	int i;

	if ((len & 1) != 0 || (offset & 1) != 0)
		return -EINVAL;

	ee->magic = DM_EEPROM_MAGIC;

	for (i = 0; i < len; i += 2)
		dm9051_read_eeprom(db, (offset + i) / 2, data + i);
	return 0;
}

static int dm9051_set_eeprom(struct net_device *ndev,
			     struct ethtool_eeprom *ee, u8 *data)
{
	struct board_info *db = to_dm9051_board(ndev);
	int offset = ee->offset;
	int len = ee->len;
	int i;

	if ((len & 1) != 0 || (offset & 1) != 0)
		return -EINVAL;

	if (ee->magic != DM_EEPROM_MAGIC)
		return -EINVAL;

	for (i = 0; i < len; i += 2)
		dm9051_write_eeprom(db, (offset + i) / 2, data + i);
	return 0;
}

static void dm9051_get_pauseparam(struct net_device *ndev,
				  struct ethtool_pauseparam *pause)
{
	struct board_info *db = to_dm9051_board(ndev);

	pause->rx_pause = db->fl.fc_rx;
	pause->tx_pause = db->fl.fc_tx;
	pause->autoneg = db->fl.aneg;
}

static int dm9051_set_pauseparam(struct net_device *ndev,
				 struct ethtool_pauseparam *pause)
{
	struct board_info *db = to_dm9051_board(ndev);
	u8 fcr = 0;

	db->fl.fc_rx = pause->rx_pause;
	db->fl.fc_tx = pause->tx_pause;
	db->fl.aneg = pause->autoneg;

	if (pause->autoneg)
		db->phydev->autoneg = AUTONEG_ENABLE;
	else
		db->phydev->autoneg = AUTONEG_DISABLE;

	mutex_lock(&db->addr_lock);
	if (pause->rx_pause || pause->tx_pause) {
		if (pause->rx_pause)
			fcr |= FCR_BKPM | FCR_FLCE;
		if (pause->tx_pause)
			fcr |= FCR_TXPEN;
	}
	if (dm9051_ior(db, DM9051_FCR) != fcr) //temp
		printk("aet pause param fcr %02x\n", fcr);
	if (dm9051_ior(db, DM9051_FCR) != fcr)
		dm9051_iow(db, DM9051_FCR, fcr); /* set pause param FlowCtrl */
	mutex_unlock(&db->addr_lock);

	return 0;
}

static const struct ethtool_ops dm9051_ethtool_ops = {
	.get_drvinfo = dm9051_get_drvinfo,
	.get_link_ksettings = phy_ethtool_get_link_ksettings,
	.set_link_ksettings = phy_ethtool_set_link_ksettings,
	.get_msglevel = dm9051_get_msglevel,
	.set_msglevel = dm9051_set_msglevel,
	.nway_reset = phy_ethtool_nway_reset,
	.get_link = ethtool_op_get_link,
	.get_eeprom_len = dm9051_get_eeprom_len,
	.get_eeprom = dm9051_get_eeprom,
	.set_eeprom = dm9051_set_eeprom,
	.get_pauseparam = dm9051_get_pauseparam,
	.set_pauseparam = dm9051_set_pauseparam,
};

static void dm9051_operation_clear(struct board_info *db)
{
	db->bc.mac_ovrsft_counter = 0;
	db->bc.large_err_counter = 0;
	db->bc.DO_FIFO_RST_counter = 0;
}

/* reset while rx error found
 */
static void dm9051_restart_code(struct board_info *db)
{
	struct net_device *ndev = db->ndev;
	char *sbuff = (char *)db->prxhdr;
	int rxlen = le16_to_cpu(db->prxhdr->rxlen);

	netdev_dbg(ndev, "dm9-rxhdr, Large-eror (rxhdr %02x %02x %02x %02x)\n",
		   sbuff[0], sbuff[1], sbuff[2], sbuff[3]);
	netdev_dbg(ndev, "dm9-pkt-Wrong, RxLen over-range (%x= %d > %x= %d)\n",
		   rxlen, rxlen, DM9051_PKT_MAX, DM9051_PKT_MAX);

//	mutex_unlock(&db->addr_lock); /* unlock, mdiobus phy read/write has mutex enclose */
//	phy_stop(db->phydev);

//	mutex_lock(&db->addr_lock); /* lock, mdiobus phy read/write has mutex enclose */
	dm9051_reset(db);
	dm9051_restart_fifo_rst(db);
//	mutex_unlock(&db->addr_lock); /* unlock, mdiobus phy read/write has mutex enclose */

//	dm9051_phy_restart(db);
//	mutex_lock(&db->addr_lock); /* lock, mdiobus phy read/write has mutex enclose */

	netdev_dbg(ndev, " RxLenErr&MacOvrSft_Er %d, RST_c %d\n",
		   db->bc.large_err_counter + db->bc.mac_ovrsft_counter,
		   db->bc.DO_FIFO_RST_counter);
}

static int dm9051_loop_rx(struct board_info *db)
{
	struct net_device *ndev = db->ndev;
	u8 rxbyte;
	int ret, rxlen;
	char sbuff[DM_RXHDR_SIZE];
	struct sk_buff *skb;
	u8 *rdptr;
	int scanrr = 0;
static int test_restart_add_larg_tst = 0;
//static int pass_count_for_disp = 0;
	while (1) {
		rxbyte = dm9051_ior(db, DM_SPI_MRCMDX); /* Dummy read */
		rxbyte = dm9051_ior(db, DM_SPI_MRCMDX); /* Dummy read */
		if (rxbyte != DM9051_PKT_RDY) {
			dm9051_iow(db, DM9051_ISR, 0xff); /* Clear ISR, clear to stop mrcmd */
			break; /* exhaust-empty */
		}
		ret = dm9051_inblk(db, sbuff, DM_RXHDR_SIZE);
		if (ret < 0) {
			return -EBUSY;
		}
		dm9051_iow(db, DM9051_ISR, 0xff); /* Clear ISR, clear to stop mrcmd */

		db->prxhdr = (struct dm9051_rxhdr *)sbuff;
		rxlen = le16_to_cpu(db->prxhdr->rxlen);

		if (db->prxhdr->rxstatus & 0xbf) {
			netdev_dbg(ndev, "warn : rxhdr.status 0x%02x\n",
				   db->prxhdr->rxstatus);
		}
		if (rxlen > DM9051_PKT_MAX) {
			db->bc.large_err_counter++;
			dm9051_restart_code(db);
			return scanrr;
		}

		skb = dev_alloc_skb(rxlen + 4);
		if (!skb) {
			dm9051_dumpblk(db, rxlen);
			return scanrr;
		}
		skb_reserve(skb, 2);
		rdptr = (u8 *)skb_put(skb, rxlen - 4);

		ret = dm9051_inblk(db, rdptr, rxlen);
		if (ret < 0) {
			dev_kfree_skb(skb);
			return -EBUSY;
		}
		dm9051_iow(db, DM9051_ISR, 0xff); /* Clear ISR, clear to stop mrcmd */

		skb->protocol = eth_type_trans(skb, db->ndev);
		if (db->ndev->features & NETIF_F_RXCSUM)
			skb_checksum_none_assert(skb);
		if (in_interrupt())
			netif_rx(skb);
		else
			netif_rx_ni(skb);
		db->ndev->stats.rx_bytes += rxlen;
		db->ndev->stats.rx_packets++;
		scanrr++;
	}

test_restart_add_larg_tst += scanrr;
#if 0
	if (test_restart > 250) {
		//struct net_device *ndev = db->ndev;
//		char *sbuff = (char *)db->prxhdr;
//		int rxlen = le16_to_cpu(db->prxhdr->rxlen);

//		netdev_info(ndev, "dm9-rxhdr, Large-eror (rxhdr %02x %02x %02x %02x)\n",
//			   sbuff[0], sbuff[1], sbuff[2], sbuff[3]);
//		netdev_info(ndev, "dm9-pkt-Wrong, RxLen over-range (%x= %d > %x= %d)\n",
//			   rxlen, rxlen, DM9051_PKT_MAX, DM9051_PKT_MAX);

	  test_restart = 0;

		db->bc.large_err_counter++;
		dm9051_restart_code(db);

//		printk("pre-[Read FCR] 0x%02x\n", dm9051_ior(db, DM9051_FCR)); /* fcr */
//		printk("pre-[Read IMR] 0x%02x\n", dm9051_ior(db, DM9051_IMR)); /* rxp to 0xc00 */
//		printk("pre-[Read RCR] 0x%02x\n", dm9051_ior(db, DM9051_RCR)); /* rcr */

		if (1) {
			u8 fcr = 0;
			if (db->fl.fc_rx)
				fcr |= FCR_BKPM | FCR_FLCE;
			if (db->fl.fc_tx)
				fcr |= FCR_TXPEN;
			dm9051_iow(db, DM9051_FCR, fcr); /* init and/or save pause param FlowCtrl */
		}
		if (1) {
			dm9051_iow(db, DM9051_IMR, db->imr_all); /* rxp to 0xc00 */
		}
		if (1) { // to be if (1)

			//.mutex_lock(&db->addr_lock);

			dm9051_iow(db, DM9051_RCR, db->rcr_all); /* EnabRX all */

			//.mutex_unlock(&db->addr_lock);
		}

//		printk("[Read FCR] 0x%02x\n", dm9051_ior(db, DM9051_FCR)); /* fcr */
//		printk("[Read IMR] 0x%02x\n", dm9051_ior(db, DM9051_IMR)); /* rxp to 0xc00 */
		printk("[Read RCR] 0x%02x, pass-count %d\n", dm9051_ior(db, DM9051_RCR), ++pass_count_for_disp); /* rcr */
		return 0;
	}
#endif
	return scanrr;
}

static int dm9051_single_tx(struct board_info *db, u8 *buff, unsigned int len)
{
	int ret;
	u8 check_val;

	/* shorter by waiting tx-end rather than tx-req */
	ret = read_poll_timeout(dm9051_ior, check_val, check_val & (NSR_TX2END | NSR_TX1END),
				1, 20, false, db, DM9051_NSR);
	if (ret) {
		netdev_err(db->ndev, "timeout transmit packet\n");
		return ret;
	}

	dm9051_outblk(db, buff, len);
	dm9051_iow(db, DM9051_TXPLL, len);
	dm9051_iow(db, DM9051_TXPLH, len >> 8);
	dm9051_iow(db, DM9051_TCR, TCR_TXREQ);
	return 0;
}

static int dm9051_loop_tx(struct board_info *db)
{
	struct net_device *ndev = db->ndev;
	int ntx = 0;
	int ret;

	while (!skb_queue_empty(&db->txq)) {
		struct sk_buff *skb;

		skb = skb_dequeue(&db->txq);
		if (skb) {
			ntx++;
			ret = dm9051_single_tx(db, skb->data, skb->len);
			dev_kfree_skb(skb);
			if (ret)
				return 0;
			ndev->stats.tx_bytes += skb->len;
			ndev->stats.tx_packets++;
		}
	}
	return ntx;
}

/* end with enable the interrupt mask
 */
static irqreturn_t dm9051_rx_threaded_irq(int irq, void *pw)
{
	struct board_info *db = pw;
	int ret;

	mutex_lock(&db->spi_lock); /* mutex essential */
	dm9051_imr_disable_lock_essential(db); /* set imr disable */
	if (netif_carrier_ok(db->ndev)) {
		mutex_lock(&db->addr_lock);
		do {
			ret = dm9051_loop_rx(db);
			if (ret < 0)
				break;
			dm9051_loop_tx(db); /* for more performance */
		} while (ret);
		mutex_unlock(&db->addr_lock);
	}
	dm9051_imr_enable_lock_essential(db); /* set imr enable */
	mutex_unlock(&db->spi_lock); /* mutex essential */

	return IRQ_HANDLED;
}

/* end with enable the interrupt mask
 */
static int dm9051_opencode_receiving(struct net_device *ndev, struct board_info *db)
{
	int ret;
	struct spi_device *spi = db->spidev;

	ndev->irq = spi->irq; /* by dts */
	ret = request_threaded_irq(spi->irq, NULL, dm9051_rx_threaded_irq,
				   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				   ndev->name, db);

	if (ret < 0) {
		netdev_err(ndev, "failed to get irq\n");
		return ret;
	}

	netdev_dbg(ndev, "[dm9051_open] %pM irq_no %d ACTIVE_LOW\n", ndev->dev_addr, ndev->irq);
	return 0;
}

static void int_tx_delay(struct work_struct *w)
{
	struct delayed_work *dw = to_delayed_work(w);
	struct board_info *db = container_of(dw, struct board_info, tx_work);

	mutex_lock(&db->spi_lock); /* mutex essential */
	mutex_lock(&db->addr_lock);
	dm9051_loop_tx(db);
	mutex_unlock(&db->addr_lock);
	mutex_unlock(&db->spi_lock); /* mutex essential */
}

static void int_rxctl_delay(struct work_struct *w)
{
	struct delayed_work *dw = to_delayed_work(w);
	struct board_info *db = container_of(dw, struct board_info, rxctrl_work);
	struct net_device *ndev = db->ndev;
	int i, oft;

	mutex_lock(&db->addr_lock);

	for (i = 0, oft = DM9051_PAR; i < ETH_ALEN; i++, oft++)
		dm9051_iow(db, oft, ndev->dev_addr[i]);

	/* Write the hash table */
	for (i = 0, oft = DM9051_MAR; i < 4; i++) {
		dm9051_iow(db, oft++, db->hash_table[i]);
		dm9051_iow(db, oft++, db->hash_table[i] >> 8);
	}

	dm9051_iow(db, DM9051_RCR, db->rcr_all); /* EnabRX all */

	mutex_unlock(&db->addr_lock);
}

/* Irq free and schedule delays cancel
 */
static void dm9051_stopcode_release(struct board_info *db)
{
	free_irq(db->spidev->irq, db);
	cancel_delayed_work_sync(&db->rxctrl_work);
	cancel_delayed_work_sync(&db->tx_work);
}

static void dm9051_control_init(struct board_info *db)
{
	mutex_init(&db->spi_lock);
	mutex_init(&db->addr_lock);
	INIT_DELAYED_WORK(&db->rxctrl_work, int_rxctl_delay);
	INIT_DELAYED_WORK(&db->tx_work, int_tx_delay);
}

static inline void dm9051_phyup_lock(struct board_info *db)
{
	int val, ret;
	mutex_lock(&db->addr_lock);

	ret = dm9051_phy_read(db, MII_BMCR, &val);

//if (!(val & 0x0800))
//{
	/* BMCR Power-up PHY */
	dev_info(&db->spidev->dev, "_mdio_write %d %04x(BMCR phyup)\n", MII_BMCR, val & ~0x0800);
	ret = dm9051_phy_write(db, MII_BMCR, val & ~0x0800); /* BMCR Power-up PHY */
//}

//if (0)
//{
	dm9051_iow(db, DM9051_GPR, 0x01); /* GPR Power-down PHY */
	dev_info(&db->spidev->dev, "*_dm9051_iow %02x %02x(GPR phydown)\n", DM9051_GPR, 0x01);
//}

	dm9051_iow(db, DM9051_GPR, 0); /* GPR Power-up PHY */
	dev_info(&db->spidev->dev, "*_dm9051_iow %02x %02x(GPR phyup)\n", DM9051_GPR, 0);
	dev_info(&db->spidev->dev, "[dm9051_iow/add delay..] %02x %02x\n", DM9051_GPR, 0);
	mdelay(1); /* need for activate phyxcer */

	mutex_unlock(&db->addr_lock);
}

static inline void dm9051_phydown_lock(struct board_info *db)
{
	int val, ret;
	mutex_lock(&db->addr_lock);

#if 0
#else
	/* In phy power down, (can be either BMCR phy down or GPR phy down!!) */
	ret = dm9051_phy_read(db, MII_BMCR, &val);
	if (val & 0x0800) {
		dev_info(&db->spidev->dev, "*NOT _dm9051_iow 0x%02x 0x%02x (keep as BMCR phy down)\n", DM9051_GPR, 0x01);
	} else {
		dev_info(&db->spidev->dev, "*DO _dm9051_iow 0x%02x 0x%02x (power-down phy)\n", DM9051_GPR, 0x01);
		dm9051_iow(db, DM9051_GPR, 0x01); /* Power-Down PHY */
	}
#endif

	mutex_unlock(&db->addr_lock);
}

static void dm9051_initcode_lock(struct board_info *db)
{
	mutex_lock(&db->addr_lock); /* Note: must */

	dm9051_reset(db);
	dm9051_restart_fifo_rst(db);

	mutex_unlock(&db->addr_lock);
}

static void dm9051_stopcode_lock(struct board_info *db)
{
	mutex_lock(&db->addr_lock);

	dm9051_iow(db, DM9051_RCR, RCR_RX_DISABLE); /* Disable RX */

	mutex_unlock(&db->addr_lock);
}

	#define DM_DM_CONF_DTS_COMPATIBLE_USAGE		"davicom,dm9051"
	u32 dm_dts_spi_max_frequency(void) {
		u32 max_spi_speed;
		struct device_node *nc;
		int rc;
		if ((nc = of_find_compatible_node(NULL, NULL, DM_DM_CONF_DTS_COMPATIBLE_USAGE))) {
			rc = of_property_read_u32(nc, "spi-max-frequency", &max_spi_speed); // read to DTS, read-spi_max_speed
			if (!rc)
				return max_spi_speed;
			return 0;
		}
		return 0;
	}
	#define dm_msg_open(ndev) dm_msg_open_receiving_dev_debug_dts_int(ndev)
	static void dm_msg_open_receiving_dev_debug_dts_int(struct net_device *ndev) {
		struct board_info *db = netdev_priv(ndev);
		struct device *dev = &db->spidev->dev;
		u32 speed; /* Read DTS here! */
		if ((speed = dm_dts_spi_max_frequency()))
			dev_info(dev, "dm9 open, spi clock: %d !\n", speed); //INT,50000000(db->Drv_Version_speed)
		else
			dev_info(dev, "*dm9 open WARN, No DTS compatible node or DTS has no spi-max-frequency\n");
	}
/* Open network device
 * Called when the network device is marked active, such as a user executing
 * 'ifconfig up' on the device
 */
static int dm9051_open(struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);
	struct spi_device *spi = db->spidev;
	int ret;

dm_msg_open(ndev);
	skb_queue_head_init(&db->txq);
	netif_start_queue(ndev);
	netif_wake_queue(ndev);

	//.ret = dm9051_opencode_receiving(ndev, db);
	//.if (ret < 0)
	//.	return ret;
	//=
	ndev->irq = spi->irq; /* by dts */
	ret = request_threaded_irq(spi->irq, NULL, dm9051_rx_threaded_irq,
				   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				   ndev->name, db);
	if (ret < 0) {
		netdev_err(ndev, "failed to get irq\n");
		return ret;
	}
	netdev_dbg(ndev, "[dm9051_open] %pM irq_no %d ACTIVE_LOW\n", ndev->dev_addr, ndev->irq);

	db->imr_all = IMR_PAR | IMR_PRM;
	db->rcr_all = RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN;
	db->lcr_all = LMCR_MODE1;

	/* init pause param FlowCtrl */
	db->fl.fc_rx = true;
	db->fl.fc_tx = true;
	db->fl.aneg = AUTONEG_ENABLE;

	/* We may have start with auto negotiation */
	db->phydev->autoneg = AUTONEG_ENABLE;
	db->phydev->speed = 0;
	db->phydev->duplex = 0;

	dm9051_phyup_lock(db); //dm9051_iow(db, DM9051_GPR, 0); /* REG_1F bit0 activate phyxcer */
	dm9051_initcode_lock(db);

	/* phy mdiobus phy read/write is enclose with mutex_lock/mutex_unlock */
	//..phy_support_asym_pause(db->phydev); /* Enable support of asym pause */
	//dm9051_phy_restart(db);
	//=
	/* configure phy advertised pause support */
	//..phy_set_asym_pause(db->phydev, db->fl.fc_rx, db->fl.fc_tx);
	phy_start(db->phydev);
	
	//phy_attached_info(db->phydev);
	//=
	if (1) {
		char *irq_str = phy_attached_info_irq(db->phydev);
		#define PHY_FMT	"attached PHY driver (irq=%s)"
		dev_info(&db->spidev->dev, PHY_FMT "\n", irq_str);
		kfree(irq_str);
	}
	
	dm9051_imr_enable_lock_essential(db);
	return 0;
}

/* Close network device
 * Called to close down a network device which has been active. Cancel any
 * work, shutdown the RX and TX process and then place the chip into a low
 * power state while it is not being used
 */
static int dm9051_stop(struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);

	dm9051_stopcode_release(db);
	netif_stop_queue(ndev);
	phy_stop(db->phydev);
	dm9051_phydown_lock(db);	
	dm9051_stopcode_lock(db);
	return 0;
}

/* event: play a schedule starter in condition
 */
static netdev_tx_t dm9051_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);

	skb_queue_tail(&db->txq, skb); /* add to skb */
	schedule_delayed_work(&db->tx_work, 0);
	return NETDEV_TX_OK;
}

/* event: play with a schedule starter
 */
static void dm9051_set_multicast_list_schedule(struct net_device *ndev)
{
	struct board_info *db = to_dm9051_board(ndev);
	u8 rcr = RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN;
	struct netdev_hw_addr *ha;
	u32 hash_val;

	/* rx control */
	if (ndev->flags & IFF_PROMISC) {
		rcr |= RCR_PRMSC;
		netdev_dbg(ndev, "set_multicast rcr |= RCR_PRMSC, rcr= %02x\n", rcr);
	}

	if (ndev->flags & IFF_ALLMULTI) {
		rcr |= RCR_ALL;
		netdev_dbg(ndev, "set_multicast rcr |= RCR_ALLMULTI, rcr= %02x\n", rcr);
	}

	db->rcr_all = rcr;

	/* broadcast address */
	db->hash_table[0] = 0;
	db->hash_table[1] = 0;
	db->hash_table[2] = 0;
	db->hash_table[3] = 0x8000;

	/* the multicast address in Hash Table : 64 bits */
	netdev_for_each_mc_addr(ha, ndev) {
		hash_val = ether_crc_le(6, ha->addr) & 0x3f;
		db->hash_table[hash_val / 16] |= (u16)1 << (hash_val % 16);
	}

	schedule_delayed_work(&db->rxctrl_work, 0);
}

/* event: write into the mac registers and eeprom directly
 */
static int dm9051_set_mac_address(struct net_device *ndev, void *p)
{
	netdev_info(ndev, "START set_mac_address %pM\n", ndev->dev_addr);
	if (1) {
	struct board_info *db = to_dm9051_board(ndev);
	int i, oft;

	int ret = eth_mac_addr(ndev, p);

	if (ret < 0) {
	  netdev_info(ndev, "*NOT set_mac_address %pM\n", ndev->dev_addr);
		return ret;
	}

	netdev_info(ndev, "set_mac_address %pM\n", ndev->dev_addr);

	netdev_dbg(ndev, "set_mac_address %pM\n", ndev->dev_addr);

	/* write to chip */
	mutex_lock(&db->addr_lock);
	for (i = 0, oft = DM9051_PAR; i < ETH_ALEN; i++, oft++)
		dm9051_iow(db, oft, ndev->dev_addr[i]);
	mutex_unlock(&db->addr_lock);
//.	struct board_info *db = to_dm9051_board(ndev);
//.	dm9051_write_mac_lock(db);
	}
	return 0;
}

/* netdev-ops
 */
static const struct of_device_id dm9051_match_table[] = {
	{ .compatible = "davicom,dm9051", },
	{},
};

static const struct spi_device_id dm9051_id_table[] = {
	{ "dm9051", 0 },
	{},
};

static
const struct net_device_ops dm9051_netdev_ops = {
	.ndo_open = dm9051_open,
	.ndo_stop = dm9051_stop,
	.ndo_start_xmit = dm9051_start_xmit,
	.ndo_set_rx_mode = dm9051_set_multicast_list_schedule,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_set_mac_address = dm9051_set_mac_address,
};

/* probe subs
 */
static void dm9051_netdev(struct net_device *ndev)
{
	ndev->mtu = 1500;
	ndev->if_port = IF_PORT_100BASET;
	ndev->netdev_ops = &dm9051_netdev_ops;
	ndev->ethtool_ops = &dm9051_ethtool_ops;
}

static void dm9051_spimsg_addtail(struct board_info *db)
{
	memset(&db->spi_xfer2, 0, sizeof(struct spi_transfer) * 2);
	spi_message_init(&db->spi_msg2);
	spi_message_add_tail(&db->spi_xfer2[0], &db->spi_msg2);
	spi_message_add_tail(&db->spi_xfer2[1], &db->spi_msg2);
}

static int dm9051_register_mdiobus(struct board_info *db)
{
	struct spi_device *spi = db->spidev;
	int ret;

	db->mdiobus = devm_mdiobus_alloc(&spi->dev);
	if (!db->mdiobus)
		return -ENOMEM;

	db->mdiobus->priv = db;
	db->mdiobus->read = dm9051_mdio_read;
	db->mdiobus->write = dm9051_mdio_write;
	db->mdiobus->name = "dm9051-mdiobus";
	db->mdiobus->phy_mask = (u32)~BIT(1);
	db->mdiobus->parent = &spi->dev;
	snprintf(db->mdiobus->id, MII_BUS_ID_SIZE,
		 "dm9051-%s.%u", dev_name(&spi->dev), spi->chip_select);

	ret = devm_mdiobus_register(&spi->dev, db->mdiobus);
	if (ret < 0) {
		dev_err(&spi->dev, "Could not register MDIO bus\n");
		return ret;
	}
	return 0;
}

static int dm9051_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct net_device *ndev;
	struct board_info *db;
	unsigned int id;
	int ret = 0;

	ndev = devm_alloc_etherdev(dev, sizeof(struct board_info));
	if (!ndev)
		return -ENOMEM;

	SET_NETDEV_DEV(ndev, dev);
	dev_set_drvdata(dev, ndev);
	db = netdev_priv(ndev);
	db->msg_enable = 0;
	db->spidev = spi;
	db->ndev = ndev;
	dm9051_netdev(ndev);

	dm9051_spimsg_addtail(db);
	dm9051_control_init(db); /* init delayed works */

	id = dm9051_chipid(db);
	if (id != DM9051_ID) {
		dev_err(dev, "chip id error\n");
		return -ENODEV;
	}
	dm9051_init_mac_addr(ndev, db);

	ret = dm9051_register_mdiobus(db); /* init mdiobus */
	if (ret) {
		dev_err(dev, "register mdiobus error\n");
		return ret;
	}

	ret = dm9051_phy_connect(db); /* phy connect as poll mode */
	if (ret) {
		dev_err(dev, "phy connect error\n");
		return ret;
	}

	dm9051_operation_clear(db);
	ret = devm_register_netdev(dev, ndev);
	if (ret) {
		dev_err(dev, "failed to register network device\n");
		goto err_netdev;
	}

	return 0;

err_netdev:
	phy_disconnect(db->phydev);
	return ret;
}

static int dm9051_drv_remove(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct net_device *ndev = dev_get_drvdata(dev);
	struct board_info *db = to_dm9051_board(ndev);

	phy_disconnect(db->phydev);
	return 0;
}

static struct spi_driver dm9051_driver = {
	.driver = {
		.name = DRVNAME_9051,
		.of_match_table = dm9051_match_table,
	},
	.probe = dm9051_probe,
	.remove = dm9051_drv_remove,
	.id_table = dm9051_id_table,
};
module_spi_driver(dm9051_driver);

MODULE_AUTHOR("Joseph CHANG <joseph_chang@davicom.com.tw>");
MODULE_DESCRIPTION("Davicom DM9051 network SPI driver");
MODULE_LICENSE("GPL");
