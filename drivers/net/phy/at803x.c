/*
 * drivers/net/phy/at803x.c
 *
 * Driver for Atheros 803x PHY
 *
 * Author: Matus Ujhelyi <ujhelyi.m@gmail.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/phy.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>

#define AT803X_INTR_ENABLE			0x12
#define AT803X_INTR_ENABLE_AUTONEG_ERR		BIT(15)
#define AT803X_INTR_ENABLE_SPEED_CHANGED	BIT(14)
#define AT803X_INTR_ENABLE_DUPLEX_CHANGED	BIT(13)
#define AT803X_INTR_ENABLE_PAGE_RECEIVED	BIT(12)
#define AT803X_INTR_ENABLE_LINK_FAIL		BIT(11)
#define AT803X_INTR_ENABLE_LINK_SUCCESS		BIT(10)
#define AT803X_INTR_ENABLE_WIRESPEED_DOWNGRADE	BIT(5)
#define AT803X_INTR_ENABLE_POLARITY_CHANGED	BIT(1)
#define AT803X_INTR_ENABLE_WOL			BIT(0)

#define AT803X_INTR_STATUS			0x13

#define AT803X_SMART_SPEED			0x14
#define AT803X_LED_CONTROL			0x18

#define AT803X_DEVICE_ADDR			0x03
#define AT803X_LOC_MAC_ADDR_0_15_OFFSET		0x804C
#define AT803X_LOC_MAC_ADDR_16_31_OFFSET	0x804B
#define AT803X_LOC_MAC_ADDR_32_47_OFFSET	0x804A
#define AT803X_MMD_ACCESS_CONTROL		0x0D
#define AT803X_MMD_ACCESS_CONTROL_DATA		0x0E
#define AT803X_FUNC_DATA			0x4003
#define AT803X_REG_CHIP_CONFIG			0x1f
#define AT803X_BT_BX_REG_SEL			0x8000

#define AT803X_DEBUG_ADDR			0x1D
#define AT803X_DEBUG_DATA			0x1E

#define AT803X_MODE_CFG_MASK			0x0F
#define AT803X_MODE_CFG_SGMII			0x01

#define AT803X_PSSR			0x11	/*PHY-Specific Status Register*/
#define AT803X_PSSR_MR_AN_COMPLETE	0x0200

#define AT803X_DEBUG_REG_0			0x00
#define AT803X_DEBUG_RX_CLK_DLY_EN		BIT(15)

#define AT803X_DEBUG_REG_5			0x05
#define AT803X_DEBUG_TX_CLK_DLY_EN		BIT(8)

#define ATH8030_PHY_ID 0x004dd076
#define ATH8031_PHY_ID 0x004dd074
#define ATH8035_PHY_ID 0x004dd072
#define AT803X_PHY_ID_MASK			0xffffffef

MODULE_DESCRIPTION("Atheros 803x PHY driver");
MODULE_AUTHOR("Matus Ujhelyi");
MODULE_LICENSE("GPL");

struct at803x_priv {
	bool phy_reset:1;
	struct gpio_desc *gpiod_reset;
};

struct at803x_context {
	u16 bmcr;
	u16 advertise;
	u16 control1000;
	u16 int_enable;
	u16 smart_speed;
	u16 led_control;
};
static int ccr = 0;

static int at803x_debug_reg_read(struct phy_device *phydev, u16 reg)
{
	int ret;

	ret = phy_write(phydev, AT803X_DEBUG_ADDR, reg);
	if (ret < 0)
		return ret;

	return phy_read(phydev, AT803X_DEBUG_DATA);
}

static int at803x_debug_reg_mask(struct phy_device *phydev, u16 reg,
				 u16 clear, u16 set)
{
	u16 val;
	int ret;

	ret = at803x_debug_reg_read(phydev, reg);
	if (ret < 0)
		return ret;

	val = ret & 0xffff;
	val &= ~clear;
	val |= set;

	return phy_write(phydev, AT803X_DEBUG_DATA, val);
}

static inline int at803x_enable_rx_delay(struct phy_device *phydev)
{
	return at803x_debug_reg_mask(phydev, AT803X_DEBUG_REG_0, 0,
					AT803X_DEBUG_RX_CLK_DLY_EN);
}

static inline int at803x_enable_tx_delay(struct phy_device *phydev)
{
	return at803x_debug_reg_mask(phydev, AT803X_DEBUG_REG_5, 0,
					AT803X_DEBUG_TX_CLK_DLY_EN);
}

/* save relevant PHY registers to private copy */
static void at803x_context_save(struct phy_device *phydev,
				struct at803x_context *context)
{
	context->bmcr = phy_read(phydev, MII_BMCR);
	context->advertise = phy_read(phydev, MII_ADVERTISE);
	context->control1000 = phy_read(phydev, MII_CTRL1000);
	context->int_enable = phy_read(phydev, AT803X_INTR_ENABLE);
	context->smart_speed = phy_read(phydev, AT803X_SMART_SPEED);
	context->led_control = phy_read(phydev, AT803X_LED_CONTROL);
}

/* restore relevant PHY registers from private copy */
static void at803x_context_restore(struct phy_device *phydev,
				   const struct at803x_context *context)
{
	phy_write(phydev, MII_BMCR, context->bmcr);
	phy_write(phydev, MII_ADVERTISE, context->advertise);
	phy_write(phydev, MII_CTRL1000, context->control1000);
	phy_write(phydev, AT803X_INTR_ENABLE, context->int_enable);
	phy_write(phydev, AT803X_SMART_SPEED, context->smart_speed);
	phy_write(phydev, AT803X_LED_CONTROL, context->led_control);
}

static int at803x_set_wol(struct phy_device *phydev,
			  struct ethtool_wolinfo *wol)
{
	struct net_device *ndev = phydev->attached_dev;
	const u8 *mac;
	int ret;
	u32 value;
	unsigned int i, offsets[] = {
		AT803X_LOC_MAC_ADDR_32_47_OFFSET,
		AT803X_LOC_MAC_ADDR_16_31_OFFSET,
		AT803X_LOC_MAC_ADDR_0_15_OFFSET,
	};

	if (!ndev)
		return -ENODEV;

	if (wol->wolopts & WAKE_MAGIC) {
		mac = (const u8 *) ndev->dev_addr;

		if (!is_valid_ether_addr(mac))
			return -EINVAL;

		for (i = 0; i < 3; i++) {
			phy_write(phydev, AT803X_MMD_ACCESS_CONTROL,
				  AT803X_DEVICE_ADDR);
			phy_write(phydev, AT803X_MMD_ACCESS_CONTROL_DATA,
				  offsets[i]);
			phy_write(phydev, AT803X_MMD_ACCESS_CONTROL,
				  AT803X_FUNC_DATA);
			phy_write(phydev, AT803X_MMD_ACCESS_CONTROL_DATA,
				  mac[(i * 2) + 1] | (mac[(i * 2)] << 8));
		}

		value = phy_read(phydev, AT803X_INTR_ENABLE);
		value |= AT803X_INTR_ENABLE_WOL;
		ret = phy_write(phydev, AT803X_INTR_ENABLE, value);
		if (ret)
			return ret;
		value = phy_read(phydev, AT803X_INTR_STATUS);
	} else {
		value = phy_read(phydev, AT803X_INTR_ENABLE);
		value &= (~AT803X_INTR_ENABLE_WOL);
		ret = phy_write(phydev, AT803X_INTR_ENABLE, value);
		if (ret)
			return ret;
		value = phy_read(phydev, AT803X_INTR_STATUS);
	}

	return ret;
}

static void at803x_get_wol(struct phy_device *phydev,
			   struct ethtool_wolinfo *wol)
{
	u32 value;

	wol->supported = WAKE_MAGIC;
	wol->wolopts = 0;

	value = phy_read(phydev, AT803X_INTR_ENABLE);
	if (value & AT803X_INTR_ENABLE_WOL)
		wol->wolopts |= WAKE_MAGIC;
}

static int at803x_suspend(struct phy_device *phydev)
{
	int value;
	int wol_enabled;

	mutex_lock(&phydev->lock);

	value = phy_read(phydev, AT803X_INTR_ENABLE);
	wol_enabled = value & AT803X_INTR_ENABLE_WOL;

	value = phy_read(phydev, MII_BMCR);

	if (wol_enabled)
		value |= BMCR_ISOLATE;
	else
		value |= BMCR_PDOWN;

	phy_write(phydev, MII_BMCR, value);

	mutex_unlock(&phydev->lock);

	return 0;
}

static int at803x_resume(struct phy_device *phydev)
{
	int value;

	value = phy_read(phydev, MII_BMCR);
	value &= ~(BMCR_PDOWN | BMCR_ISOLATE);
	phy_write(phydev, MII_BMCR, value);

	return 0;
}

static int at803x_probe(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	struct at803x_priv *priv;
	struct gpio_desc *gpiod_reset;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	if (phydev->drv->phy_id != ATH8030_PHY_ID)
		goto does_not_require_reset_workaround;

	gpiod_reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(gpiod_reset))
		return PTR_ERR(gpiod_reset);

	priv->gpiod_reset = gpiod_reset;

does_not_require_reset_workaround:
	phydev->priv = priv;

	return 0;
}
static int at8031_config_init(struct phy_device *phydev)
{
	int ret;
	int val;
	u32 features;
//	printk("===at803x_config_init \n " );

	features = (SUPPORTED_TP | SUPPORTED_MII
			| SUPPORTED_AUI | SUPPORTED_FIBRE |
			SUPPORTED_BNC|SUPPORTED_Pause |SUPPORTED_Asym_Pause | SUPPORTED_1000baseKX_Full);

	/* Do we support autonegotiation? */
	val = phy_read(phydev, MII_BMSR);
	if (val < 0)
		return val;

	if (val & BMSR_ANEGCAPABLE)
		features |= SUPPORTED_Autoneg;

	if (val & BMSR_100FULL)
		features |= SUPPORTED_100baseT_Full;
	if (val & BMSR_100HALF)
		features |= SUPPORTED_100baseT_Half;
	if (val & BMSR_10FULL)
		features |= SUPPORTED_10baseT_Full;
	if (val & BMSR_10HALF)
		features |= SUPPORTED_10baseT_Half;

	if (val & BMSR_ESTATEN) {
		val = phy_read(phydev, MII_ESTATUS);
		if (val < 0)
			return val;

		if (val & 0x8000){
			features &= ~SUPPORTED_1000baseKX_Full;
			}
		if (!(val & 0x4000)){
			features |= SUPPORTED_1000baseT_Half;
			}
		if (val & ESTATUS_1000_TFULL)
			features |= SUPPORTED_1000baseT_Full;
		if (val & ESTATUS_1000_THALF)
			features |= SUPPORTED_1000baseT_Half;
	}

    //printk("=== feature = 0x%x", features);
	phydev->supported &= features;
	phydev->advertising &= features;


	if (phydev->interface == PHY_INTERFACE_MODE_RGMII_RXID ||
			phydev->interface == PHY_INTERFACE_MODE_RGMII_ID) {
		ret = at803x_enable_rx_delay(phydev);
		if (ret < 0)
			return ret;
	}

	if (phydev->interface == PHY_INTERFACE_MODE_RGMII_TXID ||
			phydev->interface == PHY_INTERFACE_MODE_RGMII_ID) {
		ret = at803x_enable_tx_delay(phydev);
		if (ret < 0)
			return ret;
	}

	return 0;
}
static int at803x_config_init(struct phy_device *phydev)
{
	int ret;

	ret = genphy_config_init(phydev);
	if (ret < 0)
		return ret;

	if (phydev->interface == PHY_INTERFACE_MODE_RGMII_RXID ||
			phydev->interface == PHY_INTERFACE_MODE_RGMII_ID) {
		ret = at803x_enable_rx_delay(phydev);
		if (ret < 0)
			return ret;
	}

	if (phydev->interface == PHY_INTERFACE_MODE_RGMII_TXID ||
			phydev->interface == PHY_INTERFACE_MODE_RGMII_ID) {
		ret = at803x_enable_tx_delay(phydev);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int at803x_ack_interrupt(struct phy_device *phydev)
{
	int err;

	err = phy_read(phydev, AT803X_INTR_STATUS);

	return (err < 0) ? err : 0;
}

static int at803x_config_intr(struct phy_device *phydev)
{
	int err;
	int value;

	value = phy_read(phydev, AT803X_INTR_ENABLE);

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED) {
		value |= AT803X_INTR_ENABLE_AUTONEG_ERR;
		value |= AT803X_INTR_ENABLE_SPEED_CHANGED;
		value |= AT803X_INTR_ENABLE_DUPLEX_CHANGED;
		value |= AT803X_INTR_ENABLE_LINK_FAIL;
		value |= AT803X_INTR_ENABLE_LINK_SUCCESS;

		err = phy_write(phydev, AT803X_INTR_ENABLE, value);
	}
	else
		err = phy_write(phydev, AT803X_INTR_ENABLE, 0);

	return err;
}
static void at8031_link_change_notify(struct phy_device *phydev)
{
//	int ccr;
	unsigned char	mode_cfg_qual = 0;
	struct at803x_priv *priv = phydev->priv;
    //printk("===1. at803x_link_change_notify \r\n");

	/*
	 * in SGMII mode, if copper side autoneg is successful,
	 * also check SGMII side autoneg result
	 */
	ccr = phy_read(phydev, AT803X_REG_CHIP_CONFIG);
//    printk("===2. at803x_link_change_notify: ccr=0x%x \r\n", ccr);
	mode_cfg_qual = (unsigned char)((ccr & 0x00F0)>>4);
	if ((mode_cfg_qual == 0x0E)
		||(mode_cfg_qual == 0x06)
		||(mode_cfg_qual == 0x02)
		||(mode_cfg_qual == 0x03)		
		){


	/* switch to SGMII/fiber page */
    
	phy_write(phydev, AT803X_REG_CHIP_CONFIG, ccr & ~AT803X_BT_BX_REG_SEL);


		}
	else
		{
	/* switch back to copper page */
//    printk("===3. at803x_link_change_notify ccr=0x%x \r\n", ccr);
	phy_write(phydev, AT803X_REG_CHIP_CONFIG, ccr | AT803X_BT_BX_REG_SEL);
		}

	/*
	 * Conduct a hardware reset for AT8030 every time a link loss is
	 * signalled. This is necessary to circumvent a hardware bug that
	 * occurs when the cable is unplugged while TX packets are pending
	 * in the FIFO. In such cases, the FIFO enters an error mode it
	 * cannot recover from by software.
	 */
	if (phydev->drv->phy_id == ATH8030_PHY_ID) {
		if (phydev->state == PHY_NOLINK) {
			if (priv->gpiod_reset && !priv->phy_reset) {
				struct at803x_context context;

				at803x_context_save(phydev, &context);

				gpiod_set_value(priv->gpiod_reset, 0);
				msleep(1);
				gpiod_set_value(priv->gpiod_reset, 1);
				msleep(1);

				at803x_context_restore(phydev, &context);

				phydev_dbg(phydev, "%s(): phy was reset\n",
					__func__);
				priv->phy_reset = true;
			}
		} else {
			priv->phy_reset = false;
		}
	}
}
static void at803x_link_change_notify(struct phy_device *phydev)
{
	struct at803x_priv *priv = phydev->priv;

	/*
	 * Conduct a hardware reset for AT8030 every time a link loss is
	 * signalled. This is necessary to circumvent a hardware bug that
	 * occurs when the cable is unplugged while TX packets are pending
	 * in the FIFO. In such cases, the FIFO enters an error mode it
	 * cannot recover from by software.
	 */
	if (phydev->state == PHY_NOLINK) {
		if (priv->gpiod_reset && !priv->phy_reset) {
			struct at803x_context context;

			at803x_context_save(phydev, &context);

			gpiod_set_value(priv->gpiod_reset, 1);
			msleep(1);
			gpiod_set_value(priv->gpiod_reset, 0);
			msleep(1);

			at803x_context_restore(phydev, &context);

			phydev_dbg(phydev, "%s(): phy was reset\n",
				   __func__);
			priv->phy_reset = true;
		}
	} else {
		priv->phy_reset = false;
	}
}

static int at8031_config_advert(struct phy_device *phydev)
{
	u32 advertise;
	int oldadv, adv, bmsr;
	int err, changed = 0;
	ccr = phy_read(phydev, AT803X_REG_CHIP_CONFIG);
	if(ccr & 0x8000){ // copper page
		phydev->supported |= (PHY_1000BT_FEATURES);
		changed = 1;
	}	
	else
	{
		phydev->supported |= (PHY_1000BT_FEATURES);
		phy_write(phydev, MII_CTRL1000, 0x300);
		changed = 1;
	}
	phydev->advertising &= phydev->supported;
	advertise = phydev->advertising;
	adv = phy_read(phydev, MII_ADVERTISE);
	if (adv < 0)
		return adv;
	oldadv = adv;
	if(ccr & 0x8000){ // copper page
		adv |= ethtool_adv_to_mii_adv_t(advertise);
	}	
	else
	{
		adv = 0x1a0;
	}
	if (adv != oldadv) {
		err = phy_write(phydev, MII_ADVERTISE, adv);
		if (err < 0)
			return err;
		changed = 1;
	}
	bmsr = phy_read(phydev, MII_BMSR);
	if (bmsr < 0)
		return bmsr;
	if (!(bmsr & BMSR_ESTATEN))
		return changed;
	adv = phy_read(phydev, MII_CTRL1000);
	if (adv < 0)
		return adv;
	oldadv = adv;
	adv &= ~(ADVERTISE_1000FULL | ADVERTISE_1000HALF);
	if (phydev->supported & (SUPPORTED_1000baseT_Half |
				 SUPPORTED_1000baseT_Full)) {
		adv |= ethtool_adv_to_mii_ctrl1000_t(advertise);
	}
	if (adv != oldadv)
		changed = 1;
	err = phy_write(phydev, MII_CTRL1000, adv);
	if (err < 0)
		return err;
	return changed;
}
static int at8031_config_aneg(struct phy_device *phydev)
{
		int result;
		if (AUTONEG_ENABLE != phydev->autoneg)
		{
			int current_ccr = 0;
	        current_ccr = phy_read(phydev, AT803X_REG_CHIP_CONFIG);
            if( (current_ccr & 0x20) != 0) 
            {
                if(phydev->speed == 100)
                {
            phy_write(phydev, AT803X_REG_CHIP_CONFIG, 0x80bb);
            phy_write(phydev, MII_BMCR, 0x2100);
                phy_write(phydev, MII_CTRL1000, 0x0);
                }
                if(phydev->speed == 1000)
                {
                phy_write(phydev, AT803X_REG_CHIP_CONFIG, 0x81bb);
                    phy_write(phydev, MII_BMCR, 0x1140);
                phy_write(phydev, MII_CTRL1000, 0x300);
                }
            }
            else
            {
                //phy_write(phydev, AT803X_REG_CHIP_CONFIG, 0x81bb);
                //phy_write(phydev, MII_BMCR, 0x1000);
                //phy_write(phydev, MII_CTRL1000, 0x200);
                //return 0;
            }
			return genphy_setup_forced(phydev);
		}
		result = at8031_config_advert(phydev);
		if (result < 0) /* error */
			return result;
		if (result == 0) {
			int ctl = phy_read(phydev, MII_BMCR);
			if (ctl < 0)
				return ctl;
			if (!(ctl & BMCR_ANENABLE) || (ctl & BMCR_ISOLATE))
				result = 1; /* do restart aneg */
		}
		if (result > 0)
			result = genphy_restart_aneg(phydev);
		return result;
}
static int at8031_read_status(struct phy_device *phydev)
{
	int current_ccr = 0;
	current_ccr = phy_read(phydev, AT803X_REG_CHIP_CONFIG);
	if(current_ccr != ccr)
	{
		ccr = current_ccr;
		phydev->drv->config_init(phydev);
		phydev->drv->config_aneg(phydev);
	}
	if(ccr & 0x8000){ // copper
        phydev->supported = 767;
        phy_write(phydev, AT803X_REG_CHIP_CONFIG, 0x81bb);
        //phy_write(phydev, MII_BMCR, 0x1000);
        //phy_write(phydev, MII_CTRL1000, 0x200);
		return genphy_read_status(phydev);
		}
	else  // fiber page
	{
        if(ccr == 0x6b) // 100 BASE-FX
        {

            return genphy_update_link(phydev);
        }
        else
        {
			int adv;
			int err;
			int lpa;
			int lpagb = 0;
			int common_adv;
			int common_adv_gb = 0;
			err = genphy_update_link(phydev);
			if (err)
				return err;
			phydev->lp_advertising = 0;
			if (AUTONEG_ENABLE == phydev->autoneg) {
				if (phydev->supported & (SUPPORTED_1000baseT_Half
							| SUPPORTED_1000baseT_Full)) {
					lpagb = phy_read(phydev, MII_TPISTATUS);
					if (lpagb < 0)
						return lpagb;
					adv = phy_read(phydev, MII_CTRL1000);
					if (adv < 0)
						return adv;
					if(lpagb & (0x3<<9)){
					phydev->lp_advertising = ADVERTISED_1000baseT_Full;
						}
					common_adv_gb = lpagb & adv << 2;
				}
				lpa = phy_read(phydev, MII_LPA);
				if (lpa < 0)
					return lpa;
				phydev->lp_advertising |= mii_lpa_to_ethtool_lpa_t(lpa);
				adv = phy_read(phydev, MII_ADVERTISE);
				if (adv < 0)
					return adv;
				common_adv = lpa & adv;
				phydev->speed = SPEED_10;
				phydev->duplex = DUPLEX_HALF;
				phydev->pause = 0;
				phydev->asym_pause = 0;
				if (common_adv_gb & (LPA_1000FULL | LPA_1000HALF)) {
					phydev->speed = SPEED_1000;
					if (common_adv_gb & LPA_1000FULL)
						phydev->duplex = DUPLEX_FULL;
				} else if (common_adv & (LPA_100FULL | LPA_100HALF)) {
					phydev->speed = SPEED_100;
					if (common_adv & LPA_100FULL)
						phydev->duplex = DUPLEX_FULL;
				} else
					if (common_adv & LPA_10FULL)
						phydev->duplex = DUPLEX_FULL;
				if (phydev->duplex == DUPLEX_FULL) {
					phydev->pause = lpa & LPA_PAUSE_CAP ? 1 : 0;
					phydev->asym_pause = lpa & LPA_PAUSE_ASYM ? 1 : 0;
				}
			} else {
				int bmcr = phy_read(phydev, MII_BMCR);
				if (bmcr < 0)
					return bmcr;
				if (bmcr & BMCR_FULLDPLX)
					phydev->duplex = DUPLEX_FULL;
				else
					phydev->duplex = DUPLEX_HALF;
				if (bmcr & BMCR_SPEED1000)
					phydev->speed = SPEED_1000;
				else if (bmcr & BMCR_SPEED100)
					phydev->speed = SPEED_100;
				else
					phydev->speed = SPEED_10;
				phydev->pause = 0;
				phydev->asym_pause = 0;
			}
			}
			return 0;
	}
	return 0;
}
static int at8031_aneg_done(struct phy_device *phydev)
{
	int ccr;

	int aneg_done = genphy_aneg_done(phydev);
	if(ccr & 0x8000){ // copper
		phy_write(phydev, MII_CTRL1000, 0);
	}
	else  // fiber page
	{
		phy_write(phydev, MII_CTRL1000, 0x300);
	}
	if (aneg_done != BMSR_ANEGCOMPLETE)
		return aneg_done;

	/*
	 * in SGMII mode, if copper side autoneg is successful,
	 * also check SGMII side autoneg result
	 */

	/* switch to SGMII/fiber page */

	/* check if the SGMII link is OK. */
	/* switch back to copper page */

	return aneg_done;
}

static struct phy_driver at803x_driver[] = {
{
	/* ATHEROS 8035 */
	.phy_id			= ATH8035_PHY_ID,
	.name			= "Atheros 8035 ethernet",
	.phy_id_mask		= AT803X_PHY_ID_MASK,
	.probe			= at803x_probe,
	.config_init		= at803x_config_init,
	.set_wol		= at803x_set_wol,
	.get_wol		= at803x_get_wol,
	.suspend		= at803x_suspend,
	.resume			= at803x_resume,
	.features		= PHY_GBIT_FEATURES,
	.flags			= PHY_HAS_INTERRUPT,
	.config_aneg		= genphy_config_aneg,
	.read_status		= genphy_read_status,
	.ack_interrupt		= at803x_ack_interrupt,
	.config_intr		= at803x_config_intr,
}, {
	/* ATHEROS 8030 */
	.phy_id			= ATH8030_PHY_ID,
	.name			= "Atheros 8030 ethernet",
	.phy_id_mask		= AT803X_PHY_ID_MASK,
	.probe			= at803x_probe,
	.config_init		= at803x_config_init,
	.link_change_notify	= at803x_link_change_notify,
	.set_wol		= at803x_set_wol,
	.get_wol		= at803x_get_wol,
	.suspend		= at803x_suspend,
	.resume			= at803x_resume,
	.features		= PHY_BASIC_FEATURES,
	.flags			= PHY_HAS_INTERRUPT,
	.config_aneg		= genphy_config_aneg,
	.read_status		= genphy_read_status,
	.ack_interrupt		= at803x_ack_interrupt,
	.config_intr		= at803x_config_intr,
}, {
	/* ATHEROS 8031 */
	.phy_id			= ATH8031_PHY_ID,
	.name			= "Atheros 8031 ethernet",
	.phy_id_mask		= AT803X_PHY_ID_MASK,
	.probe			= at803x_probe,
	.config_init		= at8031_config_init,
	.link_change_notify	= at8031_link_change_notify,
	.set_wol		= at803x_set_wol,
	.get_wol		= at803x_get_wol,
	.suspend		= at803x_suspend,
	.resume			= at803x_resume,
	.features		= PHY_GBIT_FEATURES,
	.flags			= PHY_HAS_INTERRUPT,
	.config_aneg		= at8031_config_aneg, //genphy_config_aneg,
	.read_status		= at8031_read_status, //genphy_read_status,
	.aneg_done		= at8031_aneg_done,
	.ack_interrupt		= &at803x_ack_interrupt,
	.config_intr		= &at803x_config_intr,
} };

module_phy_driver(at803x_driver);

static struct mdio_device_id __maybe_unused atheros_tbl[] = {
	{ ATH8030_PHY_ID, AT803X_PHY_ID_MASK },
	{ ATH8031_PHY_ID, AT803X_PHY_ID_MASK },
	{ ATH8035_PHY_ID, AT803X_PHY_ID_MASK },
	{ }
};

MODULE_DEVICE_TABLE(mdio, atheros_tbl);
