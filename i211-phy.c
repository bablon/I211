#include <linux/phy.h>

#include "defines.h"

#define I211_PHY_ID	0x01410c00

static int i211_phy_config_aneg(struct phy_device *phy)
{
	int val, ret;
	u16 mii_autoneg_adv_reg;
	u16 mii_1000t_ctrl_reg = 0;

	ret = phy_modify(phy, PHY_CONTROL, MII_CR_POWER_DOWN, 0);
	if (ret < 0)
		return ret;

	val = phy_read(phy, M88PHY_SPEC_CTRL);
	if (val < 0)
		return val;

	val &= ~M88PSCR_AUTO_X_MODE;
	val |= M88PSCR_AUTO_X_MODE;
	val &= ~M88PSCR_POLARITY_REVERSAL;
	val &= ~I347AT4_PSCR_DOWNSHIFT_MASK;
	val |= I347AT4_PSCR_DOWNSHIFT_6X;
	val |= I347AT4_PSCR_DOWNSHIFT_ENABLE;

	ret = phy_write(phy, M88PHY_SPEC_CTRL, val);
	if (ret < 0)
		return ret;

	ret = phy_modify(phy, PHY_CONTROL, MII_CR_RESET, MII_CR_RESET);
	if (ret < 0)
		return ret;

	udelay(1);

	ret = phy_modify(phy, PHY_1000T_CTRL, CR_1000T_MS_ENABLE, 0);
	if (ret < 0)
		return ret;

	val = phy_read(phy, PHY_AUTONEG_ADV);
	if (val < 0)
		return val;

	mii_autoneg_adv_reg = val;

	val = phy_read(phy, PHY_1000T_CTRL);
	if (val < 0)
		return val;

	mii_1000t_ctrl_reg = val;

	mii_autoneg_adv_reg &= ~(NWAY_AR_100TX_FD_CAPS |
				 NWAY_AR_100TX_HD_CAPS |
				 NWAY_AR_10T_FD_CAPS   |
				 NWAY_AR_10T_HD_CAPS);
	mii_1000t_ctrl_reg &= ~(CR_1000T_HD_CAPS | CR_1000T_FD_CAPS);

	mii_autoneg_adv_reg |= NWAY_AR_10T_HD_CAPS;
	mii_autoneg_adv_reg |= NWAY_AR_10T_FD_CAPS;
	mii_autoneg_adv_reg |= NWAY_AR_100TX_HD_CAPS;
	mii_autoneg_adv_reg |= NWAY_AR_100TX_FD_CAPS;
	mii_1000t_ctrl_reg |= CR_1000T_FD_CAPS;
	mii_autoneg_adv_reg &= ~(NWAY_AR_ASM_DIR | NWAY_AR_PAUSE);

	ret = phy_write(phy, PHY_AUTONEG_ADV, mii_autoneg_adv_reg);
	if (ret < 0)
		return ret;

	ret = phy_write(phy, PHY_1000T_CTRL, mii_1000t_ctrl_reg);
	if (ret < 0)
		return ret;

	val = phy_read(phy, PHY_CONTROL);
	if (val < 0)
		return val;
	val |= (MII_CR_AUTO_NEG_EN | MII_CR_RESTART_AUTO_NEG);
	ret = phy_write(phy, PHY_CONTROL, val);
	if (ret < 0)
		return ret;

	printk("Restarting Auto-Neg\n");

	return 0;
}

static struct phy_driver i211_phy_driver[] = {
{
	.phy_id		= I211_PHY_ID,
	.phy_id_mask	= 0xfffffff0,
	.flags		= PHY_IS_INTERNAL,
	.name		= "Intel I211 PHY",
	.config_aneg	= i211_phy_config_aneg,
}
};

module_phy_driver(i211_phy_driver);

static struct mdio_device_id __maybe_unused i211_phy[] = {
	{ I211_PHY_ID, 0xfffffff0 },
	{}
};

MODULE_DEVICE_TABLE(mdio, i211_phy);

MODULE_LICENSE("GPL v2");
