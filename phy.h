/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright(c) 2007 - 2018 Intel Corporation. */

#ifndef _PHY_H_
#define _PHY_H_

/* IGP01E1000 Specific Registers */
#define IGP01E1000_PHY_PORT_CONFIG        0x10 /* Port Config */
#define IGP01E1000_PHY_PORT_STATUS        0x11 /* Status */
#define IGP01E1000_PHY_PORT_CTRL          0x12 /* Control */
#define IGP01E1000_PHY_LINK_HEALTH        0x13 /* PHY Link Health */
#define IGP02E1000_PHY_POWER_MGMT         0x19 /* Power Management */
#define IGP01E1000_PHY_PAGE_SELECT        0x1F /* Page Select */
#define IGP01E1000_PHY_PCS_INIT_REG       0x00B4
#define IGP01E1000_PHY_POLARITY_MASK      0x0078
#define IGP01E1000_PSCR_AUTO_MDIX         0x1000
#define IGP01E1000_PSCR_FORCE_MDI_MDIX    0x2000 /* 0=MDI, 1=MDIX */
#define IGP01E1000_PSCFR_SMART_SPEED      0x0080

#define I82580_ADDR_REG                   16
#define I82580_CFG_REG                    22
#define I82580_CFG_ASSERT_CRS_ON_TX       BIT(15)
#define I82580_CFG_ENABLE_DOWNSHIFT       (3u << 10) /* auto downshift 100/10 */
#define I82580_CTRL_REG                   23
#define I82580_CTRL_DOWNSHIFT_MASK        (7u << 10)

/* 82580 specific PHY registers */
#define I82580_PHY_CTRL_2            18
#define I82580_PHY_LBK_CTRL          19
#define I82580_PHY_STATUS_2          26
#define I82580_PHY_DIAG_STATUS       31

/* I82580 PHY Status 2 */
#define I82580_PHY_STATUS2_REV_POLARITY   0x0400
#define I82580_PHY_STATUS2_MDIX           0x0800
#define I82580_PHY_STATUS2_SPEED_MASK     0x0300
#define I82580_PHY_STATUS2_SPEED_1000MBPS 0x0200
#define I82580_PHY_STATUS2_SPEED_100MBPS  0x0100

/* I82580 PHY Control 2 */
#define I82580_PHY_CTRL2_MANUAL_MDIX      0x0200
#define I82580_PHY_CTRL2_AUTO_MDI_MDIX    0x0400
#define I82580_PHY_CTRL2_MDIX_CFG_MASK    0x0600

/* I82580 PHY Diagnostics Status */
#define I82580_DSTATUS_CABLE_LENGTH       0x03FC
#define I82580_DSTATUS_CABLE_LENGTH_SHIFT 2

/* 82580 PHY Power Management */
#define I82580_PHY_POWER_MGMT	0xE14
#define I82580_PM_SPD		0x0001 /* Smart Power Down */
#define I82580_PM_D0_LPLU		0x0002 /* For D0a states */
#define I82580_PM_D3_LPLU		0x0004 /* For all other states */
#define I82580_PM_GO_LINKD		0x0020 /* Go Link Disconnect */

/* Enable flexible speed on link-up */
#define IGP02E1000_PM_D0_LPLU             0x0002 /* For D0a states */
#define IGP02E1000_PM_D3_LPLU             0x0004 /* For all other states */
#define IGP01E1000_PLHR_SS_DOWNGRADE      0x8000
#define IGP01E1000_PSSR_POLARITY_REVERSED 0x0002
#define IGP01E1000_PSSR_MDIX              0x0800
#define IGP01E1000_PSSR_SPEED_MASK        0xC000
#define IGP01E1000_PSSR_SPEED_1000MBPS    0xC000
#define IGP02E1000_PHY_CHANNEL_NUM        4
#define IGP02E1000_PHY_AGC_A              0x11B1
#define IGP02E1000_PHY_AGC_B              0x12B1
#define IGP02E1000_PHY_AGC_C              0x14B1
#define IGP02E1000_PHY_AGC_D              0x18B1
#define IGP02E1000_AGC_LENGTH_SHIFT       9   /* Course - 15:13, Fine - 12:9 */
#define IGP02E1000_AGC_LENGTH_MASK        0x7F
#define IGP02E1000_AGC_RANGE              15

#define CABLE_LENGTH_UNDEFINED      0xFF

#endif
