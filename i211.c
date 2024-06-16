/* SPDX-License-Identifier: GPL-2.0 */

/* Copyright (c) 2024 Jiajia Liu <liujia6264@gmail.com> */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/pci_ids.h>
#include <linux/pci.h>
#include <linux/aer.h>
#include <linux/etherdevice.h>
#include <net/pkt_sched.h>
#include <net/netdev_queues.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/netdevice.h>
#include <linux/pm_runtime.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/version.h>
#include <linux/circ_buf.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 6, 0)
#include <net/page_pool.h>
#else
#include <net/page_pool/helpers.h>
#endif

#include <net/xdp.h>

#include "regs.h"
#include "defines.h"
#include "phy.h"

#define STAT_DEV_RST_SET	0x00100000
#define CTRL_DEV_RST		0x20000000
#define SW_SYNCH_MB		0x00000100
#define DESC_NEEDED (MAX_SKB_FRAGS + 4)

struct rx_buffer {
	dma_addr_t dma;
	struct page *page;
	u16 page_offset;
};

struct tx_buffer {
	struct sk_buff *skb;
	u32 bytecount;
	u16 gso_segs;
	unsigned long time_stamp;
	int protocol;
	union adv_tx_desc *next_to_watch;
	u32 flags;

	dma_addr_t dma;
	u32 len;
};

struct tx_queue_stats {
	u64 packets;
	u64 bytes;
	u64 restart_queue;
	u64 restart_queue2;
};

struct rx_queue_stats {
	u64 packets;
	u64 bytes;
	u64 drops;
	u64 csum_err;
	u64 alloc_failed;
};

struct ring {
	struct net_device *netdev;
	struct device *dev;

	struct page_pool *page_pool;

	struct circ_buf circ;

	dma_addr_t dma;
	unsigned int size;
	void __iomem *desc_tail;
	int reg_idx;
	int queue_index;

	u16 count;
	char name[4];

	struct rx_buffer *rx_buffer;
	struct tx_buffer *tx_buffer;

	union {
		/* TX */
		struct {
			struct tx_queue_stats tx_stats;
			struct u64_stats_sync tx_syncp;
			struct u64_stats_sync tx_syncp2;
		};
		/* RX */
		struct {
			struct sk_buff *skb;
			struct rx_queue_stats rx_stats;
			struct u64_stats_sync rx_syncp;
		};
	};

};

union adv_rx_desc {
	struct {
		__le64 pkt_addr;             /* Packet buffer address */
		__le64 hdr_addr;             /* Header buffer address */
	} read;
	struct {
		struct {
			struct {
				__le16 pkt_info;   /* RSS type, Packet type */
				__le16 hdr_info;   /* Split Head, buf len */
			} lo_dword;
			union {
				__le32 rss;          /* RSS Hash */
				struct {
					__le16 ip_id;    /* IP id */
					__le16 csum;     /* Packet Checksum */
				} csum_ip;
			} hi_dword;
		} lower;
		struct {
			__le32 status_error;     /* ext status/error */
			__le16 length;           /* Packet length */
			__le16 vlan;             /* VLAN tag */
		} upper;
	} wb;  /* writeback */
};

union adv_tx_desc {
	struct {
		__le64 buffer_addr;    /* Address of descriptor's data buf */
		__le32 cmd_type_len;
		__le32 olinfo_status;
	} read;
	struct {
		__le64 rsvd;       /* Reserved */
		__le32 nxtseq_seed;
		__le32 status;
	} wb;
};

/* Adv Transmit Descriptor Config Masks */
#define ADVTXD_MAC_TSTAMP   0x00080000 /* IEEE1588 Timestamp packet */
#define ADVTXD_DTYP_CTXT    0x00200000 /* Advanced Context Descriptor */
#define ADVTXD_DTYP_DATA    0x00300000 /* Advanced Data Descriptor */
#define ADVTXD_DCMD_EOP     0x01000000 /* End of Packet */
#define ADVTXD_DCMD_IFCS    0x02000000 /* Insert FCS (Ethernet CRC) */
#define ADVTXD_DCMD_RS      0x08000000 /* Report Status */
#define ADVTXD_DCMD_DEXT    0x20000000 /* Descriptor extension (1=Adv) */
#define ADVTXD_DCMD_VLE     0x40000000 /* VLAN pkt enable */
#define ADVTXD_DCMD_TSE     0x80000000 /* TCP Seg enable */
#define ADVTXD_PAYLEN_SHIFT    14 /* Adv desc PAYLEN shift */

struct adv_tx_context_desc {
	__le32 vlan_macip_lens;
	__le32 seqnum_seed;
	__le32 type_tucmd_mlhl;
	__le32 mss_l4len_idx;
};

struct phy_info {
	u32 id;
	u32 addr;
	u32 revision;
	u32 mdix;
};

struct mac_info {
	u8 addr[6];
	u8 perm_addr[6];
};

struct qvector {
	struct adapter *adap;

	int cpu;
	struct ring ring;

	struct napi_struct napi;
};

struct hw_stats {
	u64 crcerrs;
	u64 algnerrc;
	u64 rxerrc;
	u64 mpc;
	u64 ecol;
	u64 latecol;
	u64 colc;
	u64 tncrs;
	u64 cexterr;
	u64 mprc;
	u64 ruc;
	u64 roc;
};

enum fc_mode {
	fc_none = 0,
	fc_rx_pause,
	fc_tx_pause,
	fc_full,
	fc_default = 0xFF
};

struct fc_info {
	u32 high_water;     /* Flow control high-water mark */
	u32 low_water;      /* Flow control low-water mark */
	u16 pause_time;     /* Flow control pause timer */
	bool send_xon;      /* Flow control send XON */
	bool strict_ieee;   /* Strict IEEE mode */
	enum fc_mode current_mode; /* Type of flow control */
	enum fc_mode requested_mode;
};

enum adapter_state {
	STATE_RESETING,
	STATE_DOWN,
};

struct adapter {
	struct net_device *netdev;
	struct pci_dev *pdev;
	struct mii_bus *mii_bus;
	struct phy_device *phy_dev;
	int msg_enable;

	unsigned long state;
	unsigned long flags;

	u8 __iomem *io_addr;

	struct phy_info phy;
	struct mac_info mac;
	struct qvector qvector[4];

	struct hw_stats stats;
	spinlock_t stats64_lock;
	struct rtnl_link_stats64 stats64;

	struct work_struct watchdog_task;

	u32 max_frame_size;
	u32 min_frame_size;

	int get_link_status;
	unsigned long link_check_timeout;
	u16 link_speed;
	u16 link_duplex;
	unsigned long tx_timeout_count;

	struct fc_info fc;
};

enum i211_tx_flags {
	/* cmd_type flags */
	I211_TX_FLAGS_VLAN	= 0x01,
	I211_TX_FLAGS_TSO	= 0x02,
	I211_TX_FLAGS_TSTAMP	= 0x04,

	/* olinfo flags */
	I211_TX_FLAGS_IPV4	= 0x10,
	I211_TX_FLAGS_CSUM	= 0x20,
};

#define I211_TX_FLAGS_VLAN_MASK	0xffff0000
#define I211_TX_FLAGS_VLAN_SHIFT	16

static const struct pci_device_id i211_pci_table[] = {
	{ PCI_VDEVICE(INTEL, 0x1539) },
	{ 0 },
};

MODULE_DEVICE_TABLE(pci, i211_pci_table);

#define INDEX(i)	((i) & 0xff)
#define RX_DESC(R, i)	\
	(&(((union adv_rx_desc *)((R)->circ.buf))[i]))
#define TX_DESC(R, i)	\
	(&(((union adv_tx_desc *)((R)->circ.buf))[i]))
#define TXD_DCMD (ADVTXD_DCMD_EOP | ADVTXD_DCMD_RS)

/* Additional Receive Descriptor Control definitions */
#define RXDCTL_QUEUE_ENABLE  0x02000000 /* Enable specific Rx Queue */

static void i211_update_stats(struct adapter *adap);

static inline u32 i211_read(struct adapter *adap, u32 reg)
{
	return readl(adap->io_addr + reg);
}

static inline void i211_write(struct adapter *adap, u32 reg, u32 value)
{
	writel(value, adap->io_addr + reg);
}

static inline void i211_write_array(struct adapter *adap, u32 reg, u32 off, u32 value)
{
	writel(value, adap->io_addr + reg + (off << 2));
}

static inline u32 i211_read_array(struct adapter *adap, u32 reg, u32 off)
{
	return readl(adap->io_addr + reg + (off << 2));
}

static inline int i211_desc_unused(struct ring *ring)
{
	return CIRC_SPACE(ring->circ.head, ring->circ.tail, ring->count);
}

static inline void ring_head_inc(struct ring *ring)
{
	ring->circ.head = (ring->circ.head + 1) & (ring->count - 1);
}

static inline void ring_head_dec(struct ring *ring)
{
	ring->circ.head = (ring->circ.head - 1) & (ring->count - 1);
}

static inline void ring_tail_inc(struct ring *ring)
{
	ring->circ.tail = (ring->circ.tail + 1) & (ring->count - 1);
}

static inline struct netdev_queue *txring_txq(const struct ring *tx_ring)
{
	return netdev_get_tx_queue(tx_ring->netdev, tx_ring->queue_index);
}

static irqreturn_t msix_other(int irq, void *data)
{
	u32 icr;
	struct adapter *adap = data;

	icr = i211_read(adap, ICR);
	if (icr & ICR_DRSTA)
		dev_info(&adap->pdev->dev, "reset interrupt\n");

	if (icr & ICR_DOUTSYNC)
		dev_info(&adap->pdev->dev, "dma out of sync\n");

	if (icr & ICR_LSC)
		schedule_work(&adap->watchdog_task);

	i211_write(adap, EIMS, 0x1f);

	return IRQ_HANDLED;
}

static irqreturn_t msix_ring(int irq, void *data)
{
	struct qvector *q = data;

	napi_schedule(&q->napi);
	return IRQ_HANDLED;
}

static int check_reset_block(struct adapter *adap)
{
	u32 manc;

	manc = i211_read(adap, MANC);

	if (manc & MANC_BLK_PHY_RST_ON_IDE) {
		dev_warn(&adap->pdev->dev,
			 "PHY reset is blocked due to SOL/IDER session.\n");
	}

	return (manc & MANC_BLK_PHY_RST_ON_IDE) ? BLK_PHY_RESET : 0;
}

static void i211_irq_disable(struct adapter *adap)
{
	int val;
	int i;

	val = i211_read(adap, EIAM);
	i211_write(adap, EIAM, val & ~0x1f);
	i211_write(adap, EIMC, 0x1f);
	val = i211_read(adap, EIAC);
	i211_write(adap, EIAC, val & ~0x1f);
	i211_write(adap, IAM, 0);
	i211_write(adap, IMC, ~0);
	wrfl(adap);

	for (i = 0; i < 5; i++)
		synchronize_irq(pci_irq_vector(adap->pdev, i));
}

static void i211_irq_enable(struct adapter *adapter)
{
	u32 ims = IMS_LSC | IMS_DOUTSYNC | IMS_DRSTA;
	u32 val;

	val = i211_read(adapter, EIAC);
	i211_write(adapter, EIAC, val | 0x1f);
	val = i211_read(adapter, EIAM);
	i211_write(adapter, EIAM, val | 0x1f);
	i211_write(adapter, EIMS, 0x1f);
	i211_write(adapter, IMS, ims);
}

static int get_auto_rd_done(struct adapter *adapter)
{
	s32 i = 0;
	s32 ret_val = 0;


	while (i < AUTO_READ_DONE_TIMEOUT) {
		if (i211_read(adapter, EECD) & EECD_AUTO_RD)
			break;
		usleep_range(1000, 2000);
		i++;
	}

	if (i == AUTO_READ_DONE_TIMEOUT) {
		dev_err(&adapter->pdev->dev, "read done timeout\n");
		ret_val = -ERR_RESET;
		goto out;
	}

out:
	return ret_val;
}

static void put_hw_semaphore(struct adapter *adap)
{
	u32 swsm;

	swsm = i211_read(adap, SWSM);

	swsm &= ~(SWSM_SMBI | SWSM_SWESMBI);

	i211_write(adap, SWSM, swsm);
}

static s32 mac_get_hw_semaphore(struct adapter *adap)
{
	u32 swsm;
	s32 timeout = 256 + 1;
	s32 i = 0;

	/* Get the SW semaphore */
	while (i < timeout) {
		swsm = i211_read(adap, SWSM);
		if (!(swsm & SWSM_SMBI))
			break;

		udelay(50);
		i++;
	}

	if (i == timeout) {
		/* If we do not have the semaphore here, we have to give up. */
		if (i == timeout) {
			dev_err(&adap->pdev->dev,
				"Driver can't access device - SMBI bit is set.\n");
			return -ERR_NVM;
		}
	}

	/* Get the FW semaphore. */
	for (i = 0; i < timeout; i++) {
		swsm = i211_read(adap, SWSM);
		i211_write(adap, SWSM, swsm | SWSM_SWESMBI);

		/* Semaphore acquired if bit latched */
		if (i211_read(adap, SWSM) & SWSM_SWESMBI)
			break;

		udelay(50);
	}

	if (i == timeout) {
		/* Release semaphores */
		put_hw_semaphore(adap);
		dev_err(&adap->pdev->dev, "Driver can't access the NVM\n");
		return -ERR_NVM;
	}

	return 0;
}

static void mac_release_swfw_sync(struct adapter *adap, u16 mask)
{
	u32 swfw_sync;

	while (mac_get_hw_semaphore(adap))
		; /* Empty */

	swfw_sync = i211_read(adap, SW_FW_SYNC);
	swfw_sync &= ~mask;
	i211_write(adap, SW_FW_SYNC, swfw_sync);

	put_hw_semaphore(adap);
}

static int mac_acquire_swfw_sync(struct adapter *adap, u16 mask)
{
	u32 swfw_sync;
	u32 swmask = mask;
	u32 fwmask = mask << 16;
	s32 ret_val = 0;
	s32 i = 0, timeout = 200;

	while (i < timeout) {
		if (mac_get_hw_semaphore(adap)) {
			ret_val = -ERR_SWFW_SYNC;
			goto out;
		}

		swfw_sync = i211_read(adap, SW_FW_SYNC);
		if (!(swfw_sync & (fwmask | swmask)))
			break;

		/* Firmware currently using resource (fwmask) */
		put_hw_semaphore(adap);
		mdelay(5);
		i++;
	}

	if (i == timeout) {
		dev_err(&adap->pdev->dev,
			"Driver can't access resource, SW_FW_SYNC timeout.\n");
		ret_val = -ERR_SWFW_SYNC;
		goto out;
	}

	swfw_sync |= swmask;
	i211_write(adap, SW_FW_SYNC, swfw_sync);

	put_hw_semaphore(adap);
out:
	return ret_val;
}

static void i211_hw_reset(struct adapter *adapter)
{
	u32 ctrl, status;
	bool global_reset = true;

	ctrl = i211_read(adapter, CTRL);

	i211_write(adapter, IMC, 0xffffffff);
	i211_write(adapter, RCTL, 0);
	i211_write(adapter, TCTL, TCTL_PSP);
	wrfl(adapter);

	usleep_range(10000, 11000);

	if (mac_acquire_swfw_sync(adapter, SW_SYNCH_MB))
		global_reset = false;

	status = i211_read(adapter, STATUS);
	if (global_reset && !(status & STAT_DEV_RST_SET)) {
		ctrl |= CTRL_DEV_RST;
	} else {
		ctrl |= CTRL_RST;
	}
	i211_write(adapter, CTRL, ctrl);
	wrfl(adapter);
	usleep_range(5000, 6000);

	get_auto_rd_done(adapter);

	i211_write(adapter, STATUS, STAT_DEV_RST_SET);
	i211_write(adapter, IMC, 0xffffffff);
	i211_read(adapter, ICR);

	mac_release_swfw_sync(adapter, SW_SYNCH_MB);
}

static void release_hw_control(struct adapter *adapter)
{
	u32 ctrl_ext;

	/* Let firmware take over control of h/w */

	ctrl_ext = i211_read(adapter, CTRL_EXT);
	i211_write(adapter, CTRL_EXT, ctrl_ext & ~CTRL_EXT_DRV_LOAD);
}

static void get_hw_control(struct adapter *adapter)
{
	u32 ctrl_ext;

	/* Let firmware know the driver has taken over */
	ctrl_ext = i211_read(adapter, CTRL_EXT);
	i211_write(adapter, CTRL_EXT, ctrl_ext | CTRL_EXT_DRV_LOAD);
}

static int phy_acquire(struct adapter *adap)
{
	return mac_acquire_swfw_sync(adap, SWFW_PHY0_SM);
}

static void phy_release(struct adapter *adap)
{
	mac_release_swfw_sync(adap, SWFW_PHY0_SM);
}

static int read_phy_reg(struct adapter *adap, u32 offset, u16 *data)
{
	struct phy_info *phy = &adap->phy;
	u32 i, mdic = 0;
	s32 ret_val = 0;

	if (offset > MAX_PHY_REG_ADDRESS) {
		ret_val = -ERR_PARAM;
		goto out;
	}

	/* Set up Op-code, Phy Address, and register offset in the MDI
	 * Control register.  The MAC will take care of interfacing with the
	 * PHY to retrieve the desired data.
	 */
	mdic = ((offset << MDIC_REG_SHIFT) | (phy->addr << MDIC_PHY_SHIFT) | (MDIC_OP_READ));

	i211_write(adap, MDIC, mdic);

	/* Poll the ready bit to see if the MDI read completed
	 * Increasing the time out as testing showed failures with
	 * the lower time out
	 */
	for (i = 0; i < (GEN_POLL_TIMEOUT * 3); i++) {
		udelay(50);
		mdic = i211_read(adap, MDIC);
		if (mdic & MDIC_READY)
			break;
	}
	if (!(mdic & MDIC_READY)) {
		dev_err(&adap->pdev->dev, "MDI Write did not complete\n");
		ret_val = -ERR_PHY;
		goto out;
	}
	if (mdic & MDIC_ERROR) {
		dev_err(&adap->pdev->dev, "MDI Error\n");
		ret_val = -ERR_PHY;
		goto out;
	}
	*data = (u16) mdic;

out:
	return ret_val;
}

static int write_phy_reg(struct adapter *adap, u32 offset, u16 data)
{
	struct phy_info *phy = &adap->phy;
	u32 i, mdic = 0;
	s32 ret_val = 0;

	if (offset > MAX_PHY_REG_ADDRESS) {
		ret_val = -ERR_PARAM;
		goto out;
	}

	/* Set up Op-code, Phy Address, and register offset in the MDI
	 * Control register.  The MAC will take care of interfacing with the
	 * PHY to retrieve the desired data.
	 */
	mdic = (((u32)data) | (offset << MDIC_REG_SHIFT) | (phy->addr << MDIC_PHY_SHIFT) | (MDIC_OP_WRITE));

	i211_write(adap, MDIC, mdic);

	/* Poll the ready bit to see if the MDI read completed
	 * Increasing the time out as testing showed failures with
	 * the lower time out
	 */
	for (i = 0; i < (GEN_POLL_TIMEOUT * 3); i++) {
		udelay(50);
		mdic = i211_read(adap, MDIC);
		if (mdic & MDIC_READY)
			break;
	}
	if (!(mdic & MDIC_READY)) {
		dev_err(&adap->pdev->dev, "MDI Write did not complete\n");
		ret_val = -ERR_PHY;
		goto out;
	}
	if (mdic & MDIC_ERROR) {
		dev_err(&adap->pdev->dev, "MDI Err\n");
		ret_val = -ERR_PHY;
		goto out;
	}

out:
	return ret_val;
}

static int phy_get_cfg_done(struct adapter *adap)
{
	s32 timeout = PHY_CFG_TIMEOUT;
	u32 mask = NVM_CFG_DONE_PORT_0;

	while (timeout) {
		if (i211_read(adap, EEMNGCTL_I210) & mask)
			break;
		usleep_range(1000, 2000);
		timeout--;
	}
	if (!timeout)
		dev_warn(&adap->pdev->dev, "MNG configuration cycle has not completed.\n");

	return 0;
}

static int phy_reset(struct adapter *adap)
{
	int ret_val;
	u32 ctrl;

	ret_val = check_reset_block(adap);

	ret_val = phy_acquire(adap);
	if (ret_val)
		goto out;

	ctrl = i211_read(adap, CTRL);
	i211_write(adap, CTRL, ctrl | CTRL_PHY_RST);
	wrfl(adap);

	udelay(100);

	i211_write(adap, CTRL, ctrl);
	wrfl(adap);

	udelay(150);

	phy_release(adap);

	ret_val = phy_get_cfg_done(adap);

out:
	return ret_val;
}

static int get_phy_id(struct adapter *adap)
{
	s32 ret_val = 0;
	u16 phy_id;
	struct phy_info *phy = &adap->phy;

	ret_val = read_phy_reg(adap, PHY_ID1, &phy_id);
	if (ret_val)
		goto out;

	phy->id = (u32)(phy_id << 16);
	udelay(20);
	ret_val = read_phy_reg(adap, PHY_ID2, &phy_id);
	if (ret_val)
		goto out;

	phy->id |= (u32)(phy_id & PHY_REVISION_MASK);
	phy->revision = (u32)(phy_id & ~PHY_REVISION_MASK);

out:
	return ret_val;
}

static void mac_config_collision_dist(struct adapter *adap)
{
	u32 tctl;

	tctl = i211_read(adap, TCTL);

	tctl &= ~TCTL_COLD;
	tctl |= COLLISION_DISTANCE << COLD_SHIFT;

	i211_write(adap, TCTL, tctl);
	wrfl(adap);
}

static void setup_fc_watermarks(struct adapter *adap)
{
	u32 fcrtl, fcrth;

	if (adap->fc.current_mode & fc_tx_pause) {
		fcrtl = adap->fc.low_water;
		if (adap->fc.send_xon)
			fcrtl |= FCRTL_XONE;
		fcrth = adap->fc.high_water;
	}

	i211_write(adap, FCRTL, fcrtl);
	i211_write(adap, FCRTH, fcrth);
}

static void setup_fc(struct adapter *adap)
{
	u32 pba, hwm;
	struct fc_info *fc = &adap->fc;

	pba = PBA_34K;
	hwm = (pba << 10) - (adap->max_frame_size + MAX_JUMBO_FRAME_SIZE);

	fc->high_water = hwm & 0xFFFFFFF0;	/* 16-byte granularity */
	fc->low_water = fc->high_water - 16;
	fc->pause_time = 0xFFFF;
	fc->send_xon = 1;
	fc->current_mode = fc->requested_mode;
}

static s32 setup_link(struct adapter *adap)
{
	u32 ctrl;
	u32 phpm_reg;

	/* In the case of the phy reset being blocked, we already have a link.
	 * We do not need to set it up again.
	 */
	if (check_reset_block(adap))
		return -EBUSY;

	ctrl = i211_read(adap, CTRL);
	ctrl |= CTRL_SLU;
	ctrl &= ~(CTRL_FRCSPD | CTRL_FRCDPX);
	i211_write(adap, CTRL, ctrl);

	/* Clear Go Link Disconnect bit on supported devices */
	phpm_reg = i211_read(adap, I82580_PHY_POWER_MGMT);
	phpm_reg &= ~I82580_PM_GO_LINKD;
	i211_write(adap, I82580_PHY_POWER_MGMT, phpm_reg);

	/* Initialize the flow control address, type, and PAUSE timer
	 * registers to their default values.  This is done even if flow
	 * control is disabled, because it does not hurt anything to
	 * initialize these registers.
	 */
	i211_write(adap, FCT, FLOW_CONTROL_TYPE);
	i211_write(adap, FCAH, FLOW_CONTROL_ADDRESS_HIGH);
	i211_write(adap, FCAL, FLOW_CONTROL_ADDRESS_LOW);
	i211_write(adap, FCTTV, adap->fc.pause_time);

	setup_fc_watermarks(adap);

	return 0;
}

static void i211_setup_link(struct adapter *adap)
{
	phy_reset(adap);

	setup_link(adap);
}

static void i211_read_mac_addr(struct adapter *adap)
{
	u32 rar_high;
	u32 rar_low;
	u16 i;

	rar_high = i211_read(adap, RAH(0));
	rar_low = i211_read(adap, RAL(0));

	for (i = 0; i < RAL_MAC_ADDR_LEN; i++)
		adap->mac.perm_addr[i] = (u8)(rar_low >> (i*8));

	for (i = 0; i < RAH_MAC_ADDR_LEN; i++)
		adap->mac.perm_addr[i+4] = (u8)(rar_high >> (i*8));

	for (i = 0; i < ETH_ALEN; i++)
		adap->mac.addr[i] = adap->mac.perm_addr[i];
}

static int get_speed_and_duplex(struct adapter *adap, u16 *speed, u16 *duplex)
{
	u32 status;

	status = i211_read(adap, STATUS);
	if (status & STATUS_SPEED_1000) {
		*speed = SPEED_1000;
	} else if (status & STATUS_SPEED_100) {
		*speed = SPEED_100;
	} else {
		*speed = SPEED_10;
	}

	if (status & STATUS_FD) {
		*duplex = FULL_DUPLEX;
	} else {
		*duplex = HALF_DUPLEX;
	}

	return 0;
}

static void write_ivar(struct adapter *adap, int msix_vector, int index, int offset)
{
	u32 ivar = i211_read(adap, IVAR0 + (index << 2));

	/* clear any bits that are currently set */
	ivar &= ~((u32)0xFF << offset);

	/* write vector and valid bit */
	ivar |= (msix_vector | IVAR_VALID) << offset;

	i211_write(adap, IVAR0 + (index << 2), ivar);
}

static void i211_configure_msix(struct adapter *adap)
{
	u32 tmp;
	int vector = 0;

	/* Turn on MSI-X capability first, or our settings
	 * won't stick.  And it will take days to debug.
	 */
	i211_write(adap, GPIE, GPIE_MSIX_MODE | GPIE_PBA | GPIE_EIAME | GPIE_NSICR);

	/* enable msix_other interrupt */
	tmp = (vector++ | IVAR_VALID) << 8;
	i211_write(adap, IVAR_MISC, tmp);

	write_ivar(adap, 1, 0, 0);
	write_ivar(adap, 2, 0, 16);
	write_ivar(adap, 3, 0, 8);
	write_ivar(adap, 4, 0, 24);

	wrfl(adap);
}

static u32 i211_rx_offset(struct ring *ring)
{
	return SKB_WITH_OVERHEAD(2048) - 1536;
}

static bool alloc_mapped_page(struct ring *ring, struct rx_buffer *bi)
{
	unsigned int offset;

	bi->page = page_pool_alloc_frag(ring->page_pool, &offset, 2048, GFP_ATOMIC);
	if (!bi->page)
		return false;

	bi->dma = page_pool_get_dma_addr(bi->page);
	bi->page = bi->page;
	bi->page_offset = offset + i211_rx_offset(ring);

	return true;
}

static void alloc_rx_buffers(struct ring *rx_ring, u16 cleaned_count)
{
	union adv_rx_desc *rx_desc;
	struct rx_buffer *bi;
	struct circ_buf *circ = &rx_ring->circ;
	int head = circ->head;
	u16 bufsz;

	/* nothing to do */
	if (!cleaned_count)
		return;

	rx_desc = RX_DESC(rx_ring, circ->head);
	bi = &rx_ring->rx_buffer[circ->head];

	bufsz = 1536;

	do {
		if (!alloc_mapped_page(rx_ring, bi))
			break;

		rx_desc->read.pkt_addr = cpu_to_le64(bi->dma + bi->page_offset);

		ring_head_inc(rx_ring);

		rx_desc = RX_DESC(rx_ring, circ->head);
		bi = rx_ring->rx_buffer + circ->head;

		rx_desc->wb.upper.length = 0;

		cleaned_count--;
	} while (cleaned_count);

	if (circ->head != head)
		writel(circ->head, rx_ring->desc_tail);
}

#define TXDCTL_QUEUE_ENABLE  0x02000000 /* Enable specific Tx Queue */

static void configure_tx_ring(struct adapter *adap, struct ring *ring)
{
	u32 txdctl = 0;
	u64 tdba = ring->dma;
	int reg_idx = ring->reg_idx;

	i211_write(adap, TDLEN(reg_idx), 256 * sizeof(union adv_tx_desc));
	i211_write(adap, TDBAL(reg_idx), tdba & 0x00000000ffffffffULL);
	i211_write(adap, TDBAH(reg_idx), tdba >> 32);

	ring->desc_tail = adap->io_addr + TDT(reg_idx);
	i211_write(adap, TDH(reg_idx), 0);
	writel(0, ring->desc_tail);

	txdctl |= 8;
	txdctl |= 1 << 8;
	txdctl |= 16 << 16;

	/* reinitialize tx_buffer_info */
	memset(ring->tx_buffer, 0, sizeof(struct tx_buffer) * 256);

	txdctl |= TXDCTL_QUEUE_ENABLE;
	i211_write(adap, TXDCTL(reg_idx), txdctl);
}

static int clean_tx_irq(struct qvector *q, int napi_budget)
{
	struct ring *tx_ring = &q->ring;
	struct tx_buffer *tx_buffer;
	union adv_tx_desc *tx_desc;
	unsigned int total_bytes = 0, total_packets = 0;
	unsigned int budget = 128;
	struct circ_buf *circ = &tx_ring->circ;

	tx_buffer = &tx_ring->tx_buffer[circ->tail];
	tx_desc = TX_DESC(tx_ring, circ->tail);

	do {
		union adv_tx_desc *eop_desc = tx_buffer->next_to_watch;

		if (!eop_desc)
			break;

		/* prevent any other reads prior to eop_desc */
		smp_rmb();

		/* if DD is not set pending work has not been completed */
		if (!(eop_desc->wb.status & cpu_to_le32(TXD_STAT_DD)))
			break;

		tx_buffer->next_to_watch = NULL;

		/* update the statistics for this packet */
		total_bytes += tx_buffer->bytecount;
		total_packets += tx_buffer->gso_segs;

		napi_consume_skb(tx_buffer->skb, napi_budget);

		/* unmap skb header data */
		dma_unmap_single(tx_ring->dev,
				 dma_unmap_addr(tx_buffer, dma),
				 dma_unmap_len(tx_buffer, len),
				 DMA_TO_DEVICE);

		/* clear tx_buffer data */
		dma_unmap_len_set(tx_buffer, len, 0);

		while (tx_desc != eop_desc) {
			ring_tail_inc(tx_ring);
			tx_buffer = tx_ring->tx_buffer + circ->tail;
			tx_desc = TX_DESC(tx_ring, circ->tail);

			/* unmap any remaining paged data */
			if (dma_unmap_len(tx_buffer, len)) {
				dma_unmap_page(tx_ring->dev,
					       dma_unmap_addr(tx_buffer, dma),
					       dma_unmap_len(tx_buffer, len),
					       DMA_TO_DEVICE);
				dma_unmap_len_set(tx_buffer, len, 0);
			}
		}

		/* move us one more past the eop_desc for start of next pkt */
		ring_tail_inc(tx_ring);
		tx_buffer = tx_ring->tx_buffer + circ->tail;
		tx_desc = TX_DESC(tx_ring, circ->tail);

		/* issue prefetch for next Tx descriptor */
		prefetch(tx_desc);

		/* update budget accounting */
		budget--;
	} while (likely(budget));

	__netif_txq_completed_wake(txring_txq(tx_ring), total_packets,
			total_bytes, i211_desc_unused(tx_ring), DESC_NEEDED, false);

	u64_stats_update_begin(&tx_ring->tx_syncp);
	tx_ring->tx_stats.bytes += total_bytes;
	tx_ring->tx_stats.packets += total_packets;
	u64_stats_update_end(&tx_ring->tx_syncp);

	pr_debug("%s %s reap %u packets %u bytes\n", __func__,
		 tx_ring->name, total_packets, total_bytes);
	return !!budget;
}

static struct sk_buff *i211_alloc_skb(struct ring *ring,
		void *buf, u32 offset, u32 size, ktime_t timestamp)
{
	struct sk_buff *skb;

	net_prefetch(buf);

	skb = napi_build_skb(buf, 2048);
	if (unlikely(!skb))
		return NULL;

	skb_mark_for_recycle(skb);

	skb_reserve(skb, offset);
	__skb_put(skb, size);

	if (timestamp)
		skb_hwtstamps(skb)->hwtstamp = timestamp;

	return skb;
}

static inline __le32 i211_test_staterr(union adv_rx_desc *rx_desc, const u32 stat_err_bits)
{
	return rx_desc->wb.upper.status_error & cpu_to_le32(stat_err_bits);
}

static bool i211_is_non_eop(struct ring *rx_ring, union adv_rx_desc *rx_desc)
{
	ring_tail_inc(rx_ring);
	prefetch(RX_DESC(rx_ring, rx_ring->circ.tail));

	if (likely(i211_test_staterr(rx_desc, RXD_STAT_EOP)))
		return false;

	return true;
}

static inline void i211_rx_hash(struct ring *ring,
			       union adv_rx_desc *rx_desc,
			       struct sk_buff *skb)
{
	if (ring->netdev->features & NETIF_F_RXHASH)
		skb_set_hash(skb,
			     le32_to_cpu(rx_desc->wb.lower.hi_dword.rss),
			     PKT_HASH_TYPE_L3);
}

static inline void i211_rx_checksum(struct ring *ring,
				   union adv_rx_desc *rx_desc,
				   struct sk_buff *skb)
{
	/* Ignore Checksum bit is set */
	if (i211_test_staterr(rx_desc, RXD_STAT_IXSM))
		return;

	/* Rx checksum disabled via ethtool */
	if (!(ring->netdev->features & NETIF_F_RXCSUM))
		return;

	/* TCP/UDP checksum error bit is set */
	if (i211_test_staterr(rx_desc, RXDEXT_STATERR_TCPE | RXDEXT_STATERR_IPE)) {
		return;
	}
	/* It must be a TCP or UDP packet with a valid checksum */
	if (i211_test_staterr(rx_desc, RXD_STAT_TCPCS | RXD_STAT_UDPCS))
		skb->ip_summed = CHECKSUM_UNNECESSARY;

	dev_dbg(ring->dev, "cksum success: bits %08X\n",
		le32_to_cpu(rx_desc->wb.upper.status_error));
}

static void i211_process_skb_fields(struct ring *rx_ring,
				   union adv_rx_desc *rx_desc,
				   struct sk_buff *skb)
{
	struct net_device *ndev = rx_ring->netdev;

	i211_rx_hash(rx_ring, rx_desc, skb);

	i211_rx_checksum(rx_ring, rx_desc, skb);

	if (ndev->features & NETIF_F_HW_VLAN_CTAG_RX &&
	    i211_test_staterr(rx_desc, RXD_STAT_VP)) {
		u16 vid;

		if (i211_test_staterr(rx_desc, RXDEXT_STATERR_LB))
			vid = be16_to_cpu(rx_desc->wb.upper.vlan);
		else
			vid = le16_to_cpu(rx_desc->wb.upper.vlan);

		__vlan_hwaccel_put_tag(skb, htons(ETH_P_8021Q), vid);
	}

	skb_record_rx_queue(skb, rx_ring->queue_index);

	skb->protocol = eth_type_trans(skb, rx_ring->netdev);
}

static void i211_add_rx_frag(struct ring *rx_ring,
			    struct rx_buffer *rx_buffer,
			    struct sk_buff *skb,
			    unsigned int size)
{
	unsigned int truesize = 2048;
	skb_add_rx_frag(skb, skb_shinfo(skb)->nr_frags, rx_buffer->page,
			rx_buffer->page_offset, size, truesize);
}

static int clean_rx_irq(struct qvector *q, int budget)
{
	int count = 0;
	struct ring *ring = &q->ring;
	struct sk_buff *skb = ring->skb;
	u32 total_packets = 0, total_bytes = 0;
	u16 cleaned_count = i211_desc_unused(ring);

	while (count < budget) {
		union adv_rx_desc *desc;
		u32 size;
		struct rx_buffer *rxbuf;

		desc = RX_DESC(ring, ring->circ.tail);
		size = le16_to_cpu(desc->wb.upper.length);
		if (!size)
			break;

		rxbuf = ring->rx_buffer + ring->circ.tail;

		dma_sync_single_range_for_cpu(ring->dev, rxbuf->dma, rxbuf->page_offset, size, DMA_FROM_DEVICE);
		if (!skb) {
			void *pkt = page_address(rxbuf->page) + rxbuf->page_offset;
			void *base = pkt - i211_rx_offset(ring);

			skb = i211_alloc_skb(ring, base, i211_rx_offset(ring), size, 0);
			if (!skb) {
				dev_err(ring->dev, "failed to build skb\n");
				ring->rx_stats.alloc_failed++;
				break;
			}
		} else {
			i211_add_rx_frag(ring, rxbuf, skb, size);
		}

		cleaned_count++;
		count++;

		if (i211_is_non_eop(ring, desc))
			continue;

		total_packets++;
		total_bytes += skb->len;
		i211_process_skb_fields(ring, desc, skb);
		napi_gro_receive(&q->napi, skb);

		skb = NULL;
	}

	ring->skb = skb;
	u64_stats_update_begin(&ring->rx_syncp);
	ring->rx_stats.packets += total_packets;
	ring->rx_stats.bytes += total_bytes;
	u64_stats_update_end(&ring->rx_syncp);

	pr_debug("%s %s reap %u packets %u bytes\n", __func__,
		 ring->name, total_packets, total_bytes);

	if (cleaned_count)
		alloc_rx_buffers(ring, cleaned_count);

	return count;
}

static void ring_irq_enable(struct qvector *q)
{
	struct ring *ring = &q->ring;
	struct adapter *adap = q->adap;

	if (ring->tx_buffer) {
		if (ring->queue_index == 0)
			i211_write(adap, EIMS, 0x8);
		else
			i211_write(adap, EIMS, 0x10);
	} else {
		if (ring->queue_index == 0)
			i211_write(adap, EIMS, 0x2);
		else
			i211_write(adap, EIMS, 0x4);
	}
}

static int i211_poll(struct napi_struct *napi, int budget)
{
	struct qvector *q = container_of(napi, struct qvector, napi);
	struct ring *ring = &q->ring;
	int count = 0, rx_count = 0;
	bool complete = true;

	if (ring->tx_buffer) {
		complete = clean_tx_irq(q, budget);
	}

	if (ring->rx_buffer) {
		rx_count = clean_rx_irq(q, budget);
		count = rx_count;
		if (count >= budget)
			complete = false;
	}

	if (!complete)
		return budget;

	if (napi_complete_done(napi, count)) {
		ring_irq_enable(q);
	}

	return count;
}

static void init_qvectors(struct adapter *adap)
{
	int i;

	for (i = 0; i < 2; i++) {
		struct qvector *qv = &adap->qvector[i];
		struct ring *ring = &qv->ring;

		qv->adap = adap;
		ring->reg_idx = i;
		ring->queue_index = i;
		ring->netdev = adap->netdev;
		ring->dev = &adap->pdev->dev;
		snprintf(ring->name, sizeof(ring->name), "rx%d", i);

		netif_napi_add_weight(adap->netdev, &qv->napi, i211_poll, NAPI_POLL_WEIGHT);
	}

	for (; i < 4; i++) {
		struct qvector *qv = &adap->qvector[i];
		struct ring *ring = &qv->ring;

		qv->adap = adap;
		ring->reg_idx = i - 2;
		ring->queue_index = i - 2;
		ring->netdev = adap->netdev;
		ring->dev = &adap->pdev->dev;
		snprintf(ring->name, sizeof(ring->name), "tx%d", i - 2);
		netif_napi_add_weight(adap->netdev, &qv->napi, i211_poll, NAPI_POLL_WEIGHT);
	}
}

static int i211_init_interrupt(struct adapter *adap)
{
	int ret;

	ret = pci_alloc_irq_vectors(adap->pdev, 5, 5, PCI_IRQ_MSIX);
	if (ret < 0) {
		dev_err(&adap->pdev->dev, "failed to alloc irq vectors: %d\n", ret);
		return ret;
	}

	init_qvectors(adap);
	return 0;
}

static void clear_interrupt(struct adapter *adap)
{
	int i;
	for (i = 0; i < 4; i++)
		netif_napi_del(&adap->qvector[i].napi);
	pci_free_irq_vectors(adap->pdev);
}

static int request_msix_irq(struct adapter *adap)
{
	int i, ret, vector;
	struct pci_dev *pdev = adap->pdev;
	const char *names[5] = { "i211", "i211-rx0", "i211-rx1", "i211-tx0", "i211-tx1" };

	ret = request_irq(pci_irq_vector(pdev, 0), msix_other, 0, names[0], adap);
	if (ret) {
		dev_err(&pdev->dev, "failed to request irq for i211: %d\n", ret);
		goto err;
	}

	for (i = 0, vector = 1; i < 4; i++, vector++) {
		ret = request_irq(pci_irq_vector(pdev, vector), msix_ring, 0,
				  names[vector], &adap->qvector[i]);
		if (ret) {
			dev_err(&pdev->dev, "failed to request irq for %s: %d\n",
				names[vector], ret);
			goto err_queue;
		}
	}

	i211_configure_msix(adap);

	return 0;

err_queue:
	free_irq(pci_irq_vector(pdev, 0), adap);

	vector--;
	for (i = vector - 1; vector > 0; vector--, i--)
		free_irq(pci_irq_vector(pdev, vector), &adap->qvector[i]);
err:
	return ret;
}

static void free_msix_irq(struct adapter *adap)
{
	int i, vector;

	free_irq(pci_irq_vector(adap->pdev, 0), adap);
	for (i = 0, vector = 1; i < 4; i++, vector++)
		free_irq(pci_irq_vector(adap->pdev, vector), &adap->qvector[i]);
}

static void clean_tx_ring(struct ring *tx_ring)
{
	struct circ_buf *circ = &tx_ring->circ;
	struct tx_buffer *tx_buffer = &tx_ring->tx_buffer[circ->tail];

	while (circ->head != circ->tail) {
		union adv_tx_desc *eop_desc, *tx_desc;

		/* Free all the Tx ring sk_buffs or xdp frames */
		dev_kfree_skb_any(tx_buffer->skb);

		/* unmap skb header data */
		dma_unmap_single(tx_ring->dev,
				 dma_unmap_addr(tx_buffer, dma),
				 dma_unmap_len(tx_buffer, len),
				 DMA_TO_DEVICE);

		/* check for eop_desc to determine the end of the packet */
		eop_desc = tx_buffer->next_to_watch;
		tx_desc = TX_DESC(tx_ring, circ->tail);

		/* unmap remaining buffers */
		while (tx_desc != eop_desc) {
			ring_tail_inc(tx_ring);
			tx_buffer = tx_ring->tx_buffer + circ->tail;
			tx_desc = TX_DESC(tx_ring, circ->tail);

			/* unmap any remaining paged data */
			if (dma_unmap_len(tx_buffer, len))
				dma_unmap_page(tx_ring->dev,
					       dma_unmap_addr(tx_buffer, dma),
					       dma_unmap_len(tx_buffer, len),
					       DMA_TO_DEVICE);
		}

		tx_buffer->next_to_watch = NULL;

		ring_tail_inc(tx_ring);
		tx_buffer = tx_ring->tx_buffer + circ->tail;
	}

	/* reset BQL for queue */
	netdev_tx_reset_queue(txring_txq(tx_ring));
}

static void clean_rx_buffers(struct ring *rx_ring)
{
	struct circ_buf *circ = &rx_ring->circ;

	dev_kfree_skb(rx_ring->skb);
	rx_ring->skb = NULL;

	while (circ->head != circ->tail) {
		struct rx_buffer *buffer_info = &rx_ring->rx_buffer[circ->tail];

		page_pool_put_full_page(rx_ring->page_pool, buffer_info->page, true);

		ring_tail_inc(rx_ring);
	}
}

static void free_rx_resources(struct ring *ring)
{
	clean_rx_buffers(ring);
	page_pool_destroy(ring->page_pool);
	if (ring->rx_buffer) {
		vfree(ring->rx_buffer);
	}
	ring->rx_buffer = NULL;
	if (ring->circ.buf) {
		dma_free_coherent(ring->dev, ring->size, ring->circ.buf, ring->dma);
	}
}

static int setup_tx_resources(struct ring *ring)
{
	ring->tx_buffer = vmalloc(256 * sizeof(struct tx_buffer));

	ring->size = 256 * sizeof(union adv_tx_desc);
	ring->size = ALIGN(ring->size, 4096);
	ring->circ.buf = dma_alloc_coherent(ring->dev, ring->size, &ring->dma, GFP_KERNEL);
	ring->circ.head = 0;
	ring->circ.tail = 0;
	ring->count = 256;
	if (!ring->tx_buffer || !ring->circ.buf) {
		dev_err(ring->dev, "queue tx%d: tx alloc error\n", ring->queue_index);
	}

	return 0;
}

static void free_tx_resources(struct ring *ring)
{
	clean_tx_ring(ring);
	if (ring->tx_buffer) {
		vfree(ring->tx_buffer);
	}
	ring->tx_buffer = NULL;
	if (ring->circ.buf) {
		dma_free_coherent(ring->dev, ring->size, ring->circ.buf, ring->dma);
	}
}

static int setup_all_tx_resources(struct adapter *adap)
{
	int i;

	for (i = 2; i < 4; i++) {
		setup_tx_resources(&adap->qvector[i].ring);
	}

	return 0;
}

static void free_all_tx_resources(struct adapter *adap)
{
	int i;

	for (i = 2; i < 4; i++) {
		free_tx_resources(&adap->qvector[i].ring);
	}
}

static int alloc_page_pool(struct ring *ring)
{
	struct page_pool_params pp_params = {
		.order = 0,
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 6, 0)
		.flags = PP_FLAG_DMA_MAP | PP_FLAG_DMA_SYNC_DEV | PP_FLAG_PAGE_FRAG,
#else
		.flags = PP_FLAG_DMA_MAP | PP_FLAG_DMA_SYNC_DEV,
#endif
		.pool_size = 256,
		.nid = dev_to_node(ring->dev),
		.dev = ring->dev,
		.dma_dir = DMA_FROM_DEVICE,
		.offset = 0,
		.max_len = PAGE_SIZE,
	};
	struct page_pool *pp;

	pp = page_pool_create(&pp_params);
	if (IS_ERR(pp))
		return PTR_ERR(pp);

	ring->page_pool = pp;
	return 0;
}

static int setup_rx_resources(struct ring *ring)
{
	ring->size = 256 * sizeof(union adv_rx_desc);
	ring->size = ALIGN(ring->size, 4096);

	ring->circ.buf = dma_alloc_coherent(ring->dev, ring->size, &ring->dma, GFP_KERNEL);
	ring->rx_buffer = vmalloc(256 * sizeof(struct rx_buffer));

	ring->circ.head = 0;
	ring->circ.tail = 0;

	ring->count = 256;

	if (!ring->rx_buffer || !ring->circ.buf) {
		dev_err(ring->dev, "queue rx%d: rx alloc error\n", ring->queue_index);
	}

	if (alloc_page_pool(ring))
		dev_err(ring->dev, "queue rx%d: failed to alloc page pool\n", ring->queue_index);

	return 0;
}

static int setup_all_rx_resources(struct adapter *adap)
{
	int i;

	for (i = 0; i < 2; i++)
		setup_rx_resources(&adap->qvector[i].ring);

	return 0;
}

static void free_all_rx_resources(struct adapter *adap)
{
	int i;

	for (i = 0; i < 2; i++)
		free_rx_resources(&adap->qvector[i].ring);
}

#define SRRCTL_BSIZEPKT_SHIFT                     10 /* Shift _right_ */
#define SRRCTL_BSIZEHDRSIZE_SHIFT                 2  /* Shift _left_ */
#define SRRCTL_DESCTYPE_ADV_ONEBUF                0x02000000
#define SRRCTL_DESCTYPE_HDR_SPLIT_ALWAYS          0x0A000000
#define SRRCTL_DROP_EN                            0x80000000
#define SRRCTL_TIMESTAMP                          0x40000000

static void setup_srrctl(struct adapter *adap, struct ring *ring)
{
	int reg_idx = ring->reg_idx;
	u32 srrctl = 0;

	srrctl = 256 << SRRCTL_BSIZEHDRSIZE_SHIFT;
	srrctl |= 2048 >> SRRCTL_BSIZEPKT_SHIFT;
	srrctl |= SRRCTL_DESCTYPE_ADV_ONEBUF;
	srrctl |= SRRCTL_TIMESTAMP;
	/* Only set Drop Enable if VFs allocated, or we are supporting multiple
	 * queues and rx flow control is disabled
	 */
	srrctl |= SRRCTL_DROP_EN;

	i211_write(adap, SRRCTL(reg_idx), srrctl);
}

static void configure_rx_ring(struct adapter *adap, struct ring *ring)
{
	int reg_idx = ring->reg_idx;
	u32 rxdctl = 0;
	u64 rdba = ring->dma;
	union adv_rx_desc *rx_desc;

	/* disable the queue */
	i211_write(adap, RXDCTL(reg_idx), 0);

	/* Set DMA base address registers */
	i211_write(adap, RDBAL(reg_idx), rdba & 0x00000000ffffffffULL);
	i211_write(adap, RDBAH(reg_idx), rdba >> 32);
	i211_write(adap, RDLEN(reg_idx), 256 * sizeof(union adv_rx_desc));

	/* initialize head and tail */
	ring->desc_tail = adap->io_addr + RDT(reg_idx);
	i211_write(adap, RDH(reg_idx), 0);
	writel(0, ring->desc_tail);

	/* set descriptor configuration */
	setup_srrctl(adap, ring);

	rxdctl |= 8;
	rxdctl |= 8 << 8;
	rxdctl |= 4 << 16;

	/* initialize rx_buffer */
	memset(ring->rx_buffer, 0, sizeof(struct rx_buffer) * 256);

	/* initialize Rx descriptor 0 */
	rx_desc = RX_DESC(ring, 0);
	rx_desc->wb.upper.length = 0;

	/* enable receive descriptor fetching */
	rxdctl |= RXDCTL_QUEUE_ENABLE;
	i211_write(adap, RXDCTL(reg_idx), rxdctl);
}

static void configure_rx(struct adapter *adap)
{
	int i;

	for (i = 0; i < 2; i++)
		configure_rx_ring(adap, &adap->qvector[i].ring);
}

static void configure_tx(struct adapter *adap)
{
	int i;

	for (i = 2; i < 4; i++)
		configure_tx_ring(adap, &adap->qvector[i].ring);
}

static void setup_tctl(struct adapter *adap)
{
	u32 tctl;

	/* disable queue 0 which is enabled by default on 82575 and 82576 */
	i211_write(adap, TXDCTL(0), 0);

	/* Program the Transmit Control Register */
	tctl = i211_read(adap, TCTL);
	tctl &= ~TCTL_CT;
	tctl |= TCTL_PSP | TCTL_RTLC | (COLLISION_THRESHOLD << CT_SHIFT);

	mac_config_collision_dist(adap);

	/* Enable transmits */
	tctl |= TCTL_EN;

	i211_write(adap, TCTL, tctl);
}

static void setup_rctl(struct adapter *adap)
{
	u32 rctl;

	rctl = i211_read(adap, RCTL);

	rctl &= ~(3 << RCTL_MO_SHIFT);
	rctl &= ~(RCTL_LBM_TCVR | RCTL_LBM_MAC);

	rctl |= RCTL_EN | RCTL_BAM | RCTL_RDMTS_HALF;

	/* enable stripping of CRC. It's unlikely this will break BMC
	 * redirection as it did with e1000. Newer features require
	 * that the HW strips the CRC.
	 */
	rctl |= RCTL_SECRC;

	/* disable store bad packets and clear size bits. */
	rctl &= ~(RCTL_SBP | RCTL_SZ_256);

	/* enable LPE to allow for reception of jumbo frames */
	rctl |= RCTL_LPE;

	/* disable queue 0 to prevent tail write w/o re-config */
	i211_write(adap, RXDCTL(0), 0);

	i211_write(adap, RCTL, rctl);
}

static void i211_init_hw(struct adapter *adap)
{
	u32 ctrl_ext;

	ctrl_ext = i211_read(adap, CTRL_EXT);
	ctrl_ext &= ~CTRL_I2C_ENA;
	i211_write(adap, CTRL_EXT, ctrl_ext);

	get_phy_id(adap);
	i211_read_mac_addr(adap);

	setup_fc(adap);
}

static int i211_open(struct net_device *netdev)
{
	int i;
	struct adapter *adapter = netdev_priv(netdev);

	pm_runtime_get_sync(&adapter->pdev->dev);
	get_hw_control(adapter);

	/* setup tx/rx resources */
	setup_all_tx_resources(adapter);
	setup_all_rx_resources(adapter);

	i211_setup_link(adapter);

	setup_tctl(adapter);
	setup_rctl(adapter);
	/* configure tx/rx ring */
	configure_tx(adapter);
	configure_rx(adapter);

	for (i = 0; i < 2; i++) {
		alloc_rx_buffers(&adapter->qvector[i].ring, 255);
	}

	request_msix_irq(adapter);
	i211_read(adapter, TSICR);
	i211_read(adapter, ICR);

	netif_set_real_num_tx_queues(netdev, 2);
	netif_set_real_num_rx_queues(netdev, 2);

	clear_bit(STATE_DOWN, &adapter->state);

	for (i = 0; i < 4; i++) {
		napi_enable(&adapter->qvector[i].napi);
	}

	i211_irq_enable(adapter);

	netif_tx_start_all_queues(netdev);

	phy_start(adapter->phy_dev);

	pm_runtime_put(&adapter->pdev->dev);

	return 0;
}

static void i211_reset(struct adapter *adap)
{
	i211_irq_disable(adap);
	i211_hw_reset(adap);
	i211_irq_disable(adap);

	i211_init_hw(adap);
}

static void i211_down(struct adapter *adap)
{
	u32 ctl;
	int i;

	set_bit(STATE_DOWN, &adap->state);

	ctl = i211_read(adap, RCTL);
	i211_write(adap, RCTL, ctl & ~RCTL_EN);

	netif_carrier_off(adap->netdev);
	netif_tx_stop_all_queues(adap->netdev);

	ctl = i211_read(adap, TCTL);
	i211_write(adap, TCTL, ctl & ~TCTL_EN);
	wrfl(adap);
	usleep_range(10000, 11000);

	i211_irq_disable(adap);

	for (i = 0; i < 4; i++) {
		napi_synchronize(&adap->qvector[i].napi);
		napi_disable(&adap->qvector[i].napi);
	}

	/* record the stats before reset*/
	spin_lock(&adap->stats64_lock);
	i211_update_stats(adap);
	spin_unlock(&adap->stats64_lock);

	adap->link_speed = 0;
	adap->link_duplex = 0;

	if (!pci_channel_offline(adap->pdev))
		i211_reset(adap);
}

static int i211_close(struct net_device *netdev)
{
	struct adapter *adapter = netdev_priv(netdev);

	phy_stop(adapter->phy_dev);
	pm_runtime_get_sync(&adapter->pdev->dev);
	i211_down(adapter);

	free_msix_irq(adapter);
	free_all_rx_resources(adapter);
	free_all_tx_resources(adapter);
	pm_runtime_put(&adapter->pdev->dev);

	return 0;
}

#define MAX_DATA_POWER		15
#define MAX_DATA_PER_TXD	(1 << MAX_DATA_POWER)
#define TXD_COUNT(s)		DIV_ROUND_UP((s), MAX_DATA_PER_TXD)

#define ADVTXD_MACLEN_SHIFT	9  /* Adv ctxt desc mac len shift */
#define ADVTXD_TUCMD_L4T_UDP	0x00000000  /* L4 Packet TYPE of UDP */
#define ADVTXD_TUCMD_IPV4	0x00000400  /* IP Packet Type: 1=IPv4 */
#define ADVTXD_TUCMD_L4T_TCP	0x00000800  /* L4 Packet TYPE of TCP */

#define ADVTXD_L4LEN_SHIFT     8  /* Adv ctxt L4LEN shift */
#define ADVTXD_MSS_SHIFT      16  /* Adv ctxt MSS shift */

#define TX_CTXTDESC(R, i)	\
	(&(((struct adv_tx_context_desc *)((R)->circ.buf))[i]))

static void tx_ctxtdesc(struct ring *ring, struct tx_buffer *first,
			u32 vlan_macip_lens, u32 type_tucmd, u32 mss_l4len_idx)
{
	struct adv_tx_context_desc *context_desc;

	context_desc = TX_CTXTDESC(ring, ring->circ.head);

	ring_head_inc(ring);

	/* set bits to identify this as an advanced context descriptor */
	type_tucmd |= TXD_CMD_DEXT | ADVTXD_DTYP_CTXT;
	context_desc->vlan_macip_lens	= cpu_to_le32(vlan_macip_lens);
	context_desc->type_tucmd_mlhl	= cpu_to_le32(type_tucmd);
	context_desc->mss_l4len_idx	= cpu_to_le32(mss_l4len_idx);

	context_desc->seqnum_seed = 0;
}

static void tx_csum(struct ring *ring, struct tx_buffer *first)
{
	struct sk_buff *skb = first->skb;
	u32 vlan_macip_lens = 0;
	u32 type_tucmd = 0;

	if (skb->ip_summed != CHECKSUM_PARTIAL) {
		if (!(first->flags & I211_TX_FLAGS_VLAN))
			return;
		goto no_csum;
	}

	switch (skb->csum_offset) {
	case offsetof(struct tcphdr, check):
		type_tucmd = ADVTXD_TUCMD_L4T_TCP;
	case offsetof(struct udphdr, check):
		break;
	default:
		skb_checksum_help(skb);
		return;
	}

	first->flags |= I211_TX_FLAGS_CSUM;
	vlan_macip_lens = skb_checksum_start_offset(skb) - skb_network_offset(skb);

no_csum:
	vlan_macip_lens |= skb_network_offset(skb) << ADVTXD_MACLEN_SHIFT;
	vlan_macip_lens |= first->flags & I211_TX_FLAGS_VLAN_MASK;

	tx_ctxtdesc(ring, first, vlan_macip_lens, type_tucmd, 0);
}

static u32 tx_cmd_type(struct sk_buff *skb, u32 tx_flags)
{
	/* set type for advanced descriptor */
	u32 cmd_type = ADVTXD_DTYP_DATA | ADVTXD_DCMD_DEXT;

	if (!skb->no_fcs)
		cmd_type |= ADVTXD_DCMD_IFCS;

	if (tx_flags & I211_TX_FLAGS_TSO)
		cmd_type |= ADVTXD_DCMD_TSE;

	if (tx_flags & I211_TX_FLAGS_VLAN)
		cmd_type |= ADVTXD_DCMD_VLE;

	return cmd_type;
}

static void tx_olinfo_status(struct ring *tx_ring, union adv_tx_desc *tx_desc,
			u32 tx_flags, unsigned int paylen)
{
	u32 olinfo_status = paylen << ADVTXD_PAYLEN_SHIFT;

	/* insert L4 checksum */
	if (tx_flags & I211_TX_FLAGS_CSUM)
		olinfo_status |= TXD_POPTS_TXSM << 8;

	/* insert IPv4 checksum */
	if (tx_flags & I211_TX_FLAGS_IPV4)
		olinfo_status |= TXD_POPTS_IXSM << 8;

	tx_desc->read.olinfo_status = cpu_to_le32(olinfo_status);
}

static int tx_map(struct ring *tx_ring, struct tx_buffer *first, const u8 hdr_len)
{
	struct sk_buff *skb = first->skb;
	struct tx_buffer *tx_buffer;
	union adv_tx_desc *tx_desc;
	skb_frag_t *frag;
	dma_addr_t dma;
	unsigned int data_len, size;
	u32 cmd_type = tx_cmd_type(skb, first->flags);
	struct circ_buf *circ = &tx_ring->circ;
	bool doorbell, stop;

	tx_desc = TX_DESC(tx_ring, circ->head);

	tx_olinfo_status(tx_ring, tx_desc, first->flags, skb->len - hdr_len);

	size = skb_headlen(skb);
	data_len = skb->data_len;

	dma = dma_map_single(tx_ring->dev, skb->data, size, DMA_TO_DEVICE);

	tx_buffer = first;

	for (frag = &skb_shinfo(skb)->frags[0];; frag++) {
		if (dma_mapping_error(tx_ring->dev, dma)) {
			dev_err(tx_ring->dev, "%s %d dma_mapping_error\n", __func__, __LINE__);
			goto dma_error;
		}

		/* record length, and DMA address */
		dma_unmap_len_set(tx_buffer, len, size);
		dma_unmap_addr_set(tx_buffer, dma, dma);

		tx_desc->read.buffer_addr = cpu_to_le64(dma);

		if (likely(!data_len))
			break;

		tx_desc->read.cmd_type_len = cpu_to_le32(cmd_type ^ size);

		ring_head_inc(tx_ring);
		tx_desc = TX_DESC(tx_ring, circ->head);
		tx_desc->read.olinfo_status = 0;

		size = skb_frag_size(frag);
		data_len -= size;

		dma = skb_frag_dma_map(tx_ring->dev, frag, 0, size, DMA_TO_DEVICE);

		tx_buffer = &tx_ring->tx_buffer[circ->head];
	}

	/* write last descriptor with RS and EOP bits */
	cmd_type |= size | TXD_DCMD;
	tx_desc->read.cmd_type_len = cpu_to_le32(cmd_type);

	doorbell = __netdev_tx_sent_queue(txring_txq(tx_ring),
				first->bytecount, netdev_xmit_more());

	/* set the timestamp */
	first->time_stamp = jiffies;

	skb_tx_timestamp(skb);

	/* Force memory writes to complete before letting h/w know there
	 * are new descriptors to fetch.  (Only applicable for weak-ordered
	 * memory model archs, such as IA-64).
	 *
	 * We also need this memory barrier to make certain all of the
	 * status bits have been updated before next_to_watch is written.
	 */
	dma_wmb();

	first->next_to_watch = tx_desc;

	ring_head_inc(tx_ring);

	stop = !netif_txq_maybe_stop(txring_txq(tx_ring),
			i211_desc_unused(tx_ring), DESC_NEEDED, DESC_NEEDED);

	if (stop || doorbell)
		writel(circ->head, tx_ring->desc_tail);

	return 0;

dma_error:
	dev_err(tx_ring->dev, "TX DMA map failed\n");
	tx_buffer = &tx_ring->tx_buffer[circ->head];

	/* clear dma mappings for failed tx_buffer_info map */
	while (tx_buffer != first) {
		if (dma_unmap_len(tx_buffer, len))
			dma_unmap_page(tx_ring->dev,
				       dma_unmap_addr(tx_buffer, dma),
				       dma_unmap_len(tx_buffer, len),
				       DMA_TO_DEVICE);
		dma_unmap_len_set(tx_buffer, len, 0);

		ring_head_dec(tx_ring);
		tx_buffer = &tx_ring->tx_buffer[tx_ring->circ.head];
	}

	if (dma_unmap_len(tx_buffer, len))
		dma_unmap_single(tx_ring->dev,
				 dma_unmap_addr(tx_buffer, dma),
				 dma_unmap_len(tx_buffer, len),
				 DMA_TO_DEVICE);
	dma_unmap_len_set(tx_buffer, len, 0);

	dev_kfree_skb_any(tx_buffer->skb);
	tx_buffer->skb = NULL;

	return -1;
}

static int igb_tso(struct ring *tx_ring, struct tx_buffer *first, u8 *hdr_len)
{
	u32 vlan_macip_lens, type_tucmd, mss_l4len_idx;
	struct sk_buff *skb = first->skb;
	union {
		struct iphdr *v4;
		struct ipv6hdr *v6;
		unsigned char *hdr;
	} ip;
	union {
		struct tcphdr *tcp;
		struct udphdr *udp;
		unsigned char *hdr;
	} l4;
	u32 paylen, l4_offset;
	int err;

	if (skb->ip_summed != CHECKSUM_PARTIAL)
		return 0;

	if (!skb_is_gso(skb))
		return 0;

	err = skb_cow_head(skb, 0);
	if (err < 0)
		return err;

	ip.hdr = skb_network_header(skb);
	l4.hdr = skb_checksum_start(skb);

	/* ADV DTYP TUCMD MKRLOC/ISCSIHEDLEN */
	type_tucmd = (skb_shinfo(skb)->gso_type & SKB_GSO_UDP_L4) ?
		      ADVTXD_TUCMD_L4T_UDP : ADVTXD_TUCMD_L4T_TCP;

	/* initialize outer IP header fields */
	if (ip.v4->version == 4) {
		unsigned char *csum_start = skb_checksum_start(skb);
		unsigned char *trans_start = ip.hdr + (ip.v4->ihl * 4);

		/* IP header will have to cancel out any data that
		 * is not a part of the outer IP header
		 */
		ip.v4->check = csum_fold(csum_partial(trans_start,
						      csum_start - trans_start,
						      0));
		type_tucmd |= ADVTXD_TUCMD_IPV4;

		ip.v4->tot_len = 0;
		first->flags |= I211_TX_FLAGS_TSO | I211_TX_FLAGS_CSUM | I211_TX_FLAGS_IPV4;
	} else {
		ip.v6->payload_len = 0;
		first->flags |= I211_TX_FLAGS_TSO | I211_TX_FLAGS_CSUM;
	}

	/* determine offset of inner transport header */
	l4_offset = l4.hdr - skb->data;

	/* remove payload length from inner checksum */
	paylen = skb->len - l4_offset;
	if (type_tucmd & ADVTXD_TUCMD_L4T_TCP) {
		/* compute length of segmentation header */
		*hdr_len = (l4.tcp->doff * 4) + l4_offset;
		csum_replace_by_diff(&l4.tcp->check,
			(__force __wsum)htonl(paylen));
	} else {
		/* compute length of segmentation header */
		*hdr_len = sizeof(*l4.udp) + l4_offset;
		csum_replace_by_diff(&l4.udp->check,
				     (__force __wsum)htonl(paylen));
	}

	/* update gso size and bytecount with header size */
	first->gso_segs = skb_shinfo(skb)->gso_segs;
	first->bytecount += (first->gso_segs - 1) * *hdr_len;

	/* MSS L4LEN IDX */
	mss_l4len_idx = (*hdr_len - l4_offset) << ADVTXD_L4LEN_SHIFT;
	mss_l4len_idx |= skb_shinfo(skb)->gso_size << ADVTXD_MSS_SHIFT;

	/* VLAN MACLEN IPLEN */
	vlan_macip_lens = l4.hdr - ip.hdr;
	vlan_macip_lens |= (ip.hdr - skb->data) << ADVTXD_MACLEN_SHIFT;
	vlan_macip_lens |= first->flags & I211_TX_FLAGS_VLAN_MASK;

	tx_ctxtdesc(tx_ring, first, vlan_macip_lens, type_tucmd, mss_l4len_idx);

	return 1;
}

static netdev_tx_t i211_xmit_frame(struct sk_buff *skb, struct net_device *netdev)
{
	struct adapter *adap = netdev_priv(netdev);
	int count = TXD_COUNT(skb_headlen(skb));
	unsigned int i, index = skb->queue_mapping;
	struct qvector *qv = &adap->qvector[2];
	struct ring *ring;
	unsigned int unused;
	struct tx_buffer *first;
	int tso;
	u8 hdr_len;

	if (index >= 2)
		index = index % 2;

	ring = &(qv + index)->ring;

	for (i = 0; i < skb_shinfo(skb)->nr_frags; i++)
		count += TXD_COUNT(skb_frag_size(&skb_shinfo(skb)->frags[i]));

	unused = i211_desc_unused(ring);
	if (unused < (count + 3))
		return NETDEV_TX_BUSY;

	first = ring->tx_buffer + ring->circ.head;
	first->skb = skb;
	first->bytecount = skb->len;
	first->gso_segs = 1;
	first->protocol = skb->protocol;
	first->flags = 0;

	if (skb_vlan_tag_present(skb)) {
		first->flags |= I211_TX_FLAGS_VLAN;
		first->flags |= skb_vlan_tag_get(skb) << I211_TX_FLAGS_VLAN_SHIFT;
	}

	tso = igb_tso(ring, first, &hdr_len);
	if (!tso)
		tx_csum(ring, first);
	tx_map(ring, first, hdr_len);

	return NETDEV_TX_OK;
}

static void i211_update_stats(struct adapter *adap)
{
	struct rtnl_link_stats64 *net_stats = &adap->stats64;
	int i;
	u64 packets = 0, bytes = 0;
	unsigned int start;
	u32 reg, mpc;

	for (i = 0; i < 2; i++) {
		struct ring *ring = &adap->qvector[i].ring;
		u32 rqdpc = i211_read(adap, RQDPC(i));
		i211_write(adap, RQDPC(i), 0);
		if (rqdpc) {
			ring->rx_stats.drops += rqdpc;
			net_stats->rx_fifo_errors += rqdpc;
		}

		do {
			start = u64_stats_fetch_begin(&ring->rx_syncp);
			packets += ring->rx_stats.packets;
			bytes += ring->rx_stats.bytes;
		} while (u64_stats_fetch_retry(&ring->rx_syncp, start));
	}

	net_stats->rx_packets = packets;
	net_stats->rx_bytes = bytes;

	bytes = packets = 0;
	for (i = 0; i < 2; i++) {
		struct ring *ring = &adap->qvector[i+2].ring;
		do {
			start = u64_stats_fetch_begin(&ring->tx_syncp);
			packets += ring->tx_stats.packets;
			bytes += ring->tx_stats.bytes;
		} while (u64_stats_fetch_retry(&ring->tx_syncp, start));
	}

	net_stats->tx_packets = packets;
	net_stats->tx_bytes = bytes;

	adap->stats.crcerrs += i211_read(adap, CRCERRS);
	i211_read(adap, GORCH); /* clear GORCL */
	adap->stats.mprc += i211_read(adap, MPRC);
	adap->stats.roc += i211_read(adap, ROC);

	mpc = i211_read(adap, MPC);
	adap->stats.mpc += mpc;
	net_stats->rx_fifo_errors += mpc;
	adap->stats.ecol += i211_read(adap, ECOL);
	adap->stats.latecol += i211_read(adap, LATECOL);
	i211_read(adap, GOTCH); /* clear GOTCL */
	adap->stats.ruc += i211_read(adap, RUC);
	adap->stats.colc += i211_read(adap, COLC);

	adap->stats.algnerrc += i211_read(adap, ALGNERRC);
	reg = i211_read(adap, CTRL_EXT);
	if (!(reg & CTRL_EXT_LINK_MODE_MASK)) {
		adap->stats.rxerrc += i211_read(adap, RXERRC);
	}

	net_stats->multicast = adap->stats.mprc;
	net_stats->collisions = adap->stats.colc;

	net_stats->rx_errors = adap->stats.rxerrc +
		adap->stats.crcerrs + adap->stats.algnerrc +
		adap->stats.ruc + adap->stats.roc +
		adap->stats.cexterr;
	net_stats->rx_length_errors = adap->stats.ruc + adap->stats.roc;
	net_stats->rx_crc_errors = adap->stats.crcerrs;
	net_stats->rx_frame_errors = adap->stats.algnerrc;
	net_stats->rx_missed_errors = adap->stats.mpc;

	net_stats->tx_errors = adap->stats.ecol + adap->stats.latecol;
	net_stats->tx_aborted_errors = adap->stats.ecol;
	net_stats->tx_window_errors = adap->stats.latecol;
	net_stats->tx_carrier_errors = adap->stats.tncrs;
}

static void i211_get_stats64(struct net_device *netdev, struct rtnl_link_stats64 *stats)
{
	struct adapter *adap = netdev_priv(netdev);

	spin_lock(&adap->stats64_lock);
	i211_update_stats(adap);
	memcpy(stats, &adap->stats64, sizeof(*stats));
	spin_unlock(&adap->stats64_lock);
}

static int i211_ioctl(struct net_device *ndev, struct ifreq *ifr, int cmd)
{
	struct adapter *adap = netdev_priv(ndev);
	struct mii_ioctl_data *mii = if_mii(ifr);

	switch(cmd) {
	case SIOCGMIIPHY:
		mii->phy_id = 0;
		return 0;
	case SIOCGMIIREG:
		if (read_phy_reg(adap, mii->reg_num, &mii->val_out))
			return -EIO;
		return 0;
	case SIOCSMIIREG:
		if (write_phy_reg(adap, mii->reg_num, mii->val_in))
			return -EIO;
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static const struct net_device_ops i211_netdev_ops = {
	.ndo_open	= i211_open,
	.ndo_stop	= i211_close,
	.ndo_start_xmit	= i211_xmit_frame,
	.ndo_get_stats64 = i211_get_stats64,
	.ndo_eth_ioctl	= i211_ioctl,
};

static void i211_watchdog_task(struct work_struct *work)
{
	bool link_active;
	struct adapter *adap = container_of(work, struct adapter, watchdog_task);
	struct net_device *netdev = adap->netdev;
	u32 status;

	status = i211_read(adap, STATUS);
	link_active = !!(status & STATUS_LU);

	if (link_active) {
		if (!netif_carrier_ok(netdev)) {
			u32 ctrl;
			get_speed_and_duplex(adap, &adap->link_speed, &adap->link_duplex);
			ctrl = i211_read(adap, CTRL);
			netdev_info(netdev,
				"NIC Link is UP %dMbps Speed %s Duplex, Flow Control: %s\n",
				adap->link_speed,
				adap->link_duplex == FULL_DUPLEX ? "Full" : "Half",
				(ctrl & CTRL_RFCE) && (ctrl & CTRL_TFCE) ? "TX/RX" :
				(ctrl & CTRL_RFCE) ? "RX" :
				(ctrl & CTRL_TFCE) ? "TX" : "None");
		}
	} else {
		if (netif_carrier_ok(netdev)) {
			adap->link_speed = 0;
			adap->link_duplex = 0;

			netdev_info(netdev, "i211: %s NIC Link is Down\n", adap->netdev->name);
		}
	}

	phy_mac_interrupt(adap->phy_dev);

	spin_lock(&adap->stats64_lock);
	i211_update_stats(adap);
	spin_unlock(&adap->stats64_lock);

	i211_write(adap, EICS, 0x1e);
}

static void i211_get_drvinfo(struct net_device *netdev, struct ethtool_drvinfo *drv)
{
	strscpy(drv->driver, "i211", sizeof(drv->driver));
}

static const char i211_gstrings_stats[][ETH_GSTRING_LEN] = {
	"rx_packets", "rx_bytes", "rx_errors", "rx_fifo_errors",
	"tx_packets", "tx_bytes", "tx_errors", "tx_carrier_errors",
	"multicast", "collisions"
};

static int i211_get_sset_count(struct net_device *ndev, int sset)
{
	if (sset == ETH_SS_STATS)
		return ARRAY_SIZE(i211_gstrings_stats);

	return -EOPNOTSUPP;
}

static void i211_get_strings(struct net_device *ndev, u32 sset, u8 *data)
{
	if (sset == ETH_SS_STATS)
		memcpy(data, i211_gstrings_stats, sizeof(i211_gstrings_stats));
}

static void i211_get_stats(struct net_device *ndev, struct ethtool_stats *stats, u64 *data)
{
	struct adapter *adap = netdev_priv(ndev);
	const struct rtnl_link_stats64 *net_stats = &adap->stats64;
	int i = 0;

	i211_update_stats(adap);

	data[i++] = net_stats->rx_packets;
	data[i++] = net_stats->rx_bytes;
	data[i++] = net_stats->rx_errors;
	data[i++] = net_stats->rx_fifo_errors;

	data[i++] = net_stats->tx_packets;
	data[i++] = net_stats->tx_bytes;
	data[i++] = net_stats->tx_errors;
	data[i++] = net_stats->tx_carrier_errors;

	data[i++] = net_stats->multicast;
	data[i++] = net_stats->collisions;
}

static const struct ethtool_ops i211_ethtool_ops = {
	.supported_coalesce_params = ETHTOOL_COALESCE_USECS,
	.get_drvinfo	= i211_get_drvinfo,
	.get_link	= ethtool_op_get_link,
	.get_strings	= i211_get_strings,
	.get_ethtool_stats = i211_get_stats,
	.get_sset_count = i211_get_sset_count,
	.get_link_ksettings = phy_ethtool_get_link_ksettings,
};

static int i211_mdio_read(struct mii_bus *bus, int addr, int reg)
{
	u16 val;
	int ret;
	struct adapter *adap = bus->priv;

	if (addr != 0)
		return -ENODEV;

	ret = read_phy_reg(adap, reg, &val);
	if (ret)
		return -EIO;

	return val;
}

static int i211_mdio_write(struct mii_bus *bus, int addr, int reg, u16 val)
{
	int ret;
	struct adapter *adap = bus->priv;

	if (addr != 0)
		return -ENODEV;

	ret = write_phy_reg(adap, reg, val);
	if (ret)
		return -EIO;

	return 0;
}

static void i211_phy_link_change(struct net_device *ndev)
{
	struct adapter *adap = netdev_priv(ndev);

	phy_print_status(adap->phy_dev);
}

static int i211_mdio_setup(struct adapter *adap)
{
	int ret;
	struct mii_bus *mii_bus;
	char phy_name[MII_BUS_ID_SIZE + 3];

	mii_bus = mdiobus_alloc();
	if (!mii_bus)
		return -ENOMEM;

	mii_bus->name = "i211";
	mii_bus->priv = adap;
	mii_bus->parent = &adap->pdev->dev;
	mii_bus->irq[0]= PHY_MAC_INTERRUPT;
	snprintf(mii_bus->id, MII_BUS_ID_SIZE, "i211-%x-%x",
		 pci_domain_nr(adap->pdev->bus), pci_dev_id(adap->pdev));
	mii_bus->read = i211_mdio_read;
	mii_bus->write = i211_mdio_write;

	ret = mdiobus_register(mii_bus);
	if (ret) {
		dev_err(&adap->pdev->dev, "failed to register mdio bus\n");
		goto err_mdiobus_reg;
	}

	adap->mii_bus = mii_bus;
	snprintf(phy_name, sizeof(phy_name), PHY_ID_FMT, mii_bus->id, 0);
	adap->phy_dev = phy_connect(adap->netdev, phy_name, i211_phy_link_change,
				    PHY_INTERFACE_MODE_GMII);
	if (IS_ERR(adap->phy_dev)) {
		ret = PTR_ERR(adap->phy_dev);
		dev_err(&adap->pdev->dev, "failed to connect phy %d\n", ret);
		goto err_get_phy;
	}

	adap->phy_dev->mac_managed_pm = true;
	phy_support_sym_pause(adap->phy_dev);
	phy_attached_info(adap->phy_dev);

	return 0;

err_get_phy:
	mdiobus_unregister(mii_bus);
err_mdiobus_reg:
	mdiobus_free(mii_bus);

	return ret;
}

static void i211_mdio_free(struct adapter *adap)
{
	if (adap->mii_bus) {
		phy_disconnect(adap->phy_dev);
		mdiobus_unregister(adap->mii_bus);
		mdiobus_free(adap->mii_bus);
	}
}

static int i211_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	int ret;
	struct net_device *netdev;
	struct adapter *adapter;

	ret = pci_enable_device_mem(pdev);
	if (ret)
		return ret;

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
	if (ret)
		goto err_dma;

	ret = pci_request_mem_regions(pdev, "i211");
	if (ret)
		goto err_dma;

	pci_set_master(pdev);
	pci_save_state(pdev);

	ret = -ENOMEM;
	netdev = alloc_etherdev_mq(sizeof(struct adapter), 2);
	if (!netdev)
		goto err_alloc_etherdev;

	SET_NETDEV_DEV(netdev, &pdev->dev);
	pci_set_drvdata(pdev, netdev);
	adapter = netdev_priv(netdev);
	adapter->netdev = netdev;
	adapter->pdev = pdev;
	adapter->msg_enable = netif_msg_init(-1, NETIF_MSG_DRV | NETIF_MSG_PROBE | NETIF_MSG_LINK);

	INIT_WORK(&adapter->watchdog_task, i211_watchdog_task);

	set_bit(STATE_DOWN, &adapter->state);

	ret = -EIO;
	adapter->io_addr = pci_iomap(pdev, 0, 0);
	if (!adapter->io_addr)
		goto err_iomap;

	i211_init_interrupt(adapter);

	i211_irq_disable(adapter);
	i211_hw_reset(adapter);
	i211_irq_disable(adapter);

	i211_init_hw(adapter);

	eth_hw_addr_set(netdev, adapter->mac.addr);

	netdev->netdev_ops = &i211_netdev_ops;
	netdev->ethtool_ops = &i211_ethtool_ops;
	netdev->min_mtu = ETH_MIN_MTU;
	netdev->max_mtu = MAX_STD_JUMBO_FRAME_SIZE;
	adapter->max_frame_size = netdev->mtu + 26;
	adapter->min_frame_size = 64;
	adapter->fc.current_mode = fc_default;
	adapter->fc.requested_mode = fc_default;

	netdev->features |= NETIF_F_RXCSUM | NETIF_F_RXHASH | NETIF_F_HW_CSUM |
			    NETIF_F_SG | NETIF_F_TSO | NETIF_F_TSO6;
	netdev->features |= NETIF_F_HW_VLAN_CTAG_RX | NETIF_F_HW_VLAN_CTAG_TX;
	netdev->hw_features |= netdev->features;

	ret = i211_mdio_setup(adapter);
	if (ret)
		goto err_mii_bus;

	ret = register_netdev(netdev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register net device: %d\n", ret);
		goto err_register;
	}

	netif_carrier_off(netdev);

	dev_pm_set_driver_flags(&pdev->dev, DPM_FLAG_NO_DIRECT_COMPLETE);
	pm_runtime_put_noidle(&pdev->dev);

	return 0;

err_register:
	i211_mdio_free(adapter);

	free_all_rx_resources(adapter);
	free_all_tx_resources(adapter);

	release_hw_control(adapter);
err_mii_bus:
	clear_interrupt(adapter);
	pci_iounmap(pdev, adapter->io_addr);
err_iomap:
	free_netdev(netdev);
err_alloc_etherdev:
	pci_release_mem_regions(pdev);
err_dma:
	pci_disable_device(pdev);

	return ret;
}

static void i211_remove(struct pci_dev *pdev)
{
	struct net_device *netdev = pci_get_drvdata(pdev);
	struct adapter *adapter = netdev_priv(netdev);

	pm_runtime_get_noresume(&pdev->dev);

	i211_mdio_free(adapter);

	set_bit(STATE_DOWN, &adapter->state);
	cancel_work_sync(&adapter->watchdog_task);

	release_hw_control(adapter);

	unregister_netdev(netdev);

	clear_interrupt(adapter);
	pci_iounmap(pdev, adapter->io_addr);

	free_netdev(netdev);

	pci_release_mem_regions(pdev);
	pci_disable_device(pdev);
}

static void i211_shutdown(struct pci_dev *pdev)
{
	dev_info(&pdev->dev, "%s\n", __func__);
}

static int i211_suspend(struct device *dev)
{
	dev_info(dev, "%s\n", __func__);
	return 0;
}

static int i211_resume(struct device *dev)
{
	dev_info(dev, "%s\n", __func__);
	return 0;
}

static int i211_runtime_suspend(struct device *dev)
{
	dev_info(dev, "%s\n", __func__);
	return 0;
}

static int i211_runtime_resume(struct device *dev)
{
	dev_info(dev, "%s\n", __func__);
	return 0;
}

static int i211_runtime_idle(struct device *dev)
{
	dev_info(dev, "%s\n", __func__);
	return 0;
}

static const struct dev_pm_ops i211_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(i211_suspend, i211_resume)
	SET_RUNTIME_PM_OPS(i211_runtime_suspend, i211_runtime_resume, i211_runtime_idle)
};

static struct pci_driver i211_driver = {
	.name	  = "i211",
	.id_table = i211_pci_table,
	.probe	  = i211_probe,
	.remove	  = i211_remove,
	.shutdown = i211_shutdown,
#ifdef CONFIG_PM
	.driver	  = {
		.pm	= &i211_pm_ops,
	},
#endif
};

static __init int i211_init(void)
{
	return pci_register_driver(&i211_driver);
}

static __exit void i211_exit(void)
{
	pci_unregister_driver(&i211_driver);
}

module_init(i211_init);
module_exit(i211_exit);
MODULE_LICENSE("GPL v2");
