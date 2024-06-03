/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright(c) 2007 - 2018 Intel Corporation. */

#ifndef __REGS_H_
#define __REGS_H_

#define CTRL     0x00000  /* Device Control - RW */
#define STATUS   0x00008  /* Device Status - RO */
#define EECD     0x00010  /* EEPROM/Flash Control - RW */
#define EERD     0x00014  /* EEPROM Read - RW */
#define CTRL_EXT 0x00018  /* Extended Device Control - RW */
#define MDIC     0x00020  /* MDI Control - RW */
#define MDICNFG  0x00E04  /* MDI Config - RW */
#define SCTL     0x00024  /* SerDes Control - RW */
#define FCAL     0x00028  /* Flow Control Address Low - RW */
#define FCAH     0x0002C  /* Flow Control Address High -RW */
#define FCT      0x00030  /* Flow Control Type - RW */
#define CONNSW   0x00034  /* Copper/Fiber switch control - RW */
#define VET      0x00038  /* VLAN Ether Type - RW */
#define TSSDP    0x0003C  /* Time Sync SDP Configuration Register - RW */
#define ICR      0x000C0  /* Interrupt Cause Read - R/clr */
#define ITR      0x000C4  /* Interrupt Throttling Rate - RW */
#define ICS      0x000C8  /* Interrupt Cause Set - WO */
#define IMS      0x000D0  /* Interrupt Mask Set - RW */
#define IMC      0x000D8  /* Interrupt Mask Clear - WO */
#define IAM      0x000E0  /* Interrupt Acknowledge Auto Mask */
#define RCTL     0x00100  /* RX Control - RW */
#define FCTTV    0x00170  /* Flow Control Transmit Timer Value - RW */
#define TXCW     0x00178  /* TX Configuration Word - RW */
#define EICR     0x01580  /* Ext. Interrupt Cause Read - R/clr */
#define EITR(_n) (0x01680 + (0x4 * (_n)))
#define EICS     0x01520  /* Ext. Interrupt Cause Set - W0 */
#define EIMS     0x01524  /* Ext. Interrupt Mask Set/Read - RW */
#define EIMC     0x01528  /* Ext. Interrupt Mask Clear - WO */
#define EIAC     0x0152C  /* Ext. Interrupt Auto Clear - RW */
#define EIAM     0x01530  /* Ext. Interrupt Ack Auto Clear Mask - RW */
#define GPIE     0x01514  /* General Purpose Interrupt Enable - RW */
#define IVAR0    0x01700  /* Interrupt Vector Allocation (array) - RW */
#define IVAR_MISC 0x01740 /* IVAR for "other" causes - RW */
#define TCTL     0x00400  /* TX Control - RW */
#define TCTL_EXT 0x00404  /* Extended TX Control - RW */
#define TIPG     0x00410  /* TX Inter-packet gap -RW */
#define AIT      0x00458  /* Adaptive Interframe Spacing Throttle - RW */
#define LEDCTL   0x00E00  /* LED Control - RW */
#define LEDMUX   0x08130  /* LED MUX Control */
#define PBA      0x01000  /* Packet Buffer Allocation - RW */
#define PBS      0x01008  /* Packet Buffer Size */
#define EEMNGCTL 0x01010  /* MNG EEprom Control */
#define EEMNGCTL_I210 0x12030  /* MNG EEprom Control */
#define EEARBC_I210 0x12024  /* EEPROM Auto Read Bus Control */
#define EEWR     0x0102C  /* EEPROM Write Register - RW */
#define I2CCMD   0x01028  /* SFPI2C Command Register - RW */
#define FRTIMER  0x01048  /* Free Running Timer - RW */
#define TCPTIMER 0x0104C  /* TCP Timer - RW */
#define FCRTL    0x02160  /* Flow Control Receive Threshold Low - RW */
#define FCRTH    0x02168  /* Flow Control Receive Threshold High - RW */
#define FCRTV    0x02460  /* Flow Control Refresh Timer Value - RW */
#define I2CPARAMS        0x0102C /* SFPI2C Parameters Register - RW */
#define I2CBB_EN      0x00000100  /* I2C - Bit Bang Enable */
#define I2C_CLK_OUT   0x00000200  /* I2C- Clock */
#define I2C_DATA_OUT  0x00000400  /* I2C- Data Out */
#define I2C_DATA_OE_N 0x00000800  /* I2C- Data Output Enable */
#define I2C_DATA_IN   0x00001000  /* I2C- Data In */
#define I2C_CLK_OE_N  0x00002000  /* I2C- Clock Output Enable */
#define I2C_CLK_IN    0x00004000  /* I2C- Clock In */
#define MPHY_ADDR_CTRL	0x0024 /* GbE MPHY Address Control */
#define MPHY_DATA		0x0E10 /* GBE MPHY Data */
#define MPHY_STAT		0x0E0C /* GBE MPHY Statistics */

/* IEEE 1588 TIMESYNCH */
#define TSYNCRXCTL 0x0B620 /* Rx Time Sync Control register - RW */
#define TSYNCTXCTL 0x0B614 /* Tx Time Sync Control register - RW */
#define TSYNCRXCFG 0x05F50 /* Time Sync Rx Configuration - RW */
#define RXSTMPL    0x0B624 /* Rx timestamp Low - RO */
#define RXSTMPH    0x0B628 /* Rx timestamp High - RO */
#define RXSATRL    0x0B62C /* Rx timestamp attribute low - RO */
#define RXSATRH    0x0B630 /* Rx timestamp attribute high - RO */
#define TXSTMPL    0x0B618 /* Tx timestamp value Low - RO */
#define TXSTMPH    0x0B61C /* Tx timestamp value High - RO */
#define SYSTIML    0x0B600 /* System time register Low - RO */
#define SYSTIMH    0x0B604 /* System time register High - RO */
#define TIMINCA    0x0B608 /* Increment attributes register - RW */
#define TSAUXC     0x0B640 /* Timesync Auxiliary Control register */
#define TRGTTIML0  0x0B644 /* Target Time Register 0 Low  - RW */
#define TRGTTIMH0  0x0B648 /* Target Time Register 0 High - RW */
#define TRGTTIML1  0x0B64C /* Target Time Register 1 Low  - RW */
#define TRGTTIMH1  0x0B650 /* Target Time Register 1 High - RW */
#define FREQOUT0   0x0B654 /* Frequency Out 0 Control Register - RW */
#define FREQOUT1   0x0B658 /* Frequency Out 1 Control Register - RW */
#define AUXSTMPL0  0x0B65C /* Auxiliary Time Stamp 0 Register Low  - RO */
#define AUXSTMPH0  0x0B660 /* Auxiliary Time Stamp 0 Register High - RO */
#define AUXSTMPL1  0x0B664 /* Auxiliary Time Stamp 1 Register Low  - RO */
#define AUXSTMPH1  0x0B668 /* Auxiliary Time Stamp 1 Register High - RO */
#define SYSTIMR    0x0B6F8 /* System time register Residue */
#define TSICR      0x0B66C /* Interrupt Cause Register */
#define TSIM       0x0B674 /* Interrupt Mask Register */

/* Filtering Registers */
#define SAQF(_n) (0x5980 + 4 * (_n))
#define DAQF(_n) (0x59A0 + 4 * (_n))
#define SPQF(_n) (0x59C0 + 4 * (_n))
#define FTQF(_n) (0x59E0 + 4 * (_n))
#define SAQF0 E1000_SAQF(0)
#define DAQF0 E1000_DAQF(0)
#define SPQF0 E1000_SPQF(0)
#define FTQF0 E1000_FTQF(0)
#define SYNQF(_n) (0x055FC + (4 * (_n))) /* SYN Packet Queue Fltr */
#define ETQF(_n)  (0x05CB0 + (4 * (_n))) /* EType Queue Fltr */

#define RQDPC(_n) (0x0C030 + ((_n) * 0x40))

/* DMA Coalescing registers */
#define DMACR	0x02508 /* Control Register */
#define DMCTXTH	0x03550 /* Transmit Threshold */
#define DMCTLX	0x02514 /* Time to Lx Request */
#define DMCRTRH	0x05DD0 /* Receive Packet Rate Threshold */
#define DMCCNT	0x05DD4 /* Current Rx Count */
#define FCRTC	0x02170 /* Flow Control Rx high watermark */

/* TX Rate Limit Registers */
#define RTTDQSEL	0x3604 /* Tx Desc Plane Queue Select - WO */
#define RTTBCNRM	0x3690 /* Tx BCN Rate-scheduler MMW */
#define RTTBCNRC	0x36B0 /* Tx BCN Rate-Scheduler Config - WO */

/* Split and Replication RX Control - RW */
#define RXPBS	0x02404 /* Rx Packet Buffer Size - RW */

/* Thermal sensor configuration and status registers */
#define THMJT	0x08100 /* Junction Temperature */
#define THLOWTC	0x08104 /* Low Threshold Control */
#define THMIDTC	0x08108 /* Mid Threshold Control */
#define THHIGHTC	0x0810C /* High Threshold Control */
#define THSTAT	0x08110 /* Thermal Sensor Status */

/* Convenience macros
 *
 * Note: "_n" is the queue number of the register to be written to.
 *
 * Example usage:
 * _RDBAL_REG(current_rx_queue)
 */
#define RDBAL(_n)   ((_n) < 4 ? (0x02800 + ((_n) * 0x100)) \
	       		    : (0x0C000 + ((_n) * 0x40)))
#define RDBAH(_n)   ((_n) < 4 ? (0x02804 + ((_n) * 0x100)) \
	       		    : (0x0C004 + ((_n) * 0x40)))
#define RDLEN(_n)   ((_n) < 4 ? (0x02808 + ((_n) * 0x100)) \
	       		    : (0x0C008 + ((_n) * 0x40)))
#define SRRCTL(_n)  ((_n) < 4 ? (0x0280C + ((_n) * 0x100)) \
	       		    : (0x0C00C + ((_n) * 0x40)))
#define RDH(_n)     ((_n) < 4 ? (0x02810 + ((_n) * 0x100)) \
	       		    : (0x0C010 + ((_n) * 0x40)))
#define RDT(_n)     ((_n) < 4 ? (0x02818 + ((_n) * 0x100)) \
	       		    : (0x0C018 + ((_n) * 0x40)))
#define RXDCTL(_n)  ((_n) < 4 ? (0x02828 + ((_n) * 0x100)) \
	       		    : (0x0C028 + ((_n) * 0x40)))
#define TDBAL(_n)   ((_n) < 4 ? (0x03800 + ((_n) * 0x100)) \
	       		    : (0x0E000 + ((_n) * 0x40)))
#define TDBAH(_n)   ((_n) < 4 ? (0x03804 + ((_n) * 0x100)) \
	       		    : (0x0E004 + ((_n) * 0x40)))
#define TDLEN(_n)   ((_n) < 4 ? (0x03808 + ((_n) * 0x100)) \
	       		    : (0x0E008 + ((_n) * 0x40)))
#define TDH(_n)     ((_n) < 4 ? (0x03810 + ((_n) * 0x100)) \
	       		    : (0x0E010 + ((_n) * 0x40)))
#define TDT(_n)     ((_n) < 4 ? (0x03818 + ((_n) * 0x100)) \
	       		    : (0x0E018 + ((_n) * 0x40)))
#define TXDCTL(_n)  ((_n) < 4 ? (0x03828 + ((_n) * 0x100)) \
	       		    : (0x0E028 + ((_n) * 0x40)))
#define RXCTL(_n)	  ((_n) < 4 ? (0x02814 + ((_n) * 0x100)) : \
	       		      (0x0C014 + ((_n) * 0x40)))
#define DCA_RXCTRL(_n)	E1000_RXCTL(_n)
#define TXCTL(_n)   ((_n) < 4 ? (0x03814 + ((_n) * 0x100)) : \
	       		      (0x0E014 + ((_n) * 0x40)))
#define DCA_TXCTRL(_n) E1000_TXCTL(_n)
#define TDWBAL(_n)  ((_n) < 4 ? (0x03838 + ((_n) * 0x100)) \
	       		    : (0x0E038 + ((_n) * 0x40)))
#define TDWBAH(_n)  ((_n) < 4 ? (0x0383C + ((_n) * 0x100)) \
				    : (0x0E03C + ((_n) * 0x40)))

#define RXPBS	0x02404  /* Rx Packet Buffer Size - RW */
#define TXPBS	0x03404  /* Tx Packet Buffer Size - RW */

#define TDFH     0x03410  /* TX Data FIFO Head - RW */
#define TDFT     0x03418  /* TX Data FIFO Tail - RW */
#define TDFHS    0x03420  /* TX Data FIFO Head Saved - RW */
#define TDFPC    0x03430  /* TX Data FIFO Packet Count - RW */
#define DTXCTL   0x03590  /* DMA TX Control - RW */
#define CRCERRS  0x04000  /* CRC Error Count - R/clr */
#define ALGNERRC 0x04004  /* Alignment Error Count - R/clr */
#define SYMERRS  0x04008  /* Symbol Error Count - R/clr */
#define RXERRC   0x0400C  /* Receive Error Count - R/clr */
#define MPC      0x04010  /* Missed Packet Count - R/clr */
#define SCC      0x04014  /* Single Collision Count - R/clr */
#define ECOL     0x04018  /* Excessive Collision Count - R/clr */
#define MCC      0x0401C  /* Multiple Collision Count - R/clr */
#define LATECOL  0x04020  /* Late Collision Count - R/clr */
#define COLC     0x04028  /* Collision Count - R/clr */
#define DC       0x04030  /* Defer Count - R/clr */
#define TNCRS    0x04034  /* TX-No CRS - R/clr */
#define SEC      0x04038  /* Sequence Error Count - R/clr */
#define CEXTERR  0x0403C  /* Carrier Extension Error Count - R/clr */
#define RLEC     0x04040  /* Receive Length Error Count - R/clr */
#define XONRXC   0x04048  /* XON RX Count - R/clr */
#define XONTXC   0x0404C  /* XON TX Count - R/clr */
#define XOFFRXC  0x04050  /* XOFF RX Count - R/clr */
#define XOFFTXC  0x04054  /* XOFF TX Count - R/clr */
#define FCRUC    0x04058  /* Flow Control RX Unsupported Count- R/clr */
#define PRC64    0x0405C  /* Packets RX (64 bytes) - R/clr */
#define PRC127   0x04060  /* Packets RX (65-127 bytes) - R/clr */
#define PRC255   0x04064  /* Packets RX (128-255 bytes) - R/clr */
#define PRC511   0x04068  /* Packets RX (255-511 bytes) - R/clr */
#define PRC1023  0x0406C  /* Packets RX (512-1023 bytes) - R/clr */
#define PRC1522  0x04070  /* Packets RX (1024-1522 bytes) - R/clr */
#define GPRC     0x04074  /* Good Packets RX Count - R/clr */
#define BPRC     0x04078  /* Broadcast Packets RX Count - R/clr */
#define MPRC     0x0407C  /* Multicast Packets RX Count - R/clr */
#define GPTC     0x04080  /* Good Packets TX Count - R/clr */
#define GORCL    0x04088  /* Good Octets RX Count Low - R/clr */
#define GORCH    0x0408C  /* Good Octets RX Count High - R/clr */
#define GOTCL    0x04090  /* Good Octets TX Count Low - R/clr */
#define GOTCH    0x04094  /* Good Octets TX Count High - R/clr */
#define RNBC     0x040A0  /* RX No Buffers Count - R/clr */
#define RUC      0x040A4  /* RX Undersize Count - R/clr */
#define RFC      0x040A8  /* RX Fragment Count - R/clr */
#define ROC      0x040AC  /* RX Oversize Count - R/clr */
#define RJC      0x040B0  /* RX Jabber Count - R/clr */
#define MGTPRC   0x040B4  /* Management Packets RX Count - R/clr */
#define MGTPDC   0x040B8  /* Management Packets Dropped Count - R/clr */
#define MGTPTC   0x040BC  /* Management Packets TX Count - R/clr */
#define TORL     0x040C0  /* Total Octets RX Low - R/clr */
#define TORH     0x040C4  /* Total Octets RX High - R/clr */
#define TOTL     0x040C8  /* Total Octets TX Low - R/clr */
#define TOTH     0x040CC  /* Total Octets TX High - R/clr */
#define TPR      0x040D0  /* Total Packets RX - R/clr */
#define TPT      0x040D4  /* Total Packets TX - R/clr */
#define PTC64    0x040D8  /* Packets TX (64 bytes) - R/clr */
#define PTC127   0x040DC  /* Packets TX (65-127 bytes) - R/clr */
#define PTC255   0x040E0  /* Packets TX (128-255 bytes) - R/clr */
#define PTC511   0x040E4  /* Packets TX (256-511 bytes) - R/clr */
#define PTC1023  0x040E8  /* Packets TX (512-1023 bytes) - R/clr */
#define PTC1522  0x040EC  /* Packets TX (1024-1522 Bytes) - R/clr */
#define MPTC     0x040F0  /* Multicast Packets TX Count - R/clr */
#define BPTC     0x040F4  /* Broadcast Packets TX Count - R/clr */
#define TSCTC    0x040F8  /* TCP Segmentation Context TX - R/clr */
#define TSCTFC   0x040FC  /* TCP Segmentation Context TX Fail - R/clr */
#define IAC      0x04100  /* Interrupt Assertion Count */
/* Interupt Cause Rx Packet Timer Expire Count */
#define ICRXPTC  0x04104
/* Interupt Cause Rx Absolute Timer Expire Count */
#define ICRXATC  0x04108
/* Interupt Cause Tx Packet Timer Expire Count */
#define ICTXPTC  0x0410C
/* Interupt Cause Tx Absolute Timer Expire Count */
#define ICTXATC  0x04110
/* Interupt Cause Tx Queue Empty Count */
#define ICTXQEC  0x04118
/* Interupt Cause Tx Queue Minimum Threshold Count */
#define ICTXQMTC 0x0411C
/* Interupt Cause Rx Descriptor Minimum Threshold Count */
#define ICRXDMTC 0x04120
#define ICRXOC   0x04124  /* Interrupt Cause Receiver Overrun Count */
#define PCS_CFG0    0x04200  /* PCS Configuration 0 - RW */
#define PCS_LCTL    0x04208  /* PCS Link Control - RW */
#define PCS_LSTAT   0x0420C  /* PCS Link Status - RO */
#define CBTMPC      0x0402C  /* Circuit Breaker TX Packet Count */
#define HTDPMC      0x0403C  /* Host Transmit Discarded Packets */
#define CBRMPC      0x040FC  /* Circuit Breaker RX Packet Count */
#define RPTHC       0x04104  /* Rx Packets To Host */
#define HGPTC       0x04118  /* Host Good Packets TX Count */
#define HTCBDPC     0x04124  /* Host TX Circuit Breaker Dropped Count */
#define HGORCL      0x04128  /* Host Good Octets Received Count Low */
#define HGORCH      0x0412C  /* Host Good Octets Received Count High */
#define HGOTCL      0x04130  /* Host Good Octets Transmit Count Low */
#define HGOTCH      0x04134  /* Host Good Octets Transmit Count High */
#define LENERRS     0x04138  /* Length Errors Count */
#define SCVPC       0x04228  /* SerDes/SGMII Code Violation Pkt Count */
#define PCS_ANADV   0x04218  /* AN advertisement - RW */
#define PCS_LPAB    0x0421C  /* Link Partner Ability - RW */
#define PCS_NPTX    0x04220  /* AN Next Page Transmit - RW */
#define PCS_LPABNP  0x04224  /* Link Partner Ability Next Page - RW */
#define RXCSUM   0x05000  /* RX Checksum Control - RW */
#define RLPML    0x05004  /* RX Long Packet Max Length */
#define RFCTL    0x05008  /* Receive Filter Control*/
#define MTA      0x05200  /* Multicast Table Array - RW Array */
#define RA       0x05400  /* Receive Address - RW Array */
#define RA2      0x054E0  /* 2nd half of Rx address array - RW Array */
#define PSRTYPE(_i)       (0x05480 + ((_i) * 4))
#define RAL(_i)  (((_i) <= 15) ? (0x05400 + ((_i) * 8)) : \
	       			(0x054E0 + ((_i - 16) * 8)))
#define RAH(_i)  (((_i) <= 15) ? (0x05404 + ((_i) * 8)) : \
	       			(0x054E4 + ((_i - 16) * 8)))
#define VLAPQF	0x055B0  /* VLAN Priority Queue Filter VLAPQF */
#define IP4AT_REG(_i)     (0x05840 + ((_i) * 8))
#define IP6AT_REG(_i)     (0x05880 + ((_i) * 4))
#define WUPM_REG(_i)      (0x05A00 + ((_i) * 4))
#define FFMT_REG(_i)      (0x09000 + ((_i) * 8))
#define FFVT_REG(_i)      (0x09800 + ((_i) * 8))
#define FFLT_REG(_i)      (0x05F00 + ((_i) * 8))
#define VFTA     0x05600  /* VLAN Filter Table Array - RW Array */
#define VT_CTL   0x0581C  /* VMDq Control - RW */
#define WUC      0x05800  /* Wakeup Control - RW */
#define WUFC     0x05808  /* Wakeup Filter Control - RW */
#define WUS      0x05810  /* Wakeup Status - R/W1C */
#define MANC     0x05820  /* Management Control - RW */
#define IPAV     0x05838  /* IP Address Valid - RW */
#define WUPL     0x05900  /* Wakeup Packet Length - RW */

#define SW_FW_SYNC  0x05B5C /* Software-Firmware Synchronization - RW */
#define CCMCTL      0x05B48 /* CCM Control Register */
#define GIOCTL      0x05B44 /* GIO Analog Control Register */
#define SCCTL       0x05B4C /* PCIc PLL Configuration Register */
#define GCR         0x05B00 /* PCI-Ex Control */
#define FACTPS    0x05B30 /* Function Active and Power State to MNG */
#define SWSM      0x05B50 /* SW Semaphore */
#define FWSM      0x05B54 /* FW Semaphore */
#define DCA_CTRL  0x05B74 /* DCA Control - RW */

/* RSS registers */
#define MRQC      0x05818 /* Multiple Receive Control - RW */
#define IMIR(_i)      (0x05A80 + ((_i) * 4))  /* Immediate Interrupt */
#define IMIREXT(_i)   (0x05AA0 + ((_i) * 4))  /* Immediate Interrupt Ext*/
#define IMIRVP    0x05AC0 /* Immediate Interrupt RX VLAN Priority - RW */
/* MSI-X Allocation Register (_i) - RW */
#define MSIXBM(_i)    (0x01600 + ((_i) * 4))
/* Redirection Table - RW Array */
#define RETA(_i)  (0x05C00 + ((_i) * 4))
#define RSSRK(_i) (0x05C80 + ((_i) * 4)) /* RSS Random Key - RW Array */

/* VT Registers */
#define MBVFICR   0x00C80 /* Mailbox VF Cause - RWC */
#define MBVFIMR   0x00C84 /* Mailbox VF int Mask - RW */
#define VFLRE     0x00C88 /* VF Register Events - RWC */
#define VFRE      0x00C8C /* VF Receive Enables */
#define VFTE      0x00C90 /* VF Transmit Enables */
#define QDE       0x02408 /* Queue Drop Enable - RW */
#define DTXSWC    0x03500 /* DMA Tx Switch Control - RW */
#define WVBR      0x03554 /* VM Wrong Behavior - RWS */
#define RPLOLR    0x05AF0 /* Replication Offload - RW */
#define UTA       0x0A000 /* Unicast Table Array - RW */
#define IOVTCL    0x05BBC /* IOV Control Register */
#define TXSWC     0x05ACC /* Tx Switch Control */
#define LVMMC	0x03548 /* Last VM Misbehavior cause */
/* These act per VF so an array friendly macro is used */
#define P2VMAILBOX(_n)   (0x00C00 + (4 * (_n)))
#define VMBMEM(_n)       (0x00800 + (64 * (_n)))
#define VMOLR(_n)        (0x05AD0 + (4 * (_n)))
#define DVMOLR(_n)       (0x0C038 + (64 * (_n)))
#define VLVF(_n)         (0x05D00 + (4 * (_n))) /* VLAN VM Filter */
#define VMVIR(_n)        (0x03700 + (4 * (_n)))

#define wrfl(adap) ((void)i211_read(adap, STATUS))

/* DMA Coalescing registers */
#define _PCIEMISC	0x05BB8 /* PCIE misc config register */

/* Energy Efficient Ethernet "EEE" register */
#define IPCNFG	0x0E38 /* Internal PHY Configuration */
#define EEER	0x0E30 /* Energy Efficient Ethernet */
#define EEE_SU	0X0E34 /* EEE Setup */
#define EMIADD	0x10   /* Extended Memory Indirect Address */
#define EMIDATA	0x11   /* Extended Memory Indirect Data */
#define MMDAC	13     /* MMD Access Control */
#define MMDAAD	14     /* MMD Access Address/Data */

/* Thermal Sensor Register */
#define THSTAT	0x08110 /* Thermal Sensor Status */

/* OS2BMC Registers */
#define B2OSPC	0x08FE0 /* BMC2OS packets sent by BMC */
#define B2OGPRC	0x04158 /* BMC2OS packets received by host */
#define O2BGPTC	0x08FE4 /* OS2BMC packets received by BMC */
#define O2BSPC	0x0415C /* OS2BMC packets transmitted by host */

#define SRWR		0x12018  /* Shadow Ram Write Register - RW */
#define I210_FLMNGCTL	0x12038
#define I210_FLMNGDATA	0x1203C
#define I210_FLMNGCNT	0x12040

#define I210_FLSWCTL	0x12048
#define I210_FLSWDATA	0x1204C
#define I210_FLSWCNT	0x12050

#define I210_FLA		0x1201C

#define I210_DTXMXPKTSZ	0x355C

#define I210_TXDCTL(_n)	(0x0E028 + ((_n) * 0x40))

#define I210_TQAVCTRL	0x3570
#define I210_TQAVCC(_n)	(0x3004 + ((_n) * 0x40))
#define I210_TQAVHC(_n)	(0x300C + ((_n) * 0x40))

#define I210_RR2DCDELAY	0x5BF4

#define INVM_DATA_REG(_n)	(0x12120 + 4*(_n))
#define INVM_SIZE		64 /* Number of INVM Data Registers */

#define REMOVED(h) unlikely(!(h))

#endif
