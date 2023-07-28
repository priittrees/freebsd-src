#ifndef __BCMSWITCH_REG_H__
#define __BCMSWITCH_REG_H__

#define BCMSWITCH_PSPHY			0x1E    /* Pseudo PHY address */
#define   BCMSWITCH_PSPHY_PAGE		0x10    /* Page Number */
#define     BCMSWITCH_PSPHY_PAGE_NUM(page)	(((page) & 0xFF) << 8)
#define     BCMSWITCH_PSPHY_PAGE_EN	(1 << 0)
#define   BCMSWITCH_PSPHY_ADDR		0x11    /* Register Address */
#define     BCMSWITCH_PSPHY_ADDR_REG(addr)	(((addr) & 0xFF) << 8)
#define     BCMSWITCH_PSPHY_ADDR_READ	(1 << 1)
#define     BCMSWITCH_PSPHY_ADDR_WRITE	(1 << 0)
#define   BCMSWITCH_PSPHY_ASB		0x12    /* Access status */
#define     BCMSWITCH_PSPHY_OP_ERR	(1 << 1)
#define     BCMSWITCH_PSPHY_PA_ERR	(1 << 0)
#define   BCMSWITCH_PSPHY_DATA0	 	0x18    /* Access reg bits [15:0]  */
#define   BCMSWITCH_PSPHY_DATA1		0x19    /* Access reg bits [31:16] */
#define   BCMSWITCH_PSPHY_DATA2		0x1a    /* Access reg bits [47:32] */
#define   BCMSWITCH_PSPHY_DATA3		0x1b    /* Access reg bits [63:48] */

#define BCMSWITCH_CTRL_PAGE		0x00 /* Control Registers Page*/
#define   BCMSWITCH_SWITCH_MODE		0x0b /*  */
#define     BCMSWITCH_SW_FWDG_MODE	(1 << 0) /* 1 = Managed Mode */
#define     BCMSWITCH_SW_FWDG_EN		(1 << 1) /* Forwarding Enable */
#define   BCMSWITCH_IMP_OVERRIDE		0x0e
#define     BCMSWITCH_IMP_OVERRIDE_LINK	(1 << 0)
#define     BCMSWITCH_IMP_OVERRIDE_EN	(1 << 7) /* Use the register contents */
#define	  BCMSWITCH_PORT_OVERRIDE(i)	(0x58 * (i))
#define     BCMSWITCH_PORT_OVERRIDE_LINK	(1 << 0)
#define     BCMSWITCH_PORT_OVERRIDE_EN	(1 << 7) /* Use the register contents */

#define	  BCMSWITCH_SW_RST_CTRL		0x79
#define	    BCMSWITCH_SW_RST		(1 << 7)
#define	    BCMSWITCH_SW_RST_EN		(1 << 4)

#define BCMSWITCH_ARLIO_PAGE		0x05 /* ARL/VTBL Access Registers Page*/
#define   BCMSWITCH_ARL_SR		0x50
#define	    BCMSWITCH_ARL_SR_START	(1 << 7)
#define	    BCMSWITCH_ARL_SR_VALID	(1 << 0)
#define   BCMSWITCH_ARL_SR_TBL0		0x60
#define   BCMSWITCH_ARL_SR_TBL1		0x70
#define   BCMSWITCH_ARL_SR_DATA0		0x68
#define   BCMSWITCH_ARL_SR_DATA1		0x78

#define   BCMSWITCH_VT_ACCESS		0x80 
#define     VTA_START_CMD		(1 << 7)
#define     VTA_CMD_WRITE		0
#define     VTA_CMD_READ		1
#define     VTA_CMD_CLEAR		2
#define   BCMSWITCH_VT_INDEX		0x81
#define   BCMSWITCH_VT_ENTRY		0x83
#define     VTE_UNTAG_S		   	9

#define BCMSWITCH_VLAN_PAGE		0x34
#define   BCMSWITCH_VLAN_CTRL0		0x00
#define     VC0_VID_HASH_VID		(1 << 5)
#define     VC0_VID_CHK_EN		(1 << 6) /* Use VID,DA or VID,SA */
#define     VC0_VLAN_EN		   	(1 << 7) /* 802.1Q VLAN Enabled */
#define   BCMSWITCH_VLAN_CTRL1		0x01
#define     VC1_RX_MCST_FWD_EN	    	(1 << 2)
#define     VC1_RX_MCST_UNTAG_EN	(1 << 3)
#define   BCMSWITCH_VLAN_CTRL3		0x03
#define   BCMSWITCH_VLAN_CTRL4		0x05
#define     VC4_ING_VID_CHECK_S	   	6
#define     VC4_ING_VID_CHECK_MASK	(0x3 << VC4_ING_VID_CHECK_S)
#define     VC4_ING_VID_VIO_DROP	1 	/* drop VID violations */
#define   BCMSWITCH_VLAN_CTRL5		0x06
#define     VC5_DROP_VTABLE_MISS	(1 << 3)
#define   BCMSWITCH_VLAN_PORT_DEF_TAG(i)	(0x10 + 2 * (i))

#define BCMSWITCH_PVLAN_PAGE		0x31
#define   BCMSWITCH_PVLAN_PORT_MASK(i)	((i) * 2)

#endif /* __BCMSWITCH_REG_H__ */
