/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2009, Aleksandr Rybalko
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice unmodified, this list of conditions, and the following
 *    disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#ifndef _IF_RTREG_H_
#define	_IF_RTREG_H_

#define	RT_READ(sc, reg)				\
	bus_space_read_4((sc)->bst, (sc)->bsh, reg)

#define	RT_WRITE(sc, reg, val)				\
	bus_space_write_4((sc)->bst, (sc)->bsh, reg, val)

#define	RT_GDMA1_BASE		0x500
#define RT_GDMA2_BASE		RT_GDMA1_BASE + 0x1000

#define RT_GDM_IG_CTRL(gdma)	(gdma + 0x00)
#define	    INSV_EN		(1<<25)
#define	    STAG_EN		(1<<24)
#define	    GDM_ICS_EN		(1<<22)
#define	    GDM_TCS_EN		(1<<21)
#define	    GDM_UCS_EN		(1<<20)
#define	    GDM_DROP_256B	(1<<19)
#define	    GDM_STRPCRC		(1<<16)
#define	    GDM_UFRC_P_SHIFT	12
#define	    GDM_BFRC_P_SHIFT	8
#define	    GDM_MFRC_P_SHIFT	4
#define	    GDM_OFRC_P_SHIFT	0
#define	    GDM_XFRC_P_MASK	0x07
#define	    GDM_DST_PORT_CPU	0
#define	    GDM_DST_PORT_GDMA1	1
#define	    GDM_DST_PORT_GDMA2	2
#define     GDM_DST_PORT_PPE	4
#define     GDM_DST_PORT_QDMA	5
#define	    GDM_DST_PORT_DISCARD 7

#define RT_GDM_MAC_LSB(gdma)  (gdma + 0x08)
#define RT_GDM_MAC_MSB(gdma)  (gdma + 0x0c)

#define RT5350_PDMA_BASE 0x0800

#define        RT5350_TX_BASE_PTR(qid)			\
	((qid) * 0x10 + RT5350_PDMA_BASE + 0x000)
#define        RT5350_TX_MAX_CNT(qid)			\
	((qid) * 0x10 + RT5350_PDMA_BASE + 0x004)
#define        RT5350_TX_CTX_IDX(qid)			\
	((qid) * 0x10 + RT5350_PDMA_BASE + 0x008)
#define        RT5350_TX_DTX_IDX(qid)			\
	((qid) * 0x10 + RT5350_PDMA_BASE + 0x00C)

#define        RT5350_RX_BASE_PTR(qid)			\
	((qid) * 0x10 + RT5350_PDMA_BASE + 0x100)
#define        RT5350_RX_MAX_CNT(qid)			\
	((qid) * 0x10 + RT5350_PDMA_BASE + 0x104)
#define        RT5350_RX_CALC_IDX(qid)			\
	((qid) * 0x10 + RT5350_PDMA_BASE + 0x108)
#define        RT5350_RX_DRX_IDX(qid)			\
	((qid) * 0x10 + RT5350_PDMA_BASE + 0x10C)

#define RT5350_PDMA_GLO_CFG	(RT5350_PDMA_BASE + 0x204)
#define	    FE_RX_2B_OFFSET	(1<<31)
#define	    FE_TX_WB_DDONE	(1<<6)
#define	    FE_DMA_BT_SIZE4	(0<<4)
#define	    FE_DMA_BT_SIZE8	(1<<4)
#define	    FE_DMA_BT_SIZE16	(2<<4)
#define	    FE_RX_DMA_BUSY	(1<<3)
#define	    FE_RX_DMA_EN	(1<<2)
#define	    FE_TX_DMA_BUSY	(1<<1)
#define	    FE_TX_DMA_EN	(1<<0)

#define RT5350_PDMA_RST_IDX 	(RT5350_PDMA_BASE + 0x208)
#define	    FE_RST_DRX_IDX0	(1<<16)
#define	    FE_RST_DTX_IDX3	(1<<3)
#define	    FE_RST_DTX_IDX2	(1<<2)
#define	    FE_RST_DTX_IDX1	(1<<1)
#define	    FE_RST_DTX_IDX0	(1<<0)

#define RT5350_DELAY_INT_CFG	(RT5350_PDMA_BASE + 0x20C)
#define	    TXDLY_INT_EN 	(1<<31)
#define	    TXMAX_PINT_SHIFT	24
#define	    TXMAX_PTIME_SHIFT	16
#define	    RXDLY_INT_EN	(1<<15)
#define	    RXMAX_PINT_SHIFT	8
#define	    RXMAX_PTIME_SHIFT	0

#define RT5350_PDMA_INT_STATUS    (RT5350_PDMA_BASE + 0x220)
#define            RT5350_INT_RX_COHERENT      (1<<31)
#define            RT5350_RX_DLY_INT           (1<<30)
#define            RT5350_INT_TX_COHERENT      (1<<29)
#define            RT5350_TX_DLY_INT           (1<<28)
#define            RT5350_INT_RXQ3_DONE	       (1<<19)
#define            RT5350_INT_RXQ2_DONE        (1<<18)
#define            RT5350_INT_RXQ1_DONE	       (1<<17)
#define            RT5350_INT_RXQ0_DONE        (1<<16)
#define            RT5350_INT_TXQ3_DONE        (1<<3)
#define            RT5350_INT_TXQ2_DONE        (1<<2)
#define            RT5350_INT_TXQ1_DONE        (1<<1)
#define            RT5350_INT_TXQ0_DONE        (1<<0)
#define RT5350_PDMA_INT_ENABLE	(RT5350_PDMA_BASE + 0x228)
#define RT5350_PDMA_SCH_CFG0	(RT5350_PDMA_BASE + 0x280)

#define	CNTR_BASE 0x2400
#define	GDMA_RX_GBCNT0		CNTR_BASE + 0x00
#define	GDMA_RX_GPCNT0		CNTR_BASE + 0x08
#define	GDMA_RX_OERCNT0		CNTR_BASE + 0x10
#define	GDMA_RX_FERCNT0		CNTR_BASE + 0x14
#define	GDMA_RX_SHORT_ERCNT0	CNTR_BASE + 0x18
#define	GDMA_RX_LONG_ERCNT0	CNTR_BASE + 0x1C
#define	GDMA_RX_CSUM_ERCNT0	CNTR_BASE + 0x20
#define GDMA_RX_FCCNT		CNTR_BASE + 0x24
#define	GDMA_TX_SKIPCNT0	CNTR_BASE + 0x28
#define	GDMA_TX_COLCNT0		CNTR_BASE + 0x2C
#define	GDMA_TX_GBCNT0		CNTR_BASE + 0x30
#define	GDMA_TX_GPCNT0		CNTR_BASE + 0x38

#define	GE_PORT_BASE 0x10000
#define	MDIO_ACCESS	0x04
#define	    MDIO_CMD_ONGO	(1<<31)
#define	    MDIO_PHYREG_ADDR_MASK	0x3e000000
#define	    MDIO_PHYREG_ADDR_SHIFT	25
#define	    MDIO_PHY_ADDR_MASK	0x01f00000
#define	    MDIO_PHY_ADDR_SHIFT	20
#define	    MDIO_CMD_MASK	0x000c0000
#define	    MDIO_CMD_SHIFT	18
#define	    MDIO_CMD_WRITE	0x1
#define	    MDIO_CMD_READ	0x2
#define	    MDIO_CMD_READ_C45	0x3
#define	    MDIO_ST_MASK	0x30000
#define	    MDIO_ST_SHIFT	16
#define	    MDIO_ST_C45		0x0
#define	    MDIO_ST_C22		0x1
#define	    MDIO_PHY_DATA_MASK	0x0000ffff
#define	    MDIO_PHY_DATA_SHIFT	0 

#endif /* _IF_RTREG_H_ */
