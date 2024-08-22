/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2022 Soren Schmidt <sos@deepcore.dk>
 * Copyright (c) 2022 Jared McNeill <jmcneill@invisible.ca>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $Id: eqos_var.h 1009 2022-11-15 20:17:35Z sos $
 */

/*
 * DesignWare Ethernet Quality-of-Service controller
 */

#ifndef _EQOS_VAR_H
#define	_EQOS_VAR_H

#include <dev/eqos/if_eqos_reg.h>

#define	EQOS_DMA_DESC_COUNT	256

#define	EQOS_RES_MEM		0
#define	EQOS_RES_IRQ0		1
#define	EQOS_RES_COUNT		2

#define	EQOS_INTR_FLAGS		(INTR_TYPE_NET | INTR_MPSAFE)

struct eqos_dma_desc {
	uint32_t	des0;
	uint32_t	des1;
	uint32_t	des2;
	uint32_t	des3;
} __packed;

struct eqos_bufmap {
	bus_dmamap_t		map;
	struct mbuf		*mbuf;
};

struct eqos_ring {
	bus_dma_tag_t		desc_tag;
	bus_dmamap_t		desc_map;
	struct eqos_dma_desc	*desc_ring;
	bus_addr_t		desc_ring_paddr;

	bus_dma_tag_t		buf_tag;
	struct eqos_bufmap	buf_map[EQOS_DMA_DESC_COUNT];

	u_int			head;
	u_int			tail;
};

struct eqos_softc {
	device_t		dev;
	struct resource 	*res[EQOS_RES_COUNT];
	void			*irq_handle;
#ifdef FDT
	struct syscon		*grf, *php_grf;
	int			grf_offset;
	int			phy_mode;
	struct eqos_ops		*ops;
	clk_t			*clks;
	clk_t			clk_stmmaceth;
	bool			clock_in;
	uint8_t			idx;
	uint8_t			axi_rd_osr_lmt, axi_wr_osr_lmt;
	uint16_t		axi_blen[AXI_BLEN];
#endif
	uint32_t		csr_clock;
	uint32_t		csr_clock_range;
	uint32_t		hw_feature[4];
	bool			link_up;
	int			tx_watchdog;
	uint8_t			force_thresh_dma_mode;
	
	struct ifmedia eqos_ifmedia;
	struct ifnet		*ifp;
	device_t		miibus;
	struct mtx		lock;
	struct callout          callout;
	struct callout          calltxi;	
	struct eqos_ring	tx;
	struct eqos_ring	rx;

	int			phy_attached;
 	int			switch_attached;
};

DECLARE_CLASS(eqos_driver);

#endif
