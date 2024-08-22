/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2022 Soren Schmidt <sos@deepcore.dk>
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
 * $Id: eqos_fdt.c 1049 2022-12-03 14:25:46Z sos $
 */

#include "opt_platform.h"
#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/systm.h>
#include <sys/endian.h>
#include <sys/hash.h>
#include <sys/gpio.h>
#include <sys/rman.h>
#include <sys/socket.h>
#include <machine/bus.h>

#include <net/if.h>
#include <net/if_media.h>
#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>
#include <dev/mii/mii_fdt.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/extres/clk/clk.h>
#include <dev/extres/hwreset/hwreset.h>
#include <dev/extres/regulator/regulator.h>
#include <dev/extres/syscon/syscon.h>

#include <dev/eqos/if_eqos_var.h>

#include "if_eqos_if.h"
#include "syscon_if.h"
#include "gpio_if.h"
#include "rk_otp_if.h"

#define RK_MASK_HIGH(val, mask, shift) \
		(((val) << (shift)) | (mask) << ((shift) + 16))

#define RK_BITS(n)	((1 << (n)) | (1 << ((n) + 16)))
#define RK_HI_BITS(n)	((1 << ((n) + 16)))


#define	EQOS_GMAC_PHY_INTF_SEL_RGMII		0x00fc0010
#define	EQOS_GMAC_PHY_INTF_SEL_RMII		0x00fc0040
#define	EQOS_GMAC_RXCLK_DLY_ENABLE		0x00020002
#define	EQOS_GMAC_RXCLK_DLY_DISABLE		0x00020000
#define	EQOS_GMAC_TXCLK_DLY_ENABLE		0x00010001
#define	EQOS_GMAC_TXCLK_DLY_DISABLE		0x00010000
#define	EQOS_GMAC_CLK_RX_DL_CFG(val)		(0x7f000000 | val << 8)
#define	EQOS_GMAC_CLK_TX_DL_CFG(val)		(0x007f0000 | val)

#define	WR4(sc, o, v)		bus_write_4(sc->res[EQOS_RES_MEM], (o), (v))

struct eqos_ops {
	int (*set_to_rgmii)(device_t dev,int tx_delay, int rx_delay);
	int (*set_to_rmii)(device_t dev);
	int (*set_rgmii_speed)(device_t dev, int speed);
	int (*set_rmii_speed)(device_t dev, int speed);
	int (*set_clock_selection)(device_t dev, bool input);	
	bool regs_valid;
	uint32_t regs[];
};

static const char * eqos_clocks[] = {
	"aclk_mac", "pclk_mac", /*"mac_clk_tx",*/ "clk_mac_speed",
};

static const char * eqos_rmii_clocks[] = {
	"mac_clk_rx", "clk_mac_ref", "clk_mac_refout",
};
enum eqos_clk_index {
	EQOS_ACLK_MAC = 0,
	EQOS_PCLK_MAC,
/*	EQOS_MAC_CLK_TX,*/
	EQOS_CLK_MAC_SPEED,
	EQOS_MAC_CLK_RX,
	EQOS_CLK_MAC_REF,
	EQOS_CLK_MAC_REFOUT,
};
#define RK3568_GRF_GMAC0_CON0		0x0380
#define RK3568_GRF_GMAC0_CON1		0x0384
#define RK3568_GRF_GMAC1_CON0		0x0388
#define RK3568_GRF_GMAC1_CON1		0x038c

/* RK3568_GRF_GMAC0_CON1 && RK3568_GRF_GMAC1_CON1 */
#define RK3568_GMAC_PHY_INTF_SEL_RGMII	\
		(RK_BITS(4) | RK_HI_BITS(5) | RK_HI_BITS(6))
#define RK3568_GMAC_PHY_INTF_SEL_RMII	\
		(RK_HI_BITS(4) | RK_HI_BITS(5) | RK_BITS(6))
#define RK3568_GMAC_FLOW_CTRL			RK_BITS(3)
#define RK3568_GMAC_FLOW_CTRL_CLR		RK_HI_BITS(3)
#define RK3568_GMAC_RXCLK_DLY_ENABLE		RK_BITS(1)
#define RK3568_GMAC_RXCLK_DLY_DISABLE		RK_HI_BITS(1)
#define RK3568_GMAC_TXCLK_DLY_ENABLE		RK_BITS(0)
#define RK3568_GMAC_TXCLK_DLY_DISABLE		RK_HI_BITS(0)

/* RK3568_GRF_GMAC0_CON0 && RK3568_GRF_GMAC1_CON0 */
#define RK3568_GMAC_CLK_RX_DL_CFG(val)	RK_MASK_HIGH(val, 0x7F, 8)
#define RK3568_GMAC_CLK_TX_DL_CFG(val)	RK_MASK_HIGH(val, 0x7F, 0)

static int 
rk3568_set_rgmii(device_t dev, int tx_delay, int rx_delay)
{
	struct eqos_softc *sc = device_get_softc(dev);
	uint32_t reg0, reg1;
	
	reg0 = (sc->idx == 1) ? RK3568_GRF_GMAC1_CON0 :
				RK3568_GRF_GMAC0_CON0;
	reg1 = (sc->idx == 1) ? RK3568_GRF_GMAC1_CON1 :
				RK3568_GRF_GMAC0_CON1;
	
	SYSCON_WRITE_4(sc->grf, reg0,
		     RK3568_GMAC_CLK_RX_DL_CFG(rx_delay) |
		     RK3568_GMAC_CLK_TX_DL_CFG(tx_delay));

	SYSCON_WRITE_4(sc->grf, reg1,
		     RK3568_GMAC_PHY_INTF_SEL_RGMII |
		     RK3568_GMAC_RXCLK_DLY_ENABLE |
		     RK3568_GMAC_TXCLK_DLY_ENABLE);
      return 0;
}

static int
rk3568_set_rmii(device_t dev)
{
        struct eqos_softc *sc = device_get_softc(dev);
        uint32_t reg1;
        
        reg1 = (sc->idx == 1) ? RK3568_GRF_GMAC1_CON1 :
                                RK3568_GRF_GMAC0_CON1;
 
        SYSCON_WRITE_4(sc->grf, reg1, RK3568_GMAC_PHY_INTF_SEL_RMII);                                      

        return 0;
}
static int
rk3568_set_speed(device_t dev, int speed)
{
        struct eqos_softc *sc = device_get_softc(dev);
        clk_t clk_mac_speed = sc->clks[EQOS_CLK_MAC_SPEED];
        unsigned long rate;
	int error;
	
	switch (speed) {
	case IFM_10_T:
		rate = 2500000;
		break;
	case IFM_100_TX:
		rate = 25000000;
		break;
	case IFM_1000_T:
        case IFM_1000_SX:
		rate = 125000000;
		break;
	default:
		device_printf(dev, "unknown speed value for GMAC speed=%d", speed);
		return (EINVAL);
	 }
	 error = clk_set_freq(clk_mac_speed, rate, 0);
	 if(error) {
	  device_printf(dev, "can't set %s to %lu, return=%d\n", 
	  	clk_get_name(clk_mac_speed), rate, error);
	  }
	  
	return error;  
}
static const struct eqos_ops rk3568_ops = {
	.set_to_rgmii = rk3568_set_rgmii,
	.set_to_rmii = rk3568_set_rmii,
	.set_rgmii_speed = rk3568_set_speed,
	.set_rmii_speed = rk3568_set_speed,
	.regs_valid = true,
	.regs = {
		0xfe2a0000, /* gmac0 */
		0xfe010000, /* gmac1 */
		0x0, /* sentinel */
	},
};

#define RK3588_GRF_GMAC_CON7			0x031c
#define RK3588_GRF_GMAC_CON8			0x0320
#define RK3588_GRF_GMAC_CON9			0x0324

#define RK3588_GMAC_RXCLK_DLY_ENABLE(id)	RK_BITS(2 * (id) + 3)
#define RK3588_GMAC_RXCLK_DLY_DISABLE(id)	RK_HI_BITS(2 * (id) + 3)
#define RK3588_GMAC_TXCLK_DLY_ENABLE(id)	RK_BITS(2 * (id) + 2)
#define RK3588_GMAC_TXCLK_DLY_DISABLE(id)	RK_HI_BITS(2 * (id) + 2)

#define RK3588_GMAC_CLK_RX_DL_CFG(val)		RK_MASK_HIGH(val, 0xFF, 8)
#define RK3588_GMAC_CLK_TX_DL_CFG(val)		RK_MASK_HIGH(val, 0xFF, 0)

/* php_grf */
#define RK3588_GRF_GMAC_CON0			0x0008
#define RK3588_GRF_CLK_CON1			0x0070

#define RK3588_GMAC_PHY_INTF_SEL_RGMII(id)	\
	(RK_BITS(3 + (id) * 6) | RK_HI_BITS(4 + (id) * 6) | RK_HI_BITS(5 + (id) * 6))
#define RK3588_GMAC_PHY_INTF_SEL_RMII(id)	\
	(RK_HI_BITS(3 + (id) * 6) | RK_HI_BITS(4 + (id) * 6) | RK_BITS(5 + (id) * 6))

#define RK3588_GMAC_CLK_RMII_MODE(id)		RK_BITS(5 * (id))
#define RK3588_GMAC_CLK_RGMII_MODE(id)		RK_HI_BITS(5 * (id))

#define RK3588_GMAC_CLK_SELET_CRU(id)		RK_BITS(5 * (id) + 4)
#define RK3588_GMAC_CLK_SELET_IO(id)		RK_HI_BITS(5 * (id) + 4)

#define RK3588_GMA_CLK_RMII_DIV2(id)		RK_BITS(5 * (id) + 2)
#define RK3588_GMA_CLK_RMII_DIV20(id)		RK_HI_BITS(5 * (id) + 2)

#define RK3588_GMAC_CLK_RGMII_DIV1(id)		\
			(RK_HI_BITS(5 * (id) + 2) | RK_HI_BITS(5 * (id) + 3))
#define RK3588_GMAC_CLK_RGMII_DIV5(id)		\
			(RK_BITS(5 * (id) + 2) | RK_BITS(5 * (id) + 3))
#define RK3588_GMAC_CLK_RGMII_DIV50(id)		\
			(RK_HI_BITS(5 * (id) + 2) | RK_BITS(5 * (id) + 3))

#define RK3588_GMAC_CLK_RMII_GATE(id)		RK_BITS(5 * (id) + 1)
#define RK3588_GMAC_CLK_RMII_NOGATE(id)		RK_HI_BITS(5 * (id) + 1)

static int
rk3588_set_rgmii(device_t dev, int tx_delay, int rx_delay)
{
	struct eqos_softc *sc = device_get_softc(dev);
	uint32_t offset_con, id = sc->idx;

	if (!(sc->php_grf)) {
		device_printf(dev, "Missing rockchip,grf or rockchip,php_grf property\n");
		return (ENXIO);
	}

	offset_con = id == 1 ? RK3588_GRF_GMAC_CON9 :
				    RK3588_GRF_GMAC_CON8;

	SYSCON_WRITE_4(sc->php_grf, RK3588_GRF_GMAC_CON0,
		     RK3588_GMAC_PHY_INTF_SEL_RGMII(id));

	SYSCON_WRITE_4(sc->php_grf, RK3588_GRF_CLK_CON1,
		     RK3588_GMAC_CLK_RGMII_MODE(id));

	SYSCON_WRITE_4(sc->grf, RK3588_GRF_GMAC_CON7,
		     RK3588_GMAC_RXCLK_DLY_ENABLE(id) |
		     RK3588_GMAC_TXCLK_DLY_ENABLE(id));

	SYSCON_WRITE_4(sc->grf, offset_con,
		     RK3588_GMAC_CLK_RX_DL_CFG(rx_delay) |
		     RK3588_GMAC_CLK_TX_DL_CFG(tx_delay));
	return 0;		     
}

static int 
rk3588_set_rmii(device_t dev) 
{
	struct eqos_softc *sc = device_get_softc(dev);

	if (!sc->php_grf) {
		device_printf(dev, "%s: Missing rockchip,php_grf property\n", __func__);
		return ENXIO;
	}

	SYSCON_WRITE_4(sc->php_grf, RK3588_GRF_GMAC_CON0,
		     RK3588_GMAC_PHY_INTF_SEL_RMII(sc->idx));

	SYSCON_WRITE_4(sc->php_grf, RK3588_GRF_CLK_CON1,
		     RK3588_GMAC_CLK_RMII_MODE(sc->idx));
		     
	return 0;
}

static int
rk3588_set_speed(device_t dev, int speed)
{
	struct eqos_softc *sc = device_get_softc(dev);
	unsigned int val = 0, id = sc->idx;

	switch (speed) {
	case IFM_10_T:
		if (sc->phy_mode == MII_CONTYPE_RMII)
			val = RK3588_GMA_CLK_RMII_DIV20(id);
		else
			val = RK3588_GMAC_CLK_RGMII_DIV50(id);
		break;
	case IFM_100_TX:
		if (sc->phy_mode == MII_CONTYPE_RMII)
			val = RK3588_GMA_CLK_RMII_DIV2(id);
		else
			val = RK3588_GMAC_CLK_RGMII_DIV5(id);
		break;
	case IFM_1000_T:
	case IFM_1000_SX:
		if (sc->phy_mode != MII_CONTYPE_RMII)
			val = RK3588_GMAC_CLK_RGMII_DIV1(id);
		else
			goto err;
		break;
	default:
		goto err;
	}

	SYSCON_WRITE_4(sc->php_grf, RK3588_GRF_CLK_CON1, val);

	return 0;
err:
	device_printf(dev, "unknown speed value for GMAC speed=%d", speed);
	return (EINVAL);
}

static int 
rk3588_set_clock_selection(device_t dev, bool input)
{
	struct eqos_softc *sc = device_get_softc(dev);
	unsigned int val = input ? RK3588_GMAC_CLK_SELET_IO(sc->idx) :
				   RK3588_GMAC_CLK_SELET_CRU(sc->idx);

	val |= RK3588_GMAC_CLK_RMII_NOGATE(sc->idx);

	SYSCON_WRITE_4(sc->php_grf, RK3588_GRF_CLK_CON1, val);
	return 0;
}



static const struct eqos_ops rk3588_ops = {
	.set_to_rgmii = rk3588_set_rgmii,
	.set_to_rmii = rk3588_set_rmii,
	.set_rgmii_speed = rk3588_set_speed,
	.set_rmii_speed = rk3588_set_speed,
	.set_clock_selection = rk3588_set_clock_selection,
	.regs_valid = true,
	.regs = {
		0xfe1b0000, /* gmac0 */
		0xfe1c0000, /* gmac1 */
		0x0, /* sentinel */
	},
};


static const struct ofw_compat_data compat_data[] = {
	{"rockchip,rk3568-gmac",	(uintptr_t) &rk3568_ops},
	{"rockchip,rk3588-gmac",	(uintptr_t) &rk3588_ops},	
	{ NULL, 0 }
};

static int
eqos_check_ops(device_t dev) 
{
	struct eqos_softc *sc = device_get_softc(dev);
	switch (sc->phy_mode) {
	  case MII_CONTYPE_RGMII:
	  case MII_CONTYPE_RGMII_ID:
	  case MII_CONTYPE_RGMII_RXID:
	  case MII_CONTYPE_RGMII_TXID:
	  	if(!sc->ops->set_to_rgmii)
	  		return (EINVAL);
		break;  
	  case MII_CONTYPE_RMII:
	  	if(!sc->ops->set_to_rmii)
                        return (EINVAL);
                break;
	 }
	 
	return 0; 
}

static int
eqos_phy_reset(device_t dev)
{
	pcell_t gpio_prop[4];
	pcell_t delay_prop[3];
	phandle_t node, gpio_node;
	device_t gpio;
	uint32_t pin, flags;
	uint32_t pin_value;

	node = ofw_bus_get_node(dev);
	if (OF_getencprop(node, "snps,reset-gpio",
	    gpio_prop, sizeof(gpio_prop)) <= 0)
		return (0);

	if (OF_getencprop(node, "snps,reset-delays-us",
	    delay_prop, sizeof(delay_prop)) <= 0) {
		device_printf(dev,
		    "Wrong property for snps,reset-delays-us");
		return (ENXIO);
	}

	gpio_node = OF_node_from_xref(gpio_prop[0]);
	if ((gpio = OF_device_from_xref(gpio_prop[0])) == NULL) {
		device_printf(dev,
		    "Can't find gpio controller for phy reset\n");
		return (ENXIO);
	}

	if (GPIO_MAP_GPIOS(gpio, node, gpio_node,
	    nitems(gpio_prop) - 1,
	    gpio_prop + 1, &pin, &flags) != 0) {
		device_printf(dev, "Can't map gpio for phy reset\n");
		return (ENXIO);
	}

	pin_value = GPIO_PIN_LOW;
	if (OF_hasprop(node, "snps,reset-active-low"))
		pin_value = GPIO_PIN_HIGH;

	GPIO_PIN_SETFLAGS(gpio, pin, GPIO_PIN_OUTPUT);
	GPIO_PIN_SET(gpio, pin, pin_value);
	DELAY(delay_prop[0]);
	GPIO_PIN_SET(gpio, pin, !pin_value);
	DELAY(delay_prop[1]);
	GPIO_PIN_SET(gpio, pin, pin_value);
	DELAY(delay_prop[2]);

	return (0);
}
static void
eqos_fdt_axi(device_t dev)
{
	phandle_t child, node = ofw_bus_get_node(dev);
	struct eqos_softc *sc = device_get_softc(dev);
	uint32_t blen[AXI_BLEN] = { 0, 0, 0, 0, 0x10, 0x8, 0x4};	
	uint32_t temp;
	int i;
		
	sc->axi_rd_osr_lmt = 8;
	sc->axi_wr_osr_lmt = 16;
	sc->axi_blen[6] = 4;
	sc->axi_blen[5] = 8;
	sc->axi_blen[4] = 16; 	

	child = ofw_bus_find_child(node, "stmmac-axi-config");	
	if(child > 0) {
		if (OF_getencprop(child, "snps,rd_osr_lmt", &temp, sizeof(temp)) == sizeof(temp)) 
			sc->axi_rd_osr_lmt = (uint8_t)temp;
			
		if (OF_getencprop(child, "snps,wr_osr_lmt", &temp, sizeof(temp)) == sizeof(temp))
			sc->axi_wr_osr_lmt = (uint8_t)temp;

		if (OF_getencprop(child, "snps,blen", blen, sizeof(blen)) == sizeof(blen)) {
			for(i = 0; i < AXI_BLEN;i++)
				sc->axi_blen[i] = blen[i];
		}
	}
}

static int
eqos_fdt_init(device_t dev)
{
	struct eqos_softc *sc = device_get_softc(dev);
	phandle_t node = ofw_bus_get_node(dev);
	hwreset_t eqos_reset;
	regulator_t eqos_supply;
	uint32_t rx_delay, tx_delay;

	uint8_t buffer[16];
	const char *temp_name;
	char *clock_in_out;
	int i, error, n_clocks;
	


	sc->ops = (struct eqos_ops *)ofw_bus_search_compatible(dev, compat_data)->ocd_data;
	if (OF_hasprop(node, "rockchip,grf") &&
	    syscon_get_by_ofw_property(dev, node, "rockchip,grf", &sc->grf)) {
		device_printf(dev, "cannot get grf driver handle\n");
		return (ENXIO);
	}
	if (OF_hasprop(node, "rockchip,php-grf"))
	    syscon_get_by_ofw_property(dev, node, "rockchip,php-grf", &sc->php_grf);
	
	/* detect gmac index */
	if(sc->ops->regs_valid) {
	  i = 0;
	  while(sc->ops->regs[i]) {
	   if(sc->ops->regs[i] == rman_get_start(sc->res[EQOS_RES_MEM])) {
	    sc->idx = i;
	    break;
	    }
	   i++;
	   }
	 }
	sc->phy_mode = mii_fdt_get_contype(node);
        sc->clock_in = true;
        if (OF_getprop_alloc(node, "clock_in_out", (void **)&clock_in_out)) {
                if (strcmp(clock_in_out, "input") == 0)
                        sc->clock_in = true;
                else
                        sc->clock_in = false;
                OF_prop_free(clock_in_out);
        }
        /* Set the assigned clocks parent and freq 
        
        easier to control clock from cru / phy=external mess
        */
	        
        if (clk_set_assigned(dev, node) != 0) {
                device_printf(dev, "clk_set_assigned failed\n");
                goto fail;
        }
        
         error = clk_get_by_ofw_name(dev, 0, "stmmaceth", &sc->clk_stmmaceth);
         if (error != 0) {
		device_printf(dev, "could not find clock stmmaceth\n");
                 return (error);
          }
 	
 	 error = clk_enable(sc->clk_stmmaceth);
 	 if(error) {
 	   device_printf(dev, "could not enable clock stmmaceth\n");
 	   return (error);
 	   }
 	
 	 if(sc->clock_in) 
 	 	device_printf(dev, "Clock input from the PHY\n");
 	  else if(sc->phy_mode == MII_CONTYPE_RMII)  {
 	  clk_set_freq(sc->clk_stmmaceth, 50000000, 0);
  	  }	


	if (hwreset_get_by_ofw_idx(dev, node, 0, &eqos_reset)) {
		device_printf(dev, "cannot get reset\n");
		return (ENXIO);
	}
	else
		hwreset_assert(eqos_reset);

	if (OF_hasprop(node, "snps,force_thresh_dma_mode")) {
		sc->force_thresh_dma_mode = 1;
	}
		
	sc->csr_clock = 125000000;
	sc->csr_clock_range = GMAC_MAC_MDIO_ADDRESS_CR_100_150;
//	GMAC_MAC_MDIO_ADDRESS_CR_250_300;
	if (OF_getencprop(node, "tx_delay", &tx_delay, sizeof(tx_delay)) <= 0)
		tx_delay = 0x30;
	if (OF_getencprop(node, "rx_delay", &rx_delay, sizeof(rx_delay)) <= 0)
		rx_delay = 0x10;


	if (!regulator_get_by_ofw_property(dev, 0, "phy-supply",
	    &eqos_supply)) {
		if (regulator_enable(eqos_supply))
			device_printf(dev, "cannot enable 'phy' regulator\n");
	}
	else
		device_printf(dev, "no phy-supply property\n");

	if (eqos_phy_reset(dev))
		return (ENXIO);

	if (eqos_reset)
		hwreset_deassert(eqos_reset);
	n_clocks = nitems(eqos_clocks);
	if(sc->phy_mode == MII_CONTYPE_RMII) 
	 	n_clocks += nitems(eqos_rmii_clocks);
	sc->clks = (clk_t *)malloc(sizeof(clk_t) * n_clocks, M_DEVBUF, M_WAITOK | M_ZERO);		
	if(!sc->clks) {
	 	device_printf(dev,"Failed clocks malloc\n");
	 	return ENXIO;
	 }
	for(i = 0; i < n_clocks;i++) {
	 temp_name = (i < nitems(eqos_clocks)) ? eqos_clocks[i] : eqos_rmii_clocks[i - nitems(eqos_clocks)];
	 if (clk_get_by_ofw_name(dev, 0, temp_name, &sc->clks[i]) != 0) {
	  device_printf(dev, "Can't get clock %s\n", temp_name);
	  goto fail;
	  }
	 }
	 for(i = 0; i < n_clocks;i++) {
	  error = clk_enable(sc->clks[i]);
	  if(error) {
	   device_printf(dev, "Cannot enable clock %s\n",clk_get_name(sc->clks[i]));
	   goto fail;
	   }
	  } 
	 
	  if(sc->ops->set_clock_selection) {
	    sc->ops->set_clock_selection(dev, sc->clock_in);
	   }
	 error = eqos_check_ops(dev);
	 if(error) 
	 	goto fail;  
	 switch (sc->phy_mode) {
	    case MII_CONTYPE_RGMII:
	       error = sc->ops->set_to_rgmii(dev, tx_delay, rx_delay);
	       break;
	    case MII_CONTYPE_RGMII_ID:
	       error = sc->ops->set_to_rgmii(dev, 0, 0);
	       break;	    
	    case MII_CONTYPE_RGMII_RXID:
	       error = sc->ops->set_to_rgmii(dev, tx_delay, 0);
	       break;	    
	    case MII_CONTYPE_RGMII_TXID:
	       error = sc->ops->set_to_rgmii(dev, 0, rx_delay);
	       break;
	   case MII_CONTYPE_RMII:
	      error = sc->ops->set_to_rmii(dev);    
	      break;
	   default:
	      device_printf(dev,"Unknown mii con type %d\n", sc->phy_mode);
	      error = ENXIO;   
	  }  
	  if(error) 
	   goto fail; 
	/* set the MAC address if we have OTP data handy */
	if (!RK_OTP_READ(dev, buffer, 0, sizeof(buffer))) {
		uint32_t mac;

		mac = hash32_buf(buffer, sizeof(buffer), HASHINIT);
		WR4(sc, GMAC_MAC_ADDRESS0_LOW,
		    htobe32((mac & 0xffffff00) | 0x22));

		mac = hash32_buf(buffer, sizeof(buffer), mac);
		WR4(sc, GMAC_MAC_ADDRESS0_HIGH,
		    htobe16((mac & 0x0000ffff) + (device_get_unit(dev) << 8)));
	}
	eqos_fdt_axi(dev);
	return (0);
fail:
	if(sc->clks)  
		free(sc->clks, M_DEVBUF);
	return (ENXIO);  
	 
}
static int
eqos_set_speed(device_t dev, int speed)
{
	struct eqos_softc *sc = device_get_softc(dev);
	switch (sc->phy_mode) {
	case MII_CONTYPE_RGMII:
        case MII_CONTYPE_RGMII_ID:
        case MII_CONTYPE_RGMII_RXID:
        case MII_CONTYPE_RGMII_TXID:
        	if(sc->ops->set_rgmii_speed) 
        	  return sc->ops->set_rgmii_speed(dev, speed);
        	else 
        	  return 0;
        break;
        case MII_CONTYPE_RMII:
        	if(sc->ops->set_rmii_speed) 
        	  return sc->ops->set_rmii_speed(dev, speed);
        	else 
        	  return 0;        
        break;	 
        default:
        return 0;
	}
}
static int
eqos_fdt_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
        if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "DesignWare EQOS Gigabit ethernet");

	return (BUS_PROBE_DEFAULT);
}


static device_method_t eqos_fdt_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		eqos_fdt_probe),

	/* EQOS interface */
	DEVMETHOD(if_eqos_init,		eqos_fdt_init),
	DEVMETHOD(if_eqos_set_speed,    eqos_set_speed),

	DEVMETHOD_END
};

DEFINE_CLASS_1(eqos, eqos_fdt_driver, eqos_fdt_methods,
    sizeof(struct eqos_softc), eqos_driver);
DRIVER_MODULE(eqos, simplebus, eqos_fdt_driver, 0, 0);
MODULE_DEPEND(eqos, ether, 1, 1, 1);
MODULE_DEPEND(eqos, miibus, 1, 1, 1);
