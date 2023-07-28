/*-
 * Copyright (c) 2023 Priit Trees <trees@neti.ee>
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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/errno.h>
#include <sys/kernel.h>
#include <sys/kthread.h>
#include <sys/module.h>
#include <sys/socket.h>
#include <sys/sockio.h>

#include <net/if.h>
#include <net/if_media.h>
#include <net/if_types.h>

#include <dev/etherswitch/etherswitch.h>
#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "bcmswitch_reg.h"
#include "bcmswitch_var.h"
#include "bcmswitch_ps_phy.h"
#include "etherswitch_if.h"
#include "miibus_if.h"
#include "mdio_if.h"

#define BCMSWITCH_PSPHY_OPMASK                                  \
      (BCMSWITCH_PSPHY_ADDR_READ | BCMSWITCH_PSPHY_ADDR_WRITE)

static int bcmswitch_psphy_reg(device_t dev, uint8_t page, uint8_t reg,
    uint16_t opcode)
{
	int i;
	uint16_t addr;
	uint16_t asb;
	int err = 0;

	/* PHY registers are not accessible through the Pseudo PHY operation. */
	if (page >= 0x10 && page <= 0x1F) {
		device_printf(dev, "PHY page 0x%02x is not accessible.\n",
		    page);
		return (EINVAL);
	}

	/* set page number */
	MDIO_WRITE(dev, BCMSWITCH_PSPHY, BCMSWITCH_PSPHY_PAGE,
	    BCMSWITCH_PSPHY_PAGE_NUM(page) | BCMSWITCH_PSPHY_PAGE_EN);

	/* set register address */
	MDIO_WRITE(dev, BCMSWITCH_PSPHY, BCMSWITCH_PSPHY_ADDR,
	   BCMSWITCH_PSPHY_ADDR_REG(reg) | opcode);
	
	/* check if operation completed */
	for (i = 20 ; i > 0; --i) {
		addr = MDIO_READ(dev, BCMSWITCH_PSPHY, BCMSWITCH_PSPHY_ADDR);

		if ((addr & BCMSWITCH_PSPHY_OPMASK) == 0x00)
			break;
		if ((addr & BCMSWITCH_PSPHY_OPMASK) == BCMSWITCH_PSPHY_OPMASK)
			break;

		DELAY(10);
	}

	// liikuda ja tee GOTO
	if (!i)
	       err = ETIMEDOUT;	

	/* Bit 1 => Operation Error (RO/LH), when opcode = 2'b11, 
	 * this bit is set to show operation error.
	 * Bit 0 => Prohibit Access (RO/LH), for Page Number = 8'h1X,
	 * which are PHY MII registers.
	 */
	asb = MDIO_READ(dev, BCMSWITCH_PSPHY, BCMSWITCH_PSPHY_ASB);

	if (asb & BCMSWITCH_PSPHY_PA_ERR) {
		device_printf(dev, "Prohibit access to page 0x%02x\n", page);
		err = EPERM;
	}

	if (asb & BCMSWITCH_PSPHY_OP_ERR) {
		device_printf(dev, "Operation error: page 0x%02x reg 0x%02x\n",
		    page, reg);
		err = EIO;
	}

	/* clear opcode */
	if (addr & BCMSWITCH_PSPHY_OPMASK) {
		MDIO_WRITE(dev, BCMSWITCH_PSPHY, BCMSWITCH_PSPHY_ADDR,
				BCMSWITCH_PSPHY_ADDR_REG(reg));
	}
	return (err);
}

uint16_t
bcmswitch_readreg16(struct bcmswitch_softc *sc, int addr, int reg)
{
	BCMSWITCH_LOCK_ASSERT(sc, SA_XLOCKED);

	if (bcmswitch_psphy_reg(sc->dev, addr, reg, BCMSWITCH_PSPHY_ADDR_READ))
		return (0xFFFF);

	return(MDIO_READ(sc->dev, BCMSWITCH_PSPHY, BCMSWITCH_PSPHY_DATA0));
}

uint32_t
bcmswitch_readreg32(struct bcmswitch_softc *sc, int addr, int reg)
{
	BCMSWITCH_LOCK_ASSERT(sc, SA_XLOCKED);

	if (bcmswitch_psphy_reg(sc->dev, addr, reg, BCMSWITCH_PSPHY_ADDR_READ))
		return (0xFFFFFFFF);

	return(MDIO_READ(sc->dev, BCMSWITCH_PSPHY, BCMSWITCH_PSPHY_DATA0) |
	    MDIO_READ(sc->dev, BCMSWITCH_PSPHY, BCMSWITCH_PSPHY_DATA1) << 16);
}

uint64_t
bcmswitch_readreg64(struct bcmswitch_softc *sc, int addr, int reg)
{
	uint16_t data[4]; 
	uint64_t result = 0;

	BCMSWITCH_LOCK_ASSERT(sc, SA_XLOCKED);

	if (bcmswitch_psphy_reg(sc->dev, addr, reg, BCMSWITCH_PSPHY_ADDR_READ))
		return (0xFFFFFFFFFFFFFFFF);

	data[0] = MDIO_READ(sc->dev, BCMSWITCH_PSPHY, BCMSWITCH_PSPHY_DATA0);
	data[1] = MDIO_READ(sc->dev, BCMSWITCH_PSPHY, BCMSWITCH_PSPHY_DATA1);
	data[2] = MDIO_READ(sc->dev, BCMSWITCH_PSPHY, BCMSWITCH_PSPHY_DATA2);
	data[3] = MDIO_READ(sc->dev, BCMSWITCH_PSPHY, BCMSWITCH_PSPHY_DATA3);

	result |= (uint64_t)data[0] << 0;
	result |= (uint64_t)data[1] << 16;
	result |= (uint64_t)data[2] << 32;
	result |= (uint64_t)data[3] << 48;

	// tee ringi
	// printf ("data 0x%016llx\n", result);
	return (result);

}

int
bcmswitch_writereg16(struct bcmswitch_softc *sc, int addr, int reg,
    uint16_t val)
{
	BCMSWITCH_LOCK_ASSERT(sc, SA_XLOCKED);

	MDIO_WRITE(sc->dev, BCMSWITCH_PSPHY, BCMSWITCH_PSPHY_DATA0, val);

	if (bcmswitch_psphy_reg(sc->dev, addr, reg, BCMSWITCH_PSPHY_ADDR_WRITE))
		return EIO;

	return(0);
}

int
bcmswitch_writereg32(struct bcmswitch_softc *sc, int addr, int reg,
    uint32_t val)
{
	BCMSWITCH_LOCK_ASSERT(sc, SA_XLOCKED);

	MDIO_WRITE(sc->dev, BCMSWITCH_PSPHY, BCMSWITCH_PSPHY_DATA0, val & 0xFFFF);
	MDIO_WRITE(sc->dev, BCMSWITCH_PSPHY, BCMSWITCH_PSPHY_DATA1, val >> 16 & 0xFFFF);

	if (bcmswitch_psphy_reg(sc->dev, addr, reg, BCMSWITCH_PSPHY_ADDR_WRITE))
		return EIO;

	return(0);
}

int
bcmswitch_writereg64(struct bcmswitch_softc *sc, int addr, int reg,
    uint64_t val)
{
	BCMSWITCH_LOCK_ASSERT(sc, SA_XLOCKED);

	MDIO_WRITE(sc->dev, BCMSWITCH_PSPHY, BCMSWITCH_PSPHY_DATA0, val & 0xFFFF);
	MDIO_WRITE(sc->dev, BCMSWITCH_PSPHY, BCMSWITCH_PSPHY_DATA1, val >> 16 & 0xFFFF);
	MDIO_WRITE(sc->dev, BCMSWITCH_PSPHY, BCMSWITCH_PSPHY_DATA2, val >> 32 & 0xFFFF);
	MDIO_WRITE(sc->dev, BCMSWITCH_PSPHY, BCMSWITCH_PSPHY_DATA3, val >> 48 & 0xFFFF);

	if (bcmswitch_psphy_reg(sc->dev, addr, reg, BCMSWITCH_PSPHY_ADDR_WRITE))
		return EIO;

	return(0);
}
