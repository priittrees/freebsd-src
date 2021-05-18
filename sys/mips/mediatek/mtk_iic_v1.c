/*-
 * Copyright (c) 2020 - 2021 Priit Trees <trees@neti.ee>
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

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/bus.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/iicbus/iiconf.h>
#include <dev/iicbus/iicbus.h>

#include "iicbus_if.h"
#include <mips/mediatek/fdt_reset.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <mips/mediatek/mtk_iic_v1.h>

#define MAX_DIV 0xffff
#define MAXBYTECNT 64

struct mtk_iic_softc {
	device_t	dev;
	device_t	iicbus;
	struct resource	*res;
	struct mtx	mtx;
};

static struct resource_spec mtk_iic_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ -1,			0,	0 }
};

static int mtk_iic_probe(device_t);
static int mtk_iic_attach(device_t);
static int mtk_iic_detach(device_t);
static int mtk_iic_transfer_is_busy(struct mtk_iic_softc *);
static int mtk_iic_transfer_read(struct mtk_iic_softc *, uint8_t *buf, int len);
static int mtk_iic_transfer_write(struct mtk_iic_softc *, uint8_t *buf, int len);
static int mtk_iic_transfer(device_t, struct iic_msg *msgs, uint32_t nmsgs);
static int mtk_iic_reset(device_t, u_char speed, u_char addr, u_char *oldaddr);

#define MTK_IIC_LOCK(_sc)		mtx_lock(&(_sc)->mtx)
#define MTK_IIC_UNLOCK(_sc)		mtx_unlock(&(_sc)->mtx)
#define MTK_IIC_ASSERT_LOCKED(_sc)	mtx_assert(&(_sc)->mtx, MA_OWNED)
#define MTK_IIC_READ_4(_sc, _reg)	bus_read_4((_sc)->res, _reg)
#define MTK_IIC_WRITE_4(_sc, _reg, _val)				\
	bus_write_4((_sc)->res, _reg, _val)

static struct ofw_compat_data compat_data[] = {
	{ "ralink,rt2880-i2c",  1 },
	{ "ralink,rt3050-i2c",  1 },
	{ NULL,			0 }
};

static int
mtk_iic_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return(ENXIO);

	device_set_desc(dev, "MTK IIC Controller");

	return (0);
}

static int
mtk_iic_attach(device_t dev)
{
	struct mtk_iic_softc *sc;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;

	if (bus_alloc_resources(dev, mtk_iic_spec, &sc->res) != 0) {
		device_printf(dev, "cannot allocate resources for device\n");
		error = ENXIO;
		goto fail;
	}

	mtx_init(&sc->mtx, device_get_nameunit(dev), "mtk_iic", MTX_DEF);

	/* Reset */
	fdt_reset_assert_all(dev);
	fdt_reset_deassert_all(dev);

	sc->iicbus = device_add_child(dev, "iicbus", -1);
	if (sc->iicbus == NULL) {
		device_printf(dev, "cannot add iicbus child device\n");
		error = ENXIO;
		goto fail;
	}

	return (bus_generic_attach(dev));

fail:
	mtk_iic_detach(dev);
	return (error);
}

static int
mtk_iic_detach(device_t dev)
{
	struct mtk_iic_softc *sc;

	bus_generic_detach(dev);

	sc = device_get_softc(dev);
	if (sc->iicbus != NULL)
		device_delete_child(dev, sc->iicbus);
	mtx_destroy(&sc->mtx);
	if (sc->res != NULL)
		bus_release_resources(dev, mtk_iic_spec, &sc->res);

	return (0);
}

static int
mtk_iic_transfer_is_busy(struct mtk_iic_softc *sc)
{
	int i;
	uint32_t val;
	int error = IIC_EBUSBSY;

	MTK_IIC_ASSERT_LOCKED(sc);

	for (i = 0; i < 3000; i++) {
		val = MTK_IIC_READ_4(sc, MTK_IIC_STATUS);
		if ((val & MTK_IIC_STATUS_BUSY) == 0) {
			error = IIC_NOERR;
			break;
		}
	}
	if (error)
		device_printf(sc->dev,
		    "interface is busy (status=0x%02x)\n", val);

	return (error);
}

static int
mtk_iic_transfer_read(struct mtk_iic_softc *sc, uint8_t *buf, int len)
{
	uint16_t resid;
	uint32_t val;

	if (len > MAXBYTECNT)
		return(IIC_ENOTSUPP);

	MTK_IIC_ASSERT_LOCKED(sc);
	MTK_IIC_WRITE_4(sc, MTK_IIC_BYTECNT, len - 1);
	MTK_IIC_WRITE_4(sc, MTK_IIC_STARTXFR, MTK_IIC_OP_READ);
	for (resid = len; resid > 0; resid--) {
		/* TODO The time of arrival of the ACK depends on the bus
		 * speed. Need to find a method to calculate it dynamically
		 * On 100kz bus speed is the value 100.
		 */
		DELAY(100);
		int timeout = 5000;
		while(1) {
			val = MTK_IIC_READ_4(sc, MTK_IIC_STATUS);
			if ((val & MTK_IIC_STATUS_STARTERR) != 0) {
				return (IIC_EOVERFLOW);
			}
			else if ((val & MTK_IIC_STATUS_ACKERR) != 0) {
				/* Clear buffer.
				 * Controller stay busy status
				 * if thist don't do it
				 */
				DELAY(100);
				MTK_IIC_READ_4(sc, MTK_IIC_DATAIN);
				return (IIC_ENOACK);
			}
			else if((val & MTK_IIC_STATUS_DATARDY) != 0) {
				buf[len - resid] =
					MTK_IIC_READ_4(sc, MTK_IIC_DATAIN);
				break;
			}
			if (--timeout == 0) {
				device_printf(sc->dev,
				    "read timeout (status=0x%02x)\n",
				    val);
				return (IIC_ETIMEOUT);
			}
		}
	}
	return (IIC_NOERR);
}

static int
mtk_iic_transfer_write(struct mtk_iic_softc *sc, uint8_t *buf, int len)
{
	uint16_t resid;
	uint32_t val;

	if (len > MAXBYTECNT)
		return(IIC_ENOTSUPP);

	/* TODO We can write only device address.
	 * if (buf == NULL)
	 *	startxfr |= MTK_IIC_NODATA;
	 */

	/* TODO Stop contition is able disappears.
	 * if ((flag & IIC_M_NOSTOP) != 0)
	 *	startxfr |= 0x04;
	 */

	MTK_IIC_ASSERT_LOCKED(sc);
	MTK_IIC_WRITE_4(sc, MTK_IIC_BYTECNT, len - 1);
	for (resid = len; resid > 0; resid--) {
		MTK_IIC_WRITE_4(sc, MTK_IIC_DATAOUT, buf[len - resid]);
		if(resid == len)
			MTK_IIC_WRITE_4(sc, MTK_IIC_STARTXFR, MTK_IIC_OP_WRITE);
		/* TODO The time of arrival of the ACK depends on the bus
		 * speed. Need to find a method to calculate it dynamically
		 * On 100kz bus speed is the value 120.
		 */
		DELAY(120);
		int timeout = 5000;
		while(1) {
			val = MTK_IIC_READ_4(sc, MTK_IIC_STATUS);
			if ((val & MTK_IIC_STATUS_STARTERR) != 0) {
				return (IIC_EOVERFLOW);
			}
			else if ((val & MTK_IIC_STATUS_ACKERR) != 0) {
				return (IIC_ENOACK);
			}
			else if ((val & MTK_IIC_STATUS_SDOEMPTY) != 0) {
				break;
			}
			if (--timeout == 0) {
				device_printf(sc->dev,
				    "write timeout (status=0x%02x)\n",
				    val);
				return (IIC_ETIMEOUT);
			}
		}
	}
	return (IIC_NOERR);
}

static int
mtk_iic_transfer(device_t dev, struct iic_msg *msgs, uint32_t nmsgs)
{
	struct mtk_iic_softc *sc;
	uint32_t n;
	int error;

	sc = device_get_softc(dev);

	MTK_IIC_LOCK(sc);
	for (n = 0; n < nmsgs; n++)
	{
		/* Check if iic controller is free */
		error = mtk_iic_transfer_is_busy(sc);
		if (error)
			goto fail;

		/* The start condition cannot be avoided.
		 * When writing an EEPROM, the data must immediately
		 * follow the address. Now max count is 64 + device address.
		 * TODO Adding a sub address when writing.
		 * This Allows a max transaction of 64 bytes + one address
		 * + device aadress.
		 */
		if ((msgs[n].flags == IIC_M_NOSTOP) &&
				(msgs[n + 1].flags == IIC_M_NOSTART)) {

			uint8_t buf[MAXBYTECNT];
			int len;

			len = msgs[n].len + msgs[n + 1].len;
			if (len > MAXBYTECNT) {
				error = IIC_ENOTSUPP;
				goto fail;
			}

			memcpy(buf, msgs[n].buf, msgs[n].len);
			memcpy(buf + msgs[n].len, msgs[n + 1].buf,
					MAXBYTECNT - msgs[n].len);

			error = mtk_iic_transfer_write(sc, buf, len);
			if (error)
				goto fail;

			n++;
			continue;
		}

		/* Set target address */
		if (n == 0 || msgs[n].slave != msgs[n - 1].slave) {
			MTK_IIC_WRITE_4(sc, MTK_IIC_DEVADDR,
					msgs[n].slave >> 1);
		}

		/* Read or write condition */
		if ((msgs[n].flags & IIC_M_RD) != 0)
			error = mtk_iic_transfer_read(
					sc, msgs[n].buf, msgs[n].len);
		else
			error = mtk_iic_transfer_write(
					sc, msgs[n].buf, msgs[n].len);

		if (error)
			goto fail;
	}
fail:
	MTK_IIC_UNLOCK(sc);
	return (error);
}

/*
 * MT7620 and RT5350 SCLK frequency = 40 MHz / ( 2 x CLKDIV )
 * TODO TR3050 SCLK frequency = pb_clk frequency / ( 2 x CLKDIV )
 *  pb_clk frequency = 1/3 CPU clock frequency
*/

static int
mtk_iic_reset(device_t dev, u_char speed, u_char addr, u_char *oldaddr)
{
	struct mtk_iic_softc *sc;
	uint32_t busfreq;
	uint32_t div;
	uint32_t val;

	sc = device_get_softc(dev);

	/* Reset controller */
	fdt_reset_assert_all(dev);
	fdt_reset_deassert_all(dev);

	MTK_IIC_LOCK(sc);
	busfreq = IICBUS_GET_FREQUENCY(sc->iicbus, speed);
	div = 40000000/(2 * busfreq);

	if (div < 50)
		div = 50;
	else if (div > MAX_DIV)
		div = MAX_DIV;

	MTK_IIC_WRITE_4(sc, MTK_IIC_CLKDIV, div);
	val = MTK_IIC_CONFIG_ADDRDIS |
	    MTK_IIC_CONFIG_ADDRLEN(MTK_IIC_CONFIG_ADDRLEN_8) |
	    MTK_IIC_CONFIG_DEVADLEN(MTK_IIC_CONFIG_DEVADLEN_7);
	MTK_IIC_WRITE_4(sc, MTK_IIC_CONFIG, val);
	MTK_IIC_UNLOCK(sc);

	return (IIC_NOERR);
}

static phandle_t
mtk_iic_get_node(device_t bus, device_t dev)
{
	return (ofw_bus_get_node(bus));
}

static device_method_t mtk_iic_methods[] = {
	/* device interface */
	DEVMETHOD(device_probe, mtk_iic_probe),
	DEVMETHOD(device_attach, mtk_iic_attach),
	DEVMETHOD(device_detach, mtk_iic_detach),

	/* ofw interface */
	DEVMETHOD(ofw_bus_get_node, mtk_iic_get_node),

	/* iicbus interface */
	DEVMETHOD(iicbus_callback, iicbus_null_callback),
	DEVMETHOD(iicbus_reset, mtk_iic_reset),
	DEVMETHOD(iicbus_transfer, mtk_iic_transfer),

	DEVMETHOD_END
};

static driver_t mtk_iic_driver = {
	"iichb",
	mtk_iic_methods,
	sizeof(struct mtk_iic_softc),
};

static devclass_t mtk_iic_devclass;

DRIVER_MODULE(mtk_iic, simplebus, mtk_iic_driver, mtk_iic_devclass, 0, 0);
DRIVER_MODULE(iicbus, mtk_iic, iicbus_driver, iicbus_devclass, 0, 0);

MODULE_DEPEND(mtk_iic, iicbus, IICBUS_MINVER, IICBUS_PREFVER, IICBUS_MAXVER);

MODULE_VERSION(mtk_iic, 1);
