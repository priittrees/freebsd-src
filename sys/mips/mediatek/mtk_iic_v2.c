/*-
 * Copyright (c) 2019 - 2021 Priit Trees <trees@neti.ee>
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

#include <mips/mediatek/mtk_iic_v2.h>

#define	MAX_DIV 0x7ff

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
static int mtk_iic_is_busy(struct mtk_iic_softc *);
static int mtk_iic_start(device_t, u_char slave, int timeout);
static int mtk_iic_stop(device_t);
static int mtk_iic_write(device_t, const char *buf, int len, int *sent, int timeout);
static int mtk_iic_read(device_t, char *buf, int len, int *read, int last, int delay);

#define MTK_IIC_LOCK(_sc)		mtx_lock(&(_sc)->mtx)
#define MTK_IIC_UNLOCK(_sc)		mtx_unlock(&(_sc)->mtx)
#define MTK_IIC_ASSERT_LOCKED(_sc)	mtx_assert(&(_sc)->mtx, MA_OWNED)
#define MTK_IIC_READ_4(_sc, _reg)	bus_read_4((_sc)->res, _reg)
#define MTK_IIC_WRITE_4(_sc, _reg, _val)				\
	bus_write_4((_sc)->res, _reg, _val)

static struct ofw_compat_data compat_data[] = {
	{ "ralink,i2c-mt7621",		1 },
	{ "mediatek,mt7628-i2c",	1 },
	{ NULL,				0 }
};

static int
mtk_iic_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

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

	return(bus_generic_attach(dev));

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
mtk_iic_is_busy(struct mtk_iic_softc *sc)
{
	int i;
	uint32_t val;
	int error = IIC_EBUSBSY;;

	MTK_IIC_ASSERT_LOCKED(sc);

	for (i = 0; i < 3000; i++) {
		val = MTK_IIC_READ_4(sc, MTK_IIC_SM0CTL1);
		if ((val & MTK_IIC_SM0_TRI) == 0) {
			error = IIC_NOERR;
			break;
		}
	}
	if(error)
		device_printf(sc->dev,
		    "interface is busy (status=0x%02x)\n", val);

	return (error);
}

static int
mtk_iic_start(device_t dev, u_char slave, int timeout)
{
	struct mtk_iic_softc *sc;
	int error;
	uint32_t val;

	sc = device_get_softc(dev);

	MTK_IIC_LOCK(sc);
	error = mtk_iic_is_busy(sc);
	if (error)
		goto fail;

	/* START Condition */
	val = MTK_IIC_SM0_TRI | MTK_IIC_SM0_MODE(MTK_IIC_MODE_START);
	MTK_IIC_WRITE_4(sc, MTK_IIC_SM0CTL1, val);
	error = mtk_iic_is_busy(sc);
	if (error)
		goto fail;

	/* Set Control Byte and
	 * 7-bit slave address plus a read/write bit
	 */
	MTK_IIC_WRITE_4(sc, MTK_IIC_SM0D0, slave);
	val = MTK_IIC_SM0_TRI | MTK_IIC_SM0_PGLEN(0) |
	    MTK_IIC_SM0_MODE(MTK_IIC_MODE_WRITE);
	MTK_IIC_WRITE_4(sc, MTK_IIC_SM0CTL1, val);
	error = mtk_iic_is_busy(sc);
	if (error)
		goto fail;

	/* Is a device generating an ACK */
	val = MTK_IIC_READ_4(sc, MTK_IIC_SM0CTL1);
	if ((MTK_IIC_SM0_ACK(val) & 0x01) == 0) {
		error = IIC_ENOACK;
		goto fail;
	}

fail:
	MTK_IIC_UNLOCK(sc);
	return (error);
}

static int
mtk_iic_stop(device_t dev)
{
	struct mtk_iic_softc *sc;
	int error;
	uint32_t val;

	sc = device_get_softc(dev);

	MTK_IIC_LOCK(sc);
	error = mtk_iic_is_busy(sc);
	if (error)
		goto fail;

	val = MTK_IIC_READ_4(sc, MTK_IIC_SM0CTL0);
	if (( val & MTK_IIC_SM0_SDA_STATE) == 0 ) {
		error = IIC_EUNDERFLOW;
		goto fail;
	}

	/* STOP Condition */
	val = MTK_IIC_SM0_MODE(MTK_IIC_MODE_STOP) | MTK_IIC_SM0_TRI;
	MTK_IIC_WRITE_4(sc, MTK_IIC_SM0CTL1, val);
	error = mtk_iic_is_busy(sc);
	if (error)
		goto fail;

fail:
	MTK_IIC_UNLOCK(sc);
	return (error);
}

static int
mtk_iic_write(device_t dev, const char *buf, int len, int *sent, int timeout)
{
	struct mtk_iic_softc *sc;
	uint32_t data[2];
	int pglen, error;
	uint32_t val;
	int ack;

	sc = device_get_softc(dev);

	MTK_IIC_LOCK(sc);

	error = mtk_iic_is_busy(sc);
	if (error)
		goto fail;

	*sent = 0;
	while (*sent < len) {
		/* MAX page size is 8. */
		pglen = MIN(len - *sent, 8);
		memset(data, '\0', pglen);
		memcpy(data, buf, pglen);

		/* Write bytes to register. */
		MTK_IIC_WRITE_4(sc, MTK_IIC_SM0D0, data[0]);
		if (pglen > 4)
			MTK_IIC_WRITE_4(sc, MTK_IIC_SM0D1, data[1]);

		/* Write Condition. */
		val = MTK_IIC_SM0_MODE(MTK_IIC_MODE_WRITE) | MTK_IIC_SM0_TRI |
		    MTK_IIC_SM0_PGLEN(pglen - 1);
		MTK_IIC_WRITE_4(sc, MTK_IIC_SM0CTL1, val);
		error = mtk_iic_is_busy(sc);
		if (error)
			goto fail;

		*(sent) += pglen;
		buf += pglen;

		/* Check is any ack is OK or return error. */
		ack = (0x1 << pglen) - 1;
		val = MTK_IIC_SM0_ACK(MTK_IIC_READ_4(sc, MTK_IIC_SM0CTL1));
		if (ack != val) {
			error = IIC_ENOACK;
			goto fail;
		}
	}

fail:
	MTK_IIC_UNLOCK(sc);
	return (error);
}

static int
mtk_iic_read(device_t dev, char *buf, int len, int *read, int last, int delay)
{
	struct mtk_iic_softc *sc;
	uint32_t mode;
	uint32_t data[2];
	int ack;
	uint32_t val;
	int pglen, last_read, error;

	sc = device_get_softc(dev);
	MTK_IIC_LOCK(sc);

	error = mtk_iic_is_busy(sc);
	if (error)
		goto fail;

	*read = 0;
	while (*read < len) {
		/* MAX page size is 8. */
		pglen = MIN(len - *read, 8);
		memset(data, '\0', pglen);

		/* When the master device is acting as a receiver, it uses a
		 * NACK instead of an ACK after the last data byte to indicate
		 * that it is finished receiving data.
		 */
		last_read = (pglen + *read < len) ? 0 : IIC_LAST_READ;
		mode = (last_read == IIC_LAST_READ) ?
		    MTK_IIC_SM0_MODE(MTK_IIC_MODE_READ_NACK) :
		    MTK_IIC_SM0_MODE(MTK_IIC_MODE_READ_ACK);

		/* Read Condition. */
		val =  MTK_IIC_SM0_PGLEN(pglen - 1) | mode | MTK_IIC_SM0_TRI;
		MTK_IIC_WRITE_4(sc, MTK_IIC_SM0CTL1, val);
		error = mtk_iic_is_busy(sc);
		if (error)
			goto fail;

		/* Check has ack been resived or return error */
		ack = (0x1 << pglen) - 1;
		if (last_read == IIC_LAST_READ)
			ack >>= 1;
		val = MTK_IIC_SM0_ACK(MTK_IIC_READ_4(sc, MTK_IIC_SM0CTL1));
		if (ack != val) {
			error = IIC_ENOACK;
			goto fail;
		}

		/* Read data and copy to buffer */
		data[0] = MTK_IIC_READ_4(sc, MTK_IIC_SM0D0);
		if (pglen > 4)
			data[1] = MTK_IIC_READ_4(sc, MTK_IIC_SM0D1);
		memcpy(buf, data, pglen);
		(*read) += pglen;
		buf += pglen;
	}

fail:
	MTK_IIC_UNLOCK(sc);
	return (error);
}


static int
mtk_iic_reset(device_t dev, u_char speed, u_char addr, u_char *oldaddr)
{
	struct mtk_iic_softc *sc;
	uint32_t busfreq;
	uint32_t div;
	uint32_t val;

	sc = device_get_softc(dev);

	fdt_reset_assert_all(dev);
	fdt_reset_deassert_all(dev);

	MTK_IIC_LOCK(sc);
	busfreq = IICBUS_GET_FREQUENCY(sc->iicbus, speed);

	/* MT7621 seems to have 11 bits for div and
	 * MT7628|MT7688 have 12 bit for div
	 */
	div = 40000000 / busfreq - 1;

	/* MT7628|MT7688 support sdandart-mode (100kHz) and fast-mode (400kHz)
	 * MT7621 seem to be support standart-mode (100kHz)  only
	 */
	if (div < 99)
		div = 99;
	else if (div > MAX_DIV)
		div = MAX_DIV;

	val = MTK_IIC_SM0_ODRAIN | MTK_IIC_SM0_CLK_DIV(div) |
	    MTK_IIC_SM0_EN | MTK_IIC_SM0_SCL_STRECH;
	MTK_IIC_WRITE_4(sc, MTK_IIC_SM0CTL0, val);
	MTK_IIC_WRITE_4(sc, MTK_IIC_SM0CFG2, 0);
	MTK_IIC_UNLOCK(sc);
	return (IIC_NOERR);
}

static phandle_t
mtk_iic_get_node(device_t bus, device_t dev)
{
	return ofw_bus_get_node(bus);
}

static device_method_t mtk_iic_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		mtk_iic_probe),
	DEVMETHOD(device_attach,	mtk_iic_attach),
	DEVMETHOD(device_detach,	mtk_iic_detach),

	/* ofw interface */
	DEVMETHOD(ofw_bus_get_node,	mtk_iic_get_node),

	/* iicbus interface */
	DEVMETHOD(iicbus_callback,	iicbus_null_callback),
	DEVMETHOD(iicbus_reset,		mtk_iic_reset),
	DEVMETHOD(iicbus_start,		mtk_iic_start),
        DEVMETHOD(iicbus_repeated_start,mtk_iic_start),
	DEVMETHOD(iicbus_transfer,	iicbus_transfer_gen),
	DEVMETHOD(iicbus_stop,		mtk_iic_stop),
	DEVMETHOD(iicbus_read,		mtk_iic_read),
	DEVMETHOD(iicbus_write,		mtk_iic_write),

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
