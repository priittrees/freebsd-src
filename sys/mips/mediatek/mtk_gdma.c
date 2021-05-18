/*-
 * Copyright (c) 2020 Priit Trees <trees@neti.ee>
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
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/condvar.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>

#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <mips/mediatek/mtk_gdma.h>

#include "mtk_gdma_if.h"
#include "mtk_gdma.h"

#define	MTK_GDMA_MEMRES		0
#define	MTK_GDMA_IRQRES		1
#define	MTK_GDMA_RESSZ		2

struct mtk_gdma_softc;

struct mtk_gdma_channel {
	struct mtk_gdma_softc *	ch_sc;
	uint8_t			ch_index;
	void			(*ch_callback)(void *);
	void *			ch_callbackarg;
	uint32_t		ch_ctrl0;
	uint32_t		ch_ctrl1;
};

struct mtk_gdma_softc {
	device_t		dev;
	struct resource *	sc_res[MTK_GDMA_RESSZ];
	struct mtx		sc_mtx;
	void *			sc_ih;
	int			sc_ch_count;
	struct mtk_gdma_channel	* sc_channels;
};

static struct resource_spec mtk_gdma_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE },
	{ -1,			0,	0 }
};

static int mtk_gdma_probe(device_t);
static int mtk_gdma_attach(device_t);
static void mtk_gdma_intr(void *);
static int mtk_gdma_set_config(device_t, void *priv, const struct gdma_config *);
static void * mtk_gdma_alloc(device_t, bool dedicated, void (*cb)(void *),
    void *cbarg);
static void mtk_gdma_free(device_t, void *priv);
static int mtk_gdma_transfer(device_t, void *priv, bus_addr_t src,
    bus_addr_t dst, size_t nbytes);
static void mtk_gdma_halt(device_t, void *priv);

#define	MTK_GDMA_LOCK_SPIN(_sc)		mtx_lock_spin(&(_sc)->sc_mtx)
#define	MTK_GDMA_UNLOCK_SPIN(_sc)	mtx_unlock_spin(&(_sc)->sc_mtx)
#define	MTK_GDMA_READ_4(_sc, _reg)					\
	bus_read_4((_sc)->sc_res[MTK_GDMA_MEMRES], _reg)
#define	MTK_GDMA_WRITE_4(_sc, _reg, _value)				\
	bus_write_4((_sc)->sc_res[MTK_GDMA_MEMRES], _reg, _value)

static int
mtk_gdma_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "ralink,rt3883-gdma"))
		return (ENXIO);

	device_set_desc(dev, "MTK GDMA Controller");

	return (BUS_PROBE_DEFAULT);
}

/* All channel are round-robin. */
static int
mtk_gdma_attach(device_t dev)
{
	struct mtk_gdma_softc *sc;
	uint32_t val;
	uint8_t ip_ver;
	uint8_t ch_count;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;

	if (bus_alloc_resources(dev, mtk_gdma_spec, sc->sc_res)) {
		device_printf(dev, "cannot allocate resources for device\n");
		return (ENXIO);
	}

	mtx_init(&sc->sc_mtx, "mtk_gdma", NULL, MTX_SPIN);

	error = bus_setup_intr(dev, sc->sc_res[1], INTR_MPSAFE | INTR_TYPE_MISC,
	    NULL, mtk_gdma_intr, sc, &sc->sc_ih);
	if (error != 0) {
		device_printf(dev, "could not setup interrupt handler\n");
		bus_release_resources(dev, mtk_gdma_spec, sc->sc_res);
		mtx_destroy(&sc->sc_mtx);
		return (ENXIO);
	}

	/* Channel-0 doesn't have the highest priority.
	 * Channel-0 - Channel-n are round-robin
	 */
	MTK_GDMA_WRITE_4(sc, MTK_GDMA_GCT, MTK_GDMA_GCT_ARBIT_RR);

	val = MTK_GDMA_READ_4(sc, MTK_GDMA_GCT);
	ip_ver = (val >> MTK_GDMA_GCT_VER_SHIFT) & MTK_GDMA_GCT_VER_MASK;
	ch_count = 0x08 << ((val >> MTK_GDMA_GCT_CHAN_SHIFT) &
	    MTK_GDMA_GCT_CHAN_MASK);

	device_printf(dev, "ip version: %d, channels: %d\n", val, ch_count);

	/* Initialize channels */
	sc->sc_channels = malloc(sizeof(struct mtk_gdma_channel) * ch_count,
	    M_DEVBUF, M_WAITOK | M_ZERO);

	for (int i = 0; i < ch_count; i++) {
		sc->sc_channels[i].ch_sc = sc;
		sc->sc_channels[i].ch_index = i;
		sc->sc_channels[i].ch_callback = NULL;
		sc->sc_channels[i].ch_callbackarg = NULL;
	}
	sc->sc_ch_count = ch_count;

	OF_device_register_xref(OF_xref_from_node(ofw_bus_get_node(dev)), dev);

	return (0);
}

static void
mtk_gdma_intr(void *priv)
{
	struct mtk_gdma_softc *sc = priv;
	uint32_t val;

	val = MTK_GDMA_READ_4(sc, MTK_GDMA_DONE_INT);
	MTK_GDMA_WRITE_4(sc, MTK_GDMA_DONE_INT, val);

	for (int i = 0; i < sc->sc_ch_count; i++) {
		if (val & (1 << i)) {
			sc->sc_channels[i].ch_callback(
			    sc->sc_channels[i].ch_callbackarg);
		}
	}
}

/* TODO 
 * MT7620 MT76[2|8]8 suppprt Continuous Mode. 
 */ 

static int
mtk_gdma_set_config(device_t dev, void *priv, const struct gdma_config *cfg)
{
	struct mtk_gdma_channel *ch = priv;
	unsigned int burst_len;
	uint32_t ctrl0, ctrl1;

	switch (cfg->burst_len) {
	case 1:
		burst_len = MTK_GDMA_CTRL0_BURST_SIZE_WD1;
		break;
	case 2:
		burst_len = MTK_GDMA_CTRL0_BURST_SIZE_WD2;
		break;
	case 4:
		burst_len = MTK_GDMA_CTRL0_BURST_SIZE_WD4;
		break;
	case 8:
		burst_len = MTK_GDMA_CTRL0_BURST_SIZE_WD8;
		break;
	case 16:
		burst_len = MTK_GDMA_CTRL0_BURST_SIZE_WD16;
		break;
	default:
		return (EINVAL);
	}

	ctrl0 = (burst_len << MTK_GDMA_CTRL0_BURST_SHIFT) |
	    MTK_GDMA_CTRL0_DONE_INT | MTK_GDMA_CTRL0_ENABLE;

	ctrl1 = (cfg->src_dma_req << MTK_GDMA_CTRL1_SRC_REQ_SHIFT) |
	    (cfg->dst_dma_req << MTK_GDMA_CTRL1_DST_REQ_SHIFT) |
	    (ch->ch_index << MTK_GDMA_CTRL1_NEXT_SHIFT);

	if (cfg->dst_noincr)	/* MEM_TO_DEV */
		ctrl0 |= MTK_GDMA_CTRL0_DST_ADDR_FIXED;

	if (cfg->src_noincr)	/* DEV_TO_MEM */
		ctrl0 |= MTK_GDMA_CTRL0_SRC_ADDR_FIXED;

	if (cfg->soft_mode)	/* MEM_TO_MEM */
		ctrl0 |= MTK_GDMA_CTRL0_SW_MODE;

	if (cfg->coherent)	/* DEV_TO_MEM  MEM_TO_MEM */
		ctrl1 |= MTK_GDMA_CTRL1_COHERENT;

	ch->ch_ctrl0 = ctrl0;
	ch->ch_ctrl1 = ctrl1;

	return (0);
}

static void *
mtk_gdma_alloc(device_t dev, bool dedicated, void (*cb)(void *), void *cbarg)
{
	struct mtk_gdma_softc *sc;
	struct mtk_gdma_channel *ch_list;
	struct mtk_gdma_channel *ch = NULL;

	sc = device_get_softc(dev);
	ch_list = sc->sc_channels;

	MTK_GDMA_LOCK_SPIN(sc);
	for (int i = 0; i < sc->sc_ch_count; i++) {
		if (ch_list[i].ch_callback == NULL) {
			ch = &ch_list[i];
			ch->ch_callback = cb;
			ch->ch_callbackarg = cbarg;
			break;
		}
	}
	MTK_GDMA_UNLOCK_SPIN(sc);

	return (ch);
}

static void
mtk_gdma_free(device_t dev, void *priv)
{
	struct mtk_gdma_channel *ch = priv;
	struct mtk_gdma_softc *sc = ch->ch_sc;

	MTK_GDMA_LOCK_SPIN(sc);

	MTK_GDMA_WRITE_4(sc, MTK_GDMA_CTRL0(ch->ch_index), 0);

	ch->ch_callback = NULL;
	ch->ch_callbackarg = NULL;

	MTK_GDMA_UNLOCK_SPIN(sc);
}

static int
mtk_gdma_transfer(device_t dev, void *priv, bus_addr_t src, bus_addr_t dst,
    size_t nbytes)
{
	struct mtk_gdma_channel *ch = priv;
	struct mtk_gdma_softc *sc = ch->ch_sc;
	uint32_t ctrl0;

	ctrl0 = MTK_GDMA_READ_4(sc, MTK_GDMA_CTRL0(ch->ch_index));
	if (ctrl0 & MTK_GDMA_CTRL0_ENABLE) {
		device_printf(sc->dev, "chan %d is start(%08x).\n",
		    ch->ch_index, ctrl0);
		return EINVAL;
	}
	ctrl0 |= (nbytes << MTK_GDMA_CTRL0_TX_SHIFT);

	MTK_GDMA_WRITE_4(sc, MTK_GDMA_SRC_ADDR(ch->ch_index), src);
	MTK_GDMA_WRITE_4(sc, MTK_GDMA_DST_ADDR(ch->ch_index), dst);
	MTK_GDMA_WRITE_4(sc, MTK_GDMA_CTRL1(ch->ch_index), ch->ch_ctrl1);

	/* Channel transfer enable. */
	ctrl0 = ch->ch_ctrl0;
	ctrl0 |= (nbytes << MTK_GDMA_CTRL0_TX_SHIFT);
	MTK_GDMA_WRITE_4(sc, MTK_GDMA_CTRL0(ch->ch_index), ctrl0);

	return (0);
}

static void
mtk_gdma_halt(device_t dev, void *priv)
{
	struct mtk_gdma_channel *ch = priv;
	struct mtk_gdma_softc *sc = ch->ch_sc;

	MTK_GDMA_WRITE_4(sc, MTK_GDMA_CTRL0(ch->ch_index), 0);
}

static device_method_t mtk_gdma_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		mtk_gdma_probe),
	DEVMETHOD(device_attach,	mtk_gdma_attach),

	/* sunxi DMA interface */
	DEVMETHOD(mtk_gdma_alloc,	mtk_gdma_alloc),
	DEVMETHOD(mtk_gdma_free,	mtk_gdma_free),
	DEVMETHOD(mtk_gdma_set_config,	mtk_gdma_set_config),
	DEVMETHOD(mtk_gdma_transfer,	mtk_gdma_transfer),
	DEVMETHOD(mtk_gdma_halt,	mtk_gdma_halt),

	DEVMETHOD_END
};

static driver_t mtk_gdma_driver = {
	"gdma",
	mtk_gdma_methods,
	sizeof(struct mtk_gdma_softc)
};

static devclass_t mtk_gdma_devclass;

EARLY_DRIVER_MODULE(mtk_gdma, simplebus, mtk_gdma_driver, mtk_gdma_devclass,
    0, 0, BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LATE);
