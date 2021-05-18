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
 * $FreeBSD$
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/resource.h>
#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/extres/clk/clk.h>

#include <mips/mediatek/mtk_soc.h>
#include <mips/mediatek/mtk_sysctl.h>

#include "opt_snd.h"
#include <dev/sound/pcm/sound.h>
#include <dev/sound/fdt/audio_dai.h>
#include "audio_dai_if.h"

#include "mtk_i2s.h"
#include "mtk_gdma_if.h"

#define MEM_REQ		32

static const struct ofw_compat_data compat_data[] = {
	{ "ralink,rt3352-i2s",		MTK_SOC_RT3352 },
	{ "mediatek,mt7628-i2s",	MTK_SOC_MT7628 },
	{ NULL,				MTK_SOC_UNKNOWN }
};

static struct resource_spec mtk_i2s_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE | RF_SHAREABLE },
	{ -1, 0 }
};

struct mtk_i2s_channel {
	struct mtk_i2s_softc *	parent;
	struct pcm_channel *	pcm;
	struct snd_dbuf *	buf;
	int			pcm_dir;

	bus_dmamap_t		dmamap;
	void *			dmaaddr;
	bus_addr_t		physaddr;
	bus_size_t		fifo;
	device_t		dmac;
	void *			dmachan;

	int			run;
	uint32_t		pos;
};

struct mtk_i2s_softc {
	device_t		dev;
	struct resource *	res[2];
	struct mtx		mtx;
	clk_t			i2s_clk;
	struct mtk_i2s_channel	play_channel;
	struct mtk_i2s_channel	rec_channel;
	uint32_t		socid;
	bool			master;

	bus_dma_tag_t		dmat;
	unsigned		dmasize;
	uint32_t		txdma_req;
	uint32_t		rxdma_req;
};

#define	MTK_I2S_LOCK(sc)		mtx_lock(&(sc)->mtx)
#define	MTK_I2S_UNLOCK(sc)		mtx_unlock(&(sc)->mtx)
#define	MTK_I2S_READ_4(sc, reg)		bus_read_4((sc)->res[0], (reg))
#define	MTK_I2S_WRITE_4(sc, reg, val)	bus_write_4((sc)->res[0], (reg), (val))

static int mtk_i2s_probe(device_t);
static int mtk_i2s_attach(device_t);
static int mtk_i2s_detach(device_t);

static void * mtk_i2s_dma_init(device_t, struct snd_dbuf *b,
    struct pcm_channel *c, int pcm_dir);
static int mtk_i2s_dma_free(device_t, int pcm_dir);
static void mtk_i2s_dma_intr(void *priv);
static void mtk_i2s_dma_config(struct mtk_i2s_channel *);
static void mtk_i2s_dma_transfer(struct mtk_i2s_channel *);

static uint32_t sc_fmt[] = {
	SND_FORMAT(AFMT_S16_LE, 2, 0),
	0
};

static struct pcmchan_caps mtk_i2s_caps = {8000, 48000, sc_fmt, 0};

static int
mtk_i2s_probe(device_t dev)
{
	struct mtk_i2s_softc *sc = device_get_softc(dev);

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	sc->socid = ofw_bus_search_compatible(dev, compat_data)->ocd_data;
	if (sc->socid == MTK_SOC_UNKNOWN)
		return (ENXIO);

	device_set_desc(dev, "MTK I2S Controller");
	return (BUS_PROBE_DEFAULT);
}

static int
mtk_i2s_attach(device_t dev)
{
	struct mtk_i2s_softc *sc;
	int error;
	phandle_t node;
	uint32_t val;

	sc = device_get_softc(dev);
	sc->dev = dev;

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	if (bus_alloc_resources(dev, mtk_i2s_spec, sc->res) != 0) {
		device_printf(dev, "cannot allocate resources for device\n");
		error = ENXIO;
		goto fail;
	}

	/* We need sc->i2s_clk only for master mode to generate bclk and lrclk.
	 * If you want to use a mic, it is recommended to set the
	 * Ralink/Mediatek device to slave mode.
	 * Some mic require 32bit long LRCLK. Ralin\Mediateks support 16bit
	 * long LRCLK expext MT76[2|8]8. MT76[2|8]8 can support 24bit long LRCLK
	 */
	clk_get_by_ofw_name(dev, 0, "i2s_clk", &sc->i2s_clk);

	node = ofw_bus_get_node(dev);
	if (OF_getencprop(node, "txdma-req",
	    &sc->txdma_req, sizeof(sc->txdma_req)) <= 0)
		sc->txdma_req = 0;

	if (OF_getencprop(node, "rxdma-req",
	    &sc->rxdma_req, sizeof(sc->rxdma_req)) <= 0)
		sc->rxdma_req = 0;

	sc->master = false;

	/* DMA */
	sc->dmasize = 131072;
	error = bus_dma_tag_create(
	    bus_get_dma_tag(dev),
	    4, sc->dmasize,             /* alignment, boundary */
	    BUS_SPACE_MAXADDR_32BIT,    /* lowaddr */
	    BUS_SPACE_MAXADDR,          /* highaddr */
	    NULL, NULL,                 /* filter, filterarg */
	    sc->dmasize, 1,             /* maxsize, nsegs */
	    sc->dmasize, 0,             /* maxsegsize, flags */
	    NULL, NULL,                 /* lockfunc, lockarg */
	    &sc->dmat);
	if (error != 0) {
		device_printf(dev, "cannot create DMA tag\n");
		goto fail;
	}

	OF_device_register_xref(OF_xref_from_node(node), dev);

	val = MTK_I2S_CFG0_DFT_THRES;
	if (sc->socid == MTK_SOC_MT7628)
		val |= MTK_I2S_CFG0_SYS_BE;

	MTK_I2S_WRITE_4(sc, MTK_I2S_CFG0, val);
	MTK_I2S_WRITE_4(sc, MTK_I2S_INT_EN, 0);

	/* MT76[2|8]8 I2S PAD selection */
	if (sc->socid == MTK_SOC_MT7628) {
		val = mtk_sysctl_get(SYSCTL_AGPIOCFG);
		val |= 0x0f;
		mtk_sysctl_set(SYSCTL_AGPIOCFG, val);
	}

	return (0);

fail:
	mtk_i2s_detach(dev);
	return (error);
}

static int
mtk_i2s_detach(device_t dev)
{
	struct mtk_i2s_softc *sc;

	sc = device_get_softc(dev);

	if (sc->i2s_clk)
		clk_release(sc->i2s_clk);

	bus_release_resources(dev, mtk_i2s_spec, sc->res);
	mtx_destroy(&sc->mtx);

	return (0);
}

static void
mtk_i2s_setup_bclk (struct mtk_i2s_softc *sc, uint32_t speed)
{
	uint64_t clk_freq;
	int width;
	uint32_t divint, divcmp;

	width = 16;
	clk_get_freq(sc->i2s_clk, &clk_freq);
	/* FREQOUT = FREQIN * (1/2) * (1/(DIVINT + DIVCOMP/512)) */
	divint = (clk_freq / (2 * 2 * width)) / speed;
	divcmp = ((clk_freq % speed) * 512) / speed;
	device_printf(
	    sc->dev,"i2c_clk:%lluMHz width:%u speed:%u int:%d comp:%d\n",
	    clk_freq/1000000, width, speed, divint, divcmp);
	MTK_I2S_WRITE_4(sc, MTK_I2S_DIVINT, divint);
	MTK_I2S_WRITE_4(sc, MTK_I2S_DIVCMP, divcmp);
}

static void
mtk_i2s_enable_bclk (struct mtk_i2s_softc *sc)
{
	uint32_t val;
	val = MTK_I2S_READ_4(sc, MTK_I2S_CFG0);
	val |=MTK_I2S_CLK_EN;
	MTK_I2S_WRITE_4(sc, MTK_I2S_DIVCMP, val);
}

static void
mtk_i2s_disable_bclk (struct mtk_i2s_softc *sc)
{
	uint32_t val;
	val = MTK_I2S_READ_4(sc, MTK_I2S_CFG0);
	val &=~MTK_I2S_CLK_EN;
	MTK_I2S_WRITE_4(sc, MTK_I2S_DIVCMP, val);
}

static int
mtk_i2s_dai_init(device_t dev, uint32_t format)
{
	struct mtk_i2s_softc *sc;
	int fmt, pol, clk;
	uint32_t val;

	sc = device_get_softc(dev);

	fmt = AUDIO_DAI_FORMAT_FORMAT(format);
	pol = AUDIO_DAI_FORMAT_POLARITY(format);
	clk = AUDIO_DAI_FORMAT_CLOCK(format);

	/* Set format */
	val = MTK_I2S_READ_4(sc, MTK_I2S_CFG0);

	switch (clk) {
	case AUDIO_DAI_CLOCK_CBM_CFM:
		/* check, does i2s_clk is exist */
		if (!sc->i2s_clk) { 
			device_printf(dev, "cannot parse i2s_clk property.\n");
			return (EINVAL);
		}
		sc->master = true;
		val &= ~MTK_I2S_CFG0_SLAVE;
		break;
	case AUDIO_DAI_CLOCK_CBS_CFS:
		val |= MTK_I2S_CFG0_SLAVE;
		break;
	default:
		return (EINVAL);
	}

	MTK_I2S_WRITE_4(sc, MTK_I2S_CFG0, val);

	switch (pol) {
	case AUDIO_DAI_POLARITY_NB_NF:
		break;
	default:
		return (EINVAL);
	}

	val = MTK_I2S_READ_4(sc, MTK_I2S_CFG1);
	/* clear I2S audio data format*/
	val &= 0x1;
	switch (fmt) {
	case AUDIO_DAI_FORMAT_I2S:
		break;
	case AUDIO_DAI_FORMAT_LJ:
		if (sc->socid == MTK_SOC_MT7628) {
			val |= MTK_I2S_CFG1_LEFT_J;
			break;
		}
	default:
		return EINVAL;
	}

	MTK_I2S_WRITE_4(sc, MTK_I2S_CFG1, val);

	return (0);
}

static struct pcmchan_caps *
mtk_i2s_dai_get_caps(device_t dev)
{
	return (&mtk_i2s_caps);
}

static void
mtk_i2s_start(struct mtk_i2s_channel *ch)
{
	int val;
	ch->pos = 0;

	mtk_i2s_dma_config(ch);

	val = MTK_I2S_READ_4(ch->parent, MTK_I2S_CFG0);
	val |= MTK_I2S_CFG0_I2S_EN;
	val |= MTK_I2S_CFG0_DMA_EN;
	if (ch->pcm_dir == PCMDIR_PLAY)
		val |= MTK_I2S_CFG0_TX_EN;
	else
		val |= MTK_I2S_CFG0_RX_EN;
	MTK_I2S_WRITE_4(ch->parent, MTK_I2S_CFG0, val);

	/* Start DMA transfer */
	mtk_i2s_dma_transfer(ch);
}

static void
mtk_i2s_stop(struct mtk_i2s_channel *ch)
{
	int val;

	/* Disable DMA channel */
	MTK_GDMA_HALT(ch->dmac, ch->dmachan);

	val = MTK_I2S_READ_4(ch->parent, MTK_I2S_CFG0);
	val &= ~MTK_I2S_CFG0_I2S_EN;
	val &= ~MTK_I2S_CFG0_DMA_EN;
	if (ch->pcm_dir == PCMDIR_PLAY)
		val &= ~MTK_I2S_CFG0_TX_EN;
	else
		val &= ~MTK_I2S_CFG0_RX_EN;
	MTK_I2S_WRITE_4(ch->parent, MTK_I2S_CFG0, val);
}

static int
mtk_i2s_dai_trigger(device_t dev, int go, int pcm_dir)
{
	struct mtk_i2s_softc *sc;
	struct mtk_i2s_channel *ch;

	sc = device_get_softc(dev);

	if ((pcm_dir != PCMDIR_PLAY) && (pcm_dir != PCMDIR_REC))
		return (EINVAL);

	ch = pcm_dir == PCMDIR_PLAY ? &sc->play_channel : &sc->rec_channel;

	switch (go) {
	case PCMTRIG_START:
		ch->run = 1;
		mtk_i2s_stop(ch);
		if (sc->master)
			mtk_i2s_enable_bclk(sc);
		mtk_i2s_start(ch);
		break;
	case PCMTRIG_STOP:
	case PCMTRIG_ABORT:
		ch->run = 0;
		mtk_i2s_stop(ch);
		if (sc->master)
			mtk_i2s_disable_bclk(sc);
		break;
	}

	return (0);
}

static uint32_t
mtk_i2s_dai_get_ptr(device_t dev, int pcm_dir)
{
	struct mtk_i2s_softc *sc;
	struct mtk_i2s_channel *ch;

	sc = device_get_softc(dev);
	ch = pcm_dir == PCMDIR_PLAY ?
	    &sc->play_channel : &sc->rec_channel;

	return (ch->pos);
}

static void
mtk_i2s_dmamap_cb(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
	struct mtk_i2s_channel *ch = arg;

	if (error != 0)
		return;

	ch->physaddr = segs[0].ds_addr;
}

static void *
mtk_i2s_dma_init(device_t dev, struct snd_dbuf *b, struct pcm_channel *c,
    int pcm_dir)
{
	struct mtk_i2s_softc *sc;
	struct mtk_i2s_channel *ch;
	phandle_t xref;
	pcell_t *cells;
	int ncells;
	int error;

	sc = device_get_softc(dev);
	ch = pcm_dir == PCMDIR_PLAY ? &sc->play_channel : &sc->rec_channel;
	ch->parent = sc;
	ch->buf = b;
	ch->pcm = c;
	ch->pcm_dir = pcm_dir;

	error = ofw_bus_parse_xref_list_alloc(ofw_bus_get_node(sc->dev),
	    "dmas", "#dma-cells", 0, &xref, &ncells, &cells);
	if (error != 0) {
		device_printf(sc->dev, "cannot parse 'dmas' property\n");
		return NULL;
	}
	OF_prop_free(cells);

	ch->fifo = rman_get_start(sc->res[0]) +
	    (pcm_dir == PCMDIR_REC ? MTK_I2S_RR_EG : MTK_I2S_WR_EG);

	ch->dmac = OF_device_from_xref(xref);
	if (ch->dmac == NULL) {
		device_printf(sc->dev, "cannot find DMA controller\n");
		return NULL;
	}

	ch->dmachan = MTK_GDMA_ALLOC(ch->dmac, false, mtk_i2s_dma_intr, ch);
	if (ch->dmachan == NULL) {
		device_printf(sc->dev, "cannot allocate DMA channel\n");
		return NULL;
	}

	error = bus_dmamem_alloc(sc->dmat, &ch->dmaaddr,
	    BUS_DMA_NOWAIT | BUS_DMA_COHERENT, &ch->dmamap);

	if (error != 0) {
		device_printf(sc->dev, "cannot allocate channel buffer\n");
		return NULL;
	}

	error = bus_dmamap_load(sc->dmat, ch->dmamap, ch->dmaaddr,
	    sc->dmasize, mtk_i2s_dmamap_cb, ch, BUS_DMA_NOWAIT);

	if (error != 0) {
		device_printf(sc->dev, "cannot load DMA map\n");
		return NULL;
	}

	memset(ch->dmaaddr, 0, sc->dmasize);

	if (sndbuf_setup(b, ch->dmaaddr, sc->dmasize) != 0) {
		device_printf(sc->dev, "cannot setup sndbuf\n");
		return (NULL);
	}

	return (ch);
}

static void
mtk_i2s_dma_config(struct mtk_i2s_channel *ch)
{
	struct gdma_config conf;

	memset(&conf, 0, sizeof(conf));
	conf.burst_len = 4;

	if (ch->pcm_dir == PCMDIR_PLAY) {
		conf.dst_noincr = true;
		conf.src_dma_req = MEM_REQ;
		conf.dst_dma_req = ch->parent->txdma_req;
	} else {
		conf.src_noincr = true;
		conf.src_dma_req = ch->parent->rxdma_req;
		conf.dst_dma_req = MEM_REQ;
		conf.coherent = true;
	}

	MTK_GDMA_SET_CONFIG(ch->dmac, ch->dmachan, &conf);
}

static void
mtk_i2s_dma_transfer(struct mtk_i2s_channel *ch)
{
	struct snd_dbuf *buf;
	uint32_t blocksize;
	int error;
	bus_addr_t src, dst;

	if (ch->pcm_dir == PCMDIR_PLAY) {
		src = ch->physaddr + ch->pos;
		dst = ch->fifo;
	} else {
		src = ch->fifo;
		dst = ch->physaddr + ch->pos;
	}

	buf = ch->buf;
	blocksize = sndbuf_getblksz(buf);
	error = MTK_GDMA_TRANSFER(ch->dmac, ch->dmachan, src, dst, blocksize);
	if (error) {
		ch->run = 0;
		device_printf(ch->parent->dev, "DMA transfer failed: %d\n",
		    error);
	}
}

static void
mtk_i2s_dma_intr(void *priv)
{
	struct mtk_i2s_channel *ch = priv;
	struct snd_dbuf *buf;
	unsigned bufsize;
	uint32_t blocksize;

	buf = ch->buf;
	bufsize = sndbuf_getsize(buf);
	blocksize = sndbuf_getblksz(buf);

	ch->pos += blocksize;
	if (ch->pos >= bufsize)
		ch->pos -= bufsize;

	if (ch->run) {
		chn_intr(ch->pcm);
		mtk_i2s_dma_transfer(ch);
	}
}

static int
mtk_i2s_dma_free(device_t dev, int pcm_dir)
{
	struct mtk_i2s_softc *sc;
	struct mtk_i2s_channel *ch;

	sc = device_get_softc(dev);
	ch = pcm_dir == PCMDIR_PLAY ? &sc->play_channel : &sc->rec_channel;

	MTK_GDMA_FREE(ch->dmac, ch->dmachan);
	bus_dmamap_unload(sc->dmat, ch->dmamap);
	bus_dmamem_free(sc->dmat, ch->dmaaddr, ch->dmamap);

	return (0);
}

/*TODO
 * MT76[2|8]8 is supported MTK_I2S_CFG0_DATA_24 and MTK_I2S_CFG0_NORM_24
 */
static uint32_t
mtk_i2s_dai_set_chanformat(device_t dev, uint32_t format)
{
	if (format != SND_FORMAT(AFMT_S16_LE, 2, 0))
		return (EINVAL);

	return (0);
}

static int
mtk_i2s_dai_set_sysclk(device_t dev, unsigned int rate, int dai_dir)
{
	struct mtk_i2s_softc *sc;
	uint32_t val;

	sc = device_get_softc(dev);

	/* set refclk to 12M */
	val = mtk_sysctl_get(SYSCTL_CLKCFG0);
	val &= ~(0x7 << 9);
	val |= 0x01 << 9;

	if (sc->socid == MTK_SOC_RT3352)
		val |= 0x1 << 8;

	mtk_sysctl_set(SYSCTL_CLKCFG0, val);

	/* Enable MT7628 REFCLK digtal pad */
	if (sc->socid == MTK_SOC_MT7628) {
		val = mtk_sysctl_get(SYSCTL_AGPIOCFG);
		val |= 0x10;
		mtk_sysctl_set(SYSCTL_AGPIOCFG, val);

		/* Adjust REFCLK0's driving strength */
		val = mtk_sysctl_get(0x1354);
		val &= ~(0x1 << 5);
		mtk_sysctl_set(0x1354, val);
		val = mtk_sysctl_get(0x1364);
		val |= ~(0x1 << 5);
		mtk_sysctl_set(0x1364, val);
	}
	return (0);
}

static uint32_t
mtk_i2s_dai_set_chanspeed(device_t dev, uint32_t speed)
{
	struct mtk_i2s_softc *sc;
	sc = device_get_softc(dev);
	if (sc->master)
		mtk_i2s_setup_bclk(sc, speed);

	return (speed);
}

static device_method_t mtk_i2s_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		mtk_i2s_probe),
	DEVMETHOD(device_attach,	mtk_i2s_attach),
	DEVMETHOD(device_detach,	mtk_i2s_detach),

	DEVMETHOD(audio_dai_chan_init,	mtk_i2s_dma_init),
	DEVMETHOD(audio_dai_chan_free,	mtk_i2s_dma_free),

	DEVMETHOD(audio_dai_init,	mtk_i2s_dai_init),
	DEVMETHOD(audio_dai_set_sysclk,	mtk_i2s_dai_set_sysclk),
	DEVMETHOD(audio_dai_set_chanspeed,	mtk_i2s_dai_set_chanspeed),
	DEVMETHOD(audio_dai_set_chanformat,	mtk_i2s_dai_set_chanformat),
	DEVMETHOD(audio_dai_get_caps,	mtk_i2s_dai_get_caps),
	DEVMETHOD(audio_dai_trigger,	mtk_i2s_dai_trigger),
	DEVMETHOD(audio_dai_get_ptr,	mtk_i2s_dai_get_ptr),

	DEVMETHOD_END
};

static driver_t mtk_i2s_driver = {
	"i2s",
	mtk_i2s_methods,
	sizeof(struct mtk_i2s_softc),
};

static devclass_t mtk_i2s_devclass;

DRIVER_MODULE(mtk_i2s, simplebus, mtk_i2s_driver, mtk_i2s_devclass, 0, 0);
SIMPLEBUS_PNP_INFO(compat_data);
