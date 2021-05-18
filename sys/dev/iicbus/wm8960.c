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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 * Driver for Wolfson Microelectronics wm8960 audio codec
 */

#include "opt_platform.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/endian.h>

#ifdef FDT
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#endif

#include <dev/extres/clk/clk.h>

#include <dev/iicbus/iiconf.h>
#include <dev/iicbus/iicbus.h>

#include <dev/sound/fdt/audio_dai.h>
#include <dev/sound/pcm/sound.h>

#include "iicbus_if.h"
#include "mixer_if.h"
#include "audio_dai_if.h"

#include "wm8960.h"

#include <dev/fdt/fdt_common.h>

#define	WM8960_MIXER_DEVS (SOUND_MASK_VOLUME | SOUND_MASK_SPEAKER \
		| SOUND_MASK_PCM | SOUND_MASK_SPEAKER | SOUND_MASK_LINE \
		| SOUND_MASK_MIC | SOUND_MASK_IMIX | SOUND_MASK_RECLEV \
		| SOUND_MASK_IGAIN | SOUND_MASK_OGAIN | SOUND_MASK_MONITOR)

struct wm8960_softc {
	device_t	dev;
	device_t	busdev;
	struct intr_config_hook init_hook;
	clk_t		sc_mclk;
 	uint16_t	*wm8960_reg;
	int		spkd_op;
	int 		depth_3d;
};

#ifdef FDT
static struct ofw_compat_data compat_data[] = {
	{"wlf,wm8960",  1},
	{NULL,          0},
};
#endif

/* address is 7 bit lenght and data is 9 bit lenght */

static inline int
wm8960_write(struct wm8960_softc *sc, uint8_t reg, uint16_t val)
{
	uint8_t iic_addr;
	uint8_t iic_data;
	uint16_t data;
	int error;

	// device_printf(sc->dev, "%s reg 0x%x value 0x%x\n",
	//     __func__, reg, val);

	data = reg << 9 | val;
	iic_addr = data >> 8;
	iic_data = data & 0xFF;

	error = iicdev_writeto(sc->dev, iic_addr, &iic_data, 1, IIC_WAIT);
	if (error)
		return error;

	sc->wm8960_reg[reg] = val;

	return 0;
}

static inline int
wm8960_read(struct wm8960_softc *sc, uint8_t reg)
{
	return sc->wm8960_reg[reg];
}

static int
wm8960_mixer_init(struct snd_mixer *m)
{
	struct wm8960_softc *sc;

	sc = device_get_softc(mix_getdevinfo(m));
	mix_setdevs(m, WM8960_MIXER_DEVS);

	return (0);
}

static int
wm8960_mixer_uninit(struct snd_mixer *m)
{

	return (0);
}

static int
wm8960_mixer_reinit(struct snd_mixer *m)
{

	return (0);
}

static int
wm8960_mixer_set(struct snd_mixer *m, unsigned dev, unsigned left, unsigned right)
{
	struct wm8960_softc *sc;
	struct mtx *mixer_lock;
	int locked;
	uint16_t val;
			int boost;

	sc = device_get_softc(mix_getdevinfo(m));
	mixer_lock = mixer_get_lock(m);
	locked = mtx_owned(mixer_lock);

	if (locked)
		mtx_unlock(mixer_lock);

	switch (dev) {
	case SOUND_MIXER_VOLUME:
		val = 0x2F + (left * 0x50 / 100);
		wm8960_write(sc,R2, R2_OUT1VU | val);

		val = 0x2F + (right * 0x50 / 100);
		wm8960_write(sc,R3, R3_OUT1VU | val);

		return (left | (right << 8));

	case SOUND_MIXER_PCM:
		val = (left * 0xFF / 100);
		wm8960_write(sc,R10, R10_DACVU | val);

		val = wm8960_read(sc, R34);
		if (left == 0)
			val &= ~(R34_LD2LO);
		else
			val |= R34_LD2LO;
		wm8960_write(sc, R34, val);

		val = (right * 0xFF / 100);
		wm8960_write(sc,R11, R11_DACVU | val);

		val = wm8960_read(sc, R37);
		if (right == 0)
			val &= ~(R37_RD2RO);
		else
			val |= R37_RD2RO;
		wm8960_write(sc, R37, val);

		return (left | (right << 8));

	case SOUND_MIXER_SPEAKER:
		val = 0x2F + (left * 0x50 / 100);
		wm8960_write(sc,R40, R40_SPKVU | val);

		val = 0x2F + (right * 0x50 / 100);
		wm8960_write(sc,R41, R41_SPKVU | val);

		return (left | (right << 8));

	case SOUND_MIXER_LINE:
		val = wm8960_read(sc,R43);
		val &= ~(0x70);
		val |= (left * 0x7 / 100) << 4;
		wm8960_write(sc, R43, val);

		val = wm8960_read(sc,R44);
		val &= ~(0x70);
		val |= (right * 0x7 / 100) << 4;
		wm8960_write(sc, R44, val);

		return (left | (right << 8));

	case SOUND_MIXER_MIC:
		val = wm8960_read(sc,R32);
		val &= ~(R32_LMIC2B | 0x38);
		if (left != 0)
			val |= R32_LMIC2B;
		val |= (left * 0x3 / 100) << 4;
		wm8960_write(sc, R32, val);

		val = wm8960_read(sc,R33);
		val &= ~(R33_RMIC2B | 0x38);
		if (right != 0)
			val |= R33_RMIC2B;
		val |= (right * 0x3 / 100) << 4;
		wm8960_write(sc, R33, val);

		return (left | (right << 8));

	case SOUND_MIXER_IMIX:
		val = ((~(left * 0x07 / 100)) & 0x07);
		val = val << 4;
		if (left != 0)
			val |= R45_LB2LO;
		wm8960_write(sc, R45, val);

		val = ((~(right * 0x07 / 100)) & 0x07);
		val = val << 4;
		if (right != 0)
			val |= R46_RB2RO;
		wm8960_write(sc, R46,val);

		return (left | (right << 8));

	case SOUND_MIXER_RECLEV:
		val = (left * 255 / 100);
		wm8960_write(sc,R21, R21_ADCVU | val);

		val = (right * 255 / 100);
		wm8960_write(sc,R22, R21_ADCVU | val);

		return (left | (right << 8));

	case SOUND_MIXER_IGAIN:
		val = wm8960_read(sc, R34);
		val &= ~(R34_LI2LO | 0x70);
		val |= (left * 0x7 / 100) << 4;
		if (left)
			val |= R34_LI2LO;
		wm8960_write(sc, R34, val);

		val = wm8960_read(sc, R37);
		val &= ~(R37_RI2RO | 0x70);
		val |= (right * 0x7 / 100) << 4;
		if (right)
			val |= R37_RI2RO;
		wm8960_write(sc, R37,val);

		return (left | (right << 8));

	case SOUND_MIXER_OGAIN:
		boost = (left * 0x5/ 100);

		val = wm8960_read(sc, R51);
		val &=  ~(0x3F) ;
		val = ((boost << 3) | boost);

		wm8960_write(sc, R51, val);

		return (left | (left << 8));

	case SOUND_MIXER_MONITOR:
		val = (left * 0x3F / 100);
		if (left == 0)
			val |= R0_LINMUTE;
		device_printf(sc->dev,"%s val %i\n", __func__, val);
		wm8960_write(sc, R0, R0_IPVU | val);

		val = (right * 0x3F / 100);
		if (left == 0)
			val |= R1_RINMUTE;
		device_printf(sc->dev,"%s val %i\n", __func__, val);
		wm8960_write(sc, R1, R1_IPVU | val);

		return (left | (right << 8));

	default:
		break;
	}

	if (locked)
		mtx_lock(mixer_lock);

	return (0);
}

static u_int32_t
wm8960_mixer_setrecsrc(struct snd_mixer *m, u_int32_t src)
{

	return (0);
}

static kobj_method_t wm8960_mixer_methods[] = {
	KOBJMETHOD(mixer_init, 		wm8960_mixer_init),
	KOBJMETHOD(mixer_uninit, 	wm8960_mixer_uninit),
	KOBJMETHOD(mixer_reinit, 	wm8960_mixer_reinit),
	KOBJMETHOD(mixer_set, 		wm8960_mixer_set),
	KOBJMETHOD(mixer_setrecsrc, 	wm8960_mixer_setrecsrc),
	KOBJMETHOD_END
};

MIXER_DECLARE(wm8960_mixer);

uint16_t wm8960_default_reg[56] = {
	0x0097,0x0097,0x0000,0x0000,0x0000,0x0008,0x0000,0x000a,0x01c0,
	0x0000,0x00ff,0x00ff,0x0000,0x0000,0x0000,0x0000,0x0000,0x007b,
	0x0100,0x0032,0x0000,0x00c3,0x00c3,0x01c0,0x0000,0x0000,0x0000,
	0x0000,0x0000,0x0000,0x0000,0x0000,0x0100,0x0100,0x0050,0x0050,
	0x0050,0x0050,0x0000,0x0000,0x0000,0x0000,0x0040,0x0000,0x0000,
	0x0050,0x0050,0x0000,0x0002,0x0037,0x004d,0x0080,0x0008,0x0031,
	0x0026,0x00e9
};

static void
wm8960_init(void *arg)
{
	struct wm8960_softc *sc;

	sc = (struct wm8960_softc*)arg;
	config_intrhook_disestablish(&sc->init_hook);

}

static int
wm8960_probe(device_t dev)
{

#ifdef FDT
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data != 0) {
		device_set_desc(dev, "Wolfson Microelectronics WM8960 codec");
		return (BUS_PROBE_DEFAULT);
	}
#endif
	return (ENXIO);
}

static int
wm8960_set_gpio(struct wm8960_softc *sc, phandle_t node) {
	uint16_t val;
	pcell_t gpio[2];
	if (OF_getencprop(node, "wlf,gpio-cfg", gpio, sizeof(gpio)) != sizeof(gpio)) {
		device_printf(sc->dev,
		    "cannot get ALRCGPIO and GPIOPOL:GPIOSEL properties\n");
		return  (EINVAL);
	}

	device_printf(sc->dev, "ALRCGPIO 0x%x {GPIOPOL:GPIOSEL[2:0]} 0x%x\n",
	    gpio[0], gpio[1]);

	val = wm8960_read(sc, R9);
	val &= ~(R9_ALRCGPIO);
	val |= (uint16_t)gpio[0] & R9_ALRCGPIO;
	wm8960_write(sc, R9, val);

	val = wm8960_read(sc, R48);
	val &= ~(R48_GPIOPOL | R48_GPIOSEL_MASK);
	val |= (uint16_t)gpio[1] & (R48_GPIOPOL | R48_GPIOSEL_MASK);
	wm8960_write(sc, R48, val);
	return (0);
}

static int
wm8960_set_hp(struct wm8960_softc *sc, phandle_t node) {
	uint16_t val;
	pcell_t hp[3];
	if (OF_getencprop(node, "wlf,hp-cfg", hp, sizeof(hp)) != sizeof(hp)) {
		device_printf(sc->dev,
		    "cannot get HPSEL, HPSWEN:HPSWPOL and TOCLKSEL:TOEN properties\n");
		return  (EINVAL);
	}

	device_printf(sc->dev,
	    "HPSEL 0x%x {HPSWEN:HPSWPOL} 0x%x {TOCLKSEL:TOEN} 0x%x\n",
	    hp[0], hp[1], hp[2]);

	val = wm8960_read(sc, R48);
	val &= ~(R48_HPSEL_MASK);
	val |= (uint16_t)hp[0] & R48_HPSEL_MASK;
	wm8960_write(sc, R48, val);

	val = wm8960_read(sc, R24);
	val &= ~(R24_HPSWEN | R24_HPSWPOL);
	val |= (uint16_t)hp[1] & (R24_HPSWEN | R24_HPSWPOL);
	wm8960_write(sc, R24, val);

	val = wm8960_read(sc, R23);
	val &= ~(R23_TOCLKSEL | R23_TOEN);
	val |= (uint16_t)hp[2] & (R23_TOCLKSEL | R23_TOEN);
	wm8960_write(sc, R23, val);

	return (0);
}

static int
wm8960_set_spkd_op(struct wm8960_softc *sc, int spk_op)
{
	uint16_t val;
	sc->spkd_op = spk_op;

	if (spk_op == 0)
		val = R49_SPK_OP_EN_OFF;
	else if (spk_op == 1)
		val = R49_SPK_OP_EN_LSPK;
	else if (spk_op == 2)
		val = R49_SPK_OP_EN_RSPK;
	else if (spk_op == 3)
		val = R49_SPK_OP_EN_LRSPK;
	else
		return (EINVAL);

	wm8960_write(sc, R49, 0x0037 | val);

	return (0);
}

static int
wm8960_set_depth_3d(struct wm8960_softc *sc, int depth_3d)
{
	uint16_t val;

	if (depth_3d < 0x00 || depth_3d > 0x0F)
		return (EINVAL);

	sc->depth_3d = depth_3d;

	val = wm8960_read(sc, R16);
	val &=  ~(R16_3DEN | 0x0F) ;

	if (depth_3d > 0x00)
		val = ((depth_3d << 1) | R16_3DEN);

	/* TODO Upper Cut-Off Frequency and Lower Cut-Off Frequency */
	/* DAC 6dB attenuate enable */

	wm8960_write(sc, R16, val);

	return (0);
}

static int
wm8960_sysctl_audio_spkd_op(SYSCTL_HANDLER_ARGS)
{
	struct wm8960_softc *sc = arg1;
	int val;
	int err;

	val = sc->spkd_op;
	err = sysctl_handle_int(oidp, &val, 0, req);
	if (err || !req->newptr) /* error || read request */
		return (err);

	return wm8960_set_spkd_op(sc, val);
}

static int
wm8960_sysctl_audio_depth_3d(SYSCTL_HANDLER_ARGS)
{
	struct wm8960_softc *sc = arg1;
	int val;
	int err;

	val = sc->depth_3d;
	err = sysctl_handle_int(oidp, &val, 0, req);
	if (err || !req->newptr) /* error || read request */
		return (err);

	return wm8960_set_depth_3d(sc, val);
}

static int
wm8960_attach(device_t dev)
{
	struct wm8960_softc *sc;
	phandle_t node;
	int error;
	uint16_t val;
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid *tree_node;
	struct sysctl_oid_list *tree;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->busdev = device_get_parent(sc->dev);
	node = ofw_bus_get_node(dev);

	error = clk_get_by_ofw_name(dev, 0, "mclk", &sc->sc_mclk);
	if (error != 0) {
		device_printf(dev, "cannot get mclk clock\n");
		goto fail;
	}

	/*
	 * Wait until i2c is ready to set up the chip
	 */
	sc->init_hook.ich_func = wm8960_init;
	sc->init_hook.ich_arg = sc;
	if (config_intrhook_establish(&sc->init_hook) != 0) {
		error = ENOMEM;
		goto fail;
	}

	sc->wm8960_reg = wm8960_default_reg;

	/* Reset codec */
	wm8960_write(sc, R15, 0);

	/* ADCLRC AND DACLRC ENABLE */
	if (OF_hasprop(node, "wlf,shared-lrclk"))
		wm8960_write(sc, R24, R24_LRCM);

	if (OF_hasprop(node, "wlf,gpio-cfg")) {
		error = wm8960_set_gpio(sc, node);
		if (error)
			goto fail;
	}

	if (OF_hasprop(node, "wlf,hp-cfg")) {
		error = wm8960_set_hp(sc, node);
		if (error)
			goto fail;
	}

	/* Turn off software mute */
	wm8960_write(sc, R5, 0x00);

	/* Enable output mixers and input PGA */
	val = R47_LOMIX | R47_ROMIX | R47_LMIC | R47_RMIC;
	wm8960_write(sc, R47, val);

	/* Enableing outputs */
	val = R26_LOUT1|R26_ROUT1|R26_SPKL|R26_SPKR;
	wm8960_write(sc, R26, val);

	/* Enable Vmid playback and record VMIDSEL = 0x80
	 * and for all other functions.
	 * Analogue Input PGA and Boost and mic bias.
	 */
	wm8960_write(sc, R25, 0x80 | R25_VREF|R25_AINL|R25_AINR|R25_MICB);
	DELAY(100000);

	/* Connect INPUT1 to inverting input of Input PGA */
	wm8960_write(sc, R32, R32_LMN1);
	wm8960_write(sc, R33, R33_RMN1);

	OF_device_register_xref(OF_xref_from_node(node), dev);

	ctx = device_get_sysctl_ctx(sc->dev);
	tree_node = device_get_sysctl_tree(sc->dev);
	tree = SYSCTL_CHILDREN(tree_node);
	SYSCTL_ADD_PROC(ctx, tree, OID_AUTO, "spkdop",
		CTLFLAG_RW | CTLTYPE_UINT | CTLFLAG_NEEDGIANT, sc, sizeof(*sc),
		wm8960_sysctl_audio_spkd_op, "IU", "Class D Speaker Outputs, "
		 "0 - Off, 1 - Left speaker only, 2 - Right speaker only, "
		 "3 - Left and right speakers enabled");

	SYSCTL_ADD_PROC(ctx, tree, OID_AUTO, "3ddepth",
		CTLFLAG_RW | CTLTYPE_UINT | CTLFLAG_NEEDGIANT, sc, sizeof(*sc),
		wm8960_sysctl_audio_depth_3d, "IU", "3D Stereo Depth, "
		 "0 - Off , 1 - 16 3D effect depth of step 6.67%");

	return (0);

	fail:
        if (sc->sc_mclk != NULL)
                clk_release(sc->sc_mclk);

	return (ENXIO);
}

static int
wm8960_detach(device_t dev)
{
	struct wm8960_softc *sc;

	sc = device_get_softc(dev);

	if (sc->sc_mclk != NULL)
		clk_release(sc->sc_mclk);

	return (0);
}

static int
wm8960_dai_init(device_t dev, uint32_t format)
{
	struct wm8960_softc *sc;
	uint16_t val;
	int fmt, pol, clk;
	int error;

	sc = (struct wm8960_softc*)device_get_softc(dev);

	fmt = AUDIO_DAI_FORMAT_FORMAT(format);
	pol = AUDIO_DAI_FORMAT_POLARITY(format);
	clk = AUDIO_DAI_FORMAT_CLOCK(format);

	/* R7 Right Justified mode does not support 32-bit data.
	 * Audio Data Word Length is forced 16bits. deafult is 24-bit.
	 * TODO Find to method how to change it otherway.
	 */
	val = R7_WL_16;

	switch (clk) {
	case AUDIO_DAI_CLOCK_CBM_CFM:
		val |= R7_MS;
		break;
	case AUDIO_DAI_CLOCK_CBS_CFS:
		break;
	default:
		return (EINVAL);
	}

	switch (pol) {
	case AUDIO_DAI_POLARITY_NB_NF:
		break;
	case AUDIO_DAI_POLARITY_NB_IF:
		val |= R7_LRP;
		break;
	case AUDIO_DAI_POLARITY_IB_NF:
		val |= R7_BCLKINV;
		break;
	case AUDIO_DAI_POLARITY_IB_IF:
		val |= R7_BCLKINV;
		val |= R7_LRP;
		break;
	}

	switch (fmt) {
	case AUDIO_DAI_FORMAT_I2S:
		val |= R7_FORMAT_I2S;
		break;
	case AUDIO_DAI_FORMAT_LJ:
		val |= R7_FORMAT_LJ;
		break;
	case AUDIO_DAI_FORMAT_RJ:
		val |= R7_FORMAT_RJ;
		break;
	case AUDIO_DAI_FORMAT_DSPA:
		val |= R7_FORMAT_DSP;
		break;
	case AUDIO_DAI_FORMAT_DSPB:
		val |= R7_LRP;
		val |= R7_FORMAT_DSP;
		break;
	default:
		return EINVAL;
	}

	error = wm8960_write(sc, R7, val);
	return (error);
}

static int
wm8960_dai_setup_mixer(device_t dev, device_t pcmdev)
{

	mixer_init(pcmdev, &wm8960_mixer_class, dev);

	return (0);
}

/* Setup system clock via pll. Supported mclk is 8,192MHz to 32,768MHz */
static int
wm8960_sysclk_pll(struct wm8960_softc* sc, unsigned int mclk,
    unsigned int sysclk)
{
	uint32_t val;
	uint16_t clocking = R4_CLKSEL | R4_SYSCLKDIV_SYSCLK_2;
	uint16_t plln = R52_SDM;
	int r;
	uint32_t k;

	/* sysclk is equal = f2 / 4 / SYSCLKDIV. SYSCLKDIV can be 1 or 2 */
	sysclk *= 4 * 2;

	/* The PLL performs best when f2 is between 90MHz and 100MHz.
	 * Its stability peaks at r == 8
	 */
        r = sysclk / mclk;
	if (r < 6) {
		mclk >>= 1;
		plln |= R52_PLLPRESCALE;
		r = sysclk / mclk;
	}

	if ((r < 6) || (r > 12)) {
		device_printf(sc->dev,"Unsupported PLL divaider: %d\n", r);
		return EINVAL;
	}

	k = (unsigned long long)(sysclk % mclk) * (1 << 24) / mclk;

	wm8960_write(sc, R4, clocking);
	wm8960_write(sc, R52, plln | r);
	wm8960_write(sc, R53, (k >> 16) & 0xff);
	wm8960_write(sc, R54, (k >> 8) & 0xff);
	wm8960_write(sc, R55, (k & 0xff));

	val = wm8960_read(sc, R26);
	val |= R26_PLL_EN;
	wm8960_write(sc, R26, val);

	return (0);
}

/* Use sysclt speed as rate speed 11.2896MHz or 12.288MHz */
static int
wm8960_dai_set_sysclk(device_t dev, unsigned int rate, int dai_dir)
{
	struct wm8960_softc* sc;
	int error;
	uint64_t mclk;

	sc = device_get_softc(dev);

	if (!(rate == 11289600 || rate == 12288000)) {
		device_printf(sc->dev,"unsupported rate %d MHz\n",
		     rate / 1000000);
		return (EINVAL);
	}

	error = clk_get_freq(sc->sc_mclk, &mclk);
	if (error != 0) {
		device_printf(sc->dev,
		    "failed to get mclk frequency: err=%d\n", error);
		return (error);
	}

	error = wm8960_sysclk_pll(sc, mclk, rate);
	if (error != 0)
		return (error);

	/* Class D clocks set to an appropriate value to produce a class D
	 * clock of between 700kHz and 800kHz for best performance.
	 * Setup BCLK divaid 4.
	 */
	wm8960_write(sc, R8, R8_DCLKDIV_SYSCLK_16 | R8_BCLKDIV_SYSCLK_4);

	/*TODO ADCDIV, DACDIV, OPCLKDIV and TOCLKSEL */

	return (0);
}

static int
wm8960_dai_trigger(device_t dev, int go, int pcm_dir)
{
	struct wm8960_softc* sc;
	int val;
	sc = (struct wm8960_softc*)device_get_softc(dev);
	switch (go) {
	case PCMTRIG_START:
		if (pcm_dir == PCMDIR_PLAY) {
			/* enable DAC */
			val = wm8960_read(sc, R26);
			val |= R26_DACL|R26_DACR;
			wm8960_write(sc, R26, val);
		 } else if (pcm_dir == PCMDIR_REC){
		 	/* enable ADC */
		 	val = wm8960_read(sc, R25);
		 	val |= R25_ADCR|R25_ADCL;
		 	wm8960_write(sc, R25, val);
		}
		break;

	case PCMTRIG_STOP:
	case PCMTRIG_ABORT:
		if (pcm_dir == PCMDIR_PLAY) {
			/* disable DAC */
			val = wm8960_read(sc, R26);
			val &= ~(R26_DACL|R26_DACR);
			wm8960_write(sc, R26, val);
		 } else if (pcm_dir == PCMDIR_REC){
		 	/* disable ADC */
		 	val = wm8960_read(sc, R25);
		 	val &= ~(R25_ADCR|R25_ADCL);
		 	wm8960_write(sc, R25, val);
		}
		break;
	}
	return (0);
}

static device_method_t wm8960_methods[] = {
	/* device_if methods */
	DEVMETHOD(device_probe,		wm8960_probe),
	DEVMETHOD(device_attach,	wm8960_attach),
	DEVMETHOD(device_detach,	wm8960_detach),

	DEVMETHOD(audio_dai_init,	wm8960_dai_init),
	DEVMETHOD(audio_dai_setup_mixer,	wm8960_dai_setup_mixer),
	DEVMETHOD(audio_dai_trigger,	wm8960_dai_trigger),
	DEVMETHOD(audio_dai_set_sysclk,	wm8960_dai_set_sysclk),

	DEVMETHOD_END,
};

static driver_t wm8960_driver = {
	"wm8960codec",
	wm8960_methods,
	sizeof(struct wm8960_softc),
};

static devclass_t wm8960_devclass;

DRIVER_MODULE(wm8960codec, iicbus, wm8960_driver, wm8960_devclass, NULL, NULL);
MODULE_VERSION(wm8960codec, 1);
MODULE_DEPEND(wm8960codec, iicbus, IICBUS_MINVER, IICBUS_PREFVER, IICBUS_MAXVER);
IICBUS_FDT_PNP_INFO(compat_data);
