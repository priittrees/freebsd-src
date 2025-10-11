/*-;
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 - 2025 Priit Trees <trees@neti.ee>
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
 */
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>

#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/gpio.h>
#include <sys/proc.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <machine/intr.h>

#include <dev/gpio/gpiobusvar.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/fdt/fdt_pinctrl.h>

#include <arm64/mediatek/mediatek_pinctrl.h>
#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>
#include <dev/regulator/regulator.h>

#include <dt-bindings/pinctrl/mt65xx.h>

#include <dev/syscon/syscon.h>

#include "syscon_if.h"
#include "pic_if.h"
#include "gpio_if.h"

#if 0
#define dprintf(format, arg...) printf("%s: " format, __func__, arg)
#else
#define dprintf(format, arg...)
#endif

#define	MTK_GPIO_DEFAULT_CAPS	(GPIO_PIN_INPUT | GPIO_PIN_OUTPUT |	\
	  GPIO_PIN_PULLUP | GPIO_PIN_PULLDOWN);

#if 0
#define	MTK_GPIO_INTR_CAPS	(GPIO_INTR_LEVEL_LOW | GPIO_INTR_LEVEL_HIGH |	\
	  GPIO_INTR_EDGE_RISING | GPIO_INTR_EDGE_FALLING | GPIO_INTR_EDGE_BOTH)
#endif

#define	MTK_GPIO_NONE		0
#define	MTK_GPIO_PULLUP		1
#define	MTK_GPIO_PULLDOWN	2

#define	MTK_GPIO_INPUT		0
#define	MTK_GPIO_OUTPUT		1

struct mtk_gpio_conf {
	struct mtk_padconf *padconf;
};

extern struct mtk_padconf mt7622_padconf;

struct mtk_gpio_conf mt7622_gpio_conf = {
	.padconf = &mt7622_padconf,
};

static struct ofw_compat_data compat_data[] = {
	{"mediatek,mt7622-pinctrl",		(uintptr_t)&mt7622_gpio_conf},
	{NULL,					0}
};

struct clk_list {
	TAILQ_ENTRY(clk_list)	next;
	clk_t			clk;
};

struct gpio_irqsrc {
	struct intr_irqsrc	isrc;
	u_int			irq;
	uint32_t		mode;
	uint32_t		pin;
	uint32_t		intnum;
	uint32_t		intfunc;
	uint32_t		oldfunc;
	bool			enabled;
};

#define	MTK_BASE_MEMRES		0
#define	MTK_EINT_MEMRES		1
#define	MTK_GPIO_IRQRES		2
#define	MTK_GPIO_RESSZ		3

struct mtk_gpio_softc {
	device_t		sc_dev;
	device_t		sc_busdev;
	struct resource *	sc_res[MTK_GPIO_RESSZ];
	struct mtx		sc_mtx;
	struct resource *	sc_mem_res;
	struct resource *	sc_irq_res;
	void *			sc_intrhand;
	struct mtk_gpio_conf	*conf;
	TAILQ_HEAD(, clk_list)	clk_list;

	struct gpio_irqsrc 	*gpio_pic_irqsrc;
	int			nirqs;
};

static struct resource_spec mtk_gpio_res_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_MEMORY,	1,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE | RF_SHAREABLE },
	{ -1,			0,	0 }
};

#define	MTK_BASE_READ_4(_sc, _reg)					\
	bus_read_4((_sc)->sc_res[MTK_BASE_MEMRES], _reg)
#define MTK_BASE_WRITE_4(_sc, _reg, _value)				\
	bus_write_4((_sc)->sc_res[MTK_BASE_MEMRES], _reg, _value)
#define	MTK_EINT_READ_4(_sc, _reg)					\
	bus_read_4((_sc)->sc_res[MTK_EINT_MEMRES], _reg)
#define MTK_EINT_WRITE_4(_sc, _reg, _value)				\
	bus_write_4((_sc)->sc_res[MTK_EINT_MEMRES], _reg, _value)
#define	MTK_GPIO_LOCK(_sc)		mtx_lock_spin(&(_sc)->sc_mtx)
#define	MTK_GPIO_UNLOCK(_sc)		mtx_unlock_spin(&(_sc)->sc_mtx)
#define	MTK_GPIO_LOCK_ASSERT(_sc)	mtx_assert(&(_sc)->sc_mtx, MA_OWNED)

static char *mtk_gpio_parse_function(phandle_t node);
static const char **mtk_gpio_parse_groups(phandle_t node, int *groups_nb);
static const char **mtk_gpio_parse_pins(phandle_t node, int *pins_nb);
static uint32_t mtk_gpio_parse_bias(phandle_t node);
static int mtk_gpio_parse_drive_strength(phandle_t node, uint32_t *drive);

static int mtk_gpio_pin_get(device_t dev, uint32_t pin, unsigned int *value);
static int mtk_gpio_pin_set(device_t dev, uint32_t pin, unsigned int value);
static int mtk_gpio_pin_get_locked(struct mtk_gpio_softc *sc, uint32_t pin, unsigned int *value);
static int mtk_gpio_pin_set_locked(struct mtk_gpio_softc *sc, uint32_t pin, unsigned int value);

static void mtk_gpio_intr(void *arg);
static void mtk_gpio_pic_disable_intr(device_t dev, struct intr_irqsrc *isrc);
static void mtk_gpio_pic_disable_intr_locked(struct mtk_gpio_softc *sc, struct intr_irqsrc *isrc);
static void mtk_gpio_pic_post_filter(device_t dev, struct intr_irqsrc *isrc);
static int mtk_gpio_register_isrcs(struct mtk_gpio_softc *sc);
static int mtk_field_lookup (struct mtk_gpio_softc *sc, int field,
    int pin, uint32_t *offset, uint32_t *mask, uint8_t *bitpos) {
	const struct mtk_pin_field_calc *c;
	const struct mtk_pin_reg_calc *rc;
	int start = 0, end, check;
	bool found = false;
	uint32_t bits;

        rc  = &sc->conf->padconf->pin_reg[field];
	if (rc == NULL) {
		device_printf(sc->sc_dev,
		    "Not support field %d for this soc\n", field);
		return (EINVAL);
	}

        end = rc->nranges - 1;

	while (start <= end) {
		check = (start + end) >> 1;
		if (pin >= rc->range[check].s_pin
		    && pin <= rc->range[check].e_pin) {
			found = true;
			break;
		}
		else if (start == end)
			break;
		else if (pin < rc->range[check].s_pin)
			end = check - 1;
		else
			start = check + 1;

	}

	if (!found) {
		device_printf(sc->sc_dev,
		    "Not support field %d for pin = %d\n",
		    field, pin);
		return (EINVAL);
	}

	c = &rc->range[check];

	bits = c->fixed ? c->s_bit : c->s_bit +
	    (pin - c->s_pin) * (c->x_bits);

	*offset = c->s_addr + c->x_addrs * (bits / c->sz_reg);
	*bitpos = bits % c->sz_reg;
	*mask = (1 << c->x_bits) - 1;

	return (0);
}

static uint32_t
mtk_gpio_get_function(struct mtk_gpio_softc *sc, uint32_t pin)
{
	uint32_t offset;
	uint32_t mask;
	uint32_t reg_data;
	uint8_t  bitpos;

	/* Must be called with lock held. */
	MTK_GPIO_LOCK_ASSERT(sc);

	if (pin > sc->conf->padconf->npins)
		return (0);

	mtk_field_lookup(sc, PINCTRL_PIN_REG_DIR, pin, &offset, &mask, &bitpos);
	reg_data = MTK_BASE_READ_4(sc, offset);
	dprintf("pin %i offset 0x%08x reg 0x%08x\n", pin, offset, reg_data);

	return ((reg_data & (mask << bitpos)) ?
		    MTK_GPIO_OUTPUT : MTK_GPIO_INPUT);
}

static int
mtk_gpio_set_function(struct mtk_gpio_softc *sc, uint32_t pin, uint32_t f)
{

	uint32_t offset;
	uint32_t mask;
	uint32_t reg_data;
	uint8_t  bitpos;

	/* Check if the function exists in the padconf data */
//TODO

	/* Must be called with lock held. */
	MTK_GPIO_LOCK_ASSERT(sc);

	mtk_field_lookup(sc, PINCTRL_PIN_REG_DIR, pin, &offset, &mask, &bitpos);
        reg_data = MTK_BASE_READ_4(sc, offset);
	dprintf("pin %i offset 0x%08x reg before set 0x%08x\n", pin, offset, reg_data);
	reg_data &= ~(mask<< bitpos);
	reg_data |= (f << bitpos);
	dprintf("pin %i offset 0x%08x reg set values 0x%08x\n", pin, offset, reg_data);
	MTK_BASE_WRITE_4(sc, offset, reg_data);

	return (0);
}

static int
mtk_gpio_get_pud(struct mtk_gpio_softc *sc, uint32_t pin)
{
	uint32_t offset;
	uint32_t mask;
	uint32_t reg_data;
	uint8_t  bitpos;
	int pu, pd;

	/* Must be called with lock held. */
	MTK_GPIO_LOCK_ASSERT(sc);

	mtk_field_lookup(sc, PINCTRL_PIN_REG_PD, pin, &offset, &mask, &bitpos);
	reg_data = MTK_BASE_READ_4(sc, offset);
	dprintf("pd pin %i offset 0x%08x reg 0x%08x\n", pin, offset, reg_data);
	pd  = (reg_data & (mask << bitpos)) ? 1 : 0;

	mtk_field_lookup(sc, PINCTRL_PIN_REG_PU, pin, &offset, &mask, &bitpos);
	reg_data = MTK_BASE_READ_4(sc, offset);
	dprintf("pu pin %i offset 0x%08x reg 0x%08x\n", pin, offset, reg_data);
	pu = (reg_data & (mask << bitpos)) ? 1 : 0;

	if (pu == 0 && pd == 0)
		return (MTK_GPIO_NONE);
	else if (pu == 1 && pd == 0)
		return (MTK_GPIO_PULLUP);
	else if (pu == 0 && pd == 1)
		return (MTK_GPIO_PULLDOWN);
	else
		return (EINVAL);
}

static void
mtk_gpio_set_pud(struct mtk_gpio_softc *sc, uint32_t pin, uint32_t state)
{
	uint32_t offset;
	uint32_t mask;
	uint32_t reg_data;
	uint8_t  bitpos;
	int pu = 0, pd = 0;

	if (mtk_gpio_get_pud(sc, pin) == state)
		return;

	/* Must be called with lock held. */
	MTK_GPIO_LOCK_ASSERT(sc);

	if (state == MTK_GPIO_NONE) {
		pu = 0;
		pd = 0;
	}
	else if (state == MTK_GPIO_PULLUP) {
		pu = 1;
		pd = 0;
	}
	else if (state == MTK_GPIO_PULLDOWN) {
		pu = 0;
		pd = 1;
	}

	mtk_field_lookup(sc, PINCTRL_PIN_REG_PU, pin, &offset, &mask, &bitpos);
	reg_data = MTK_BASE_READ_4(sc, offset);
	dprintf("pu pin %i offset 0x%08x reg before set 0x%08x\n", pin, offset, reg_data);
	reg_data &= ~(mask << bitpos);
	reg_data |= (pu << bitpos);
	dprintf("pu pin %i offset 0x%08x reg set values 0x%08x\n", pin, offset, reg_data);
	MTK_BASE_WRITE_4(sc, offset, reg_data);

	mtk_field_lookup(sc, PINCTRL_PIN_REG_PD, pin, &offset, &mask, &bitpos);
	dprintf("pd pin %i offset 0x%08x reg before set 0x%08x\n", pin, offset, reg_data);
        reg_data = MTK_BASE_READ_4(sc, offset);
	reg_data &= ~(mask << bitpos);
	reg_data |= (pd << bitpos);
	dprintf("pd pin %i offset 0x%08x reg set values 0x%08x\n", pin, offset, reg_data);
	MTK_BASE_WRITE_4(sc, offset, reg_data);
}

//TODO
/* mt7622 drive values.
 * 4mA when (e8, e4) = (0, 0)
 * 8mA when (e8, e4) = (0, 1)
 * 12mA when (e8, e4) = (1, 0)
 * 16mA when (e8, e4) = (1, 1)
 * Need to develop for other devices
 */
static uint32_t
mtk_gpio_get_drv(struct mtk_gpio_softc *sc, uint32_t pin)
{
	uint32_t offset;
	uint32_t mask;
	uint32_t reg_data;
	uint8_t  bitpos;
	int e8, e4;

	/* Must be called with lock held. */
	MTK_GPIO_LOCK_ASSERT(sc);

	mtk_field_lookup(sc, PINCTRL_PIN_REG_E4, pin, &offset, &mask, &bitpos);
        reg_data = MTK_BASE_READ_4(sc, offset);
        e4 = ((reg_data >> bitpos) & mask);
	dprintf("e4 pin %i offset 0x%08x reg 0x%08x\n", pin, offset, reg_data);
	printf ("%s e4 val 0x%x\n", __func__, e4);

	mtk_field_lookup(sc, PINCTRL_PIN_REG_E8, pin, &offset, &mask, &bitpos);
        reg_data = MTK_BASE_READ_4(sc, offset);
        e8 = ((reg_data >> bitpos) & mask);
	dprintf("e8 pin %i offset 0x%08x reg 0x%08x\n", pin, offset, reg_data);
	printf ("%s e8 val 0x%x\n", __func__, e8);

	if((e8 == 0) && (e4 == 0))
		return (4);
	else if	((e8 == 0) && (e4 == 1))
		return (8);
	else if	((e8 == 1) && (e4 == 0))
		return (12);
	else if((e8 == 1) && (e4 == 1))
		return (16);
	else
		return(EINVAL);
}

static void
mtk_gpio_set_drv(struct mtk_gpio_softc *sc, uint32_t pin, uint32_t drive)
{
	uint32_t offset;
	uint32_t mask;
	uint32_t reg_data;
	uint8_t  bitpos;
	int e8 = 0, e4 = 0;

	if (mtk_gpio_get_drv(sc, pin) == drive)
		return;

	/* Must be called with lock held. */
	MTK_GPIO_LOCK_ASSERT(sc);

	if (drive == 4) {
		e8 = 0; e4 = 0;
	}
	else if (drive == 8) {
		e8 = 0; e4 = 1;
	}
	else if (drive == 12) {
		e8 = 1; e4 = 0;
	}
	else if (drive == 16) {
		e8 = 1; e4 = 1;
	}
	else {
		return;
	}

	mtk_field_lookup(sc, PINCTRL_PIN_REG_E4, pin, &offset, &mask, &bitpos);
        reg_data = MTK_BASE_READ_4(sc, offset);
	dprintf("e4 pin %i offset 0x%08x reg before set 0x%08x\n", pin, offset, reg_data);
	printf ("%s e4 val 0x%x\n", __func__, (reg_data >> bitpos) & mask);
        reg_data &= ~(mask << bitpos);
        reg_data |= (e4 << bitpos);
	dprintf("e4 pin %i offset 0x%08x reg set values 0x%08x\n", pin, offset, reg_data);
        MTK_BASE_WRITE_4(sc, offset, reg_data);

	mtk_field_lookup(sc, PINCTRL_PIN_REG_E8, pin, &offset, &mask, &bitpos);
	dprintf("e8 pin %i offset 0x%08x reg before set 0x%08x\n", pin, offset, reg_data);
        reg_data = MTK_BASE_READ_4(sc, offset);
	printf ("%s e8 val 0x%x\n", __func__, (reg_data >> bitpos) & mask);
        reg_data &= ~(mask << bitpos);
        reg_data |= (e8 << bitpos);
	dprintf("e8 pin %i offset 0x%08x reg set values 0x%08x\n", pin, offset, reg_data);
        MTK_BASE_WRITE_4(sc, offset, reg_data);
}

static int
mtk_gpio_pin_configure(struct mtk_gpio_softc *sc, uint32_t pin, uint32_t flags)
{
	int err = 0;

	/* Must be called with lock held. */
	MTK_GPIO_LOCK_ASSERT(sc);

	if (pin > sc->conf->padconf->npins)
		return (EINVAL);

	/* Manage input/output. */
	if (flags & GPIO_PIN_INPUT) {
		err = mtk_gpio_set_function(sc, pin, MTK_GPIO_INPUT);
	} else if ((flags & GPIO_PIN_OUTPUT) &&
	    mtk_gpio_get_function(sc, pin) != MTK_GPIO_OUTPUT) {
		if (flags & GPIO_PIN_PRESET_LOW) {
			mtk_gpio_pin_set_locked(sc, pin, 0);
		} else if (flags & GPIO_PIN_PRESET_HIGH) {
			mtk_gpio_pin_set_locked(sc, pin, 1);
		} else {
//TODO
#if 0
			/* Read the pin and preset output to current state. */
			err = mtk_gpio_set_function(sc, pin, MTK_GPIO_INPUT);
			if (err == 0) {
				mtk_gpio_pin_get_locked(sc, pin, &val);
				mtk_gpio_pin_set_locked(sc, pin, val);

			}
#endif
		}
		if (err == 0)
			err = mtk_gpio_set_function(sc, pin, MTK_GPIO_OUTPUT);
	}

	if (err)
		return (err);

	/* Manage Pull-up/pull-down. */
	if (flags & GPIO_PIN_PULLUP)
		mtk_gpio_set_pud(sc, pin, MTK_GPIO_PULLUP);
	else if (flags & GPIO_PIN_PULLDOWN)
		mtk_gpio_set_pud(sc, pin, MTK_GPIO_PULLDOWN);
	else
		mtk_gpio_set_pud(sc, pin, MTK_GPIO_NONE);

	return (0);
}

static device_t
mtk_gpio_get_bus(device_t dev)
{
	struct mtk_gpio_softc *sc;

	sc = device_get_softc(dev);

	return (sc->sc_busdev);
}

static int
mtk_gpio_pin_max(device_t dev, int *maxpin)
{
	struct mtk_gpio_softc *sc;

	sc = device_get_softc(dev);

	*maxpin = sc->conf->padconf->npins - 1;
	return (0);
}

static int
mtk_gpio_pin_getcaps(device_t dev, uint32_t pin, uint32_t *caps)
{
	struct mtk_gpio_softc *sc;

	sc = device_get_softc(dev);
	if (pin >= sc->conf->padconf->npins)
		return (EINVAL);

	*caps = MTK_GPIO_DEFAULT_CAPS;

	return (0);
}

static int
mtk_gpio_pin_getflags(device_t dev, uint32_t pin, uint32_t *flags)
{
	struct mtk_gpio_softc *sc;
	int func;
	int pud;

	sc = device_get_softc(dev);

	if (pin >= sc->conf->padconf->npins)
		return (EINVAL);

	MTK_GPIO_LOCK(sc);
	func = mtk_gpio_get_function(sc, pin);
	switch (func) {
	case MTK_GPIO_INPUT:
		*flags = GPIO_PIN_INPUT;
		break;
	case MTK_GPIO_OUTPUT:
		*flags = GPIO_PIN_OUTPUT;
		break;
	default:
		*flags = 0;
		break;
	}

	pud = mtk_gpio_get_pud(sc, pin);
	switch (pud) {
	case MTK_GPIO_PULLDOWN:
		*flags |= GPIO_PIN_PULLDOWN;
		break;
	case MTK_GPIO_PULLUP:
		*flags |= GPIO_PIN_PULLUP;
		break;
	default:
		break;
	}

	MTK_GPIO_UNLOCK(sc);
	return (0);
}

static int
mtk_gpio_pin_getname(device_t dev, uint32_t pin, char *name)
{
	struct mtk_gpio_softc *sc;

	sc = device_get_softc(dev);
	if (pin >= sc->conf->padconf->npins)
		return (EINVAL);

	snprintf(name, GPIOMAXNAME - 1, "%s",
	    sc->conf->padconf->pins_names[pin].name);
	name[GPIOMAXNAME - 1] = '\0';

	return (0);
}

static int
mtk_gpio_pin_setflags(device_t dev, uint32_t pin, uint32_t flags)
{
	struct mtk_gpio_softc *sc;
	int err;

	sc = device_get_softc(dev);
	if (pin > sc->conf->padconf->npins)
		return (EINVAL);

	MTK_GPIO_LOCK(sc);
	err = mtk_gpio_pin_configure(sc, pin, flags);
	MTK_GPIO_UNLOCK(sc);

	return (err);
}

static int
mtk_gpio_pin_set_locked(struct mtk_gpio_softc *sc, uint32_t pin,
    unsigned int value)
{
	uint32_t reg_data;
	uint32_t offset;
	uint32_t mask;
	uint8_t  bitpos;

	MTK_GPIO_LOCK_ASSERT(sc);

	if (pin > sc->conf->padconf->npins)
		return (EINVAL);

	mtk_field_lookup(sc, PINCTRL_PIN_REG_DO, pin, &offset, &mask, &bitpos);
	reg_data = MTK_BASE_READ_4(sc, offset);
	dprintf("do pin %i offset 0x%08x reg before set 0x%08x\n", pin, offset, reg_data);

	if (value)
		reg_data |= (1 << bitpos);
	else
		reg_data &= ~(1 << bitpos);

	dprintf("do pin %i offset 0x%08x reg set values 0x%08x\n", pin, offset, reg_data);
	MTK_BASE_WRITE_4(sc, offset, reg_data);

	return (0);
}

static int
mtk_gpio_pin_set(device_t dev, uint32_t pin, unsigned int value)
{
	struct mtk_gpio_softc *sc;
	int ret;

	sc = device_get_softc(dev);

	MTK_GPIO_LOCK(sc);
	ret = mtk_gpio_pin_set_locked(sc, pin, value);
	MTK_GPIO_UNLOCK(sc);

	return (ret);
}

static int
mtk_gpio_pin_get_locked(struct mtk_gpio_softc *sc,uint32_t pin,
    unsigned int *val)
{
	uint32_t reg_data;
	uint32_t offset;
	uint32_t mask;
	uint8_t  bitpos;

	int dir;

	MTK_GPIO_LOCK_ASSERT(sc);

	if (pin > sc->conf->padconf->npins)
		return (EINVAL);

//TODO oehhhh need check dir. drop pin 1 printf
	dir = mtk_gpio_get_function(sc, pin);
	if (dir == MTK_GPIO_OUTPUT) {
		mtk_field_lookup(sc, PINCTRL_PIN_REG_DO,
		    pin, &offset, &mask, &bitpos);
		reg_data = MTK_BASE_READ_4(sc, offset);
		if (pin != 1) // cleaning the MMC passage.
		dprintf("do pin %i offset 0x%08x reg 0x%08x\n",
		    pin, offset, reg_data);
		*val = (reg_data & (mask << bitpos)) ? 1 : 0;
		}
	else {
		mtk_field_lookup(sc, PINCTRL_PIN_REG_DI,
		    pin, &offset, &mask, &bitpos);
		reg_data = MTK_BASE_READ_4(sc, offset);
		if (pin != 1) // cleaning the MMC passage.
		dprintf("di pin %i offset 0x%08x reg 0x%08x\n",
		    pin, offset, reg_data);
		*val = (reg_data & (mask << bitpos)) ? 1 : 0;
	}

	return (0);
}

static char *
mtk_gpio_parse_function(phandle_t node)
{
	char *function;

	if (OF_getprop_alloc(node, "function",
	    (void **)&function) != -1)
		return (function);

	return (NULL);
}

static const char **
mtk_gpio_parse_pins(phandle_t node, int *pins_nb)
{
	const char **pinlist;

	*pins_nb = ofw_bus_string_list_to_array(node, "pins", &pinlist);
	if (*pins_nb > 0)
		return (pinlist);

	return (NULL);
}

static const char **
mtk_gpio_parse_groups(phandle_t node, int *groups_nb)
{
	const char **grouplist;

	*groups_nb = ofw_bus_string_list_to_array(node, "groups", &grouplist);
	if (*groups_nb > 0)
		return (grouplist);

	return (NULL);
}

static uint32_t
mtk_gpio_parse_bias(phandle_t node)
{
	if (OF_hasprop(node, "bias-disable"))
		return (MTK_GPIO_NONE);
	if (OF_hasprop(node, "bias-pull-up"))
		return (MTK_GPIO_PULLUP);
	if (OF_hasprop(node, "bias-pull-down"))
		return (MTK_GPIO_PULLDOWN);

	return (MTK_GPIO_NONE);
}

static int
mtk_gpio_parse_drive_strength(phandle_t node, uint32_t *drive)
{
	uint32_t drive_str;

	if (OF_getencprop(node, "drive-strength", &drive_str,
	    sizeof(drive_str)) != -1) {
		*drive = drive_str;
		return (0);
	}

	return (1);
}

static int
mtk_gpio_pin_get(device_t dev, uint32_t pin, unsigned int *val)
{
	struct mtk_gpio_softc *sc;
	int ret;

	sc = device_get_softc(dev);

	MTK_GPIO_LOCK(sc);
	ret = mtk_gpio_pin_get_locked(sc, pin, val);
	MTK_GPIO_UNLOCK(sc);

	return (ret);
}

static int
mtk_gpio_pin_toggle(device_t dev, uint32_t pin)
{
	struct mtk_gpio_softc *sc;
	uint32_t reg_data;
	uint32_t offset;
	uint32_t mask;
	uint8_t  bitpos;

	sc = device_get_softc(dev);

	if (pin > sc->conf->padconf->npins)
		return (EINVAL);

	MTK_GPIO_LOCK(sc);
	mtk_field_lookup(sc, PINCTRL_PIN_REG_DO, pin, &offset, &mask, &bitpos);
	reg_data = MTK_BASE_READ_4(sc, offset);
	dprintf("do pin %i offset 0x%08x reg before set 0x%08x\n", pin, offset, reg_data);
	if (reg_data & (1 << bitpos))
		reg_data &= ~(1 << pin);
	else
		reg_data |= (1 << pin);

	dprintf("do pin %i offset 0x%08x reg set values 0x%08x\n", pin, offset, reg_data);
	MTK_BASE_WRITE_4(sc, offset, reg_data);
	MTK_GPIO_UNLOCK(sc);

	return (0);
}

static int
mtk_gpio_map_gpios(device_t bus, phandle_t dev, phandle_t gparent, int gcells,
    pcell_t *gpios, uint32_t *pin, uint32_t *flags)
{
	printf("%s\n", __func__);
	struct mtk_gpio_softc *sc;

	sc = device_get_softc(bus);

	if (gpios[0] >= sc->conf->padconf->npins)
		return (EINVAL);

	*pin = gpios[0];
	*flags = gpios[1];

	return (0);
}

static int
mtk_find_pinnum_by_name(struct mtk_gpio_softc *sc, const char *pinname)
{

	printf("%s pinname\n", __func__);
	int i;

	for (i = 0; i < sc->conf->padconf->npins; i++)
		if (!strcmp(pinname, sc->conf->padconf->pins_names[i].name))
			return i;

	return (-1);
}

static int
mtk_find_group_by_name(struct mtk_gpio_softc *sc, const char *group)
{
	int i;

	for (i = 0; i < sc->conf->padconf->npins_group ; i++)
		    if (!strcmp(group, sc->conf->padconf->pins_group[i].name))
			return (i);

	return (-1);
}

static int
mtk_gpio_set_mode(device_t dev, const char *group)
{
	struct mtk_gpio_softc *sc;
	uint32_t offset;
	uint32_t mask;
	uint32_t reg_data;
	uint8_t  bitpos;
	int val, i;
	int npins, id;
	const int *pins, *groups;
	int mode, pin;

	sc = device_get_softc(dev);

	id  = mtk_find_group_by_name(sc, group);
	pins = sc->conf->padconf->pins_group[id].pins;
	npins = sc->conf->padconf->pins_group[id].npins;
	groups =sc->conf->padconf->pins_group[id].funcs;

	for(i = 0; i < npins; i++) {
		MTK_GPIO_LOCK(sc);
		pin = pins[i];
		mode = groups[i];
		mtk_field_lookup(sc, PINCTRL_PIN_REG_MODE, pin, &offset, &mask, &bitpos);
		reg_data = MTK_BASE_READ_4(sc, offset);
		val = (reg_data >> bitpos) & mask;
		if (bootverbose)
			printf ("before pin %02i bit 0x%02x reg 0x%04x reg_data 0x%08x val 0x%02x\n",
			    pin, bitpos, offset, reg_data, val);
		reg_data &= ~(mask << bitpos);
		reg_data |= (mode & mask) << bitpos;
		MTK_BASE_WRITE_4(sc, offset, reg_data);
		MTK_GPIO_UNLOCK(sc);
		if (bootverbose)
			printf ("after  pin %02i bit 0x%02x reg 0x%04x reg_data 0x%08x val 0x%02x\n",
			    pin, bitpos, offset, reg_data, mode);
	}
	return (0);
}

static int
mtk_fdt_configure_pins(device_t dev, phandle_t cfgxref)
{
	struct mtk_gpio_softc *sc;
	phandle_t node;
	phandle_t mnode;
	const char **pinlist = NULL;
	const char **grouplist = NULL;
	char *pin_function = NULL;
	uint32_t pin_drive, pin_pull;
	int pins_nb, pin_num;
	int groups_nb, i, ret = 0;
	bool set_drive = false;

	sc = device_get_softc(dev);
	node = OF_node_from_xref(cfgxref);
	printf ("node 0x%x\n", node);

	mnode = ofw_bus_find_child(node, "mux");
	if (mnode <= 0) {
		device_printf(sc->sc_dev, "Cannot find mux subnode\n");
		return (ENOENT);
	}

	/* Set up GPIO mode. Structure check */
	pin_function = mtk_gpio_parse_function(mnode);
	if (pin_function == NULL) {
		device_printf(sc->sc_dev, "Cannot find function property\n");
		ret = ENOENT;
		goto out;
	}

	if (bootverbose)
		device_printf(sc->sc_dev, "Function: %s\n", pin_function);

	grouplist = mtk_gpio_parse_groups(mnode, &groups_nb);
	if (grouplist == NULL) {
		device_printf(sc->sc_dev, "Cannot find group property\n");
		ret = ENOENT;
		goto out;
	}

	for (i = 0; i < groups_nb; i++) {
		if (bootverbose)
			device_printf(sc->sc_dev, "Group: %s\n", grouplist[i]);
		mtk_gpio_set_mode(dev, grouplist[i]);
	}

	for (node = OF_child(node); node != 0; node = OF_peer(node)) {
		if (!ofw_bus_node_status_okay(node))
			continue;
		printf ("node 0x%x\n", node);
		if (node == mnode)
			continue;

		/* Getting all prop for configuring pins */
		pinlist = mtk_gpio_parse_pins(node, &pins_nb);
		if (pinlist == NULL) {
			device_printf(sc->sc_dev,
			    "Cannot find pins property\n");
			ret = ENOENT;
			goto out;
		}

		if (mtk_gpio_parse_drive_strength(node, &pin_drive) == 0)
			set_drive = true;
		else
			set_drive = false;

		pin_pull = mtk_gpio_parse_bias(node);

		if (set_drive)
			printf("%s pin_drive  %i \n",
			    __func__, pin_drive);

		printf("%s pin_pull %i \n", __func__, pin_pull);

		for (i = 0; i < pins_nb; i++) {
			printf("%s pin %s\n", __func__, pinlist[i]);
			pin_num = mtk_find_pinnum_by_name(sc, pinlist[i]);
			if (pin_num == -1) {
				device_printf(sc->sc_dev,
				    "Cannot find pins %s\n", pinlist[i]);
				ret = ENOENT;
				goto out;
			}

			MTK_GPIO_LOCK(sc);
			if (set_drive)
				mtk_gpio_set_drv(sc, pin_num, pin_drive);
			if (pin_pull != MTK_GPIO_NONE)
				mtk_gpio_set_pud(sc, pin_num, pin_pull);
//TODO input-enabl mt7622 don't have ies.
			MTK_GPIO_UNLOCK(sc);
		}
	}

 out:
	OF_prop_free(grouplist);
	OF_prop_free(pinlist);
	OF_prop_free(pin_function);
	return (ret);
}

static int
mtk_gpio_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Mediatek GPIO controller");
	return (BUS_PROBE_DEFAULT);
}

static int
mtk_gpio_attach(device_t dev)
{
	printf("%s\n", __func__);
//	int error;
	phandle_t gpio;
	struct mtk_gpio_softc *sc;
	struct clk_list *clkp, *clkp_tmp;
	clk_t clk;
//	hwreset_t rst = NULL;
	int off, err, clkret;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;


	/* Find our node. */
	gpio = ofw_bus_get_node(sc->sc_dev);
	if (!OF_hasprop(gpio, "gpio-controller"))
		/* Node is not a GPIO controller. */
		goto fail;
#if 0
	if (OF_hasprop(gpio, "mediatek,pctl-regmap") &&
	    syscon_get_by_ofw_property(dev, gpio,
	    "mediatek,pctl-regmap", &sc->syscon) != 0) {
                device_printf(dev, "cannot get syscfg_pctl_a driver handle\n");
                return (ENXIO);
        }
#endif

	mtx_init(&sc->sc_mtx, "mtk gpio", "gpio", MTX_SPIN);

	if (bus_alloc_resources(dev, mtk_gpio_res_spec, sc->sc_res) != 0) {
		device_printf(dev, "cannot allocate device resources\n");
		return (ENXIO);
	}

	if (bus_setup_intr(dev, sc->sc_res[MTK_GPIO_IRQRES],
	    INTR_TYPE_CLK | INTR_MPSAFE, NULL, mtk_gpio_intr, sc,
	    &sc->sc_intrhand)) {
		device_printf(dev, "cannot setup interrupt handler\n");
		goto fail;
	}

	/* Use the right pin data for the current SoC */
	sc->conf = (struct mtk_gpio_conf *)ofw_bus_search_compatible(dev,
	    compat_data)->ocd_data;

#if 0
	if (hwreset_get_by_ofw_idx(dev, 0, 0, &rst) == 0) {
		error = hwreset_deassert(rst);
		if (error != 0) {
			device_printf(dev, "cannot de-assert reset\n");
			goto fail;
		}
	}
#endif
	TAILQ_INIT(&sc->clk_list);
	for (off = 0, clkret = 0; clkret == 0; off++) {
		clkret = clk_get_by_ofw_index(dev, 0, off, &clk);
		if (clkret != 0)
			break;
		err = clk_enable(clk);
		if (err != 0) {
			device_printf(dev, "Could not enable clock %s\n",
			    clk_get_name(clk));
			goto fail;
		}
		clkp = malloc(sizeof(*clkp), M_DEVBUF, M_WAITOK | M_ZERO);
		clkp->clk = clk;
		TAILQ_INSERT_TAIL(&sc->clk_list, clkp, next);
	}
	if (clkret != 0 && clkret != ENOENT) {
		device_printf(dev, "Could not find clock at offset %d (%d)\n",
		    off, clkret);
		goto fail;
	}

	mtk_gpio_register_isrcs(sc);
	intr_pic_register(dev, OF_xref_from_node(ofw_bus_get_node(dev)));

	/*
	 * Register as a pinctrl device
	 */

	fdt_pinctrl_register(dev, NULL);
	fdt_pinctrl_configure_tree(dev);

	sc->sc_busdev = gpiobus_add_bus(dev);
	if (sc->sc_busdev == NULL)
		goto fail;

	return (0);

fail:
	if (sc->sc_irq_res)
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->sc_irq_res);
	if (sc->sc_mem_res)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->sc_mem_res);
	mtx_destroy(&sc->sc_mtx);

	/* Disable clock */
	TAILQ_FOREACH_SAFE(clkp, &sc->clk_list, next, clkp_tmp) {
		err = clk_disable(clkp->clk);
		if (err != 0)
			device_printf(dev, "Could not disable clock %s\n",
			    clk_get_name(clkp->clk));
		err = clk_release(clkp->clk);
		if (err != 0)
			device_printf(dev, "Could not release clock %s\n",
			    clk_get_name(clkp->clk));
		TAILQ_REMOVE(&sc->clk_list, clkp, next);
		free(clkp, M_DEVBUF);
	}
#if 0
	/* Assert resets */
	if (rst) {
		hwreset_assert(rst);
		hwreset_release(rst);
	}
#endif
	return (ENXIO);
}

static int
mtk_gpio_detach(device_t dev)
{

	return (EBUSY);
}

static void
mtk_gpio_intr(void *arg)
{
	struct mtk_gpio_softc *sc;

	sc = (struct mtk_gpio_softc *)arg;

	MTK_GPIO_LOCK(sc);
	printf("%s vau. hello world\n", __func__);
	for (int i = 0; i < 6; i++) {
		printf("%s enit stat0 val 0x%x\n",
		    __func__, MTK_EINT_READ_4(sc, i * 0x4));
	}
	MTK_GPIO_UNLOCK(sc);
}

/*
 * Interrupts support
 */

static int
mtk_gpio_register_isrcs(struct mtk_gpio_softc *sc)
{
	printf("%s\n", __func__);

	return (0);
}

static void
mtk_gpio_pic_disable_intr_locked(struct mtk_gpio_softc *sc, struct intr_irqsrc *isrc)
{
	printf("%s\n", __func__);

}

static void
mtk_gpio_pic_disable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	printf("%s\n", __func__);

}

static void
mtk_gpio_pic_enable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	printf("%s\n", __func__);

}

static int
mtk_gpio_pic_map_gpio(struct mtk_gpio_softc *sc, struct intr_map_data_gpio *dag,
    u_int *irqp, u_int *mode)
{
	printf("%s\n", __func__);
	return (0);
}

static int
mtk_gpio_pic_map_intr(device_t dev, struct intr_map_data *data,
    struct intr_irqsrc **isrcp)
{
	printf("%s\n", __func__);
	return (0);
}

static int
mtk_gpio_pic_setup_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{

	printf("%s\n", __func__);
	return (0);
}

static int
mtk_gpio_pic_teardown_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{

	printf("%s\n", __func__);
	return (0);
}

static void
mtk_gpio_pic_post_filter(device_t dev, struct intr_irqsrc *isrc)
{

	printf("%s\n", __func__);
}

static void
mtk_gpio_pic_post_ithread(device_t dev, struct intr_irqsrc *isrc)
{

	printf("%s\n", __func__);
}

static void
mtk_gpio_pic_pre_ithread(device_t dev, struct intr_irqsrc *isrc)
{
	printf("%s\n", __func__);

}


/*
 * OFWBUS Interface
 */
static phandle_t
mtk_gpio_get_node(device_t dev, device_t bus)
{
	printf("%s\n", __func__);

	/* We only have one child, the GPIO bus, which needs our own node. */
	return (ofw_bus_get_node(dev));
}

static device_method_t mtk_gpio_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		mtk_gpio_probe),
	DEVMETHOD(device_attach,	mtk_gpio_attach),
	DEVMETHOD(device_detach,	mtk_gpio_detach),

	/* Interrupt controller interface */
	DEVMETHOD(pic_disable_intr,	mtk_gpio_pic_disable_intr),
	DEVMETHOD(pic_enable_intr,	mtk_gpio_pic_enable_intr),
	DEVMETHOD(pic_map_intr,		mtk_gpio_pic_map_intr),
	DEVMETHOD(pic_setup_intr,	mtk_gpio_pic_setup_intr),
	DEVMETHOD(pic_teardown_intr,	mtk_gpio_pic_teardown_intr),
	DEVMETHOD(pic_post_filter,	mtk_gpio_pic_post_filter),
	DEVMETHOD(pic_post_ithread,	mtk_gpio_pic_post_ithread),
	DEVMETHOD(pic_pre_ithread,	mtk_gpio_pic_pre_ithread),

	/* GPIO protocol */
	DEVMETHOD(gpio_get_bus,		mtk_gpio_get_bus),
	DEVMETHOD(gpio_pin_max,		mtk_gpio_pin_max),
	DEVMETHOD(gpio_pin_getname,	mtk_gpio_pin_getname),
	DEVMETHOD(gpio_pin_getflags,	mtk_gpio_pin_getflags),
	DEVMETHOD(gpio_pin_getcaps,	mtk_gpio_pin_getcaps),
	DEVMETHOD(gpio_pin_setflags,	mtk_gpio_pin_setflags),
	DEVMETHOD(gpio_pin_get,		mtk_gpio_pin_get),
	DEVMETHOD(gpio_pin_set,		mtk_gpio_pin_set),
	DEVMETHOD(gpio_pin_toggle,	mtk_gpio_pin_toggle),
	DEVMETHOD(gpio_map_gpios,	mtk_gpio_map_gpios),

	/* ofw_bus interface */
	DEVMETHOD(ofw_bus_get_node,	mtk_gpio_get_node),

	/* fdt_pinctrl interface */
	DEVMETHOD(fdt_pinctrl_configure,mtk_fdt_configure_pins),

	DEVMETHOD_END
};

static driver_t mtk_gpio_driver = {
	"gpio",
	mtk_gpio_methods,
	sizeof(struct mtk_gpio_softc),
};

EARLY_DRIVER_MODULE(mtk_gpio, simplebus, mtk_gpio_driver, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LATE);
