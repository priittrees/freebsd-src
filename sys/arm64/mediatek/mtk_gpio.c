/*-;
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 - 2026 Priit Trees <trees@neti.ee>
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

#include <dev/clk/clk.h>

#include "pic_if.h"
#include "gpio_if.h"

#include "mtk_pinctrl.h"

//#define DEBUG	1

#ifdef DEBUG
#define DPRINTF(format, arg...) printf("%s: " format, __func__, arg)
#else
#define DPRINTF(format, arg...)
#endif

#define	MTK_GPIO_INTR_CAPS	(GPIO_INTR_LEVEL_LOW | 			\
	GPIO_INTR_LEVEL_HIGH |	GPIO_INTR_EDGE_RISING |			\
	GPIO_INTR_EDGE_FALLING | GPIO_INTR_EDGE_BOTH)

#define	MTK_GPIO_DEFAULT_CAPS	(GPIO_PIN_INPUT | GPIO_PIN_OUTPUT |	\
	  GPIO_PIN_PULLUP | GPIO_PIN_PULLDOWN | MTK_GPIO_INTR_CAPS)


#define MTK_GPIO_PIN		1

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
	bool			edge_both;
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
	bool 			*is_gpio;
};

static struct resource_spec mtk_gpio_res_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_MEMORY,	1,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE | RF_SHAREABLE },
	{ -1,			0,	0 }
};

#define	MTK_GPIO_READ_4(_sc, _reg)					\
	bus_read_4((_sc)->sc_res[MTK_BASE_MEMRES], _reg)
#define MTK_GPIO_WRITE_4(_sc, _reg, _value)				\
	bus_write_4((_sc)->sc_res[MTK_BASE_MEMRES], _reg, _value)
#define	MTK_EINT_READ_4(_sc, _reg)					\
	bus_read_4((_sc)->sc_res[MTK_EINT_MEMRES], _reg)
#define MTK_EINT_WRITE_4(_sc, _reg, _value)				\
	bus_write_4((_sc)->sc_res[MTK_EINT_MEMRES], _reg, _value)
#define	MTK_GPIO_LOCK(_sc)		mtx_lock_spin(&(_sc)->sc_mtx)
#define	MTK_GPIO_UNLOCK(_sc)		mtx_unlock_spin(&(_sc)->sc_mtx)
#define	MTK_GPIO_LOCK_ASSERT(_sc)	mtx_assert(&(_sc)->sc_mtx, MA_OWNED)

#define	MTK_EINT_STA(_irq)		(((_irq >> 5) << 2) + 0x00)
#define	MTK_EINT_ACK(_irq)		(((_irq >> 5) << 2) + 0x40)
#define	MTK_EINT_MASK(_irq)		(((_irq >> 5) << 2) + 0x80)
#define	MTK_EINT_MASK_SET(_irq)		(((_irq >> 5) << 2) + 0xc0)
#define	MTK_EINT_MASK_CLR(_irq)		(((_irq >> 5) << 2) + 0x100)
#define MTK_EINT_SENS(_irq)		(((_irq >> 5) << 2) + 0x140)
#define	MTK_EINT_SENS_SET(_irq)		(((_irq >> 5) << 2) + 0x180)
#define	MTK_EINT_SENS_CLR(_irq)		(((_irq >> 5) << 2) + 0x1c0)
#define MTK_EINT_SOFT(_irq)		(((_irq >> 5) << 2) + 0x200)
#define MTK_EINT_SOFT_SET(_irq)		(((_irq >> 5) << 2) + 0x240)
#define MTK_EINT_SOFT_CLK(_irq)		(((_irq >> 5) << 2) + 0x280)
#define MTK_EINT_POL(_irq)		(((_irq >> 5) << 2) + 0x300)
#define	MTK_EINT_POL_SET(_irq)		(((_irq >> 5) << 2) + 0x340)
#define	MTK_EINT_POL_CLR(_irq)		(((_irq >> 5) << 2) + 0x380)
#define MTK_EINT_DO_EN(_irq)		(((_irq >> 5) << 2) + 0x400)

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

static int
mtk_find_reg_by_pin(struct mtk_gpio_softc *sc, int field, uint32_t pin,
     uint32_t *reg, uint32_t *mask, uint8_t *bit)
{
	const struct mtk_pinctrl *ctrl = &sc->conf->padconf->pinctrl[field];
	int npins = sc->conf->padconf->npins;
	int i;
	if (ctrl == NULL) {
		device_printf(sc->sc_dev,
		    "Not support field %d for this soc\n", field);
		return (EINVAL);
	}

	for (i = 0; i <= npins; i++)
	{
		if (ctrl->reg[i].pin == pin) {
			*reg = ctrl->reg[i].reg;
			*mask = ctrl->reg[i].mask;
			*bit = ctrl->reg[i].bit;
			return(0);
		}
	}

	device_printf(sc->sc_dev,
	    "Not support field %d for pin = %d\n", field, pin);
	return (EINVAL);
}

static uint32_t
mtk_gpio_get_dir(struct mtk_gpio_softc *sc, uint32_t pin)
{
	uint32_t offset;
	uint32_t mask;
	uint32_t reg_data;
	uint8_t  bitpos;

	/* Must be called with lock held. */
	MTK_GPIO_LOCK_ASSERT(sc);

	if (pin > sc->conf->padconf->npins)
		return (0);

	mtk_find_reg_by_pin(sc, PINCTRL_PIN_REG_DIR,
	    pin, &offset, &mask, &bitpos);
	reg_data = MTK_GPIO_READ_4(sc, offset);
	DPRINTF("pin %3i dir offset 0x%08x data 0x%08x\n",
	    pin, offset, reg_data);

	return ((reg_data & (mask << bitpos)) ?
		    MTK_GPIO_OUTPUT : MTK_GPIO_INPUT);
}

static int
mtk_gpio_set_dir(struct mtk_gpio_softc *sc, uint32_t pin, uint32_t dir)
{

	uint32_t offset;
	uint32_t mask;
	uint32_t reg_data;
	uint8_t  bitpos;

	/* Must be called with lock held. */
	MTK_GPIO_LOCK_ASSERT(sc);

	mtk_find_reg_by_pin(sc, PINCTRL_PIN_REG_DIR,
	    pin, &offset, &mask, &bitpos);
        reg_data = MTK_GPIO_READ_4(sc, offset);
	DPRINTF("pin %3i dir offset 0x%08x data before set 0x%08x\n",
	    pin, offset, reg_data);
	reg_data &= ~(mask<< bitpos);
	reg_data |= (dir << bitpos);
	DPRINTF("pin %3i dir offset 0x%08x set value 0x%08x\n",
	    pin, offset, reg_data);
	MTK_GPIO_WRITE_4(sc, offset, reg_data);

	return (0);
}

static int
mtk_gpio_get_bias(struct mtk_gpio_softc *sc, uint32_t pin)
{
	uint32_t offset;
	uint32_t mask;
	uint32_t reg_data;
	uint8_t  bitpos;
	int pu, pd;

	/* Must be called with lock held. */
	MTK_GPIO_LOCK_ASSERT(sc);

	mtk_find_reg_by_pin(sc, PINCTRL_PIN_REG_PD,
	    pin, &offset, &mask, &bitpos);
	reg_data = MTK_GPIO_READ_4(sc, offset);
	DPRINTF("pin %3i pd offset 0x%08x data 0x%08x\n",
	    pin, offset, reg_data);
	pd  = (reg_data & (mask << bitpos)) ? 1 : 0;

	mtk_find_reg_by_pin(sc, PINCTRL_PIN_REG_PU,
	    pin, &offset, &mask, &bitpos);
	reg_data = MTK_GPIO_READ_4(sc, offset);
	DPRINTF("pin %3i pu offset 0x%08x data 0x%08x\n",
	    pin, offset, reg_data);
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
mtk_gpio_set_bias(struct mtk_gpio_softc *sc, uint32_t pin, uint32_t state)
{
	uint32_t offset;
	uint32_t mask;
	uint32_t reg_data;
	uint8_t  bitpos;
	int pu = 0, pd = 0;

	if (mtk_gpio_get_bias(sc, pin) == state)
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
	else {
		device_printf(sc->sc_dev, "%s invalid state 0x%x to pin %i\n",
		    __func__, state, pin);
		return;
	}

	mtk_find_reg_by_pin(sc, PINCTRL_PIN_REG_PU,
	    pin, &offset, &mask, &bitpos);
	reg_data = MTK_GPIO_READ_4(sc, offset);
	DPRINTF("pin %3i pu offset 0x%08x data before set 0x%08x\n",
	    pin, offset, reg_data);
	reg_data &= ~(mask << bitpos);
	reg_data |= (pu << bitpos);
	DPRINTF("pin %3i pu offset 0x%08x set value 0x%08x\n",
	    pin, offset, reg_data);
	MTK_GPIO_WRITE_4(sc, offset, reg_data);

	mtk_find_reg_by_pin(sc, PINCTRL_PIN_REG_PD,
	    pin, &offset, &mask, &bitpos);
	DPRINTF("pin %3i pd offset 0x%08x data before set 0x%08x\n",
	    pin, offset, reg_data);
        reg_data = MTK_GPIO_READ_4(sc, offset);
	reg_data &= ~(mask << bitpos);
	reg_data |= (pd << bitpos);
	DPRINTF("pin %3i pd offset 0x%08x set value 0x%08x\n",
	    pin, offset, reg_data);
	MTK_GPIO_WRITE_4(sc, offset, reg_data);
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

	mtk_find_reg_by_pin(sc, PINCTRL_PIN_REG_E4,
	    pin, &offset, &mask, &bitpos);
        reg_data = MTK_GPIO_READ_4(sc, offset);
        e4 = ((reg_data >> bitpos) & mask);
	DPRINTF("pin %3i e4 offset 0x%08x data 0x%08x\n",
	    pin, offset, reg_data);

	mtk_find_reg_by_pin(sc, PINCTRL_PIN_REG_E8,
	    pin, &offset, &mask, &bitpos);
        reg_data = MTK_GPIO_READ_4(sc, offset);
        e8 = ((reg_data >> bitpos) & mask);
	DPRINTF("pin %3i e8 offset 0x%08x data 0x%08x\n",
	    pin, offset, reg_data);

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
		device_printf(sc->sc_dev, "%s invalid drive %i to pin %i\n",
		    __func__, drive, pin);
		return;
	}

	mtk_find_reg_by_pin(sc, PINCTRL_PIN_REG_E4,
	    pin, &offset, &mask, &bitpos);
        reg_data = MTK_GPIO_READ_4(sc, offset);
	DPRINTF("pin %3i e4 offset 0x%08x data before set 0x%08x\n",
	    pin, offset, reg_data);
        reg_data &= ~(mask << bitpos);
        reg_data |= (e4 << bitpos);
	DPRINTF("pin %3i e4 offset 0x%08x set value 0x%08x\n",
	    pin, offset, reg_data);
        MTK_GPIO_WRITE_4(sc, offset, reg_data);

	mtk_find_reg_by_pin(sc, PINCTRL_PIN_REG_E8,
	    pin, &offset, &mask, &bitpos);
	DPRINTF("pin %3i e8 offset 0x%08x data before set 0x%08x\n",
	    pin, offset, reg_data);
        reg_data = MTK_GPIO_READ_4(sc, offset);
        reg_data &= ~(mask << bitpos);
        reg_data |= (e8 << bitpos);
	DPRINTF("pin %3i e8 offset 0x%08x set value 0x%08x\n",
	    pin, offset, reg_data);
        MTK_GPIO_WRITE_4(sc, offset, reg_data);
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
	int dir;
	int pud;

	sc = device_get_softc(dev);

	if (pin >= sc->conf->padconf->npins)
		return (EINVAL);

	if (sc->is_gpio[pin] == false)
		return (EINVAL);

	MTK_GPIO_LOCK(sc);
	dir = mtk_gpio_get_dir(sc, pin);
	switch (dir) {
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

	pud = mtk_gpio_get_bias(sc, pin);
	switch (pud) {
	case MTK_GPIO_PULLDOWN:
		*flags |= GPIO_PIN_PULLDOWN;
		break;
	case MTK_GPIO_PULLUP:
		*flags |= GPIO_PIN_PULLUP;
		break;
	default:
		/* bias-disable */
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
	    sc->conf->padconf->pins[pin].name);
	name[GPIOMAXNAME - 1] = '\0';

	return (0);
}

static int
mtk_gpio_pin_setflags(device_t dev, uint32_t pin, uint32_t flags)
{
	struct mtk_gpio_softc *sc;
	int err = 0;
	int val;

	sc = device_get_softc(dev);
	if (pin > sc->conf->padconf->npins)
		return (EINVAL);

	if (sc->is_gpio[pin] == false)
		return (EINVAL);

	/* Manage input/output. */
	MTK_GPIO_LOCK(sc);
	if (flags & GPIO_PIN_INPUT) {
		err = mtk_gpio_set_dir(sc, pin, MTK_GPIO_INPUT);
	} else if ((flags & GPIO_PIN_OUTPUT) &&
	    mtk_gpio_get_dir(sc, pin) != MTK_GPIO_OUTPUT) {
		if (flags & GPIO_PIN_PRESET_LOW) {
			mtk_gpio_pin_set_locked(sc, pin, 0);
		} else if (flags & GPIO_PIN_PRESET_HIGH) {
			mtk_gpio_pin_set_locked(sc, pin, 1);
		} else {
			/* Read the pin and preset output to current state. */
			err = mtk_gpio_set_dir(sc, pin, MTK_GPIO_INPUT);
			if (err == 0) {
				mtk_gpio_pin_get_locked(sc, pin, &val);
				mtk_gpio_pin_set_locked(sc, pin, val);

			}
		}
		if (err == 0)
			err = mtk_gpio_set_dir(sc, pin, MTK_GPIO_OUTPUT);
	}

	if (err)
		return (err);

	/* Manage Pull-up/pull-down/bias-disable. */
	if (flags & GPIO_PIN_PULLUP)
		mtk_gpio_set_bias(sc, pin, MTK_GPIO_PULLUP);
	else if (flags & GPIO_PIN_PULLDOWN)
		mtk_gpio_set_bias(sc, pin, MTK_GPIO_PULLDOWN);
	else
		mtk_gpio_set_bias(sc, pin, MTK_GPIO_NONE);

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

	mtk_find_reg_by_pin(sc, PINCTRL_PIN_REG_DO,
	    pin, &offset, &mask, &bitpos);
	reg_data = MTK_GPIO_READ_4(sc, offset);
	DPRINTF("pin %3i do offset 0x%08x data before set 0x%08x\n",
	    pin, offset, reg_data);

	if (value)
		reg_data |= (1 << bitpos);
	else
		reg_data &= ~(1 << bitpos);

	DPRINTF("pin %3i do offset 0x%08x set value 0x%08x\n",
	    pin, offset, reg_data);
	MTK_GPIO_WRITE_4(sc, offset, reg_data);

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
	mtk_find_reg_by_pin(sc, PINCTRL_PIN_REG_DO,
	    pin, &offset, &mask, &bitpos);
	reg_data = MTK_GPIO_READ_4(sc, offset);
	DPRINTF("pin %3i do offset 0x%08x data before set 0x%08x\n",
	    pin, offset, reg_data);
	if (reg_data & (1 << bitpos))
		reg_data &= ~(1 << pin);
	else
		reg_data |= (1 << pin);

	DPRINTF("pin %3i do offset 0x%08x set value 0x%08x\n",
	    pin, offset, reg_data);
	MTK_GPIO_WRITE_4(sc, offset, reg_data);
	MTK_GPIO_UNLOCK(sc);

	return (0);
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

	dir = mtk_gpio_get_dir(sc, pin);
	if (dir == MTK_GPIO_OUTPUT) {
		mtk_find_reg_by_pin(sc, PINCTRL_PIN_REG_DO,
		    pin, &offset, &mask, &bitpos);
		reg_data = MTK_GPIO_READ_4(sc, offset);
		DPRINTF("pin %3i do offset 0x%08x data 0x%08x\n",
		    pin, offset, reg_data);
		*val = (reg_data & (mask << bitpos)) ? 1 : 0;
		}
	else {
		mtk_find_reg_by_pin(sc, PINCTRL_PIN_REG_DI,
		    pin, &offset, &mask, &bitpos);
		reg_data = MTK_GPIO_READ_4(sc, offset);
		DPRINTF("pin %3i di offset 0x%08x data 0x%08x\n",
		    pin, offset, reg_data);
		*val = (reg_data & (mask << bitpos)) ? 1 : 0;
	}
	return (0);
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
mtk_gpio_parse_groups(phandle_t node, int *groups_nb)
{
	const char **grouplist;

	*groups_nb = ofw_bus_string_list_to_array(node, "groups", &grouplist);
	if (*groups_nb > 0)
		return (grouplist);

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
	uint32_t value;

	if (OF_getencprop(node, "drive-strength", &value,
	    sizeof(value)) != -1) {
		*drive = value;
		return (0);
	}

	return (-1);
}

static int
mtk_gpio_map_gpios(device_t bus, phandle_t dev, phandle_t gparent, int gcells,
    pcell_t *gpios, uint32_t *pin, uint32_t *flags)
{
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
	int i;

	for (i = 0; i < sc->conf->padconf->npins; i++)
		if (!strcmp(pinname, sc->conf->padconf->pins[i].name))
			return i;

	return (-1);
}

static int
mtk_find_group_by_name(struct mtk_gpio_softc *sc, const char *group)
{
	int i;

	for (i = 0; i < sc->conf->padconf->ngroups ; i++)
		    if (!strcmp(group, sc->conf->padconf->groups[i].name))
			return (i);

	return (-1);
}

static int
mtk_gpio_set_func(struct mtk_gpio_softc *sc, int pin, uint8_t func)
{
	uint32_t offset;
	uint32_t mask;
	uint32_t reg_data;
	uint8_t  bitpos;
	int error;
	error = mtk_find_reg_by_pin(sc, PINCTRL_PIN_REG_MODE,
	    pin, &offset, &mask, &bitpos);

	if(error)
		return (error);

	MTK_GPIO_LOCK(sc);
	reg_data = MTK_GPIO_READ_4(sc, offset);
	DPRINTF("pin %3i bit pos 0x%02x mode offset 0x%08x data 0x%08x\n",
		pin, bitpos, offset, reg_data);
	reg_data &= ~(mask << bitpos);
	reg_data |= (func & mask) << bitpos;
	MTK_GPIO_WRITE_4(sc, offset, reg_data);
	DPRINTF("pin %3i bit pos 0x%02x mode offset 0x%08x set value 0x%08x\n",
		pin, bitpos, offset, reg_data);
	MTK_GPIO_UNLOCK(sc);
	return (0);
}

static int
mtk_gpio_set_mode(device_t dev, const char *group)
{
	struct mtk_gpio_softc *sc;
	struct mtk_pin_func *pconf;
	int i;
	int npins, id;

	sc = device_get_softc(dev);

	id  = mtk_find_group_by_name(sc, group);
	if (id < 0) {
		device_printf(sc->sc_dev, "Group %s not found\n",
			group);
		return(EINVAL);
	}

	pconf = sc->conf->padconf->groups[id].pconf;
	npins = sc->conf->padconf->groups[id].npins;
#ifdef DEBUG 
	for(i = 0; i < npins; i++) {
		DPRINTF("Group: %s id %02i pin %3i set func %i\n",
		    sc->conf->padconf->groups[id].name,
		    id, pconf[i].pin, pconf[i].func);
	}
#endif

	for(i = 0; i < npins; i++) {
		mtk_gpio_set_func(sc, pconf[i].pin, pconf[i].func);
		sc->is_gpio[pconf[i].pin] = false;
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

	mnode = ofw_bus_find_child(node, "mux");
	if (mnode <= 0) {
		device_printf(sc->sc_dev,
		    "Cannot find mux from node 0x%x\n",node);
		return (ENOENT);
	}

	/* Set up GPIO mode. Structure check */
	pin_function = mtk_gpio_parse_function(mnode);
	if (pin_function == NULL) {
		device_printf(sc->sc_dev,
		    "Cannot find function property for node  0x%x\n",node);
		ret = ENOENT;
		goto out;
	}

	if (bootverbose)
		device_printf(sc->sc_dev, "Node 0x%x function: %s\n",
		    node, pin_function);

	grouplist = mtk_gpio_parse_groups(mnode, &groups_nb);
	if (grouplist == NULL) {
		device_printf(sc->sc_dev,
		    "Cannot find group property for node 0x%x\n",node);
		ret = ENOENT;
		goto out;
	}

	for (i = 0; i < groups_nb; i++) {
		mtk_gpio_set_mode(dev, grouplist[i]);
	}

	for (node = OF_child(node); node != 0; node = OF_peer(node)) {
		if (!ofw_bus_node_status_okay(node))
			continue;

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

		for (i = 0; i < pins_nb; i++) {
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
				mtk_gpio_set_bias(sc, pin_num, pin_pull);
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
	phandle_t gpio;
	struct mtk_gpio_softc *sc;
	struct clk_list *clkp, *clkp_tmp;
	clk_t clk;
	int npins;
	int off, err, clkret;
	int i;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;


	/* Find our node. */
	gpio = ofw_bus_get_node(sc->sc_dev);
	if (!OF_hasprop(gpio, "gpio-controller")){
		/* Node is not a GPIO controller. */
		goto fail;
	}

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

	npins = sc->conf->padconf->npins;

	sc->is_gpio = malloc(npins * sizeof(*sc->is_gpio),
	    M_DEVBUF, M_WAITOK | M_ZERO);

	/* Set the cached value to gpio
	 * By default, all medatek gpio pins are associated with a function.
	 * We will make a cache and assume that all pins are GPIO pins.
	 * In the "fdt_pinctrl_configure" function, we set these pins to
	 * false so that the function is actually configured
	 */
	for(i = 0; i < npins; i++)
	{
		sc->is_gpio[i] = true;
	}

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

	/*
	 * We set unused function pins to gpio modes.
	 */

	for (i = 0; i < npins; i++) {
		if (sc->is_gpio[i] == true)
		{
			mtk_gpio_set_func(sc, i, MTK_GPIO_PIN);
			DPRINTF("pin %3i is set to GPIO mode\n", i);
		}
	}

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

	return (ENXIO);
}

static int
mtk_gpio_detach(device_t dev)
{
	return (EBUSY);
}

/*
 * Interrupts support
 */

/* The device does not directly support both edges. 
 * If the polarity of the pin changes, the EINT_POL
 * value must be changed. There is a risk of losing
 * a fast polarity change because the Software
 * Interrupt is not implemented.
 */

static void
mtk_gpio_intr(void *arg)
{
	struct mtk_gpio_softc *sc;
	struct intr_irqsrc *isrc;
	uint32_t val;
	uint32_t mask;
	u_int irq;

	sc = (struct mtk_gpio_softc *)arg;

	MTK_GPIO_LOCK(sc);

	for (irq = 0; irq < sc->conf->padconf->npins; irq++) {
		if (!sc->gpio_pic_irqsrc[irq].enabled)
			continue;

		val = MTK_EINT_READ_4(sc, MTK_EINT_STA(irq));
		mask = 1U << irq;

		if ((mask & val) == 0)
			continue;

		isrc = &sc->gpio_pic_irqsrc[irq].isrc;
		if (intr_isrc_dispatch(isrc, curthread->td_intr_frame) != 0) {
			mtk_gpio_pic_disable_intr_locked(sc, isrc);
			mtk_gpio_pic_post_filter(sc->sc_dev, isrc);
			device_printf(sc->sc_dev, "Stray irq %u disabled\n",
			    irq);
		}

	}
	MTK_GPIO_UNLOCK(sc);
}

static int
mtk_gpio_register_isrcs(struct mtk_gpio_softc *sc)
{
	const char *name;
	int nirqs;
	int pin;
	int err;

	name = device_get_nameunit(sc->sc_dev);

	for (nirqs = 0, pin = 0; pin < sc->conf->padconf->npins; pin++) {
                if (sc->conf->padconf->pins[pin].eint_mux == 0)
                        continue;

                nirqs++;
        }

	sc->gpio_pic_irqsrc = malloc(sizeof(*sc->gpio_pic_irqsrc) * nirqs,
	    M_DEVBUF, M_WAITOK | M_ZERO);

	for (nirqs = 0, pin = 0; pin < sc->conf->padconf->npins; pin++) {
		if (sc->conf->padconf->pins[pin].eint_mux == 0)
			continue;

		sc->gpio_pic_irqsrc[nirqs].pin = pin;
		sc->gpio_pic_irqsrc[nirqs].irq = nirqs;
		sc->gpio_pic_irqsrc[nirqs].mode = GPIO_INTR_CONFORM;

		err = intr_isrc_register(&sc->gpio_pic_irqsrc[nirqs].isrc,
		    sc->sc_dev, 0, "%s,%u", name, nirqs);

		if (err) {
			device_printf(sc->sc_dev,
			    "intr_isrs_register failed for irq %d\n", nirqs);
		}

		MTK_EINT_WRITE_4(sc, MTK_EINT_MASK_SET(nirqs),
		    (1U << (nirqs & 0x1f)));

		nirqs++;
	}

	sc->nirqs = nirqs;
	return (0);
}

static void
mtk_gpio_pic_disable_intr_locked(struct mtk_gpio_softc *sc,
    struct intr_irqsrc *isrc)
{
	u_int irq;
	uint32_t mask;

	MTK_GPIO_LOCK_ASSERT(sc);
	irq = ((struct gpio_irqsrc *)isrc)->irq;
	mask = 1U << (irq & 0x1f);
	if (sc->gpio_pic_irqsrc[irq].edge_both) {
		MTK_EINT_WRITE_4(sc, MTK_EINT_SOFT_CLK(irq), mask);
	}
	MTK_EINT_WRITE_4(sc, MTK_EINT_MASK_SET(irq), mask);

	sc->gpio_pic_irqsrc[irq].enabled = false;
}

static void
mtk_gpio_pic_disable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	struct mtk_gpio_softc *sc;

	sc = device_get_softc(dev);

	MTK_GPIO_LOCK(sc);
	mtk_gpio_pic_disable_intr_locked(sc, isrc);
	MTK_GPIO_UNLOCK(sc);

}

static void
mtk_gpio_pic_enable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	struct mtk_gpio_softc *sc;
	u_int irq;
	uint32_t mask;
	unsigned int pinval = 0;

	sc = device_get_softc(dev);
	irq = ((struct gpio_irqsrc *)isrc)->irq;
	mask = 1U << (irq & 0x1f);

	MTK_GPIO_LOCK(sc);
	if (sc->gpio_pic_irqsrc[irq].edge_both) {
		mtk_gpio_pin_get_locked(sc, irq, &pinval);
		if (pinval == 1 )
			MTK_EINT_WRITE_4(sc, MTK_EINT_POL_CLR(irq), mask);
		else
			MTK_EINT_WRITE_4(sc, MTK_EINT_POL_SET(irq), mask);
	}
	MTK_EINT_WRITE_4(sc, MTK_EINT_MASK_CLR(irq), mask);
	MTK_GPIO_UNLOCK(sc);

	sc->gpio_pic_irqsrc[irq].enabled = true;
}

static int
mtk_gpio_pic_map_gpio(struct mtk_gpio_softc *sc,
    struct intr_map_data_gpio *dag,
    u_int *irqp, u_int *mode)
{
	u_int irq;
	int pin;

	irq = dag->gpio_pin_num;

	for (pin = 0; pin < sc->nirqs; pin++)
		if (sc->gpio_pic_irqsrc[pin].pin == irq)
			break;

	if (pin >= sc->nirqs) {
		device_printf(sc->sc_dev, "Invalid interrupt number %u\n", irq);
		return (EINVAL);
	}

	switch (dag->gpio_intr_mode) {
	case GPIO_INTR_LEVEL_LOW:
	case GPIO_INTR_LEVEL_HIGH:
	case GPIO_INTR_EDGE_RISING:
	case GPIO_INTR_EDGE_FALLING:
	case GPIO_INTR_EDGE_BOTH:
		break;
	default:
		device_printf(sc->sc_dev, "Unsupported interrupt mode 0x%8x\n",
		    dag->gpio_intr_mode);
		return (EINVAL);
	}

	*irqp = pin;
	if (mode != NULL)
		*mode = dag->gpio_intr_mode;

	return (0);
}

static int
mtk_gpio_pic_map_intr(device_t dev, struct intr_map_data *data,
    struct intr_irqsrc **isrcp)
{
	struct mtk_gpio_softc *sc;
	u_int irq;
	int err;

	sc = device_get_softc(dev);
	switch (data->type) {
	case INTR_MAP_DATA_GPIO:
		err = mtk_gpio_pic_map_gpio(sc,
		    (struct intr_map_data_gpio *)data, &irq, NULL);
		break;
	default:
		return (ENOTSUP);
	};

	if (err == 0)
		*isrcp = &sc->gpio_pic_irqsrc[irq].isrc;

	return (0);
}

static int
mtk_gpio_pic_setup_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{
	struct mtk_gpio_softc *sc;
	uint32_t mask, val;
	u_int irq, mode;
	int err;

	sc = device_get_softc(dev);

	err = 0;
	switch (data->type) {
	case INTR_MAP_DATA_GPIO:
		err = mtk_gpio_pic_map_gpio(sc,
		    (struct intr_map_data_gpio *)data, &irq, &mode);
		if (err != 0)
			return (err);
		break;
	default:
		return (ENOTSUP);
	};

	mask = 1U << (irq & 0x1f);

	MTK_GPIO_LOCK(sc);
	switch (mode) {
	case GPIO_INTR_LEVEL_LOW:
		sc->gpio_pic_irqsrc[irq].edge_both = false;
		MTK_EINT_WRITE_4(sc, MTK_EINT_POL_CLR(irq), mask);
		MTK_EINT_WRITE_4(sc, MTK_EINT_SENS_SET(irq), mask);
		break;
	case GPIO_INTR_LEVEL_HIGH:
		sc->gpio_pic_irqsrc[irq].edge_both = false;
		MTK_EINT_WRITE_4(sc, MTK_EINT_POL_SET(irq), mask);
		MTK_EINT_WRITE_4(sc, MTK_EINT_SENS_SET(irq), mask);
		break;
	case GPIO_INTR_EDGE_RISING:
		sc->gpio_pic_irqsrc[irq].edge_both = false;
		MTK_EINT_WRITE_4(sc, MTK_EINT_POL_SET(irq), mask);
		MTK_EINT_WRITE_4(sc, MTK_EINT_SENS_CLR(irq), mask);
		break;
	case GPIO_INTR_EDGE_FALLING:
		sc->gpio_pic_irqsrc[irq].edge_both = false;
		MTK_EINT_WRITE_4(sc, MTK_EINT_POL_CLR(irq), mask);
		MTK_EINT_WRITE_4(sc, MTK_EINT_SENS_CLR(irq), mask);
		break;
	case GPIO_INTR_EDGE_BOTH:
		sc->gpio_pic_irqsrc[irq].edge_both = true;
		/* pol is setted on mtk_gpio_pic_enable_intr */
		MTK_EINT_WRITE_4(sc, MTK_EINT_SENS_SET(irq), mask);
		break;
	}

//TODO 
/* Switch the pin to interrupt mode */

	val = MTK_EINT_READ_4(sc, MTK_EINT_DO_EN(irq));
	val |= mask;
	MTK_EINT_WRITE_4(sc, MTK_EINT_DO_EN(irq), val);

	MTK_EINT_WRITE_4(sc, MTK_EINT_ACK(irq), mask);
	MTK_GPIO_UNLOCK(sc);

	return (0);
}

static int
mtk_gpio_pic_teardown_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{
	struct mtk_gpio_softc *sc;
	uint32_t mask, val;
	u_int irq;

	sc = device_get_softc(dev);
	irq = ((struct gpio_irqsrc *)isrc)->irq;
	mask = 1U << (irq & 0x1f);
	if (isrc->isrc_handlers == 0) {
		MTK_GPIO_LOCK(sc);
		MTK_EINT_WRITE_4(sc, MTK_EINT_ACK(irq), mask);
		val = MTK_EINT_READ_4(sc, MTK_EINT_DO_EN(irq));
		val &= ~mask;
		MTK_EINT_WRITE_4(sc, MTK_EINT_DO_EN(irq), val);
//TODO
/* Switch back the pin to it's original function */
		MTK_GPIO_UNLOCK(sc);
	}

	return (0);
}

static void
mtk_gpio_pic_post_filter(device_t dev, struct intr_irqsrc *isrc)
{
	struct mtk_gpio_softc *sc;
	uint32_t mask;
	u_int irq;

	sc = device_get_softc(dev);
	irq = ((struct gpio_irqsrc *)isrc)->irq;
	mask = 1U << (irq & 0x1f);

	arm_irq_memory_barrier(0);
	MTK_EINT_WRITE_4(sc, MTK_EINT_ACK(irq), mask);
}

static void
mtk_gpio_pic_post_ithread(device_t dev, struct intr_irqsrc *isrc)
{
	struct mtk_gpio_softc *sc;
	uint32_t mask;
	u_int irq;

	sc = device_get_softc(dev);
	irq = ((struct gpio_irqsrc *)isrc)->irq;
	mask = 1U << (irq & 0x1f);

	arm_irq_memory_barrier(0);
	MTK_EINT_WRITE_4(sc, MTK_EINT_ACK(irq), mask);
	mtk_gpio_pic_enable_intr(dev, isrc);
/* If the pin value changes faster than the interrupt is processed,
 * we lose one edge. To avoid this we can call a software interrupt
 */ 
#if 0
	if (sc->gpio_pic_irqsrc[irq].edge_both) {
//TODO check if the previous pin value is the same as the current one.
//if not, rise software interrupt
		MTK_EINT_WRITE_4(sc, MTK_EINT_SOFT_SET(irq), mask);
	}
#endif
}


static void
mtk_gpio_pic_pre_ithread(device_t dev, struct intr_irqsrc *isrc)
{
	struct mtk_gpio_softc *sc;

	sc = device_get_softc(dev);

	MTK_GPIO_LOCK(sc);
	mtk_gpio_pic_disable_intr_locked(sc, isrc);
	MTK_GPIO_UNLOCK(sc);
}

/*
 * OFWBUS Interface
 */
static phandle_t
mtk_gpio_get_node(device_t dev, device_t bus)
{
	/* We only have one child, the GPIO bus, which needs our own node. */
	return (ofw_bus_get_node(dev));
}

static device_method_t mtk_gpio_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		mtk_gpio_probe),
	DEVMETHOD(device_attach,	mtk_gpio_attach),
	DEVMETHOD(device_detach,	mtk_gpio_detach),

	/* Bus interface */
	DEVMETHOD(bus_setup_intr,	bus_generic_setup_intr),
	DEVMETHOD(bus_teardown_intr,	bus_generic_teardown_intr),

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
