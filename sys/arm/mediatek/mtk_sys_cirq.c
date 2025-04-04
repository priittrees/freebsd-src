/*-
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

//#include <sys/conf.h>
//#include <machine/fdt.h>
//#include <machine/resource.h>

#include <sys/cdefs.h>
#include "opt_platform.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/proc.h>
#include <sys/rman.h>
#include <machine/bus.h>
#include <machine/intr.h> 

#include <dev/fdt/fdt_intr.h>
#include <dev/ofw/openfirm.h> 
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dt-bindings/interrupt-controller/irq.h>

#include "pic_if.h"


#define cirq_read_4(_sc, _reg)          bus_read_4(_sc->sc_mem_res, _reg)
#define cirq_write_4(_sc, _reg, _val)   bus_write_4(_sc->sc_mem_res, _reg, _val)

static struct ofw_compat_data compat_data[] = {
	{"mediatek,mt6577-sysirq", 	1},
	{NULL,				0}
};

struct mtk_sys_cirq_sc {
	device_t		sc_dev;
	struct resource		*sc_mem_res;
	device_t		sc_parent;

	struct intr_map_data_fdt *parent_map_data;
};

static struct intr_map_data *
mtk_sys_cirq_gicp_convert_map_data(struct mtk_sys_cirq_sc *sc,
    struct intr_map_data *data)
{
	struct intr_map_data_fdt *daf;
	
	daf = (struct intr_map_data_fdt *)data;

	/* We only support GIC forward for now */
	if (daf->ncells != 3)
		return (NULL);

	/* Check if this is a GIC_SPI type */
	if (daf->cells[0] != 0)
		return (NULL);

	/* Construct GIC compatible mapping. */
	sc->parent_map_data->ncells = 3;
	sc->parent_map_data->cells[0] = 0; /* SPI */
	sc->parent_map_data->cells[1] = daf->cells[1];
	sc->parent_map_data->cells[2] = IRQ_TYPE_LEVEL_HIGH;

	return ((struct intr_map_data *)sc->parent_map_data);
	
}

static int
mtk_sys_cirq_activate_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{
	struct mtk_sys_cirq_sc *sc = device_get_softc(dev);
	data = mtk_sys_cirq_gicp_convert_map_data(sc, data);
	if (data == NULL)
		return (EINVAL);

	return (PIC_ACTIVATE_INTR(sc->sc_parent, isrc, res, data));
}

static void
mtk_sys_cirq_disable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	struct mtk_sys_cirq_sc *sc = device_get_softc(dev);

	PIC_DISABLE_INTR(sc->sc_parent, isrc);
}

static void
mtk_sys_cirq_enable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	struct mtk_sys_cirq_sc *sc = device_get_softc(dev);

	PIC_ENABLE_INTR(sc->sc_parent, isrc);
}

static int
mtk_sys_cirq_map_intr(device_t dev, struct intr_map_data *data,
    struct intr_irqsrc **isrcp)
{
	struct mtk_sys_cirq_sc *sc = device_get_softc(dev);
	int ret;

	if (data->type != INTR_MAP_DATA_FDT)
		return (ENOTSUP);

	data = mtk_sys_cirq_gicp_convert_map_data(sc, data);
	if (data == NULL)
		return (EINVAL);

	ret = PIC_MAP_INTR(sc->sc_parent, data, isrcp);
	(*isrcp)->isrc_dev = sc->sc_dev;
	return ret;
}

static int
mtk_sys_cirq_deactivate_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{
	struct mtk_sys_cirq_sc *sc = device_get_softc(dev);
	data = mtk_sys_cirq_gicp_convert_map_data(sc, data);
	if (data == NULL)
		return (EINVAL);

	return (PIC_DEACTIVATE_INTR(sc->sc_parent, isrc, res, data));
}

static int
mtk_sys_cirq_setup_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{
	struct mtk_sys_cirq_sc *sc = device_get_softc(dev);
	data = mtk_sys_cirq_gicp_convert_map_data(sc, data);
	if (data == NULL)
		return (EINVAL);


	
	uint32_t reg, val;
	u_int irq;
	struct intr_map_data_fdt *daf;
	daf = (struct intr_map_data_fdt *)data;
	irq = daf->cells[1];

	reg = irq / 32 * 4;
	val = cirq_read_4(sc, reg);
	val |= 1 << (irq & 0x1f);
	cirq_write_4(sc, reg, val);

	return (PIC_SETUP_INTR(sc->sc_parent, isrc, res, data));
}

static int
mtk_sys_cirq_teardown_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{
	struct mtk_sys_cirq_sc *sc = device_get_softc(dev);
	data = mtk_sys_cirq_gicp_convert_map_data(sc, data);
	if (data == NULL)
		return (EINVAL);

	return (PIC_TEARDOWN_INTR(sc->sc_parent, isrc, res, data));
}

static void
mtk_sys_cirq_pre_ithread(device_t dev, struct intr_irqsrc *isrc)
{
	struct mtk_sys_cirq_sc *sc = device_get_softc(dev);

	PIC_PRE_ITHREAD(sc->sc_parent, isrc);
}

static void
mtk_sys_cirq_post_ithread(device_t dev, struct intr_irqsrc *isrc)
{
	struct mtk_sys_cirq_sc *sc = device_get_softc(dev);

	PIC_POST_ITHREAD(sc->sc_parent, isrc);
}

static void
mtk_sys_cirq_post_filter(device_t dev, struct intr_irqsrc *isrc)
{
	struct mtk_sys_cirq_sc *sc = device_get_softc(dev);

	PIC_POST_FILTER(sc->sc_parent, isrc);
}

#ifdef SMP
static int
mtk_sys_cirq_bind_intr(device_t dev, struct intr_irqsrc *isrc)
{
	struct mtk_sys_cirq_sc *sc = device_get_softc(dev);

	return (PIC_BIND_INTR(sc->sc_parent, isrc));
}
#endif

static int
mtk_sys_cirq_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	return (BUS_PROBE_DEFAULT);
}

static int
mtk_sys_cirq_detach(device_t dev)
{
	struct mtk_sys_cirq_sc *sc;

	sc = device_get_softc(dev);
	if (sc->sc_mem_res != NULL) {
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->sc_mem_res);
		sc->sc_mem_res = NULL;
	}
	return (0);
}

static int
mtk_sys_cirq_attach(device_t dev)
{
	struct mtk_sys_cirq_sc *sc;
	phandle_t node, intr_parent, xref;
	int rid;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;
	node = ofw_bus_get_node(dev);

	/* Look for our parent */
	if ((intr_parent = ofw_bus_find_iparent(node)) == 0) {
		device_printf(dev,
		    "Cannot find our parent interrupt controller\n");
		return (ENXIO);
	}
	if ((sc->sc_parent = OF_device_from_xref(intr_parent)) == NULL) {
		device_printf(dev,
		    "cannot find parent interrupt controller device\n");
		return (ENXIO);
	}

	rid = 0;
	sc->sc_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->sc_mem_res == NULL) {
		device_printf(dev, "can't allocate resources\n");
		return (ENXIO);
	}

	/* Register ourself as a interrupt controller */
	xref = OF_xref_from_node(node);
	if (intr_pic_register(dev, xref) == NULL) {
		device_printf(dev, "Cannot register GICP\n");
		return (ENXIO);
	}

	/* Allocate GIC compatible mapping entry (3 cells) */
	sc->parent_map_data = (struct intr_map_data_fdt *)intr_alloc_map_data(
	    INTR_MAP_DATA_FDT, sizeof(struct intr_map_data_fdt) +
	    + 3 * sizeof(phandle_t), M_WAITOK | M_ZERO);

	/* Register ourself to device can find us */
	OF_device_register_xref(xref, dev);

	return (0);
}

static device_method_t mtk_sys_cirq_methods[] = {
	DEVMETHOD(device_probe,		mtk_sys_cirq_probe),
	DEVMETHOD(device_attach,	mtk_sys_cirq_attach),
	DEVMETHOD(device_detach,	mtk_sys_cirq_detach),

	/* Interrupt controller interface */
	DEVMETHOD(pic_activate_intr,	mtk_sys_cirq_activate_intr),
	DEVMETHOD(pic_disable_intr,	mtk_sys_cirq_disable_intr),
	DEVMETHOD(pic_enable_intr,	mtk_sys_cirq_enable_intr),
	DEVMETHOD(pic_map_intr,		mtk_sys_cirq_map_intr),
	DEVMETHOD(pic_deactivate_intr,	mtk_sys_cirq_deactivate_intr),
	DEVMETHOD(pic_setup_intr,	mtk_sys_cirq_setup_intr),
	DEVMETHOD(pic_teardown_intr,	mtk_sys_cirq_teardown_intr),
	DEVMETHOD(pic_pre_ithread,	mtk_sys_cirq_pre_ithread),
	DEVMETHOD(pic_post_ithread,	mtk_sys_cirq_post_ithread),
	DEVMETHOD(pic_post_filter,	mtk_sys_cirq_post_filter),
#ifdef SMP
	DEVMETHOD(pic_bind_intr,	mtk_sys_cirq_bind_intr),
#endif
	DEVMETHOD_END
};

DEFINE_CLASS_0(mtk_sys_cirq, mtk_sys_cirq_driver, mtk_sys_cirq_methods,
    sizeof(struct mtk_sys_cirq_sc));
EARLY_DRIVER_MODULE(mtk_sys_cirq, simplebus, mtk_sys_cirq_driver, NULL, NULL,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_MIDDLE + 1);
