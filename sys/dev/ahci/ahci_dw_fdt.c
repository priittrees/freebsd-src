/*-
 * SPDX-License-Identifier: BSD-2-Clause
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

/* AHCI controller driver for NXP QorIQ Layerscape SoCs. */

#include <sys/cdefs.h>
#include <sys/stdint.h>
#include <sys/stddef.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/sysctl.h>
#include <sys/rman.h>
#include <sys/unistd.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/ahci/ahci.h>

#include <dev/extres/clk/clk.h>
#include <dev/extres/phy/phy.h>
#include <dev/extres/hwreset/hwreset.h>

#define DW_AHCI_RK3568		1
#define DW_AHCI_GENERIC		20

#define DW_AHCI_MAX_PHYS	8
#define DW_AHCI_MAX_CLKS	8

struct ahci_dw_fdt_controller {
	struct ahci_controller	ctrl;	/* Must be the first field. */
	clk_t 			clks[DW_AHCI_MAX_CLKS];
	phy_t			phys[DW_AHCI_MAX_PHYS];
	hwreset_array_t         hwreset;
	int			soc_type;
};

static const struct ofw_compat_data ahci_dw_fdt_compat_data[] = {
	{"rockchip,rk3568-dwc-ahci",	DW_AHCI_RK3568},
	{"snps,dwc-ahci",		DW_AHCI_GENERIC},
	{NULL,			0}
};

static void
ahci_dw_fdt_extres_release(device_t dev)
{
        struct ahci_dw_fdt_controller *ctrl;
        
        int i = 0;

        ctrl = device_get_softc(dev);
/* release clocks */
        while (i < DW_AHCI_MAX_CLKS && ctrl->clks[i]) {
        	clk_release(ctrl->clks[i]);
        	ctrl->clks[i] = NULL;
        	i++;
         }
/* release phys */         
	i = 0;         
        while (i < DW_AHCI_MAX_PHYS && ctrl->phys[i]) {
        	phy_release(ctrl->phys[i]);
        	ctrl->phys[i] = NULL;
        	i++;
         }         
/* release resets */
        if (ctrl->hwreset != NULL)
                hwreset_array_release(ctrl->hwreset);
/* resource mapping */        

}



static int
ahci_dw_fdt_phy_init(device_t dev) 
{
	struct ahci_dw_fdt_controller *ctrl;
	int error, i, ncells;
	phandle_t node = ofw_bus_get_node(dev);

	ctrl = device_get_softc(dev);
	if(!OF_hasprop(node, "phys")) 
		return (0);
        error = ofw_bus_parse_xref_list_get_length(node, "phys",
            "#phy-cells", &ncells);

        if (error != 0 || ncells < 1) {
                device_printf(dev, "couldn't find enough phys (need >= 1)\n");
                error = ENXIO;
                goto out;
        }

	for (i = 0;i < ncells;i++) {
		error = phy_get_by_ofw_idx(dev, node, i, &ctrl->phys[i]);
		if (error) {
			device_printf(dev, "Could not get phy at #%d\n", i);
			error = ENXIO;
			goto out;
		 }

		error = phy_enable(ctrl->phys[i]);
		if (error) {
			device_printf(dev, "Could not enable phy #%d\n", i);
			error = ENXIO;
			goto out;
		 }
	}
	return (0);
out:
	return (error);
}

static int
ahci_dw_fdt_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_search_compatible(dev, ahci_dw_fdt_compat_data)->ocd_data)
		return (ENXIO);

	device_set_desc(dev, "DesignWare AHCI controller");
	return (BUS_PROBE_DEFAULT);
}

static int
ahci_dw_fdt_attach(device_t dev)
{
	struct ahci_dw_fdt_controller *ctrl;
	struct ahci_controller *ahci;
	phandle_t node;
	int ret, ncells, i;

	node = ofw_bus_get_node(dev);
	ctrl = device_get_softc(dev);
	ctrl->soc_type =
	    ofw_bus_search_compatible(dev, ahci_dw_fdt_compat_data)->ocd_data;
	ahci = &ctrl->ctrl;
	ahci->dev = dev;
	ahci->r_rid = 0;
	ahci->quirks = AHCI_Q_FORCE_PI|AHCI_Q_RESTORE_CAP;
	
	if(OF_hasprop(node, "resets")) {
		ret = hwreset_array_get_ofw(dev, 0, &ctrl->hwreset);
	        if (ret != 0) {
        	        device_printf(dev, "Cannot get resets\n");
        	        goto fail;
	        }
	}        
        
        ret = ofw_bus_parse_xref_list_get_length(node, "clocks",
            "#clock-cells", &ncells);
        if (ret != 0 || ncells < 2) {
                device_printf(dev, "couldn't find enough clocks (need >=2)\n");
                ret = ENXIO;
                goto fail;
        }

	for (i = 0;i < ncells;i++) {
		ret = clk_get_by_ofw_index(dev, node, i, &ctrl->clks[i]);
		if (ret) {
			device_printf(dev, "Could not get clock #%d\n", i);
			ret = ENXIO;
			goto fail;
		 }

		ret = clk_enable(ctrl->clks[i]);
		if (ret) {
			device_printf(dev, "Could not enable clock #%s\n", 
			clk_get_name(ctrl->clks[i]));
			ret = ENXIO;
			goto fail;
		 }
	}
	if(ctrl->hwreset) {
	        ret = hwreset_array_assert(ctrl->hwreset);
	        if (ret != 0) {
                	device_printf(dev, "Cannot assert reset\n");
                	goto fail;
	        }
	        DELAY(500);
	        ret = hwreset_array_deassert(ctrl->hwreset);
	        if (ret != 0) {
                	device_printf(dev, "Cannot deassert reset\n");
                	goto fail;
	        }	        
	}
	
	ahci->r_mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &ahci->r_rid, RF_ACTIVE);
	if (!ahci->r_mem) {
		device_printf(dev,
		    "Could not allocate resources for controller\n");
		ret = ENXIO;
		goto fail;
	}


	/* Setup controller defaults. */
	ahci->numirqs = 1;
	ret = ahci_dw_fdt_phy_init(dev);
	if (ret) 
		goto fail;

	/* Reset controller. */
	ret = ahci_ctlr_reset(dev);
	if (ret)
		goto fail;

	ret = ahci_attach(dev);
	if (ret) {
		device_printf(dev,
		    "Could not initialize AHCI, error: %d\n", ret);
		goto fail;
	}
	return (0);

fail:
	if (ahci->r_mem)
		bus_free_resource(dev, SYS_RES_MEMORY, ahci->r_mem);
	ahci_dw_fdt_extres_release(dev);
	return (ret);
}

static int
ahci_dw_fdt_detach(device_t dev)
{
	ahci_dw_fdt_extres_release(dev);
	return ahci_detach(dev);
}

static const device_method_t ahci_dw_fdt_methods[] = {
	DEVMETHOD(device_probe,			ahci_dw_fdt_probe),
	DEVMETHOD(device_attach,		ahci_dw_fdt_attach),
	DEVMETHOD(device_detach,		ahci_dw_fdt_detach),
	DEVMETHOD(bus_alloc_resource,		ahci_alloc_resource),
	DEVMETHOD(bus_release_resource,		ahci_release_resource),
	DEVMETHOD(bus_setup_intr,   		ahci_setup_intr),
	DEVMETHOD(bus_teardown_intr,		ahci_teardown_intr),
	DEVMETHOD(bus_print_child,		ahci_print_child),
	DEVMETHOD(bus_child_location,		ahci_child_location),
	DEVMETHOD(bus_get_dma_tag,  		ahci_get_dma_tag),
	DEVMETHOD_END
};

static driver_t ahci_dw_fdt_driver = {
	"ahci",
	ahci_dw_fdt_methods,
	sizeof(struct ahci_dw_fdt_controller),
};

DRIVER_MODULE(ahci_dw, simplebus, ahci_dw_fdt_driver, NULL, NULL);
DRIVER_MODULE(ahci_dw, ofwbus, ahci_dw_fdt_driver, NULL, NULL);
