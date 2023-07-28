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
// #include <sys/taskqueue.h>
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

#include "bcmswitch_var.h"
#include "bcmswitch_reg.h"
#include "bcmswitch_ps_phy.h"
#include "etherswitch_if.h"
#include "miibus_if.h"
#include "mdio_if.h"


MALLOC_DECLARE(M_BCMSWITCH);
MALLOC_DEFINE(M_BCMSWITCH, "bcmswitch", "bcmswitch switch");

static etherswitch_info_t etherswitch_info = {
        .es_nports =            0,
        .es_nvlangroups =       0,
        .es_vlan_caps =         ETHERSWITCH_VLAN_PORT | ETHERSWITCH_VLAN_DOT1Q,
        .es_name =              "Broadcom series switch"
};

static void bcmswitch_identify(driver_t *, device_t);
static int bcmswitch_probe(device_t);
static int bcmswitch_attach(device_t);
static int bcmswitch_detach(device_t);
static int bcmswitch_readphy(device_t, int, int);
static int bcmswitch_writephy(device_t, int, int, int);
static int bcmswitch_readphy_locked(device_t, int, int);
static int bcmswitch_writephy_locked(device_t, int, int, int);
static etherswitch_info_t* bcmswitch_getinfo(device_t);
static int bcmswitch_getconf(device_t, etherswitch_conf_t *);
static int bcmswitch_setconf(device_t, etherswitch_conf_t *);
static void bcmswitch_lock(device_t);
static void bcmswitch_unlock(device_t);
static int bcmswitch_getport(device_t, etherswitch_port_t *);
static int bcmswitch_setport(device_t, etherswitch_port_t *);
static int bcmswitch_readreg_wrapper(device_t, int);
static int bcmswitch_writereg_wrapper(device_t, int, int);
static int bcmswitch_setvgroup_wrapper(device_t, etherswitch_vlangroup_t *);
static int bcmswitch_getvgroup_wrapper(device_t, etherswitch_vlangroup_t *);
static int bcmswitch_setvgroup(device_t, etherswitch_vlangroup_t *);
static int bcmswitch_getvgroup(device_t, etherswitch_vlangroup_t *);

//static void bcmswitch_tick(void *, int);

static int bcmswitch_ifmedia_upd(struct ifnet *);
static void bcmswitch_ifmedia_sts(struct ifnet *, struct ifmediareq *);

static int bcmswitch_get_pvid(bcmswitch_softc_t *sc, int port, int *pvid);
static void bcmswitch_set_pvid(bcmswitch_softc_t *sc, int port, int pvid);

static int bcmswitch_set_vlan_mode(bcmswitch_softc_t *, uint32_t);
static int bcmswitch_get_port_vlan(bcmswitch_softc_t *, etherswitch_vlangroup_t *);

static __inline bool bcmswitch_is_cpuport(bcmswitch_softc_t *, int);
static __inline bool bcmswitch_is_fixedport(bcmswitch_softc_t *, int);
static __inline bool bcmswitch_is_phyport(bcmswitch_softc_t *, int);
static __inline bool bcmswitch_is_portenabled(bcmswitch_softc_t *, int);
static __inline struct mii_data *bcmswitch_miiforphy(bcmswitch_softc_t *,
    unsigned int);


static void bcmswitch_co_tick(void *arg);

static device_method_t bcmswitch_methods[] = {
	/* device interface */
	DEVMETHOD(device_identify,		bcmswitch_identify),
	DEVMETHOD(device_probe,			bcmswitch_probe),
	DEVMETHOD(device_attach,		bcmswitch_attach),
	DEVMETHOD(device_detach,		bcmswitch_detach),

	/* bus interface */
	DEVMETHOD(bus_add_child,		device_add_child_ordered),

	/* mii interface */
	DEVMETHOD(miibus_readreg,		bcmswitch_readphy_locked),
	DEVMETHOD(miibus_writereg,		bcmswitch_writephy_locked),

	/* etherswitch interface */
	DEVMETHOD(etherswitch_getinfo,		bcmswitch_getinfo),
	DEVMETHOD(etherswitch_getconf,		bcmswitch_getconf),
	DEVMETHOD(etherswitch_setconf,		bcmswitch_setconf),
	DEVMETHOD(etherswitch_lock,		bcmswitch_lock),
	DEVMETHOD(etherswitch_unlock,		bcmswitch_unlock),
	DEVMETHOD(etherswitch_getport,		bcmswitch_getport),
	DEVMETHOD(etherswitch_setport,		bcmswitch_setport),
	DEVMETHOD(etherswitch_readreg,		bcmswitch_readreg_wrapper),
	DEVMETHOD(etherswitch_writereg,		bcmswitch_writereg_wrapper),
	DEVMETHOD(etherswitch_readphyreg,	bcmswitch_readphy),
	DEVMETHOD(etherswitch_writephyreg,	bcmswitch_writephy),
	DEVMETHOD(etherswitch_setvgroup,	bcmswitch_setvgroup_wrapper),
	DEVMETHOD(etherswitch_getvgroup,	bcmswitch_getvgroup_wrapper),

	DEVMETHOD_END
};

DEFINE_CLASS_0(bcmswitch, bcmswitch_driver, bcmswitch_methods, sizeof(bcmswitch_softc_t));
DRIVER_MODULE(bcmswitch, mdio, bcmswitch_driver, 0, 0);
DRIVER_MODULE(etherswitch, bcmswitch, etherswitch_driver, 0, 0);
DRIVER_MODULE(miibus, bcmswitch, miibus_driver, 0, 0);
MODULE_DEPEND(bcmswitch, mdio, 1, 1, 1);

static void
bcmswitch_identify(driver_t *driver, device_t parent)
{
	if (device_find_child(parent, "bcmswitch", -1) == NULL)
		BUS_ADD_CHILD(parent, 0, "bcmswitch", -1);
}

static int
bcmswitch_probe(device_t dev)
{
	bcmswitch_softc_t *sc;
	phandle_t switch_node;

	sc = device_get_softc(dev);
	switch_node = ofw_bus_find_compatible(OF_finddevice("/"),
	    "brcm,bcm53125");
	if (switch_node == 0) {
			return (ENXIO);
	}

	if (bootverbose)
		device_printf(dev, "Found switch_node: 0x%x\n", switch_node);

	sc->dev = dev;
	sc->node = switch_node;

	sc->num_ports = BCMSWITCH_MAX_PORTS;

	device_set_desc(dev, "Broadcom switch driver");

	return (BUS_PROBE_DEFAULT);
}

static int
bcmswitch_parse_fixed_link(bcmswitch_softc_t *sc, phandle_t node, uint32_t port)
{
	int speed;
	phandle_t fixed_link;

	fixed_link = ofw_bus_find_child(node, "fixed-link");

	if (fixed_link != 0) {
		sc->fixed_mask |= (1 << port);

		if (OF_getencprop(fixed_link,
		    "speed", &speed, sizeof(speed)) < 0) {
			device_printf(sc->dev,
			    "Port %d has a fixed-link node without a speed "
			    "property\n", port);
			return (ENXIO);
		}
	}

	return (0);
}

static int
bcmswitch_parse_ethernet(bcmswitch_softc_t *sc, phandle_t port_handle, uint32_t port) {
	phandle_t switch_eth, switch_eth_handle;

	if (OF_getencprop(port_handle, "ethernet", (void*)&switch_eth_handle,
	    sizeof(switch_eth_handle)) > 0) {
		if (switch_eth_handle > 0) {
			switch_eth = OF_node_from_xref(switch_eth_handle);

			device_printf(sc->dev, "CPU port at %d\n", port);
			sc->cpuports_mask |= (1 << port);

			return (bcmswitch_parse_fixed_link(sc, switch_eth, port));
		} else
			device_printf(sc->dev,
				"Port %d has ethernet property but it points "
				"to an invalid location\n", port);
	}

	return (0);
}

static int
bcmswitch_parse_child_fdt(bcmswitch_softc_t *sc, phandle_t child, int *pport)
{
	uint32_t port;

	if (pport == NULL)
		return (ENXIO);

	if (OF_getencprop(child, "reg", (void *)&port, sizeof(port)) < 0)
		return (ENXIO);
	if (port >= sc->num_ports)
		return (ENXIO);
	*pport = port;

	if (bcmswitch_parse_fixed_link(sc, child, port) != 0)
		return (ENXIO);

	if (bcmswitch_parse_ethernet(sc, child, port) != 0)
		return (ENXIO);

        if ((sc->fixed_mask & (1 << port)) != 0)
                device_printf(sc->dev, "fixed port at %d\n", port);
        else
                device_printf(sc->dev, "PHY at port %d\n", port);

	return (0);
}

static int
bcmswitch_init_interface(bcmswitch_softc_t *sc, int port)
{
	char name[IFNAMSIZ];
	
	snprintf(name, IFNAMSIZ, "%sport", device_get_nameunit(sc->dev));
	
	sc->ifp[port] = if_alloc(IFT_ETHER);
	if (sc->ifp[port] == NULL)
		return (ENOMEM);
	sc->ifp[port]->if_softc = sc;
	sc->ifp[port]->if_flags |= IFF_UP | IFF_BROADCAST |
	    IFF_DRV_RUNNING | IFF_SIMPLEX;
	sc->ifname[port] = malloc(strlen(name) + 1, M_BCMSWITCH, M_NOWAIT);
	if (sc->ifname[port] == NULL) {
		if_free(sc->ifp[port]);
		return (ENOMEM);
	}
	memcpy(sc->ifname[port], name, strlen(name) + 1);
	if_initname(sc->ifp[port], sc->ifname[port], port);
	
	return (0);
}

static int
bcmswitch_attach_miibus(bcmswitch_softc_t *sc, int port)
{
	int err;
	
	err = mii_attach(sc->dev, &sc->miibus[port], sc->ifp[port],
	    bcmswitch_ifmedia_upd, bcmswitch_ifmedia_sts, BMSR_DEFCAPMASK,
	    port + sc->phy_base, MII_OFFSET_ANY, 0);
	if (err != 0)
		return (err);
	
	return (0);
}

/* Managed switch implementations should be configured to disable forwarding
 * on power-on to allow the processor to configure the internal address table
 * and other parameters before frame forwarding is enabled.
 */

static void brc53sw_forward(struct bcmswitch_softc *sc, bool enable)
{
	uint16_t val;

	val = bcmswitch_readreg16(sc, BCMSWITCH_CTRL_PAGE, BCMSWITCH_SWITCH_MODE);

	if (enable)
		val |= BCMSWITCH_SW_FWDG_EN;
	else
		val &= ~BCMSWITCH_SW_FWDG_EN;

	bcmswitch_writereg16(sc, BCMSWITCH_CTRL_PAGE, BCMSWITCH_SWITCH_MODE, val);

}

static void
bcmswitch_port_vlan_assign(bcmswitch_softc_t *sc, int port, uint32_t members)
{
	bcmswitch_writereg16(sc, BCMSWITCH_PVLAN_PAGE,
	    BCMSWITCH_PVLAN_PORT_MASK(port), members);
}

static int
bcmswitch_set_port_vlan(bcmswitch_softc_t *sc, etherswitch_vlangroup_t *vg)
{
	uint32_t port;
	port = vg->es_vlangroup;
	if (port > sc->num_ports)
		return (EINVAL);
	if (vg->es_member_ports != vg->es_untagged_ports) {
		device_printf(sc->dev, "Tagged ports not supported.\n");
		return (EINVAL);
	}
	bcmswitch_port_vlan_assign(sc, port, vg->es_untagged_ports);
	vg->es_vid = port | ETHERSWITCH_VID_VALID;
	return (0);
}

static int
bcmswitch_get_port_vlan(bcmswitch_softc_t *sc, etherswitch_vlangroup_t *vg) 
{
	uint32_t port, reg;

	port = vg->es_vlangroup;
	if (port > sc->num_ports)
		return (EINVAL);

	if (!bcmswitch_is_portenabled(sc, port)) {
		vg->es_vid = port;
		return (0);
	}

	reg = bcmswitch_readreg16(sc, BCMSWITCH_PVLAN_PAGE,
	    BCMSWITCH_PVLAN_PORT_MASK(port));

	vg->es_untagged_ports = vg->es_member_ports = reg;
	vg->es_vid = port | ETHERSWITCH_VID_VALID;
	vg->es_fid = 0;

	return (0);
}

// ----------------------- ETHERSWITCH_VLAN_DOT1Q ---------------------  // 

/* If the MII port is configured as a management port, then the tag is not
 * stripped even if the untag bit is set.
 */

static void 
bcmswitch_init_dot1q(struct bcmswitch_softc *sc)
{
	uint16_t res;

	res = bcmswitch_readreg16(sc, BCMSWITCH_VLAN_PAGE, BCMSWITCH_VLAN_CTRL0);
	res |= VC0_VLAN_EN | VC0_VID_CHK_EN | VC0_VID_HASH_VID;
	bcmswitch_writereg16(sc, BCMSWITCH_VLAN_PAGE, BCMSWITCH_VLAN_CTRL0, res);

	res = bcmswitch_readreg16(sc, BCMSWITCH_VLAN_PAGE, BCMSWITCH_VLAN_CTRL1);
	res |= VC1_RX_MCST_UNTAG_EN | VC1_RX_MCST_FWD_EN;
	bcmswitch_writereg16(sc, BCMSWITCH_VLAN_PAGE, BCMSWITCH_VLAN_CTRL1, res);

	bcmswitch_writereg16(sc, BCMSWITCH_VLAN_PAGE, BCMSWITCH_VLAN_CTRL3, 0);

	res = bcmswitch_readreg16(sc, BCMSWITCH_VLAN_PAGE, BCMSWITCH_VLAN_CTRL4);
	res &= ~VC4_ING_VID_CHECK_MASK;
	res |= VC4_ING_VID_VIO_DROP << VC4_ING_VID_CHECK_S;
	bcmswitch_writereg16(sc, BCMSWITCH_VLAN_PAGE, BCMSWITCH_VLAN_CTRL4, res);

	res = bcmswitch_readreg16(sc, BCMSWITCH_VLAN_PAGE, BCMSWITCH_VLAN_CTRL5);
	res |= VC5_DROP_VTABLE_MISS;
	bcmswitch_writereg16(sc, BCMSWITCH_VLAN_PAGE, BCMSWITCH_VLAN_CTRL5, res);
}

static int
bcmswitch_vlan_table_ctrl(struct bcmswitch_softc *sc, uint8_t op)
{
	uint16_t res;
	bcmswitch_writereg16(sc, BCMSWITCH_ARLIO_PAGE,
	    BCMSWITCH_VT_ACCESS, VTA_START_CMD | op);

	for (int i = 0; i < 10; i++) {
		res = bcmswitch_readreg16(sc, BCMSWITCH_ARLIO_PAGE,
		    BCMSWITCH_VT_ACCESS);
		if (!(res & VTA_START_CMD))
			return 0;

		DELAY(100);
	}

	return EIO;
}

static void 
bcmswitch_update_vlan_entry(struct bcmswitch_softc *sc, uint16_t vid, 
    uint16_t members, uint16_t untag)
{
        uint32_t res;

        bcmswitch_writereg16(sc, BCMSWITCH_ARLIO_PAGE, BCMSWITCH_VT_INDEX, vid);
        res = (untag << VTE_UNTAG_S) | members;
        bcmswitch_writereg32(sc, BCMSWITCH_ARLIO_PAGE, BCMSWITCH_VT_ENTRY, res);
        bcmswitch_vlan_table_ctrl(sc, VTA_CMD_WRITE);
}

static int
bcmswitch_set_dot1q_vlan(struct bcmswitch_softc *sc, etherswitch_vlangroup_t *vg)
{
	int i, vlan;

	vlan = vg->es_vid & ETHERSWITCH_VID_MASK;

	/* Set VLAN to '0' removes it from table. */
	if (vlan == 0) {
		bcmswitch_update_vlan_entry(sc,
				sc->vlans[vg->es_vlangroup], 0, 0);
		sc->vlans[vg->es_vlangroup] = 0;
		return (0);
	}

	/* Is this VLAN already in table ? */
	for (i = 0; i < etherswitch_info.es_nvlangroups; i++)
		if (i != vg->es_vlangroup && vlan == sc->vlans[i])
			return (EINVAL);

	sc->vlans[vg->es_vlangroup] = vlan;
	bcmswitch_update_vlan_entry(sc, vlan,
			vg->es_member_ports & sc->ports_mask,
			vg->es_untagged_ports & sc->ports_mask);

	return (0);
}

static int
bcmswitch_get_dot1q_vlan(struct bcmswitch_softc *sc, etherswitch_vlangroup_t *vg)
{
        uint32_t res;

        vg->es_fid = 0;
        vg->es_vid = sc->vlans[vg->es_vlangroup];
        vg->es_untagged_ports = vg->es_member_ports = 0;
        if (vg->es_vid == 0)
                return (0);

        bcmswitch_writereg16(sc, BCMSWITCH_ARLIO_PAGE, BCMSWITCH_VT_INDEX, vg->es_vid);
        bcmswitch_vlan_table_ctrl(sc, VTA_CMD_READ);
        res = bcmswitch_readreg32(sc, BCMSWITCH_ARLIO_PAGE, BCMSWITCH_VT_ENTRY);

        vg->es_vid |= ETHERSWITCH_VID_VALID;
        vg->es_untagged_ports = (res >> VTE_UNTAG_S) & 0x1FF;
        vg->es_member_ports = res & 0x1FF;

        return(0);
}

/* Default IEEE 802.1Q VLAN ID. 0x001 */
static void
bcmswitch_set_pvid(struct bcmswitch_softc *sc, int port, int pvid)
{
	int res;
	res = bcmswitch_readreg16(sc, BCMSWITCH_VLAN_PAGE,
			BCMSWITCH_VLAN_PORT_DEF_TAG(port));
	res = (res & ~(0xFFF));
	res |= pvid;
	bcmswitch_writereg16(sc, BCMSWITCH_VLAN_PAGE,
	    BCMSWITCH_VLAN_PORT_DEF_TAG(port), res);
}

static int
bcmswitch_get_pvid(struct bcmswitch_softc *sc, int port, int *pvid)
{

	if (pvid == NULL)
		return (ENXIO);

	*pvid = (bcmswitch_readreg16(sc, BCMSWITCH_VLAN_PAGE,
	    BCMSWITCH_VLAN_PORT_DEF_TAG(port)) & 0xFFF);

	return (0);

}

//---------------------------------------------------------------------//

static int 
bcmswitch_hw_setup(struct bcmswitch_softc *sc)
{
	// phandle_t child, ports;
	int port;
	uint16_t res;
	BCMSWITCH_LOCK_ASSERT(sc, SA_XLOCKED);
	for (port = 0; port < sc->num_ports; port++) {
                if (bcmswitch_is_cpuport(sc,port)) {
                        /* IMP port state override
                         * TODO Default speed is 1000M full duplex
                         */
                        res = bcmswitch_readreg16(sc, BCMSWITCH_CTRL_PAGE,
                            BCMSWITCH_IMP_OVERRIDE);
                        res |= BCMSWITCH_IMP_OVERRIDE_EN |
			    BCMSWITCH_IMP_OVERRIDE_LINK;
                        bcmswitch_writereg16(sc, BCMSWITCH_CTRL_PAGE,
                            BCMSWITCH_IMP_OVERRIDE, res);
                }
                else if (bcmswitch_is_fixedport(sc, port)) {
                        /* TODO  Port State Override Register 
                         * (Page 00h: Address 58h - 5Fh)
                         * Default speed is 1000M full duplex
                         */
                        res = bcmswitch_readreg16(sc, BCMSWITCH_CTRL_PAGE,
                            BCMSWITCH_PORT_OVERRIDE(port));
                        res |= BCMSWITCH_PORT_OVERRIDE_EN |
			    BCMSWITCH_PORT_OVERRIDE_LINK;
                        bcmswitch_writereg16(sc, BCMSWITCH_CTRL_PAGE,
                            BCMSWITCH_PORT_OVERRIDE(port), res);
                }
	}
	return (0);
}

//---------------------------------------------------------------------//
static int
bcmswitch_attach(device_t dev)
{

	bcmswitch_softc_t *sc;
	phandle_t child, ports;
	int err, port;

	sc = device_get_softc(dev);
	sc->dev = dev;

	sx_init(&sc->sx, "bcmswitch");

	BCMSWITCH_LOCK(sc);
	ports = ofw_bus_find_child(sc->node, "ports");
	//sc->sc_tq = taskqueue_create("bcmswitch_taskq", M_NOWAIT,
	//    taskqueue_thread_enqueue, &sc->sc_tq);

	// TIMEOUT_TASK_INIT(sc->sc_tq, &sc->sc_tt, 0, bcmswitch_tick, sc);
	// taskqueue_start_threads(&sc->sc_tq, 1, PI_NET, "%s taskq",
	//    device_get_nameunit(dev));

        if (ports == 0) {
                device_printf(dev, "failed to parse DTS: no ports found for "
		    "switch\n");
                BCMSWITCH_UNLOCK(sc);
                return (ENXIO);
        }

        /* Stop forwarding and set switch managed mode */
	/* TODO Software forwarding mode. Unmanaged/Managed */
	brc53sw_forward(sc, false);

	for (child = OF_child(ports); child != 0; child = OF_peer(child)) {
		err = bcmswitch_parse_child_fdt(sc, child, &port);
		if (err != 0) {
			device_printf(sc->dev, "failed to parse DTS\n");
			goto out_fail;
		}

		/* Port is in use. */
		sc->ports_mask |= (1 << port);

		err = bcmswitch_init_interface(sc, port);
		if (err != 0) {
			device_printf(sc->dev, "failed to init interface\n");
			goto out_fail;
		}

                if (bcmswitch_is_cpuport(sc,port)) {
                        /* Don't attach miibus at CPU/fixed ports */
                        continue;
                }

		err = bcmswitch_attach_miibus(sc, port);
		if (err != 0) {
			device_printf(sc->dev, "failed to attach miibus\n");
			goto out_fail;
		}

	}

	etherswitch_info.es_nports = sc->num_ports;

	/* Default to port vlan. */
	bcmswitch_set_vlan_mode(sc, ETHERSWITCH_VLAN_PORT);

	BCMSWITCH_UNLOCK(sc);

	bus_generic_probe(dev);
	bus_generic_attach(dev);

	// taskqueue_enqueue_timeout(sc->sc_tq, &sc->sc_tt, hz);
	
	callout_init(&sc->callout_tick, 0);
	bcmswitch_co_tick(sc);

	return (0);

out_fail:
	bcmswitch_detach(dev);

	return(err);
}


static int
bcmswitch_readphy(device_t dev, int phy, int reg)
{
	bcmswitch_softc_t *sc;
	uint32_t val;
	
	sc = device_get_softc(dev);
	BCMSWITCH_LOCK_ASSERT(sc, SA_UNLOCKED);

	BCMSWITCH_LOCK(sc);
	val = bcmswitch_readphy_locked(dev, phy, reg);
	BCMSWITCH_UNLOCK(sc);
	
	return (val);
}

static int
bcmswitch_readphy_locked(device_t dev, int phy, int reg)
{
	bcmswitch_softc_t *sc;
	uint32_t val;
	
	sc = device_get_softc(dev);
	BCMSWITCH_LOCK_ASSERT(sc, SA_XLOCKED);

	if (!bcmswitch_is_phyport(sc, phy) || (reg < 0 || reg >= 32))
	{
		device_printf(dev, "Wrong register address %0x.\n", reg);
		return (EINVAL);
	}

	val = MDIO_READ(dev, phy, reg);
	
	return (val);
}

static int
bcmswitch_writephy(device_t dev, int phy, int reg, int data)
{
	bcmswitch_softc_t *sc;
	uint32_t val;

	sc = device_get_softc(dev);
	BCMSWITCH_LOCK_ASSERT(sc, SA_UNLOCKED);

	BCMSWITCH_LOCK(sc);
	val = bcmswitch_writephy_locked(dev, phy, reg, data);
	BCMSWITCH_UNLOCK(sc);

	return (val);
}

static int
bcmswitch_writephy_locked(device_t dev, int phy, int reg, int val)
{
	bcmswitch_softc_t *sc;

	sc = device_get_softc(dev);
	BCMSWITCH_LOCK_ASSERT(sc, SA_XLOCKED);

	if (!bcmswitch_is_phyport(sc, phy) || (reg < 0 || reg >= 32))
	{
		device_printf(dev, "Wrong register address %0x.\n", reg);
		return (EINVAL);
	}

	MDIO_WRITE(dev, phy, reg, val);

	return (0);
}

static int
bcmswitch_detach(device_t dev)
{
	int phy;
	bcmswitch_softc_t *sc;

	sc = device_get_softc(dev);

	callout_drain(&sc->callout_tick);

	// if (device_is_attached(dev))
	//	taskqueue_drain_timeout(sc->sc_tq, &sc->sc_tt);

	//if (sc->sc_tq != NULL)
	//	taskqueue_free(sc->sc_tq);

	device_delete_children(dev);

	sx_destroy(&sc->sx);
	for (phy = 0; phy < sc->num_ports; phy++) {
		if (sc->ifp[phy] != NULL)
			if_free(sc->ifp[phy]);
		if (sc->ifname[phy] != NULL)
			free(sc->ifname[phy], M_BCMSWITCH);
	}

	return (0);
}

static etherswitch_info_t*
bcmswitch_getinfo(device_t dev)
{

        return (&etherswitch_info);
}

static int
bcmswitch_getconf(device_t dev, etherswitch_conf_t *conf)
{
	struct bcmswitch_softc *sc;

	/* Return the VLAN mode. */
	sc = device_get_softc(dev);
	conf->cmd = ETHERSWITCH_CONF_VLAN_MODE;
	conf->vlan_mode = sc->vlan_mode;

	return (0);
}

static int
bcmswitch_setconf(device_t dev, etherswitch_conf_t *conf)
{
	struct bcmswitch_softc *sc;

	/* Set the VLAN mode. */
	sc = device_get_softc(dev);
	if (conf->cmd & ETHERSWITCH_CONF_VLAN_MODE) {
		BCMSWITCH_LOCK(sc);
		bcmswitch_set_vlan_mode(sc, conf->vlan_mode);
		BCMSWITCH_UNLOCK(sc);
	}

	return (0);
}

static void
bcmswitch_lock(device_t dev)
{
	struct bcmswitch_softc *sc;

        sc = device_get_softc(dev);

        BCMSWITCH_LOCK_ASSERT(sc, SA_UNLOCKED);
        BCMSWITCH_LOCK(sc);
}

static void
bcmswitch_unlock(device_t dev)
{
	struct bcmswitch_softc *sc;

        sc = device_get_softc(dev);

	BCMSWITCH_LOCK_ASSERT(sc, SA_XLOCKED);
	BCMSWITCH_UNLOCK(sc);
}

static int
bcmswitch_getport(device_t dev, etherswitch_port_t *p)
{
	struct mii_data *mii;
	int err = 0;
	struct ifmediareq *ifmr;

	bcmswitch_softc_t *sc = device_get_softc(dev);
	BCMSWITCH_LOCK_ASSERT(sc, SA_UNLOCKED);

	if (p->es_port >= sc->num_ports || p->es_port < 0)
		return (EINVAL);
	if (!bcmswitch_is_portenabled(sc, p->es_port))
		return (0);

	BCMSWITCH_LOCK(sc);
	bcmswitch_get_pvid(sc, p->es_port, &p->es_pvid);

	p->es_flags = 0;

	if (bcmswitch_is_fixedport(sc, p->es_port)) {
		if (bcmswitch_is_cpuport(sc, p->es_port))
			p->es_flags |= ETHERSWITCH_PORT_CPU;
		ifmr = &p->es_ifmr;
		ifmr->ifm_status = IFM_ACTIVE | IFM_AVALID;
		ifmr->ifm_count = 0;
		ifmr->ifm_active = IFM_1000_T;
		ifmr->ifm_active |= IFM_ETHER | IFM_FDX;
		ifmr->ifm_current = ifmr->ifm_active;
		ifmr->ifm_mask = 0;
	} else {
		mii = bcmswitch_miiforphy(sc, p->es_port);
		err = ifmedia_ioctl(mii->mii_ifp, &p->es_ifr,
		    &mii->mii_media, SIOCGIFMEDIA);
	}
	BCMSWITCH_UNLOCK(sc);

	return (err);
}

static int
bcmswitch_setport(device_t dev, etherswitch_port_t *p)
{
	struct mii_data *mii;
	int err = 0;

	bcmswitch_softc_t *sc = device_get_softc(dev);
	BCMSWITCH_LOCK_ASSERT(sc, SA_UNLOCKED);
	
	if (p->es_port >= sc->num_ports || p->es_port < 0)
		return (EINVAL);
	if (!bcmswitch_is_portenabled(sc, p->es_port))
		return (0);

	BCMSWITCH_LOCK(sc);

	if (p->es_pvid != 0)
		bcmswitch_set_pvid(sc, p->es_port, p->es_pvid);

	if (bcmswitch_is_phyport(sc, p->es_port)) {
		mii = bcmswitch_miiforphy(sc, p->es_port);
		err = ifmedia_ioctl(mii->mii_ifp, &p->es_ifr, &mii->mii_media,
		    SIOCSIFMEDIA);
	}
	BCMSWITCH_UNLOCK(sc);

	return (err);
} 

static int
bcmswitch_init_vlan(struct bcmswitch_softc *sc)
{
        int i, port;
	uint32_t members;

	/* Reset switsch */
	bcmswitch_writereg16(sc, BCMSWITCH_CTRL_PAGE, 0x79, 0x90);
	// tee see ringi
	DELAY(1000);

	brc53sw_forward(sc, false);

	/* Init switch params */ 
	bcmswitch_hw_setup(sc);

        /* Clear VLAN table */
        bcmswitch_vlan_table_ctrl(sc, VTA_CMD_CLEAR);

	/* Set the 802.1q mode. */
	if (sc->vlan_mode == ETHERSWITCH_VLAN_DOT1Q)
                bcmswitch_init_dot1q(sc);

	/* Assign the member ports to each origin port. */
	for (port = 0; port < sc->num_ports; port++) {
		members = 0;
		if (bcmswitch_is_portenabled(sc, port)) {
			for (i = 0; i < sc->num_ports; i++) {
				if (i == port || !bcmswitch_is_portenabled(sc, i))
					continue;
				members |= (1 << i);
			}
		}
		bcmswitch_port_vlan_assign(sc, port,  members);
	}

	/* Reset internal VLAN table. */
	for (i = 0; i < nitems(sc->vlans); i++)
		sc->vlans[i] = 0;

	/* Create default VLAN (1). */
	if (sc->vlan_mode == ETHERSWITCH_VLAN_DOT1Q) {
		sc->vlans[0] = 1;
		bcmswitch_update_vlan_entry(sc, sc->vlans[0], sc->ports_mask,
		    sc->ports_mask);

	}

	brc53sw_forward(sc, true);

        return (0);
}

static int
bcmswitch_set_vlan_mode(struct bcmswitch_softc *sc, uint32_t mode)
{
	BCMSWITCH_LOCK_ASSERT(sc, SA_XLOCKED);
	switch (mode) {
	case ETHERSWITCH_VLAN_PORT:
		sc->vlan_mode = ETHERSWITCH_VLAN_PORT;
		etherswitch_info.es_nvlangroups = sc->num_ports;
		return (bcmswitch_init_vlan(sc));
		break;
	case ETHERSWITCH_VLAN_DOT1Q:
		sc->vlan_mode = ETHERSWITCH_VLAN_DOT1Q;
		etherswitch_info.es_nvlangroups = BCMSWITCH_NUM_VLANS;
		return (bcmswitch_init_vlan(sc));
		break;
	default:
		return (EINVAL);
	}
}

/*
 * Registers in this switch are divided into sections, specified in
 * documentation. So as to access any of them, section index and reg index
 * is necessary. etherswitchcfg uses only one variable, so indexes were
 * compressed into addr_reg: 256 * section_index + reg_index.
 */ 
static int
bcmswitch_readreg_wrapper(device_t dev, int addr_reg)
{
	struct bcmswitch_softc *sc;
	uint64_t val;

	sc = device_get_softc(dev);
	val = bcmswitch_readreg32(sc, addr_reg / 256, addr_reg % 256 );
	return (val);
}

static int
bcmswitch_writereg_wrapper(device_t dev, int addr_reg, int val)
{
	struct bcmswitch_softc *sc;

	sc = device_get_softc(dev);
	bcmswitch_writereg32(sc, addr_reg / 256, addr_reg % 256, val);
	return (0);
}

static int
bcmswitch_setvgroup_wrapper(device_t dev, etherswitch_vlangroup_t *vg)
{
	bcmswitch_softc_t *sc;
	int ret;

	sc = device_get_softc(dev);
	BCMSWITCH_LOCK_ASSERT(sc, SA_UNLOCKED);

	BCMSWITCH_LOCK(sc);
	ret = bcmswitch_setvgroup(dev, vg);
	BCMSWITCH_UNLOCK(sc);

	return (ret);
}

static int
bcmswitch_getvgroup_wrapper(device_t dev, etherswitch_vlangroup_t *vg)
{
	struct bcmswitch_softc *sc;
	int ret;

	sc = device_get_softc(dev);
	BCMSWITCH_LOCK_ASSERT(sc, SA_UNLOCKED);

	BCMSWITCH_LOCK(sc);
	ret = bcmswitch_getvgroup(dev, vg);
	BCMSWITCH_UNLOCK(sc);

	return (ret);
}

static int
bcmswitch_setvgroup(device_t dev, etherswitch_vlangroup_t *vg)
{
	bcmswitch_softc_t *sc;

	sc = device_get_softc(dev);
	BCMSWITCH_LOCK_ASSERT(sc, SA_XLOCKED);

	if (sc->vlan_mode == ETHERSWITCH_VLAN_PORT)
		return (bcmswitch_set_port_vlan(sc, vg));
	else         if (sc->vlan_mode == ETHERSWITCH_VLAN_DOT1Q)
		return(bcmswitch_set_dot1q_vlan(sc, vg));

        return (EINVAL);
}

static int
bcmswitch_getvgroup(device_t dev, etherswitch_vlangroup_t *vg)
{
	struct bcmswitch_softc *sc;

	sc = device_get_softc(dev);
	BCMSWITCH_LOCK_ASSERT(sc, SA_XLOCKED);

	if (sc->vlan_mode == ETHERSWITCH_VLAN_PORT)
		return (bcmswitch_get_port_vlan(sc, vg));

	else if (sc->vlan_mode == ETHERSWITCH_VLAN_DOT1Q)
                return (bcmswitch_get_dot1q_vlan(sc, vg));

	return (EINVAL);
}

// --------------------------------------------------------- //

static __inline struct mii_data*
bcmswitch_miiforphy(bcmswitch_softc_t *sc, unsigned int phy)
{
	if (!bcmswitch_is_phyport(sc, phy))
		return (NULL);

	return (device_get_softc(sc->miibus[phy]));
}

static int
bcmswitch_ifmedia_upd(struct ifnet *ifp)
{
	bcmswitch_softc_t *sc;
	struct mii_data *mii;

	sc = ifp->if_softc;
	mii = bcmswitch_miiforphy(sc, ifp->if_dunit);
	if (mii == NULL)
		return (ENXIO);
	mii_mediachg(mii);

	return (0);
}

static void
bcmswitch_ifmedia_sts(struct ifnet *ifp, struct ifmediareq *ifmr)
{
	bcmswitch_softc_t *sc;
	struct mii_data *mii;

	sc = ifp->if_softc;
	mii = bcmswitch_miiforphy(sc, ifp->if_dunit);

	if (mii == NULL)
		return;

	mii_pollstat(mii);
	ifmr->ifm_active = mii->mii_media_active;
	ifmr->ifm_status = mii->mii_media_status;
}

static __inline bool
bcmswitch_is_cpuport(bcmswitch_softc_t *sc, int port)
{

        return ((sc->cpuports_mask & (1 << port)) ? true : false);
}

static __inline bool
bcmswitch_is_fixedport(bcmswitch_softc_t *sc, int port)
{

        return ((sc->fixed_mask & (1 << port)) ? true : false);
}

static __inline bool
bcmswitch_is_phyport(bcmswitch_softc_t *sc, int port)
{
        uint32_t phy_mask;
        phy_mask = ~(sc->fixed_mask | sc->cpuports_mask);

        return ((phy_mask & (1 << port)) ? true : false);
}

static __inline bool
bcmswitch_is_portenabled(bcmswitch_softc_t *sc, int port)
{

        return ((sc->ports_mask & (1 << port)) ? true : false);
}

static void
bcmswitch_co_tick(void *arg)
{
        bcmswitch_softc_t *sc;
        struct mii_data *mii;
        struct mii_softc *miisc;
        // uint16_t portstatus;
        int port;

	sc = arg;

	BCMSWITCH_LOCK_ASSERT(sc, SA_UNLOCKED);

	BCMSWITCH_LOCK(sc);
	for (port = 0; port < sc->num_ports; port++) {
		/* Tick only on PHY ports */
		if (!bcmswitch_is_portenabled(sc, port))
			continue;

		if (!bcmswitch_is_phyport(sc, port))
			continue;

		mii = bcmswitch_miiforphy(sc, port);
		if (mii == NULL)
			continue;

		LIST_FOREACH(miisc, &mii->mii_phys, mii_list) {
			if (IFM_INST(mii->mii_media.ifm_cur->ifm_media)
			    != miisc->mii_inst)
				continue;

			ukphy_status(miisc);
			mii_phy_update(miisc, MII_POLLSTAT);
		}
		
	}
	BCMSWITCH_UNLOCK(sc);
	callout_reset(&sc->callout_tick, hz, bcmswitch_co_tick, sc);
}
