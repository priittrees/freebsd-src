#ifndef __BCMSWITCH_VAR_H__
#define __BCMSWITCH_VAR_H__


/* Size of the ALR table in hardware */
#define BCMSWITCH_NUM_ARL_ENTRIES         4096

#define BCMSWITCH_NUM_VLANS		4096
#define BCMSWITCH_MAX_PORTS		9

#define BCMSWITCH_LOCK(_sc)               sx_xlock(&(_sc)->sx)
#define BCMSWITCH_UNLOCK(_sc)             sx_unlock(&(_sc)->sx)
#define BCMSWITCH_LOCK_ASSERT(_sc, _what) sx_assert(&(_sc)->sx, (_what))
#define MDIO_READ(dev, addr, reg)                                       \
    MDIO_READREG(device_get_parent(dev), (addr), (reg))
#define MDIO_WRITE(dev, addr, reg, val)                                 \
    MDIO_WRITEREG(device_get_parent(dev), (addr), (reg), (val))

typedef struct bcmswitch_softc {
        device_t                dev;
        phandle_t               node;

        struct sx               sx;
        struct ifnet            *ifp[BCMSWITCH_MAX_PORTS];
        char                    *ifname[BCMSWITCH_MAX_PORTS];
        device_t                miibus[BCMSWITCH_MAX_PORTS];
        // struct taskqueue        *sc_tq;
        // struct timeout_task     sc_tt;

        int                     vlans[BCMSWITCH_NUM_VLANS];
        uint32_t                vlan_mode;
        uint32_t                cpuports_mask;
        uint32_t                fixed_mask;
        uint32_t                ports_mask;
        int                     phy_base;
        int                     sw_addr;
        int                     num_ports;

        struct callout  callout_tick;

} bcmswitch_softc_t;


#endif /* __BCMSWITCH_VAR_H__ */
