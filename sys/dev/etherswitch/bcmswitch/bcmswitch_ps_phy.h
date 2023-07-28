#ifndef __BCMSWITCH_HW_PSEUDO_PHY_H__
#define __BCMSWITCH_HW_PSEUDO_PHY_H__

extern uint16_t bcmswitch_readreg16(struct bcmswitch_softc *sc, int addr, int reg);

extern uint32_t bcmswitch_readreg32(struct bcmswitch_softc *sc, int addr, int reg);

extern uint64_t bcmswitch_readreg64(struct bcmswitch_softc *sc, int addr, int reg);

extern int bcmswitch_writereg16(struct bcmswitch_softc *sc, int addr, int reg,
    uint16_t val);

extern int bcmswitch_writereg32(struct bcmswitch_softc *sc, int addr, int reg,
    uint32_t val);

extern int bcmswitch_writereg64(struct bcmswitch_softc *sc, int addr, int reg,
    uint64_t val);


#endif  /* __BCMSWITCH_HW_PSEUDO_PHY_H__ */

