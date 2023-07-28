# $FreeBSD$

.PATH: ${SRCTOP}/sys/dev/etherswitch/bcmswitch

KMOD=   bcmswitch
SRCS=   bcmswitch.c bcmswitch_ps_phy.c

SRCS+=	bus_if.h etherswitch_if.h mdio_if.h miibus_if.h ofw_bus_if.h

.include <bsd.kmod.mk>
