# $FreeBSD$

# Modules for dev/thermal drivers.

.PATH: ${SRCTOP}/sys/dev/thermal

KMOD=	thermal
SRCS=	ofw_thermal.c bcm2711_thermal.c rpi_poe_fan.c

# Generated files...
SRCS+=	\
	bus_if.h \
	device_if.h \
	opt_platform.h \
	ofw_bus_if.h \
	syscon_if.h \
	thermal_if.c \
	thermal_if.h

EXPORT_SYMS=	YES

.include <bsd.kmod.mk>
