/*-
 * Copyright (c) 2022-present Doug Rabson
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

/*
 * Driver for the bcm2711 temperature sensor
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/bus.h>
#include <machine/resource.h>
#include <machine/bus.h>
#include <sys/rman.h>
#include <sys/sysctl.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "syscon_if.h"
#include "thermal_if.h"

#define AVS_TEMPERATURE         0x200 /* offset in the parent MFD */
#define AVS_TEMPERATURE_VALID   ((1 << 16) | (1 << 10))
#define AVS_TEMPERATURE_DATA    ((1 << 10) - 1)

static struct ofw_compat_data compat_data[] = {
        {"brcm,bcm2711-thermal",        1},
	{NULL,				0}
};

struct bcm2711_thermal_softc {
        struct syscon   *sc_syscon;
};

static int
bcm2711_thermal_get_temperature(device_t dev, int *ret)
{
        struct bcm2711_thermal_softc *sc = device_get_softc(dev);
        uint32_t val;

        val = SYSCON_READ_4(sc->sc_syscon, AVS_TEMPERATURE);
        if (!(val & AVS_TEMPERATURE_VALID))
                return (EIO);
        val &= AVS_TEMPERATURE_DATA;
        
        *ret = (int) val;
        return (0);
}

static int
bcm2711_thermal_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Thermal Sensor");

	return (BUS_PROBE_DEFAULT);
}

static int
bcm2711_thermal_attach(device_t dev)
{
        struct bcm2711_thermal_softc *sc = device_get_softc(dev);
        phandle_t node;
        int error;

	error = syscon_get_handle_default(dev, &sc->sc_syscon);
	if (error != 0) {
		device_printf(dev, "Cannot get syscon handle from parent\n");
		return (error);
	}

	node = ofw_bus_get_node(dev);
	OF_device_register_xref(OF_xref_from_node(node), dev);

        return (0);
}

static int
bcm2711_thermal_detach(device_t dev)
{

        return (0);
}

static device_method_t bcm2711_thermal_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		bcm2711_thermal_probe),
	DEVMETHOD(device_attach,	bcm2711_thermal_attach),
	DEVMETHOD(device_detach,	bcm2711_thermal_detach),

        /* Thermal interface */
        DEVMETHOD(thermal_get_temperature, bcm2711_thermal_get_temperature),

	DEVMETHOD_END
};

static driver_t bcm2711_thermal_driver = {
	"thermal-sensor",
	bcm2711_thermal_methods,
	sizeof(struct bcm2711_thermal_softc),
};

EARLY_DRIVER_MODULE(bcm2711_thermal, simple_mfd, bcm2711_thermal_driver, 0, 0,
    BUS_PASS_SUPPORTDEV + BUS_PASS_ORDER_FIRST);
OFWBUS_PNP_INFO(compat_data);
