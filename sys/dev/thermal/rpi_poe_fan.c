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
 * Driver for the POE+ HAT fan.
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

#include <arm/broadcom/bcm2835/bcm2835_firmware.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "thermal_if.h"

#define POE_FAN_PWM_CUR		0 /* register for current fan pwm */
#define POE_FAN_PWM_DEF		1 /* register for default fan pwm */
#define POE_FAN_MAX_PWN		255

static struct ofw_compat_data compat_data[] = {
        {"raspberrypi,rpi-poe-fan",     1},
	{NULL,				0}
};

struct poe_fan_softc {
	device_t	sc_firmware;
	ssize_t		sc_level_count;
	uint32_t	*sc_levels;
	uint32_t	sc_max_state;
	uint32_t	sc_min_state;
	uint32_t	sc_cur_state;
	uint32_t	sc_cur_level;
};

struct poe_fan_tag_data {
	uint32_t	td_reg;
	uint32_t	td_val;
	uint32_t	td_ret;
};

static int
poe_fan_read_reg(device_t dev, int reg, uint32_t *valuep)
{
	struct poe_fan_softc *sc = device_get_softc(dev);
	struct poe_fan_tag_data msg;
	int error;

	msg.td_reg = reg;
	error = bcm2835_firmware_property(sc->sc_firmware,
	    BCM2835_FIRMWARE_TAG_GET_POE_HAT_VAL, &msg, sizeof(msg));
	if (error)
		return (error);
	if (msg.td_ret)
		return (EIO);
	*valuep = msg.td_val;
	return (0);
}

static int
poe_fan_write_reg(device_t dev, int reg, uint32_t value)
{
	struct poe_fan_softc *sc = device_get_softc(dev);
	struct poe_fan_tag_data msg;
	int error;

	msg.td_reg = reg;
	msg.td_val = value;
	error = bcm2835_firmware_property(sc->sc_firmware,
	    BCM2835_FIRMWARE_TAG_SET_POE_HAT_VAL, &msg, sizeof(msg));
	if (error)
		return (error);
	if (msg.td_ret)
		return (EIO);
	return (0);
}

static void
poe_fan_set_state(device_t dev, int state)
{
	struct poe_fan_softc *sc = device_get_softc(dev);
	int error;

	if (state < sc->sc_min_state || state > sc->sc_max_state) {
		device_printf(dev, "ignoring out-of-range state value %d\n", state);
	}
	if (state != sc->sc_cur_state) {
		int val = sc->sc_levels[state];
		error = poe_fan_write_reg(dev, POE_FAN_PWM_CUR, val);
		if (error) {
			device_printf(dev, "failed to set fan pwm to %d", val);
			return;
		}
		sc->sc_cur_state = state;
	}
}

static int
poe_fan_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "POE+ Fan Controller");

	return (BUS_PROBE_DEFAULT);
}

static int
poe_fan_attach(device_t dev)
{
	struct poe_fan_softc *sc;
	struct sysctl_ctx_list *sctx;
	struct sysctl_oid *soid;
	int error, ret;
	uint32_t revision;
	phandle_t node, xref;
	ssize_t len, i;

	sc = device_get_softc(dev);
	node = ofw_bus_get_node(dev);

	ret = OF_getencprop(node, "firmware", &xref, sizeof(xref));
	if (ret == -1) {
		device_printf(dev, "failed to read firmware property\n");
		return (ENXIO);
	}
	sc->sc_firmware = OF_device_from_xref(xref);
	if (sc->sc_firmware == NULL) {
		device_printf(dev, "failed to find firmware device\n");
		return (ENXIO);
	}

	error = bcm2835_firmware_property(sc->sc_firmware,
	    BCM2835_FIRMWARE_TAG_GET_FIRMWARE_REVISION,
	    &revision, sizeof(revision));
	if (error) {
		device_printf(dev, "failed to get firmware revision\n");
		return (error);
	}
	if (revision < 0x60af72e8) {
		device_printf(dev, "firmware revision %#x too old\n", revision);
		return (ENXIO);
	}

	error = poe_fan_read_reg(dev, POE_FAN_PWM_CUR, &sc->sc_cur_level);
	if (error) {
		device_printf(dev, "failed to read current cooling level\n");
		return (error);
	}

	len = OF_getencprop_alloc_multi(
		node, "cooling-levels", sizeof(uint32_t),
		(void**)&sc->sc_levels);
	if (len < 0) {
		device_printf(dev, "failed to read cooling-levels\n");
		return (ENXIO);
	}
	sc->sc_level_count = len;

	for (i = sc->sc_level_count - 1; i >= 0; i--) {
		if (sc->sc_cur_level >= sc->sc_levels[i])
			break;
	}
	sc->sc_cur_state = i;

	OF_getencprop(node, "cooling-max-state", &sc->sc_max_state, sizeof(uint32_t));
	OF_getencprop(node, "cooling-min-state", &sc->sc_min_state, sizeof(uint32_t));

	if (TRUE || bootverbose) {
		device_printf(dev, "cooling-levels: ");
		for (i = 0; i < sc->sc_level_count; i++)
			printf("%s%u", i == 0 ? "" : ", ", sc->sc_levels[i]);
		printf("\n");
		device_printf(dev, "cooling-max-state: %u\n", sc->sc_max_state);
		device_printf(dev, "cooling-min-state: %u\n", sc->sc_min_state);
		device_printf(dev, "current pwm: %u, current state: %u\n",
		    sc->sc_cur_level, sc->sc_cur_state);
	}

	OF_device_register_xref(OF_xref_from_node(node), dev);

	// Add state and pwm value to sysctl
	sctx = device_get_sysctl_ctx(dev);
	soid = device_get_sysctl_tree(dev);
	SYSCTL_ADD_U32(sctx, SYSCTL_CHILDREN(soid), OID_AUTO, "state",
	    CTLFLAG_RD, &sc->sc_cur_state, 0, "Fan state");
	SYSCTL_ADD_U32(sctx, SYSCTL_CHILDREN(soid), OID_AUTO, "level",
	    CTLFLAG_RD, &sc->sc_cur_level, 0, "Fan cooling level");

        return (0);
}

static int
poe_fan_detach(device_t dev)
{

        return (0);
}

static device_method_t poe_fan_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		poe_fan_probe),
	DEVMETHOD(device_attach,	poe_fan_attach),
	DEVMETHOD(device_detach,	poe_fan_detach),

        /* Thermal interface */
	DEVMETHOD(thermal_set_state,	poe_fan_set_state),

	DEVMETHOD_END
};

static driver_t poe_fan_driver = {
	"fan",
	poe_fan_methods,
	sizeof(struct poe_fan_softc),
};

EARLY_DRIVER_MODULE(rpi_poe_fan, ofwbus, poe_fan_driver, 0, 0,
	BUS_PASS_SUPPORTDEV);
OFWBUS_PNP_INFO(compat_data);

