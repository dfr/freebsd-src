/*-
 * Copyright (c) 2024-present Doug Rabson
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
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/hwreset/hwreset.h>

#include "hwreset_if.h"

/*
 * Each bank has six 32bit registers. The first sets bits in the reset
 * controller when written, the second clears bits and the third can be read for
 * the current status. The remaining registers are unused.
 */

#define RESET_BANK(id)			((id) / 32)
#define RESET_BIT(id)			(1ul << ((id) & 31))
#define RESET_BANK_OFFSET(id)		(RESET_BANK(id) * 6 * sizeof(uint32_t))
#define RESET_SET_OFFSET(id)		(RESET_BANK_OFFSET(id) + 0)
#define RESET_CLEAR_OFFSET(id)		(RESET_BANK_OFFSET(id) + 4)
#define RESET_STATUS_OFFSET(id)		(RESET_BANK_OFFSET(id) + 8)


static struct ofw_compat_data compat_data[] = {
	{ "brcm,brcmstb-reset",	1 },
	{ NULL,			0 }
};

struct brcmstb_reset_softc {
	struct resource	*sc_res;
};

static int
brcmstb_reset_assert(device_t dev, intptr_t id, bool reset)
{
	struct brcmstb_reset_softc *sc;
	uint32_t off = reset ? RESET_SET_OFFSET(id) : RESET_CLEAR_OFFSET(id);

	sc = device_get_softc(dev);
	bus_write_4(sc->sc_res, off, RESET_BIT(id));

	return (0);
}

static int
brcmstb_reset_is_asserted(device_t dev, intptr_t id, bool *reset)
{
	struct brcmstb_reset_softc *sc;
	uint32_t off = RESET_STATUS_OFFSET(id);

	sc = device_get_softc(dev);
	*reset = (bus_read_4(sc->sc_res, off) & RESET_BIT(id)) != 0;

	return (0);
}

static int
brcmstb_reset_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Broadcom 2712 Reset Controller");
	return (BUS_PROBE_DEFAULT);
}

static int
brcmstb_reset_attach(device_t dev)
{
	struct brcmstb_reset_softc *sc;
        int rid;

	sc = device_get_softc(dev);
        rid = 0;
	sc->sc_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (!sc->sc_res) {
		device_printf(dev, "cannot allocate memory window\n");
		return (ENXIO);
	}
	hwreset_register_ofw_provider(dev);

	return (0);
}

static device_method_t brcmstb_reset_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		brcmstb_reset_probe),
	DEVMETHOD(device_attach,	brcmstb_reset_attach),

	/* Reset interface */
	DEVMETHOD(hwreset_assert,	brcmstb_reset_assert),
	DEVMETHOD(hwreset_is_asserted,	brcmstb_reset_is_asserted),

	DEVMETHOD_END
};

static driver_t brcmstb_reset_driver = {
	"reset",
	brcmstb_reset_methods,
	sizeof(struct brcmstb_reset_softc),
};

EARLY_DRIVER_MODULE(hwreset_brcmstb, simplebus, brcmstb_reset_driver, 0, 0,
    BUS_PASS_RESOURCE + BUS_PASS_ORDER_MIDDLE);
MODULE_VERSION(brcmstb_reset, 1);
