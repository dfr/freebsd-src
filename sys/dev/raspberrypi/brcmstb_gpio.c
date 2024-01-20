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
#include <sys/gpio.h>
#include <sys/rman.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/gpio/gpiobusvar.h>

#include "hwreset_if.h"

/*
 * Each bank has eight 32bit registers and manages the state of 32 gpio pins
 */

#define GPIO_LOCK(_sc)                  mtx_lock_spin(&(_sc)->sc_mtx)
#define GPIO_UNLOCK(_sc)                mtx_unlock_spin(&(_sc)->sc_mtx)

#define GPIO_BANK(id)			((id) / 32)
#define GPIO_BIT(id)			((id) & 31)
#define GPIO_BIT_MASK(id)		(1ul << GPIO_BIT(id))
#define GPIO_BANK_OFFSET(id)		(GPIO_BANK(id) * 8 * sizeof(uint32_t))

#define GPIO_REG_ODEN                   0
#define GPIO_REG_DATA                   0
#define GPIO_REG_IODIR                  0
#define GPIO_REG_EC                     0
#define GPIO_REG_EI                     0
#define GPIO_REG_MASK                   0
#define GPIO_REG_LEVEL                  0
#define GPIO_REG_STAT                   0

#define GPIO_DATA_OFFSET(id)		(GPIO_BANK_OFFSET(id) + GPIO_REG_DATA)
#define GPIO_IODIR_OFFSET(id)		(GPIO_BANK_OFFSET(id) + GPIO_REG_IODIR)

static struct ofw_compat_data compat_data[] = {
	{ "brcm,brcmstb-gpio",	1 },
	{ NULL,			0 }
};

struct brcmstb_gpio_softc {
	struct resource	*sc_res;
	struct mtx sc_mtx;
        device_t sc_busdev;
        int sc_rid;
        int sc_nbanks;
        uint32_t *sc_bank_widths;
        int sc_npins;
};

static int
brcmstb_gpio_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Broadcom 2712 GPIO Controller");
	return (BUS_PROBE_DEFAULT);
}

static int
brcmstb_gpio_attach(device_t dev)
{
	struct brcmstb_gpio_softc *sc;

	sc = device_get_softc(dev);
        mtx_init(&sc->sc_mtx, device_get_nameunit(dev), "brcmgpio", MTX_SPIN);
        sc->sc_rid = 0;
	sc->sc_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->sc_rid, RF_ACTIVE);
	if (!sc->sc_res) {
		device_printf(dev, "cannot allocate memory window\n");
		return (ENXIO);
	}

        sc->sc_bank_widths = NULL;
        sc->sc_nbanks = OF_getencprop_alloc_multi(
                ofw_bus_get_node(dev),
                "brcm,gpio-bank-widths",
                sizeof(*sc->sc_bank_widths),
                (void**)&sc->sc_bank_widths);
        if (sc->sc_nbanks < 0) {
                device_printf(dev, "failed to find or decode brcm,gpio-bank-widths property\n");
                return (ENXIO);
        }
        /*
         * Assume that pin numbers for a bank are 32*bank..32*(bank+1)+31
         * inclusive. We set npins to one plus the maximum pin index in the last
         * bank.
         */
        sc->sc_npins = 32 * (sc->sc_nbanks - 1) + sc->sc_bank_widths[sc->sc_nbanks - 1];

	sc->sc_busdev = gpiobus_attach_bus(dev);
        if (sc->sc_busdev == NULL) {
                mtx_destroy(&sc->sc_mtx);
                bus_release_resource(dev, SYS_RES_MEMORY, sc->sc_rid, sc->sc_res);
                free(sc->sc_bank_widths, M_OFWPROP);
                return (ENXIO);
        }

	return (0);
}

static int
brcmstb_gpio_detach(device_t dev)
{
	struct brcmstb_gpio_softc *sc;

	sc = device_get_softc(dev);
	if (sc->sc_busdev)
		gpiobus_detach_bus(dev);
        if (sc->sc_res)
                bus_release_resource(dev, SYS_RES_MEMORY, sc->sc_rid, sc->sc_res);
        if (sc->sc_bank_widths)
                free(sc->sc_bank_widths, M_OFWPROP);
        mtx_destroy(&sc->sc_mtx);

        return (0);
}

static device_t
brcmstb_gpio_get_bus(device_t dev)
{
	struct brcmstb_gpio_softc *sc;

	sc = device_get_softc(dev);
        return (sc->sc_busdev);
}

static int
brcmstb_gpio_pin_max(device_t dev, int *maxpin)
{
	struct brcmstb_gpio_softc *sc;

	sc = device_get_softc(dev);
        *maxpin = sc->sc_npins - 1;

        return (0);
}

static int
brcmstb_gpio_valid_pin(device_t dev, uint32_t pin)
{
	struct brcmstb_gpio_softc *sc;

	sc = device_get_softc(dev);
        if (pin >= sc->sc_npins)
                return (EINVAL);
        if (GPIO_BIT(pin) >= sc->sc_bank_widths[GPIO_BANK(pin)])
                return (EINVAL);

        return (0);
}

static int
brcmstb_gpio_pin_getname(device_t dev, uint32_t pin, char *name)
{
        int err;

        /*
         * Possibly change to use gpio-line-names property
         */
        err = brcmstb_gpio_valid_pin(dev, pin);
        if (err)
                return (err);
        snprintf(name, GPIOMAXNAME, "pin-%u", pin);

        return (0);
}

static int
brcmstb_gpio_pin_getcaps(device_t dev, uint32_t pin, uint32_t *caps)
{
        int err;

        err = brcmstb_gpio_valid_pin(dev, pin);
        if (err)
                return (err);
        *caps = GPIO_PIN_INPUT | GPIO_PIN_OUTPUT;

        return (0);
}

static int
brcmstb_gpio_pin_getflags(device_t dev, uint32_t pin, uint32_t *flags)
{
        int err;

        err = brcmstb_gpio_valid_pin(dev, pin);
        if (err)
                return (err);
        *flags = GPIO_PIN_INPUT | GPIO_PIN_OUTPUT;

        return (0);
}

static int
brcmstb_gpio_pin_setflags(device_t dev, uint32_t pin, uint32_t flags)
{
	struct brcmstb_gpio_softc *sc;
        uint32_t valid = GPIO_PIN_INPUT | GPIO_PIN_OUTPUT;
	uint32_t off = GPIO_DATA_OFFSET(pin);
        uint32_t bit = GPIO_BIT_MASK(pin);
        uint32_t val;
        int err;

	sc = device_get_softc(dev);
        err = brcmstb_gpio_valid_pin(dev, pin);
        if (err) {
		device_printf(dev, "pin_setflags: pin %u is not valid\n", pin);
                return (err);
	}

        /*
         * Exactly one of input and output can be set
         */
        if (flags & ~valid)
                return (EINVAL);
        if (flags != GPIO_PIN_INPUT && flags != GPIO_PIN_OUTPUT)
                return (EINVAL);
        
        GPIO_LOCK(sc);
        val = bus_read_4(sc->sc_res, off);
        if (flags == GPIO_PIN_INPUT)
                val |= bit;
        else
                val &= ~bit;
        bus_write_4(sc->sc_res, off, val);
        GPIO_UNLOCK(sc);


        return (0);
}


static int
brcmstb_gpio_pin_set(device_t dev, uint32_t pin, uint32_t pin_value)
{
	struct brcmstb_gpio_softc *sc;
	uint32_t off = GPIO_DATA_OFFSET(pin);
        uint32_t bit = GPIO_BIT_MASK(pin);
        uint32_t val;
        int err;

	sc = device_get_softc(dev);
        err = brcmstb_gpio_valid_pin(dev, pin);
        if (err)
                return (err);

        GPIO_LOCK(sc);
        val = bus_read_4(sc->sc_res, off);
        if (pin_value)
                val |= bit;
        else
                val &= ~bit;
	bus_write_4(sc->sc_res, off, val);
        GPIO_UNLOCK(sc);

	return (0);
}

static int
brcmstb_gpio_pin_get(device_t dev, uint32_t pin, uint32_t *pin_value)
{
	struct brcmstb_gpio_softc *sc;
	uint32_t off = GPIO_DATA_OFFSET(pin);
        uint32_t bit = GPIO_BIT_MASK(pin);
        uint32_t val;
        int err;

	sc = device_get_softc(dev);
        err = brcmstb_gpio_valid_pin(dev, pin);
        if (err)
                return (err);

        val = bus_read_4(sc->sc_res, off);
        *pin_value = (val & bit) != 0;

	return (0);
}

static int
brcmstb_gpio_pin_toggle(device_t dev, uint32_t pin)
{
	struct brcmstb_gpio_softc *sc;
	uint32_t off = GPIO_DATA_OFFSET(pin);
        uint32_t bit = GPIO_BIT_MASK(pin);
        uint32_t val;
        int err;

	sc = device_get_softc(dev);
        err = brcmstb_gpio_valid_pin(dev, pin);
        if (err)
                return (err);

        GPIO_LOCK(sc);
        val = bus_read_4(sc->sc_res, off);
        val ^= bit;
        bus_write_4(sc->sc_res, off, val);
        GPIO_UNLOCK(sc);

	return (0);
}

static device_method_t brcmstb_gpio_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		brcmstb_gpio_probe),
	DEVMETHOD(device_attach,	brcmstb_gpio_attach),
	DEVMETHOD(device_detach,	brcmstb_gpio_detach),

	/* GPIO interface */
	DEVMETHOD(gpio_get_bus, 	brcmstb_gpio_get_bus),
	DEVMETHOD(gpio_pin_max, 	brcmstb_gpio_pin_max),
	DEVMETHOD(gpio_pin_getname, 	brcmstb_gpio_pin_getname),
	DEVMETHOD(gpio_pin_getflags,	brcmstb_gpio_pin_getflags),
	DEVMETHOD(gpio_pin_getcaps, 	brcmstb_gpio_pin_getcaps),
	DEVMETHOD(gpio_pin_setflags,	brcmstb_gpio_pin_setflags),
	DEVMETHOD(gpio_pin_get, 	brcmstb_gpio_pin_get),
	DEVMETHOD(gpio_pin_set, 	brcmstb_gpio_pin_set),
	DEVMETHOD(gpio_pin_toggle, 	brcmstb_gpio_pin_toggle),

	DEVMETHOD_END
};

static driver_t brcmstb_gpio_driver = {
	"gpio",
	brcmstb_gpio_methods,
	sizeof(struct brcmstb_gpio_softc),
};

EARLY_DRIVER_MODULE(brcmstb_gpio, simplebus, brcmstb_gpio_driver, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LATE);
