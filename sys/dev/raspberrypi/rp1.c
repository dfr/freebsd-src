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
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>

#define RP1_PCI_VENDOR          0x1de4
#define RP1_PCI_DEVICE          0x0001

struct rp1_softc {
        struct resource *sc_res[3];
        int sc_rid[3];
};

static int
rp1_pci_probe(device_t dev)
{

        if (pci_get_vendor(dev) == RP1_PCI_VENDOR ||
            pci_get_device(dev) == RP1_PCI_DEVICE) {
                device_set_desc(dev, "RaspberryPi RP1 Southbridge");
                return (BUS_PROBE_DEFAULT);
        }
        return (ENXIO);
}

static int
rp1_pci_attach(device_t dev)
{
        struct rp1_softc *sc;
        int i;

        sc = device_get_softc(dev);
        for (i = 0; i < 3; i++) {
                sc->sc_rid[i] = PCIR_BAR(i);
                sc->sc_res[i] = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->sc_rid[i], RF_ACTIVE);
                if (sc->sc_res[i]) {
                        device_printf(dev, "rid %#x: start=%#lx, size=%#lx\n",
                            sc->sc_rid[i],
                            rman_get_start(sc->sc_res[i]),
                            rman_get_size(sc->sc_res[i]));
                }
        }

        return (0);
}

static int
rp1_pci_detach(device_t dev)
{
        struct rp1_softc *sc;
        int i;

        sc = device_get_softc(dev);
        for (i = 0; i < 3; i++) {
                if (sc->sc_res[i])
                        bus_release_resource(dev, SYS_RES_MEMORY, sc->sc_rid[i], sc->sc_res[i]);
        }

        return (0);
}

static device_method_t rp1_pci_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		rp1_pci_probe),
	DEVMETHOD(device_attach,	rp1_pci_attach),
	DEVMETHOD(device_detach,	rp1_pci_detach),
	DEVMETHOD_END
};

static driver_t rp1_pci_driver = {
        "rpone",
	rp1_pci_methods,
	sizeof(struct rp1_softc)
};
                
DRIVER_MODULE(rpone, pci, rp1_pci_driver, NULL, NULL);
