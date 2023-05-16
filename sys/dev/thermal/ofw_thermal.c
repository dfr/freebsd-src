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
 * Driver for thermal zone management on RPI 4
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/limits.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/bus.h>
#include <machine/resource.h>
#include <machine/bus.h>
#include <sys/rman.h>
#include <sys/sysctl.h>
#include <sys/callout.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "thermal_if.h"

// Based on https://www.kernel.org/doc/Documentation/devicetree/bindings/thermal/thermal.txt

enum thermal_trip_type {
        THERMAL_TRIP_ACTIVE,    // active cooling
        THERMAL_TRIP_PASSIVE,   // passive cooling
        THERMAL_TRIP_HOT,       // emergency
        THERMAL_TRIP_CRITICAL,  // hardware not reliable
};

struct thermal_trip {
        SLIST_ENTRY(thermal_trip) tt_link;
        phandle_t       tt_phandle;
        enum thermal_trip_type tt_type;
        int             tt_temperature;
        int             tt_hysteresis;
};

struct thermal_cooling_map {
        SLIST_ENTRY(thermal_cooling_map) tm_link;
        phandle_t       tm_cooling_device;
        int32_t         tm_min;
        int32_t         tm_max;
        struct thermal_trip *tm_trip;
};        

struct thermal_zone {
        SLIST_ENTRY(thermal_zone) tz_link;
        char            *tz_name;
        struct callout  tz_callout;
        unsigned int    tz_polling_delay_ms;
        unsigned int    tz_polling_delay_passive_ms;
        phandle_t       tz_sensor;
        // coefficients to convert raw temperature reading to milli-celcius
        int             tz_slope;
        int             tz_offset;

        int             tz_last_temperature;

        SLIST_HEAD(, thermal_trip) tz_trips;
        SLIST_HEAD(, thermal_cooling_map) tz_cooling_maps;
};

struct thermal_softc {
        SLIST_HEAD(, thermal_zone) sc_zones;
};

static void thermal_zone_schedule(struct thermal_zone *tz);

static int
thermal_trip_parse(device_t dev, phandle_t node, struct thermal_trip **resultp)
{
        struct thermal_trip *tt;
        char *type_name;
        enum thermal_trip_type type;
        unsigned int temperature, hysteresis;
        int ret;

        ret = OF_getprop_alloc(node, "type", (void **)&type_name);
        if (ret == -1) {
                device_printf(dev, "missing trip type");
                return (ENXIO);
        }
        if (strcmp(type_name, "active") == 0)
                type = THERMAL_TRIP_ACTIVE;
        else if (strcmp(type_name, "passive") == 0)
                type = THERMAL_TRIP_PASSIVE;
        else if (strcmp(type_name, "hot") == 0)
                type = THERMAL_TRIP_HOT;
        else if (strcmp(type_name, "critical") == 0)
                type = THERMAL_TRIP_CRITICAL;
        else {
                device_printf(dev, "unknown trip type %s", type_name);
                free(type_name, M_OFWPROP);
                return (ENXIO);
        }
        free(type_name, M_OFWPROP);

        ret = OF_getencprop(node, "temperature", &temperature, sizeof(unsigned int));
        if (ret == -1)
                return (ENXIO);
        ret = OF_getencprop(node, "hysteresis", &hysteresis, sizeof(unsigned int));
        if (ret == -1)
                return (ENXIO);

        tt = malloc(sizeof(struct thermal_trip), M_DEVBUF, M_WAITOK | M_ZERO);
        tt->tt_phandle = OF_xref_from_node(node);
        tt->tt_type = type;
        tt->tt_temperature = temperature;
        tt->tt_hysteresis = hysteresis;

        *resultp = tt;
        return (0);
}

static void
thermal_trips_parse(device_t dev, struct thermal_zone *tz, phandle_t node)
{
        phandle_t child;
        int error;

        for (child = OF_child(node); child; child = OF_peer(child)) {
                struct thermal_trip *tt;
                error = thermal_trip_parse(dev, child, &tt);
                if (error == 0)
                        SLIST_INSERT_HEAD(&tz->tz_trips, tt, tt_link);
        }
}

static int
thermal_cooling_map_parse(device_t dev, struct thermal_zone *tz, phandle_t node,
        struct thermal_cooling_map **resultp)
{
        struct thermal_cooling_map *tm;
        struct thermal_trip *tt;
        int ndevices, nparams;
        pcell_t *params;
        phandle_t cooling, trip;
        int error, ret;
        
        error  = ofw_bus_parse_xref_list_get_length(node, "cooling-device", "#cooling-cells",
            &ndevices);
        if (error)
                return (error);
        if (ndevices != 1) {
                device_printf(dev, "expected one cooling-device\n");
                return (ENXIO);
        }

        error = ofw_bus_parse_xref_list_alloc(node, "cooling-device", "#cooling-cells",
            0, &cooling, &nparams, &params);
        if (error)
                return (error);
        if (nparams != 2) {
                device_printf(dev, "expected two parameters for cooling-device\n");
                free(params, M_OFWPROP);
                return (ENXIO);
        }
        ret = OF_getencprop(node, "trip", &trip, sizeof(phandle_t));
        if (ret == -1) {
                device_printf(dev, "failed to get trip\n");
                return (ENXIO);
        }

        tm = malloc(sizeof(struct thermal_cooling_map), M_DEVBUF, M_WAITOK | M_ZERO);
        tm->tm_cooling_device = cooling;
        if (!tm->tm_cooling_device) {
                device_printf(dev, "failed to resolve cooling-device reference %u\n", cooling);
                free(tm, M_DEVBUF);
                return (ENOENT);
        }
        tm->tm_min = params[0];
        tm->tm_max = params[1];

        // This assumes that trips always precedes cooling-maps
        SLIST_FOREACH(tt, &tz->tz_trips, tt_link) {
                if (tt->tt_phandle == trip) {
                        tm->tm_trip = tt;
                        break;
                }
        }
        if (!tm->tm_trip) {
                device_printf(dev, "failed to match trip for phandle %#x\n", trip);
                free(tm, M_DEVBUF);
                return (ENOENT);
        }

        *resultp = tm;
        return (0);
}

static void
thermal_cooling_maps_parse(device_t dev, struct thermal_zone *tz, phandle_t node)
{
        struct thermal_cooling_map *tm;
        phandle_t child;
        int error;

        for (child = OF_child(node); child; child = OF_peer(child)) {
                error = thermal_cooling_map_parse(dev, tz, child, &tm);
                if (error)
                        continue;
                SLIST_INSERT_HEAD(&tz->tz_cooling_maps, tm, tm_link);
        }
}

static phandle_t
thermal_child_by_name(phandle_t node, const char* name)
{
        phandle_t child;
        char *child_name;
        int ret, match;

        for (child = OF_child(node); child; child = OF_peer(child)) {
                ret = OF_getprop_alloc(child, "name", (void **)&child_name);
                if (ret == -1)
                        continue;
                match = (strcmp(child_name, name) == 0);
                free(child_name, M_OFWPROP);
                if (match)
                        return (child);
        }
        return (0);
}

static struct thermal_zone *
thermal_zone_parse(device_t dev, phandle_t node)
{
        struct thermal_zone *tz;
        char *name;
        int ret;
        phandle_t child, xref;
        int coeff[2];

        
        ret = OF_getprop_alloc(node, "name", (void **)&name);
        if (ret == -1)
                return (NULL);
                
        if (bootverbose)
                device_printf(dev, "parsing thermal zone %s\n", name);

        tz = malloc(sizeof(struct thermal_zone), M_DEVBUF, M_WAITOK | M_ZERO);
        tz->tz_name = name;
        callout_init(&tz->tz_callout, TRUE);

        ret = OF_getencprop(node, "polling-delay", &tz->tz_polling_delay_ms, sizeof(unsigned int));
        if (ret == -1)
                printf("%s: failed to get polling-delay\n", name);
        ret = OF_getencprop(node, "polling-delay-passive", &tz->tz_polling_delay_passive_ms, sizeof(unsigned int));
        if (ret == -1)
                printf("%s: failed to get polling-delay-passive\n", name);
        ret = OF_getencprop(node, "thermal-sensors", &xref, sizeof(xref));
        if (ret == -1)
                device_printf(dev, "%s: failed to get thermal-sensors\n", name);
        tz->tz_sensor = xref;

        // The coefficients property should have two signed ints
        ret = OF_getencprop(node, "coefficients", (unsigned int*)coeff, sizeof(coeff));
        if (ret == -1) {
                device_printf(dev, "%s: failed to get coefficients\n", name);
                coeff[0] = 1;
                coeff[1] = 0;
        }
        tz->tz_slope = coeff[0];
        tz->tz_offset = coeff[1];

        // We should have two child nodes, "trips" and "cooling-maps"
        child = thermal_child_by_name(node, "trips");
        if (child)
                thermal_trips_parse(dev, tz, child);
        child = thermal_child_by_name(node, "cooling-maps");
        if (child)
                thermal_cooling_maps_parse(dev, tz, child);

        return (tz);
}

static int
thermal_zone_get_temperature(struct thermal_zone *tz, int *resultp)
{
        int error, raw;
        device_t sdev = OF_device_from_xref(tz->tz_sensor);

        if (!sdev)
                return (ENXIO);
        error = THERMAL_GET_TEMPERATURE(sdev, &raw);
        if (error)
                return (error);
        *resultp = raw * tz->tz_slope + tz->tz_offset;
        return (0);
}

static void
thermal_zone_poll(void *arg)
{
        struct thermal_zone *tz = arg;
        struct thermal_cooling_map *tm;
        device_t cooling_device = NULL;
        int error, temp;
        int new_state;

        if (bootverbose)
                printf("%s, polling sensor\n", tz->tz_name);

        error = thermal_zone_get_temperature(tz, &temp);
        if (error)
                goto out;
        if (temp > tz->tz_last_temperature)
                new_state = -1;
        else if (temp < tz->tz_last_temperature)
                new_state = INT_MAX;
        else
                goto out;

        //printf("%s: last temperature: %d, new temperature: %d\n", tz->tz_name, tz->tz_last_temperature, temp);
        SLIST_FOREACH(tm, &tz->tz_cooling_maps, tm_link) {
                int upper = tm->tm_trip->tt_temperature;
                int lower = upper - tm->tm_trip->tt_hysteresis;
                if (temp > tz->tz_last_temperature) {
                        // Increasing temperature - test against upper trip level
                        if (temp >= upper) {
                                //printf("rising: %d > %d: %d -> %d\n", temp, upper, new_state, tm->tm_max);
                                if (tm->tm_max > new_state) {
                                        new_state = tm->tm_max;
                                        cooling_device = OF_device_from_xref(tm->tm_cooling_device);
                                }
                        }
                } else {
                        // Lowering temperature - test against lower trip level
                        if (temp < lower) {
                                //printf("falling: %d < %d: %d -> %d\n", temp, lower, new_state, tm->tm_min);
                                if (tm->tm_min < new_state) {
                                        new_state = tm->tm_min;
                                        cooling_device = OF_device_from_xref(tm->tm_cooling_device);
                                }
                        }
                }
        }
        tz->tz_last_temperature = temp;
        if (cooling_device) {
                THERMAL_SET_STATE(cooling_device, new_state);
        }

out:
        thermal_zone_schedule(tz);
}

static void
thermal_zone_schedule(struct thermal_zone *tz)
{

        callout_reset(&tz->tz_callout, tz->tz_polling_delay_ms * hz / 1000,
            thermal_zone_poll, tz);
}

static void
thermal_zone_stop(struct thermal_zone *tz)
{

        callout_drain(&tz->tz_callout);
}

static void
thermal_zone_print(device_t dev, struct thermal_zone *tz)
{
        struct thermal_trip *tt;
        struct thermal_cooling_map *tm;
        static const char* types[] = {
                "active",
                "passive",
                "hot",
                "critical",
        };
        int error, temp;
    
        device_printf(dev, "zone: %s, polling every %ums\n", tz->tz_name, tz->tz_polling_delay_ms);
        device_printf(dev, "  thermal coefficients: slope %d, offset %d\n", tz->tz_slope, tz->tz_offset);
        error = thermal_zone_get_temperature(tz, &temp);
        if (error)
                device_printf(dev, "failed to get temperature: %d\n", error);
        else
                device_printf(dev, "  current temperature: %d\n", temp);
        tz->tz_last_temperature = temp;
        SLIST_FOREACH(tt, &tz->tz_trips, tt_link) {
                device_printf(dev, "  trip: phandle %#x, type %s, temperature %d, hysteresis %d\n",
                    tt->tt_phandle, types[tt->tt_type], tt->tt_temperature, tt->tt_hysteresis);
        }
        SLIST_FOREACH(tm, &tz->tz_cooling_maps, tm_link) {
                device_t cdev = OF_device_from_xref(tm->tm_cooling_device);
                device_printf(dev, "  map: cooling-device %s, trip %#x\n",
                    cdev ? device_get_nameunit(cdev) : "???", tm->tm_trip->tt_phandle);
        }
}

static int
thermal_parse_zones(device_t dev)
{
        struct thermal_softc *sc = device_get_softc(dev);
        phandle_t node, child;
        struct thermal_zone *tz;

        SLIST_INIT(&sc->sc_zones);
        
        node = ofw_bus_get_node(dev);
        for (child = OF_child(node); child; child = OF_peer(child)) {
                tz = thermal_zone_parse(dev, child);
                SLIST_INSERT_HEAD(&sc->sc_zones, tz, tz_link);
        }

        return (0);
}

static int
thermal_probe(device_t dev)
{

        if (strcmp(ofw_bus_get_name(dev), "thermal-zones"))
                return (ENXIO);

        device_set_desc(dev, "Thermal Zone Control");
        return (BUS_PROBE_DEFAULT);
}

static int
thermal_attach(device_t dev)
{
        struct thermal_softc *sc = device_get_softc(dev);
        struct thermal_zone *tz;
        int error;

        error = thermal_parse_zones(dev);
        if (error)
                return (error);

        SLIST_FOREACH(tz, &sc->sc_zones, tz_link) {
                thermal_zone_schedule(tz);
                thermal_zone_print(dev, tz);
        }
        
        return (0);
}

static int
thermal_detach(device_t dev)
{
        struct thermal_softc *sc = device_get_softc(dev);
        struct thermal_zone *tz;

        SLIST_FOREACH(tz, &sc->sc_zones, tz_link) {
                thermal_zone_stop(tz);
        }        
        return (0);
}

static device_method_t thermal_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		thermal_probe),
	DEVMETHOD(device_attach,	thermal_attach),
	DEVMETHOD(device_detach,	thermal_detach),

	DEVMETHOD_END
};

static driver_t thermal_driver = {
	"thermal",
	thermal_methods,
	sizeof(struct thermal_softc),
};

EARLY_DRIVER_MODULE(rpi_thermal, ofwbus, thermal_driver, 0, 0,
        BUS_PASS_SUPPORTDEV + BUS_PASS_ORDER_LATE);
MODULE_VERSION(thermal, 1);
