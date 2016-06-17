/*-
 * Copyright (c) 2016 Jared McNeill <jmcneill@invisible.ca>
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

/*
 * Allwinner multi bus gates clock
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <machine/bus.h>

#include <dev/fdt/simplebus.h>
#include <dev/fdt/fdt_common.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/extres/clk/clk.h>

#include "clkdev_if.h"

struct aw_multigate_softc {
	struct simplebus_softc	sc;
};

static struct ofw_compat_data compat_data[] = {
	{ "allwinner,sunxi-multi-bus-gates-clk",	1 },
	{ NULL, 0 }
};

static int
aw_multigate_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Allwinner Multi Bus Gates Clock");
	return (BUS_PROBE_DEFAULT);
}

static int
aw_multigate_attach(device_t dev)
{
	struct aw_multigate_softc *sc;
	phandle_t node, child;
	device_t cdev;

	sc = device_get_softc(dev);
	node = ofw_bus_get_node(dev);

	simplebus_init(dev, node);

	/* Attach child devices */
	for (child = OF_child(node); child > 0; child = OF_peer(child)) {
		cdev = simplebus_add_device(dev, child, 0, NULL, -1, NULL);
		if (cdev != NULL)
			device_probe_and_attach(cdev);
	}

	return (bus_generic_attach(dev));
}

static device_method_t aw_multigate_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		aw_multigate_probe),
	DEVMETHOD(device_attach,	aw_multigate_attach),

	DEVMETHOD_END
};

DEFINE_CLASS_1(aw_multigate, aw_multigate_driver, aw_multigate_methods,
    sizeof(struct aw_multigate_softc), simplebus_driver);

static devclass_t aw_multigate_devclass;

EARLY_DRIVER_MODULE(aw_multigate, simplebus, aw_multigate_driver,
    aw_multigate_devclass, 0, 0, BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE);

MODULE_VERSION(aw_multigate, 1);
