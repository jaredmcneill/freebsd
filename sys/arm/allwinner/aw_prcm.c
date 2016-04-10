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
 * Allwinner Power/Reset/Clock Management
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

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/extres/clk/clk.h>

#include "clkdev_if.h"

struct aw_prcm_softc {
	struct simplebus_softc	sc;
	struct resource		*res;
	struct mtx		mtx;
};

static struct resource_spec aw_prcm_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE|RF_SHAREABLE },
	{ -1, 0 }
};

static struct ofw_compat_data compat_data[] = {
	{ "allwinner,sun6i-a31-prcm",	1 },
	{ NULL, 0 }
};

static int
aw_prcm_check_addr(struct resource *res, bus_addr_t addr)
{
	if (addr < rman_get_start(res) || addr >= rman_get_end(res))
		return (EINVAL);
	return (0);
}

static int
aw_prcm_write_4(device_t dev, bus_addr_t addr, uint32_t val)
{
	struct aw_prcm_softc *sc;

	sc = device_get_softc(dev);

	if (aw_prcm_check_addr(sc->res, addr) != 0)
		return (EINVAL);

	mtx_assert(&sc->mtx, MA_OWNED);
	bus_write_4(sc->res, addr - rman_get_start(sc->res), val);

	return (0);
}

static int
aw_prcm_read_4(device_t dev, bus_addr_t addr, uint32_t *val)
{
	struct aw_prcm_softc *sc;

	sc = device_get_softc(dev);

	if (aw_prcm_check_addr(sc->res, addr) != 0)
		return (EINVAL);

	mtx_assert(&sc->mtx, MA_OWNED);
	*val = bus_read_4(sc->res, addr - rman_get_start(sc->res));

	return (0);
}

static int
aw_prcm_modify_4(device_t dev, bus_addr_t addr, uint32_t clr, uint32_t set)
{
	struct aw_prcm_softc *sc;
	uint32_t val;

	sc = device_get_softc(dev);

	if (aw_prcm_check_addr(sc->res, addr) != 0)
		return (EINVAL);

	mtx_assert(&sc->mtx, MA_OWNED);
	val = bus_read_4(sc->res, addr - rman_get_start(sc->res));
	val &= ~clr;
	val |= set;
	bus_write_4(sc->res, addr - rman_get_start(sc->res), val);

	return (0);
}

static void
aw_prcm_device_lock(device_t dev)
{
	struct aw_prcm_softc *sc;

	sc = device_get_softc(dev);
	mtx_lock(&sc->mtx);
}

static void
aw_prcm_device_unlock(device_t dev)
{
	struct aw_prcm_softc *sc;

	sc = device_get_softc(dev);
	mtx_unlock(&sc->mtx);
}

static int
aw_prcm_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Allwinner Power/Reset/Clock Management");
	return (BUS_PROBE_GENERIC);
}

static int
aw_prcm_attach(device_t dev)
{
	struct aw_prcm_softc *sc;
	phandle_t node, child;
	device_t cdev;
	int error;

	sc = device_get_softc(dev);
	node = ofw_bus_get_node(dev);

	error = bus_alloc_resources(dev, aw_prcm_spec, &sc->res);
	if (error != 0) {
		device_printf(dev, "cannot allocate resources for device\n");
		return (error);
	}

	simplebus_init(dev, node);

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	/* Attach child devices */
	for (child = OF_child(node); child > 0; child = OF_peer(child)) {
		cdev = simplebus_add_device(dev, child, 0, NULL, -1, NULL);
		if (cdev != NULL)
			device_probe_and_attach(cdev);
	}

	return (bus_generic_attach(dev));
}

static device_method_t aw_prcm_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		aw_prcm_probe),
	DEVMETHOD(device_attach,	aw_prcm_attach),

	/* clkdev interface */
	DEVMETHOD(clkdev_write_4,	aw_prcm_write_4),
	DEVMETHOD(clkdev_read_4,	aw_prcm_read_4),
	DEVMETHOD(clkdev_modify_4,	aw_prcm_modify_4),
	DEVMETHOD(clkdev_device_lock,	aw_prcm_device_lock),
	DEVMETHOD(clkdev_device_unlock,	aw_prcm_device_unlock),

	DEVMETHOD_END
};

DEFINE_CLASS_1(aw_prcm, aw_prcm_driver, aw_prcm_methods,
    sizeof(struct aw_prcm_softc), simplebus_driver);

static devclass_t aw_prcm_devclass;

EARLY_DRIVER_MODULE(aw_prcm, simplebus, aw_prcm_driver, aw_prcm_devclass,
    0, 0, BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE);

MODULE_VERSION(aw_prcm, 1);
