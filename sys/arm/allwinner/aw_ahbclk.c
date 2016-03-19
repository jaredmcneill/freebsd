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
 * Allwinner AHB clock
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

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/extres/clk/clk_mux.h>

#include "clkdev_if.h"

#define	AHB_CLK_SRC_SEL_WIDTH	2
#define	AHB_CLK_SRC_SEL_SHIFT	6

struct aw_ahbclk_softc {
	struct resource		*res;
	struct mtx		mtx;
};

struct resource_spec aw_ahbclk_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ -1, 0 }
};

static const char *aw_ahbclk_srcs[] = { "axi", "pll6_div_4", "pll6_other" };

static int
aw_ahbclk_write_4(device_t dev, bus_addr_t addr, uint32_t val)
{
	struct aw_ahbclk_softc *sc;

	sc = device_get_softc(dev);
	bus_write_4(sc->res, addr, val);

	return (0);
}

static int
aw_ahbclk_read_4(device_t dev, bus_addr_t addr, uint32_t *val)
{
	struct aw_ahbclk_softc *sc;

	sc = device_get_softc(dev);
	*val = bus_read_4(sc->res, addr);

	return (0);
}

static int
aw_ahbclk_modify_4(device_t dev, bus_addr_t addr, uint32_t clr, uint32_t set)
{
	struct aw_ahbclk_softc *sc;
	uint32_t val;

	sc = device_get_softc(dev);
	val = bus_read_4(sc->res, addr);
	bus_write_4(sc->res, addr, (val & ~clr) | set);

	return (0);
}

static void
aw_ahbclk_device_lock(device_t dev)
{
	struct aw_ahbclk_softc *sc;

	sc = device_get_softc(dev);
	mtx_lock(&sc->mtx);
}

static void
aw_ahbclk_device_unlock(device_t dev)
{
	struct aw_ahbclk_softc *sc;

	sc = device_get_softc(dev);
	mtx_unlock(&sc->mtx);
}

static int
aw_ahbclk_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "allwinner,sun5i-a13-ahb-clk"))
		return (ENXIO);

	device_set_desc(dev, "Allwinner AHB Clock");
	return (BUS_PROBE_DEFAULT);
}

static int
aw_ahbclk_attach(device_t dev)
{
	struct aw_ahbclk_softc *sc;
	struct clk_mux_def def;
	struct clkdom *clkdom;
	phandle_t node;
	int error;

	clkdom = NULL;
	node = ofw_bus_get_node(dev);
	sc = device_get_softc(dev);

	if (bus_alloc_resources(dev, aw_ahbclk_spec, &sc->res) != 0) {
		device_printf(dev, "cannot allocate resources\n");
		return (ENXIO);
	}

	clkdom = clkdom_create(dev);

	memset(&def, 0, sizeof(def));
	def.clkdef.id = 0;
	def.clkdef.parent_names = aw_ahbclk_srcs;
	def.clkdef.parent_cnt = nitems(aw_ahbclk_srcs);
	def.offset = 0;
	def.shift = AHB_CLK_SRC_SEL_SHIFT;
	def.width = AHB_CLK_SRC_SEL_WIDTH;

	error = clk_parse_ofw_clk_name(dev, node, &def.clkdef.name);
	if (error != 0) {
		device_printf(dev, "cannot parse clock name\n");
		error = ENXIO;
		goto fail;
	}

	error = clknode_mux_register(clkdom, &def);
	if (error != 0) {
		device_printf(dev, "cannot register mux clock\n");
		error = ENXIO;
		goto fail;
	}

	error = clkdom_finit(clkdom);
	if (error != 0) {
		device_printf(dev, "cannot finalize clkdom initialization\n");
		error = ENXIO;
		goto fail;
	}

	free(__DECONST(char *, def.clkdef.name), M_OFWPROP);

	if (bootverbose)
		clkdom_dump(clkdom);

	return (0);

fail:
	free(__DECONST(char *, def.clkdef.name), M_OFWPROP);
	bus_release_resources(dev, aw_ahbclk_spec, &sc->res);
	return (error);
}

static device_method_t aw_ahbclk_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		aw_ahbclk_probe),
	DEVMETHOD(device_attach,	aw_ahbclk_attach),

	/* clkdev interface */
	DEVMETHOD(clkdev_write_4,	aw_ahbclk_write_4),
	DEVMETHOD(clkdev_read_4,	aw_ahbclk_read_4),
	DEVMETHOD(clkdev_modify_4,	aw_ahbclk_modify_4),
	DEVMETHOD(clkdev_device_lock,	aw_ahbclk_device_lock),
	DEVMETHOD(clkdev_device_unlock,	aw_ahbclk_device_unlock),

	DEVMETHOD_END
};

static driver_t aw_ahbclk_driver = {
	"aw_ahbclk",
	aw_ahbclk_methods,
	sizeof(struct aw_ahbclk_softc),
};

static devclass_t aw_ahbclk_devclass;

EARLY_DRIVER_MODULE(aw_ahbclk, simplebus, aw_ahbclk_driver,
    aw_ahbclk_devclass, 0, 0, BUS_PASS_TIMER);
