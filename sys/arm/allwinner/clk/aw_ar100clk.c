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
 * Allwinner A31 AR100 clock
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
#include <dev/ofw/ofw_subr.h>

#include <dev/extres/clk/clk.h>

#include "clkdev_if.h"

#define	AR100_CLK_DIV		(0x1f << 8)
#define	AR100_CLK_DIV_SHIFT	8
#define	AR100_CLK_SHIFT		(0x3 << 4)
#define	AR100_CLK_SHIFT_SHIFT	4
#define	AR100_SRC_SEL		(0x3 << 16)
#define	AR100_SRC_SEL_MAX	3
#define	AR100_SRC_SEL_SHIFT	16

static struct ofw_compat_data compat_data[] = {
	{ "allwinner,sun6i-a31-ar100-clk",	1 },
	{ NULL, 0 }
};

struct aw_ar100clk_sc {
	device_t		clkdev;
	bus_addr_t		reg;
};

#define	CLK_READ(sc, val)	CLKDEV_READ_4((sc)->clkdev, (sc)->reg, (val))
#define	CLK_WRITE(sc, val)	CLKDEV_WRITE_4((sc)->clkdev, (sc)->reg, (val))
#define	DEVICE_LOCK(sc)		CLKDEV_DEVICE_LOCK((sc)->clkdev)
#define	DEVICE_UNLOCK(sc)	CLKDEV_DEVICE_UNLOCK((sc)->clkdev)

static int
aw_ar100clk_init(struct clknode *clk, device_t dev)
{
	struct aw_ar100clk_sc *sc;
	uint32_t val, index;

	sc = clknode_get_softc(clk);

	DEVICE_LOCK(sc);
	CLK_READ(sc, &val);
	DEVICE_UNLOCK(sc);

	index = (val & AR100_SRC_SEL) >> AR100_SRC_SEL_SHIFT;

	clknode_init_parent_idx(clk, index);
	return (0);
}

static int
aw_ar100clk_recalc_freq(struct clknode *clk, uint64_t *freq)
{
	struct aw_ar100clk_sc *sc;
	uint32_t val, shift, div;

	sc = clknode_get_softc(clk);

	DEVICE_LOCK(sc);
	CLK_READ(sc, &val);
	DEVICE_UNLOCK(sc);

	div = (val & AR100_CLK_DIV) >> AR100_CLK_DIV_SHIFT;
	shift = (val & AR100_CLK_SHIFT) >> AR100_CLK_SHIFT_SHIFT;

	*freq = (*freq >> shift) / (div + 1);

	return (0);
}

static int
aw_ar100clk_set_mux(struct clknode *clk, int index)
{
	struct aw_ar100clk_sc *sc;
	uint32_t val;

	sc = clknode_get_softc(clk);

	if (index < 0 || index > AR100_SRC_SEL_MAX)
		return (ERANGE);

	DEVICE_LOCK(sc);
	CLK_READ(sc, &val);
	val &= ~AR100_SRC_SEL;
	val |= (index << AR100_SRC_SEL_SHIFT);
	CLK_WRITE(sc, val);
	DEVICE_UNLOCK(sc);

	return (0);
}

static clknode_method_t aw_ar100clk_clknode_methods[] = {
	/* Device interface */
	CLKNODEMETHOD(clknode_init,		aw_ar100clk_init),
	CLKNODEMETHOD(clknode_recalc_freq,	aw_ar100clk_recalc_freq),
	CLKNODEMETHOD(clknode_set_mux,		aw_ar100clk_set_mux),
	CLKNODEMETHOD_END
};
DEFINE_CLASS_1(aw_ar100clk_clknode, aw_ar100clk_clknode_class,
    aw_ar100clk_clknode_methods, sizeof(struct aw_ar100clk_sc), clknode_class);

static int
aw_ar100clk_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Allwinner A31 AR100 Clock");
	return (BUS_PROBE_DEFAULT);
}

static int
aw_ar100clk_attach(device_t dev)
{
	struct clknode_init_def def;
	struct aw_ar100clk_sc *sc;
	struct clkdom *clkdom;
	struct clknode *clk;
	clk_t clk_parent;
	bus_addr_t paddr;
	bus_size_t psize;
	phandle_t node;
	int error, ncells, i;

	node = ofw_bus_get_node(dev);

	if (ofw_reg_to_paddr(node, 0, &paddr, &psize, NULL) != 0) {
		device_printf(dev, "cannot parse 'reg' property\n");
		return (ENXIO);
	}

	error = ofw_bus_parse_xref_list_get_length(node, "clocks",
	    "#clock-cells", &ncells);
	if (error != 0) {
		device_printf(dev, "cannot get clock count\n");
		return (error);
	}

	clkdom = clkdom_create(dev);

	memset(&def, 0, sizeof(def));
	def.id = 1;
	def.parent_names = malloc(sizeof(char *) * ncells, M_OFWPROP,
	    M_WAITOK);
	for (i = 0; i < ncells; i++) {
		error = clk_get_by_ofw_index(dev, i, &clk_parent);
		if (error != 0) {
			device_printf(dev, "cannot get clock %d\n", i);
			goto fail;
		}
		def.parent_names[i] = clk_get_name(clk_parent);
		clk_release(clk_parent);
	}
	def.parent_cnt = ncells;

	error = clk_parse_ofw_clk_name(dev, node, &def.name);
	if (error != 0) {
		device_printf(dev, "cannot parse clock name\n");
		error = ENXIO;
		goto fail;
	}

	clk = clknode_create(clkdom, &aw_ar100clk_clknode_class, &def);
	if (clk == NULL) {
		device_printf(dev, "cannot create clknode\n");
		error = ENXIO;
		goto fail;
	}
	sc = clknode_get_softc(clk);
	sc->reg = paddr;
	sc->clkdev = device_get_parent(dev);

	clknode_register(clkdom, clk);

	if (clkdom_finit(clkdom) != 0) {
		device_printf(dev, "cannot finalize clkdom initialization\n");
		error = ENXIO;
		goto fail;
	}

	if (bootverbose)
		clkdom_dump(clkdom);

	return (0);

fail:
	return (error);
}

static device_method_t aw_ar100clk_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		aw_ar100clk_probe),
	DEVMETHOD(device_attach,	aw_ar100clk_attach),

	DEVMETHOD_END
};

static driver_t aw_ar100clk_driver = {
	"aw_ar100clk",
	aw_ar100clk_methods,
	0
};

static devclass_t aw_ar100clk_devclass;

EARLY_DRIVER_MODULE(aw_ar100clk, simplebus, aw_ar100clk_driver,
    aw_ar100clk_devclass, 0, 0, BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE);
