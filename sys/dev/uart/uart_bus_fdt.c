/*-
 * Copyright (c) 2009-2010 The FreeBSD Foundation
 * All rights reserved.
 *
 * This software was developed by Semihalf under sponsorship from
 * the FreeBSD Foundation.
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
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
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

#include "opt_platform.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>

#include <machine/bus.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/uart/uart.h>
#include <dev/uart/uart_bus.h>
#include <dev/uart/uart_cpu.h>
#include <dev/uart/uart_cpu_fdt.h>

#ifdef EXT_RESOURCES
#include <dev/extres/clk/clk.h>
#include <dev/extres/hwreset/hwreset.h>
#endif

static int uart_fdt_probe(device_t);
static int uart_fdt_attach(device_t);
static int uart_fdt_detach(device_t);

static device_method_t uart_fdt_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		uart_fdt_probe),
	DEVMETHOD(device_attach,	uart_fdt_attach),
	DEVMETHOD(device_detach,	uart_fdt_detach),
	{ 0, 0 }
};

struct uart_fdt_softc {
	struct uart_softc	uart_sc;
#ifdef EXT_RESOURCES
	clk_t			baudclk;
	clk_t			apb_pclk;
	hwreset_t		reset;
#endif
};

static driver_t uart_fdt_driver = {
	uart_driver_name,
	uart_fdt_methods,
	sizeof(struct uart_softc),
};

#ifdef EXT_RESOURCES
static int
uart_fdt_get_clock_extres(device_t dev, clk_t *baudclk, clk_t *apb_pclk)
{
	/* Baud clock is either the named clock "baudclk" or the first clock */
	if (clk_get_by_ofw_name(dev, "baudclk", baudclk) != 0 &&
	    clk_get_by_ofw_index(dev, 0, baudclk) != 0)
		return (ENOENT);

	if (apb_pclk == NULL)
		return (0);

	/* apb_pclk is optional */
	(void)clk_get_by_ofw_name(dev, "apb_pclk", apb_pclk);

	return (0);
}
#endif

int
uart_fdt_get_clock(device_t dev, phandle_t node, pcell_t *cell)
{
#ifdef EXT_RESOURCES
	uint64_t freq;
	clk_t clk;
	int error;
#endif

	/* clock-frequency is a FreeBSD-only extention. */
	if ((OF_getencprop(node, "clock-frequency", cell, sizeof(*cell))) > 0)
		return (0);

	/* Try to retrieve parent 'bus-frequency' */
	/* XXX this should go to simple-bus fixup or so */
	if ((OF_getencprop(OF_parent(node), "bus-frequency", cell,
	    sizeof(*cell))) > 0)
		return (0);

#ifdef EXT_RESOURCES
	/* Get clock rate from clk API */
	if (dev != NULL && uart_fdt_get_clock_extres(dev, &clk, NULL) == 0) {
		error = clk_get_freq(clk, &freq);
		clk_release(clk);
		if (error == 0) {
			*cell = (pcell_t)freq;
			return (0);
		}
	}
#endif

	/* Not found */
	*cell = 0;
	return (0);
}

int
uart_fdt_get_shift(phandle_t node, pcell_t *cell)
{

	if ((OF_getencprop(node, "reg-shift", cell, sizeof(*cell))) <= 0)
		return (-1);
	return (0);
}

static uintptr_t
uart_fdt_find_device(device_t dev)
{
	struct ofw_compat_data **cd;
	const struct ofw_compat_data *ocd;

	SET_FOREACH(cd, uart_fdt_class_and_device_set) {
		ocd = ofw_bus_search_compatible(dev, *cd);
		if (ocd->ocd_data != 0)
			return (ocd->ocd_data);
	}
	return (0);
}

static int
uart_fdt_probe(device_t dev)
{
	struct uart_softc *sc;
	phandle_t node;
	pcell_t clock, shift;
	int err;

	sc = device_get_softc(dev);

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	sc->sc_class = (struct uart_class *)uart_fdt_find_device(dev);
	if (sc->sc_class == NULL)
		return (ENXIO);

	node = ofw_bus_get_node(dev);

	if ((err = uart_fdt_get_clock(dev, node, &clock)) != 0)
		return (err);
	if (uart_fdt_get_shift(node, &shift) != 0)
		shift = uart_getregshift(sc->sc_class);

	return (uart_bus_probe(dev, (int)shift, (int)clock, 0, 0));
}

static int
uart_fdt_attach(device_t dev)
{
#ifdef EXT_RESOURCES
	struct uart_fdt_softc *sc;
	int error;

	sc = device_get_softc(dev);

	if (uart_fdt_get_clock_extres(dev, &sc->baudclk, &sc->apb_pclk) == 0) {
		error = clk_enable(sc->baudclk);
		if (error != 0) {
			device_printf(dev, "cannot enable baud clock\n");
			return (error);
		}
		if (sc->apb_pclk != NULL) {
			error = clk_enable(sc->apb_pclk);
			if (error != 0) {
				device_printf(dev,
				    "cannot enable periph clock\n");
				return (error);
			}
		}
	}

	if (hwreset_get_by_ofw_idx(dev, 0, &sc->reset) == 0) {
		error = hwreset_deassert(sc->reset);
		if (error != 0) {
			device_printf(dev, "cannot de-assert reset\n");
			return (error);
		}
	}
#endif

	return (uart_bus_attach(dev));
}

static int
uart_fdt_detach(device_t dev)
{
#ifdef EXT_RESOURCES
	struct uart_fdt_softc *sc;
#endif
	int error;

	error = uart_bus_detach(dev);
	if (error != 0)
		return (error);

#ifdef EXT_RESOURCES
	sc = device_get_softc(dev);

	if (sc->reset != NULL) {
		error = hwreset_assert(sc->reset);
		if (error != 0) {
			device_printf(dev, "cannot assert reset\n");
			return (error);
		}
		hwreset_release(sc->reset);
	}

	if (sc->apb_pclk != NULL) {
		error = clk_disable(sc->apb_pclk);
		if (error != 0)
			return (error);
		error = clk_release(sc->apb_pclk);
		if (error != 0)
			return (error);
	}
	if (sc->baudclk != NULL) {
		error = clk_disable(sc->baudclk);
		if (error != 0)
			return (error);
		error = clk_release(sc->baudclk);
		if (error != 0)
			return (error);
	}
#endif

	return (0);
}

DRIVER_MODULE(uart, simplebus, uart_fdt_driver, uart_devclass, 0, 0);
DRIVER_MODULE(uart, ofwbus, uart_fdt_driver, uart_devclass, 0, 0);
