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
 * Allwinner USB PHY
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/gpio.h>
#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/extres/clk/clk.h>
#include <dev/extres/hwreset/hwreset.h>

#include "gpio_if.h"

#define	USBPHY_NUMOFF		3
#define	GPIO_POLARITY(flags)	(((flags) & 1) ? GPIO_PIN_LOW : GPIO_PIN_HIGH)

static struct ofw_compat_data compat_data[] = {
	{ "allwinner,sun4i-a10-usb-phy",	1 },
	{ "allwinner,sun5i-a13-usb-phy",	1 },
	{ "allwinner,sun6i-a31-usb-phy",	1 },
	{ "allwinner,sun7i-a20-usb-phy",	1 },
	{ NULL,					0 }
};

static struct resource_spec awusbphy_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ -1, 0 }
};

static int
awusbphy_gpio_set(device_t dev, phandle_t node, const char *pname)
{
	pcell_t gpio_prop[4];
	phandle_t gpio_node;
	device_t gpio_dev;
	uint32_t pin, flags;
	ssize_t len;
	int val;

	len = OF_getencprop(node, pname, gpio_prop, sizeof(gpio_prop));
	if (len == -1)
		return (0);

	if (len != sizeof(gpio_prop)) {
		device_printf(dev, "property %s length was %d, expected %d\n",
		    pname, len, sizeof(gpio_prop));
		return (ENXIO);
	}

	gpio_node = OF_node_from_xref(gpio_prop[0]);
	gpio_dev = OF_device_from_xref(gpio_prop[0]);
	if (gpio_dev == NULL) {
		device_printf(dev, "failed to get the GPIO device for %s\n",
		    pname);
		return (ENOENT);
	}

	if (GPIO_MAP_GPIOS(gpio_dev, node, gpio_node,
	    sizeof(gpio_prop) / sizeof(gpio_prop[0]) - 1, gpio_prop + 1,
	    &pin, &flags) != 0) {
		device_printf(dev, "failed to map the GPIO pin for %s\n",
		    pname);
		return (ENXIO);
	}

	val = GPIO_POLARITY(flags);

	GPIO_PIN_SETFLAGS(gpio_dev, pin, GPIO_PIN_OUTPUT);
	GPIO_PIN_SET(gpio_dev, pin, val);

	return (0);
}

static int
awusbphy_supply_set(device_t dev, const char *pname)
{
	phandle_t node, reg_node;
	pcell_t reg_xref;

	node = ofw_bus_get_node(dev);

	if (OF_getencprop(node, pname, &reg_xref, sizeof(reg_xref)) == -1)
		return (0);

	reg_node = OF_node_from_xref(reg_xref);

	return (awusbphy_gpio_set(dev, reg_node, "gpio"));
}

static int
awusbphy_init(device_t dev, struct resource *res)
{
	char pname[20];
	phandle_t node;
	int error, off;
	hwreset_t rst;
	uint32_t val;
	clk_t clk;

	node = ofw_bus_get_node(dev);

	/* Enable clocks */
	for (off = 0; clk_get_by_ofw_index(dev, off, &clk) == 0; off++) {
		error = clk_enable(clk);
		if (error != 0) {
			device_printf(dev, "couldn't enable clock %s\n",
			    clk_get_name(clk));
			return (error);
		}
	}

	/* De-assert resets */
	for (off = 0; hwreset_get_by_ofw_idx(dev, off, &rst) == 0; off++) {
		error = hwreset_deassert(rst);
		if (error != 0) {
			device_printf(dev, "couldn't de-assert reset %d\n",
			    off);
			return (error);
		}
	}

	/* Configure GPIOs */
	for (off = 0; off < USBPHY_NUMOFF; off++) {
		snprintf(pname, sizeof(pname), "usb%d_id_det-gpio", off);
		error = awusbphy_gpio_set(dev, node, pname);
		if (error)
			return (error);

		snprintf(pname, sizeof(pname), "usb%d_vbus_det-gpio", off);
		error = awusbphy_gpio_set(dev, node, pname);
		if (error)
			return (error);

		snprintf(pname, sizeof(pname), "usb%d_vbus-supply", off);
		error = awusbphy_supply_set(dev, pname);
		if (error)
			return (error);
	}

#define PHY_CSR			0x00
#define	ID_PULLUP_EN		(1 << 17)
#define	DPDM_PULLUP_EN		(1 << 16)
#define	FORCE_ID		(0x3 << 14)
#define	FORCE_ID_SHIFT		14
#define	FORCE_ID_LOW		2
#define	FORCE_VBUS_VALID	(0x3 << 12)
#define	FORCE_VBUS_VALID_SHIFT	12
#define	FORCE_VBUS_VALID_HIGH	3
#define	VBUS_CHANGE_DET		(1 << 6)
#define	ID_CHANGE_DET		(1 << 5)
#define	DPDM_CHANGE_DET		(1 << 4)

	/* Enable OTG PHY */
	val = bus_read_4(res, PHY_CSR);
	val &= ~(VBUS_CHANGE_DET | ID_CHANGE_DET | DPDM_CHANGE_DET);
	val |= (ID_PULLUP_EN | DPDM_PULLUP_EN);
	val &= ~FORCE_ID;
	val |= (FORCE_ID_LOW << FORCE_ID_SHIFT);
	val &= ~FORCE_VBUS_VALID;
	val |= (FORCE_VBUS_VALID_HIGH << FORCE_VBUS_VALID_SHIFT);
	bus_write_4(res, PHY_CSR, val);

	return (0);
}

static int
awusbphy_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Allwinner USB PHY");
	return (BUS_PROBE_DEFAULT);
}

static int
awusbphy_attach(device_t dev)
{
	struct resource *res;
	int error;

	error = bus_alloc_resources(dev, awusbphy_spec, &res);
	if (error != 0) {
		device_printf(dev, "failed to allocate bus resources\n");
		return (error);
	}

	error = awusbphy_init(dev, res);
	if (error)
		device_printf(dev, "failed to initialize USB PHY, error %d\n",
		    error);

	bus_release_resources(dev, awusbphy_spec, &res);

	return (error);
}

static device_method_t awusbphy_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		awusbphy_probe),
	DEVMETHOD(device_attach,	awusbphy_attach),

	DEVMETHOD_END
};

static driver_t awusbphy_driver = {
	"awusbphy",
	awusbphy_methods,
	0,
};

static devclass_t awusbphy_devclass;

EARLY_DRIVER_MODULE(awusbphy, simplebus, awusbphy_driver, awusbphy_devclass,
    0, 0, BUS_PASS_TIMER + BUS_PASS_ORDER_MIDDLE);
MODULE_VERSION(awusbphy, 1);
