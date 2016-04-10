/*-
 * Copyright (c) 2015 Emmanuel Vadot <manu@bidouilliste.com>
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
* X-Power AXP209/AXP221 PMU for Allwinner SoCs
*/
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/eventhandler.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/clock.h>
#include <sys/time.h>
#include <sys/bus.h>
#include <sys/proc.h>
#include <sys/reboot.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <sys/sysctl.h>

#include <dev/iicbus/iicbus.h>
#include <dev/iicbus/iiconf.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "iicbus_if.h"

/* Power State Register */
#define	AXP_PSR			0x00
#define	AXP_PSR_ACIN		0x80
#define	AXP_PSR_ACIN_SHIFT	7
#define	AXP_PSR_VBUS		0x20
#define	AXP_PSR_VBUS_SHIFT	5

/* Shutdown and battery control */
#define	AXP_SHUTBAT		0x32
#define	AXP_SHUTBAT_SHUTDOWN	0x80

/* Temperature monitor */
#define	AXP209_TEMPMON		0x5e
#define	AXP209_TEMPMON_MIN	1447	/* -144.7C */
#define	AXP221_TEMPMON		0x56
#define	AXP221_TEMPMON_MIN	2437	/* -243.7C */

#define	AXP_TEMPMON_H(a)	((a) << 4)
#define	AXP_TEMPMON_L(a)	((a) & 0xf)
#define	AXP_0C_TO_K		2732

struct axp2xx_conf {
	const char *	desc;
	u_int		tempmon_reg;
	int		tempmon_min;
}; 

static const struct axp2xx_conf axp209_conf = {
	.desc = "X-Power AXP209 Power Management Unit",
	.tempmon_reg = AXP209_TEMPMON,
	.tempmon_min = AXP209_TEMPMON_MIN,
};

static const struct axp2xx_conf axp221_conf = {
	.desc = "X-Power AXP221 Power Management Unit",
	.tempmon_reg = AXP221_TEMPMON,
	.tempmon_min = AXP221_TEMPMON_MIN,
};

static struct ofw_compat_data compat_data[] = {
	{ "x-powers,axp209",	(uintptr_t)&axp209_conf },
	{ "x-powers,axp221",	(uintptr_t)&axp221_conf },
	{ NULL,			(uintptr_t)NULL }
};

#define	AXP_CONF(d)	\
	(void *)ofw_bus_search_compatible((d), compat_data)->ocd_data

struct axp2xx_softc {
	uint32_t			addr;
	const struct axp2xx_conf *	conf;
	struct intr_config_hook 	enum_hook;
};

enum axp2xx_sensor {
	AXP_TEMP
};

static int
axp2xx_read(device_t dev, uint8_t reg, uint8_t *data, uint8_t size)
{
	struct axp2xx_softc *sc = device_get_softc(dev);
	struct iic_msg msg[2];

	msg[0].slave = sc->addr;
	msg[0].flags = IIC_M_WR;
	msg[0].len = 1;
	msg[0].buf = &reg;

	msg[1].slave = sc->addr;
	msg[1].flags = IIC_M_RD;
	msg[1].len = size;
	msg[1].buf = data;

	return (iicbus_transfer(dev, msg, 2));
}

static int
axp2xx_write(device_t dev, uint8_t reg, uint8_t data)
{
	struct axp2xx_softc *sc = device_get_softc(dev);
	struct iic_msg msg[2];

	msg[0].slave = sc->addr;
	msg[0].flags = IIC_M_WR;
	msg[0].len = 1;
	msg[0].buf = &reg;

	msg[1].slave = sc->addr;
	msg[1].flags = IIC_M_WR;
	msg[1].len = 1;
	msg[1].buf = &data;

	return (iicbus_transfer(dev, msg, 2));
}

static int
axp2xx_sysctl(SYSCTL_HANDLER_ARGS)
{
	device_t dev = arg1;
	enum axp2xx_sensor sensor = arg2;
	struct axp2xx_softc *sc;
	uint8_t data[2];
	int val, error;

	if (sensor != AXP_TEMP)
		return (ENOENT);

	sc = device_get_softc(dev);

	error = axp2xx_read(dev, sc->conf->tempmon_reg, data, 2);
	if (error != 0)
		return (error);

	/* Temperature is between -144.7C and 264.8C, step +0.1C */
	val = (AXP_TEMPMON_H(data[0]) | AXP_TEMPMON_L(data[1])) -
	    sc->conf->tempmon_min + AXP_0C_TO_K;

	return sysctl_handle_opaque(oidp, &val, sizeof(val), req);
}

static void
axp2xx_shutdown(void *devp, int howto)
{
	device_t dev;

	if (!(howto & RB_POWEROFF))
		return;
	dev = (device_t)devp;

	if (bootverbose)
		device_printf(dev, "Shutdown AXP PMU\n");

	axp2xx_write(dev, AXP_SHUTBAT, AXP_SHUTBAT_SHUTDOWN);
}

static int
axp2xx_probe(device_t dev)
{
	const struct axp2xx_conf *conf;

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	conf = AXP_CONF(dev);
	if (conf == NULL)
		return (ENXIO);

	device_set_desc(dev, conf->desc);

	return (BUS_PROBE_DEFAULT);
}

static int
axp2xx_attach(device_t dev)
{
	struct axp2xx_softc *sc;
	const char *pwr_name[] = {"Battery", "AC", "USB", "AC and USB"};
	uint8_t data;
	uint8_t pwr_src;

	sc = device_get_softc(dev);

	sc->addr = iicbus_get_addr(dev);
	sc->conf = AXP_CONF(dev);

	if (bootverbose) {
		/*
		 * Read the Power State register.
		 * Shift the AC presence into bit 0.
		 * Shift the Battery presence into bit 1.
		 */
		axp2xx_read(dev, AXP_PSR, &data, 1);
		pwr_src = ((data & AXP_PSR_ACIN) >> AXP_PSR_ACIN_SHIFT) |
		    ((data & AXP_PSR_VBUS) >> (AXP_PSR_VBUS_SHIFT - 1));

		device_printf(dev, "AXP Powered by %s\n", pwr_name[pwr_src]);
	}

	EVENTHANDLER_REGISTER(shutdown_final, axp2xx_shutdown, dev,
	    SHUTDOWN_PRI_LAST);

	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)),
	    OID_AUTO, "temp",
	    CTLTYPE_INT | CTLFLAG_RD,
	    dev, AXP_TEMP, axp2xx_sysctl, "IK", "Internal temperature");

	return (0);
}

static device_method_t axp2xx_methods[] = {
	DEVMETHOD(device_probe,		axp2xx_probe),
	DEVMETHOD(device_attach,	axp2xx_attach),
	{0, 0},
};

static driver_t axp2xx_driver = {
	"axp2xx_pmu",
	axp2xx_methods,
	sizeof(struct axp2xx_softc),
};

static devclass_t axp2xx_devclass;

DRIVER_MODULE(axp2xx, iicbus, axp2xx_driver, axp2xx_devclass, 0, 0);
MODULE_VERSION(axp2xx, 1);
MODULE_DEPEND(axp2xx, iicbus, 1, 1, 1);
