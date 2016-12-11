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
 * Ingenic JZ4780 SMB Controller
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/time.h>
#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/iicbus/iiconf.h>
#include <dev/iicbus/iicbus.h>

#include <dev/extres/clk/clk.h>

#include <mips/ingenic/jz4780_smb.h>

#include "iicbus_if.h"

#define	JZSMB_TIMEOUT	(300UL * hz / 1000)

#ifndef timersub
#define timersub(tvp, uvp, vvp)                                         \
        do {                                                            \
                (vvp)->tv_sec = (tvp)->tv_sec - (uvp)->tv_sec;          \
                (vvp)->tv_usec = (tvp)->tv_usec - (uvp)->tv_usec;       \
                if ((vvp)->tv_usec < 0) {                               \
                        (vvp)->tv_sec--;                                \
                        (vvp)->tv_usec += 1000000;                      \
                }                                                       \
        } while (0)
#endif

static struct ofw_compat_data compat_data[] = {
	{ "ingenic,jz4780-i2c",		1 },
	{ NULL,				0 }
};

static struct resource_spec jzsmb_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ -1, 0 }
};

struct jzsmb_softc {
	struct resource	*res;
	struct mtx	mtx;
	clk_t		clk;
	device_t	iicbus;
	int		busy;
	uint32_t	i2c_freq;
	uint64_t	bus_freq;
	uint32_t	status;

	struct iic_msg	*msg;
};

#define	SMB_LOCK(sc)			mtx_lock(&(sc)->mtx)
#define	SMB_UNLOCK(sc)			mtx_unlock(&(sc)->mtx)
#define	SMB_ASSERT_LOCKED(sc)		mtx_assert(&(sc)->mtx, MA_OWNED)
#define	SMB_READ(sc, reg)		bus_read_2((sc)->res, (reg))
#define	SMB_WRITE(sc, reg, val)		bus_write_2((sc)->res, (reg), (val))

static phandle_t
jzsmb_get_node(device_t bus, device_t dev)
{
	return (ofw_bus_get_node(bus));
}

static int
jzsmb_enable(struct jzsmb_softc *sc, int enable)
{
	SMB_ASSERT_LOCKED(sc);

	if (enable) {
		SMB_WRITE(sc, SMBENB, SMBENB_SMBENB);
		while ((SMB_READ(sc, SMBENBST) & SMBENBST_SMBEN) == 0)
			;
	} else {
		SMB_WRITE(sc, SMBENB, 0);
		while ((SMB_READ(sc, SMBENBST) & SMBENBST_SMBEN) != 0)
			;
	}

	return (0);
}

static int
jzsmb_reset_locked(device_t dev, u_char speed, u_char addr, u_char *oldaddr)
{
	struct jzsmb_softc *sc;
	uint16_t con;
	uint32_t period;
	int hcnt, lcnt, setup_time, hold_time;

	sc = device_get_softc(dev);

	SMB_ASSERT_LOCKED(sc);

	/* Setup master mode operation */

	/* Disable SMB */
	jzsmb_enable(sc, 0);

	/* Disable interrupts */
	SMB_WRITE(sc, SMBINTM, 0);

	/* Set supported speed mode and expected SCL frequency */
	period = sc->bus_freq / sc->i2c_freq;
	con = SMBCON_REST | SMBCON_SLVDIS | SMBCON_MD;
	switch (sc->i2c_freq) {
	case 100000:
		con |= SMBCON_SPD_STANDARD;
		setup_time = 300;
		hold_time = 400;
		hcnt = (period * 4000) / (4700 + 4000);
		lcnt = period - hcnt;
		hcnt -= 8;
		if (hcnt < 6)
			hcnt = 6;
		lcnt -= 1;
		if (lcnt < 8)
			lcnt = 8;
		SMB_WRITE(sc, SMBCON, con);
		SMB_WRITE(sc, SMBSHCNT, hcnt);
		SMB_WRITE(sc, SMBSLCNT, lcnt);
		break;
	case 400000:
		con |= SMBCON_SPD_FAST;
		setup_time = 450;
		hold_time = 450;
		hcnt = (period * 600) / (1300 + 600);
		lcnt = period - hcnt;
		hcnt -= 8;
		if (hcnt < 6)
			hcnt = 6;
		lcnt -= 1;
		if (lcnt < 8)
			lcnt = 8;
		SMB_WRITE(sc, SMBCON, con);
		SMB_WRITE(sc, SMBFHCNT, hcnt);
		SMB_WRITE(sc, SMBFLCNT, lcnt);
		break;
	default:
		return (EINVAL);
	}

	setup_time = ((setup_time * sc->bus_freq / 1000) / 1000000) + 1;
	if (setup_time > 255)
		setup_time = 255;
	if (setup_time <= 0)
		setup_time = 1;
	SMB_WRITE(sc, SMBSDASU, setup_time);

	hold_time = ((hold_time * sc->bus_freq / 1000) / 1000000) - 1;
	if (hold_time > 255)
		hold_time = 255;

	if (hold_time >= 0)
		SMB_WRITE(sc, SMBSDAHD, hold_time | SMBSDAHD_HDENB);
	else
		SMB_WRITE(sc, SMBSDAHD, 0);

	SMB_WRITE(sc, SMBTAR, addr >> 1);

	if (addr != 0) {
		/* Enable SMB */
		jzsmb_enable(sc, 1);
	}

	return (0);
}

static int
jzsmb_reset(device_t dev, u_char speed, u_char addr, u_char *oldaddr)
{
	struct jzsmb_softc *sc;
	int error;

	sc = device_get_softc(dev);

	SMB_LOCK(sc);
	error = jzsmb_reset_locked(dev, speed, addr, oldaddr);
	SMB_UNLOCK(sc);

	return (error);
}

static int
jzsmb_transfer_read(device_t dev, struct iic_msg *msg)
{
	struct jzsmb_softc *sc;
	struct timeval start, cur, diff;
	uint16_t con, resid;
	int timeo;

	sc = device_get_softc(dev);
	timeo = JZSMB_TIMEOUT * msg->len;

	SMB_ASSERT_LOCKED(sc);

	con = SMB_READ(sc, SMBCON);
	con |= SMBCON_STPHLD;
	SMB_WRITE(sc, SMBCON, con);

	getmicrouptime(&start);
	for (resid = msg->len; resid > 0; resid--) {
		for (int i = 0; i < min(resid, 8); i++)
			SMB_WRITE(sc, SMBDC, SMBDC_CMD);
		for (;;) {
			getmicrouptime(&cur);
			timersub(&cur, &start, &diff);
			if ((SMB_READ(sc, SMBST) & SMBST_RFNE) != 0) {
				msg->buf[msg->len - resid] =
				    SMB_READ(sc, SMBDC) & SMBDC_DAT;
				break;
			} else
				DELAY(1000);

			if (tvtohz(&diff) >= timeo) {
				device_printf(dev,
				    "read timeout (slave=0x%02x, flags=0x%02x, status=0x%02x)\n",
				    msg->slave, msg->flags, SMB_READ(sc, SMBST));
				return (EIO);
			}
		}
	}

	con = SMB_READ(sc, SMBCON);
	con &= ~SMBCON_STPHLD;
	SMB_WRITE(sc, SMBCON, con);

	return (0);
}

static int
jzsmb_transfer_write(device_t dev, struct iic_msg *msg, int stop_hold)
{
	struct jzsmb_softc *sc;
	struct timeval start, cur, diff;
	uint16_t con, resid;
	int timeo;

	sc = device_get_softc(dev);
	timeo = JZSMB_TIMEOUT * msg->len;

	SMB_ASSERT_LOCKED(sc);

	con = SMB_READ(sc, SMBCON);
	con |= SMBCON_STPHLD;
	SMB_WRITE(sc, SMBCON, con);

	getmicrouptime(&start);
	for (resid = msg->len; resid > 0; resid--) {
		for (;;) {
			getmicrouptime(&cur);
			timersub(&cur, &start, &diff);
			if ((SMB_READ(sc, SMBST) & SMBST_TFNF) != 0) {
				SMB_WRITE(sc, SMBDC,
				    msg->buf[msg->len - resid]);
				break;
			} else
				DELAY((1000 * hz) / JZSMB_TIMEOUT);

			if (tvtohz(&diff) >= timeo) {
				device_printf(dev,
				    "write timeout (slave=0x%02x, flags=0x%02x, status=0x%02x)\n",
				    msg->slave, msg->flags, SMB_READ(sc, SMBST));
				return (EIO);
			}
		}
	}

	if (!stop_hold) {
		con = SMB_READ(sc, SMBCON);
		con &= ~SMBCON_STPHLD;
		SMB_WRITE(sc, SMBCON, con);
	}

	return (0);
}

static int
jzsmb_transfer(device_t dev, struct iic_msg *msgs, uint32_t nmsgs)
{
	struct jzsmb_softc *sc;
	uint32_t n;
	uint16_t con;
	int error;

	sc = device_get_softc(dev);

	SMB_LOCK(sc);
	while (sc->busy)
		mtx_sleep(sc, &sc->mtx, 0, "i2cbuswait", 0);
	sc->busy = 1;
	sc->status = 0;

	for (n = 0; n < nmsgs; n++) {
		/* Set target address */
		if (n == 0 || msgs[n].slave != msgs[n - 1].slave)
			jzsmb_reset_locked(dev, sc->i2c_freq,
			    msgs[n].slave, NULL);

		/* Set read or write */
		if ((msgs[n].flags & IIC_M_RD) != 0)
			error = jzsmb_transfer_read(dev, &msgs[n]);
		else
			error = jzsmb_transfer_write(dev, &msgs[n],
			    n < nmsgs - 1);

		if (error != 0)
			goto done;
	}

done:
	/* Send stop if necessary */
	con = SMB_READ(sc, SMBCON);
	con &= ~SMBCON_STPHLD;
	SMB_WRITE(sc, SMBCON, con);

	/* Disable SMB */
	jzsmb_enable(sc, 0);

	sc->msg = NULL;
	sc->busy = 0;
	wakeup(sc);
	SMB_UNLOCK(sc);

	return (error);
}

static int
jzsmb_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Ingenic JZ4780 SMB Controller");

	return (BUS_PROBE_DEFAULT);
}

static int
jzsmb_attach(device_t dev)
{
	struct jzsmb_softc *sc;
	phandle_t node;
	int error;

	sc = device_get_softc(dev);
	node = ofw_bus_get_node(dev);
	mtx_init(&sc->mtx, device_get_nameunit(dev), "jzsmb", MTX_DEF);

	error = clk_get_by_ofw_index(dev, 0, 0, &sc->clk);
	if (error != 0) {
		device_printf(dev, "cannot get clock\n");
		goto fail;
	}
	error = clk_enable(sc->clk);
	if (error != 0) {
		device_printf(dev, "cannot enable clock\n");
		goto fail;
	}
	error = clk_get_freq(sc->clk, &sc->bus_freq);
	if (error != 0 || sc->bus_freq == 0) {
		device_printf(dev, "cannot get bus frequency\n");
		return (error);
	}

	if (bus_alloc_resources(dev, jzsmb_spec, &sc->res) != 0) {
		device_printf(dev, "cannot allocate resources for device\n");
		error = ENXIO;
		goto fail;
	}

	if (OF_getencprop(node, "clock-frequency", &sc->i2c_freq,
	    sizeof(sc->i2c_freq)) != 0 || sc->i2c_freq == 0)
		sc->i2c_freq = 100000;	/* Default to standard mode */

	sc->iicbus = device_add_child(dev, "iicbus", -1);
	if (sc->iicbus == NULL) {
		device_printf(dev, "cannot add iicbus child device\n");
		error = ENXIO;
		goto fail;
	}

	bus_generic_attach(dev);

	return (0);

fail:
	bus_release_resources(dev, jzsmb_spec, &sc->res);
	if (sc->clk != NULL)
		clk_release(sc->clk);
	mtx_destroy(&sc->mtx);
	return (error);
}

static device_method_t jzsmb_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		jzsmb_probe),
	DEVMETHOD(device_attach,	jzsmb_attach),

	/* Bus interface */
	DEVMETHOD(bus_setup_intr,	bus_generic_setup_intr),
	DEVMETHOD(bus_teardown_intr,	bus_generic_teardown_intr),
	DEVMETHOD(bus_alloc_resource,	bus_generic_alloc_resource),
	DEVMETHOD(bus_release_resource,	bus_generic_release_resource),
	DEVMETHOD(bus_activate_resource, bus_generic_activate_resource),
	DEVMETHOD(bus_deactivate_resource, bus_generic_deactivate_resource),
	DEVMETHOD(bus_adjust_resource,	bus_generic_adjust_resource),
	DEVMETHOD(bus_set_resource,	bus_generic_rl_set_resource),
	DEVMETHOD(bus_get_resource,	bus_generic_rl_get_resource),

	/* OFW methods */
	DEVMETHOD(ofw_bus_get_node,	jzsmb_get_node),

	/* iicbus interface */
	DEVMETHOD(iicbus_callback,	iicbus_null_callback),
	DEVMETHOD(iicbus_reset,		jzsmb_reset),
	DEVMETHOD(iicbus_transfer,	jzsmb_transfer),

	DEVMETHOD_END
};

static driver_t jzsmb_driver = {
	"iichb",
	jzsmb_methods,
	sizeof(struct jzsmb_softc),
};

static devclass_t jzsmb_devclass;

EARLY_DRIVER_MODULE(iicbus, jzsmb, iicbus_driver, iicbus_devclass, 0, 0,
    BUS_PASS_RESOURCE + BUS_PASS_ORDER_MIDDLE);
EARLY_DRIVER_MODULE(jzsmb, simplebus, jzsmb_driver, jzsmb_devclass, 0, 0,
    BUS_PASS_RESOURCE + BUS_PASS_ORDER_MIDDLE);
MODULE_VERSION(jzsmb, 1);
