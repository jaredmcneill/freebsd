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
 * Allwinner P2WI (Push-Pull Two Wire Interface)
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

#include <dev/iicbus/iiconf.h>

#include "iicbus_if.h"

#define	P2WI_CTRL		0x00
#define	 START_TRANS		(1 << 7)
#define	 GLOBAL_INT_ENB		(1 << 1)
#define	 SOFT_RESET		(1 << 0)
#define	P2WI_CCR		0x04
#define	P2WI_INTE		0x08
#define	P2WI_INTS		0x0c
#define	 INT_TRANS_ERR_ID(x)	(((x) >> 8) & 0xf)
#define	 INT_LOAD_BSY		(1 << 2)
#define	 INT_TRANS_ERR		(1 << 1)
#define	 INT_TRANS_OVER		(1 << 0)
#define	 INT_MASK		(INT_LOAD_BSY|INT_TRANS_ERR|INT_TRANS_OVER)
#define	P2WI_DADDR0		0x10
#define	P2WI_DADDR1		0x14
#define	P2WI_DLEN		0x18
#define	 DLEN_READ		(1 << 4)
#define	P2WI_DATA0		0x1c
#define	P2WI_DATA1		0x20

#define	P2WI_MAXLEN		8
#define	P2WI_RESET_RETRY	100
#define	P2WI_I2C_TIMEOUT	(5 * hz)

static struct ofw_compat_data compat_data[] = {
	{ "allwinner,sun6i-a31-p2wi",		1 },
	{ NULL,					0 }
};

static struct resource_spec p2wi_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE },
	{ -1, 0 }
};

struct p2wi_softc {
	struct resource	*res[2];
	struct mtx	mtx;
	device_t	iicbus;
	void		*ih;
	int		busy;
	uint32_t	status;

	struct iic_msg	*msg;
};

#define	P2WI_LOCK(sc)			mtx_lock(&(sc)->mtx)
#define	P2WI_UNLOCK(sc)			mtx_unlock(&(sc)->mtx)
#define	P2WI_READ(sc, reg)		bus_read_4((sc)->res[0], (reg))
#define	P2WI_WRITE(sc, reg, val)	bus_write_4((sc)->res[0], (reg), (val))

static phandle_t
p2wi_get_node(device_t bus, device_t dev)
{
	return (ofw_bus_get_node(bus));
}

static int
p2wi_reset(device_t dev, u_char speed, u_char addr, u_char *oldaddr)
{
	struct p2wi_softc *sc;
	int retry;

	sc = device_get_softc(dev);

	P2WI_LOCK(sc);

	/* Write soft-reset bit and wait for it to self-clear. */
	P2WI_WRITE(sc, P2WI_CTRL, SOFT_RESET);
	for (retry = P2WI_RESET_RETRY; retry > 0; retry--)
		if ((P2WI_READ(sc, P2WI_CTRL) & SOFT_RESET) == 0)
			break;

	P2WI_UNLOCK(sc);

	if (retry == 0) {
		device_printf(dev, "soft reset timeout\n");
		return (ETIMEDOUT);
	}

	return (IIC_ENOADDR);
}

static uint32_t
p2wi_encode(const uint8_t *buf, u_int len, u_int off)
{
	uint32_t val;
	u_int n;

	val = 0;
	for (n = off; n < MIN(len, 4 + off); n++)
		val |= ((uint32_t)buf[n] << ((n - off) * NBBY));

	return val;
}

static void
p2wi_decode(const uint32_t val, uint8_t *buf, u_int len, u_int off)
{
	u_int n;

	for (n = off; n < MIN(len, 4 + off); n++)
		buf[n] = (val >> ((n - off) * NBBY)) & 0xff;
}

static int
p2wi_transfer(device_t dev, struct iic_msg *msgs, uint32_t nmsgs)
{
	struct p2wi_softc *sc;
	uint32_t daddr[2], data[2], dlen;
	int error;

	sc = device_get_softc(dev);

	/* Transfers must contain exactly two messages. The first is always
	 * a write, containing a list of data byte offsets. Data will either
	 * be read from or written to the corresponding data byte in the
	 * second message. We can transfer up to 8 bytes of data per request.
	 */
	if (nmsgs != 2 || (msgs[0].flags & IIC_M_RD) == IIC_M_RD ||
	    msgs[0].len != msgs[1].len || msgs[0].len == 0 ||
	    msgs[0].len > P2WI_MAXLEN)
		return (EINVAL);

	P2WI_LOCK(sc);
	while (sc->busy)
		mtx_sleep(sc, &sc->mtx, 0, "i2cbuswait", 0);
	sc->busy = 1;
	sc->status = 0;

	/* Program data access address registers */
	daddr[0] = p2wi_encode(msgs[0].buf, msgs[0].len, 0);
	daddr[1] = p2wi_encode(msgs[0].buf, msgs[0].len, 4);
	P2WI_WRITE(sc, P2WI_DADDR0, daddr[0]);
	P2WI_WRITE(sc, P2WI_DADDR1, daddr[1]);

	if ((msgs[1].flags & IIC_M_RD) == 0) {
		/* Write data */
		data[0] = p2wi_encode(msgs[1].buf, msgs[1].len, 0);
		data[1] = p2wi_encode(msgs[1].buf, msgs[1].len, 4);
		P2WI_WRITE(sc, P2WI_DATA0, data[0]);
		P2WI_WRITE(sc, P2WI_DATA1, data[1]);
	}

	/* Program data length register and transfer direction */
	dlen = msgs[0].len - 1;
	if ((msgs[1].flags & IIC_M_RD) == IIC_M_RD)
		dlen |= DLEN_READ;
	P2WI_WRITE(sc, P2WI_DLEN, dlen);

	/* Enable interrupts */
	P2WI_WRITE(sc, P2WI_INTE, INT_MASK);

	/* Start the transfer */
	P2WI_WRITE(sc, P2WI_CTRL, GLOBAL_INT_ENB | START_TRANS);

	/* Wait for transfer to complete */
	error = mtx_sleep(sc, &sc->mtx, 0, "i2ciowait", P2WI_I2C_TIMEOUT);
	if (error == 0 && (sc->status & INT_TRANS_OVER) == 0) {
		device_printf(dev, "transfer error, status 0x%08x\n",
		    sc->status);
		error = EIO;
	}

	if (error == 0 && (msgs[1].flags & IIC_M_RD) == IIC_M_RD) {
		/* Read data */
		data[0] = P2WI_READ(sc, P2WI_DATA0);
		data[1] = P2WI_READ(sc, P2WI_DATA1);
		p2wi_decode(data[0], msgs[1].buf, msgs[1].len, 0);
		p2wi_decode(data[1], msgs[1].buf, msgs[1].len, 4);
	}

	sc->msg = NULL;
	sc->busy = 0;
	wakeup(sc);
	P2WI_UNLOCK(sc);

	return (error);
}

static void
p2wi_intr(void *arg)
{
	struct p2wi_softc *sc;

	sc = arg;

	P2WI_LOCK(sc);
	sc->status |= P2WI_READ(sc, P2WI_INTS);
	if ((sc->status & INT_MASK) != 0)
		wakeup(sc);
	P2WI_UNLOCK(sc);
}

static int
p2wi_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Allwinner P2WI");
	return (BUS_PROBE_DEFAULT);
}

static int
p2wi_attach(device_t dev)
{
	struct p2wi_softc *sc;
	int error;

	sc = device_get_softc(dev);
	mtx_init(&sc->mtx, device_get_nameunit(dev), "p2wi", MTX_DEF);

	if (bus_alloc_resources(dev, p2wi_spec, sc->res) != 0) {
		device_printf(dev, "cannot allocate resources for device\n");
		error = ENXIO;
		goto fail;
	}

	error = bus_setup_intr(dev, sc->res[1], INTR_TYPE_MISC | INTR_MPSAFE,
	    NULL, p2wi_intr, sc, &sc->ih);
	if (error != 0) {
		device_printf(dev, "cannot setup interrupt handler\n");
		goto fail;
	}

	sc->iicbus = device_add_child(dev, "iicbus", -1);
	if (sc->iicbus == NULL) {
		device_printf(dev, "cannot add iicbus child device\n");
		error = ENXIO;
		goto fail;
	}

	bus_generic_attach(dev);

	return (0);

fail:
	if (sc->ih != NULL)
		bus_teardown_intr(dev, sc->res[1], sc->ih);
	bus_release_resources(dev, p2wi_spec, sc->res);
	mtx_destroy(&sc->mtx);
	return (error);
}

static device_method_t p2wi_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		p2wi_probe),
	DEVMETHOD(device_attach,	p2wi_attach),

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
	DEVMETHOD(ofw_bus_get_node,	p2wi_get_node),

	/* iicbus interface */
	DEVMETHOD(iicbus_callback,	iicbus_null_callback),
	DEVMETHOD(iicbus_reset,		p2wi_reset),
	DEVMETHOD(iicbus_transfer,	p2wi_transfer),

	DEVMETHOD_END
};

static driver_t p2wi_driver = {
	"p2wi",
	p2wi_methods,
	sizeof(struct p2wi_softc),
};

static devclass_t p2wi_devclass;

DRIVER_MODULE(p2wi, simplebus, p2wi_driver, p2wi_devclass, 0, 0);
MODULE_VERSION(p2wi, 1);
