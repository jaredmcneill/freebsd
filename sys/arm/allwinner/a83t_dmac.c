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
 */

/*
 * Allwinner A83T DMA controller
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/condvar.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/endian.h>

#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <arm/allwinner/a83t_dmac.h>
#include <dev/extres/clk/clk.h>
#include <dev/extres/hwreset/hwreset.h>

#include "sunxi_dma_if.h"

#define	DMA_CHANNELS	8
#define	DMA_ALIGN	4
#define	DMA_WAIT_CYC	8

struct a83tdmac_softc;

struct a83tdmac_channel {
	struct a83tdmac_softc *	ch_sc;
	uint8_t			ch_index;
	void			(*ch_callback)(void *);
	void *			ch_callbackarg;

	bus_dma_segment_t	ch_dmasegs[1];
	bus_dmamap_t		ch_dmamap;
	void			*ch_dmadesc;
	bus_size_t		ch_dmadesclen;
	bus_addr_t		ch_physaddr;
};

struct a83tdmac_softc {
	struct resource *	sc_res[2];
	struct mtx		sc_mtx;
	void *			sc_ih;
	bus_dma_tag_t		sc_dmat;

	struct a83tdmac_channel	sc_dma_channels[DMA_CHANNELS];
};

static struct resource_spec a83tdmac_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE },
	{ -1, 0 }
};

#define	DMA_READ(sc, reg)	bus_read_4((sc)->sc_res[0], (reg))
#define	DMA_WRITE(sc, reg, val)	bus_write_4((sc)->sc_res[0], (reg), (val))

static void a83tdmac_intr(void *);
static void a83tdmac_dmamap_cb(void *, bus_dma_segment_t *, int, int);

static int
a83tdmac_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "allwinner,sun8i-a83t-dma"))
		return (ENXIO);

	device_set_desc(dev, "Allwinner DMA controller");
	return (BUS_PROBE_DEFAULT);
}

static int
a83tdmac_attach(device_t dev)
{
	struct a83tdmac_softc *sc;
	struct a83tdmac_channel *ch;
	unsigned int index, dmasize;
	hwreset_t rst;
	clk_t clk;
	int error;

	sc = device_get_softc(dev);

	if (bus_alloc_resources(dev, a83tdmac_spec, sc->sc_res)) {
		device_printf(dev, "cannot allocate resources for device\n");
		return (ENXIO);
	}

	mtx_init(&sc->sc_mtx, "a83t dmac", NULL, MTX_DEF);

	/* Activate DMA controller clock */
	error = hwreset_get_by_ofw_idx(dev, 0, &rst);
	if (error != 0) {
		device_printf(dev, "cannot get hwreset\n");
		return (error);
	}
	error = hwreset_deassert(rst);
	if (error != 0) {
		device_printf(dev, "cannot de-assert hwreset\n");
		return (error);
	}
	error = clk_get_by_ofw_index(dev, 0, &clk);
	if (error != 0) {
		device_printf(dev, "cannot get clock\n");
		return (error);
	}
	error = clk_enable(clk);
	if (error != 0) {
		device_printf(dev, "cannot enable clock\n");
		return (error);
	}

	/* Disable all interrupts and clear pending status */
	DMA_WRITE(sc, DMA_IRQ_EN_REG, 0);
	DMA_WRITE(sc, DMA_IRQ_PEND_REG, DMA_READ(sc, DMA_IRQ_PEND_REG));

	/* Create bus DMA tag for descriptors */
	dmasize = sizeof(struct a83t_dma_desc);
	error = bus_dma_tag_create(
	    bus_get_dma_tag(dev),
	    DMA_ALIGN, 0,		/* alignment, boundary */
	    BUS_SPACE_MAXADDR_32BIT,	/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    dmasize, 1,			/* maxsize, nsegs */
	    dmasize, 0,			/* maxsegsize, flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->sc_dmat);
	if (error != 0) {
		device_printf(dev, "cannot create DMA tag\n");
		return (error);
	}

	/* Initialize channels */
	for (index = 0; index < DMA_CHANNELS; index++) {
		ch = &sc->sc_dma_channels[index];
		ch->ch_sc = sc;
		ch->ch_index = index;
		ch->ch_callback = NULL;
		ch->ch_callbackarg = NULL;
		ch->ch_dmadesclen = dmasize;

		error = bus_dmamem_alloc(sc->sc_dmat, &ch->ch_dmadesc,
		    BUS_DMA_WAITOK | BUS_DMA_ZERO | BUS_DMA_COHERENT,
		    &ch->ch_dmamap);
		if (error != 0) {
			device_printf(dev, "cannot allocate DMA descriptor\n");
			return (error);
		}
		error = bus_dmamap_load(sc->sc_dmat, ch->ch_dmamap,
		    ch->ch_dmadesc, ch->ch_dmadesclen, a83tdmac_dmamap_cb, ch,
		    BUS_DMA_WAITOK);
		if (error != 0) {
			device_printf(dev, "cannot load DMA map\n");
			return (error);
		}

		DMA_WRITE(sc, DMA_EN_REG(index), 0);
	}

	error = bus_setup_intr(dev, sc->sc_res[1], INTR_MPSAFE | INTR_TYPE_MISC,
	    NULL, a83tdmac_intr, sc, &sc->sc_ih);
	if (error != 0) {
		device_printf(dev, "could not setup interrupt handler\n");
		bus_release_resources(dev, a83tdmac_spec, sc->sc_res);
		mtx_destroy(&sc->sc_mtx);
		return (ENXIO);
	}

	return (0);
}

static void
a83tdmac_intr(void *priv)
{
	struct a83tdmac_softc *sc;
	uint32_t pend, bit, mask;
	uint8_t index;

	sc = priv;

	pend = DMA_READ(sc, DMA_IRQ_PEND_REG);
	DMA_WRITE(sc, DMA_IRQ_PEND_REG, pend);

	while ((bit = ffs(pend & DMA_PKG_IRQ_MASK)) != 0) {
		mask = (1U << (bit - 1));
		pend &= ~mask;
		index = (bit - 1) / 4;
		if (sc->sc_dma_channels[index].ch_callback == NULL)
			continue;
		sc->sc_dma_channels[index].ch_callback(
		    sc->sc_dma_channels[index].ch_callbackarg);
	}
}

static void
a83tdmac_dmamap_cb(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
	struct a83tdmac_channel *ch;

	ch = arg;

	if (error != 0)
		return;

	ch->ch_physaddr = segs[0].ds_addr;
}

static int
a83tdmac_set_config(device_t dev, void *priv,
    const struct sunxi_dma_config *cfg)
{
	struct a83tdmac_channel *ch;
	struct a83tdmac_softc *sc;
	struct a83t_dma_desc *desc;
	uint32_t val;
	unsigned int dst_dw, dst_bl, dst_am;
	unsigned int src_dw, src_bl, src_am;

	ch = priv;
	sc = ch->ch_sc;
	desc = ch->ch_dmadesc;

	switch (cfg->dst_width) {
	case 8:
		dst_dw = DMA_DEST_DATA_WIDTH_8BIT;
		break;
	case 16:
		dst_dw = DMA_DEST_DATA_WIDTH_16BIT;
		break;
	case 32:
		dst_dw = DMA_DEST_DATA_WIDTH_32BIT;
		break;
	default:
		return (EINVAL);
	}
	switch (cfg->dst_burst_len) {
	case 1:
		dst_bl = DMA_DEST_BST_LEN_1;
		break;
	case 8:
		dst_bl = DMA_DEST_BST_LEN_8;
		break;
	default:
		return (EINVAL);
	}
	dst_am = cfg->dst_noincr ? DMA_DEST_ADDR_MODE_IO :
	    DMA_DEST_ADDR_MODE_LINEAR;
	switch (cfg->src_width) {
	case 8:
		src_dw = DMA_SRC_DATA_WIDTH_8BIT;
		break;
	case 16:
		src_dw = DMA_SRC_DATA_WIDTH_16BIT;
		break;
	case 32:
		src_dw = DMA_SRC_DATA_WIDTH_32BIT;
		break;
	default:
		return (EINVAL);
	}
	switch (cfg->src_burst_len) {
	case 1:
		src_bl = DMA_SRC_BST_LEN_1;
		break;
	case 8:
		src_bl = DMA_SRC_BST_LEN_8;
		break;
	default:
		return (EINVAL);
	}
	src_am = cfg->src_noincr ? DMA_SRC_ADDR_MODE_IO :
	    DMA_SRC_ADDR_MODE_LINEAR;

	val = (dst_dw << DMA_DEST_DATA_WIDTH_SHIFT) |
	      (dst_bl << DMA_DEST_BST_LEN_SHIFT) |
	      (dst_am << DMA_DEST_ADDR_MODE_SHIFT) |
	      (cfg->dst_drqtype << DMA_DEST_DRQ_TYPE_SHIFT) |
	      (src_dw << DMA_SRC_DATA_WIDTH_SHIFT) |
	      (src_bl << DMA_SRC_BST_LEN_SHIFT) |
	      (src_am << DMA_SRC_ADDR_MODE_SHIFT) |
	      (cfg->src_drqtype << DMA_SRC_DRQ_TYPE_SHIFT);

	desc->config = htole32(val);
	desc->param = htole32(DMA_WAIT_CYC);

	bus_dmamap_sync(sc->sc_dmat, ch->ch_dmamap, BUS_DMASYNC_PREWRITE);

	return (0);
}

static void *
a83tdmac_alloc(device_t dev, bool dedicated, void (*cb)(void *), void *cbarg)
{
	struct a83tdmac_softc *sc = device_get_softc(dev);
	struct a83tdmac_channel *ch_list;
	struct a83tdmac_channel *ch;
	uint32_t irqen;
	uint8_t ch_count, index;

	sc = device_get_softc(dev);
	ch = NULL;
	ch_list = sc->sc_dma_channels;
	ch_count = DMA_CHANNELS;

	mtx_lock(&sc->sc_mtx);
	for (index = 0; index < ch_count; index++) {
		if (ch_list[index].ch_callback == NULL) {
			ch = &ch_list[index];
			ch->ch_callback = cb;
			ch->ch_callbackarg = cbarg;

			irqen = DMA_READ(sc, DMA_IRQ_EN_REG);
			irqen |= DMA_PKG_IRQ_EN(index);
			DMA_WRITE(sc, DMA_IRQ_EN_REG, irqen);

			break;
		}
	}
	mtx_unlock(&sc->sc_mtx);

	return (ch);
}

static void
a83tdmac_free(device_t dev, void *priv)
{
	struct a83tdmac_channel *ch = priv;
	struct a83tdmac_softc *sc = ch->ch_sc;
	uint32_t irqen;

	mtx_lock(&sc->sc_mtx);

	irqen = DMA_READ(sc, DMA_IRQ_EN_REG);
	irqen &= ~DMA_PKG_IRQ_EN(ch->ch_index);
	DMA_WRITE(sc, DMA_IRQ_EN_REG, irqen);

	ch->ch_callback = NULL;
	ch->ch_callbackarg = NULL;

	mtx_unlock(&sc->sc_mtx);
}

static int
a83tdmac_transfer(device_t dev, void *priv, bus_addr_t src, bus_addr_t dst,
    size_t nbytes)
{
	struct a83tdmac_channel *ch;
	struct a83tdmac_softc *sc;
	struct a83t_dma_desc *desc;

	ch = priv;
	sc = ch->ch_sc;
	desc = ch->ch_dmadesc;

	desc->src = htole32(src);
	desc->dst = htole32(dst);
	desc->bcnt = htole32(nbytes);
	desc->link = htole32(DMA_LINK_END);

	bus_dmamap_sync(sc->sc_dmat, ch->ch_dmamap, BUS_DMASYNC_PREWRITE);

	DMA_WRITE(sc, DMA_DESC_ADDR_REG(ch->ch_index), ch->ch_physaddr);
	DMA_WRITE(sc, DMA_EN_REG(ch->ch_index), DMA_EN);

	return (0);
}

static void
a83tdmac_halt(device_t dev, void *priv)
{
	struct a83tdmac_channel *ch;
	struct a83tdmac_softc *sc;

	ch = priv;
	sc = ch->ch_sc;

	DMA_WRITE(sc, DMA_EN_REG(ch->ch_index), 0);
}

static device_method_t a83tdmac_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		a83tdmac_probe),
	DEVMETHOD(device_attach,	a83tdmac_attach),

	/* sunxi DMA interface */
	DEVMETHOD(sunxi_dma_alloc,	a83tdmac_alloc),
	DEVMETHOD(sunxi_dma_free,	a83tdmac_free),
	DEVMETHOD(sunxi_dma_set_config,	a83tdmac_set_config),
	DEVMETHOD(sunxi_dma_transfer,	a83tdmac_transfer),
	DEVMETHOD(sunxi_dma_halt,	a83tdmac_halt),

	DEVMETHOD_END
};

static driver_t a83tdmac_driver = {
	"a83tdmac",
	a83tdmac_methods,
	sizeof(struct a83tdmac_softc)
};

static devclass_t a83tdmac_devclass;

DRIVER_MODULE(a83tdmac, simplebus, a83tdmac_driver, a83tdmac_devclass, 0, 0);
