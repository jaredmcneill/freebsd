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
 * Allwinner Digital Audio Interface
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

#include <dev/sound/pcm/sound.h>
#include <dev/sound/chip.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "sunxi_dma_if.h"
#include "mixer_if.h"

#define	DA_CTL			0x00
#define	DA_FMT0			0x04
#define	DA_FMT1			0x08
#define	DA_ISTA			0x0c
#define	DA_RXFIFO		0x10
#define	DA_FCTL			0x14
#define	DA_FSTA			0x18
#define	DA_INT			0x1c
#define	DA_TXFIFO		0x20
#define	DA_CLKD			0x24
#define	DA_TXCNT		0x28
#define	DA_RXCNT		0x2c
#define	DA_CHCFG		0x30
#define	DA_TXCHSEL(n)		(0x34 + (n) * 4)
#define	DA_TXCHMAP(n)		(0x44 + (n) * 4)
#define	DA_RXCHSEL		0x54
#define	DA_RXCHMAP		0x58

#define	DRQTYPE_SDRAM		1

#define	DMA_WIDTH		32
#define	DMA_BURST_LEN		8

#define	DMABUF_MIN		4096
#define	DMABUF_DEFAULT		65536
#define	DMABUF_MAX		131072

#define	DA_SAMPLERATE		48000

#define	DA_CLKFREQ		24576000

static uint32_t aw_daudio_fmt[] = {
	SND_FORMAT(AFMT_S16_LE, 2, 0),
	0
};

static struct pcmchan_caps aw_daudio_caps = {
	DA_SAMPLERATE, DA_SAMPLERATE, aw_daudio_fmt, 0
};

struct aw_daudio_info;

struct aw_daudio_chinfo {
	struct snd_dbuf		*buffer;
	struct pcm_channel	*channel;	
	struct aw_daudio_info	*parent;
	bus_dmamap_t		dmamap;
	void			*dmaaddr;
	bus_addr_t		physaddr;
	bus_addr_t		fifo;
	device_t		dmac;
	void			*dmachan;
	unsigned int		drqtype;

	int			dir;
	int			run;
	uint32_t		pos;
	uint32_t		blocksize;
};

struct aw_daudio_info {
	device_t		dev;
	struct mtx		*lock;
	bus_dma_tag_t		dmat;
	unsigned		dmasize;

	struct aw_daudio_chinfo	play;
};

/*
 * Mixer interface
 */

static int
aw_daudio_mixer_init(struct snd_mixer *m)
{
	mix_setdevs(m, SOUND_MASK_PCM);

	return (0);
}

static int
aw_daudio_mixer_set(struct snd_mixer *m, unsigned dev, unsigned left,
    unsigned right)
{
	return (-1);
}

static kobj_method_t aw_daudio_mixer_methods[] = {
	KOBJMETHOD(mixer_init,		aw_daudio_mixer_init),
	KOBJMETHOD(mixer_set,		aw_daudio_mixer_set),
	KOBJMETHOD_END
};
MIXER_DECLARE(aw_daudio_mixer);


/*
 * Channel interface
 */

static void
aw_daudio_dmamap_cb(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
	struct aw_daudio_chinfo *ch = arg;

	if (error != 0)
		return;

	ch->physaddr = segs[0].ds_addr;
}

static void
aw_daudio_transfer(struct aw_daudio_chinfo *ch)
{
	bus_addr_t src, dst;
	int error;

	if (ch->dir == PCMDIR_PLAY) {
		src = ch->physaddr + ch->pos;
		dst = ch->fifo;
	} else {
		src = ch->fifo;
		dst = ch->physaddr + ch->pos;
	}

	error = SUNXI_DMA_TRANSFER(ch->dmac, ch->dmachan, src, dst,
	    ch->blocksize);
	if (error) {
		ch->run = 0;
		device_printf(ch->parent->dev, "DMA transfer failed: %d\n",
		    error);
	}
}

static void
aw_daudio_dmaconfig(struct aw_daudio_chinfo *ch)
{
	struct sunxi_dma_config conf;

	memset(&conf, 0, sizeof(conf));
	conf.src_width = conf.dst_width = DMA_WIDTH;
	conf.src_burst_len = conf.dst_burst_len = DMA_BURST_LEN;
	if (ch->dir == PCMDIR_REC) {
		conf.src_drqtype = ch->drqtype;
		conf.src_noincr = true;
		conf.dst_drqtype = DRQTYPE_SDRAM;
	} else {
		conf.src_drqtype = DRQTYPE_SDRAM;
		conf.dst_drqtype = ch->drqtype;
		conf.dst_noincr = true;
	}

	SUNXI_DMA_SET_CONFIG(ch->dmac, ch->dmachan, &conf);
}

static void
aw_daudio_dmaintr(void *priv)
{
	struct aw_daudio_chinfo *ch = priv;
	unsigned bufsize;

	bufsize = sndbuf_getsize(ch->buffer);

	ch->pos += ch->blocksize;
	if (ch->pos >= bufsize)
		ch->pos -= bufsize;

	if (ch->run) {
		chn_intr(ch->channel);
		aw_daudio_transfer(ch);
	}
}

static void
aw_daudio_start(struct aw_daudio_chinfo *ch)
{
	ch->pos = 0;

	/* Configure DMA channel */
	aw_daudio_dmaconfig(ch);

	/* Start DMA transfer */
	aw_daudio_transfer(ch);
}

static void
aw_daudio_stop(struct aw_daudio_chinfo *ch)
{
	/* Disable DMA channel */
	SUNXI_DMA_HALT(ch->dmac, ch->dmachan);
}

static void *
aw_daudio_chan_init(kobj_t obj, void *devinfo, struct snd_dbuf *b,
    struct pcm_channel *c, int dir)
{
	struct aw_daudio_info *sc = devinfo;
	struct aw_daudio_chinfo *ch = &sc->play;
	pcell_t prop[4];
	int error, len;

	ch->parent = sc;
	ch->channel = c;
	ch->buffer = b;
	ch->dir = dir;
	ch->fifo = rman_get_start(sc->res[0]) +
	    (dir == PCMDIR_REC ? DA_RXFIFO : DA_TXFIFO);

	len = OF_getencprop(ofw_bus_get_node(sc->dev), "dmas", prop, sizeof(prop));
	if (len > 0 && (len / sizeof(prop[0])) == 4) {
		ch->dmac = OF_device_from_xref(prop[0]);
		ch->drqtype = prop[1];
	}
	if (ch->dmac == NULL) {
		device_printf(sc->dev, "cannot find DMA controller\n");
		return (NULL);
	}
	ch->dmachan = SUNXI_DMA_ALLOC(ch->dmac, true, aw_daudio_dmaintr, ch);
	if (ch->dmachan == NULL) {
		device_printf(sc->dev, "cannot allocate DMA channel\n");
		return (NULL);
	}

	error = bus_dmamem_alloc(sc->dmat, &ch->dmaaddr,
	    BUS_DMA_NOWAIT | BUS_DMA_COHERENT | BUS_DMA_ZERO, &ch->dmamap);
	if (error != 0) {
		device_printf(sc->dev, "cannot allocate channel buffer\n");
		return (NULL);
	}
	error = bus_dmamap_load(sc->dmat, ch->dmamap, ch->dmaaddr,
	    sc->dmasize, aw_daudio_dmamap_cb, ch, BUS_DMA_NOWAIT);
	if (error != 0) {
		device_printf(sc->dev, "cannot load DMA map\n");
		return (NULL);
	}

	if (sndbuf_setup(ch->buffer, ch->dmaaddr, sc->dmasize) != 0) {
		device_printf(sc->dev, "cannot setup sndbuf\n");
		return (NULL);
	}

	return (ch);
}

static int
aw_daudio_chan_free(kobj_t obj, void *data)
{
	struct aw_daudio_chinfo *ch = data;
	struct aw_daudio_info *sc = ch->parent;

	SUNXI_DMA_FREE(ch->dmac, ch->dmachan);
	bus_dmamap_unload(sc->dmat, ch->dmamap);
	bus_dmamem_free(sc->dmat, ch->dmaaddr, ch->dmamap);

	return (0);
}

static int
aw_daudio_chan_setformat(kobj_t obj, void *data, uint32_t format)
{
	return (0);
}

static uint32_t
aw_daudio_chan_setspeed(kobj_t obj, void *data, uint32_t speed)
{
	return (DA_SAMPLERATE);
}

static uint32_t
aw_daudio_chan_setblocksize(kobj_t obj, void *data, uint32_t blocksize)
{
	struct aw_daudio_chinfo *ch = data;

	ch->blocksize = blocksize & ~3;

	return (ch->blocksize);
}

static int
aw_daudio_chan_trigger(kobj_t obj, void *data, int go)
{
	struct aw_daudio_chinfo *ch = data;
	struct aw_daudio_info *sc = ch->parent;

	if (!PCMTRIG_COMMON(go))
		return (0);

	snd_mtxlock(sc->lock);
	switch (go) {
	case PCMTRIG_START:
		ch->run = 1;
		aw_daudio_start(ch);
		break;
	case PCMTRIG_STOP:
	case PCMTRIG_ABORT:
		ch->run = 0;
		aw_daudio_stop(ch);
		break;
	default:
		break;
	}
	snd_mtxunlock(sc->lock);

	return (0);
}

static uint32_t
aw_daudio_chan_getptr(kobj_t obj, void *data)
{
	struct aw_daudio_chinfo *ch = data;

	return (ch->pos);
}

static struct pcmchan_caps *
aw_daudio_chan_getcaps(kobj_t obj, void *data)
{
	return (&aw_daudio_caps);
}

static kobj_method_t aw_daudio_chan_methods[] = {
	KOBJMETHOD(channel_init,		aw_daudio_chan_init),
	KOBJMETHOD(channel_free,		aw_daudio_chan_free),
	KOBJMETHOD(channel_setformat,		aw_daudio_chan_setformat),
	KOBJMETHOD(channel_setspeed,		aw_daudio_chan_setspeed),
	KOBJMETHOD(channel_setblocksize,	aw_daudio_chan_setblocksize),
	KOBJMETHOD(channel_trigger,		aw_daudio_chan_trigger),
	KOBJMETHOD(channel_getptr,		aw_daudio_chan_getptr),
	KOBJMETHOD(channel_getcaps,		aw_daudio_chan_getcaps),
	KOBJMETHOD_END
};
CHANNEL_DECLARE(aw_daudio_chan);


/*
 * Device interface
 */

static int
aw_daudio_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "allwinner,sun8i-a83t-i2s"))
		return (ENXIO);

	device_set_desc(dev, "Allwinner Digital Audio");
	return (BUS_PROBE_DEFAULT);
}

static int
aw_daudio_attach(device_t dev)
{
	struct aw_daudio_info *sc;
	char status[SND_STATUSLEN];
	clk_t clk_apb, clk_daudio;
	int error;

	clk_apb = NULL;
	clk_daudio = NULL;

	sc = malloc(sizeof(*sc), M_DEVBUF, M_WAITOK | M_ZERO);
	sc->dev = dev;
	sc->lock = snd_mtxcreate(device_get_nameunit(dev), "aw_daudio softc");

	sc->dmasize = pcm_getbuffersize(dev, DMABUF_MIN,
	    DMABUF_DEFAULT, DMABUF_MAX);
	error = bus_dma_tag_create(
	    bus_get_dma_tag(dev),
	    4, sc->dmasize,		/* alignment, boundary */
	    BUS_SPACE_MAXADDR_32BIT,	/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    sc->dmasize, 1,		/* maxsize, nsegs */
	    sc->dmasize, 0,		/* maxsegsize, flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->dmat);
	if (error != 0) {
		device_printf(dev, "cannot create DMA tag\n");
		goto fail;
	}

	/* Get clocks */
	error = clk_get_by_ofw_name(dev, "apb", &clk_apb);
	if (error != 0) {
		device_printf(dev, "cannot find apb clock\n");
		goto fail;
	}
	error = clk_get_by_ofw_name(dev, "i2s", &clk_daudio);
	if (error != 0) {
		device_printf(dev, "cannot find i2s clock\n");
		goto fail;
	}

	/* Gating APB clock for daudio */
	error = clk_enable(clk_apb);
	if (error != 0) {
		device_printf(dev, "cannot enable apb clock\n");
		goto fail;
	}
	/* Set module clock frequency */
	error = clk_set_freq(clk_daudio, DA_CLKFREQ, CLK_SET_ROUND_DOWN);
	if (error != 0) {
		device_printf(dev, "cannot set i2s clock frequency\n");
		goto fail;
	}
	/* Enable module clock */
	error = clk_enable(clk_daudio);
	if (error != 0) {
		device_printf(dev, "cannot enable i2s clock\n");
		goto fail;
	}

	/* Enable module */
	

	if (mixer_init(dev, &aw_daudio_mixer_class, sc)) {
		device_printf(dev, "mixer_init failed\n");
		goto fail;
	}

	pcm_setflags(dev, pcm_getflags(dev) | SD_F_MPSAFE);
	pcm_setflags(dev, pcm_getflags(dev) | SD_F_SOFTPCMVOL);

	if (pcm_register(dev, sc, 1, 0)) {
		device_printf(dev, "pcm_register failed\n");
		goto fail;
	}

	pcm_addchan(dev, PCMDIR_PLAY, &aw_daudio_chan_class, sc);

	snprintf(status, SND_STATUSLEN, "at %s", ofw_bus_get_name(dev));
	pcm_setstatus(dev, status);

	return (0);

fail:
	if (clk_daudio != NULL)
		clk_release(clk_daudio);
	if (clk_apb != NULL)
		clk_release(clk_apb);
	snd_mtxfree(sc->lock);
	free(sc, M_DEVBUF);

	return (error);
}

static device_method_t aw_daudio_pcm_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		aw_daudio_probe),
	DEVMETHOD(device_attach,	aw_daudio_attach),

	DEVMETHOD_END
};

static driver_t aw_daudio_pcm_driver = {
	"pcm",
	aw_daudio_pcm_methods,
	PCM_SOFTC_SIZE,
};

DRIVER_MODULE(aw_daudio, simplebus, aw_daudio_pcm_driver, pcm_devclass, 0, 0);
MODULE_DEPEND(aw_daudio, sound, SOUND_MINVER, SOUND_PREFVER, SOUND_MAXVER);
MODULE_VERSION(aw_daudio, 1);
