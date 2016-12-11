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
 * Ingenic JZ4780 LCD Controller
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
#include <sys/fbio.h>
#include <vm/vm.h>
#include <vm/vm_extern.h>
#include <vm/vm_kern.h>
#include <vm/pmap.h>

#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/videomode/videomode.h>
#include <dev/videomode/edidvar.h>

#include <dev/extres/clk/clk.h>

#include <mips/ingenic/jz4780_lcd.h>

#include "fb_if.h"
#include "hdmi_if.h"

#define	FB_DEFAULT_W	800
#define	FB_DEFAULT_H	600
#define	FB_DEFAULT_REF	60
#define	FB_BPP		32
#define	FB_ALIGN	(16 * 4)
#define	FB_MAX_BW	(1920 * 1080 * 60)

#define	PCFG_MAGIC	0xc7ff2100

#define	DOT_CLOCK_TO_HZ(c)	((c) * 1000)

#ifndef VM_MEMATTR_WRITE_COMBINING
#define	VM_MEMATTR_WRITE_COMBINING VM_MEMATTR_UNCACHEABLE
#endif

struct jzlcd_softc {
	device_t		dev;
	device_t		fbdev;
	struct resource		*res;

	/* Clocks */
	clk_t			clk;
	clk_t			clk_pix;

	/* Framebuffer */
	struct fb_info		info;
	size_t			fbsize;
	bus_addr_t		paddr;
	vm_offset_t		vaddr;

	/* HDMI */
	eventhandler_tag	hdmi_evh;

	/* Frame descriptor DMA */
	bus_dma_tag_t		fdesc_tag;
	bus_dmamap_t		fdesc_map;
	bus_addr_t		fdesc_paddr;
	struct lcd_frame_descriptor	*fdesc;

	/* FB DMA */
	bus_dma_tag_t		fb_tag;
	bus_dmamap_t		fb_map;
	bus_addr_t		fb_paddr;
	void			*fb;
};

static struct resource_spec jzlcd_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ -1, 0 }
};

#define	LCD_READ(sc, reg)		bus_read_4((sc)->res, (reg))
#define	LCD_WRITE(sc, reg, val)		bus_write_4((sc)->res, (reg), (val))

static int
jzlcd_allocfb(struct jzlcd_softc *sc)
{
	sc->vaddr = kmem_alloc_contig(kernel_arena, sc->fbsize,
	    M_NOWAIT | M_ZERO, 0, ~0, FB_ALIGN, 0, VM_MEMATTR_WRITE_COMBINING);
	if (sc->vaddr == 0) {
		device_printf(sc->dev, "failed to allocate FB memory\n");
		return (ENOMEM);
	}
	sc->paddr = pmap_kextract(sc->vaddr);

	return (0);
}

static void
jzlcd_freefb(struct jzlcd_softc *sc)
{
	kmem_free(kernel_arena, sc->vaddr, sc->fbsize);
}

static int
jzlcd_set_videomode(struct jzlcd_softc *sc, const struct videomode *mode)
{
	u_int hbp, hfp, hsw, vbp, vfp, vsw;
	u_int hds, hde, ht, vds, vde, vt;
	uint32_t ctrl;
	int error;

	hbp = mode->htotal - mode->hsync_end;
	hfp = mode->hsync_start - mode->hdisplay;
	hsw = mode->hsync_end - mode->hsync_start;
	vbp = mode->vtotal - mode->vsync_end;
	vfp = mode->vsync_start - mode->vdisplay;
	vsw = mode->vsync_end - mode->vsync_start;

	hds = hsw + hbp;
	hde = hds + mode->hdisplay;
	ht = hde + hfp;

	vds = vsw + vbp;
	vde = vds + mode->vdisplay;
	vt = vde + vfp;

	/* Setup frame descriptor */
	sc->fdesc->next = sc->fdesc_paddr;
	sc->fdesc->physaddr = sc->paddr;
	sc->fdesc->id = 1;
	sc->fdesc->cmd = mode->hdisplay * mode->vdisplay * FB_BPP;
	sc->fdesc->offs = 0;
	sc->fdesc->pw = 0;
	sc->fdesc->cnum_pos = LCDPOS_BPP01_18_24;
	sc->fdesc->dessize = LCDDESSIZE_ALPHA |
	    ((mode->vdisplay - 1) << LCDDESSIZE_HEIGHT_SHIFT) |
	    ((mode->hdisplay - 1) << LCDDESSIZE_WIDTH_SHIFT);
	bus_dmamap_sync(sc->fdesc_tag, sc->fdesc_map, BUS_DMASYNC_PREWRITE);

	/* Setup timings */
	LCD_WRITE(sc, LCDVAT,
	    (ht << LCDVAT_HT_SHIFT) | (vt << LCDVAT_VT_SHIFT));
	LCD_WRITE(sc, LCDDAH,
	    (hds << LCDDAH_HDS_SHIFT) | (hde << LCDDAH_HDE_SHIFT));
	LCD_WRITE(sc, LCDDAV,
	    (vds << LCDDAV_VDS_SHIFT) | (vde << LCDDAV_VDE_SHIFT));
	LCD_WRITE(sc, LCDHSYNC, hsw);
	LCD_WRITE(sc, LCDVSYNC, vsw);

	/* Set configuration */
	LCD_WRITE(sc, LCDCFG, LCDCFG_NEWDES | LCDCFG_RECOVER | LCDCFG_24 |
	    LCDCFG_PSM | LCDCFG_CLSM | LCDCFG_SPLM | LCDCFG_REVM | LCDCFG_PCP);
	LCD_WRITE(sc, LCDCTRL, LCDCTRL_BST_64 | LCDCTRL_OFUM);
	LCD_WRITE(sc, LCDPCFG, PCFG_MAGIC);
	LCD_WRITE(sc, LCDRGBC, LCDRGBC_RGBFMT);

	/* Enable frame descriptor */
	LCD_WRITE(sc, LCDDA0, sc->fdesc_paddr);

	/* Set display clock */
	error = clk_set_freq(sc->clk_pix, DOT_CLOCK_TO_HZ(mode->dot_clock), 0);
	if (error != 0) {
		device_printf(sc->dev, "failed to set pixel clock to %u Hz\n",
		    DOT_CLOCK_TO_HZ(mode->dot_clock));
		return (error);
	}

	/* Start the controller! */
	LCD_WRITE(sc, LCDSTATE, 0);
	LCD_WRITE(sc, LCDOSDS, 0);
	ctrl = LCD_READ(sc, LCDCTRL);
	ctrl |= LCDCTRL_ENA;
	ctrl &= ~LCDCTRL_DIS;
	LCD_WRITE(sc, LCDCTRL, ctrl);

	/* XXX DEBUG */
	DELAY(1000000);
	device_printf(sc->dev, "descriptor at %08x\n", sc->fdesc_paddr);
	device_printf(sc->dev, " LCDDA0:      %08x\n", LCD_READ(sc, LCDDA0));
	device_printf(sc->dev, " LCDSA0:      %08x\n", LCD_READ(sc, LCDSA0));
	device_printf(sc->dev, " LCDFID0:     %08x\n", LCD_READ(sc, LCDFID0));
	device_printf(sc->dev, " LCDCMD0:     %08x\n", LCD_READ(sc, LCDCMD0));
	device_printf(sc->dev, " LCDOFFS0:    %08x\n", LCD_READ(sc, LCDOFFS0));
	device_printf(sc->dev, " LCDPW0:      %08x\n", LCD_READ(sc, LCDPW0));
	device_printf(sc->dev, " LCDCPOS0:    %08x\n", LCD_READ(sc, LCDPOS0));
	device_printf(sc->dev, " LCDDESSIZE0: %08x\n", LCD_READ(sc, LCDDESSIZE0));
	device_printf(sc->dev, " -\n");
	device_printf(sc->dev, " LCDVAT:      %08x\n", LCD_READ(sc, LCDVAT));
	device_printf(sc->dev, " LCDDAH:      %08x\n", LCD_READ(sc, LCDDAH));
	device_printf(sc->dev, " LCDDAV:      %08x\n", LCD_READ(sc, LCDDAV));
	device_printf(sc->dev, " LCDHSYNC:    %08x\n", LCD_READ(sc, LCDHSYNC));
	device_printf(sc->dev, " LCDVSYNC:    %08x\n", LCD_READ(sc, LCDVSYNC));
	device_printf(sc->dev, " -\n");
	device_printf(sc->dev, " LCDCFG:      %08x\n", LCD_READ(sc, LCDCFG));
	device_printf(sc->dev, " LCDCTRL:     %08x\n", LCD_READ(sc, LCDCTRL));
	device_printf(sc->dev, " LCDPCFG:     %08x\n", LCD_READ(sc, LCDPCFG));
	device_printf(sc->dev, " LCDRGBC:     %08x\n", LCD_READ(sc, LCDRGBC));

	return (0);
}

static int
jzlcd_configure(struct jzlcd_softc *sc, const struct videomode *mode)
{
	size_t fbsize;
	int error;

	fbsize = round_page(mode->hdisplay * mode->vdisplay * (FB_BPP / NBBY));

	/* Detach the old FB device */
	if (sc->fbdev != NULL) {
		device_delete_child(sc->dev, sc->fbdev);
		sc->fbdev = NULL;
	}

	/* If the FB size has changed, free the old FB memory */
	if (sc->fbsize > 0 && sc->fbsize != fbsize) {
		jzlcd_freefb(sc);
		sc->vaddr = 0;
	}

	/* Allocate the FB if necessary */
	sc->fbsize = fbsize;
	if (sc->vaddr == 0) {
		error = jzlcd_allocfb(sc);
		if (error != 0) {
			device_printf(sc->dev, "failed to allocate FB memory\n");
			return (ENXIO);
		}
	}

	/* Setup video mode */
	error = jzlcd_set_videomode(sc, mode);
	if (error != 0)
		return (error);

	/* Attach framebuffer device */
	sc->info.fb_name = device_get_nameunit(sc->dev);
	sc->info.fb_vbase = (intptr_t)sc->vaddr;
	sc->info.fb_pbase = sc->paddr;
	sc->info.fb_size = sc->fbsize;
	sc->info.fb_bpp = sc->info.fb_depth = FB_BPP;
	sc->info.fb_stride = mode->hdisplay * (FB_BPP / NBBY);
	sc->info.fb_width = mode->hdisplay;
	sc->info.fb_height = mode->vdisplay;

	sc->fbdev = device_add_child(sc->dev, "fbd", device_get_unit(sc->dev));
	if (sc->fbdev == NULL) {
		device_printf(sc->dev, "failed to add fbd child\n");
		return (ENOENT);
	}

	error = device_probe_and_attach(sc->fbdev);
	if (error != 0) {
		device_printf(sc->dev, "failed to attach fbd device\n");
		return (error);
	}

	return (0);
}

static void
jzlcd_hdmi_event(void *arg, device_t hdmi_dev)
{
	const struct videomode *mode;
	struct videomode hdmi_mode;
	struct jzlcd_softc *sc;
	struct edid_info ei;
	uint8_t *edid;
	uint32_t edid_len;
	int error;

	sc = arg;
	edid = NULL;
	edid_len = 0;
	mode = NULL;

	error = HDMI_GET_EDID(hdmi_dev, &edid, &edid_len);
	if (error != 0) {
		device_printf(sc->dev, "failed to get EDID: %d\n", error);
	} else {
		error = edid_parse(edid, &ei);
		if (error != 0) {
			device_printf(sc->dev, "failed to parse EDID: %d\n",
			    error);
		} else {
			if (bootverbose)
				edid_print(&ei);
			mode = ei.edid_preferred_mode;
		}
	}

	/* If the preferred mode could not be determined, use the default */
	//if (mode == NULL)
	if (1)
		mode = pick_mode_by_ref(FB_DEFAULT_W, FB_DEFAULT_H,
		    FB_DEFAULT_REF);

	if (mode == NULL) {
		device_printf(sc->dev, "failed to find usable video mode\n");
		return;
	}

	if (bootverbose)
		device_printf(sc->dev, "using %dx%d\n",
		    mode->hdisplay, mode->vdisplay);

	/* Configure LCD controller */
	error = jzlcd_configure(sc, mode);
	if (error != 0) {
		device_printf(sc->dev, "failed to configure FB: %d\n", error);
		return;
	}

	hdmi_mode = *mode;
	hdmi_mode.hskew = mode->hsync_end - mode->hsync_start;
	hdmi_mode.flags |= VID_HSKEW;
	HDMI_SET_VIDEOMODE(hdmi_dev, &hdmi_mode);
}

static void
jzlcd_dmamap_cb(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
	if (error != 0)
		return;
	*(bus_addr_t *)arg = segs[0].ds_addr;
}

static int
jzlcd_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "ingenic,jz4780-lcd"))
		return (ENXIO);

	device_set_desc(dev, "Ingenic JZ4780 LCD Controller");
	return (BUS_PROBE_DEFAULT);
}

static int
jzlcd_attach(device_t dev)
{
	struct jzlcd_softc *sc;
	int error;

	sc = device_get_softc(dev);

	sc->dev = dev;

	if (bus_alloc_resources(dev, jzlcd_spec, &sc->res)) {
		device_printf(dev, "cannot allocate resources for device\n");
		return (ENXIO);
	}

	if (clk_get_by_ofw_name(dev, 0, "lcd_clk", &sc->clk) != 0 ||
	    clk_get_by_ofw_name(dev, 0, "lcd_pixclk", &sc->clk_pix) != 0) {
		device_printf(dev, "cannot get clocks\n");
		return (ENXIO);
	}
	if (clk_enable(sc->clk) != 0 || clk_enable(sc->clk_pix) != 0) {
		device_printf(dev, "cannot enable clocks\n");
		return (ENXIO);
	}

	error = bus_dma_tag_create(
	    bus_get_dma_tag(dev),
	    16, 0,
	    BUS_SPACE_MAXADDR_32BIT,
	    BUS_SPACE_MAXADDR,
	    NULL, NULL,
	    sizeof(struct lcd_frame_descriptor), 1,
	    sizeof(struct lcd_frame_descriptor),
	    0,
	    NULL, NULL,
	    &sc->fdesc_tag);
	if (error != 0) {
		device_printf(dev, "cannot create bus dma tag\n");
		return (ENXIO);
	}

	error = bus_dmamem_alloc(sc->fdesc_tag, (void **)&sc->fdesc,
	    BUS_DMA_NOCACHE | BUS_DMA_WAITOK | BUS_DMA_ZERO, &sc->fdesc_map);
	if (error != 0) {
		device_printf(dev, "cannot allocate dma descriptor\n");
		return (ENXIO);
	}

	error = bus_dmamap_load(sc->fdesc_tag, sc->fdesc_map, sc->fdesc,
	    sizeof(struct lcd_frame_descriptor), jzlcd_dmamap_cb,
	    &sc->fdesc_paddr, 0);
	if (error != 0) {
		device_printf(dev, "cannot load dma map\n");
		return (ENXIO);
	}

	sc->hdmi_evh = EVENTHANDLER_REGISTER(hdmi_event,
	    jzlcd_hdmi_event, sc, 0);

	return (0);
}

static struct fb_info *
jzlcd_fb_getinfo(device_t dev)
{
	struct jzlcd_softc *sc;

	sc = device_get_softc(dev);

	return (&sc->info);
}

static device_method_t jzlcd_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		jzlcd_probe),
	DEVMETHOD(device_attach,	jzlcd_attach),

	/* FB interface */
	DEVMETHOD(fb_getinfo,		jzlcd_fb_getinfo),

	DEVMETHOD_END
};

static driver_t jzlcd_driver = {
	"fb",
	jzlcd_methods,
	sizeof(struct jzlcd_softc),
};

static devclass_t jzlcd_devclass;

DRIVER_MODULE(fb, simplebus, jzlcd_driver, jzlcd_devclass, 0, 0);
