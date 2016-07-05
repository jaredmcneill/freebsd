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

#ifndef _A83T_DMAC_H_
#define	_A83T_DMAC_H_

#define	DMA_IRQ_EN_REG		0x000
#define	 DMA_HLAF_IRQ_EN(n)	(1 << ((n) * 4 + 0))
#define	 DMA_PKG_IRQ_EN(n)	(1 << ((n) * 4 + 1))
#define	 DMA_QUEUE_IRQ_EN(n)	(1 << ((n) * 4 + 2))
#define	DMA_IRQ_PEND_REG	0x010
#define	 DMA_HLAF_IRQ_PEND(n)	(1 << ((n) * 4 + 0))
#define	 DMA_PKG_IRQ_PEND(n)	(1 << ((n) * 4 + 1))
#define	 DMA_QUEUE_IRQ_PEND(n)	(1 << ((n) * 4 + 2))
#define	 DMA_PKG_IRQ_MASK	0x22222222
#define	DMA_AUTO_GATE_REG	0x020
#define	 DMA_CHAN_CIRCUIT	(1 << 0)
#define	 DMA_COMMON_CIRCUIT	(1 << 1)
#define	 DMA_MCLK_CIRCUIT	(1 << 2)
#define	DMA_SECURE_REG		0x028
#define	 DMA_SECURE(n)		(1 << (n))
#define	DMA_STA_REG		0x030
#define	 DMA_STATUS(n)		(1 << (n))
#define	DMA_EN_REG(n)		(0x100 + (n) * 0x40 + 0x0)
#define	 DMA_EN			(1 << 0)
#define	DMA_PAU_REG(n)		(0x100 + (n) * 0x40 + 0x4)
#define	 DMA_PAUSE		(1 << 0)
#define	DMA_DESC_ADDR_REG(n)	(0x100 + (n) * 0x40 + 0x8)
#define	DMA_CFG_REG(n)		(0x100 + (n) * 0x40 + 0xc)
#define	 DMA_DEST_DATA_WIDTH	(0x3 << 25)
#define	  DMA_DEST_DATA_WIDTH_SHIFT	25
#define	  DMA_DEST_DATA_WIDTH_8BIT	0
#define	  DMA_DEST_DATA_WIDTH_16BIT	1
#define	  DMA_DEST_DATA_WIDTH_32BIT	2
#define	 DMA_DEST_BST_LEN	(0x3 << 23)
#define	  DMA_DEST_BST_LEN_SHIFT	23
#define	  DMA_DEST_BST_LEN_1		0
#define	  DMA_DEST_BST_LEN_8		2
#define	 DMA_DEST_ADDR_MODE	(0x3 << 21)
#define	  DMA_DEST_ADDR_MODE_SHIFT	21
#define	  DMA_DEST_ADDR_MODE_LINEAR	0
#define	  DMA_DEST_ADDR_MODE_IO		1
#define	 DMA_DEST_DRQ_TYPE	(0x1f << 16)
#define	  DMA_DEST_DRQ_TYPE_SHIFT	16
#define	 DMA_SRC_DATA_WIDTH	(0x3 << 9)
#define	  DMA_SRC_DATA_WIDTH_SHIFT	9
#define	  DMA_SRC_DATA_WIDTH_8BIT	0
#define	  DMA_SRC_DATA_WIDTH_16BIT	1
#define	  DMA_SRC_DATA_WIDTH_32BIT	2
#define	 DMA_SRC_BST_LEN	(0x3 << 7)
#define	  DMA_SRC_BST_LEN_SHIFT		7
#define	  DMA_SRC_BST_LEN_1		0
#define	  DMA_SRC_BST_LEN_8		2
#define	 DMA_SRC_ADDR_MODE	(0x3 << 5)
#define	  DMA_SRC_ADDR_MODE_SHIFT	5
#define	  DMA_SRC_ADDR_MODE_LINEAR	0
#define	  DMA_SRC_ADDR_MODE_IO		1
#define	 DMA_SRC_DRQ_TYPE	(0x1f << 0)
#define	  DMA_SRC_DRQ_TYPE_SHIFT	0
#define	DMA_CUR_SRC_REG(n)	(0x100 + (n) * 0x40 + 0x10)
#define	DMA_CUR_DEST_REG(n)	(0x100 + (n) * 0x40 + 0x14)
#define	DMA_BCNT_LEFT_REG(n)	(0x100 + (n) * 0x40 + 0x18)
#define	 DMA_BCNT_LEFT		(0x1ffffff << 0)
#define	  DMA_BCNT_LEFT_SHIFT		0
#define	DMA_PARA_REG(n)		(0x100 + (n) * 0x40 + 0x1c)
#define	 DATA_BLK_SIZE		(0xff << 8)
#define	  DATA_BLK_SIZE_SHIFT		8
#define	 WAIT_CYC		(0xf << 0)
#define	  WAIT_CYC_SHIFT		0

struct a83t_dma_desc {
	uint32_t	config;
	uint32_t	src;
	uint32_t	dst;
	uint32_t	bcnt;
	uint32_t	param;
	uint32_t	link;
#define	DMA_LINK_END	0xfffff800
} __packed;

#endif /* !_A83T_DMAC_H_ */
