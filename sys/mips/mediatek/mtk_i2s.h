/*-
 * Copyright (c) 2020 - 2021 Priit Trees trees@neti.ee
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#ifndef	__MTK_I2S_H__
#define	__MTK_I2S_H__

#define	MTK_I2S_CFG0		0x00
#define	 MTK_I2S_CFG0_I2S_EN	(1U << 31)
#define	 MTK_I2S_CFG0_DMA_EN	(1U << 30)
#define	 MTK_I2S_CFG0_BYTE_SWAP	(1U << 28)
#define	 MTK_I2S_CFG0_TX_EN	(1U << 24)
#define	 MTK_I2S_CFG0_RX_EN	(1U << 20)

#define	 MTK_I2S_CFG0_SLAVE	(1U << 16)
#define	 MTK_I2S_CFG0_RX_THRES	12
#define	 MTK_I2S_CFG0_TX_THRES	4

/* for MTK7628 */
#define	 MTK_I2S_CFG0_FMT_LE	(1U << 29)
#define	 MTK_I2S_CFG0_SYS_BE	(1U << 28)
#define	 MTK_I2S_CFG0_NORM_24	(1U << 18)
#define	 MTK_I2S_CFG0_DATA_24	(1U << 17)

#define	MTK_I2S_CFG0_DFT_THRES ((4 << MTK_I2S_CFG0_RX_THRES) | \
                   (4 << MTK_I2S_CFG0_TX_THRES))

#define	MTK_I2S_INT_STATUS	0x04
#define	MTK_I2S_INT_EN		0x08
#define	MTK_I2S_FF_STATUS	0x0c
#define	MTK_I2S_WR_EG		0x10
#define	MTK_I2S_RR_EG		0x14
#define	MTK_I2S_CFG1		0x18
#define	 MTK_I2S_LBK_EN		(1U << 31)
#define	 MTK_I2S_EXT_LBK_EN	(1U << 31)

/* for MTK7628 */
#define	 MTK_I2S_CFG1_LEFT_J	(1U << 0)

#define	MTK_I2S_DIVCMP		0x20
#define	 MTK_I2S_CLK_EN		(1U << 31)
#define	MTK_I2S_DIVINT		0x24

#endif	/* __MTK_I2S_H__ */
