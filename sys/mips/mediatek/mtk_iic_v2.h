/*-
 * Copyright (c) 2019 - 2021 Priit Trees <trees@neti.ee>
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
 */

#ifndef	__MTK_IIC_V2_H__
#define	__MTK_IIC_V2_H__

#define	MTK_IIC_SM0CFG0			0x08
#define	MTK_IIC_SM0DOUT			0x10
#define	MTK_IIC_SM0DIN			0x14
#define	MTK_IIC_SM0ST			0x18
#define	 MTK_IIC_SM0_RDATA_RDY		(1U << 2)
#define	 MTK_IIC_SM0_WDATA_EMPTY	(1U << 1)
#define	 MTK_IIC_SM0_BUSY		(1U << 0)
#define	MTK_IIC_SM0AUTO			0x1C
#define	 MTK_IIC_SM0_START_RW(x)	(((x) & 0x01) << 0)
#define	MTK_IIC_SM0CFG1			0x20
#define	 MTK_IIC_SM0_BYTECNT(x)		(((x) & 0x3f) << 0)
#define	MTK_IIC_SM0CFG2			0x28
#define	 MTK_IIC_SM0_IS_AUTOMODE	(1U << 0)
#define	MTK_IIC_SM0CTL0			0x40
#define	 MTK_IIC_SM0_CLK_DIV(x)		(((x) & 0xFFF) << 16)
#define	 MTK_IIC_SM0_ODRAIN		(1U << 31)
#define	 MTK_IIC_SM0_SIF_VSYNC		(1U << 15)
#define	 MTK_IIC_SM0_VSYNC_MODE		(((x) & 0x03) << 12)
#define	 MTK_IIC_SM0_CS_STATUS		(1U << 4)
#define	 MTK_IIC_SM0_SCL_STATE		(1U << 3)
#define	 MTK_IIC_SM0_SDA_STATE		(1U << 2)
#define	 MTK_IIC_SM0_EN			(1U << 1)
#define	 MTK_IIC_SM0_SCL_STRECH		(1U << 0)
#define	MTK_IIC_SM0CTL1			0x44
#define	 MTK_IIC_SM0_ACK(x)		(((x) >> 16) & 0xFF)
#define	 MTK_IIC_SM0_PGLEN(x)		(((x) & 0x07) << 8)
#define	 MTK_IIC_SM0_MODE(x)		(((x) & 0x07) << 4)
#define	 MTK_IIC_SM0_TRI		(1U << 0)
#define	MTK_IIC_SM0D0			0x50		/* for data 0 - 3 */
#define	MTK_IIC_SM0D1			0x54		/* for data 4 - 7 */
#define	MTK_IIC_PINTEN			0x5C
#define	 MTK_IIC_SM0_INT_EN		(1U << 0)
#define	MTK_IIC_PINTST			0x60
#define	 MTK_IIC_SM0_INT_ST		(1U << 0)
#define	MTK_IIC_PINTCL			0x64
#define	 MTK_IIC_SM0_INT_CL		(1U << 0)

#define	MTK_IIC_OP_WRITE		0
#define	MTK_IIC_OP_READ			1
#define	MTK_IIC_VSYNC_DISABLE		0
#define	MTK_IIC_VSYNC_PULSE		1
#define	MTK_IIC_VSYNC_RISING		2
#define	MTK_IIC_MODE_START		1
#define	MTK_IIC_MODE_WRITE		2
#define	MTK_IIC_MODE_STOP		3
#define	MTK_IIC_MODE_READ_NACK		4
#define	MTK_IIC_MODE_READ_ACK		5

#endif	/* _MTK_IIC_V2_H_ */
