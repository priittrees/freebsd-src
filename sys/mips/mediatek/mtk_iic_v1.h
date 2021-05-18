/*-
 * Copyright (c) 2020 - 2021 Priit Trees <trees@neti.ee>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice unmodified, this list of conditions, and the following
 *    disclaimer.
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

#ifndef	_MTK_IIC_V1_H_
#define	_MTK_IIC_V1_H_

#define	MTK_IIC_CONFIG			0x00
#define	 MTK_IIC_CONFIG_ADDRLEN(x)	(((x) & 0x7) << 5)
#define	 MTK_IIC_CONFIG_ADDRLEN_7 	6
#define	 MTK_IIC_CONFIG_ADDRLEN_8 	7
#define	 MTK_IIC_CONFIG_DEVADLEN(x)	(((x) & 0x7) << 2)
#define	 MTK_IIC_CONFIG_DEVADLEN_6	5
#define	 MTK_IIC_CONFIG_DEVADLEN_7	6
#define	 MTK_IIC_CONFIG_ADDRDIS		(1U << 1)
#define	 MTK_IIC_CONFIG_DEVDIS		(1U << 0)
#define	MTK_IIC_CLKDIV			0x04
#define	MTK_IIC_DEVADDR			0x08
#define	MTK_IIC_ADDR			0x0C
#define	MTK_IIC_DATAOUT			0x10
#define	MTK_IIC_DATAIN			0x14
#define	MTK_IIC_STATUS			0x18
#define	 MTK_IIC_STATUS_STARTERR	(1U << 4)
#define	 MTK_IIC_STATUS_ACKERR		(1U << 3)
#define	 MTK_IIC_STATUS_DATARDY		(1U << 2)
#define	 MTK_IIC_STATUS_SDOEMPTY	(1U << 1)
#define	 MTK_IIC_STATUS_BUSY		(1U << 0)
#define	MTK_IIC_STARTXFR		0x1C
#define	 MTK_IIC_NODATA			(1U << 1)
#define	 MTK_IIC_CONFIG_RWDIR(x)	(((x) & 0x1) << 0)
#define	MTK_IIC_BYTECNT			0x20
#define	 MTK_IIC_BYTECNT_BYCNT(x)	(((x) & 0x3f))

#define	MTK_IIC_OP_READ			1
#define	MTK_IIC_OP_WRITE		0

#endif	/* _MTK_IIC_V1_H_ */
