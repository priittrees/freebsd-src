/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 - 2026 Priit Trees <trees@neti.ee>
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
 *
 */

#ifndef _MTK_PINCTRL_H_
#define _MTK_PINCTRL_H_

enum {
	PINCTRL_PIN_REG_MODE,
	PINCTRL_PIN_REG_DIR,
	PINCTRL_PIN_REG_DI,
	PINCTRL_PIN_REG_DO,
	PINCTRL_PIN_REG_SR,
	PINCTRL_PIN_REG_SMT,
	PINCTRL_PIN_REG_PD,
	PINCTRL_PIN_REG_PU,
	PINCTRL_PIN_REG_E4,
	PINCTRL_PIN_REG_E8,
	PINCTRL_PIN_REG_TDSEL,
	PINCTRL_PIN_REG_RDSEL,
	PINCTRL_PIN_REG_DRV,
	PINCTRL_PIN_REG_PUPD,
	PINCTRL_PIN_REG_R0,
	PINCTRL_PIN_REG_R1,
	PINCTRL_PIN_REG_IES,
	PINCTRL_PIN_REG_PULLEN,
	PINCTRL_PIN_REG_PULLSEL,
	PINCTRL_PIN_REG_DRV_EN,
	PINCTRL_PIN_REG_DRV_E0,
	PINCTRL_PIN_REG_DRV_E1,
	PINCTRL_PIN_REG_DRV_ADV,
	PINCTRL_PIN_REG_RSEL,
	PINCTRL_PIN_REG_MAX,
};

struct mtk_pin {
	uint8_t		pin;
	const char	*name;
	uint16_t	eint_mux;
	uint16_t	eint_num;
};

#define PINCTRL_PIN(_pin, _name, _eint_mux, _eint_num){		\
	.pin = _pin,						\
	.name = _name,						\
	.eint_mux = _eint_mux,					\
	.eint_num = _eint_num,					\
}

struct mtk_pin_func {
	int pin;
	int func;
};

#define PINCTRL_GROUP(_name, _pconf){				\
	.name = _name,						\
	.pconf = _pconf,					\
	.npins = sizeof(_pconf) / sizeof(struct mtk_pin_func)	\
}

struct mtk_group_pins {
	char 			*name;
	struct mtk_pin_func	*pconf;
	int			npins;
};

struct mtk_pinctrl_reg {
	uint8_t		pin;
	uint32_t	reg;
	uint32_t	bit;
	uint32_t	mask;
};

#define PINCTRL_REG(_pin, _reg, _bit, _mask)	\
{						\
	.pin = _pin,				\
	.reg = _reg,				\
	.bit = _bit,				\
	.mask = _mask,				\
}

struct mtk_pinctrl{
	const struct mtk_pinctrl_reg *reg;
	unsigned int nregs;
};

#define PINCTRL(_reg) {						\
	.reg = _reg,						\
	.nregs = sizeof(_reg) / sizeof(struct mtk_pinctrl),	\
}

struct mtk_padconf {
	const struct mtk_pin		*	pins;
	uint32_t				npins;
	const struct mtk_pinctrl	*	pinctrl;
	const struct mtk_group_pins	*       groups;
	uint32_t				ngroups;
};
#endif /* _MTK_PINCTRL_H_ */
