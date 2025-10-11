#ifndef _MEDIATEKPINCTRL_H
#define _MEDIATEKPINCTRL_H

/*
 * Most of the code is imported from Linux. Currently, this is a faster,
 * more convenient, and more error-free solution.
 */

/* List these attributes which could be modified for the pin */
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

/* Group the pins by the driving current */
enum {
	DRV_FIXED,
	DRV_GRP0,
	DRV_GRP1,
	DRV_GRP2,
	DRV_GRP3,
	DRV_GRP4,
	DRV_GRP_MAX,
};

/**
 * struct mtk_drive_desc - the structure that holds the information
 *                          of the driving current
 * @min:        the minimum current of this group
 * @max:        the maximum current of this group
 * @step:       the step current of this group
 * @scal:       the weight factor
 *
 * formula: output = ((input) / step - 1) * scal
 */
struct mtk_drive_desc {
        uint8_t min;
        uint8_t max;
        uint8_t step;
        uint8_t scal;
};

/**
 * struct mtk_eint_desc - the structure that providing information
 *                             for eint data per pin
 * @eint_m:             the eint mux for this pin
 * @eitn_n:             the eint number for this pin
 */
struct mtk_eint_desc {
        uint16_t eint_m;
        uint16_t eint_n;
};

/* The groups of drive strength */
static const struct mtk_drive_desc mtk_drive[] = {
        [DRV_GRP0] = { 4, 16, 4, 1 },
};

/**
 * struct mtk_pin_desc - the structure that providing information
 *                             for each pin of chips
 * @number:             unique pin number from the global pin number space
 * @name:               name for this pin
 * @eint:               the eint data for this pin
 * @drv_n:              the index with the driving group
 * @funcs:              all available functions for this pins (only used in
 *                      those drivers compatible to pinctrl-mtk-common.c-like
 *                      ones)
 */
struct mtk_pin_desc {
        unsigned int number;
        const char *name;
        struct mtk_eint_desc eint;
        uint8_t drv_n;
        struct mtk_func_desc *funcs;
};

#define MTK_PIN(_number, _name, _eint_m, _eint_n, _drv_n) {     \
	.number = _number,                      \
	.name = _name,                          \
	.eint = {                               \
		.eint_m = _eint_m,              \
		.eint_n = _eint_n,              \
	},                                      \
	.drv_n = _drv_n,                        \
	.funcs = NULL,                          \
}

struct mtk_pin_field_calc {
        uint16_t s_pin;
        uint16_t e_pin;
        uint8_t  i_base;
        uint32_t s_addr;
        uint8_t  x_addrs;
        uint8_t  s_bit;
        uint8_t  x_bits;
        uint8_t  sz_reg;
        uint8_t  fixed;
};

#define PIN_FIELD_CALC(_s_pin, _e_pin, _i_base, _s_addr, _x_addrs,      \
                       _s_bit, _x_bits, _sz_reg, _fixed) {              \
                .s_pin = _s_pin,                                        \
                .e_pin = _e_pin,                                        \
                .i_base = _i_base,                                      \
                .s_addr = _s_addr,                                      \
                .x_addrs = _x_addrs,                                    \
                .s_bit = _s_bit,                                        \
                .x_bits = _x_bits,                                      \
                .sz_reg = _sz_reg,                                      \
                .fixed = _fixed,                                        \
        }

#define PIN_FIELD(_s_pin, _e_pin, _s_addr, _x_addrs, _s_bit, _x_bits)   \
        PIN_FIELD_CALC(_s_pin, _e_pin, 0, _s_addr, _x_addrs, _s_bit,    \
                       _x_bits, 32, 0)

#define PINS_FIELD(_s_pin, _e_pin, _s_addr, _x_addrs, _s_bit, _x_bits)  \
        PIN_FIELD_CALC(_s_pin, _e_pin, 0, _s_addr, _x_addrs, _s_bit,    \
                       _x_bits, 32, 1)

struct mtk_pin_group {
        char *name;
	int npins;
        int *pins;
	int *funcs;
};

#define PINCTRL_PIN_GROUP(_name, grp ){			\
	.name = _name,					\
	.npins = sizeof(grp##_pins) / sizeof(int),	\
	.pins = grp##_pins,				\
	.funcs = grp##_funcs				\
}


struct mtk_padconf {
        const struct mtk_pin_desc *		pins_names;
        uint32_t				npins;
	const struct mtk_pin_reg_calc	*	pin_reg;
	const struct mtk_pin_group	*	pins_group;
	uint32_t				npins_group;
};

/* struct mtk_pin_reg_calc - the structure that holds all ranges used to
 *                           determine which register the pin would make use of
 *                           for certain pin attribute.
 * @range:                   the start address for the range
 * @nranges:                 the number of items in the range
 */
struct mtk_pin_reg_calc {
        const struct mtk_pin_field_calc *range;
        unsigned int nranges;
};

#define MTK_RANGE(_a) {							\
	.range = (_a),							\
	.nranges = sizeof(_a) / sizeof(struct mtk_pin_field_calc),	\
}

/* struct mtk_pin_field - the structure that holds the information of the field
 *                        used to describe the attribute for the pin
 * @base:               the index pointing to the entry in base address list
 * @offset:             the register offset relative to the base address
 * @mask:               the mask used to filter out the field from the register
 * @bitpos:             the start bit relative to the register
 * @next:               the indication that the field would be extended to the
                        next register
 */
struct mtk_pin_field {
        uint8_t  index;
        uint32_t offset;
        uint32_t mask;
        uint8_t  bitpos;
        uint8_t  next;
};

#endif /* _MT7622_PINCTRL_H_ */
