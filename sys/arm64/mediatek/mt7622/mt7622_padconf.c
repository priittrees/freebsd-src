/*-
 * SPDX-License-Identifier: BSD-2-Clause
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

/*
 * Most of the code is imported from Linux. Currently, this is a faster,
 * more convenient, and more error-free solution.
 * fix mt7622_emmc_rst_funcs value. It must be 2.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/types.h>

#include <arm64/mediatek/mediatek_pinctrl.h>

#define MT7622_PIN(_number, _name)					\
        MTK_PIN(_number, _name, 1, _number, DRV_GRP0)

static const struct mtk_pin_desc mt7622_pins[] = {
        MT7622_PIN(0, "GPIO_A"),
        MT7622_PIN(1, "I2S1_IN"),
        MT7622_PIN(2, "I2S1_OUT"),
        MT7622_PIN(3, "I2S_BCLK"),
        MT7622_PIN(4, "I2S_WS"),
        MT7622_PIN(5, "I2S_MCLK"),
        MT7622_PIN(6, "TXD0"),
        MT7622_PIN(7, "RXD0"),
        MT7622_PIN(8, "SPI_WP"),
        MT7622_PIN(9, "SPI_HOLD"),
        MT7622_PIN(10, "SPI_CLK"),
        MT7622_PIN(11, "SPI_MOSI"),
        MT7622_PIN(12, "SPI_MISO"),
        MT7622_PIN(13, "SPI_CS"),
        MT7622_PIN(14, "I2C_SDA"),
        MT7622_PIN(15, "I2C_SCL"),
        MT7622_PIN(16, "I2S2_IN"),
        MT7622_PIN(17, "I2S3_IN"),
        MT7622_PIN(18, "I2S4_IN"),
        MT7622_PIN(19, "I2S2_OUT"),
        MT7622_PIN(20, "I2S3_OUT"),
        MT7622_PIN(21, "I2S4_OUT"),
        MT7622_PIN(22, "GPIO_B"),
        MT7622_PIN(23, "MDC"),
        MT7622_PIN(24, "MDIO"),
        MT7622_PIN(25, "G2_TXD0"),
        MT7622_PIN(26, "G2_TXD1"),
        MT7622_PIN(27, "G2_TXD2"),
        MT7622_PIN(28, "G2_TXD3"),
        MT7622_PIN(29, "G2_TXEN"),
        MT7622_PIN(30, "G2_TXC"),
        MT7622_PIN(31, "G2_RXD0"),
        MT7622_PIN(32, "G2_RXD1"),
        MT7622_PIN(33, "G2_RXD2"),
        MT7622_PIN(34, "G2_RXD3"),
        MT7622_PIN(35, "G2_RXDV"),
        MT7622_PIN(36, "G2_RXC"),
        MT7622_PIN(37, "NCEB"),
        MT7622_PIN(38, "NWEB"),
        MT7622_PIN(39, "NREB"),
        MT7622_PIN(40, "NDL4"),
        MT7622_PIN(41, "NDL5"),
        MT7622_PIN(42, "NDL6"),
        MT7622_PIN(43, "NDL7"),
        MT7622_PIN(44, "NRB"),
        MT7622_PIN(45, "NCLE"),
        MT7622_PIN(46, "NALE"),
        MT7622_PIN(47, "NDL0"),
        MT7622_PIN(48, "NDL1"),
        MT7622_PIN(49, "NDL2"),
        MT7622_PIN(50, "NDL3"),
        MT7622_PIN(51, "MDI_TP_P0"),
        MT7622_PIN(52, "MDI_TN_P0"),
        MT7622_PIN(53, "MDI_RP_P0"),
        MT7622_PIN(54, "MDI_RN_P0"),
        MT7622_PIN(55, "MDI_TP_P1"),
        MT7622_PIN(56, "MDI_TN_P1"),
        MT7622_PIN(57, "MDI_RP_P1"),
        MT7622_PIN(58, "MDI_RN_P1"),
        MT7622_PIN(59, "MDI_RP_P2"),
        MT7622_PIN(60, "MDI_RN_P2"),
        MT7622_PIN(61, "MDI_TP_P2"),
        MT7622_PIN(62, "MDI_TN_P2"),
        MT7622_PIN(63, "MDI_TP_P3"),
        MT7622_PIN(64, "MDI_TN_P3"),
        MT7622_PIN(65, "MDI_RP_P3"),
        MT7622_PIN(66, "MDI_RN_P3"),
        MT7622_PIN(67, "MDI_RP_P4"),
        MT7622_PIN(68, "MDI_RN_P4"),
        MT7622_PIN(69, "MDI_TP_P4"),
        MT7622_PIN(70, "MDI_TN_P4"),
        MT7622_PIN(71, "PMIC_SCL"),
        MT7622_PIN(72, "PMIC_SDA"),
        MT7622_PIN(73, "SPIC1_CLK"),
        MT7622_PIN(74, "SPIC1_MOSI"),
        MT7622_PIN(75, "SPIC1_MISO"),
        MT7622_PIN(76, "SPIC1_CS"),
        MT7622_PIN(77, "GPIO_D"),
        MT7622_PIN(78, "WATCHDOG"),
        MT7622_PIN(79, "RTS3_N"),
        MT7622_PIN(80, "CTS3_N"),
        MT7622_PIN(81, "TXD3"),
        MT7622_PIN(82, "RXD3"),
        MT7622_PIN(83, "PERST0_N"),
        MT7622_PIN(84, "PERST1_N"),
        MT7622_PIN(85, "WLED_N"),
        MT7622_PIN(86, "EPHY_LED0_N"),
        MT7622_PIN(87, "AUXIN0"),
        MT7622_PIN(88, "AUXIN1"),
        MT7622_PIN(89, "AUXIN2"),
        MT7622_PIN(90, "AUXIN3"),
        MT7622_PIN(91, "TXD4"),
        MT7622_PIN(92, "RXD4"),
        MT7622_PIN(93, "RTS4_N"),
        MT7622_PIN(94, "CTS4_N"),
        MT7622_PIN(95, "PWM1"),
        MT7622_PIN(96, "PWM2"),
        MT7622_PIN(97, "PWM3"),
        MT7622_PIN(98, "PWM4"),
        MT7622_PIN(99, "PWM5"),
        MT7622_PIN(100, "PWM6"),
        MT7622_PIN(101, "PWM7"),
        MT7622_PIN(102, "GPIO_E"),
};

static const struct mtk_pin_field_calc mt7622_pin_mode_range[] = {
        PIN_FIELD(0, 0, 0x320, 0x10, 16, 4),
        PIN_FIELD(1, 4, 0x3a0, 0x10, 16, 4),
        PIN_FIELD(5, 5, 0x320, 0x10, 0, 4),
        PINS_FIELD(6, 7, 0x300, 0x10, 4, 4),
        PIN_FIELD(8, 9, 0x350, 0x10, 20, 4),
        PINS_FIELD(10, 13, 0x300, 0x10, 8, 4),
        PIN_FIELD(14, 15, 0x320, 0x10, 4, 4),
        PIN_FIELD(16, 17, 0x320, 0x10, 20, 4),
        PIN_FIELD(18, 21, 0x310, 0x10, 16, 4),
        PIN_FIELD(22, 22, 0x380, 0x10, 16, 4),
        PINS_FIELD(23, 24, 0x300, 0x10, 24, 4),
        PINS_FIELD(25, 36, 0x300, 0x10, 12, 4),
        PINS_FIELD(37, 50, 0x300, 0x10, 20, 4),
        PIN_FIELD(51, 70, 0x330, 0x10, 4, 4),
        PINS_FIELD(71, 72, 0x300, 0x10, 16, 4),
        PIN_FIELD(73, 76, 0x310, 0x10, 0, 4),
        PIN_FIELD(77, 77, 0x320, 0x10, 28, 4),
        PIN_FIELD(78, 78, 0x320, 0x10, 12, 4),
        PIN_FIELD(79, 82, 0x3a0, 0x10, 0, 4),
        PIN_FIELD(83, 83, 0x350, 0x10, 28, 4),
        PIN_FIELD(84, 84, 0x330, 0x10, 0, 4),
        PIN_FIELD(85, 90, 0x360, 0x10, 4, 4),
        PIN_FIELD(91, 94, 0x390, 0x10, 16, 4),
        PIN_FIELD(95, 97, 0x380, 0x10, 20, 4),
        PIN_FIELD(98, 101, 0x390, 0x10, 0, 4),
        PIN_FIELD(102, 102, 0x360, 0x10, 0, 4),
};

static const struct mtk_pin_field_calc mt7622_pin_dir_range[] = {
        PIN_FIELD(0, 102, 0x0, 0x10, 0, 1),
};

static const struct mtk_pin_field_calc mt7622_pin_di_range[] = {
        PIN_FIELD(0, 102, 0x200, 0x10, 0, 1),
};

static const struct mtk_pin_field_calc mt7622_pin_do_range[] = {
        PIN_FIELD(0, 102, 0x100, 0x10, 0, 1),
};

static const struct mtk_pin_field_calc mt7622_pin_sr_range[] = {
        PIN_FIELD(0, 31, 0x910, 0x10, 0, 1),
        PIN_FIELD(32, 50, 0xa10, 0x10, 0, 1),
        PIN_FIELD(51, 70, 0x810, 0x10, 0, 1),
        PIN_FIELD(71, 72, 0xb10, 0x10, 0, 1),
        PIN_FIELD(73, 86, 0xb10, 0x10, 4, 1),
        PIN_FIELD(87, 90, 0xc10, 0x10, 0, 1),
        PIN_FIELD(91, 102, 0xb10, 0x10, 18, 1),
};

static const struct mtk_pin_field_calc mt7622_pin_smt_range[] = {
        PIN_FIELD(0, 31, 0x920, 0x10, 0, 1),
        PIN_FIELD(32, 50, 0xa20, 0x10, 0, 1),
        PIN_FIELD(51, 70, 0x820, 0x10, 0, 1),
        PIN_FIELD(71, 72, 0xb20, 0x10, 0, 1),
        PIN_FIELD(73, 86, 0xb20, 0x10, 4, 1),
        PIN_FIELD(87, 90, 0xc20, 0x10, 0, 1),
        PIN_FIELD(91, 102, 0xb20, 0x10, 18, 1),
};

static const struct mtk_pin_field_calc mt7622_pin_pu_range[] = {
        PIN_FIELD(0, 31, 0x930, 0x10, 0, 1),
        PIN_FIELD(32, 50, 0xa30, 0x10, 0, 1),
        PIN_FIELD(51, 70, 0x830, 0x10, 0, 1),
        PIN_FIELD(71, 72, 0xb30, 0x10, 0, 1),
        PIN_FIELD(73, 86, 0xb30, 0x10, 4, 1),
        PIN_FIELD(87, 90, 0xc30, 0x10, 0, 1),
        PIN_FIELD(91, 102, 0xb30, 0x10, 18, 1),
};

static const struct mtk_pin_field_calc mt7622_pin_pd_range[] = {
        PIN_FIELD(0, 31, 0x940, 0x10, 0, 1),
        PIN_FIELD(32, 50, 0xa40, 0x10, 0, 1),
        PIN_FIELD(51, 70, 0x840, 0x10, 0, 1),
        PIN_FIELD(71, 72, 0xb40, 0x10, 0, 1),
        PIN_FIELD(73, 86, 0xb40, 0x10, 4, 1),
        PIN_FIELD(87, 90, 0xc40, 0x10, 0, 1),
        PIN_FIELD(91, 102, 0xb40, 0x10, 18, 1),
};

static const struct mtk_pin_field_calc mt7622_pin_e4_range[] = {
        PIN_FIELD(0, 31, 0x960, 0x10, 0, 1),
        PIN_FIELD(32, 50, 0xa60, 0x10, 0, 1),
        PIN_FIELD(51, 70, 0x860, 0x10, 0, 1),
        PIN_FIELD(71, 72, 0xb60, 0x10, 0, 1),
        PIN_FIELD(73, 86, 0xb60, 0x10, 4, 1),
        PIN_FIELD(87, 90, 0xc60, 0x10, 0, 1),
        PIN_FIELD(91, 102, 0xb60, 0x10, 18, 1),
};

static const struct mtk_pin_field_calc mt7622_pin_e8_range[] = {
        PIN_FIELD(0, 31, 0x970, 0x10, 0, 1),
        PIN_FIELD(32, 50, 0xa70, 0x10, 0, 1),
        PIN_FIELD(51, 70, 0x870, 0x10, 0, 1),
        PIN_FIELD(71, 72, 0xb70, 0x10, 0, 1),
        PIN_FIELD(73, 86, 0xb70, 0x10, 4, 1),
        PIN_FIELD(87, 90, 0xc70, 0x10, 0, 1),
        PIN_FIELD(91, 102, 0xb70, 0x10, 18, 1),
};

static const struct mtk_pin_field_calc mt7622_pin_tdsel_range[] = {
        PIN_FIELD(0, 31, 0x980, 0x4, 0, 4),
        PIN_FIELD(32, 50, 0xa80, 0x4, 0, 4),
        PIN_FIELD(51, 70, 0x880, 0x4, 0, 4),
        PIN_FIELD(71, 72, 0xb80, 0x4, 0, 4),
        PIN_FIELD(73, 86, 0xb80, 0x4, 16, 4),
        PIN_FIELD(87, 90, 0xc80, 0x4, 0, 4),
        PIN_FIELD(91, 102, 0xb88, 0x4, 8, 4),
};

static const struct mtk_pin_field_calc mt7622_pin_rdsel_range[] = {
        PIN_FIELD(0, 31, 0x990, 0x4, 0, 6),
        PIN_FIELD(32, 50, 0xa90, 0x4, 0, 6),
        PIN_FIELD(51, 58, 0x890, 0x4, 0, 6),
        PIN_FIELD(59, 60, 0x894, 0x4, 28, 6),
        PIN_FIELD(61, 62, 0x894, 0x4, 16, 6),
        PIN_FIELD(63, 66, 0x898, 0x4, 8, 6),
        PIN_FIELD(67, 68, 0x89c, 0x4, 12, 6),
        PIN_FIELD(69, 70, 0x89c, 0x4, 0, 6),
        PIN_FIELD(71, 72, 0xb90, 0x4, 0, 6),
        PIN_FIELD(73, 86, 0xb90, 0x4, 24, 6),
        PIN_FIELD(87, 90, 0xc90, 0x4, 0, 6),
        PIN_FIELD(91, 102, 0xb9c, 0x4, 12, 6),
};

static const struct mtk_pin_reg_calc mt7622_reg_cals[PINCTRL_PIN_REG_MAX] = {
        [PINCTRL_PIN_REG_MODE] = MTK_RANGE(mt7622_pin_mode_range),
        [PINCTRL_PIN_REG_DIR] = MTK_RANGE(mt7622_pin_dir_range),
        [PINCTRL_PIN_REG_DI] = MTK_RANGE(mt7622_pin_di_range),
        [PINCTRL_PIN_REG_DO] = MTK_RANGE(mt7622_pin_do_range),
        [PINCTRL_PIN_REG_SR] = MTK_RANGE(mt7622_pin_sr_range),
        [PINCTRL_PIN_REG_SMT] = MTK_RANGE(mt7622_pin_smt_range),
        [PINCTRL_PIN_REG_PU] = MTK_RANGE(mt7622_pin_pu_range),
        [PINCTRL_PIN_REG_PD] = MTK_RANGE(mt7622_pin_pd_range),
        [PINCTRL_PIN_REG_E4] = MTK_RANGE(mt7622_pin_e4_range),
        [PINCTRL_PIN_REG_E8] = MTK_RANGE(mt7622_pin_e8_range),
        [PINCTRL_PIN_REG_TDSEL] = MTK_RANGE(mt7622_pin_tdsel_range),
        [PINCTRL_PIN_REG_RDSEL] = MTK_RANGE(mt7622_pin_rdsel_range),
};

/* ANTSEL */
static int mt7622_antsel0_pins[] = { 91, };
static int mt7622_antsel0_funcs[] = { 5, };
static int mt7622_antsel1_pins[] = { 92, };
static int mt7622_antsel1_funcs[] = { 5, };
static int mt7622_antsel2_pins[] = { 93, };
static int mt7622_antsel2_funcs[] = { 5, };
static int mt7622_antsel3_pins[] = { 94, };
static int mt7622_antsel3_funcs[] = { 5, };
static int mt7622_antsel4_pins[] = { 95, };
static int mt7622_antsel4_funcs[] = { 5, };
static int mt7622_antsel5_pins[] = { 96, };
static int mt7622_antsel5_funcs[] = { 5, };
static int mt7622_antsel6_pins[] = { 97, };
static int mt7622_antsel6_funcs[] = { 5, };
static int mt7622_antsel7_pins[] = { 98, };
static int mt7622_antsel7_funcs[] = { 5, };
static int mt7622_antsel8_pins[] = { 99, };
static int mt7622_antsel8_funcs[] = { 5, };
static int mt7622_antsel9_pins[] = { 100, };
static int mt7622_antsel9_funcs[] = { 5, };
static int mt7622_antsel10_pins[] = { 101, };
static int mt7622_antsel10_funcs[] = { 5, };
static int mt7622_antsel11_pins[] = { 102, };
static int mt7622_antsel11_funcs[] = { 5, };
static int mt7622_antsel12_pins[] = { 73, };
static int mt7622_antsel12_funcs[] = { 5, };
static int mt7622_antsel13_pins[] = { 74, };
static int mt7622_antsel13_funcs[] = { 5, };
static int mt7622_antsel14_pins[] = { 75, };
static int mt7622_antsel14_funcs[] = { 5, };
static int mt7622_antsel15_pins[] = { 76, };
static int mt7622_antsel15_funcs[] = { 5, };
static int mt7622_antsel16_pins[] = { 77, };
static int mt7622_antsel16_funcs[] = { 5, };
static int mt7622_antsel17_pins[] = { 22, };
static int mt7622_antsel17_funcs[] = { 5, };
static int mt7622_antsel18_pins[] = { 79, };
static int mt7622_antsel18_funcs[] = { 5, };
static int mt7622_antsel19_pins[] = { 80, };
static int mt7622_antsel19_funcs[] = { 5, };
static int mt7622_antsel20_pins[] = { 81, };
static int mt7622_antsel20_funcs[] = { 5, };
static int mt7622_antsel21_pins[] = { 82, };
static int mt7622_antsel21_funcs[] = { 5, };
static int mt7622_antsel22_pins[] = { 14, };
static int mt7622_antsel22_funcs[] = { 5, };
static int mt7622_antsel23_pins[] = { 15, };
static int mt7622_antsel23_funcs[] = { 5, };
static int mt7622_antsel24_pins[] = { 16, };
static int mt7622_antsel24_funcs[] = { 5, };
static int mt7622_antsel25_pins[] = { 17, };
static int mt7622_antsel25_funcs[] = { 5, };
static int mt7622_antsel26_pins[] = { 18, };
static int mt7622_antsel26_funcs[] = { 5, };
static int mt7622_antsel27_pins[] = { 19, };
static int mt7622_antsel27_funcs[] = { 5, };
static int mt7622_antsel28_pins[] = { 20, };
static int mt7622_antsel28_funcs[] = { 5, };
static int mt7622_antsel29_pins[] = { 21, };
static int mt7622_antsel29_funcs[] = { 5, };

/* EMMC */
static int mt7622_emmc_pins[] = { 40, 41, 42, 43, 44, 45, 47, 48, 49, 50, };
static int mt7622_emmc_funcs[] = { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, };

static int mt7622_emmc_rst_pins[] = { 37, };
static int mt7622_emmc_rst_funcs[] = { 2, };

/* LED for EPHY */
static int mt7622_ephy_leds_pins[] = { 86, 91, 92, 93, 94, };
static int mt7622_ephy_leds_funcs[] = { 0, 0, 0, 0, 0, };
static int mt7622_ephy0_led_pins[] = { 86, };
static int mt7622_ephy0_led_funcs[] = { 0, };
static int mt7622_ephy1_led_pins[] = { 91, };
static int mt7622_ephy1_led_funcs[] = { 2, };
static int mt7622_ephy2_led_pins[] = { 92, };
static int mt7622_ephy2_led_funcs[] = { 2, };
static int mt7622_ephy3_led_pins[] = { 93, };
static int mt7622_ephy3_led_funcs[] = { 2, };
static int mt7622_ephy4_led_pins[] = { 94, };
static int mt7622_ephy4_led_funcs[] = { 2, };

/* Embedded Switch */
static int mt7622_esw_pins[] = { 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61,
                                 62, 63, 64, 65, 66, 67, 68, 69, 70, };
static int mt7622_esw_funcs[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                  0, 0, 0, 0, 0, 0, 0, 0, 0, };
static int mt7622_esw_p0_p1_pins[] = { 51, 52, 53, 54, 55, 56, 57, 58, };
static int mt7622_esw_p0_p1_funcs[] = { 0, 0, 0, 0, 0, 0, 0, 0, };
static int mt7622_esw_p2_p3_p4_pins[] = { 59, 60, 61, 62, 63, 64, 65, 66, 67,
                                          68, 69, 70, };
static int mt7622_esw_p2_p3_p4_funcs[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                           0, 0, 0, };
/* RGMII via ESW */
static int mt7622_rgmii_via_esw_pins[] = { 59, 60, 61, 62, 63, 64, 65, 66,
                                           67, 68, 69, 70, };
static int mt7622_rgmii_via_esw_funcs[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                            0, };

/* RGMII via GMAC1 */
static int mt7622_rgmii_via_gmac1_pins[] = { 59, 60, 61, 62, 63, 64, 65, 66,
                                             67, 68, 69, 70, };
static int mt7622_rgmii_via_gmac1_funcs[] = { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
                                              2, };

/* RGMII via GMAC2 */
static int mt7622_rgmii_via_gmac2_pins[] = { 25, 26, 27, 28, 29, 30, 31, 32,
                                             33, 34, 35, 36, };
static int mt7622_rgmii_via_gmac2_funcs[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                              0, };

/* I2C */
static int mt7622_i2c0_pins[] = { 14, 15, };
static int mt7622_i2c0_funcs[] = { 0, 0, };
static int mt7622_i2c1_0_pins[] = { 55, 56, };
static int mt7622_i2c1_0_funcs[] = { 0, 0, };
static int mt7622_i2c1_1_pins[] = { 73, 74, };
static int mt7622_i2c1_1_funcs[] = { 3, 3, };
static int mt7622_i2c1_2_pins[] = { 87, 88, };
static int mt7622_i2c1_2_funcs[] = { 0, 0, };
static int mt7622_i2c2_0_pins[] = { 57, 58, };
static int mt7622_i2c2_0_funcs[] = { 0, 0, };
static int mt7622_i2c2_1_pins[] = { 75, 76, };
static int mt7622_i2c2_1_funcs[] = { 3, 3, };
static int mt7622_i2c2_2_pins[] = { 89, 90, };
static int mt7622_i2c2_2_funcs[] = { 0, 0, };

/* I2S */
static int mt7622_i2s_in_mclk_bclk_ws_pins[] = { 3, 4, 5, };
static int mt7622_i2s_in_mclk_bclk_ws_funcs[] = { 3, 3, 0, };
static int mt7622_i2s1_in_data_pins[] = { 1, };
static int mt7622_i2s1_in_data_funcs[] = { 0, };
static int mt7622_i2s2_in_data_pins[] = { 16, };
static int mt7622_i2s2_in_data_funcs[] = { 0, };
static int mt7622_i2s3_in_data_pins[] = { 17, };
static int mt7622_i2s3_in_data_funcs[] = { 0, };
static int mt7622_i2s4_in_data_pins[] = { 18, };
static int mt7622_i2s4_in_data_funcs[] = { 0, };
static int mt7622_i2s_out_mclk_bclk_ws_pins[] = { 3, 4, 5, };
static int mt7622_i2s_out_mclk_bclk_ws_funcs[] = { 0, 0, 0, };
static int mt7622_i2s1_out_data_pins[] = { 2, };
static int mt7622_i2s1_out_data_funcs[] = { 0, };
static int mt7622_i2s2_out_data_pins[] = { 19, };
static int mt7622_i2s2_out_data_funcs[] = { 0, };
static int mt7622_i2s3_out_data_pins[] = { 20, };
static int mt7622_i2s3_out_data_funcs[] = { 0, };
static int mt7622_i2s4_out_data_pins[] = { 21, };
static int mt7622_i2s4_out_data_funcs[] = { 0, };

/* IR */
static int mt7622_ir_0_tx_pins[] = { 16, };
static int mt7622_ir_0_tx_funcs[] = { 4, };
static int mt7622_ir_1_tx_pins[] = { 59, };
static int mt7622_ir_1_tx_funcs[] = { 5, };
static int mt7622_ir_2_tx_pins[] = { 99, };
static int mt7622_ir_2_tx_funcs[] = { 3, };
static int mt7622_ir_0_rx_pins[] = { 17, };
static int mt7622_ir_0_rx_funcs[] = { 4, };
static int mt7622_ir_1_rx_pins[] = { 60, };
static int mt7622_ir_1_rx_funcs[] = { 5, };
static int mt7622_ir_2_rx_pins[] = { 100, };
static int mt7622_ir_2_rx_funcs[] = { 3, };

/* MDIO */
static int mt7622_mdc_mdio_pins[] = { 23, 24, };
static int mt7622_mdc_mdio_funcs[] = { 0, 0, };

/* PCIE */
static int mt7622_pcie0_0_waken_pins[] = { 14, };
static int mt7622_pcie0_0_waken_funcs[] = { 2, };
static int mt7622_pcie0_0_clkreq_pins[] = { 15, };
static int mt7622_pcie0_0_clkreq_funcs[] = { 2, };
static int mt7622_pcie0_1_waken_pins[] = { 79, };
static int mt7622_pcie0_1_waken_funcs[] = { 4, };
static int mt7622_pcie0_1_clkreq_pins[] = { 80, };
static int mt7622_pcie0_1_clkreq_funcs[] = { 4, };
static int mt7622_pcie1_0_waken_pins[] = { 14, };
static int mt7622_pcie1_0_waken_funcs[] = { 3, };
static int mt7622_pcie1_0_clkreq_pins[] = { 15, };
static int mt7622_pcie1_0_clkreq_funcs[] = { 3, };

static int mt7622_pcie0_pad_perst_pins[] = { 83, };
static int mt7622_pcie0_pad_perst_funcs[] = { 0, };
static int mt7622_pcie1_pad_perst_pins[] = { 84, };
static int mt7622_pcie1_pad_perst_funcs[] = { 0, };

/* PMIC bus */
static int mt7622_pmic_bus_pins[] = { 71, 72, };
static int mt7622_pmic_bus_funcs[] = { 0, 0, };

/* Parallel NAND */
static int mt7622_pnand_pins[] = { 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47,
                                   48, 49, 50, };
static int mt7622_pnand_funcs[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                    0, };

/* PWM */
static int mt7622_pwm_ch1_0_pins[] = { 51, };
static int mt7622_pwm_ch1_0_funcs[] = { 3, };
static int mt7622_pwm_ch1_1_pins[] = { 73, };
static int mt7622_pwm_ch1_1_funcs[] = { 4, };
static int mt7622_pwm_ch1_2_pins[] = { 95, };
static int mt7622_pwm_ch1_2_funcs[] = { 0, };
static int mt7622_pwm_ch2_0_pins[] = { 52, };
static int mt7622_pwm_ch2_0_funcs[] = { 3, };
static int mt7622_pwm_ch2_1_pins[] = { 74, };
static int mt7622_pwm_ch2_1_funcs[] = { 4, };
static int mt7622_pwm_ch2_2_pins[] = { 96, };
static int mt7622_pwm_ch2_2_funcs[] = { 0, };
static int mt7622_pwm_ch3_0_pins[] = { 53, };
static int mt7622_pwm_ch3_0_funcs[] = { 3, };
static int mt7622_pwm_ch3_1_pins[] = { 75, };
static int mt7622_pwm_ch3_1_funcs[] = { 4, };
static int mt7622_pwm_ch3_2_pins[] = { 97, };
static int mt7622_pwm_ch3_2_funcs[] = { 0, };
static int mt7622_pwm_ch4_0_pins[] = { 54, };
static int mt7622_pwm_ch4_0_funcs[] = { 3, };
static int mt7622_pwm_ch4_1_pins[] = { 67, };
static int mt7622_pwm_ch4_1_funcs[] = { 3, };
static int mt7622_pwm_ch4_2_pins[] = { 76, };
static int mt7622_pwm_ch4_2_funcs[] = { 4, };
static int mt7622_pwm_ch4_3_pins[] = { 98, };
static int mt7622_pwm_ch4_3_funcs[] = { 0, };
static int mt7622_pwm_ch5_0_pins[] = { 68, };
static int mt7622_pwm_ch5_0_funcs[] = { 3, };
static int mt7622_pwm_ch5_1_pins[] = { 77, };
static int mt7622_pwm_ch5_1_funcs[] = { 4, };
static int mt7622_pwm_ch5_2_pins[] = { 99, };
static int mt7622_pwm_ch5_2_funcs[] = { 0, };
static int mt7622_pwm_ch6_0_pins[] = { 69, };
static int mt7622_pwm_ch6_0_funcs[] = { 3, };
static int mt7622_pwm_ch6_1_pins[] = { 78, };
static int mt7622_pwm_ch6_1_funcs[] = { 4, };
static int mt7622_pwm_ch6_2_pins[] = { 81, };
static int mt7622_pwm_ch6_2_funcs[] = { 4, };
static int mt7622_pwm_ch6_3_pins[] = { 100, };
static int mt7622_pwm_ch6_3_funcs[] = { 0, };

/* SD */
static int mt7622_sd_0_pins[] = { 16, 17, 18, 19, 20, 21, };
static int mt7622_sd_0_funcs[] = { 2, 2, 2, 2, 2, 2, };
static int mt7622_sd_1_pins[] = { 25, 26, 27, 28, 29, 30, };
static int mt7622_sd_1_funcs[] = { 2, 2, 2, 2, 2, 2, };

/* Serial NAND */
static int mt7622_snfi_pins[] = { 8, 9, 10, 11, 12, 13, };
static int mt7622_snfi_funcs[] = { 2, 2, 2, 2, 2, 2, };

/* SPI NOR */
static int mt7622_spi_pins[] = { 8, 9, 10, 11, 12, 13 };
static int mt7622_spi_funcs[] = { 0, 0, 0, 0, 0, 0, };

/* SPIC */
static int mt7622_spic0_0_pins[] = { 63, 64, 65, 66, };
static int mt7622_spic0_0_funcs[] = { 4, 4, 4, 4, };
static int mt7622_spic0_1_pins[] = { 79, 80, 81, 82, };
static int mt7622_spic0_1_funcs[] = { 3, 3, 3, 3, };
static int mt7622_spic1_0_pins[] = { 67, 68, 69, 70, };
static int mt7622_spic1_0_funcs[] = { 4, 4, 4, 4, };
static int mt7622_spic1_1_pins[] = { 73, 74, 75, 76, };
static int mt7622_spic1_1_funcs[] = { 0, 0, 0, 0, };
static int mt7622_spic2_0_pins[] = { 10, 11, 12, 13, };
static int mt7622_spic2_0_funcs[] = { 0, 0, 0, 0, };
static int mt7622_spic2_0_wp_hold_pins[] = { 8, 9, };
static int mt7622_spic2_0_wp_hold_funcs[] = { 0, 0, };

/* TDM */
static int mt7622_tdm_0_out_mclk_bclk_ws_pins[] = { 8, 9, 10, };
static int mt7622_tdm_0_out_mclk_bclk_ws_funcs[] = { 3, 3, 3, };
static int mt7622_tdm_0_in_mclk_bclk_ws_pins[] = { 11, 12, 13, };
static int mt7622_tdm_0_in_mclk_bclk_ws_funcs[] = { 3, 3, 3, };
static int mt7622_tdm_0_out_data_pins[] = { 20, };
static int mt7622_tdm_0_out_data_funcs[] = { 3, };
static int mt7622_tdm_0_in_data_pins[] = { 21, };
static int mt7622_tdm_0_in_data_funcs[] = { 3, };
static int mt7622_tdm_1_out_mclk_bclk_ws_pins[] = { 57, 58, 59, };
static int mt7622_tdm_1_out_mclk_bclk_ws_funcs[] = { 3, 3, 3, };
static int mt7622_tdm_1_in_mclk_bclk_ws_pins[] = { 60, 61, 62, };
static int mt7622_tdm_1_in_mclk_bclk_ws_funcs[] = { 3, 3, 3, };
static int mt7622_tdm_1_out_data_pins[] = { 55, };
static int mt7622_tdm_1_out_data_funcs[] = { 3, };
static int mt7622_tdm_1_in_data_pins[] = { 56, };
static int mt7622_tdm_1_in_data_funcs[] = { 3, };

/* UART */
static int mt7622_uart0_0_tx_rx_pins[] = { 6, 7, };
static int mt7622_uart0_0_tx_rx_funcs[] = { 0, 0, };
static int mt7622_uart1_0_tx_rx_pins[] = { 55, 56, };
static int mt7622_uart1_0_tx_rx_funcs[] = { 2, 2, };
static int mt7622_uart1_0_rts_cts_pins[] = { 57, 58, };
static int mt7622_uart1_0_rts_cts_funcs[] = { 2, 2, };
static int mt7622_uart1_1_tx_rx_pins[] = { 73, 74, };
static int mt7622_uart1_1_tx_rx_funcs[] = { 2, 2, };
static int mt7622_uart1_1_rts_cts_pins[] = { 75, 76, };
static int mt7622_uart1_1_rts_cts_funcs[] = { 2, 2, };
static int mt7622_uart2_0_tx_rx_pins[] = { 3, 4, };
static int mt7622_uart2_0_tx_rx_funcs[] = { 2, 2, };
static int mt7622_uart2_0_rts_cts_pins[] = { 1, 2, };
static int mt7622_uart2_0_rts_cts_funcs[] = { 2, 2, };
static int mt7622_uart2_1_tx_rx_pins[] = { 51, 52, };
static int mt7622_uart2_1_tx_rx_funcs[] = { 0, 0, };
static int mt7622_uart2_1_rts_cts_pins[] = { 53, 54, };
static int mt7622_uart2_1_rts_cts_funcs[] = { 0, 0, };
static int mt7622_uart2_2_tx_rx_pins[] = { 59, 60, };
static int mt7622_uart2_2_tx_rx_funcs[] = { 4, 4, };
static int mt7622_uart2_2_rts_cts_pins[] = { 61, 62, };
static int mt7622_uart2_2_rts_cts_funcs[] = { 4, 4, };
static int mt7622_uart2_3_tx_rx_pins[] = { 95, 96, };
static int mt7622_uart2_3_tx_rx_funcs[] = { 3, 3, };
static int mt7622_uart3_0_tx_rx_pins[] = { 57, 58, };
static int mt7622_uart3_0_tx_rx_funcs[] = { 5, 5, };
static int mt7622_uart3_1_tx_rx_pins[] = { 81, 82, };
static int mt7622_uart3_1_tx_rx_funcs[] = { 0, 0, };
static int mt7622_uart3_1_rts_cts_pins[] = { 79, 80, };
static int mt7622_uart3_1_rts_cts_funcs[] = { 0, 0, };
static int mt7622_uart4_0_tx_rx_pins[] = { 61, 62, };
static int mt7622_uart4_0_tx_rx_funcs[] = { 5, 5, };
static int mt7622_uart4_1_tx_rx_pins[] = { 91, 92, };
static int mt7622_uart4_1_tx_rx_funcs[] = { 0, 0, };
static int mt7622_uart4_1_rts_cts_pins[] = { 93, 94 };
static int mt7622_uart4_1_rts_cts_funcs[] = { 0, 0, };
static int mt7622_uart4_2_tx_rx_pins[] = { 97, 98, };
static int mt7622_uart4_2_tx_rx_funcs[] = { 2, 2, };
static int mt7622_uart4_2_rts_cts_pins[] = { 95, 96 };
static int mt7622_uart4_2_rts_cts_funcs[] = { 2, 2, };

/* Watchdog */
static int mt7622_watchdog_pins[] = { 78, };
static int mt7622_watchdog_funcs[] = { 0, };

/* WLAN LED */
static int mt7622_wled_pins[] = { 85, };
static int mt7622_wled_funcs[] = { 0, };

static const struct mtk_pin_group mt7622_groups[] = {
        PINCTRL_PIN_GROUP("antsel0", mt7622_antsel0),
        PINCTRL_PIN_GROUP("antsel1", mt7622_antsel1),
        PINCTRL_PIN_GROUP("antsel2", mt7622_antsel2),
        PINCTRL_PIN_GROUP("antsel3", mt7622_antsel3),
        PINCTRL_PIN_GROUP("antsel4", mt7622_antsel4),
        PINCTRL_PIN_GROUP("antsel5", mt7622_antsel5),
        PINCTRL_PIN_GROUP("antsel6", mt7622_antsel6),
        PINCTRL_PIN_GROUP("antsel7", mt7622_antsel7),
        PINCTRL_PIN_GROUP("antsel8", mt7622_antsel8),
        PINCTRL_PIN_GROUP("antsel9", mt7622_antsel9),
        PINCTRL_PIN_GROUP("antsel10", mt7622_antsel10),
        PINCTRL_PIN_GROUP("antsel11", mt7622_antsel11),
        PINCTRL_PIN_GROUP("antsel12", mt7622_antsel12),
        PINCTRL_PIN_GROUP("antsel13", mt7622_antsel13),
        PINCTRL_PIN_GROUP("antsel14", mt7622_antsel14),
        PINCTRL_PIN_GROUP("antsel15", mt7622_antsel15),
        PINCTRL_PIN_GROUP("antsel16", mt7622_antsel16),
        PINCTRL_PIN_GROUP("antsel17", mt7622_antsel17),
        PINCTRL_PIN_GROUP("antsel18", mt7622_antsel18),
        PINCTRL_PIN_GROUP("antsel19", mt7622_antsel19),
        PINCTRL_PIN_GROUP("antsel20", mt7622_antsel20),
        PINCTRL_PIN_GROUP("antsel21", mt7622_antsel21),
        PINCTRL_PIN_GROUP("antsel22", mt7622_antsel22),
        PINCTRL_PIN_GROUP("antsel23", mt7622_antsel23),
        PINCTRL_PIN_GROUP("antsel24", mt7622_antsel24),
        PINCTRL_PIN_GROUP("antsel25", mt7622_antsel25),
        PINCTRL_PIN_GROUP("antsel26", mt7622_antsel26),
        PINCTRL_PIN_GROUP("antsel27", mt7622_antsel27),
        PINCTRL_PIN_GROUP("antsel28", mt7622_antsel28),
        PINCTRL_PIN_GROUP("antsel29", mt7622_antsel29),
        PINCTRL_PIN_GROUP("emmc", mt7622_emmc),
        PINCTRL_PIN_GROUP("emmc_rst", mt7622_emmc_rst),
        PINCTRL_PIN_GROUP("ephy_leds", mt7622_ephy_leds),
        PINCTRL_PIN_GROUP("ephy0_led", mt7622_ephy0_led),
        PINCTRL_PIN_GROUP("ephy1_led", mt7622_ephy1_led),
        PINCTRL_PIN_GROUP("ephy2_led", mt7622_ephy2_led),
        PINCTRL_PIN_GROUP("ephy3_led", mt7622_ephy3_led),
        PINCTRL_PIN_GROUP("ephy4_led", mt7622_ephy4_led),
        PINCTRL_PIN_GROUP("esw", mt7622_esw),
        PINCTRL_PIN_GROUP("esw_p0_p1", mt7622_esw_p0_p1),
        PINCTRL_PIN_GROUP("esw_p2_p3_p4", mt7622_esw_p2_p3_p4),
        PINCTRL_PIN_GROUP("rgmii_via_esw", mt7622_rgmii_via_esw),
        PINCTRL_PIN_GROUP("rgmii_via_gmac1", mt7622_rgmii_via_gmac1),
        PINCTRL_PIN_GROUP("rgmii_via_gmac2", mt7622_rgmii_via_gmac2),
        PINCTRL_PIN_GROUP("i2c0", mt7622_i2c0),
        PINCTRL_PIN_GROUP("i2c1_0", mt7622_i2c1_0),
        PINCTRL_PIN_GROUP("i2c1_1", mt7622_i2c1_1),
        PINCTRL_PIN_GROUP("i2c1_2", mt7622_i2c1_2),
        PINCTRL_PIN_GROUP("i2c2_0", mt7622_i2c2_0),
        PINCTRL_PIN_GROUP("i2c2_1", mt7622_i2c2_1),
        PINCTRL_PIN_GROUP("i2c2_2", mt7622_i2c2_2),
        PINCTRL_PIN_GROUP("i2s_out_mclk_bclk_ws", mt7622_i2s_out_mclk_bclk_ws),
        PINCTRL_PIN_GROUP("i2s_in_mclk_bclk_ws", mt7622_i2s_in_mclk_bclk_ws),
        PINCTRL_PIN_GROUP("i2s1_in_data", mt7622_i2s1_in_data),
        PINCTRL_PIN_GROUP("i2s2_in_data", mt7622_i2s2_in_data),
        PINCTRL_PIN_GROUP("i2s3_in_data", mt7622_i2s3_in_data),
        PINCTRL_PIN_GROUP("i2s4_in_data", mt7622_i2s4_in_data),
        PINCTRL_PIN_GROUP("i2s1_out_data", mt7622_i2s1_out_data),
        PINCTRL_PIN_GROUP("i2s2_out_data", mt7622_i2s2_out_data),
        PINCTRL_PIN_GROUP("i2s3_out_data", mt7622_i2s3_out_data),
        PINCTRL_PIN_GROUP("i2s4_out_data", mt7622_i2s4_out_data),
        PINCTRL_PIN_GROUP("ir_0_tx", mt7622_ir_0_tx),
        PINCTRL_PIN_GROUP("ir_1_tx", mt7622_ir_1_tx),
        PINCTRL_PIN_GROUP("ir_2_tx", mt7622_ir_2_tx),
        PINCTRL_PIN_GROUP("ir_0_rx", mt7622_ir_0_rx),
        PINCTRL_PIN_GROUP("ir_1_rx", mt7622_ir_1_rx),
        PINCTRL_PIN_GROUP("ir_2_rx", mt7622_ir_2_rx),
        PINCTRL_PIN_GROUP("mdc_mdio", mt7622_mdc_mdio),
        PINCTRL_PIN_GROUP("pcie0_0_waken", mt7622_pcie0_0_waken),
        PINCTRL_PIN_GROUP("pcie0_0_clkreq", mt7622_pcie0_0_clkreq),
        PINCTRL_PIN_GROUP("pcie0_1_waken", mt7622_pcie0_1_waken),
        PINCTRL_PIN_GROUP("pcie0_1_clkreq", mt7622_pcie0_1_clkreq),
        PINCTRL_PIN_GROUP("pcie1_0_waken", mt7622_pcie1_0_waken),
        PINCTRL_PIN_GROUP("pcie1_0_clkreq", mt7622_pcie1_0_clkreq),
        PINCTRL_PIN_GROUP("pcie0_pad_perst", mt7622_pcie0_pad_perst),
        PINCTRL_PIN_GROUP("pcie1_pad_perst", mt7622_pcie1_pad_perst),
        PINCTRL_PIN_GROUP("par_nand", mt7622_pnand),
        PINCTRL_PIN_GROUP("pmic_bus", mt7622_pmic_bus),
        PINCTRL_PIN_GROUP("pwm_ch1_0", mt7622_pwm_ch1_0),
        PINCTRL_PIN_GROUP("pwm_ch1_1", mt7622_pwm_ch1_1),
        PINCTRL_PIN_GROUP("pwm_ch1_2", mt7622_pwm_ch1_2),
        PINCTRL_PIN_GROUP("pwm_ch2_0", mt7622_pwm_ch2_0),
        PINCTRL_PIN_GROUP("pwm_ch2_1", mt7622_pwm_ch2_1),
        PINCTRL_PIN_GROUP("pwm_ch2_2", mt7622_pwm_ch2_2),
        PINCTRL_PIN_GROUP("pwm_ch3_0", mt7622_pwm_ch3_0),
        PINCTRL_PIN_GROUP("pwm_ch3_1", mt7622_pwm_ch3_1),
        PINCTRL_PIN_GROUP("pwm_ch3_2", mt7622_pwm_ch3_2),
        PINCTRL_PIN_GROUP("pwm_ch4_0", mt7622_pwm_ch4_0),
        PINCTRL_PIN_GROUP("pwm_ch4_1", mt7622_pwm_ch4_1),
        PINCTRL_PIN_GROUP("pwm_ch4_2", mt7622_pwm_ch4_2),
        PINCTRL_PIN_GROUP("pwm_ch4_3", mt7622_pwm_ch4_3),
        PINCTRL_PIN_GROUP("pwm_ch5_0", mt7622_pwm_ch5_0),
        PINCTRL_PIN_GROUP("pwm_ch5_1", mt7622_pwm_ch5_1),
        PINCTRL_PIN_GROUP("pwm_ch5_2", mt7622_pwm_ch5_2),
        PINCTRL_PIN_GROUP("pwm_ch6_0", mt7622_pwm_ch6_0),
        PINCTRL_PIN_GROUP("pwm_ch6_1", mt7622_pwm_ch6_1),
        PINCTRL_PIN_GROUP("pwm_ch6_2", mt7622_pwm_ch6_2),
        PINCTRL_PIN_GROUP("pwm_ch6_3", mt7622_pwm_ch6_3),
        PINCTRL_PIN_GROUP("sd_0", mt7622_sd_0),
        PINCTRL_PIN_GROUP("sd_1", mt7622_sd_1),
        PINCTRL_PIN_GROUP("snfi", mt7622_snfi),
        PINCTRL_PIN_GROUP("spi_nor", mt7622_spi),
        PINCTRL_PIN_GROUP("spic0_0", mt7622_spic0_0),
        PINCTRL_PIN_GROUP("spic0_1", mt7622_spic0_1),
        PINCTRL_PIN_GROUP("spic1_0", mt7622_spic1_0),
        PINCTRL_PIN_GROUP("spic1_1", mt7622_spic1_1),
        PINCTRL_PIN_GROUP("spic2_0", mt7622_spic2_0),
        PINCTRL_PIN_GROUP("spic2_0_wp_hold", mt7622_spic2_0_wp_hold),
        PINCTRL_PIN_GROUP("tdm_0_out_mclk_bclk_ws",
                          mt7622_tdm_0_out_mclk_bclk_ws),
        PINCTRL_PIN_GROUP("tdm_0_in_mclk_bclk_ws",
                          mt7622_tdm_0_in_mclk_bclk_ws),
        PINCTRL_PIN_GROUP("tdm_0_out_data",  mt7622_tdm_0_out_data),
        PINCTRL_PIN_GROUP("tdm_0_in_data", mt7622_tdm_0_in_data),
        PINCTRL_PIN_GROUP("tdm_1_out_mclk_bclk_ws",
                          mt7622_tdm_1_out_mclk_bclk_ws),
        PINCTRL_PIN_GROUP("tdm_1_in_mclk_bclk_ws",
                          mt7622_tdm_1_in_mclk_bclk_ws),
        PINCTRL_PIN_GROUP("tdm_1_out_data",  mt7622_tdm_1_out_data),
        PINCTRL_PIN_GROUP("tdm_1_in_data", mt7622_tdm_1_in_data),
        PINCTRL_PIN_GROUP("uart0_0_tx_rx", mt7622_uart0_0_tx_rx),
        PINCTRL_PIN_GROUP("uart1_0_tx_rx", mt7622_uart1_0_tx_rx),
        PINCTRL_PIN_GROUP("uart1_0_rts_cts", mt7622_uart1_0_rts_cts),
        PINCTRL_PIN_GROUP("uart1_1_tx_rx", mt7622_uart1_1_tx_rx),
        PINCTRL_PIN_GROUP("uart1_1_rts_cts", mt7622_uart1_1_rts_cts),
        PINCTRL_PIN_GROUP("uart2_0_tx_rx", mt7622_uart2_0_tx_rx),
        PINCTRL_PIN_GROUP("uart2_0_rts_cts", mt7622_uart2_0_rts_cts),
        PINCTRL_PIN_GROUP("uart2_1_tx_rx", mt7622_uart2_1_tx_rx),
        PINCTRL_PIN_GROUP("uart2_1_rts_cts", mt7622_uart2_1_rts_cts),
        PINCTRL_PIN_GROUP("uart2_2_tx_rx", mt7622_uart2_2_tx_rx),
        PINCTRL_PIN_GROUP("uart2_2_rts_cts", mt7622_uart2_2_rts_cts),
        PINCTRL_PIN_GROUP("uart2_3_tx_rx", mt7622_uart2_3_tx_rx),
        PINCTRL_PIN_GROUP("uart3_0_tx_rx", mt7622_uart3_0_tx_rx),
        PINCTRL_PIN_GROUP("uart3_1_tx_rx", mt7622_uart3_1_tx_rx),
        PINCTRL_PIN_GROUP("uart3_1_rts_cts", mt7622_uart3_1_rts_cts),
        PINCTRL_PIN_GROUP("uart4_0_tx_rx", mt7622_uart4_0_tx_rx),
        PINCTRL_PIN_GROUP("uart4_1_tx_rx", mt7622_uart4_1_tx_rx),
        PINCTRL_PIN_GROUP("uart4_1_rts_cts", mt7622_uart4_1_rts_cts),
        PINCTRL_PIN_GROUP("uart4_2_tx_rx", mt7622_uart4_2_tx_rx),
        PINCTRL_PIN_GROUP("uart4_2_rts_cts", mt7622_uart4_2_rts_cts),
        PINCTRL_PIN_GROUP("watchdog", mt7622_watchdog),
        PINCTRL_PIN_GROUP("wled", mt7622_wled),
};

const struct mtk_padconf mt7622_padconf = {
	.pins_names = mt7622_pins,
	.npins = sizeof(mt7622_pins) / sizeof(struct mtk_pin_desc),
	.pin_reg = mt7622_reg_cals,
	.pins_group = mt7622_groups,
	.npins_group = sizeof(mt7622_groups) / sizeof(struct mtk_pin_group),
};
