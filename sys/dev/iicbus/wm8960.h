/*-
 * Copyright (c) 2020 - 2021 Priit Trees <trees@neti.ee>
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

#ifndef __WM8960_H__
#define __WM8960_H__

enum {
	R0,			// Left Input volume
#define R0_IPVU			1 << 8
#define R0_LINMUTE		1 << 7
#define R0_LIZC			1 << 6
#define R0_LINVOL		0x3f << 0
	R1,			// Right Input volume
#define R1_IPVU			1 << 8
#define R1_RINMUTE		1 << 7
#define R1_RIZC			1 << 6
#define R1_RINVOL		0x3f << 0
	R2,			// LOUT1 volume
#define R2_OUT1VU		1 << 8
#define R2_LO1ZC		1 << 7
#define R2_LOUT1VOL		0x7f << 0
	R3,			// ROUT1 volume
#define R3_OUT1VU		1 << 8
#define R3_RO1ZC		1 << 7
#define R3_ROUT1VOL		0x7f << 0
	R4,			// Clocking (1)
#define R4_ADCDIV		0x07 << 6
#define R4_ADCDIV_SYSCLK_1	0x00 << 6
#define R4_ADCDIV_SYSCLK_1_5	0x01 << 6
#define R4_ADCDIV_SYSCLK_2	0x02 << 6
#define R4_ADCDIV_SYSCLK_3	0x03 << 6
#define R4_ADCDIV_SYSCLK_4	0x04 << 6
#define R4_ADCDIV_SYSCLK_5_5	0x05 << 6
#define R4_ADCDIV_SYSCLK_6	0x06 << 6
#define R4_DACDIV		0x07 << 3
#define R4_DACDIV_SYSCLK_1	0x00 << 6
#define R4_DACDIV_SYSCLK_1_5	0x01 << 6
#define R4_DACDIV_SYSCLK_2	0x02 << 6
#define R4_DACDIV_SYSCLK_3	0x03 << 6
#define R4_DACDIV_SYSCLK_4	0x04 << 6
#define R4_DACDIV_SYSCLK_5_5	0x05 << 6
#define R4_DACDIV_SYSCLK_6	0x06 << 6
#define R4_SYSCLKDIV		0x03 << 1
#define R4_SYSCLKDIV_SYSCLK_1	0x00 << 1
#define R4_SYSCLKDIV_SYSCLK_2	0x02 << 1
#define R4_CLKSEL		1 << 0
	R5,			// ADC & DAC Control (CTR1)
#define R5_DACDIV2		1 << 7
#define R5_ADCPOL		0x03 << 5
#define R5_ADCPOL_NI		0x00
#define R5_ADCPOL_LI		0x01
#define R5_ADCPOL_RI		0x02
#define R5_ADCPOL_LRI		0x03
#define R5_DACMU		1 << 3
#define R5_DEEMPH		0x03 << 1
#define R5_DEEMPH_48000		0x03
#define R5_DEEMPH_44100		0x02
#define R5_DEEMPH_32000		0x01
#define R5_DEEMPH_NO		0x00
#define R5_ADCHPD		1 << 0
	R6,			// ADC & DAC Control (CTR2)
#define R6_DACPOL		0x03 << 5
#define R6_DACPOL_NI		0x00
#define R6_DACPOL_LI		0x01
#define R6_DACPOL_RI		0x02
#define R6_DACPOL_LRI		0x03
#define R6_DACSMM		1 << 3
#define R6_DACMR		1 << 2
#define R6_DACSLOPE		1 << 1
	R7,			// Audio Interface
#define R7_ALRSWAP		1 << 8
#define R7_BCLKINV		1 << 7
#define R7_MS			1 << 6
#define R7_DLRSWAP		1 << 5
#define R7_LRP			1 << 4
#define R7_WL			0x03 << 2
#define R7_WL_16		0x00 << 2
#define R7_WL_20		0x01 << 2
#define R7_WL_24		0x02 << 2
#define R7_WL_32		0x03 << 2
#define R7_FORMAT		0x03 << 0
#define R7_FORMAT_RJ		0x00 << 0
#define R7_FORMAT_LJ		0x01 << 0
#define R7_FORMAT_I2S 		0x02 << 0
#define R7_FORMAT_DSP		0x03 << 0
	R8,			// Clocking (2)
#define R8_DCLKDIV		0x07 << 6
#define R8_DCLKDIV_SYSCLK_1_5 	0x00 << 6
#define R8_DCLKDIV_SYSCLK_2	0x01 << 6
#define R8_DCLKDIV_SYSCLK_3	0x02 << 6
#define R8_DCLKDIV_SYSCLK_4	0x03 << 6
#define R8_DCLKDIV_SYSCLK_6	0x04 << 6
#define R8_DCLKDIV_SYSCLK_8	0x05 << 6
#define R8_DCLKDIV_SYSCLK_12	0x06 << 6
#define R8_DCLKDIV_SYSCLK_16	0x07 << 6
#define R8_BCLKDIV		0x0F << 0
#define R8_BCLKDIV_SYSCLK	0x00 << 0
#define R8_BCLKDIV_SYSCLK_1_5	0x01 << 0
#define R8_BCLKDIV_SYSCLK_2	0x02 << 0
#define R8_BCLKDIV_SYSCLK_3	0x03 << 0
#define R8_BCLKDIV_SYSCLK_4	0x04 << 0
#define R8_BCLKDIV_SYSCLK_5_5	0x05 << 0
#define R8_BCLKDIV_SYSCLK_6	0x06 << 0
#define R8_BCLKDIV_SYSCLK_8	0x07 << 0
#define R8_BCLKDIV_SYSCLK_11	0x08 << 0
#define R8_BCLKDIV_SYSCLK_12	0x09 << 0
#define R8_BCLKDIV_SYSCLK_16	0x0a << 0
#define R8_BCLKDIV_SYSCLK_22	0x0b << 0
#define R8_BCLKDIV_SYSCLK_24	0x0c << 0
#define R8_BCLKDIV_SYSCLK_32	0x0d << 0
//#define R8_BCLKDIV_SYSCLK_32	0x0e << 0
//#define R8_BCLKDIV_SYSCLK_32	0x0f << 0
	R9,			// Audio Interface
#define R9_ALRCGPIO		1 << 6
#define R9_WL8			1 << 5
#define R9_DACCOMP		0x03 << 3
#define R9_DACCOMP_OFF		0x00
#define R9_DACCOMP_U_LAW	0x02
#define R9_DACCOMP_A_LAW	0x03
#define R9_ADCCOMP		0x03 << 3
#define R9_ADCCOMP_OFF		0x00
#define R9_ADCCOMP_U_LAW	0x02
#define R9_ADCCOMP_A_LAW	0x03
#define R9_LOOPBACK		1 << 0
	R10,			// Left DAC volume
#define R10_DACVU		1 << 8
#define R10_LDACVOL		0x7F << 0
	R11,			// Right DAC volume
#define R11_DACVU		1 << 8
#define R11_RDACVOL		0x7F << 0
	R12,			// Reserved
	R13,			// Reserved
	R14,			// Reserved
	R15,			// Reset
	R16,			// 3D control
#define R16_3DUC		1 << 6
#define R16_3DLC		1 << 5
#define R16_3DDEPTH		0x0F << 1
#define R16_3DEN		1 << 0
	R17,			// ALC1
#define R17_ALCSEL		0x03 << 7
#define R17_ALCSEL_ALC_OFF	0x00
#define R17_ALCSEL_RCHAN	0x01
#define R17_ALCSEL_LCHAN	0x02
#define R17_ALCSEL_STEREO	0x03
#define R17_MAXGAIN		0x07 << 4
#define R17_ALCL		0x0F << 0
	R18,			// ALC2
#define R18_MINGAIN		0x7 << 4
#define R18_MINGAIN_N17DOT25	0x0
#define R18_MINGAIN_N11DOT25	0x1
#define R18_MINGAIN_N5DOT25	0x2
#define R18_MINGAIN_P0DOT75	0x3
#define R18_MINGAIN_P6DOT75	0x4
#define R18_MINGAIN_P12DOT75	0x5
#define R18_MINGAIN_P18DOT75	0x6
#define R18_MINGAIN_P24DOT75	0x7
#define R18_HLD			0xF << 0
	R19,			// ALC3
#define R19_ALCMODE		1 << 8
#define R19_DCY			0x0F << 4
#define R19_ATK			0x0F << 0
	R20,			// Noise Gate
#define R20_NGTH		0x1F << 3
#define R20_NGAT		1 << 0
	R21,			// Left ADC volume
#define R21_ADCVU		1 << 8
#define R21_LADCVOL		0x7F << 0
	R22,			// Right ADC volume
#define R22_ADCVU		1 << 8
#define R22_RADCVOL		0x7F << 0
	R23,			// Additional control(1)
#define R23_TSDEN		1 << 8
#define R23_VSEL		0x03 << 6
#define R23_VSEL_AVDD_2dot7	0x01
#define R23_VSEL_AVDD_3dot3	0x02
//#define R23_VSEL_AVDD_2dot7	0x03
#define R23_DMONOMIX		1 << 4
#define R23_DATSEL		0x03 << 2
#define R23_DATSEL_LEFT_RIGHT	0x00
#define R23_DATSEL_LEFT_LEFT	0x01
#define R23_DATSEL_RIGHT_RIGHT	0x02
#define R23_DATSEL_RIGHT_LEFT	0x03
#define R23_TOCLKSEL		1 << 1
#define R23_TOEN		1 << 0
	R24,			// Additional control(2)
#define R24_HPSWEN		1 << 6
#define R24_HPSWPOL		1 << 5
#define R24_TRIS		1 << 3
#define R24_LRCM		1 << 2
	R25,			// Pwr Mgmt (1)
#define R25_VMIDSEL		0x03 << 7
#define R25_VREF		1 << 6
#define R25_AINL		1 << 5
#define R25_AINR		1 << 4
#define R25_ADCL		1 << 3
#define R25_ADCR		1 << 2
#define R25_MICB		1 << 1
#define R25_DIGENB		1 << 0
	R26,			// Pwr Mgmt (2)
#define R26_DACL		1 << 8
#define R26_DACR		1 << 7
#define R26_LOUT1		1 << 6
#define R26_ROUT1		1 << 5
#define R26_SPKL		1 << 4
#define R26_SPKR		1 << 3
#define R26_OUT3		1 << 1
#define R26_PLL_EN		1 << 0
	R27,			// Additional Control (3)
#define R27_VROI		1 << 6
#define R27_OUT3CAP		1 << 3
#define R27_ADC_ALC_SR		0x07 << 0
#define R27_ADC_ALC_SR_44100_48000	0x00
#define R27_ADC_ALC_SR_32000	0x01
#define R27_ADC_ALC_SR_22050_24000	0x02
#define R27_ADC_ALC_SR_16000	0x03
#define R27_ADC_ALC_SR_11250_12000	0x04
#define R27_ADC_ALC_SR_8000	0x05
	R28,			// Anti-pop 1
#define R28_POBCTRL		1 << 7
#define R28_BUFDCOPEN		1 << 4
#define R28_BUFIOEN		1 << 3
#define R28_SOFT_ST		1 << 2
#define R28_HPSTBY		1 << 0
	R29,			// Anti-pop 2
#define R29_DISOP		1 << 6
#define R29_DRES		0x03 << 4
#define R29_DRES_400		0x00
#define R29_DRES_200		0x01
#define R29_DRES_600		0x02
#define R29_DRES_150		0x03
	R30,			// Reserved
	R31,			// Reserved
	R32,			// ADCL signal path
#define R32_LMN1		1 << 8
#define R32_LMP3		1 << 7
#define R32_LMP2		1 << 6
#define R32_LMICBOOST		0x03 << 4
#define R32_LMICBOOST_P0	0x0
#define R32_LMICBOOST_P13	0x1
#define R32_LMICBOOST_P20	0x2
#define R32_LMICBOOST_P29	0x3
#define R32_LMIC2B		1 << 3
	R33,			// ADCR signal path
#define R33_RMN1		1 << 8
#define R33_RMP3		1 << 7
#define R33_RMP2		1 << 6
#define R33_RMICBOOST		0x03 << 4
#define R33_RMICBOOST_P0	0x00
#define R33_RMICBOOST_P13	0x01
#define R33_RMICBOOST_P20	0x02
#define R33_RMICBOOST_P29	0x03
#define R33_RMIC2B		1 << 3
	R34,			// Left out Mix (1)
#define R34_LD2LO		1 << 8
#define R34_LI2LO		1 << 7
#define R34_LI2LOVOL		0x07 << 4
	R35,			// Reserved
	R36,			// Reserved
	R37,			// Right out Mix (2)
#define R37_RD2RO		1 << 8
#define R37_RI2RO		1 << 7
#define R37_RI2ROVOL		0x07 << 4
	R38,			// Mono out Mix (1)
#define R38_L2MO		1 << 7
	R39,			// Mono out Mix (2)
#define R39_R2MO		1 << 7
	R40,			// LOUT2 volume
#define R40_SPKVU		1 << 8
#define R40_SPKLZC		1 << 7
#define R40_SPKLVOL		0x7F << 6
	R41,			// ROUT2 volume
#define R41_SPKVU		1 << 8
#define R41_SPKRZC		1 << 7
#define R41_SPKRVOL		0x7F << 6
	R42,			// MONOOUT volume
#define R42_MOUTVOL 		1 << 6
	R43,			// Input boost mixer (1)
#define R43_LIN3BOOST		0x07 << 4
#define R43_LIN2BOOST		0x07 << 1
	R44,			// Input boost mixer (2)
#define R44_RIN3BOOST		0x07 << 4
#define R44_RIN2BOOST		0x07 << 1
	R45,			// Bypass (1)
#define R45_LB2LO		1 << 7
#define R45_LB2LOVOL		0x07 << 4
	R46,			// Bypass (2)
#define R46_RB2RO		1 << 7
#define R46_RB2ROVOL		0x07 << 4
	R47,			// Pwr Mgmt (3)
#define R47_LMIC		1 << 5
#define R47_RMIC		1 << 4
#define R47_LOMIX		1 << 3
#define R47_ROMIX		1 << 2
	R48,			// Additional Control (4)
#define R48_GPIOPOL		1 << 7
#define R48_GPIOSEL_MASK	0x07 << 4
#define R48_GPIOSEL_JACK_DETECT_INPUT	0x00 << 4
#define R48_GPIOSEL_TEMP_OK	0x2 << 4
#define R48_GPIOSEL_DEBOUNCED_JACKDETECT_OUTPUT 0x3 << 4
#define R48_GPIOSEL_SYSCLK_OUTOUT	0x4 << 4
#define R48_GPIOSEL_PLL_LOCK	0x5 << 4
#define R48_GPIOSEL_LOGIC_0	0x6 << 4
#define R48_GPIOSEL_LOGIC_1	0x7 << 4
#define R48_HPSEL_MASK		0x03 << 2
#define R48_HPSEL_GPIO1 	0x00 << 2
#define R48_HPSEL_JD2		0x02 << 2
#define R48_HPSEL_JD3		0x03 << 2
#define R48_TSENSEN		1 << 1
#define R48_MBSEL		1 << 0
	R49,			// Class D Control (1)
#define R49_SPK_OP_EN		0x03 <<	6
#define R49_SPK_OP_EN_OFF	0x00 << 6
#define R49_SPK_OP_EN_LSPK	0x01 << 6
#define R49_SPK_OP_EN_RSPK	0x02 << 6
#define R49_SPK_OP_EN_LRSPK	0x03 << 6
	R50,			// Reserved
	R51,			// Class D Control (3)
#define R51_DCGAIN		0x07 << 3
#define R51_DCGAIN_BOOST_0 	0x0
#define R51_DCGAIN_BOOST_2DOT1 	0x1
#define R51_DCGAIN_BOOST_2DOT9 	0x2
#define R51_DCGAIN_BOOST_3DOT6 	0x3
#define R51_DCGAIN_BOOST_4DOT5 	0x4
#define R51_DCGAIN_BOOST_5DOT1	0x5
#define R51_ACGAIN		0x07 << 0
#define R51_ACGAIN_BOOST_0 	0x0
#define R51_ACGAIN_BOOST_2DOT1 	0x1
#define R51_ACGAIN_BOOST_2DOT9 	0x2
#define R51_ACGAIN_BOOST_3DOT6 	0x3
#define R51_ACGAIN_BOOST_4DOT5 	0x4
#define R51_ACGAIN_BOOST_5DOT1	0x5
	R52,			// PLL N
#define R52_OPCLKDIV		0x07 << 6
#define R52_OPCLKDIV_SYSCLK	0x00 << 6
#define R52_OPCLKDIV_SYSCLK_2	0x01 << 6
#define R52_OPCLKDIV_SYSCLK_3	0x02 << 6
#define R52_OPCLKDIV_SYSCLK_4	0x03 << 6
#define R52_OPCLKDIV_SYSCLK_5_5	0x04 << 6
#define R52_OPCLKDIV_SYSCLK_6	0x05 << 6
#define R52_SDM			1 << 5
#define R52_PLLPRESCALE		1 << 4
#define R52_PLLN_3_0		0x0F << 0
	R53,			// PLL K 1
#define R53_PLLK_23_16
	R54,			// PLL K 2
#define R54_PLLK_15_8
	R55,			// PLL K 3
#define R55_PLLK_7_0
};

#endif
