// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2015-2017, The Linux Foundation.
// Copyright (c) 2019, Linaro Limited

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/soundwire/sdw.h>
#include <linux/soundwire/sdw_registers.h>
#include <linux/soundwire/sdw_type.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#define WSA881X_DIGITAL_BASE		0x3000
#define WSA881X_ANALOG_BASE		0x3100

/* Digital register address space */
#define WSA881X_CHIP_ID0			(WSA881X_DIGITAL_BASE + 0x0000)
#define WSA881X_CHIP_ID1			(WSA881X_DIGITAL_BASE + 0x0001)
#define WSA881X_CHIP_ID2			(WSA881X_DIGITAL_BASE + 0x0002)
#define WSA881X_CHIP_ID3			(WSA881X_DIGITAL_BASE + 0x0003)
#define WSA881X_BUS_ID				(WSA881X_DIGITAL_BASE + 0x0004)
#define WSA881X_CDC_RST_CTL			(WSA881X_DIGITAL_BASE + 0x0005)
#define WSA881X_CDC_TOP_CLK_CTL			(WSA881X_DIGITAL_BASE + 0x0006)
#define WSA881X_CDC_ANA_CLK_CTL			(WSA881X_DIGITAL_BASE + 0x0007)
#define WSA881X_CDC_DIG_CLK_CTL			(WSA881X_DIGITAL_BASE + 0x0008)
#define WSA881X_CLOCK_CONFIG			(WSA881X_DIGITAL_BASE + 0x0009)
#define WSA881X_ANA_CTL				(WSA881X_DIGITAL_BASE + 0x000A)
#define WSA881X_SWR_RESET_EN			(WSA881X_DIGITAL_BASE + 0x000B)
#define WSA881X_RESET_CTL			(WSA881X_DIGITAL_BASE + 0x000C)
#define WSA881X_TADC_VALUE_CTL			(WSA881X_DIGITAL_BASE + 0x000F)
#define WSA881X_TEMP_DETECT_CTL			(WSA881X_DIGITAL_BASE + 0x0010)
#define WSA881X_TEMP_MSB			(WSA881X_DIGITAL_BASE + 0x0011)
#define WSA881X_TEMP_LSB			(WSA881X_DIGITAL_BASE + 0x0012)
#define WSA881X_TEMP_CONFIG0			(WSA881X_DIGITAL_BASE + 0x0013)
#define WSA881X_TEMP_CONFIG1			(WSA881X_DIGITAL_BASE + 0x0014)
#define WSA881X_CDC_CLIP_CTL			(WSA881X_DIGITAL_BASE + 0x0015)
#define WSA881X_SDM_PDM9_LSB			(WSA881X_DIGITAL_BASE + 0x0016)
#define WSA881X_SDM_PDM9_MSB			(WSA881X_DIGITAL_BASE + 0x0017)
#define WSA881X_CDC_RX_CTL			(WSA881X_DIGITAL_BASE + 0x0018)
#define WSA881X_DEM_BYPASS_DATA0		(WSA881X_DIGITAL_BASE + 0x0019)
#define WSA881X_DEM_BYPASS_DATA1		(WSA881X_DIGITAL_BASE + 0x001A)
#define WSA881X_DEM_BYPASS_DATA2		(WSA881X_DIGITAL_BASE + 0x001B)
#define WSA881X_DEM_BYPASS_DATA3		(WSA881X_DIGITAL_BASE + 0x001C)
#define WSA881X_OTP_CTRL0			(WSA881X_DIGITAL_BASE + 0x001D)
#define WSA881X_OTP_CTRL1			(WSA881X_DIGITAL_BASE + 0x001E)
#define WSA881X_HDRIVE_CTL_GROUP1		(WSA881X_DIGITAL_BASE + 0x001F)
#define WSA881X_INTR_MODE			(WSA881X_DIGITAL_BASE + 0x0020)
#define WSA881X_INTR_MASK			(WSA881X_DIGITAL_BASE + 0x0021)
#define WSA881X_INTR_STATUS			(WSA881X_DIGITAL_BASE + 0x0022)
#define WSA881X_INTR_CLEAR			(WSA881X_DIGITAL_BASE + 0x0023)
#define WSA881X_INTR_LEVEL			(WSA881X_DIGITAL_BASE + 0x0024)
#define WSA881X_INTR_SET			(WSA881X_DIGITAL_BASE + 0x0025)
#define WSA881X_INTR_TEST			(WSA881X_DIGITAL_BASE + 0x0026)
#define WSA881X_PDM_TEST_MODE			(WSA881X_DIGITAL_BASE + 0x0030)
#define WSA881X_ATE_TEST_MODE			(WSA881X_DIGITAL_BASE + 0x0031)
#define WSA881X_PIN_CTL_MODE			(WSA881X_DIGITAL_BASE + 0x0032)
#define WSA881X_PIN_CTL_OE			(WSA881X_DIGITAL_BASE + 0x0033)
#define WSA881X_PIN_WDATA_IOPAD			(WSA881X_DIGITAL_BASE + 0x0034)
#define WSA881X_PIN_STATUS			(WSA881X_DIGITAL_BASE + 0x0035)
#define WSA881X_DIG_DEBUG_MODE			(WSA881X_DIGITAL_BASE + 0x0037)
#define WSA881X_DIG_DEBUG_SEL			(WSA881X_DIGITAL_BASE + 0x0038)
#define WSA881X_DIG_DEBUG_EN			(WSA881X_DIGITAL_BASE + 0x0039)
#define WSA881X_SWR_HM_TEST1			(WSA881X_DIGITAL_BASE + 0x003B)
#define WSA881X_SWR_HM_TEST2			(WSA881X_DIGITAL_BASE + 0x003C)
#define WSA881X_TEMP_DETECT_DBG_CTL		(WSA881X_DIGITAL_BASE + 0x003D)
#define WSA881X_TEMP_DEBUG_MSB			(WSA881X_DIGITAL_BASE + 0x003E)
#define WSA881X_TEMP_DEBUG_LSB			(WSA881X_DIGITAL_BASE + 0x003F)
#define WSA881X_SAMPLE_EDGE_SEL			(WSA881X_DIGITAL_BASE + 0x0044)
#define WSA881X_IOPAD_CTL			(WSA881X_DIGITAL_BASE + 0x0045)
#define WSA881X_SPARE_0				(WSA881X_DIGITAL_BASE + 0x0050)
#define WSA881X_SPARE_1				(WSA881X_DIGITAL_BASE + 0x0051)
#define WSA881X_SPARE_2				(WSA881X_DIGITAL_BASE + 0x0052)
#define WSA881X_OTP_REG_0			(WSA881X_DIGITAL_BASE + 0x0080)
#define WSA881X_OTP_REG_1			(WSA881X_DIGITAL_BASE + 0x0081)
#define WSA881X_OTP_REG_2			(WSA881X_DIGITAL_BASE + 0x0082)
#define WSA881X_OTP_REG_3			(WSA881X_DIGITAL_BASE + 0x0083)
#define WSA881X_OTP_REG_4			(WSA881X_DIGITAL_BASE + 0x0084)
#define WSA881X_OTP_REG_5			(WSA881X_DIGITAL_BASE + 0x0085)
#define WSA881X_OTP_REG_6			(WSA881X_DIGITAL_BASE + 0x0086)
#define WSA881X_OTP_REG_7			(WSA881X_DIGITAL_BASE + 0x0087)
#define WSA881X_OTP_REG_8			(WSA881X_DIGITAL_BASE + 0x0088)
#define WSA881X_OTP_REG_9			(WSA881X_DIGITAL_BASE + 0x0089)
#define WSA881X_OTP_REG_10			(WSA881X_DIGITAL_BASE + 0x008A)
#define WSA881X_OTP_REG_11			(WSA881X_DIGITAL_BASE + 0x008B)
#define WSA881X_OTP_REG_12			(WSA881X_DIGITAL_BASE + 0x008C)
#define WSA881X_OTP_REG_13			(WSA881X_DIGITAL_BASE + 0x008D)
#define WSA881X_OTP_REG_14			(WSA881X_DIGITAL_BASE + 0x008E)
#define WSA881X_OTP_REG_15			(WSA881X_DIGITAL_BASE + 0x008F)
#define WSA881X_OTP_REG_16			(WSA881X_DIGITAL_BASE + 0x0090)
#define WSA881X_OTP_REG_17			(WSA881X_DIGITAL_BASE + 0x0091)
#define WSA881X_OTP_REG_18			(WSA881X_DIGITAL_BASE + 0x0092)
#define WSA881X_OTP_REG_19			(WSA881X_DIGITAL_BASE + 0x0093)
#define WSA881X_OTP_REG_20			(WSA881X_DIGITAL_BASE + 0x0094)
#define WSA881X_OTP_REG_21			(WSA881X_DIGITAL_BASE + 0x0095)
#define WSA881X_OTP_REG_22			(WSA881X_DIGITAL_BASE + 0x0096)
#define WSA881X_OTP_REG_23			(WSA881X_DIGITAL_BASE + 0x0097)
#define WSA881X_OTP_REG_24			(WSA881X_DIGITAL_BASE + 0x0098)
#define WSA881X_OTP_REG_25			(WSA881X_DIGITAL_BASE + 0x0099)
#define WSA881X_OTP_REG_26			(WSA881X_DIGITAL_BASE + 0x009A)
#define WSA881X_OTP_REG_27			(WSA881X_DIGITAL_BASE + 0x009B)
#define WSA881X_OTP_REG_28			(WSA881X_DIGITAL_BASE + 0x009C)
#define WSA881X_OTP_REG_29			(WSA881X_DIGITAL_BASE + 0x009D)
#define WSA881X_OTP_REG_30			(WSA881X_DIGITAL_BASE + 0x009E)
#define WSA881X_OTP_REG_31			(WSA881X_DIGITAL_BASE + 0x009F)
#define WSA881X_OTP_REG_63			(WSA881X_DIGITAL_BASE + 0x00BF)

/* Analog Register address space */
#define WSA881X_BIAS_REF_CTRL			(WSA881X_ANALOG_BASE + 0x0000)
#define WSA881X_BIAS_TEST			(WSA881X_ANALOG_BASE + 0x0001)
#define WSA881X_BIAS_BIAS			(WSA881X_ANALOG_BASE + 0x0002)
#define WSA881X_TEMP_OP				(WSA881X_ANALOG_BASE + 0x0003)
#define WSA881X_TEMP_IREF_CTRL			(WSA881X_ANALOG_BASE + 0x0004)
#define WSA881X_TEMP_ISENS_CTRL			(WSA881X_ANALOG_BASE + 0x0005)
#define WSA881X_TEMP_CLK_CTRL			(WSA881X_ANALOG_BASE + 0x0006)
#define WSA881X_TEMP_TEST			(WSA881X_ANALOG_BASE + 0x0007)
#define WSA881X_TEMP_BIAS			(WSA881X_ANALOG_BASE + 0x0008)
#define WSA881X_TEMP_ADC_CTRL			(WSA881X_ANALOG_BASE + 0x0009)
#define WSA881X_TEMP_DOUT_MSB			(WSA881X_ANALOG_BASE + 0x000A)
#define WSA881X_TEMP_DOUT_LSB			(WSA881X_ANALOG_BASE + 0x000B)
#define WSA881X_ADC_EN_MODU_V			(WSA881X_ANALOG_BASE + 0x0010)
#define WSA881X_ADC_EN_MODU_I			(WSA881X_ANALOG_BASE + 0x0011)
#define WSA881X_ADC_EN_DET_TEST_V		(WSA881X_ANALOG_BASE + 0x0012)
#define WSA881X_ADC_EN_DET_TEST_I		(WSA881X_ANALOG_BASE + 0x0013)
#define WSA881X_ADC_SEL_IBIAS			(WSA881X_ANALOG_BASE + 0x0014)
#define WSA881X_ADC_EN_SEL_IBAIS		(WSA881X_ANALOG_BASE + 0x0015)
#define WSA881X_SPKR_DRV_EN			(WSA881X_ANALOG_BASE + 0x001A)
#define WSA881X_SPKR_DRV_GAIN			(WSA881X_ANALOG_BASE + 0x001B)
#define WSA881X_PA_GAIN_SEL_MASK		BIT(3)
#define WSA881X_PA_GAIN_SEL_REG			BIT(3)
#define WSA881X_PA_GAIN_SEL_DRE			0
#define WSA881X_SPKR_PAG_GAIN_MASK		GENMASK(7, 4)
#define WSA881X_SPKR_DAC_CTL			(WSA881X_ANALOG_BASE + 0x001C)
#define WSA881X_SPKR_DRV_DBG			(WSA881X_ANALOG_BASE + 0x001D)
#define WSA881X_SPKR_PWRSTG_DBG			(WSA881X_ANALOG_BASE + 0x001E)
#define WSA881X_SPKR_OCP_CTL			(WSA881X_ANALOG_BASE + 0x001F)
#define WSA881X_SPKR_OCP_MASK			GENMASK(7, 6)
#define WSA881X_SPKR_OCP_EN			BIT(7)
#define WSA881X_SPKR_OCP_HOLD			BIT(6)
#define WSA881X_SPKR_CLIP_CTL			(WSA881X_ANALOG_BASE + 0x0020)
#define WSA881X_SPKR_BBM_CTL			(WSA881X_ANALOG_BASE + 0x0021)
#define WSA881X_SPKR_MISC_CTL1			(WSA881X_ANALOG_BASE + 0x0022)
#define WSA881X_SPKR_MISC_CTL2			(WSA881X_ANALOG_BASE + 0x0023)
#define WSA881X_SPKR_BIAS_INT			(WSA881X_ANALOG_BASE + 0x0024)
#define WSA881X_SPKR_PA_INT			(WSA881X_ANALOG_BASE + 0x0025)
#define WSA881X_SPKR_BIAS_CAL			(WSA881X_ANALOG_BASE + 0x0026)
#define WSA881X_SPKR_BIAS_PSRR			(WSA881X_ANALOG_BASE + 0x0027)
#define WSA881X_SPKR_STATUS1			(WSA881X_ANALOG_BASE + 0x0028)
#define WSA881X_SPKR_STATUS2			(WSA881X_ANALOG_BASE + 0x0029)
#define WSA881X_BOOST_EN_CTL			(WSA881X_ANALOG_BASE + 0x002A)
#define WSA881X_BOOST_EN_MASK			BIT(7)
#define WSA881X_BOOST_EN			BIT(7)
#define WSA881X_BOOST_CURRENT_LIMIT		(WSA881X_ANALOG_BASE + 0x002B)
#define WSA881X_BOOST_PS_CTL			(WSA881X_ANALOG_BASE + 0x002C)
#define WSA881X_BOOST_PRESET_OUT1		(WSA881X_ANALOG_BASE + 0x002D)
#define WSA881X_BOOST_PRESET_OUT2		(WSA881X_ANALOG_BASE + 0x002E)
#define WSA881X_BOOST_FORCE_OUT			(WSA881X_ANALOG_BASE + 0x002F)
#define WSA881X_BOOST_LDO_PROG			(WSA881X_ANALOG_BASE + 0x0030)
#define WSA881X_BOOST_SLOPE_COMP_ISENSE_FB	(WSA881X_ANALOG_BASE + 0x0031)
#define WSA881X_BOOST_RON_CTL			(WSA881X_ANALOG_BASE + 0x0032)
#define WSA881X_BOOST_LOOP_STABILITY		(WSA881X_ANALOG_BASE + 0x0033)
#define WSA881X_BOOST_ZX_CTL			(WSA881X_ANALOG_BASE + 0x0034)
#define WSA881X_BOOST_START_CTL			(WSA881X_ANALOG_BASE + 0x0035)
#define WSA881X_BOOST_MISC1_CTL			(WSA881X_ANALOG_BASE + 0x0036)
#define WSA881X_BOOST_MISC2_CTL			(WSA881X_ANALOG_BASE + 0x0037)
#define WSA881X_BOOST_MISC3_CTL			(WSA881X_ANALOG_BASE + 0x0038)
#define WSA881X_BOOST_ATEST_CTL			(WSA881X_ANALOG_BASE + 0x0039)
#define WSA881X_SPKR_PROT_FE_GAIN		(WSA881X_ANALOG_BASE + 0x003A)
#define WSA881X_SPKR_PROT_FE_CM_LDO_SET		(WSA881X_ANALOG_BASE + 0x003B)
#define WSA881X_SPKR_PROT_FE_ISENSE_BIAS_SET1	(WSA881X_ANALOG_BASE + 0x003C)
#define WSA881X_SPKR_PROT_FE_ISENSE_BIAS_SET2	(WSA881X_ANALOG_BASE + 0x003D)
#define WSA881X_SPKR_PROT_ATEST1		(WSA881X_ANALOG_BASE + 0x003E)
#define WSA881X_SPKR_PROT_ATEST2		(WSA881X_ANALOG_BASE + 0x003F)
#define WSA881X_SPKR_PROT_FE_VSENSE_VCM		(WSA881X_ANALOG_BASE + 0x0040)
#define WSA881X_SPKR_PROT_FE_VSENSE_BIAS_SET1	(WSA881X_ANALOG_BASE + 0x0041)
#define WSA881X_BONGO_RESRV_REG1		(WSA881X_ANALOG_BASE + 0x0042)
#define WSA881X_BONGO_RESRV_REG2		(WSA881X_ANALOG_BASE + 0x0043)
#define WSA881X_SPKR_PROT_SAR			(WSA881X_ANALOG_BASE + 0x0044)
#define WSA881X_SPKR_STATUS3			(WSA881X_ANALOG_BASE + 0x0045)

#define SWRS_SCP_FRAME_CTRL_BANK(m)		(0x60 + 0x10 * (m))
#define SWRS_SCP_HOST_CLK_DIV2_CTL_BANK(m)	(0xE0 + 0x10 * (m))
#define SWR_SLV_MAX_REG_ADDR	0x390
#define SWR_SLV_START_REG_ADDR	0x40
#define SWR_SLV_MAX_BUF_LEN	20
#define BYTES_PER_LINE		12
#define SWR_SLV_RD_BUF_LEN	8
#define SWR_SLV_WR_BUF_LEN	32
#define SWR_SLV_MAX_DEVICES	2
#define WSA881X_MAX_SWR_PORTS   4
#define WSA881X_VERSION_ENTRY_SIZE 27
#define WSA881X_OCP_CTL_TIMER_SEC 2
#define WSA881X_OCP_CTL_TEMP_CELSIUS 25
#define WSA881X_OCP_CTL_POLL_TIMER_SEC 60
#define WSA881X_PROBE_TIMEOUT 1000

#define WSA881X_PA_GAIN_TLV(xname, reg, shift, max, invert, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |\
		 SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, .get = snd_soc_get_volsw,\
	.put = wsa881x_put_pa_gain, \
	.private_value = SOC_SINGLE_VALUE(reg, shift, max, invert, 0) }

static struct reg_default wsa881x_defaults[] = {
	{ WSA881X_CHIP_ID0, 0x00 },
	{ WSA881X_CHIP_ID1, 0x00 },
	{ WSA881X_CHIP_ID2, 0x00 },
	{ WSA881X_CHIP_ID3, 0x02 },
	{ WSA881X_BUS_ID, 0x00 },
	{ WSA881X_CDC_RST_CTL, 0x00 },
	{ WSA881X_CDC_TOP_CLK_CTL, 0x03 },
	{ WSA881X_CDC_ANA_CLK_CTL, 0x00 },
	{ WSA881X_CDC_DIG_CLK_CTL, 0x00 },
	{ WSA881X_CLOCK_CONFIG, 0x00 },
	{ WSA881X_ANA_CTL, 0x08 },
	{ WSA881X_SWR_RESET_EN, 0x00 },
	{ WSA881X_TEMP_DETECT_CTL, 0x01 },
	{ WSA881X_TEMP_MSB, 0x00 },
	{ WSA881X_TEMP_LSB, 0x00 },
	{ WSA881X_TEMP_CONFIG0, 0x00 },
	{ WSA881X_TEMP_CONFIG1, 0x00 },
	{ WSA881X_CDC_CLIP_CTL, 0x03 },
	{ WSA881X_SDM_PDM9_LSB, 0x00 },
	{ WSA881X_SDM_PDM9_MSB, 0x00 },
	{ WSA881X_CDC_RX_CTL, 0x7E },
	{ WSA881X_DEM_BYPASS_DATA0, 0x00 },
	{ WSA881X_DEM_BYPASS_DATA1, 0x00 },
	{ WSA881X_DEM_BYPASS_DATA2, 0x00 },
	{ WSA881X_DEM_BYPASS_DATA3, 0x00 },
	{ WSA881X_OTP_CTRL0, 0x00 },
	{ WSA881X_OTP_CTRL1, 0x00 },
	{ WSA881X_HDRIVE_CTL_GROUP1, 0x00 },
	{ WSA881X_INTR_MODE, 0x00 },
	{ WSA881X_INTR_STATUS, 0x00 },
	{ WSA881X_INTR_CLEAR, 0x00 },
	{ WSA881X_INTR_LEVEL, 0x00 },
	{ WSA881X_INTR_SET, 0x00 },
	{ WSA881X_INTR_TEST, 0x00 },
	{ WSA881X_PDM_TEST_MODE, 0x00 },
	{ WSA881X_ATE_TEST_MODE, 0x00 },
	{ WSA881X_PIN_CTL_MODE, 0x00 },
	{ WSA881X_PIN_CTL_OE, 0x00 },
	{ WSA881X_PIN_WDATA_IOPAD, 0x00 },
	{ WSA881X_PIN_STATUS, 0x00 },
	{ WSA881X_DIG_DEBUG_MODE, 0x00 },
	{ WSA881X_DIG_DEBUG_SEL, 0x00 },
	{ WSA881X_DIG_DEBUG_EN, 0x00 },
	{ WSA881X_SWR_HM_TEST1, 0x08 },
	{ WSA881X_SWR_HM_TEST2, 0x00 },
	{ WSA881X_TEMP_DETECT_DBG_CTL, 0x00 },
	{ WSA881X_TEMP_DEBUG_MSB, 0x00 },
	{ WSA881X_TEMP_DEBUG_LSB, 0x00 },
	{ WSA881X_SAMPLE_EDGE_SEL, 0x0C },
	{ WSA881X_SPARE_0, 0x00 },
	{ WSA881X_SPARE_1, 0x00 },
	{ WSA881X_SPARE_2, 0x00 },
	{ WSA881X_OTP_REG_0, 0x01 },
	{ WSA881X_OTP_REG_1, 0xFF },
	{ WSA881X_OTP_REG_2, 0xC0 },
	{ WSA881X_OTP_REG_3, 0xFF },
	{ WSA881X_OTP_REG_4, 0xC0 },
	{ WSA881X_OTP_REG_5, 0xFF },
	{ WSA881X_OTP_REG_6, 0xFF },
	{ WSA881X_OTP_REG_7, 0xFF },
	{ WSA881X_OTP_REG_8, 0xFF },
	{ WSA881X_OTP_REG_9, 0xFF },
	{ WSA881X_OTP_REG_10, 0xFF },
	{ WSA881X_OTP_REG_11, 0xFF },
	{ WSA881X_OTP_REG_12, 0xFF },
	{ WSA881X_OTP_REG_13, 0xFF },
	{ WSA881X_OTP_REG_14, 0xFF },
	{ WSA881X_OTP_REG_15, 0xFF },
	{ WSA881X_OTP_REG_16, 0xFF },
	{ WSA881X_OTP_REG_17, 0xFF },
	{ WSA881X_OTP_REG_18, 0xFF },
	{ WSA881X_OTP_REG_19, 0xFF },
	{ WSA881X_OTP_REG_20, 0xFF },
	{ WSA881X_OTP_REG_21, 0xFF },
	{ WSA881X_OTP_REG_22, 0xFF },
	{ WSA881X_OTP_REG_23, 0xFF },
	{ WSA881X_OTP_REG_24, 0x03 },
	{ WSA881X_OTP_REG_25, 0x01 },
	{ WSA881X_OTP_REG_26, 0x03 },
	{ WSA881X_OTP_REG_27, 0x11 },
	{ WSA881X_OTP_REG_63, 0x40 },
	/* WSA881x Analog registers */
	{ WSA881X_BIAS_REF_CTRL, 0x6C },
	{ WSA881X_BIAS_TEST, 0x16 },
	{ WSA881X_BIAS_BIAS, 0xF0 },
	{ WSA881X_TEMP_OP, 0x00 },
	{ WSA881X_TEMP_IREF_CTRL, 0x56 },
	{ WSA881X_TEMP_ISENS_CTRL, 0x47 },
	{ WSA881X_TEMP_CLK_CTRL, 0x87 },
	{ WSA881X_TEMP_TEST, 0x00 },
	{ WSA881X_TEMP_BIAS, 0x51 },
	{ WSA881X_TEMP_DOUT_MSB, 0x00 },
	{ WSA881X_TEMP_DOUT_LSB, 0x00 },
	{ WSA881X_ADC_EN_MODU_V, 0x00 },
	{ WSA881X_ADC_EN_MODU_I, 0x00 },
	{ WSA881X_ADC_EN_DET_TEST_V, 0x00 },
	{ WSA881X_ADC_EN_DET_TEST_I, 0x00 },
	{ WSA881X_ADC_EN_SEL_IBAIS, 0x10 },
	{ WSA881X_SPKR_DRV_EN, 0x74 },
	{ WSA881X_SPKR_DRV_DBG, 0x15 },
	{ WSA881X_SPKR_PWRSTG_DBG, 0x00 },
	{ WSA881X_SPKR_OCP_CTL, 0xD4 },
	{ WSA881X_SPKR_CLIP_CTL, 0x90 },
	{ WSA881X_SPKR_PA_INT, 0x54 },
	{ WSA881X_SPKR_BIAS_CAL, 0xAC },
	{ WSA881X_SPKR_STATUS1, 0x00 },
	{ WSA881X_SPKR_STATUS2, 0x00 },
	{ WSA881X_BOOST_EN_CTL, 0x18 },
	{ WSA881X_BOOST_CURRENT_LIMIT, 0x7A },
	{ WSA881X_BOOST_PRESET_OUT2, 0x70 },
	{ WSA881X_BOOST_FORCE_OUT, 0x0E },
	{ WSA881X_BOOST_LDO_PROG, 0x16 },
	{ WSA881X_BOOST_SLOPE_COMP_ISENSE_FB, 0x71 },
	{ WSA881X_BOOST_RON_CTL, 0x0F },
	{ WSA881X_BOOST_ZX_CTL, 0x34 },
	{ WSA881X_BOOST_START_CTL, 0x23 },
	{ WSA881X_BOOST_MISC1_CTL, 0x80 },
	{ WSA881X_BOOST_MISC2_CTL, 0x00 },
	{ WSA881X_BOOST_MISC3_CTL, 0x00 },
	{ WSA881X_BOOST_ATEST_CTL, 0x00 },
	{ WSA881X_SPKR_PROT_FE_GAIN, 0x46 },
	{ WSA881X_SPKR_PROT_FE_CM_LDO_SET, 0x3B },
	{ WSA881X_SPKR_PROT_FE_ISENSE_BIAS_SET1, 0x8D },
	{ WSA881X_SPKR_PROT_FE_ISENSE_BIAS_SET2, 0x8D },
	{ WSA881X_SPKR_PROT_ATEST1, 0x01 },
	{ WSA881X_SPKR_PROT_FE_VSENSE_VCM, 0x8D },
	{ WSA881X_SPKR_PROT_FE_VSENSE_BIAS_SET1, 0x4D },
	{ WSA881X_SPKR_PROT_SAR, 0x00 },
	{ WSA881X_SPKR_STATUS3, 0x00 },
};

static const struct reg_default wsa881x_defaults_ana[] = {
	{ WSA881X_BIAS_REF_CTRL, 0x6C },
	{ WSA881X_BIAS_TEST, 0x16 },
	{ WSA881X_BIAS_BIAS, 0xF0 },
	{ WSA881X_TEMP_OP, 0x00 },
	{ WSA881X_TEMP_IREF_CTRL, 0x56 },
	{ WSA881X_TEMP_ISENS_CTRL, 0x47 },
	{ WSA881X_TEMP_CLK_CTRL, 0x87 },
	{ WSA881X_TEMP_TEST, 0x00 },
	{ WSA881X_TEMP_BIAS, 0x51 },
	{ WSA881X_TEMP_ADC_CTRL, 0x00 },
	{ WSA881X_TEMP_DOUT_MSB, 0x00 },
	{ WSA881X_TEMP_DOUT_LSB, 0x00 },
	{ WSA881X_ADC_EN_MODU_V, 0x00 },
	{ WSA881X_ADC_EN_MODU_I, 0x00 },
	{ WSA881X_ADC_EN_DET_TEST_V, 0x00 },
	{ WSA881X_ADC_EN_DET_TEST_I, 0x00 },
	{ WSA881X_ADC_SEL_IBIAS, 0x25 },
	{ WSA881X_ADC_EN_SEL_IBAIS, 0x10 },//
	{ WSA881X_SPKR_DRV_EN, 0x74 },
	{ WSA881X_SPKR_DRV_GAIN, 0x01 },
	{ WSA881X_SPKR_DAC_CTL, 0x40 },
	{ WSA881X_SPKR_DRV_DBG, 0x15 },
	{ WSA881X_SPKR_PWRSTG_DBG, 0x00 },
	{ WSA881X_SPKR_OCP_CTL, 0xD4 },
	{ WSA881X_SPKR_CLIP_CTL, 0x90 },
	{ WSA881X_SPKR_BBM_CTL, 0x00 },
	{ WSA881X_SPKR_MISC_CTL1, 0x80 },
	{ WSA881X_SPKR_MISC_CTL2, 0x00 },
	{ WSA881X_SPKR_BIAS_INT, 0x56 },
	{ WSA881X_SPKR_PA_INT, 0x54 },
	{ WSA881X_SPKR_BIAS_CAL, 0xAC },
	{ WSA881X_SPKR_BIAS_PSRR, 0x54 },
	{ WSA881X_SPKR_STATUS1, 0x00 },
	{ WSA881X_SPKR_STATUS2, 0x00 },
	{ WSA881X_BOOST_EN_CTL, 0x18 },
	{ WSA881X_BOOST_CURRENT_LIMIT, 0x7A },
	{ WSA881X_BOOST_PS_CTL, 0xC0 },
	{ WSA881X_BOOST_PRESET_OUT1, 0x77 },
	{ WSA881X_BOOST_PRESET_OUT2, 0x70 },
	{ WSA881X_BOOST_FORCE_OUT, 0x0E },
	{ WSA881X_BOOST_LDO_PROG, 0x16 },
	{ WSA881X_BOOST_SLOPE_COMP_ISENSE_FB, 0x71 },
	{ WSA881X_BOOST_RON_CTL, 0x0F },
	{ WSA881X_BOOST_LOOP_STABILITY, 0xAD },
	{ WSA881X_BOOST_ZX_CTL, 0x34 },
	{ WSA881X_BOOST_START_CTL, 0x23 },
	{ WSA881X_BOOST_MISC1_CTL, 0x80 },
	{ WSA881X_BOOST_MISC2_CTL, 0x00 },
	{ WSA881X_BOOST_MISC3_CTL, 0x00 },
	{ WSA881X_BOOST_ATEST_CTL, 0x00 },
	{ WSA881X_SPKR_PROT_FE_GAIN, 0x46 },
	{ WSA881X_SPKR_PROT_FE_CM_LDO_SET, 0x3B },
	{ WSA881X_SPKR_PROT_FE_ISENSE_BIAS_SET1, 0x8D },
	{ WSA881X_SPKR_PROT_FE_ISENSE_BIAS_SET2, 0x8D },
	{ WSA881X_SPKR_PROT_ATEST1, 0x01 },
	{ WSA881X_SPKR_PROT_ATEST2, 0x00 },
	{ WSA881X_SPKR_PROT_FE_VSENSE_VCM, 0x8D },
	{ WSA881X_SPKR_PROT_FE_VSENSE_BIAS_SET1, 0x4D },
	{ WSA881X_BONGO_RESRV_REG1, 0x00 },
	{ WSA881X_BONGO_RESRV_REG2, 0x00 },
	{ WSA881X_SPKR_PROT_SAR, 0x00 },
	{ WSA881X_SPKR_STATUS3, 0x00 },
};

static const struct reg_sequence wsa881x_pre_pmu_pa_2_0[] = {
	{ WSA881X_SPKR_DRV_GAIN, 0x41, 0 },
	{ WSA881X_SPKR_MISC_CTL1, 0x87, 0 },
};

static const struct reg_sequence wsa881x_vi_txfe_en_2_0[] = {
	{ WSA881X_SPKR_PROT_FE_VSENSE_VCM, 0x85, 0 },
	{ WSA881X_SPKR_PROT_ATEST2, 0x0A, 0 },
	{ WSA881X_SPKR_PROT_FE_GAIN, 0x47, 0 },
};

static const struct reg_sequence wsa881x_vi_txfe_en_2_0_ana[] = {
	{ WSA881X_SPKR_PROT_FE_VSENSE_VCM, 0x0, 0 },
	{ WSA881X_SPKR_PROT_ATEST2, 0x0A, 0 },
	{ WSA881X_SPKR_PROT_FE_GAIN, 0x41, 0 },
};

/* Default register reset values for WSA881x rev 2.0 */
static struct reg_sequence wsa881x_rev_2_0[] = {
	{ WSA881X_RESET_CTL, 0x00, 0x00 },
	{ WSA881X_TADC_VALUE_CTL, 0x01, 0x00 },
	{ WSA881X_INTR_MASK, 0x1B, 0x00 },
	{ WSA881X_IOPAD_CTL, 0x00, 0x00 },
	{ WSA881X_OTP_REG_28, 0x3F, 0x00 },
	{ WSA881X_OTP_REG_29, 0x3F, 0x00 },
	{ WSA881X_OTP_REG_30, 0x01, 0x00 },
	{ WSA881X_OTP_REG_31, 0x01, 0x00 },
	{ WSA881X_TEMP_ADC_CTRL, 0x03, 0x00 },
	{ WSA881X_ADC_SEL_IBIAS, 0x45, 0x00 },
	{ WSA881X_SPKR_DRV_GAIN, 0xC1, 0x00 },
	{ WSA881X_SPKR_DAC_CTL, 0x42, 0x00 },
	{ WSA881X_SPKR_BBM_CTL, 0x02, 0x00 },
	{ WSA881X_SPKR_MISC_CTL1, 0x40, 0x00 },
	{ WSA881X_SPKR_MISC_CTL2, 0x07, 0x00 },
	{ WSA881X_SPKR_BIAS_INT, 0x5F, 0x00 },
	{ WSA881X_SPKR_BIAS_PSRR, 0x44, 0x00 },
	{ WSA881X_BOOST_PS_CTL, 0xA0, 0x00 },
	{ WSA881X_BOOST_PRESET_OUT1, 0xB7, 0x00 },
	{ WSA881X_BOOST_LOOP_STABILITY, 0x8D, 0x00 },
	{ WSA881X_SPKR_PROT_ATEST2, 0x02, 0x00 },
	{ WSA881X_BONGO_RESRV_REG1, 0x5E, 0x00 },
	{ WSA881X_BONGO_RESRV_REG2, 0x07, 0x00 },
};

enum wsa_port_ids {
	WSA881X_PORT_DAC,
	WSA881X_PORT_COMP,
	WSA881X_PORT_BOOST,
	WSA881X_PORT_VISENSE,
};

/* 4 ports */
static struct sdw_dpn_prop wsa_sink_dpn_prop[WSA881X_MAX_SWR_PORTS] = {
	{
		/* DAC */
		.num = 1,
		.type = SDW_DPN_SIMPLE,
		.min_ch = 1,
		.max_ch = 1,
		.simple_ch_prep_sm = true,
		.read_only_wordlength = true,
	}, {
		/* COMP */
		.num = 2,
		.type = SDW_DPN_SIMPLE,
		.min_ch = 1,
		.max_ch = 1,
		.simple_ch_prep_sm = true,
		.read_only_wordlength = true,
	}, {
		/* BOOST */
		.num = 3,
		.type = SDW_DPN_SIMPLE,
		.min_ch = 1,
		.max_ch = 1,
		.simple_ch_prep_sm = true,
		.read_only_wordlength = true,
	}, {
		/* VISENSE */
		.num = 4,
		.type = SDW_DPN_SIMPLE,
		.min_ch = 1,
		.max_ch = 1,
		.simple_ch_prep_sm = true,
		.read_only_wordlength = true,
	}
};

static const struct sdw_port_config wsa881x_pconfig[WSA881X_MAX_SWR_PORTS] = {
	{
		.num = 1,
		.ch_mask = 0x1,
	}, {
		.num = 2,
		.ch_mask = 0xf,
	}, {
		.num = 3,
		.ch_mask = 0x3,
	}, {	/* IV feedback */
		.num = 4,
		.ch_mask = 0x3,
	},
};

static bool wsa881x_readable_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case WSA881X_CHIP_ID0:
	case WSA881X_CHIP_ID1:
	case WSA881X_CHIP_ID2:
	case WSA881X_CHIP_ID3:
	case WSA881X_BUS_ID:
	case WSA881X_CDC_RST_CTL:
	case WSA881X_CDC_TOP_CLK_CTL:
	case WSA881X_CDC_ANA_CLK_CTL:
	case WSA881X_CDC_DIG_CLK_CTL:
	case WSA881X_CLOCK_CONFIG:
	case WSA881X_ANA_CTL:
	case WSA881X_SWR_RESET_EN:
	case WSA881X_RESET_CTL:
	case WSA881X_TADC_VALUE_CTL:
	case WSA881X_TEMP_DETECT_CTL:
	case WSA881X_TEMP_MSB:
	case WSA881X_TEMP_LSB:
	case WSA881X_TEMP_CONFIG0:
	case WSA881X_TEMP_CONFIG1:
	case WSA881X_CDC_CLIP_CTL:
	case WSA881X_SDM_PDM9_LSB:
	case WSA881X_SDM_PDM9_MSB:
	case WSA881X_CDC_RX_CTL:
	case WSA881X_DEM_BYPASS_DATA0:
	case WSA881X_DEM_BYPASS_DATA1:
	case WSA881X_DEM_BYPASS_DATA2:
	case WSA881X_DEM_BYPASS_DATA3:
	case WSA881X_OTP_CTRL0:
	case WSA881X_OTP_CTRL1:
	case WSA881X_HDRIVE_CTL_GROUP1:
	case WSA881X_INTR_MODE:
	case WSA881X_INTR_MASK:
	case WSA881X_INTR_STATUS:
	case WSA881X_INTR_CLEAR:
	case WSA881X_INTR_LEVEL:
	case WSA881X_INTR_SET:
	case WSA881X_INTR_TEST:
	case WSA881X_PDM_TEST_MODE:
	case WSA881X_ATE_TEST_MODE:
	case WSA881X_PIN_CTL_MODE:
	case WSA881X_PIN_CTL_OE:
	case WSA881X_PIN_WDATA_IOPAD:
	case WSA881X_PIN_STATUS:
	case WSA881X_DIG_DEBUG_MODE:
	case WSA881X_DIG_DEBUG_SEL:
	case WSA881X_DIG_DEBUG_EN:
	case WSA881X_SWR_HM_TEST1:
	case WSA881X_SWR_HM_TEST2:
	case WSA881X_TEMP_DETECT_DBG_CTL:
	case WSA881X_TEMP_DEBUG_MSB:
	case WSA881X_TEMP_DEBUG_LSB:
	case WSA881X_SAMPLE_EDGE_SEL:
	case WSA881X_IOPAD_CTL:
	case WSA881X_SPARE_0:
	case WSA881X_SPARE_1:
	case WSA881X_SPARE_2:
	case WSA881X_OTP_REG_0:
	case WSA881X_OTP_REG_1:
	case WSA881X_OTP_REG_2:
	case WSA881X_OTP_REG_3:
	case WSA881X_OTP_REG_4:
	case WSA881X_OTP_REG_5:
	case WSA881X_OTP_REG_6:
	case WSA881X_OTP_REG_7:
	case WSA881X_OTP_REG_8:
	case WSA881X_OTP_REG_9:
	case WSA881X_OTP_REG_10:
	case WSA881X_OTP_REG_11:
	case WSA881X_OTP_REG_12:
	case WSA881X_OTP_REG_13:
	case WSA881X_OTP_REG_14:
	case WSA881X_OTP_REG_15:
	case WSA881X_OTP_REG_16:
	case WSA881X_OTP_REG_17:
	case WSA881X_OTP_REG_18:
	case WSA881X_OTP_REG_19:
	case WSA881X_OTP_REG_20:
	case WSA881X_OTP_REG_21:
	case WSA881X_OTP_REG_22:
	case WSA881X_OTP_REG_23:
	case WSA881X_OTP_REG_24:
	case WSA881X_OTP_REG_25:
	case WSA881X_OTP_REG_26:
	case WSA881X_OTP_REG_27:
	case WSA881X_OTP_REG_28:
	case WSA881X_OTP_REG_29:
	case WSA881X_OTP_REG_30:
	case WSA881X_OTP_REG_31:
	case WSA881X_OTP_REG_63:
	case WSA881X_BIAS_REF_CTRL:
	case WSA881X_BIAS_TEST:
	case WSA881X_BIAS_BIAS:
	case WSA881X_TEMP_OP:
	case WSA881X_TEMP_IREF_CTRL:
	case WSA881X_TEMP_ISENS_CTRL:
	case WSA881X_TEMP_CLK_CTRL:
	case WSA881X_TEMP_TEST:
	case WSA881X_TEMP_BIAS:
	case WSA881X_TEMP_ADC_CTRL:
	case WSA881X_TEMP_DOUT_MSB:
	case WSA881X_TEMP_DOUT_LSB:
	case WSA881X_ADC_EN_MODU_V:
	case WSA881X_ADC_EN_MODU_I:
	case WSA881X_ADC_EN_DET_TEST_V:
	case WSA881X_ADC_EN_DET_TEST_I:
	case WSA881X_ADC_SEL_IBIAS:
	case WSA881X_ADC_EN_SEL_IBAIS:
	case WSA881X_SPKR_DRV_EN:
	case WSA881X_SPKR_DRV_GAIN:
	case WSA881X_SPKR_DAC_CTL:
	case WSA881X_SPKR_DRV_DBG:
	case WSA881X_SPKR_PWRSTG_DBG:
	case WSA881X_SPKR_OCP_CTL:
	case WSA881X_SPKR_CLIP_CTL:
	case WSA881X_SPKR_BBM_CTL:
	case WSA881X_SPKR_MISC_CTL1:
	case WSA881X_SPKR_MISC_CTL2:
	case WSA881X_SPKR_BIAS_INT:
	case WSA881X_SPKR_PA_INT:
	case WSA881X_SPKR_BIAS_CAL:
	case WSA881X_SPKR_BIAS_PSRR:
	case WSA881X_SPKR_STATUS1:
	case WSA881X_SPKR_STATUS2:
	case WSA881X_BOOST_EN_CTL:
	case WSA881X_BOOST_CURRENT_LIMIT:
	case WSA881X_BOOST_PS_CTL:
	case WSA881X_BOOST_PRESET_OUT1:
	case WSA881X_BOOST_PRESET_OUT2:
	case WSA881X_BOOST_FORCE_OUT:
	case WSA881X_BOOST_LDO_PROG:
	case WSA881X_BOOST_SLOPE_COMP_ISENSE_FB:
	case WSA881X_BOOST_RON_CTL:
	case WSA881X_BOOST_LOOP_STABILITY:
	case WSA881X_BOOST_ZX_CTL:
	case WSA881X_BOOST_START_CTL:
	case WSA881X_BOOST_MISC1_CTL:
	case WSA881X_BOOST_MISC2_CTL:
	case WSA881X_BOOST_MISC3_CTL:
	case WSA881X_BOOST_ATEST_CTL:
	case WSA881X_SPKR_PROT_FE_GAIN:
	case WSA881X_SPKR_PROT_FE_CM_LDO_SET:
	case WSA881X_SPKR_PROT_FE_ISENSE_BIAS_SET1:
	case WSA881X_SPKR_PROT_FE_ISENSE_BIAS_SET2:
	case WSA881X_SPKR_PROT_ATEST1:
	case WSA881X_SPKR_PROT_ATEST2:
	case WSA881X_SPKR_PROT_FE_VSENSE_VCM:
	case WSA881X_SPKR_PROT_FE_VSENSE_BIAS_SET1:
	case WSA881X_BONGO_RESRV_REG1:
	case WSA881X_BONGO_RESRV_REG2:
	case WSA881X_SPKR_PROT_SAR:
	case WSA881X_SPKR_STATUS3:
		return true;
	default:
		return false;
	}
}

static bool wsa881x_volatile_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case WSA881X_CHIP_ID0:
	case WSA881X_CHIP_ID1:
	case WSA881X_CHIP_ID2:
	case WSA881X_CHIP_ID3:
	case WSA881X_BUS_ID:
	case WSA881X_TEMP_MSB:
	case WSA881X_TEMP_LSB:
	case WSA881X_SDM_PDM9_LSB:
	case WSA881X_SDM_PDM9_MSB:
	case WSA881X_OTP_CTRL1:
	case WSA881X_INTR_STATUS:
	case WSA881X_ATE_TEST_MODE:
	case WSA881X_PIN_STATUS:
	case WSA881X_SWR_HM_TEST2:
	case WSA881X_SPKR_STATUS1:
	case WSA881X_SPKR_STATUS2:
	case WSA881X_SPKR_STATUS3:
	case WSA881X_OTP_REG_0:
	case WSA881X_OTP_REG_1:
	case WSA881X_OTP_REG_2:
	case WSA881X_OTP_REG_3:
	case WSA881X_OTP_REG_4:
	case WSA881X_OTP_REG_5:
	case WSA881X_OTP_REG_31:
	case WSA881X_TEMP_DOUT_MSB:
	case WSA881X_TEMP_DOUT_LSB:
	case WSA881X_TEMP_OP:
	case WSA881X_SPKR_PROT_SAR:
		return true;
	default:
		return false;
	}
}

/*
 * Private data Structure for wsa881x. All parameters related to
 * WSA881X codec needs to be defined here.
 */
struct wsa881x_priv {
	struct regmap *regmap;
	struct regmap *regmap_analog;
	struct device *dev;
	bool analog_mode; /* In analog mode, only I2C communication is available */
	struct sdw_slave *slave;
	struct sdw_stream_config sconfig;
	struct sdw_stream_runtime *sruntime;
	struct sdw_port_config port_config[WSA881X_MAX_SWR_PORTS];
	struct gpio_desc *sd_n;
	/*
	 * Logical state for SD_N GPIO: high for shutdown, low for enable.
	 * For backwards compatibility.
	 */
	unsigned int sd_n_val;
	struct gpio_desc *mclk_pin;
	struct gpio_desc *ivsense_clk_pin;
	struct gpio_desc *ivsense_data_pin;
	int version;
	int active_ports;
	bool port_prepared[WSA881X_MAX_SWR_PORTS];
	bool port_enable[WSA881X_MAX_SWR_PORTS];
	bool boost_enable;
	bool visense_enable;
	struct mutex bg_lock;
	struct mutex res_lock;
	u8 clk_cnt;
	u8 bg_cnt;
	struct clk *mclk;
	struct i2c_client *ana_client;
	struct i2c_client *dig_client;
};

static unsigned int wsa881x_i2c_read(struct snd_soc_component *component,
				     unsigned int reg)
{
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(component);
	struct regmap *regmap;
	u32 val;

	/*
	 * Route digital reg accesses to the digital part and analog to the
	 * analog part and translate it from SDW to I2C addresses.
	 */
	if (reg >= WSA881X_ANALOG_BASE) {
		regmap = wsa881x->regmap_analog;
		reg -= WSA881X_ANALOG_BASE;
	} else {
		regmap = wsa881x->regmap;
		reg -= WSA881X_DIGITAL_BASE;
	}

	regmap_read(regmap, reg, &val);

	return val & GENMASK(7, 0);
}

static int wsa881x_i2c_write(struct snd_soc_component *component,
			     unsigned int reg, unsigned int val)
{
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(component);
	struct regmap *regmap;

	/*
	 * Route digital reg accesses to the digital part and analog to the
	 * analog part and translate it from SDW to I2C addresses.
	 */
	if (reg >= WSA881X_ANALOG_BASE) {
		regmap = wsa881x->regmap_analog;
		reg -= WSA881X_ANALOG_BASE;
	} else {
		regmap = wsa881x->regmap;
		reg -= WSA881X_DIGITAL_BASE;
	}

	val &= GENMASK(7, 0);

	return regmap_write(regmap, reg, val);
}

static struct regmap_config wsa881x_i2c_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_NONE,
	.max_register = WSA881X_SPKR_STATUS3,
	.reg_format_endian = REGMAP_ENDIAN_NATIVE,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.use_single_read = true,
	.use_single_write = true,
};

static struct regmap_config wsa881x_regmap_config = {
	.reg_bits = 32,
	.val_bits = 8,
	.cache_type = REGCACHE_MAPLE,
	.reg_defaults = wsa881x_defaults,
	.max_register = WSA881X_SPKR_STATUS3,
	.num_reg_defaults = ARRAY_SIZE(wsa881x_defaults),
	.volatile_reg = wsa881x_volatile_register,
	.readable_reg = wsa881x_readable_register,
	.reg_format_endian = REGMAP_ENDIAN_NATIVE,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
};

enum {
	G_18DB = 0,
	G_16P5DB,
	G_15DB,
	G_13P5DB,
	G_12DB,
	G_10P5DB,
	G_9DB,
	G_7P5DB,
	G_6DB,
	G_4P5DB,
	G_3DB,
	G_1P5DB,
	G_0DB,
};

static void wsa881x_init(struct wsa881x_priv *wsa881x, struct snd_soc_component *component)
{
	unsigned int val = 0;


	pr_err("DETECTED WSA881X CHIP_ID0 = 0x%x", snd_soc_component_read(component, WSA881X_CHIP_ID0));
	wsa881x->version = snd_soc_component_read(component, WSA881X_CHIP_ID1);
	pr_err("DETECTED WSA881X CHIP_ID1 = 0x%x", wsa881x->version);
	pr_err("DETECTED WSA881X CHIP_ID2 = 0x%x", snd_soc_component_read(component, WSA881X_CHIP_ID2));
	pr_err("DETECTED WSA881X CHIP_ID3 = 0x%x", snd_soc_component_read(component, WSA881X_CHIP_ID3));
	regmap_register_patch(wsa881x->regmap, wsa881x_rev_2_0,
			      ARRAY_SIZE(wsa881x_rev_2_0));

	/* Enable software reset output from soundwire slave */
	if (!wsa881x->analog_mode)
		snd_soc_component_update_bits(component, WSA881X_SWR_RESET_EN, 0x07, 0x07);

	/* Bring out of analog reset */
	snd_soc_component_update_bits(component, WSA881X_CDC_RST_CTL, 0x02, 0x02);

	/* Bring out of digital reset */
	snd_soc_component_update_bits(component, WSA881X_CDC_RST_CTL, 0x01, 0x01);
	snd_soc_component_update_bits(component, WSA881X_CLOCK_CONFIG, 0x10, 0x10);
	snd_soc_component_update_bits(component, WSA881X_SPKR_OCP_CTL, 0x02, 0x02);
	snd_soc_component_update_bits(component, WSA881X_SPKR_MISC_CTL1, 0xC0, 0x80);
	snd_soc_component_update_bits(component, WSA881X_SPKR_MISC_CTL1, 0x06, 0x06);
	snd_soc_component_update_bits(component, WSA881X_SPKR_BIAS_INT, 0xFF, 0x00);
	snd_soc_component_update_bits(component, WSA881X_SPKR_PA_INT, 0xF0, 0x40);
	snd_soc_component_update_bits(component, WSA881X_SPKR_PA_INT, 0x0E, 0x0E);
	snd_soc_component_update_bits(component, WSA881X_BOOST_LOOP_STABILITY, 0x03, 0x03);
	snd_soc_component_update_bits(component, WSA881X_BOOST_MISC2_CTL, 0xFF, 0x14);
	snd_soc_component_update_bits(component, WSA881X_BOOST_START_CTL, 0x80, 0x80);
	snd_soc_component_update_bits(component, WSA881X_BOOST_START_CTL, 0x03, 0x00);
	snd_soc_component_update_bits(component, WSA881X_BOOST_SLOPE_COMP_ISENSE_FB, 0x0C, 0x04);
	snd_soc_component_update_bits(component, WSA881X_BOOST_SLOPE_COMP_ISENSE_FB, 0x03, 0x00);

	val = snd_soc_component_read(component, WSA881X_OTP_REG_0);
	if (val)
		snd_soc_component_update_bits(component, WSA881X_BOOST_PRESET_OUT1, 0xF0, 0x70);

	snd_soc_component_update_bits(component, WSA881X_BOOST_PRESET_OUT2, 0xF0, 0x30);
	snd_soc_component_update_bits(component, WSA881X_SPKR_DRV_EN, 0x08, 0x08);
	snd_soc_component_update_bits(component, WSA881X_BOOST_CURRENT_LIMIT, 0x0F, 0x08);
	snd_soc_component_update_bits(component, WSA881X_SPKR_OCP_CTL, 0x30, 0x30);
	snd_soc_component_update_bits(component, WSA881X_SPKR_OCP_CTL, 0x0C, 0x00);
	snd_soc_component_update_bits(component, WSA881X_OTP_REG_28, 0x3F, 0x3A);
	snd_soc_component_update_bits(component, WSA881X_BONGO_RESRV_REG1, 0xFF, 0xB2);
	snd_soc_component_update_bits(component, WSA881X_BONGO_RESRV_REG2, 0xFF, 0x05);
}

static int wsa881x_component_probe(struct snd_soc_component *comp)
{
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(comp);

	snd_soc_component_init_regmap(comp, wsa881x->regmap);

	return 0;
}

static int wsa881x_i2c_component_probe(struct snd_soc_component *comp)
{
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(comp);

	wsa881x_init(wsa881x, comp);

	return 0;
}

static int wsa881x_put_pa_gain(struct snd_kcontrol *kc,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kc);
	struct soc_mixer_control *mc =
			(struct soc_mixer_control *)kc->private_value;
	int max = mc->max;
	unsigned int mask = (1 << fls(max)) - 1;
	int val, ret, min_gain, max_gain;

	ret = pm_runtime_resume_and_get(comp->dev);
	if (ret < 0 && ret != -EACCES)
		return ret;

	max_gain = (max - ucontrol->value.integer.value[0]) & mask;
	/*
	 * Gain has to set incrementally in 4 steps
	 * as per HW sequence
	 */
	if (max_gain > G_4P5DB)
		min_gain = G_0DB;
	else
		min_gain = max_gain + 3;
	/*
	 * 1ms delay is needed before change in gain
	 * as per HW requirement.
	 */
	usleep_range(1000, 1010);

	for (val = min_gain; max_gain <= val; val--) {
		ret = snd_soc_component_update_bits(comp,
			      WSA881X_SPKR_DRV_GAIN,
			      WSA881X_SPKR_PAG_GAIN_MASK,
			      val << 4);
		if (ret < 0)
			dev_err(comp->dev, "Failed to change PA gain");

		usleep_range(1000, 1010);
	}

	pm_runtime_mark_last_busy(comp->dev);
	pm_runtime_put_autosuspend(comp->dev);

	return 1;
}

static int wsa881x_get_port(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct wsa881x_priv *data = snd_soc_component_get_drvdata(comp);
	struct soc_mixer_control *mixer =
		(struct soc_mixer_control *)kcontrol->private_value;
	int portidx = mixer->reg;

	ucontrol->value.integer.value[0] = data->port_enable[portidx];


	return 0;
}

static int wsa881x_boost_ctrl(struct snd_soc_component *component, bool enable)
{
	u8 out1_val = snd_soc_component_read(component, WSA881X_OTP_REG_0) ? 0x70 : 0xb0;

	if (enable) {
		snd_soc_component_update_bits(component,
					      WSA881X_BOOST_LOOP_STABILITY,
					      0x03, 0x03);
		snd_soc_component_update_bits(component,
					      WSA881X_BOOST_MISC2_CTL,
					      0xFF, 0x14);
		snd_soc_component_update_bits(component,
					      WSA881X_BOOST_START_CTL,
					      0x80, 0x80);
		snd_soc_component_update_bits(component,
					      WSA881X_BOOST_START_CTL,
					      0x03, 0x00);
		snd_soc_component_update_bits(component,
					      WSA881X_BOOST_SLOPE_COMP_ISENSE_FB,
					      0x0C, 0x04);
		snd_soc_component_update_bits(component,
					      WSA881X_BOOST_SLOPE_COMP_ISENSE_FB,
					      0x03, 0x00);
		snd_soc_component_update_bits(component,
					      WSA881X_BOOST_PRESET_OUT1,
					      0xF0, out1_val);
		snd_soc_component_update_bits(component,
					      WSA881X_ANA_CTL, 0x03, 0x01);
		snd_soc_component_update_bits(component,
					      WSA881X_SPKR_DRV_EN,
					      0x08, 0x08);
		snd_soc_component_update_bits(component,
					      WSA881X_ANA_CTL, 0x04, 0x04);
		snd_soc_component_update_bits(component,
					      WSA881X_BOOST_CURRENT_LIMIT,
					      0x0F, 0x08);

		snd_soc_component_update_bits(component, WSA881X_BOOST_EN_CTL,
					      WSA881X_BOOST_EN_MASK,
					      WSA881X_BOOST_EN);
	} else {
		snd_soc_component_update_bits(component, WSA881X_BOOST_EN_CTL,
					      WSA881X_BOOST_EN_MASK, 0);
	}
	/*
	 * 1.5ms sleep is needed after boost enable/disable as per
	 * HW requirement
	 */
	usleep_range(1500, 1510);
	return 0;
}

static int wsa881x_set_port(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct wsa881x_priv *data = snd_soc_component_get_drvdata(comp);
	struct soc_mixer_control *mixer =
		(struct soc_mixer_control *)kcontrol->private_value;
	int portidx = mixer->reg;

	if (ucontrol->value.integer.value[0]) {
		if (data->port_enable[portidx])
			return 0;

		data->port_enable[portidx] = true;
	} else {
		if (!data->port_enable[portidx])
			return 0;

		data->port_enable[portidx] = false;
	}

	if (portidx == WSA881X_PORT_BOOST) /* Boost Switch */
		wsa881x_boost_ctrl(comp, data->port_enable[portidx]);

	return 1;
}

static const char * const smart_boost_lvl_text[] = {
	"6.625 V", "6.750 V", "6.875 V", "7.000 V",
	"7.125 V", "7.250 V", "7.375 V", "7.500 V",
	"7.625 V", "7.750 V", "7.875 V", "8.000 V",
	"8.125 V", "8.250 V", "8.375 V", "8.500 V"
};

static const struct soc_enum smart_boost_lvl_enum =
	SOC_ENUM_SINGLE(WSA881X_BOOST_PRESET_OUT1, 0,
			ARRAY_SIZE(smart_boost_lvl_text),
			smart_boost_lvl_text);

static const DECLARE_TLV_DB_SCALE(pa_gain, 0, 150, 0);

static const struct snd_kcontrol_new wsa881x_snd_controls[] = {
	SOC_ENUM("Smart Boost Level", smart_boost_lvl_enum),
	WSA881X_PA_GAIN_TLV("PA Volume", WSA881X_SPKR_DRV_GAIN,
			    4, 0xC, 1, pa_gain),
	SOC_SINGLE_EXT("DAC Switch", WSA881X_PORT_DAC, 0, 1, 0,
		       wsa881x_get_port, wsa881x_set_port),
	SOC_SINGLE_EXT("COMP Switch", WSA881X_PORT_COMP, 0, 1, 0,
		       wsa881x_get_port, wsa881x_set_port),
	SOC_SINGLE_EXT("BOOST Switch", WSA881X_PORT_BOOST, 0, 1, 0,
		       wsa881x_get_port, wsa881x_set_port),
	SOC_SINGLE_EXT("VISENSE Switch", WSA881X_PORT_VISENSE, 0, 1, 0,
		       wsa881x_get_port, wsa881x_set_port),
};

static const struct snd_soc_dapm_route wsa881x_audio_map[] = {
	{ "RDAC", NULL, "IN" },
	{ "RDAC", NULL, "DCLK" },
	{ "RDAC", NULL, "ACLK" },
	{ "RDAC", NULL, "Bandgap" },
	{ "SPKR PGA", NULL, "RDAC" },
	{ "SPKR", NULL, "SPKR PGA" },
};

static int wsa881x_visense_txfe_ctrl(struct snd_soc_component *comp,
				     bool enable)
{
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(comp);

	if (enable) {
		regmap_multi_reg_write(wsa881x->regmap, wsa881x_vi_txfe_en_2_0_ana,
				       ARRAY_SIZE(wsa881x_vi_txfe_en_2_0_ana));
	} else {
		snd_soc_component_update_bits(comp,
					      WSA881X_SPKR_PROT_FE_VSENSE_VCM,
					      0x08, 0x08);
		/*
		 * 200us sleep is needed after visense txfe disable as per
		 * HW requirement.
		 */
		usleep_range(200, 210);
		snd_soc_component_update_bits(comp, WSA881X_SPKR_PROT_FE_GAIN,
					      0x01, 0x00);
	}
	return 0;
}

static int wsa881x_visense_adc_ctrl(struct snd_soc_component *comp,
				    bool enable)
{
	snd_soc_component_update_bits(comp, WSA881X_ADC_EN_MODU_V, BIT(7),
				      (enable << 7));
	snd_soc_component_update_bits(comp, WSA881X_ADC_EN_MODU_I, BIT(7),
				      (enable << 7));
	return 0;
}

static void wsa881x_bandgap_ctrl(struct snd_soc_component *component,
				 bool enable)
{
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(component);

	mutex_lock(&wsa881x->bg_lock);

	if (enable) {
		if (wsa881x->bg_cnt > 0)
			return;

		snd_soc_component_update_bits(component,
					      WSA881X_TEMP_OP, 0x08, 0x08);
		/* 400usec sleep is needed as per HW requirement */
		usleep_range(400, 410);
		snd_soc_component_update_bits(component,
					      WSA881X_TEMP_OP, 0x04, 0x04);
		wsa881x->bg_cnt++;
	} else {
		if (wsa881x->bg_cnt <= 0)
			return;

		snd_soc_component_update_bits(component,
					      WSA881X_TEMP_OP, 0x04, 0x00);
		snd_soc_component_update_bits(component,
					      WSA881X_TEMP_OP, 0x08, 0x00);
		wsa881x->bg_cnt--;
	}

	mutex_unlock(&wsa881x->bg_lock);
}

static void wsa881x_clk_ctrl(struct snd_soc_component *component, bool enable)
{
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(component);

	mutex_lock(&wsa881x->res_lock);

	if (enable) {
		if (wsa881x->clk_cnt > 0)
			return;

		snd_soc_component_write(component,
					WSA881X_CDC_RST_CTL, 0x02);
		snd_soc_component_write(component,
					WSA881X_CDC_RST_CTL, 0x03);
		snd_soc_component_write(component,
					WSA881X_CLOCK_CONFIG, 0x01);

		snd_soc_component_write(component,
					WSA881X_CDC_DIG_CLK_CTL, 0x01);
		snd_soc_component_write(component,
					WSA881X_CDC_ANA_CLK_CTL, 0x01);
		wsa881x->clk_cnt++;
	} else {
		if (wsa881x->clk_cnt <= 0)
			return;

		snd_soc_component_write(component,
					WSA881X_CDC_ANA_CLK_CTL, 0x00);
		snd_soc_component_write(component,
					WSA881X_CDC_DIG_CLK_CTL, 0x00);
		snd_soc_component_update_bits(component,
					WSA881X_CDC_TOP_CLK_CTL, 0x01, 0x00);
		wsa881x->clk_cnt--;
	}

	mutex_unlock(&wsa881x->res_lock);
}

static void wsa881x_rdac_ctrl(struct snd_soc_component *component, bool enable)
{
	if (enable) {
		snd_soc_component_update_bits(component,
					WSA881X_ANA_CTL, 0x08, 0x00);
		snd_soc_component_update_bits(component,
					WSA881X_SPKR_DRV_GAIN, 0x08, 0x08);
		snd_soc_component_update_bits(component,
					WSA881X_SPKR_DAC_CTL, 0x20, 0x20);
		snd_soc_component_update_bits(component,
					WSA881X_SPKR_DAC_CTL, 0x20, 0x00);
		snd_soc_component_update_bits(component,
					WSA881X_SPKR_DAC_CTL, 0x40, 0x40);
		snd_soc_component_update_bits(component,
					WSA881X_SPKR_DAC_CTL, 0x80, 0x80);
		snd_soc_component_update_bits(component,
					WSA881X_SPKR_BIAS_CAL, 0x01, 0x01);
		snd_soc_component_update_bits(component,
					WSA881X_SPKR_OCP_CTL, 0x30, 0x30);
		snd_soc_component_update_bits(component,
					WSA881X_SPKR_OCP_CTL, 0x0C, 0x00);
		snd_soc_component_update_bits(component,
					WSA881X_SPKR_DRV_GAIN, 0xF0, 0x40);
		snd_soc_component_update_bits(component,
					WSA881X_SPKR_MISC_CTL1, 0x01, 0x01);
	} else {
		/* Ensure class-D amp is off */
		snd_soc_component_update_bits(component,
					WSA881X_SPKR_DAC_CTL, 0x80, 0x00);
	}
}

static int wsa881x_rdac_event(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(component);

	if (!wsa881x->analog_mode)
		return 0;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		gpiod_direction_output(wsa881x->sd_n, !wsa881x->sd_n_val);
		wsa881x_clk_ctrl(component, true);
		snd_soc_component_update_bits(component,
					      WSA881X_SPKR_DAC_CTL, 0x02, 0x02);
		wsa881x_bandgap_ctrl(component, true);
		snd_soc_component_update_bits(component,
					      WSA881X_SPKR_MISC_CTL1, 0xC0, 0x80);
		snd_soc_component_update_bits(component,
					      WSA881X_SPKR_MISC_CTL1, 0x06, 0x06);
		snd_soc_component_update_bits(component,
					      WSA881X_SPKR_PA_INT, 0xF0, 0x20);
		if (wsa881x->boost_enable)
			wsa881x_boost_ctrl(component, true);
		break;
	case SND_SOC_DAPM_POST_PMU:
		wsa881x_rdac_ctrl(component, true);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		wsa881x_rdac_ctrl(component, false);
		if (wsa881x->visense_enable) {
			wsa881x_visense_adc_ctrl(component, false);
			wsa881x_visense_txfe_ctrl(component, false);
			gpiod_direction_output(wsa881x->ivsense_clk_pin, 0);
			gpiod_direction_output(wsa881x->ivsense_data_pin, 0);
		}
		break;
	case SND_SOC_DAPM_POST_PMD:
		if (wsa881x->boost_enable)
			wsa881x_boost_ctrl(component, false);
		wsa881x_clk_ctrl(component, false);
		wsa881x_bandgap_ctrl(component, false);
		gpiod_direction_output(wsa881x->sd_n, wsa881x->sd_n_val);
		break;
	}

	return 0;
}

static int wsa881x_spkr_pa_event(struct snd_soc_dapm_widget *w,
				 struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(comp);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_component_update_bits(comp, WSA881X_SPKR_OCP_CTL,
					      WSA881X_SPKR_OCP_MASK,
					      WSA881X_SPKR_OCP_EN);
		regmap_multi_reg_write(wsa881x->regmap, wsa881x_pre_pmu_pa_2_0,
				       ARRAY_SIZE(wsa881x_pre_pmu_pa_2_0));

		snd_soc_component_update_bits(comp, WSA881X_SPKR_DRV_GAIN,
					      WSA881X_PA_GAIN_SEL_MASK,
					      WSA881X_PA_GAIN_SEL_REG);
		break;
	case SND_SOC_DAPM_POST_PMU:
		if (wsa881x->port_prepared[WSA881X_PORT_VISENSE]) {
			gpiod_direction_output(wsa881x->ivsense_clk_pin, 1);
			gpiod_direction_output(wsa881x->ivsense_data_pin, 1);
			wsa881x_visense_txfe_ctrl(comp, true);
			snd_soc_component_update_bits(comp,
						      WSA881X_ADC_EN_SEL_IBAIS,
						      0x07, 0x01);
			wsa881x_visense_adc_ctrl(comp, true);
		}

		break;
	case SND_SOC_DAPM_POST_PMD:
		if (wsa881x->port_prepared[WSA881X_PORT_VISENSE]) {
			wsa881x_visense_adc_ctrl(comp, false);
			wsa881x_visense_txfe_ctrl(comp, false);
		}

		snd_soc_component_update_bits(comp, WSA881X_SPKR_OCP_CTL,
					      WSA881X_SPKR_OCP_MASK,
					      WSA881X_SPKR_OCP_EN |
					      WSA881X_SPKR_OCP_HOLD);
		break;
	}
	return 0;
}

static const struct snd_soc_dapm_widget wsa881x_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("IN"),
	SND_SOC_DAPM_DAC_E("RDAC", NULL, SND_SOC_NOPM, 7, 0,
			   wsa881x_rdac_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("SPKR PGA", SND_SOC_NOPM, 0, 0, NULL, 0,
			   wsa881x_spkr_pa_event, SND_SOC_DAPM_PRE_PMU |
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("DCLK", WSA881X_CDC_DIG_CLK_CTL, 0, 0, NULL,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("ACLK", WSA881X_CDC_ANA_CLK_CTL, 0, 0, NULL,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("Bandgap", WSA881X_TEMP_OP, 3, 0,
			    NULL,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_OUTPUT("SPKR"),
};

static int wsa881x_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct wsa881x_priv *wsa881x = dev_get_drvdata(dai->dev);
	int i;

	if (wsa881x->analog_mode)
		return 0;

	wsa881x->active_ports = 0;
	for (i = 0; i < WSA881X_MAX_SWR_PORTS; i++) {
		if (!wsa881x->port_enable[i])
			continue;

		wsa881x->port_config[wsa881x->active_ports] =
							wsa881x_pconfig[i];
		wsa881x->active_ports++;
	}

	return sdw_stream_add_slave(wsa881x->slave, &wsa881x->sconfig,
				    wsa881x->port_config, wsa881x->active_ports,
				    wsa881x->sruntime);
}

static int wsa881x_hw_free(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *dai)
{
	struct wsa881x_priv *wsa881x = dev_get_drvdata(dai->dev);

	if (!wsa881x->analog_mode)
		sdw_stream_remove_slave(wsa881x->slave, wsa881x->sruntime);

	return 0;
}

static int wsa881x_set_sdw_stream(struct snd_soc_dai *dai,
				  void *stream, int direction)
{
	struct wsa881x_priv *wsa881x = dev_get_drvdata(dai->dev);

	if (!wsa881x->analog_mode)
		wsa881x->sruntime = stream;

	return 0;
}

static int wsa881x_digital_mute(struct snd_soc_dai *dai, int mute, int stream)
{
	struct wsa881x_priv *wsa881x = dev_get_drvdata(dai->dev);

	if (mute)
		regmap_update_bits(wsa881x->regmap, WSA881X_SPKR_DRV_EN, 0x80,
				   0x00);
	else
		regmap_update_bits(wsa881x->regmap, WSA881X_SPKR_DRV_EN, 0x80,
				   0x80);

	return 0;
}

static const struct snd_soc_dai_ops wsa881x_dai_ops = {
	.hw_params = wsa881x_hw_params,
	.hw_free = wsa881x_hw_free,
	.mute_stream = wsa881x_digital_mute,
	.set_stream = wsa881x_set_sdw_stream,
};

static struct snd_soc_dai_driver wsa881x_dais[] = {
	{
		.name = "SPKR",
		.id = 0,
		.playback = {
			.stream_name = "SPKR Playback",
			.rates = SNDRV_PCM_RATE_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.rate_max = 48000,
			.rate_min = 48000,
			.channels_min = 1,
			.channels_max = 1,
		},
		.ops = &wsa881x_dai_ops,
	},
};

static const struct snd_soc_component_driver wsa881x_i2c_component_drv = {
	.name = "WSA881x",
	.probe = wsa881x_i2c_component_probe,
	.read = wsa881x_i2c_read,
	.write = wsa881x_i2c_write,
	.controls = wsa881x_snd_controls,
	.num_controls = ARRAY_SIZE(wsa881x_snd_controls),
	.dapm_widgets = wsa881x_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(wsa881x_dapm_widgets),
	.dapm_routes = wsa881x_audio_map,
	.num_dapm_routes = ARRAY_SIZE(wsa881x_audio_map),
	.endianness = 1,
};

static const struct snd_soc_component_driver wsa881x_component_drv = {
	.name = "WSA881x",
	.probe = wsa881x_component_probe,
	.controls = wsa881x_snd_controls,
	.num_controls = ARRAY_SIZE(wsa881x_snd_controls),
	.dapm_widgets = wsa881x_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(wsa881x_dapm_widgets),
	.dapm_routes = wsa881x_audio_map,
	.num_dapm_routes = ARRAY_SIZE(wsa881x_audio_map),
	.endianness = 1,
};

static int wsa881x_update_status(struct sdw_slave *slave,
				 enum sdw_slave_status status)
{
	struct wsa881x_priv *wsa881x = dev_get_drvdata(&slave->dev);

	// if (status == SDW_SLAVE_ATTACHED && slave->dev_num > 0)
	// 	wsa881x_init(wsa881x);

	return 0;
}

static int wsa881x_port_prep(struct sdw_slave *slave,
			     struct sdw_prepare_ch *prepare_ch,
			     enum sdw_port_prep_ops state)
{
	struct wsa881x_priv *wsa881x = dev_get_drvdata(&slave->dev);

	if (state == SDW_OPS_PORT_POST_PREP)
		wsa881x->port_prepared[prepare_ch->num - 1] = true;
	else
		wsa881x->port_prepared[prepare_ch->num - 1] = false;

	return 0;
}

static int wsa881x_bus_config(struct sdw_slave *slave,
			      struct sdw_bus_params *params)
{
	sdw_write(slave, SWRS_SCP_HOST_CLK_DIV2_CTL_BANK(params->next_bank),
		  0x01);

	return 0;
}

static const struct sdw_slave_ops wsa881x_slave_ops = {
	.update_status = wsa881x_update_status,
	.bus_config = wsa881x_bus_config,
	.port_prep = wsa881x_port_prep,
};

static int wsa881x_common_probe(struct wsa881x_priv *wsa881x)
{
	struct device *dev = wsa881x->dev;

	wsa881x->sd_n = devm_gpiod_get_optional(dev, "powerdown",
						GPIOD_FLAGS_BIT_NONEXCLUSIVE);
	if (IS_ERR(wsa881x->sd_n))
		return dev_err_probe(dev, PTR_ERR(wsa881x->sd_n),
				     "Shutdown Control GPIO not found\n");

	/*
	 * Backwards compatibility work-around.
	 *
	 * The SD_N GPIO is active low, however upstream DTS used always active
	 * high.  Changing the flag in driver and DTS will break backwards
	 * compatibility, so add a simple value inversion to work with both old
	 * and new DTS.
	 *
	 * This won't work properly with DTS using the flags properly in cases:
	 * 1. Old DTS with proper ACTIVE_LOW, however such case was broken
	 *    before as the driver required the active high.
	 * 2. New DTS with proper ACTIVE_HIGH (intended), which is rare case
	 *    (not existing upstream) but possible. This is the price of
	 *    backwards compatibility, therefore this hack should be removed at
	 *    some point.
	 */
	wsa881x->sd_n_val = gpiod_is_active_low(wsa881x->sd_n);
	if (!wsa881x->sd_n_val)
		dev_warn(dev, "Using ACTIVE_HIGH for shutdown GPIO. Your DTB might be outdated or you use unsupported configuration for the GPIO.");

	wsa881x->dev = dev;

	gpiod_direction_output(wsa881x->sd_n, !wsa881x->sd_n_val);
	dev_set_drvdata(dev, wsa881x);
	pm_runtime_set_autosuspend_delay(dev, 3000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_mark_last_busy(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	return devm_snd_soc_register_component(dev, &wsa881x_i2c_component_drv,//sdfgsdfgsdfg
					       wsa881x_dais, ARRAY_SIZE(wsa881x_dais));
}

static int wsa881x_sdw_probe(struct sdw_slave *pdev, const struct sdw_device_id *id)
{
	struct device *dev = &pdev->dev;
	struct wsa881x_priv *wsa881x;

	wsa881x = devm_kzalloc(dev, sizeof(*wsa881x), GFP_KERNEL);
	if (!wsa881x)
		return -ENOMEM;

	wsa881x->slave = pdev;
	wsa881x->dev = dev;
	wsa881x->sconfig.ch_count = 1;
	wsa881x->sconfig.bps = 1;
	wsa881x->sconfig.frame_rate = 48000;
	wsa881x->sconfig.direction = SDW_DATA_DIR_RX;
	wsa881x->sconfig.type = SDW_STREAM_PDM;
	pdev->prop.sink_ports = GENMASK(WSA881X_MAX_SWR_PORTS, 0);
	pdev->prop.sink_dpn_prop = wsa_sink_dpn_prop;
	pdev->prop.scp_int1_mask = SDW_SCP_INT1_BUS_CLASH | SDW_SCP_INT1_PARITY;

	wsa881x->regmap = devm_regmap_init_sdw(pdev, &wsa881x_regmap_config);
	if (IS_ERR(wsa881x->regmap))
		return dev_err_probe(dev, PTR_ERR(wsa881x->regmap), "regmap_init failed\n");

	return wsa881x_common_probe(wsa881x);
}

/*
 * Only digital signal handling is supported when using Soundwire.
 * If one wants to use the analog capabilities of the WSA881x chips, they must
 * be configured through I2C. Depending on the resistor configuration, the
 * "digital logic" is accessible at addresses 0xe and 0xf (spkr0/left and
 * spkr1/right respectively) and the analog part is @ reg+0x36.
 */
#define I2C_ANALOG_OFFSET		0x36
static int wsa881x_i2c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct wsa881x_priv *wsa881x;

	wsa881x = devm_kzalloc(dev, sizeof(*wsa881x), GFP_KERNEL);
	if (!wsa881x)
		return -ENOMEM;

	wsa881x->analog_mode = true;
	wsa881x->dev = dev;

	wsa881x->dig_client = client;
	wsa881x->ana_client = devm_i2c_new_dummy_device(&client->dev, client->adapter,
							client->addr + I2C_ANALOG_OFFSET);
	if (IS_ERR(wsa881x->ana_client))
		return PTR_ERR(wsa881x->ana_client);

	wsa881x->mclk = devm_clk_get(dev, NULL);
	if (IS_ERR(wsa881x->mclk))
		return PTR_ERR(wsa881x->mclk);

	wsa881x->mclk_pin = devm_gpiod_get(dev, "mclk", GPIOD_FLAGS_BIT_NONEXCLUSIVE);
	if (IS_ERR(wsa881x->mclk_pin))
		return dev_err_probe(dev, PTR_ERR(wsa881x->mclk_pin),
				     "MCLK GPIO not found\n");

	wsa881x->ivsense_clk_pin = devm_gpiod_get(dev, "ivsense-clk",
						  GPIOD_FLAGS_BIT_NONEXCLUSIVE);
	if (IS_ERR(wsa881x->ivsense_clk_pin))
		return dev_err_probe(dev, PTR_ERR(wsa881x->ivsense_clk_pin),
				     "IV sense clk GPIO not found\n");

	wsa881x->ivsense_data_pin = devm_gpiod_get(dev, "ivsense-data",
						   GPIOD_FLAGS_BIT_NONEXCLUSIVE);
	if (IS_ERR(wsa881x->ivsense_data_pin))
		return dev_err_probe(dev, PTR_ERR(wsa881x->ivsense_data_pin),
				     "IV sense data GPIO not found\n");

	wsa881x->regmap = devm_regmap_init_i2c(client,
					       &wsa881x_i2c_regmap_config);
	if (IS_ERR(wsa881x->regmap))
		return dev_err_probe(dev, PTR_ERR(wsa881x->regmap),
				     "digital regmap_init failed\n");

	wsa881x->regmap_analog = devm_regmap_init_i2c(wsa881x->ana_client,
						      &wsa881x_i2c_regmap_config);
	if (IS_ERR(wsa881x->regmap_analog))
		return dev_err_probe(dev, PTR_ERR(wsa881x->regmap_analog),
				     "analog regmap_init failed\n");

	return wsa881x_common_probe(wsa881x);
}

static int __maybe_unused wsa881x_runtime_suspend(struct device *dev)
{
	struct regmap *regmap = dev_get_regmap(dev, NULL);
	struct wsa881x_priv *wsa881x = dev_get_drvdata(dev);

	if (wsa881x->analog_mode) {
		clk_disable_unprepare(wsa881x->mclk);
		gpiod_direction_output(wsa881x->mclk_pin, 0);
		gpiod_direction_output(wsa881x->sd_n, wsa881x->sd_n_val);

		return 0;
	}
	gpiod_direction_output(wsa881x->sd_n, wsa881x->sd_n_val);

	regcache_cache_only(regmap, true);
	regcache_mark_dirty(regmap);

	return 0;
}

static int __maybe_unused wsa881x_runtime_resume(struct device *dev)
{
	struct sdw_slave *slave = dev_to_sdw_dev(dev);
	struct regmap *regmap = dev_get_regmap(dev, NULL);
	struct wsa881x_priv *wsa881x = dev_get_drvdata(dev);
	unsigned long time;

	if (wsa881x->analog_mode) {
		gpiod_direction_output(wsa881x->mclk_pin, 1);
		clk_prepare_enable(wsa881x->mclk);
		clk_set_rate(wsa881x->mclk, 9600000);
	}
	gpiod_direction_output(wsa881x->sd_n, !wsa881x->sd_n_val);
	gpiod_direction_output(wsa881x->sd_n, wsa881x->sd_n_val);
	gpiod_direction_output(wsa881x->sd_n, !wsa881x->sd_n_val);

	if (wsa881x->analog_mode)
		return 0;

	time = wait_for_completion_timeout(&slave->initialization_complete,
					   msecs_to_jiffies(WSA881X_PROBE_TIMEOUT));
	if (!time) {
		dev_err(dev, "Initialization not complete, timed out\n");
		gpiod_direction_output(wsa881x->sd_n, wsa881x->sd_n_val);
		return -ETIMEDOUT;
	}

	regcache_cache_only(regmap, false);
	regcache_sync(regmap);

	return 0;
}

static const struct dev_pm_ops wsa881x_pm_ops = {
	SET_RUNTIME_PM_OPS(wsa881x_runtime_suspend, wsa881x_runtime_resume, NULL)
};

static const struct of_device_id wsa881x_of_match_table[] = {
	{ .compatible = "qcom,wsa881x" },
	{ }
};
MODULE_DEVICE_TABLE(of, wsa881x_of_match_table);

/* I2C is used for controlling analog inputs */
static struct i2c_driver wsa881x_i2c_codec_driver = {
	.probe = wsa881x_i2c_probe,
	.driver = {
		.name = "wsa881x-i2c-codec",
		.of_match_table = wsa881x_of_match_table,
	},
};
module_i2c_driver(wsa881x_i2c_codec_driver);

static const struct sdw_device_id wsa881x_slave_id[] = {
	SDW_SLAVE_ENTRY(0x0217, 0x2010, 0),
	SDW_SLAVE_ENTRY(0x0217, 0x2110, 0),
	{},
};
MODULE_DEVICE_TABLE(sdw, wsa881x_slave_id);

static struct sdw_driver wsa881x_codec_driver = {
	.probe	= wsa881x_sdw_probe,
	.ops = &wsa881x_slave_ops,
	.id_table = wsa881x_slave_id,
	.driver = {
		.name	= "wsa881x-codec",
		.pm = &wsa881x_pm_ops,
	}
};
module_sdw_driver(wsa881x_codec_driver);

MODULE_DESCRIPTION("WSA881x codec driver");
MODULE_LICENSE("GPL v2");
