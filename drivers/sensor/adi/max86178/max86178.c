/*
 * Copyright (c) 2026 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_max86178

#include "max86178.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(max86178, CONFIG_SENSOR_LOG_LEVEL);

/* I2C bus operations */
#if defined(MAX86178_BUS_I2C)
static int max86178_bus_check_i2c(const union max86178_bus *bus)
{
	if (!i2c_is_ready_dt(&bus->i2c)) {
		return -ENODEV;
	}
	return 0;
}

static int max86178_reg_access_i2c(const struct device *dev, bool read, uint8_t reg_addr,
				   uint8_t *data, size_t length)
{
	const struct max86178_dev_config *config = dev->config;

	if (read) {
		return i2c_burst_read_dt(&config->bus.i2c, reg_addr, data, length);
	} else {
		return i2c_burst_write_dt(&config->bus.i2c, reg_addr, data, length);
	}
}
#endif /* MAX86178_BUS_I2C */

/* SPI bus operations */
#if defined(MAX86178_BUS_SPI)
static int max86178_bus_check_spi(const union max86178_bus *bus)
{
	if (!spi_is_ready_dt(&bus->spi)) {
		return -ENODEV;
	}
	return 0;
}

static int max86178_reg_access_spi(const struct device *dev, bool read, uint8_t reg_addr,
				   uint8_t *data, size_t length)
{
	const struct max86178_dev_config *config = dev->config;
	uint8_t addr_buf[2];
	int ret;

	addr_buf[0] = reg_addr;
	addr_buf[1] = read ? BIT(7) : 0; /* Set MSB for read operations */

	const struct spi_buf tx_bufs[] = {
		{
			.buf = addr_buf,
			.len = 2,
		},
		{
			.buf = read ? NULL : data,
			.len = read ? 0 : length,
		},
	};

	const struct spi_buf_set tx = {
		.buffers = tx_bufs,
		.count = ARRAY_SIZE(tx_bufs),
	};

	const struct spi_buf rx_bufs[2] = {
        {
            .buf = NULL,
            .len = ARRAY_SIZE(addr_buf),
        },
        {
            .buf = read ? data : NULL,
            .len = read ? length : 0,
        },
    };

	const struct spi_buf_set rx = {
		.buffers = rx_bufs,
		.count = ARRAY_SIZE(rx_bufs),
	};

	ret = spi_transceive_dt(&config->bus.spi, &tx, &rx);
	if (ret < 0) {
		LOG_ERR("SPI transceive failed: %d", ret);
		return ret;
	}

	return 0;
}
#endif /* MAX86178_BUS_SPI */

int max86178_reg_read(const struct device *dev, uint8_t reg_addr, uint8_t *data, size_t length)
{
	const struct max86178_dev_config *config = dev->config;

	return config->reg_access(dev, true, reg_addr, data, length);
}

int max86178_reg_write(const struct device *dev, uint8_t reg_addr, uint8_t *data, size_t length)
{
	const struct max86178_dev_config *config = dev->config;

	return config->reg_access(dev, false, reg_addr, data, length);
}

int max86178_reg_update(const struct device *dev, uint8_t reg_addr, uint8_t mask, uint8_t value)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, reg_addr, &reg_val, 1);
	if (ret < 0) {
		return ret;
	}

	reg_val &= ~mask;
	reg_val |= FIELD_PREP(mask, value);


	return max86178_reg_write(dev, reg_addr, &reg_val, 1);
}

static int max86178_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	/* Implement sample fetch logic */
	return -ENOTSUP;
}

static int max86178_channel_get(const struct device *dev, enum sensor_channel chan,
				struct sensor_value *val)
{
	/* Implement channel get logic */
	return -ENOTSUP;
}

static int max86178_attr_set(const struct device *dev, enum sensor_channel chan,
			     enum sensor_attribute attr, const struct sensor_value *val)
{
	/* Implement attribute set logic */
	return -ENOTSUP;
}

static int max86178_attr_get(const struct device *dev, enum sensor_channel chan,
			     enum sensor_attribute attr, struct sensor_value *val)
{
	/* Implement attribute get logic */
	return -ENOTSUP;
}

static const struct sensor_driver_api max86178_api = {
	.sample_fetch = max86178_sample_fetch,
	.channel_get = max86178_channel_get,
	.attr_set = max86178_attr_set,
	.attr_get = max86178_attr_get,
};

static int max86178_osc_enable(const struct device *dev, bool enable)
{
	return max86178_reg_update(dev, MAX86178_PLL_CFG1, MAX86178_PLL_CFG1_PLL_EN_MSK, enable ? 1 : 0);
}

static int max86178_set_pll_cfg6(const struct device *dev, enum ref_clk_sel ref_clk, enum clk_ref_sel clk_freq_sel, enum max86178_clk_fine_tune clk_fine_tune)
{
	uint8_t reg_val;
	int ret;

	reg_val = FIELD_PREP(MAX86178_PLL_CFG6_REF_CLK_SEL_MSK, ref_clk) |
		  FIELD_PREP(MAX86178_PLL_CFG6_CLK_FREQ_SEL_MSK, clk_freq_sel) |
		  FIELD_PREP(MAX86178_PLL_CFG6_CLK_FINE_TUNE_MSK, clk_fine_tune);

	ret = max86178_reg_write(dev, MAX86178_PLL_CFG6, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set PLL CFG6: %d", ret);
		return ret;
	}

	return 0;
}

static int get_clk_freq_sel(const struct device *dev, uint8_t *freq_sel)
{
	int ret = 0;
	uint8_t reg_val;
	uint8_t ref_clk_sel;
	ret = max86178_reg_read(dev, MAX86178_PLL_CFG6, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read PLL CFG6: %d", ret);
		return ret;
	}

	ref_clk_sel = FIELD_GET(MAX86178_PLL_CFG6_REF_CLK_SEL_MSK, reg_val);
	*freq_sel = ref_clk_sel;

	return 0;
}

static int validate_mdiv(const struct device *dev, uint16_t mdiv)
{
	int ret = 0;
	uint8_t freq_sel;
	
	ret = get_clk_freq_sel(dev, &freq_sel);
	if (ret < 0) {
		LOG_ERR("Failed to get clock frequency selection: %d", ret);
		return ret;
	}

	mdiv = mdiv + 1;
	switch (freq_sel) {
	case MAX86178_REF_CLK_32000:
		if (mdiv < 125 || mdiv > 875) {
			LOG_ERR("Invalid mdiv value: %d for 32kHz reference clock", mdiv);
			return -EINVAL;
		}
		break;
	case MAX86178_REF_CLK_32768:
		if (mdiv < 123 || mdiv > 854) {
			LOG_ERR("Invalid mdiv value: %d for 32.768kHz reference clock", mdiv);
			return -EINVAL;
		}
		break;
	default:
		LOG_ERR("Unknown reference clock selection: %d", freq_sel);
		return -EINVAL;
	}
	return 0;
}

static int max86178_get_mdiv(const struct device *dev, uint32_t *mdiv)
{
	int ret = 0;
	uint8_t reg_val;

	ret = max86178_reg_read(dev, MAX86178_PLL_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read PLL CFG1: %d", ret);
		return ret;
	}

	*mdiv = FIELD_GET(MAX86178_PLL_CFG1_MDIV_MSB_MSK, reg_val) << 8;

	ret = max86178_reg_read(dev, MAX86178_PLL_CFG2, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read PLL CFG2: %d", ret);
		return ret;
	}
	*mdiv |= reg_val;

	*mdiv = *mdiv + 1; /* MDIV value is register value + 1 */
	return 0;
}

static int max86178_get_ecg_fdiv(const struct device *dev, uint32_t *ecg_fdiv)
{
	int ret = 0;
	uint8_t reg_val;

	ret = max86178_reg_read(dev, MAX86178_PLL_CFG3, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read PLL CFG3: %d", ret);
		return ret;
	}

	*ecg_fdiv = FIELD_GET(MAX86178_PLL_CFG4_ECG_FDIV_MSK, reg_val);
	switch (*ecg_fdiv) {
	case 0:
		LOG_INF("ECG FDIV: ECG_ADC_CLK Disabled");
		break;
	case 1:
		*ecg_fdiv = 1;
		break;
	case 2:
		*ecg_fdiv = 2;
		break;
	case 3:
		*ecg_fdiv = 3;
		break;
	case 4:
		*ecg_fdiv = 8;
		break;
	case 5:
	case 6:
	case 7:
		*ecg_fdiv = 16;
		break;
	default:
		LOG_ERR("Unknown ECG FDIV register value: %d", *ecg_fdiv);
		return -EINVAL;
	}

	return 0;
}

static int max86178_get_ecg_ndiv(const struct device *dev, uint32_t *ecg_ndiv)
{
	int ret = 0;
	uint8_t reg_val;

	ret = max86178_reg_read(dev, MAX86178_PLL_CFG4, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read PLL CFG4: %d", ret);
		return ret;
	}

	*ecg_ndiv = FIELD_GET(MAX86178_PLL_CFG4_ECG_NDIV_MSB_MSK, reg_val) << 8;

	ret = max86178_reg_read(dev, MAX86178_PLL_CFG5, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read PLL CFG5: %d", ret);
		return ret;
	}
	*ecg_ndiv |= reg_val;

	if (*ecg_ndiv < 16){
		LOG_WRN("ECG NDIV value %d is less than minimum of 16. Using 16 instead.", *ecg_ndiv);
		*ecg_ndiv = 16; /* Minimum ndiv value is 16 */
	}
	return 0;
}

static int max86178_get_bioz_ndiv(const struct device *dev, uint32_t *bioz_ndiv)
{
	int ret = 0;
	uint8_t reg_val;

	ret = max86178_reg_read(dev, MAX86178_PLL_CFG3, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read PLL CFG3: %d", ret);
		return ret;
	}

	*bioz_ndiv = FIELD_GET(MAX86178_PLL_CFG3_BIOZ_NDIV_MSK, reg_val);
	switch (*bioz_ndiv) {
	case 0:
		*bioz_ndiv = 256;
		break;
	case 1:
		*bioz_ndiv = 512;
		break;
	case 2:
		*bioz_ndiv = 1024;
		break;
	case 3:
		*bioz_ndiv = 1024;
		break;
	default:
		LOG_ERR("Unknown BIOZ NDIV register value: %d", *bioz_ndiv);
		return -EINVAL;
	}

	return 0;
}

static int max86178_get_bioz_kdiv(const struct device *dev, uint32_t *bioz_kdiv)
{
	int ret = 0;
	uint8_t reg_val;

	ret = max86178_reg_read(dev, MAX86178_PLL_CFG3, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read PLL CFG3: %d", ret);
		return ret;
	}

	*bioz_kdiv = FIELD_GET(MAX86178_PLL_CFG3_BIOZ_KDIV_MSK, reg_val);
	switch (*bioz_kdiv) {
	case 0:
		*bioz_kdiv = 1;
		break;
	case 1:
		*bioz_kdiv = 2;
		break;
	case 2:
		*bioz_kdiv = 4;
		break;
	case 3:
		*bioz_kdiv = 8;
		break;
	case 4:
		*bioz_kdiv = 16;
		break;
	case 5:
		*bioz_kdiv = 32;
		break;
	case 6:
		*bioz_kdiv = 64;
		break;
	case 7:
		*bioz_kdiv = 128;
		break;
	case 8:
		*bioz_kdiv = 256;
		break;
	case 9:
		*bioz_kdiv = 512;
		break;
	case 10:
		*bioz_kdiv = 1024;
		break;
	case 11:
		*bioz_kdiv = 2048;
		break;
	case 12:
		*bioz_kdiv = 4096;
		break;
	case 13:
	case 14:
	case 15:
		*bioz_kdiv = 8192;
		break;
	default:
		LOG_ERR("Unknown BIOZ KDIV register value: %d", *bioz_kdiv);
		return -EINVAL;
	}

	return 0;
}

static int max86178_get_pll_clk(const struct device *dev, uint32_t *pll_clk)
{
	int ret = 0;
	uint8_t reg_val;
	uint8_t clk_freq_sel;
	uint32_t mdiv;

	/* Read MDIV */
	ret = max86178_reg_read(dev, MAX86178_PLL_CFG2, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read MDIV LSB: %d", ret);
		return ret;
	}
	mdiv = reg_val;

	ret = max86178_reg_read(dev, MAX86178_PLL_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read MDIV MSB: %d", ret);
		return ret;
	}
	mdiv |= FIELD_GET(MAX86178_MDIV_MSB_MSK, reg_val) << 8;
	mdiv = mdiv + 1; /* M = MDIV + 1 */

	/* Read clock frequency selection */
	ret = get_clk_freq_sel(dev, &clk_freq_sel);
	if (ret < 0) {
		LOG_ERR("Failed to get clock frequency selection: %d", ret);
		return ret;
	}

	switch (clk_freq_sel) {
	case MAX86178_REF_CLK_32000:
		*pll_clk = 32000 * mdiv;
		break;
	case MAX86178_REF_CLK_32768:
		*pll_clk = 32768 * mdiv;
		break;
	default:
		LOG_ERR("Unknown reference clock selection: %d", clk_freq_sel);
		return -EINVAL;
	}

	return 0;
}

static int max86178_get_ecg_pll_clk(const struct device *dev, uint32_t *ecg_pll_clk)
{
	int ret = 0;
	uint32_t pll_clk;
	uint8_t reg_val;
	uint8_t fdiv_reg_val;
	uint8_t fdiv;

	ret = max86178_get_pll_clk(dev, &pll_clk);
	if (ret < 0) {
		LOG_ERR("Failed to get PLL clock: %d", ret);
		return ret;
	}

	ret = max86178_reg_read(dev, MAX86178_PLL_CFG4, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read ECG FDIV: %d", ret);
		return ret;
	}
	fdiv_reg_val = FIELD_GET(MAX86178_PLL_CFG4_ECG_FDIV_MSK, reg_val);

	switch (fdiv_reg_val) {
	case 0:
		fdiv = 0;
		break;
	case 1:
		fdiv = 1;
		break;
	case 2:
		fdiv = 2;
		break;
	case 3:
		fdiv = 4;
		break;
	case 4:
		fdiv = 8;
		break;
	case 5:
	case 6:
	case 7:
		fdiv = 16;
		break;
	default:
		LOG_ERR("Unknown ECG FDIV register value: %d", fdiv_reg_val);
		return -EINVAL;
	}
	*ecg_pll_clk = pll_clk / fdiv;

	return 0;
}

static int max86178_get_ecg_adc_clk(const struct device *dev, uint32_t *ecg_adc_clk)
{
	int ret = 0;
	uint32_t ecg_pll_clk;
	uint8_t reg_val;
	uint16_t ndiv;

	ret = max86178_get_ecg_pll_clk(dev, &ecg_pll_clk);
	if (ret < 0) {
		LOG_ERR("Failed to get ECG PLL clock: %d", ret);
		return ret;
	}

	ret = max86178_reg_read(dev, MAX86178_PLL_CFG4, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read ECG NDIV and decimation rate: %d", ret);
		return ret;
	}
	ndiv = FIELD_GET(MAX86178_PLL_CFG4_ECG_NDIV_MSB_MSK, reg_val) << 8;

	ret = max86178_reg_read(dev, MAX86178_PLL_CFG5, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read ECG NDIV LSB: %d", ret);
		return ret;
	}
	ndiv |= reg_val;

	if (ndiv < 16){
		ndiv = 16; /* Minimum ndiv value is 16 */
	}
	*ecg_adc_clk = ecg_pll_clk / ndiv;
	return 0;
}

static int max86178_get_bioz_adc_clk(const struct device *dev, uint32_t *bioz_adc_clk)
{
	int ret = 0;
	uint32_t pll_clk;
	uint8_t reg_val;
	uint8_t ecg_f_div_reg_val;
	uint8_t ecg_f_div;
	uint8_t resp_en;
	uint16_t bioz_ndiv;

	ret = max86178_get_pll_clk(dev, &pll_clk);
	if (ret < 0) {
		LOG_ERR("Failed to get PLL clock: %d", ret);
		return ret;
	}

	ret = max86178_reg_read(dev, MAX86178_RESP_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read RESP_CFG1: %d", ret);
		return ret;
	}
	resp_en = FIELD_GET(MAX86178_RESP_CFG1_RESP_EN_MSK, reg_val);
	if (resp_en == 0) {
		*bioz_adc_clk = pll_clk / bioz_ndiv;
		return 0;
	}

	ret = max86178_reg_read(dev, MAX86178_PLL_CFG4, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read ECG FDIV: %d", ret);
		return ret;
	}
	ecg_f_div_reg_val = FIELD_GET(MAX86178_PLL_CFG4_ECG_FDIV_MSK, reg_val);
	switch (ecg_f_div_reg_val) {
	case 0:
		LOG_ERR("ECG KDIV cannot be 0 when RESP_CFG1_RESP_EN is set");
		return -EINVAL;
	case 1:
		ecg_f_div = 1;
		break;
	case 2:
		ecg_f_div = 2;
		break;
	case 3:
		ecg_f_div = 4;
		break;
	case 4:
		ecg_f_div = 8;
		break;
	case 5:
	case 6:
	case 7:
		ecg_f_div = 16;
		break;
	default:
		LOG_ERR("Unknown ECG FDIV register value: %d", ecg_f_div_reg_val);
		return -EINVAL;
	}

	*bioz_adc_clk = pll_clk / (ecg_f_div * bioz_ndiv);
	return 0;
}

static int max86178_get_bioz_synth_clk(const struct device *dev, uint32_t *bioz_synth_clk)
{
	int ret = 0;
	uint32_t pll_clk;
	uint8_t reg_val;
	uint8_t ecg_f_div_reg_val;
	uint8_t ecg_f_div;
	uint8_t resp_en;
	uint16_t bioz_kdiv;

	ret = max86178_get_pll_clk(dev, &pll_clk);
	if (ret < 0) {
		LOG_ERR("Failed to get PLL clock: %d", ret);
		return ret;
	}

	ret = max86178_reg_read(dev, MAX86178_RESP_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read RESP_CFG1: %d", ret);
		return ret;
	}

	resp_en = FIELD_GET(MAX86178_RESP_CFG1_RESP_EN_MSK, reg_val);
	if (resp_en == 0) {
		*bioz_synth_clk = pll_clk / bioz_kdiv;
		return 0;
	}

}

static int max86178_set_mdiv(const struct device *dev, uint16_t mdiv)
{
	int ret = 0;
	uint8_t reg_val;
	ret = validate_mdiv(dev, mdiv);
	if (ret < 0) {
		LOG_ERR("MDIV validation failed: %d", ret);
		return ret;
	}
	/* Get MSB of mdiv */
	reg_val = FIELD_GET(MAX86178_MDIV_MSB_MSK, mdiv);

	ret = max86178_reg_update(dev, MAX86178_PLL_CFG1, MAX86178_PLL_CFG1_MDIV_MSB_MSK, reg_val);
	if (ret < 0) {
		LOG_ERR("Failed to set MDIV MSB: %d", ret);
		return ret;
	}
	/* Get LSB of mdiv */
	reg_val = mdiv & 0xFF;
	ret = max86178_reg_write(dev, MAX86178_PLL_CFG2, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set MDIV LSB: %d", ret);
		return ret;
	}
	return 0;

}

static int max86178_set_ecg_fdiv(const struct device *dev, uint8_t fdiv)
{
	int ret;

	ret = max86178_reg_update(dev,MAX86178_PLL_CFG4, MAX86178_PLL_CFG4_ECG_FDIV_MSK, fdiv);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG FDIV: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_set_ecg_ndiv(const struct device *dev, uint16_t ndiv)
{
	int ret;
	uint8_t reg_val;

	/* Get MSB of ndiv */
	reg_val = FIELD_GET(MAX86178_NDIV_MSB_MSK, ndiv);

	ret = max86178_reg_update(dev, MAX86178_PLL_CFG4, MAX86178_PLL_CFG4_ECG_NDIV_MSB_MSK, reg_val);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG NDIV MSB: %d", ret);
		return ret;
	}
	/* Get LSB of ndiv */
	reg_val = ndiv & 0xFF;
	ret = max86178_reg_write(dev, MAX86178_PLL_CFG5, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG NDIV LSB: %d", ret);
		return ret;
	}
	return 0;
}

static int validate_ecg_fdiv_ndiv(const struct device *dev, uint32_t pll_clk, uint8_t fdiv_reg_val, uint16_t ndiv_reg_val)
{
	int ret;
	uint32_t ecg_adc_clk;

	uint8_t fdiv;
	switch (fdiv_reg_val) {
	case 0:
		fdiv = 0;
		break;
	case 1:
		fdiv = 1;
		break;
	case 2:
		fdiv = 2;
		break;
	case 3:
		fdiv = 4;
		break;
	case 4:
		fdiv = 8;
		break;
	case 5:
	case 6:
	case 7:
		fdiv = 16;
		break;
	default:
		LOG_ERR("Unknown ECG FDIV register value: %d", fdiv_reg_val);
		return -EINVAL;
	}

	if (ndiv_reg_val < 16){
		LOG_WRN("ECG NDIV value %d is less than minimum of 16. Using 16 instead.", ndiv_reg_val);
		ndiv_reg_val = 16; /* Minimum ndiv value is 16 */
	}
	
	ecg_adc_clk = pll_clk / (fdiv * ndiv_reg_val);
	if (ecg_adc_clk < MAX86178_ECG_ADC_CLK_MIN || ecg_adc_clk > MAX86178_ECG_ADC_CLK_MAX) {
		LOG_ERR("Invalid ECG ADC clock frequency: %d Hz. Must be between %d and %d Hz",
			ecg_adc_clk, MAX86178_ECG_ADC_CLK_MIN, MAX86178_ECG_ADC_CLK_MAX);
		return -EINVAL;
	}

	return 0;
}

static int max86178_set_ecg_dec_rate(const struct device *dev, enum max86178_ecg_dec_rate dec_rate)
{
	int ret;
	ret = max86178_reg_update(dev, MAX86178_ECG_CFG1, MAX86178_ECG_CFG1_ECG_DEC_RATE_MSK, dec_rate);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG Decimation Rate: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_set_ecg_clk_cfg(const struct device *dev, uint8_t fdiv_reg_val, uint16_t ndiv_reg_val, enum max86178_ecg_dec_rate dec_rate)
{
	int ret;
	uint32_t pll_clk;
	uint8_t reg_val;

	ret = max86178_get_pll_clk(dev, &pll_clk);
	if (ret < 0) {
		LOG_ERR("Failed to get PLL clock: %d", ret);
		return ret;
	}

	/* Validate ECG FDIV and NDIV */
	ret = validate_ecg_fdiv_ndiv(dev, pll_clk, fdiv_reg_val, ndiv_reg_val);
	if (ret < 0) {
		LOG_ERR("ECG FDIV/NDIV validation failed: %d", ret);
		return ret;
	}

	/* Set ECG FDIV */
	ret = max86178_set_ecg_fdiv(dev, fdiv_reg_val);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG FDIV: %d", ret);
		return ret;
	}

	/* Set ECG NDIV */
	ret = max86178_set_ecg_ndiv(dev, ndiv_reg_val);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG NDIV: %d", ret);
		return ret;
	}

	/* Set ECG Decimation Rate */
	ret = max86178_set_ecg_dec_rate(dev, dec_rate);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG Decimation Rate: %d", ret);
		return ret;
	}

	return 0;
}

static bool validate_bioz_ndiv(const struct device *dev, uint32_t pll_clk, uint8_t resp_en, uint8_t ecg_fdiv_reg_val, uint8_t bioz_ndiv_reg_val)
{
	int ret;
	uint32_t bioz_adc_clk;
	uint16_t bioz_ndiv = 0;
	switch (bioz_ndiv_reg_val) {
	case 0:
		bioz_ndiv = 256;
		break;
	case 1:
		bioz_ndiv = 512;
		break;
	case 2:	
		bioz_ndiv = 1024;
		break;
	case 3:
		bioz_ndiv = 1024;
		break;
	default:
		LOG_ERR("Unknown BIOZ NDIV register value: %d", bioz_ndiv_reg_val);
		return false;
	}

	if (resp_en == 0) {
		bioz_adc_clk = pll_clk / bioz_ndiv;
	} else {
		uint8_t ecg_f_div;

		switch (ecg_fdiv_reg_val) {
		case 0:
			LOG_ERR("ECG FDIV cannot be 0 when RESP_CFG1_RESP_EN is set");
			return false;
		case 1:
			ecg_f_div = 1;
			break;
		case 2:
			ecg_f_div = 2;
			break;
		case 3:
			ecg_f_div = 4;
			break;
		case 4:
			ecg_f_div = 8;
			break;
		case 5:
		case 6:
		case 7:
			ecg_f_div = 16;
			break;
		default:
			LOG_ERR("Unknown ECG FDIV register value: %d", ecg_fdiv_reg_val);
			return false;
		}
		bioz_adc_clk = pll_clk / (ecg_f_div * bioz_ndiv);
	}
	if (bioz_adc_clk < MAX86178_BIOZ_ADC_CLK_MIN || bioz_adc_clk > MAX86178_BIOZ_ADC_CLK_MAX) {
		LOG_ERR("Invalid BIOZ ADC clock frequency: %d Hz. Must be between %d and %d Hz",
			bioz_adc_clk, MAX86178_BIOZ_ADC_CLK_MIN, MAX86178_BIOZ_ADC_CLK_MAX);
		return false;
	}
	return true;
}

static int max86178_set_resp_en(const struct device *dev, bool enable)
{
	return max86178_reg_update(dev, MAX86178_RESP_CFG1, MAX86178_RESP_CFG1_RESP_EN_MSK, enable ? 1 : 0);
}

static int max86178_set_bioz_ndiv(const struct device *dev, uint8_t bioz_ndiv_reg_val)
{
	return max86178_reg_update(dev, MAX86178_PLL_CFG3, MAX86178_PLL_CFG3_BIOZ_NDIV_MSK, bioz_ndiv_reg_val);
}

static int max86178_set_bioz_adc_osr(const struct device *dev, uint8_t bioz_adc_osr_reg_val)
{
	return max86178_reg_update(dev, MAX86178_BIOZ_CFG1, MAX86178_BIOZ_CFG1_BIOZ_ADC_OSR_MSK, bioz_adc_osr_reg_val);
}

static int max86178_set_bioz_adc_clk_cfg(const struct device *dev, uint8_t ecg_fdiv_reg_val, uint8_t bioz_ndiv_reg_val, bool resp_en, uint8_t bioz_adc_osr_reg_val)
{
	int ret = 0;
	uint32_t pll_clk;
	uint8_t reg_val;

	/* Get PLL clock */
	ret = max86178_get_pll_clk(dev, &pll_clk);
	if (ret < 0) {
		LOG_ERR("Failed to get PLL clock: %d", ret);
		return ret;
	}

	/* Validate BIOZ NDIV */
	if (!validate_bioz_ndiv(dev, pll_clk, resp_en, ecg_fdiv_reg_val, bioz_ndiv_reg_val)) {
		LOG_ERR("BIOZ NDIV validation failed");
		return -EINVAL;
	}

	ret = max86178_set_resp_en(dev, resp_en);
	if (ret < 0) {
		LOG_ERR("Failed to set RESP_EN: %d", ret);
		return ret;
	}

	/* Set BIOZ NDIV */
	ret = max86178_set_bioz_ndiv(dev, bioz_ndiv_reg_val);
	if (ret < 0) {
		LOG_ERR("Failed to set BIOZ NDIV: %d", ret);
		return ret;
	}

	ret = max86178_set_bioz_adc_osr(dev, bioz_adc_osr_reg_val);
	if (ret < 0) {
		LOG_ERR("Failed to set BIOZ ADC OSR: %d", ret);
		return ret;
	}

	return 0;
}

static bool validate_bioz_kdiv(const struct device *dev, uint32_t pll_clk, bool resp_en, uint8_t ecg_fdiv_reg_val, uint8_t bioz_kdiv_reg_val)
{
	int ret;
	uint32_t bioz_synth_clk;
	uint8_t bioz_kdiv = 0;
	switch (bioz_kdiv_reg_val) {
	case 0:
		bioz_kdiv = 1;
		break;
	case 1:
		bioz_kdiv = 2;
		break;
	case 2:
		bioz_kdiv = 4;
		break;
	case 3:
		bioz_kdiv = 8;
		break;
	case 4:
		bioz_kdiv = 16;
		break;
	case 5:
		bioz_kdiv = 32;
		break;
	case 6:
		bioz_kdiv = 64;
		break;
	case 7:
		bioz_kdiv = 128;
		break;
	case 8:
		bioz_kdiv = 256;
		break;
	case 9:
		bioz_kdiv = 512;
		break;
	case 10:
		bioz_kdiv = 1024;
		break;
	case 11:
		bioz_kdiv = 2048;
		break;
	case 12:
		bioz_kdiv = 4096;
		break;
	case 13:
	case 14:
	case 15:
		bioz_kdiv = 8192;
		break;
	default:
		LOG_ERR("Unknown BIOZ KDIV register value: %d", bioz_kdiv_reg_val);
		return false;
	}

	if (resp_en == 0)
	{
		bioz_synth_clk = pll_clk / bioz_kdiv;
	} else {
		uint8_t ecg_f_div;

		switch (ecg_fdiv_reg_val) {
		case 0:
			LOG_ERR("ECG FDIV cannot be 0 when RESP_CFG1_RESP_EN is set");
			return false;
		case 1:
			ecg_f_div = 1;
			break;
		case 2:
			ecg_f_div = 2;
			break;
		case 3:
			ecg_f_div = 4;
			break;
		case 4:
			ecg_f_div = 8;
			break;
		case 5:
		case 6:
		case 7:
			ecg_f_div = 16;
			break;
		default:
			LOG_ERR("Unknown ECG FDIV register value: %d", ecg_fdiv_reg_val);
			return false;
		}
		bioz_synth_clk = pll_clk / (ecg_f_div * bioz_kdiv);
	}

	if (bioz_synth_clk < MAX86178_BIOZ_SYNTH_CLK_MIN || bioz_synth_clk > MAX86178_BIOZ_SYNTH_CLK_MAX) {
		LOG_ERR("Invalid BIOZ Synth clock frequency: %d Hz. Must be between %d and %d Hz",
			bioz_synth_clk, MAX86178_BIOZ_SYNTH_CLK_MIN, MAX86178_BIOZ_SYNTH_CLK_MAX);
		return false;
	}
	return true;

}

static int max86178_set_bioz_kdiv(const struct device *dev, uint8_t bioz_kdiv_reg_val)
{
	return max86178_reg_update(dev, MAX86178_PLL_CFG3, MAX86178_PLL_CFG3_BIOZ_KDIV_MSK, bioz_kdiv_reg_val);
}

static int max86178_set_bioz_dac_osr(const struct device *dev, uint8_t bioz_dac_osr_reg_val)
{
	return max86178_reg_update(dev, MAX86178_BIOZ_CFG1, MAX86178_BIOZ_CFG1_BIOZ_DAC_OSR_MSK, bioz_dac_osr_reg_val);
}

static int max86178_bioz_synth_clk_cfg(const struct device *dev, bool resp_en, uint8_t ecg_fdiv_reg_val, uint8_t bioz_kdiv_reg_val, uint8_t bioz_dac_osr_reg_val)
{
	int ret = 0;
	uint32_t pll_clk;
	uint8_t reg_val;

	/* Get PLL clock */
	ret = max86178_get_pll_clk(dev, &pll_clk);
	if (ret < 0) {
		LOG_ERR("Failed to get PLL clock: %d", ret);
		return ret;
	}
	/* Validate BIOZ KDIV */
	if (!validate_bioz_kdiv(dev, pll_clk, resp_en, ecg_fdiv_reg_val, bioz_kdiv_reg_val)) {
		LOG_ERR("BIOZ KDIV validation failed");
		return -EINVAL;
	}

	/* Set BIOZ KDIV */
	ret = max86178_set_bioz_kdiv(dev, bioz_kdiv_reg_val);
	if (ret < 0) {
		LOG_ERR("Failed to set BIOZ KDIV: %d", ret);
		return ret;
	}

	ret = max86178_set_bioz_dac_osr(dev, bioz_dac_osr_reg_val);
	if (ret < 0) {
		LOG_ERR("Failed to set BIOZ DAC OSR: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_clk_init(const struct device *dev)
{
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;
	/* Set PLL Configuration 6 - Reference Clock, Clock Frequency Selection, and Clock Fine Tune */
	ret = max86178_set_pll_cfg6(dev, config->clk_cfg.osc_cfg.ref_clk, config->clk_cfg.osc_cfg.clk_freq_sel,
				 config->clk_cfg.osc_cfg.clk_fine_tune);
	if (ret < 0) {
		LOG_ERR("Failed to set PLL CFG6: %d", ret);
		return ret;
	}

	/* Validate MDIV */
	ret = validate_mdiv(dev, config->clk_cfg.osc_cfg.mdiv);
	if (ret < 0) {
		LOG_ERR("MDIV validation failed: %d", ret);
		return ret;
	}

	/* Set MDIV */
	ret = max86178_set_mdiv(dev, config->clk_cfg.osc_cfg.mdiv);
	if (ret < 0) {
		LOG_ERR("Failed to set MDIV: %d", ret);
		return ret;
	}

	/* Set ECG Clock Configuration */
	ret = max86178_set_ecg_clk_cfg(dev, config->clk_cfg.ecg_cfg.ecg_fdiv, config->clk_cfg.ecg_cfg.ecg_ndiv,
					config->clk_cfg.ecg_cfg.ecg_dec_rate);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG clock configuration: %d", ret);
		return ret;
	}

	/* Set BIOZ ADC Clock Configuration */
	ret = max86178_set_bioz_adc_clk_cfg(dev, config->clk_cfg.ecg_cfg.ecg_fdiv, config->clk_cfg.bioz_cfg.bioz_ndiv, config->resp_cfg.resp_en, config->clk_cfg.bioz_cfg.bioz_adc_osr);
	if (ret < 0) {
		LOG_ERR("Failed to set BIOZ ADC clock configuration: %d", ret);
		return ret;
	}

	/* Set BIOZ Synth Clock Configuration */
	ret = max86178_bioz_synth_clk_cfg(dev, config->resp_cfg.resp_en, config->clk_cfg.ecg_cfg.ecg_fdiv, config->clk_cfg.bioz_cfg.bioz_kdiv, config->clk_cfg.bioz_cfg.bioz_dac_osr);
	if (ret < 0) {
		LOG_ERR("Failed to set BIOZ Synth clock configuration: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_fifo_init(const struct device *dev)
{
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;
	uint8_t reg_val;

	if (config->fifo_cfg.fifo_watermark == 0 || config->fifo_cfg.fifo_watermark > 256) {
		LOG_ERR("Invalid FIFO watermark level: %d. Must be between 1 and 256.", config->fifo_cfg.fifo_watermark);
		return -EINVAL;
	}
	/* Set FIFO Watermark */
	reg_val = 256 - config->fifo_cfg.fifo_watermark; /* FIFO counts down, so watermark is set as (256 - desired level) */
	ret = max86178_reg_write(dev, MAX86178_FIFO_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set FIFO watermark: %d", ret);
		return ret;
	}

	/* Set Rollover Enable */
	ret = max86178_reg_update(dev, MAX86178_FIFO_CFG2, MAX86178_FIFO_CFG2_FIFO_RO_MSK,
				 config->fifo_cfg.fifo_rollover_en ? 1 : 0);
	if (ret < 0) {
		LOG_ERR("Failed to set FIFO rollover enable: %d", ret);
		return ret;
	}

	/* Set Almost Full Type*/
	ret = max86178_reg_update(dev, MAX86178_FIFO_CFG2, MAX86178_FIFO_CFG2_A_FULL_TYPE_MSK,
				 config->fifo_cfg.fifo_a_full_type ? 1 : 0);
	if (ret < 0) {
		LOG_ERR("Failed to set FIFO almost full type: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_fr_clk_div_set(const struct device *dev, uint16_t fr_clk_div)
{
	int ret = 0;
	uint8_t reg_val;

	reg_val = FIELD_GET(MAX86178_FR_CLK_DIV_MSB_MSK, fr_clk_div);

	ret = max86178_reg_update(dev, MAX86178_FR_CLK_DIV_MSB, MAX86178_FR_CLK_DIV_MSB_MSK, reg_val);
	if (ret < 0) {
		LOG_ERR("Failed to set FR_CLK_DIV MSB: %d", ret);
		return ret;
	}

	reg_val = fr_clk_div & 0xFF;
	ret = max86178_reg_write(dev, MAX86178_FR_CLK_DIV_LSB, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set FR_CLK_DIV LSB: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_ppg_init(const struct device *dev)
{
	/* Implement PPG sensor initialization */
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;

	/* Set FR_CLK_DIV value */
	ret = max86178_fr_clk_div_set(dev, config->clk_cfg.ppg_cfg.ppg_fr_clk_div);
	if (ret < 0) {
		LOG_ERR("Failed to set PPG FR_CLK_DIV: %d", ret);
		return ret;
	}


	

	return 0;
}

static int max86178_soft_reset(const struct device *dev)
{
	/* Implement soft reset logic */
	int ret;
	/* Disable PLL for reset sequence */
	ret = max86178_reg_update(dev, MAX86178_PLL_CFG1, MAX86178_PLL_CFG1_PLL_EN_MSK, 0);
	if (ret < 0) {
		LOG_ERR("Failed to disable PLL: %d", ret);
		return ret;
	}

	/* Perform soft reset */
	ret = max86178_reg_update(dev, MAX86178_SYS_CFG1, MAX86178_SYS_CFG1_RESET_MSK, 1);
	if (ret < 0) {
		LOG_ERR("Failed to perform soft reset: %d", ret);
		return ret;
	}
	
	/* Enable PLL after reset sequence */
	ret = max86178_reg_update(dev, MAX86178_PLL_CFG1, MAX86178_PLL_CFG1_PLL_EN_MSK, 1);
	if (ret < 0) {
		LOG_ERR("Failed to enable PLL: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_init(const struct device *dev)
{
	const struct max86178_dev_config *config = dev->config;
	int ret;

	/* Check if bus is ready */
	ret = config->bus_is_ready(&config->bus);
	if (ret < 0) {
		LOG_ERR("Bus not ready");
		return ret;
	}

	/* Verify device ID */
	uint8_t part_id;
	ret = max86178_reg_read(dev, MAX86178_PART_ID, &part_id, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read PART ID");
		return ret;
	}
	if ((part_id & MAX86178_PART_ID_MSK) != MAX86178_PART_ID_VAL) {
		LOG_ERR("Unexpected PART ID: 0x%02X", part_id);
		return -ENODEV;
	}
	LOG_INF("Device ID verified: 0x%02X", part_id);

	/* Perform soft reset */
	ret = max86178_soft_reset(dev);
	if (ret < 0) {
		LOG_ERR("Device soft reset failed");
		return ret;
	}

	ret = max86178_osc_enable(dev, false);
	if (ret < 0) {
		LOG_ERR("Failed to disable internal oscillator: %d", ret);
		return ret;
	}
	/* Initialize clocks */
	ret = max86178_clk_init(dev);
	if (ret < 0) {
		LOG_ERR("Clock initialization failed");
		return ret;
	}

	/* Initialize FIFO */
	ret = max86178_fifo_init(dev);
	if (ret < 0) {
		LOG_ERR("FIFO initialization failed");
		return ret;
	}

	/* Initialize PPG */
	ret = max86178_ppg_init(dev);
	if (ret < 0) {
		LOG_ERR("PPG initialization failed");
		return ret;
	}

	LOG_INF("MAX86178 initialized");
	return 0;
}
/*******************************************************************************
 * CLOCK CONFIGURATION PARSING MACROS
 ******************************************************************************/

/* Clock Oscillator Configuration */
#define MAX86178_CLK_OSC_CFG(inst)                                                                 \
	{                                                                                          \
		.ref_clk = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), clk), osc),            \
				      ref_clk_sel, 0),                                             \
		.clk_freq_sel = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), clk), osc),       \
					   clk_freq_sel, 0),                                       \
		.clk_fine_tune = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), clk), osc),      \
					    clk_fine_tune, 0),                                     \
		.mdiv = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), clk), osc), mdiv, 0),     \
	}

/* PPG Clock Configuration */
#define MAX86178_CLK_PPG_CFG(inst)                                                                 \
	{                                                                                          \
		.ppg_fr_clk_div =                                                                  \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), clk), ppg), fr_clk_div,   \
				   0),                                                             \
	}

/* ECG Clock Configuration */
#define MAX86178_CLK_ECG_CFG(inst)                                                                 \
	{                                                                                          \
		.ecg_fdiv = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), clk), ecg),           \
				       ecg_fdiv, 0),                                               \
		.ecg_ndiv = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), clk), ecg),           \
				       ecg_ndiv, 0),                                               \
		.ecg_dec_rate = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), clk), ecg),       \
					   ecg_dec_rate, 0),                                       \
	}

/* BioZ Clock Configuration */
#define MAX86178_CLK_BIOZ_CFG(inst)                                                                \
	{                                                                                          \
		.bioz_kdiv = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), clk), bioz),         \
					bioz_kdiv, 0),                                             \
		.bioz_ndiv = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), clk), bioz),         \
					bioz_ndiv, 0),                                             \
		.bioz_adc_osr = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), clk), bioz),      \
					   bioz_adc_osr, 0),                                       \
		.bioz_dac_osr = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), clk), bioz),      \
					   bioz_dac_osr, 0),                                       \
	}

/* Complete Clock Configuration */
#define MAX86178_CLK_CFG(inst)                                                                     \
	{                                                                                          \
		.osc_cfg = MAX86178_CLK_OSC_CFG(inst),                                             \
		.ppg_cfg = MAX86178_CLK_PPG_CFG(inst),                                             \
		.ecg_cfg = MAX86178_CLK_ECG_CFG(inst),                                             \
		.bioz_cfg = MAX86178_CLK_BIOZ_CFG(inst),                                           \
	}

/*******************************************************************************
 * FIFO CONFIGURATION PARSING MACROS
 ******************************************************************************/

 #define MAX86178_FIFO_CFG(inst)                                                                    \
	{                                                                                          \
		.fifo_watermark = DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), fifo), fifo_watermark, 256), \
		.fifo_rollover_en = DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), fifo), fifo_rollover_en, 0), \
		.fifo_a_full_type = DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), fifo), fifo_a_full_type, 0), \
	}

/*******************************************************************************
 * PPG CONFIGURATION PARSING MACROS
 ******************************************************************************/

/* PPG Measurement Slot Configuration - for a specific measurement index */
#define MAX86178_PPG_MEAS_CFG_BY_IDX(inst, idx)                                                    \
	{                                                                                          \
		.drva = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx),        \
				   drva, 0),                                                       \
		.drvb = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx),        \
				   drvb, 0),                                                       \
		.amb_mode = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx),    \
				       amb_mode, 0),                                               \
		.avg_num = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx),     \
				      avg_num, 0),                                                 \
		.sinc3_sel = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx),   \
					sinc3_sel, 0),                                             \
		.filt_sel = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx),    \
				       filt_sel, 0),                                               \
		.filt2_sel = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx),   \
					filt2_sel, 0),                                             \
		.ppg1_dac_off =                                                                    \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx),        \
				   ppg1_dac_off, 0),                                               \
		.ppg1_adc_rge =                                                                    \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx),        \
				   ppg1_adc_rge, 0),                                               \
		.ppg2_dac_off =                                                                    \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx),        \
				   ppg2_dac_off, 0),                                               \
		.ppg2_adc_rge =                                                                    \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx),        \
				   ppg2_adc_rge, 0),                                               \
		.tint = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx),        \
				   tint, 0),                                                       \
		.pd_setlng = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx),   \
					pd_setlng, 0),                                             \
		.led_setlng =                                                                      \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx),        \
				   led_setlng, 0),                                                 \
		.led_rge = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx),     \
				      led_rge, 0),                                                 \
		.pd1_sel = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx),     \
				      pd1_sel, 1),                                                 \
		.pd2_sel = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx),     \
				      pd2_sel, 1),                                                 \
		.pd3_sel = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx),     \
				      pd3_sel, 1),                                                 \
		.pd4_sel = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx),     \
				      pd4_sel, 1),                                                 \
		.drva_pa = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx),     \
				      drva_pa, 0),                                                 \
		.drvb_pa = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx),     \
				      drvb_pa, 0),                                                 \
	}

/* PPG Threshold Configuration */
#define MAX86178_PPG_THRESHOLD_CFG(inst)                                                           \
	{                                                                                          \
		.thresh1_meas_sel =                                                                \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), threshold),         \
				   thresh1_meas_sel, 0),                                           \
		.thresh2_meas_sel =                                                                \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), threshold),         \
				   thresh2_meas_sel, 0),                                           \
		.thresh1_chan_sel =                                                                \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), threshold),         \
				   thresh1_chan_sel, 0),                                           \
		.thresh2_chan_sel =                                                                \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), threshold),         \
				   thresh2_chan_sel, 0),                                           \
		.time_hyst = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), threshold),    \
					time_hyst, 0),                                             \
		.level_hyst = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), threshold),   \
					 level_hyst, 0),                                           \
		.thresh1_hi = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), threshold),   \
					 thresh1_hi, 0),                                           \
		.thresh1_lo = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), threshold),   \
					 thresh1_lo, 0),                                           \
		.thresh2_hi = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), threshold),   \
					 thresh2_hi, 0),                                           \
		.thresh2_lo = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), threshold),   \
					 thresh2_lo, 0),                                           \
	}

/* Complete PPG Configuration */
#define MAX86178_PPG_CFG(inst)                                                                     \
	{                                                                                          \
		.meas_en = {                                                                       \
			DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), meas1_en, false),             \
			DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), meas2_en, false),             \
			DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), meas3_en, false),             \
			DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), meas4_en, false),             \
			DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), meas5_en, false),             \
			DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), meas6_en, false),             \
		},                                                                                 \
		.ppg1_pwrdn = DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), ppg1_pwrdn, 0),        \
		.ppg2_pwrdn = DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), ppg2_pwrdn, 0),        \
		.ppg_sync_mode =                                                                   \
			DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), ppg_sync_mode, 0),           \
		.prox_data_en = DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), prox_data_en,        \
					   false),                                                 \
		.prox_auto_en = DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), prox_auto_en,        \
					   false),                                                 \
		.alc_disable = DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), alc_disable, false),  \
		.collect_raw_data =                                                                \
			DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), collect_raw_data, false),    \
		.meas1_config_sel =                                                                \
			DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), meas1_config_sel, false),    \
		.smp_ave = DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), smp_ave, 0),              \
		.pd1_bias = DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), pd1_bias, 1),            \
		.pd2_bias = DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), pd2_bias, 1),            \
		.pd3_bias = DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), pd3_bias, 1),            \
		.pd4_bias = DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), pd4_bias, 1),            \
		.meas_cfg = {                                                                      \
			MAX86178_PPG_MEAS_CFG_BY_IDX(inst, 1),                                     \
			MAX86178_PPG_MEAS_CFG_BY_IDX(inst, 2),                                     \
			MAX86178_PPG_MEAS_CFG_BY_IDX(inst, 3),                                     \
			MAX86178_PPG_MEAS_CFG_BY_IDX(inst, 4),                                     \
			MAX86178_PPG_MEAS_CFG_BY_IDX(inst, 5),                                     \
			MAX86178_PPG_MEAS_CFG_BY_IDX(inst, 6),                                     \
		},                                                                                 \
		.threshold_cfg = MAX86178_PPG_THRESHOLD_CFG(inst),                                 \
	}

/*******************************************************************************
 * ECG CONFIGURATION PARSING MACROS
 ******************************************************************************/

/* ECG Setup Configuration */
#define MAX86178_ECG_SETUP_CFG(inst)                                                               \
	{                                                                                          \
		.ecg_en = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), setup), ecg_en,   \
				     false),                                                       \
		.ecg_input_pol =                                                                   \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), setup),             \
				   ecg_input_pol, 0),                                              \
		.ecg_pga_gain =                                                                    \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), setup),             \
				   ecg_pga_gain, 0),                                               \
		.ecg_ina_rge = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), setup),      \
					  ecg_ina_rge, 0),                                         \
		.ecg_ina_gain = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), setup),     \
					   ecg_ina_gain, 0),                                       \
		.ecg_imp_hi = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), setup),       \
					 ecg_imp_hi, false),                                       \
		.ecg_auto_rec = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), setup),     \
					   ecg_auto_rec, 0),                                       \
		.ecg_mux_sel = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), setup),      \
					  ecg_mux_sel, 0),                                         \
		.en_ecg_fast_rec =                                                                 \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), setup),             \
				   en_ecg_fast_rec, 0),                                            \
		.ecg_fast_rec_thres =                                                              \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), setup),             \
				   ecg_fast_rec_thres, 0),                                         \
	}

/* ECG Calibration Configuration */
#define MAX86178_ECG_CAL_CFG(inst)                                                                 \
	{                                                                                          \
		.ecg_cal_high =                                                                    \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), calibration),       \
				   ecg_cal_high, 0),                                               \
		.ecg_freq = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), calibration),   \
				       ecg_cal_freq, 0),                                           \
		.ecg_cal_duty =                                                                    \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), calibration),       \
				   ecg_cal_duty, 0),                                               \
		.ecg_cal_en =                                                                      \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), calibration),       \
				   ecg_cal_en, 0),                                                 \
		.ecg_open_p =                                                                      \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), calibration),       \
				   ecg_open_p, 0),                                                 \
		.ecg_open_n =                                                                      \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), calibration),       \
				   ecg_open_n, 0),                                                 \
		.ecg_cal_mode =                                                                    \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), calibration),       \
				   ecg_cal_mode, 0),                                               \
		.ecg_cal_mag =                                                                     \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), calibration),       \
				   ecg_cal_mag, 0),                                                \
		.ecg_cal_p_sel =                                                                   \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), calibration),       \
				   ecg_cal_p_sel, 0),                                              \
		.ecg_cal_n_sel =                                                                   \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), calibration),       \
				   ecg_cal_n_sel, 0),                                              \
	}

/* ECG Lead Detect Configuration */
#define MAX86178_ECG_LEAD_DETECT_CFG(inst)                                                         \
	{                                                                                          \
		.en_ecg_lon =                                                                      \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), lead_detect),       \
				   en_ecg_lon, 0),                                                 \
		.en_ecg_loff =                                                                     \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), lead_detect),       \
				   en_ecg_loff, 0),                                                \
		.ecg_loff_mode =                                                                   \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), lead_detect),       \
				   ecg_loff_mode, 0),                                              \
		.ecg_loff_freq =                                                                   \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), lead_detect),       \
				   ecg_loff_freq, 0),                                              \
		.ecg_loff_ipol =                                                                   \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), lead_detect),       \
				   ecg_loff_ipol, 0),                                              \
		.ecg_loff_current_mag =                                                            \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), lead_detect),       \
				   ecg_loff_current_mag, 0),                                       \
		.ecg_loff_thresh =                                                                 \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), lead_detect),       \
				   ecg_loff_thresh, 0),                                            \
	}

/* ECG Lead Bias Configuration */
#define MAX86178_ECG_LEAD_BIAS_CFG(inst)                                                           \
	{                                                                                          \
		.ecg_rbias_value =                                                                 \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), lead_bias),         \
				   ecg_rbias_value, 0),                                            \
		.en_ecg_rbias_p =                                                                  \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), lead_bias),         \
				   en_ecg_rbias_p, 0),                                             \
		.en_ecg_rbias_n =                                                                  \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), lead_bias),         \
				   en_ecg_rbias_n, 0),                                             \
	}

/* ECG RLD Configuration */
#define MAX86178_ECG_RLD_CFG(inst)                                                                 \
	{                                                                                          \
		.rld_en = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), rld), rld_en,     \
				     0),                                                           \
		.rld_mode = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), rld),           \
				       rld_mode, 0),                                               \
		.rld_rbias = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), rld),          \
					rld_rbias, 0),                                             \
		.en_rld_oor = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), rld),         \
					 en_rld_oor, 0),                                           \
		.actv_cm_p = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), rld),          \
					actv_cm_p, 0),                                             \
		.actv_cm_n = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), rld),          \
					actv_cm_n, 0),                                             \
		.rld_gain = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), rld),           \
				       rld_gain, 0),                                               \
		.rld_ext_res = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), rld),        \
					  rld_ext_res, false),                                     \
		.rld_sel_ecg = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), rld),        \
					  rld_sel_ecg, false),                                     \
		.rld_bw = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), rld), rld_bw,     \
				     0),                                                           \
		.body_bias_dac = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), rld),      \
					    body_bias_dac, 0),                                     \
	}

/* Complete ECG Configuration */
#define MAX86178_ECG_CFG(inst)                                                                     \
	{                                                                                          \
		.setup = MAX86178_ECG_SETUP_CFG(inst),                                             \
		.calibration = MAX86178_ECG_CAL_CFG(inst),                                         \
		.lead_detect = MAX86178_ECG_LEAD_DETECT_CFG(inst),                                 \
		.lead_bias = MAX86178_ECG_LEAD_BIAS_CFG(inst),                                     \
		.rld_cfg = MAX86178_ECG_RLD_CFG(inst),                                             \
	}

/*******************************************************************************
 * BIOZ CONFIGURATION PARSING MACROS
 ******************************************************************************/

/* BioZ Setup Configuration */
#define MAX86178_BIOZ_SETUP_CFG(inst)                                                              \
	{                                                                                          \
		.ecg_bioz_bg_en =                                                                  \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),            \
				   ecg_bioz_bg_en, 0),                                             \
		.bioz_en = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),         \
				      bioz_en, 0),                                                 \
		.bioz_dhpf = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),       \
					bioz_dhpf, 0),                                             \
		.bioz_dlpf = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),       \
					bioz_dlpf, 0),                                             \
		.en_bioz_thresh =                                                                  \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),            \
				   en_bioz_thresh, 0),                                             \
		.bioz_ext_res =                                                                    \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),            \
				   bioz_ext_res, 0),                                               \
		.bioz_vdrv_mag =                                                                   \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),            \
				   bioz_vdrv_mag, 0),                                              \
		.bioz_idrv_rge =                                                                   \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),            \
				   bioz_idrv_rge, 0),                                              \
		.bioz_drv_mode =                                                                   \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),            \
				   bioz_drv_mode, 0),                                              \
		.en_util_mode =                                                                    \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),            \
				   en_util_mode, false),                                           \
		.bioz_dc_code_sel =                                                                \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),            \
				   bioz_dc_code_sel, 0),                                           \
		.bioz_dc_dac_code =                                                                \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),            \
				   bioz_dc_dac_code, 0),                                           \
		.bioz_ahpf = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),       \
					bioz_ahpf, 0),                                             \
		.bioz_ina_mode =                                                                   \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),            \
				   bioz_ina_mode, 0),                                              \
		.bioz_dm_dis = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),     \
					  bioz_dm_dis, 0),                                         \
		.bioz_gain = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),       \
					bioz_gain, 0),                                             \
		.bioz_ext_cap =                                                                    \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),            \
				   bioz_ext_cap, 0),                                               \
		.bioz_dc_restore =                                                                 \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),            \
				   bioz_dc_restore, 0),                                            \
		.bioz_drv_reset =                                                                  \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),            \
				   bioz_drv_reset, 0),                                             \
		.bioz_dac_reset =                                                                  \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),            \
				   bioz_dac_reset, false),                                         \
		.bioz_amp_rge =                                                                    \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),            \
				   bioz_amp_rge, 0),                                               \
		.bioz_amp_bw = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),     \
					  bioz_amp_bw, 0),                                         \
		.rld_sel_bioz =                                                                    \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),            \
				   rld_sel_bioz, false),                                           \
		.rld_drv = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),         \
				      rld_drv, 0),                                                 \
		.bioz_cmres_dis =                                                                  \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),            \
				   bioz_cmres_dis, 0),                                             \
		.bioz_stbyon = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),     \
					  bioz_stbyon, 0),                                         \
		.bioz_ipol = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),       \
					bioz_ipol, 0),                                             \
		.bioz_fast = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),       \
					bioz_fast, 0),                                             \
		.bioz_ina_chop_en =                                                                \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),            \
				   bioz_ina_chop_en, 0),                                           \
		.bioz_ch_fsel =                                                                    \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),            \
				   bioz_ch_fsel, 0),                                               \
		.bioz_lo_thresh =                                                                  \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),            \
				   bioz_lo_thresh, 0),                                             \
		.bioz_hi_thresh =                                                                  \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),            \
				   bioz_hi_thresh, 0),                                             \
	}

/* BioZ Calibration Configuration */
#define MAX86178_BIOZ_CAL_CFG(inst)                                                                \
	{                                                                                          \
		.bmux_rsel =                                                                       \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), calibration),      \
				   bmux_rsel, 0),                                                  \
		.bmux_bist_en =                                                                    \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), calibration),      \
				   bmux_bist_en, 0),                                               \
		.connect_cal_only =                                                                \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), calibration),      \
				   connect_cal_only, false),                                       \
		.bioz_mux_en =                                                                     \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), calibration),      \
				   bioz_mux_en, false),                                            \
		.bioz_cal_en =                                                                     \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), calibration),      \
				   bioz_cal_en, false),                                            \
		.bmux_gsr_rsel =                                                                   \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), calibration),      \
				   bmux_gsr_rsel, 0),                                              \
		.gsr_load_en =                                                                     \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), calibration),      \
				   gsr_load_en, 0),                                                \
		.en_ext_inload =                                                                   \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), calibration),      \
				   en_ext_inload, 0),                                              \
		.en_int_inload =                                                                   \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), calibration),      \
				   en_int_inload, 0),                                              \
		.bip_assign =                                                                      \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), calibration),      \
				   bip_assign, 0),                                                 \
		.bin_assign =                                                                      \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), calibration),      \
				   bin_assign, 0),                                                 \
		.drvp_assign =                                                                     \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), calibration),      \
				   drvp_assign, 0),                                                \
		.drvn_assign =                                                                     \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), calibration),      \
				   drvn_assign, 0),                                                \
	}

/* BioZ Lead Detect Configuration */
#define MAX86178_BIOZ_LEAD_DETECT_CFG(inst)                                                        \
	{                                                                                          \
		.en_bioz_lon =                                                                     \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), lead_detect),      \
				   en_bioz_lon, 0),                                                \
		.en_bioz_loff =                                                                    \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), lead_detect),      \
				   en_bioz_loff, 0),                                               \
		.en_ext_bioz_loff =                                                                \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), lead_detect),      \
				   en_ext_bioz_loff, 0),                                           \
		.en_bioz_drv_oor =                                                                 \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), lead_detect),      \
				   en_bioz_drv_oor, 0),                                            \
		.bioz_loff_ipol =                                                                  \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), lead_detect),      \
				   bioz_loff_ipol, false),                                         \
		.bioz_loff_current_mag =                                                           \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), lead_detect),      \
				   bioz_loff_current_mag, 0),                                      \
		.resp_cg_mag4x =                                                                   \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), lead_detect),      \
				   resp_cg_mag4x, false),                                          \
		.resp_cg_mag =                                                                     \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), lead_detect),      \
				   resp_cg_mag, 0),                                                \
		.bioz_loff_thresh =                                                                \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), lead_detect),      \
				   bioz_loff_thresh, 0),                                           \
	}

/* BioZ Lead Bias Configuration */
#define MAX86178_BIOZ_LEAD_BIAS_CFG(inst)                                                          \
	{                                                                                          \
		.en_bioz_rbias_p =                                                                 \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), lead_bias),        \
				   en_bioz_rbias_p, false),                                        \
		.en_bioz_rbias_n =                                                                 \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), lead_bias),        \
				   en_bioz_rbias_n, false),                                        \
		.bioz_rbias_value =                                                                \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), lead_bias),        \
				   bioz_rbias_value, 0),                                           \
	}

/* Complete BioZ Configuration */
#define MAX86178_BIOZ_CFG(inst)                                                                    \
	{                                                                                          \
		.setup = MAX86178_BIOZ_SETUP_CFG(inst),                                            \
		.calibration = MAX86178_BIOZ_CAL_CFG(inst),                                        \
		.lead_detect = MAX86178_BIOZ_LEAD_DETECT_CFG(inst),                                \
		.lead_bias = MAX86178_BIOZ_LEAD_BIAS_CFG(inst),                                    \
	}

/*******************************************************************************
 * RESPIRATION CONFIGURATION PARSING MACROS
 ******************************************************************************/

#define MAX86178_RESP_CFG(inst)                                                                    \
	{                                                                                          \
		.resp_en = DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), resp), resp_en, false),         \
		.cg_mode = DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), resp), cg_mode, 0),             \
		.cg_chop_clk = DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), resp), cg_chop_clk, 0),     \
		.cg_lpf_duty = DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), resp), cg_lpf_duty, 0),     \
	}

/*******************************************************************************
 * DEVICE INSTANTIATION MACROS
 ******************************************************************************/

/* Device instantiation macros */
#define MAX86178_CONFIG_I2C(inst)                                                                  \
	{                                                                                          \
		.bus = {.i2c = I2C_DT_SPEC_INST_GET(inst)},                                        \
		.bus_is_ready = max86178_bus_check_i2c,                                            \
		.reg_access = max86178_reg_access_i2c,                                             \
		.clk_cfg = MAX86178_CLK_CFG(inst),                                                 \
		.ppg_cfg = MAX86178_PPG_CFG(inst),                                                 \
		.ecg_cfg = MAX86178_ECG_CFG(inst),                                                 \
		.bioz_cfg = MAX86178_BIOZ_CFG(inst),                                               \
		.resp_cfg = MAX86178_RESP_CFG(inst),                                               \
	}

#define MAX86178_CONFIG_SPI(inst)                                                                  \
	{                                                                                          \
		.bus = {.spi = SPI_DT_SPEC_INST_GET(                                               \
				inst, SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB,    \
				0)},                                                               \
		.bus_is_ready = max86178_bus_check_spi,                                            \
		.reg_access = max86178_reg_access_spi,                                             \
		.clk_cfg = MAX86178_CLK_CFG(inst),                                                 \
		.ppg_cfg = MAX86178_PPG_CFG(inst),                                                 \
		.ecg_cfg = MAX86178_ECG_CFG(inst),                                                 \
		.bioz_cfg = MAX86178_BIOZ_CFG(inst),                                               \
		.fifo_cfg = MAX86178_FIFO_CFG(inst),                                               \
		.resp_cfg = MAX86178_RESP_CFG(inst),                                               \
	}

#define MAX86178_DEFINE(inst)                                                                      \
	static struct max86178_data max86178_data_##inst;                                          \
	static const struct max86178_dev_config max86178_config_##inst =                           \
		COND_CODE_1(DT_INST_ON_BUS(inst, i2c), (MAX86178_CONFIG_I2C(inst)),                \
			    (MAX86178_CONFIG_SPI(inst)));                                          \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, max86178_init, NULL, &max86178_data_##inst,            \
				     &max86178_config_##inst, POST_KERNEL,                         \
				     CONFIG_SENSOR_INIT_PRIORITY, &max86178_api);

DT_INST_FOREACH_STATUS_OKAY(MAX86178_DEFINE)
