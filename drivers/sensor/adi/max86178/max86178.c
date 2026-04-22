/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_max86178

#include "max86178.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(MAX86178, CONFIG_SENSOR_LOG_LEVEL);

/* I2C bus operations */
#if defined(MAX86178_BUS_I2C)
static int max86178_bus_check_i2c(const union max86178_bus *bus)
{
	if (!i2c_is_ready_dt(&bus->i2c)) {
		LOG_ERR("I2C bus not ready");
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
		LOG_ERR("SPI bus not ready");
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

/* Helper function to convert sensor channel to MEAS index (0-5) */
static int max86178_chan_to_meas_idx(enum sensor_channel chan, uint8_t *meas_idx)
{
	switch ((int)chan) {
	case SENSOR_CHAN_PPG_MEAS1:
		*meas_idx = 0;
		return 0;
	case SENSOR_CHAN_PPG_MEAS2:
		*meas_idx = 1;
		return 0;
	case SENSOR_CHAN_PPG_MEAS3:
		*meas_idx = 2;
		return 0;
	case SENSOR_CHAN_PPG_MEAS4:
		*meas_idx = 3;
		return 0;
	case SENSOR_CHAN_PPG_MEAS5:
		*meas_idx = 4;
		return 0;
	case SENSOR_CHAN_PPG_MEAS6:
		*meas_idx = 5;
		return 0;
	default:
		return -EINVAL;
	}
}

/* Helper function to get the base register address for a measurement */
static uint8_t max86178_meas_base_reg(uint8_t meas_idx)
{
	return MAX86178_MEAS1_SEL + (meas_idx * 8);
}

/*******************************************************************************
 * PPG THRESHOLD CONFIGURATION ATTRIBUTES
 ******************************************************************************/

static int max86178_set_ppg_thresh1_hi(const struct device *dev, const struct sensor_value *val)
{
	int ret;
	uint8_t thresh_hi = val->val1 & 0xFF;

	ret = max86178_reg_write(dev, MAX86178_PPG_HI_THRESH1, &thresh_hi, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set PPG HI threshold 1: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ppg_thresh1_hi(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_PPG_HI_THRESH1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read PPG HI threshold 1: %d", ret);
		return ret;
	}

	val->val1 = reg_val;
	val->val2 = 0;
	return 0;
}

static int max86178_set_ppg_thresh1_lo(const struct device *dev, const struct sensor_value *val)
{
	int ret;
	uint8_t thresh_lo = val->val1 & 0xFF;

	ret = max86178_reg_write(dev, MAX86178_PPG_LO_THRESH1, &thresh_lo, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set PPG LO threshold 1: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ppg_thresh1_lo(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_PPG_LO_THRESH1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read PPG LO threshold 1: %d", ret);
		return ret;
	}

	val->val1 = reg_val;
	val->val2 = 0;
	return 0;
}

static int max86178_set_ppg_thresh2_hi(const struct device *dev, const struct sensor_value *val)
{
	int ret;
	uint8_t thresh_hi = val->val1 & 0xFF;

	ret = max86178_reg_write(dev, MAX86178_PPG_HI_THRESH2, &thresh_hi, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set PPG HI threshold 2: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ppg_thresh2_hi(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_PPG_HI_THRESH2, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read PPG HI threshold 2: %d", ret);
		return ret;
	}

	val->val1 = reg_val;
	val->val2 = 0;
	return 0;
}

static int max86178_set_ppg_thresh2_lo(const struct device *dev, const struct sensor_value *val)
{
	int ret;
	uint8_t thresh_lo = val->val1 & 0xFF;

	ret = max86178_reg_write(dev, MAX86178_PPG_LO_THRESH2, &thresh_lo, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set PPG LO threshold 2: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ppg_thresh2_lo(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_PPG_LO_THRESH2, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read PPG LO threshold 2: %d", ret);
		return ret;
	}

	val->val1 = reg_val;
	val->val2 = 0;
	return 0;
}

static int max86178_set_ppg_thresh1_meas_sel(const struct device *dev,
					     const struct sensor_value *val)
{
	int ret;

	/* Validate enum range (0 = DISABLED, 1-6 = MEAS1-6) */
	if (val->val1 < MAX86178_PPG_THRESH_DISABLED || val->val1 > MAX86178_PPG_THRESH_MEAS6) {
		LOG_ERR("Invalid thresh1 meas sel value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_THRESH_MEAS_SEL,
				  MAX86178_THRESH_MEAS_SEL_THRESH1_MEAS_SEL_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set PPG thresh1 meas sel: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ppg_thresh1_meas_sel(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_THRESH_MEAS_SEL, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read PPG thresh1 meas sel: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_THRESH_MEAS_SEL_THRESH1_MEAS_SEL_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_ppg_thresh2_meas_sel(const struct device *dev,
					     const struct sensor_value *val)
{
	int ret;

	/* Validate enum range (0 = DISABLED, 1-6 = MEAS1-6) */
	if (val->val1 < MAX86178_PPG_THRESH_DISABLED || val->val1 > MAX86178_PPG_THRESH_MEAS6) {
		LOG_ERR("Invalid thresh2 meas sel value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_THRESH_MEAS_SEL,
				  MAX86178_THRESH_MEAS_SEL_THRESH2_MEAS_SEL_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set PPG thresh2 meas sel: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ppg_thresh2_meas_sel(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_THRESH_MEAS_SEL, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read PPG thresh2 meas sel: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_THRESH_MEAS_SEL_THRESH2_MEAS_SEL_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_ppg_thresh1_chan_sel(const struct device *dev,
					     const struct sensor_value *val)
{
	int ret;

	/* Validate enum range (0-1: PPG_CHAN1, PPG_CHAN2) */
	if (val->val1 < MAX86178_PPG_THRESH_PPG_CHAN1 ||
	    val->val1 > MAX86178_PPG_THRESH_PPG_CHAN2) {
		LOG_ERR("Invalid thresh1 chan sel value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_THRESH_HYST,
				  MAX86178_THRESH_HYST_THRESH1_PPG_SEL_MSK, (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set PPG thresh1 chan sel: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ppg_thresh1_chan_sel(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_THRESH_HYST, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read PPG thresh1 chan sel: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_THRESH_HYST_THRESH1_PPG_SEL_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_ppg_thresh2_chan_sel(const struct device *dev,
					     const struct sensor_value *val)
{
	int ret;

	/* Validate enum range (0-1: PPG_CHAN1, PPG_CHAN2) */
	if (val->val1 < MAX86178_PPG_THRESH_PPG_CHAN1 ||
	    val->val1 > MAX86178_PPG_THRESH_PPG_CHAN2) {
		LOG_ERR("Invalid thresh2 chan sel value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_THRESH_HYST,
				  MAX86178_THRESH_HYST_THRESH2_PPG_SEL_MSK, (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set PPG thresh2 chan sel: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ppg_thresh2_chan_sel(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_THRESH_HYST, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read PPG thresh2 chan sel: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_THRESH_HYST_THRESH2_PPG_SEL_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_ppg_time_hyst(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate enum range (0-3: DISABLED, 2, 4, 8 samples) */
	if (val->val1 < MAX86178_PPG_TIME_HYST_DISABLED ||
	    val->val1 > MAX86178_PPG_TIME_HYST_8_SAMPLES) {
		LOG_ERR("Invalid time hyst value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_THRESH_HYST, MAX86178_THRESH_HYST_TIME_HYST_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set PPG time hyst: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ppg_time_hyst(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_THRESH_HYST, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read PPG time hyst: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_THRESH_HYST_TIME_HYST_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_ppg_level_hyst(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate enum range (0-7: DISABLED, 2, 4, 8, 16, 32, 64, 128 samples) */
	if (val->val1 < MAX86178_PPG_LEVEL_HYST_DISABLED ||
	    val->val1 > MAX86178_PPG_LEVEL_HYST_128_SAMPLES) {
		LOG_ERR("Invalid level hyst value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_THRESH_HYST, MAX86178_THRESH_HYST_LEVEL_HYST_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set PPG level hyst: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ppg_level_hyst(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_THRESH_HYST, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read PPG level hyst: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_THRESH_HYST_LEVEL_HYST_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/*******************************************************************************
 * PPG CONFIGURATION ATTRIBUTES
 ******************************************************************************/

static int max86178_set_ppg1_pwrdn(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	if (val->val1 < MAX86178_PPG_CHAN_ENABLED || val->val1 > MAX86178_PPG_CHAN_PWRDN) {
		LOG_ERR("Invalid PPG1 power down value: %d", val->val1);
		return -EINVAL;
	}
	ret = max86178_reg_update(dev, MAX86178_PPG_CFG2, MAX86178_PPG_CFG2_PPG1_PWRDN_MSK,
				  val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set PPG1 power down: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_get_ppg1_pwrdn(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_PPG_CFG2, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read PPG1 power down: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_PPG_CFG2_PPG1_PWRDN_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_ppg2_pwrdn(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate enum range (0 = ENABLED, 1 = PWRDN) */
	if (val->val1 < MAX86178_PPG_CHAN_ENABLED || val->val1 > MAX86178_PPG_CHAN_PWRDN) {
		LOG_ERR("Invalid PPG2 power down value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_PPG_CFG2, MAX86178_PPG_CFG2_PPG2_PWRDN_MSK,
				  val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set PPG2 power down: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_get_ppg2_pwrdn(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_PPG_CFG2, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read PPG2 power down: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_PPG_CFG2_PPG2_PWRDN_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_ppg_sync_mode(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate enum range (0 = INTERNAL_SYNC, 1 = EXTERNAL_SYNC) */
	if (val->val1 < MAX86178_PPG_INTERNAL_SYNC || val->val1 > MAX86178_PPG_EXTERNAL_SYNC) {
		LOG_ERR("Invalid PPG sync mode value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_PPG_CFG2, MAX86178_PPG_CFG2_PPG_SYNC_MODE_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set PPG sync mode: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ppg_sync_mode(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_PPG_CFG2, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read PPG sync mode: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_PPG_CFG2_PPG_SYNC_MODE_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_prox_data_en(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate bool range (0-1) */
	if (val->val1 < 0 || val->val1 > 1) {
		LOG_ERR("Invalid proximity data enable value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_PPG_CFG4, MAX86178_PPG_CFG4_PROX_DATA_EN_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set proximity data enable: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_prox_data_en(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_PPG_CFG4, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read proximity data enable: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_PPG_CFG4_PROX_DATA_EN_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_prox_auto_en(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate bool range (0-1) */
	if (val->val1 < 0 || val->val1 > 1) {
		LOG_ERR("Invalid proximity auto enable value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_PPG_CFG4, MAX86178_PPG_CFG4_PROX_AUTO_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set proximity auto enable: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_prox_auto_en(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_PPG_CFG4, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read proximity auto enable: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_PPG_CFG4_PROX_AUTO_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_alc_disable(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate bool range (0-1) */
	if (val->val1 < 0 || val->val1 > 1) {
		LOG_ERR("Invalid ALC disable value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_PPG_CFG3, MAX86178_PPG_CFG3_ALC_DISABLE_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set ALC disable: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_alc_disable(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_PPG_CFG3, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read ALC disable: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_PPG_CFG3_ALC_DISABLE_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_collect_raw_data(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate bool range (0-1) */
	if (val->val1 < 0 || val->val1 > 1) {
		LOG_ERR("Invalid collect raw data value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_PPG_CFG3, MAX86178_PPG_CFG3_COLLECT_RAW_DATA_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set collect raw data: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_collect_raw_data(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_PPG_CFG3, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read collect raw data: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_PPG_CFG3_COLLECT_RAW_DATA_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_meas1_config_sel(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate bool range (0-1) */
	if (val->val1 < 0 || val->val1 > 1) {
		LOG_ERR("Invalid MEAS1 config sel value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_PPG_CFG3, MAX86178_PPG_CFG3_MEAS1_CONFIG_SEL_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set MEAS1 config sel: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_meas1_config_sel(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_PPG_CFG3, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read MEAS1 config select: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_PPG_CFG3_MEAS1_CONFIG_SEL_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_pd_bias(const struct device *dev, const struct sensor_value *val,
				uint8_t pd_num)
{
	uint8_t mask;
	int ret;

	/* Validate enum range (0-3: NOT_RECOMMENDED, 0-125pF, 125-250pF, 250-500pF) */
	if (val->val1 < MAX86178_PPG_PD_BIAS_NOT_RECOMMENDED ||
	    val->val1 > MAX86178_PPG_PD_BIAS_250pF_TO_500pF) {
		LOG_ERR("Invalid PD%d bias value: %d", pd_num, val->val1);
		return -EINVAL;
	}

	switch (pd_num) {
	case 1:
		mask = MAX86178_PD_BIAS_PD1_MSK;
		break;
	case 2:
		mask = MAX86178_PD_BIAS_PD2_MSK;
		break;
	case 3:
		mask = MAX86178_PD_BIAS_PD3_MSK;
		break;
	case 4:
		mask = MAX86178_PD_BIAS_PD4_MSK;
		break;
	default:
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_PD_BIAS, mask, (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set PD%d bias: %d", pd_num, ret);
		return ret;
	}

	return 0;
}

static int max86178_get_pd_bias(const struct device *dev, struct sensor_value *val, uint8_t pd_num)
{
	uint8_t reg_val;
	uint8_t mask;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_PD_BIAS, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read PD%d bias: %d", pd_num, ret);
		return ret;
	}

	switch (pd_num) {
	case 1:
		mask = MAX86178_PD_BIAS_PD1_MSK;
		break;
	case 2:
		mask = MAX86178_PD_BIAS_PD2_MSK;
		break;
	case 3:
		mask = MAX86178_PD_BIAS_PD3_MSK;
		break;
	case 4:
		mask = MAX86178_PD_BIAS_PD4_MSK;
		break;
	default:
		return -EINVAL;
	}

	val->val1 = FIELD_GET(mask, reg_val);
	val->val2 = 0;
	return 0;
}
/*******************************************************************************
 * NUMERIC VALUE CONVERSION HELPERS
 *
 * These functions convert between real-world numeric values (Hz, µA, mV, etc.)
 * and hardware register enum values.
 ******************************************************************************/

/* PPG ADC Range: Convert µA to enum (4, 8, 16, 32 µA) */
static int ppg_adc_range_to_enum(int32_t microamps, uint8_t *enum_val)
{
	switch (microamps) {
	case 4:
		*enum_val = MAX86178_PPG_ADC_RGE_4uA;
		return 0;
	case 8:
		*enum_val = MAX86178_PPG_ADC_RGE_8uA;
		return 0;
	case 16:
		*enum_val = MAX86178_PPG_ADC_RGE_16uA;
		return 0;
	case 32:
		*enum_val = MAX86178_PPG_ADC_RGE_32uA;
		return 0;
	default:
		return -EINVAL;
	}
}

static int ppg_adc_range_from_enum(uint8_t enum_val, int32_t *microamps)
{
	switch (enum_val) {
	case MAX86178_PPG_ADC_RGE_4uA:
		*microamps = 4;
		return 0;
	case MAX86178_PPG_ADC_RGE_8uA:
		*microamps = 8;
		return 0;
	case MAX86178_PPG_ADC_RGE_16uA:
		*microamps = 16;
		return 0;
	case MAX86178_PPG_ADC_RGE_32uA:
		*microamps = 32;
		return 0;
	default:
		return -EINVAL;
	}
}

static int ppg_led_range_to_enum(int32_t microamps, uint8_t *enum_val)
{
	switch (microamps) {
	case 32000:
		*enum_val = MAX86178_PPG_LED_RGE_32mA;
		return 0;
	case 64000:
		*enum_val = MAX86178_PPG_LED_RGE_64mA;
		return 0;
	case 96000:
		*enum_val = MAX86178_PPG_LED_RGE_96mA;
		return 0;
	case 128000:
		*enum_val = MAX86178_PPG_LED_RGE_128mA;
		return 0;
	default:
		return -EINVAL;
	}
}

static int ppg_led_range_from_enum(uint8_t enum_val, int32_t *microamps)
{
	switch (enum_val) {
	case MAX86178_PPG_LED_RGE_32mA:
		*microamps = 32000;
		return 0;
	case MAX86178_PPG_LED_RGE_64mA:
		*microamps = 64000;
		return 0;
	case MAX86178_PPG_LED_RGE_96mA:
		*microamps = 96000;
		return 0;
	case MAX86178_PPG_LED_RGE_128mA:
		*microamps = 128000;
		return 0;
	default:
		return -EINVAL;
	}
}

static int smp_ave_to_enum(int32_t samples, uint8_t *enum_val)
{
	switch (samples) {
	case 1:
		*enum_val = MAX86178_SMP_AVE_1;
		return 0;
	case 2:
		*enum_val = MAX86178_SMP_AVE_2;
		return 0;
	case 4:
		*enum_val = MAX86178_SMP_AVE_4;
		return 0;
	case 8:
		*enum_val = MAX86178_SMP_AVE_8;
		return 0;
	case 16:
		*enum_val = MAX86178_SMP_AVE_16;
		return 0;
	default:
		return -EINVAL;
	}
}

static int smp_ave_from_enum(uint8_t enum_val, int32_t *samples)
{
	switch (enum_val) {
	case MAX86178_SMP_AVE_1:
		*samples = 1;
		return 0;
	case MAX86178_SMP_AVE_2:
		*samples = 2;
		return 0;
	case MAX86178_SMP_AVE_4:
		*samples = 4;
		return 0;
	case MAX86178_SMP_AVE_8:
		*samples = 8;
		return 0;
	case MAX86178_SMP_AVE_16:
		*samples = 16;
		return 0;
	default:
		return -EINVAL;
	}
}

static int ppg_avg_num_to_enum(int32_t samples, uint8_t *enum_val)
{
	switch (samples) {
	case 1:
		*enum_val = MAX86178_PPG_AVG_NUM_1;
		return 0;
	case 2:
		*enum_val = MAX86178_PPG_AVG_NUM_2;
		return 0;
	case 4:
		*enum_val = MAX86178_PPG_AVG_NUM_4;
		return 0;
	case 8:
		*enum_val = MAX86178_PPG_AVG_NUM_8;
		return 0;
	case 16:
		*enum_val = MAX86178_PPG_AVG_NUM_16;
		return 0;
	case 32:
		*enum_val = MAX86178_PPG_AVG_NUM_32;
		return 0;
	case 64:
		*enum_val = MAX86178_PPG_AVG_NUM_64;
		return 0;
	case 128:
		*enum_val = MAX86178_PPG_AVG_NUM_128;
		return 0;
	default:
		return -EINVAL;
	}
}

static int ppg_avg_num_from_enum(uint8_t enum_val, int32_t *samples)
{
	switch (enum_val) {
	case MAX86178_PPG_AVG_NUM_1:
		*samples = 1;
		return 0;
	case MAX86178_PPG_AVG_NUM_2:
		*samples = 2;
		return 0;
	case MAX86178_PPG_AVG_NUM_4:
		*samples = 4;
		return 0;
	case MAX86178_PPG_AVG_NUM_8:
		*samples = 8;
		return 0;
	case MAX86178_PPG_AVG_NUM_16:
		*samples = 16;
		return 0;
	case MAX86178_PPG_AVG_NUM_32:
		*samples = 32;
		return 0;
	case MAX86178_PPG_AVG_NUM_64:
		*samples = 64;
		return 0;
	case MAX86178_PPG_AVG_NUM_128:
		*samples = 128;
		return 0;
	default:
		return -EINVAL;
	}
}

static int ecg_pga_gain_to_enum(int32_t gain, uint8_t *enum_val)
{
	switch (gain) {
	case 1:
		*enum_val = MAX86178_ECG_PGA_GAIN_1;
		return 0;
	case 2:
		*enum_val = MAX86178_ECG_PGA_GAIN_2;
		return 0;
	case 4:
		*enum_val = MAX86178_ECG_PGA_GAIN_4;
		return 0;
	case 8:
		*enum_val = MAX86178_ECG_PGA_GAIN_8;
		return 0;
	default:
		return -EINVAL;
	}
}

static int ecg_pga_gain_from_enum(uint8_t enum_val, int32_t *gain)
{
	switch (enum_val) {
	case MAX86178_ECG_PGA_GAIN_1:
		*gain = 1;
		return 0;
	case MAX86178_ECG_PGA_GAIN_2:
		*gain = 2;
		return 0;
	case MAX86178_ECG_PGA_GAIN_4:
		*gain = 4;
		return 0;
	case MAX86178_ECG_PGA_GAIN_8:
		*gain = 8;
		return 0;
	default:
		return -EINVAL;
	}
}

static int ecg_loff_freq_to_enum(int32_t hz, uint8_t *enum_val)
{
	switch (hz) {
	case 0:
		*enum_val = MAX86178_ECG_LOFF_FREQ_DISABLED;
		return 0;
	case 4:
		*enum_val = MAX86178_ECG_LOFF_FREQ_4Hz;
		return 0;
	case 8:
		*enum_val = MAX86178_ECG_LOFF_FREQ_8Hz;
		return 0;
	case 16:
		*enum_val = MAX86178_ECG_LOFF_FREQ_16Hz;
		return 0;
	case 32:
		*enum_val = MAX86178_ECG_LOFF_FREQ_32Hz;
		return 0;
	case 64:
		*enum_val = MAX86178_ECG_LOFF_FREQ_64Hz;
		return 0;
	case 128:
		*enum_val = MAX86178_ECG_LOFF_FREQ_128Hz;
		return 0;
	case 256:
		*enum_val = MAX86178_ECG_LOFF_FREQ_256Hz;
		return 0;
	default:
		return -EINVAL;
	}
}

static int ecg_loff_freq_from_enum(uint8_t enum_val, int32_t *hz)
{
	switch (enum_val) {
	case MAX86178_ECG_LOFF_FREQ_DISABLED:
		*hz = 0;
		return 0;
	case MAX86178_ECG_LOFF_FREQ_4Hz:
		*hz = 4;
		return 0;
	case MAX86178_ECG_LOFF_FREQ_8Hz:
		*hz = 8;
		return 0;
	case MAX86178_ECG_LOFF_FREQ_16Hz:
		*hz = 16;
		return 0;
	case MAX86178_ECG_LOFF_FREQ_32Hz:
		*hz = 32;
		return 0;
	case MAX86178_ECG_LOFF_FREQ_64Hz:
		*hz = 64;
		return 0;
	case MAX86178_ECG_LOFF_FREQ_128Hz:
		*hz = 128;
		return 0;
	case MAX86178_ECG_LOFF_FREQ_256Hz:
		*hz = 256;
		return 0;
	default:
		return -EINVAL;
	}
}

static int ecg_loff_imag_to_enum(int32_t nanoamps, uint8_t *enum_val)
{
	switch (nanoamps) {
	case 0:
		*enum_val = MAX86178_ECG_LOFF_IMAG_0nA;
		return 0;
	case 5:
		*enum_val = MAX86178_ECG_LOFF_IMAG_5nA;
		return 0;
	case 10:
		*enum_val = MAX86178_ECG_LOFF_IMAG_10nA;
		return 0;
	case 20:
		*enum_val = MAX86178_ECG_LOFF_IMAG_20nA;
		return 0;
	case 50:
		*enum_val = MAX86178_ECG_LOFF_IMAG_50nA;
		return 0;
	case 100:
		*enum_val = MAX86178_ECG_LOFF_IMAG_100nA;
		return 0;
	case 200:
		*enum_val = MAX86178_ECG_LOFF_IMAG_200nA;
		return 0;
	case 400:
		*enum_val = MAX86178_ECG_LOFF_IMAG_400nA;
		return 0;
	default:
		return -EINVAL;
	}
}

static int ecg_loff_imag_from_enum(uint8_t enum_val, int32_t *nanoamps)
{
	switch (enum_val) {
	case MAX86178_ECG_LOFF_IMAG_0nA:
		*nanoamps = 0;
		return 0;
	case MAX86178_ECG_LOFF_IMAG_5nA:
		*nanoamps = 5;
		return 0;
	case MAX86178_ECG_LOFF_IMAG_10nA:
		*nanoamps = 10;
		return 0;
	case MAX86178_ECG_LOFF_IMAG_20nA:
		*nanoamps = 20;
		return 0;
	case MAX86178_ECG_LOFF_IMAG_50nA:
		*nanoamps = 50;
		return 0;
	case MAX86178_ECG_LOFF_IMAG_100nA:
		*nanoamps = 100;
		return 0;
	case MAX86178_ECG_LOFF_IMAG_200nA:
		*nanoamps = 200;
		return 0;
	case MAX86178_ECG_LOFF_IMAG_400nA:
		*nanoamps = 400;
		return 0;
	default:
		return -EINVAL;
	}
}

static int bioz_adc_osr_to_enum(int32_t osr, uint8_t *enum_val)
{
	switch (osr) {
	case 8:
		*enum_val = MAX86178_BIOZ_ADC_OSR_8;
		return 0;
	case 16:
		*enum_val = MAX86178_BIOZ_ADC_OSR_16;
		return 0;
	case 32:
		*enum_val = MAX86178_BIOZ_ADC_OSR_32;
		return 0;
	case 64:
		*enum_val = MAX86178_BIOZ_ADC_OSR_64;
		return 0;
	case 128:
		*enum_val = MAX86178_BIOZ_ADC_OSR_128;
		return 0;
	case 256:
		*enum_val = MAX86178_BIOZ_ADC_OSR_256;
		return 0;
	case 512:
		*enum_val = MAX86178_BIOZ_ADC_OSR_512;
		return 0;
	case 1024:
		*enum_val = MAX86178_BIOZ_ADC_OSR_1024;
		return 0;
	default:
		return -EINVAL;
	}
}

static int bioz_adc_osr_from_enum(uint8_t enum_val, int32_t *osr)
{
	switch (enum_val) {
	case MAX86178_BIOZ_ADC_OSR_8:
		*osr = 8;
		return 0;
	case MAX86178_BIOZ_ADC_OSR_16:
		*osr = 16;
		return 0;
	case MAX86178_BIOZ_ADC_OSR_32:
		*osr = 32;
		return 0;
	case MAX86178_BIOZ_ADC_OSR_64:
		*osr = 64;
		return 0;
	case MAX86178_BIOZ_ADC_OSR_128:
		*osr = 128;
		return 0;
	case MAX86178_BIOZ_ADC_OSR_256:
		*osr = 256;
		return 0;
	case MAX86178_BIOZ_ADC_OSR_512:
		*osr = 512;
		return 0;
	case MAX86178_BIOZ_ADC_OSR_1024:
		*osr = 1024;
		return 0;
	default:
		return -EINVAL;
	}
}

static int bioz_dac_osr_to_enum(int32_t osr, uint8_t *enum_val)
{
	switch (osr) {
	case 32:
		*enum_val = MAX86178_BIOZ_DAC_OSR_32;
		return 0;
	case 64:
		*enum_val = MAX86178_BIOZ_DAC_OSR_64;
		return 0;
	case 128:
		*enum_val = MAX86178_BIOZ_DAC_OSR_128;
		return 0;
	case 256:
		*enum_val = MAX86178_BIOZ_DAC_OSR_256;
		return 0;
	default:
		return -EINVAL;
	}
}

static int bioz_dac_osr_from_enum(uint8_t enum_val, int32_t *osr)
{
	switch (enum_val) {
	case MAX86178_BIOZ_DAC_OSR_32:
		*osr = 32;
		return 0;
	case MAX86178_BIOZ_DAC_OSR_64:
		*osr = 64;
		return 0;
	case MAX86178_BIOZ_DAC_OSR_128:
		*osr = 128;
		return 0;
	case MAX86178_BIOZ_DAC_OSR_256:
		*osr = 256;
		return 0;
	default:
		return -EINVAL;
	}
}

static int bioz_vdrv_mag_to_enum(int32_t microvolts, uint8_t *enum_val)
{
	switch (microvolts) {
	case 50000:
		*enum_val = MAX86178_BIOZ_VDRV_MAG_50mV;
		return 0;
	case 100000:
		*enum_val = MAX86178_BIOZ_VDRV_MAG_100mV;
		return 0;
	case 250000:
		*enum_val = MAX86178_BIOZ_VDRV_MAG_250mV;
		return 0;
	case 500000:
		*enum_val = MAX86178_BIOZ_VDRV_MAG_500mV;
		return 0;
	default:
		return -EINVAL;
	}
}

static int bioz_vdrv_mag_from_enum(uint8_t enum_val, int32_t *microvolts)
{
	switch (enum_val) {
	case MAX86178_BIOZ_VDRV_MAG_50mV:
		*microvolts = 50000;
		return 0;
	case MAX86178_BIOZ_VDRV_MAG_100mV:
		*microvolts = 100000;
		return 0;
	case MAX86178_BIOZ_VDRV_MAG_250mV:
		*microvolts = 250000;
		return 0;
	case MAX86178_BIOZ_VDRV_MAG_500mV:
		*microvolts = 500000;
		return 0;
	default:
		return -EINVAL;
	}
}

static int bioz_gain_to_enum(const struct sensor_value *val, uint8_t *enum_val)
{
	/* Check for exact matches first */
	if (val->val1 == 1 && val->val2 == 0) {
		*enum_val = MAX86178_BIOZ_GAIN_1;
		return 0;
	} else if (val->val1 == 2 && val->val2 == 0) {
		*enum_val = MAX86178_BIOZ_GAIN_2;
		return 0;
	} else if (val->val1 == 5 && val->val2 == 0) {
		*enum_val = MAX86178_BIOZ_GAIN_5;
		return 0;
	} else if (val->val1 == 10 && val->val2 == 0) {
		*enum_val = MAX86178_BIOZ_GAIN_10;
		return 0;
	}

	return -EINVAL;
}

static int bioz_gain_from_enum(uint8_t enum_val, struct sensor_value *val)
{
	switch (enum_val) {
	case MAX86178_BIOZ_GAIN_1:
		val->val1 = 1;
		val->val2 = 0;
		return 0;
	case MAX86178_BIOZ_GAIN_2:
		val->val1 = 2;
		val->val2 = 500000;
		return 0;
	case MAX86178_BIOZ_GAIN_5:
		val->val1 = 5;
		val->val2 = 0;
		return 0;
	case MAX86178_BIOZ_GAIN_10:
		val->val1 = 10;
		val->val2 = 0;
		return 0;
	default:
		return -EINVAL;
	}
}

static int ecg_rbias_value_to_enum(int32_t megaohms, uint8_t *enum_val)
{
	switch (megaohms) {
	case 50:
		*enum_val = MAX86178_ECG_RBIAS_50M;
		return 0;
	case 100:
		*enum_val = MAX86178_ECG_RBIAS_100M;
		return 0;
	case 200:
		*enum_val = MAX86178_ECG_RBIAS_200M;
		return 0;
	default:
		return -EINVAL;
	}
}

static int ecg_rbias_value_from_enum(uint8_t enum_val, int32_t *megaohms)
{
	switch (enum_val) {
	case MAX86178_ECG_RBIAS_50M:
		*megaohms = 50;
		return 0;
	case MAX86178_ECG_RBIAS_100M:
		*megaohms = 100;
		return 0;
	case MAX86178_ECG_RBIAS_200M:
		*megaohms = 200;
		return 0;
	default:
		return -EINVAL;
	}
}

static int rld_gain_to_enum(int32_t gain, uint8_t *enum_val)
{
	switch (gain) {
	case 12:
		*enum_val = MAX86178_RLD_GAIN_12;
		return 0;
	case 24:
		*enum_val = MAX86178_RLD_GAIN_24;
		return 0;
	case 48:
		*enum_val = MAX86178_RLD_GAIN_48;
		return 0;
	case 97:
		*enum_val = MAX86178_RLD_GAIN_97;
		return 0;
	default:
		return -EINVAL;
	}
}

static int rld_gain_from_enum(uint8_t enum_val, int32_t *gain)
{
	switch (enum_val) {
	case MAX86178_RLD_GAIN_12:
		*gain = 12;
		return 0;
	case MAX86178_RLD_GAIN_24:
		*gain = 24;
		return 0;
	case MAX86178_RLD_GAIN_48:
		*gain = 48;
		return 0;
	case MAX86178_RLD_GAIN_97:
		*gain = 97;
		return 0;
	default:
		return -EINVAL;
	}
}

static int bioz_rbias_value_to_enum(int32_t megaohms, uint8_t *enum_val)
{
	switch (megaohms) {
	case 50:
		*enum_val = MAX86178_BIOZ_RBIAS_50M;
		return 0;
	case 100:
		*enum_val = MAX86178_BIOZ_RBIAS_100M;
		return 0;
	case 200:
		*enum_val = MAX86178_BIOZ_RBIAS_200M;
		return 0;
	default:
		return -EINVAL;
	}
}

static int bioz_rbias_value_from_enum(uint8_t enum_val, int32_t *megaohms)
{
	switch (enum_val) {
	case MAX86178_BIOZ_RBIAS_50M:
		*megaohms = 50;
		return 0;
	case MAX86178_BIOZ_RBIAS_100M:
		*megaohms = 100;
		return 0;
	case MAX86178_BIOZ_RBIAS_200M:
		*megaohms = 200;
		return 0;
	default:
		return -EINVAL;
	}
}

static int bmux_rsel_to_enum(int32_t ohms, uint8_t *enum_val)
{
	switch (ohms) {
	case 200:
		*enum_val = MAX86178_BMUX_RSEL_200_OHM;
		return 0;
	case 500:
		*enum_val = MAX86178_BMUX_RSEL_500_OHM;
		return 0;
	case 800:
		*enum_val = MAX86178_BMUX_RSEL_800_OHM;
		return 0;
	case 5000:
		*enum_val = MAX86178_BMUX_RSEL_5000_OHM;
		return 0;
	default:
		return -EINVAL;
	}
}

static int bmux_rsel_from_enum(uint8_t enum_val, int32_t *ohms)
{
	switch (enum_val) {
	case MAX86178_BMUX_RSEL_200_OHM:
		*ohms = 200;
		return 0;
	case MAX86178_BMUX_RSEL_500_OHM:
		*ohms = 500;
		return 0;
	case MAX86178_BMUX_RSEL_800_OHM:
		*ohms = 800;
		return 0;
	case MAX86178_BMUX_RSEL_5000_OHM:
		*ohms = 5000;
		return 0;
	default:
		return -EINVAL;
	}
}

static int bmux_gsr_rsel_to_enum(int32_t kiloohms, uint8_t *enum_val)
{
	switch (kiloohms) {
	case 25:
		*enum_val = MAX86178_BMUX_GSR_RSEL_25K;
		return 0;
	case 100:
		*enum_val = MAX86178_BMUX_GSR_RSEL_100K;
		return 0;
	case 500:
		*enum_val = MAX86178_BMUX_GSR_RSEL_500K;
		return 0;
	case 1000:
		*enum_val = MAX86178_BMUX_GSR_RSEL_1000K;
		return 0;
	default:
		return -EINVAL;
	}
}

static int bmux_gsr_rsel_from_enum(uint8_t enum_val, int32_t *kiloohms)
{
	switch (enum_val) {
	case MAX86178_BMUX_GSR_RSEL_25K:
		*kiloohms = 25;
		return 0;
	case MAX86178_BMUX_GSR_RSEL_100K:
		*kiloohms = 100;
		return 0;
	case MAX86178_BMUX_GSR_RSEL_500K:
		*kiloohms = 500;
		return 0;
	case MAX86178_BMUX_GSR_RSEL_1000K:
		*kiloohms = 1000;
		return 0;
	default:
		return -EINVAL;
	}
}

static int max86178_set_smp_ave(const struct device *dev, const struct sensor_value *val)
{
	uint8_t enum_val;
	int ret;

	ret = smp_ave_to_enum(val->val1, &enum_val);
	if (ret < 0) {
		LOG_ERR("Invalid sample average value: %d (valid: 1, 2, 4, 8, 16)", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_PPG_CFG3, MAX86178_PPG_CFG3_SMP_AVE_MSK, enum_val);
	if (ret < 0) {
		LOG_ERR("Failed to set sample average: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_smp_ave(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int32_t samples;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_PPG_CFG3, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read sample average: %d", ret);
		return ret;
	}

	ret = smp_ave_from_enum(FIELD_GET(MAX86178_PPG_CFG3_SMP_AVE_MSK, reg_val), &samples);
	if (ret < 0) {
		return ret;
	}

	val->val1 = samples;
	val->val2 = 0;
	return 0;
}

static int max86178_set_ppg_drva(const struct device *dev, enum sensor_channel chan,
				 const struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	int ret;

	if (val->val1 < MAX86178_PPG_DRVA_LED1_DRV || val->val1 >= MAX86178_PPG_DRVA_COUNT) {
		LOG_ERR("Invalid DRVA value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_update(dev, base_reg + 0, MAX86178_MEAS_SEL_DRVA_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set MEAS%d DRVA: %d", meas_idx + 1, ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ppg_drva(const struct device *dev, enum sensor_channel chan,
				 struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	uint8_t reg_val;
	int ret;

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_read(dev, base_reg + 0, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read MEAS%d DRVA: %d", meas_idx + 1, ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_MEAS_SEL_DRVA_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_ppg_drvb(const struct device *dev, enum sensor_channel chan,
				 const struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	int ret;

	if (val->val1 < MAX86178_PPG_DRVB_LED1_DRV || val->val1 >= MAX86178_PPG_DRVB_COUNT) {
		LOG_ERR("Invalid DRVB value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_update(dev, base_reg + 0, MAX86178_MEAS_SEL_DRVB_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set MEAS%d DRVB: %d", meas_idx + 1, ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ppg_drvb(const struct device *dev, enum sensor_channel chan,
				 struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	uint8_t reg_val;
	int ret;

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_read(dev, base_reg + 0, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read MEAS%d DRVB: %d", meas_idx + 1, ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_MEAS_SEL_DRVB_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_ppg_drva_pa(const struct device *dev, enum sensor_channel chan,
				    const struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	uint8_t pa_value;
	int ret;

	if (val->val1 < 0x00 || val->val1 > 0xFF) {
		LOG_ERR("Invalid DRVA PA value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	pa_value = (uint8_t)val->val1;
	ret = max86178_reg_write(dev, base_reg + 6, &pa_value, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set MEAS%d DRVA PA: %d", meas_idx + 1, ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ppg_drva_pa(const struct device *dev, enum sensor_channel chan,
				    struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	uint8_t reg_val;
	int ret;

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_read(dev, base_reg + 6, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read MEAS%d DRVA PA: %d", meas_idx + 1, ret);
		return ret;
	}

	val->val1 = reg_val;
	val->val2 = 0;
	return 0;
}

static int max86178_set_ppg_drvb_pa(const struct device *dev, enum sensor_channel chan,
				    const struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	uint8_t pa_value;
	int ret;

	/* Validate range (0x00-0xFF) */
	if (val->val1 < 0x00 || val->val1 > 0xFF) {
		LOG_ERR("Invalid DRVB PA value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	pa_value = (uint8_t)val->val1;
	ret = max86178_reg_write(dev, base_reg + 7, &pa_value, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set MEAS%d DRVB PA: %d", meas_idx + 1, ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ppg_drvb_pa(const struct device *dev, enum sensor_channel chan,
				    struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	uint8_t reg_val;
	int ret;

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_read(dev, base_reg + 7, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read MEAS%d DRVB PA: %d", meas_idx + 1, ret);
		return ret;
	}

	val->val1 = reg_val;
	val->val2 = 0;
	return 0;
}

static int max86178_set_ppg_amb_mode(const struct device *dev, enum sensor_channel chan,
				     const struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	int ret;

	/* Validate enum range */
	if (val->val1 < MAX86178_PPG_AMB_NORMAL_MODE ||
	    val->val1 > MAX86178_PPG_AMB_DIRECT_AMB_CONV) {
		LOG_ERR("Invalid AMB mode value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_update(dev, base_reg + 0, MAX86178_MEAS_SEL_AMB_MSK, (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set MEAS%d AMB mode: %d", meas_idx + 1, ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ppg_amb_mode(const struct device *dev, enum sensor_channel chan,
				     struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	uint8_t reg_val;
	int ret;

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_read(dev, base_reg + 0, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read MEAS%d AMB mode: %d", meas_idx + 1, ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_MEAS_SEL_AMB_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_ppg_avg_num(const struct device *dev, enum sensor_channel chan,
				    const struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	uint8_t enum_val;
	int ret;

	ret = ppg_avg_num_to_enum(val->val1, &enum_val);
	if (ret < 0) {
		LOG_ERR("Invalid AVG NUM value: %d (valid: 1, 2, 4, 8, 16, 32, 64, 128)",
			val->val1);
		return -EINVAL;
	}

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_update(dev, base_reg + 1, MAX86178_MEAS_CFG1_AVER_MSK, enum_val);
	if (ret < 0) {
		LOG_ERR("Failed to set MEAS%d AVG NUM: %d", meas_idx + 1, ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ppg_avg_num(const struct device *dev, enum sensor_channel chan,
				    struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	uint8_t reg_val;
	int32_t samples;
	int ret;

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_read(dev, base_reg + 1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read MEAS%d AVG NUM: %d", meas_idx + 1, ret);
		return ret;
	}

	ret = ppg_avg_num_from_enum(FIELD_GET(MAX86178_MEAS_CFG1_AVER_MSK, reg_val), &samples);
	if (ret < 0) {
		return ret;
	}

	val->val1 = samples;
	val->val2 = 0;
	return 0;
}

static int max86178_set_ppg_sinc3_sel(const struct device *dev, enum sensor_channel chan,
				      const struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	int ret;

	if (val->val1 < MAX86178_PPG_SINC3_OFF || val->val1 > MAX86178_PPG_SINC3_ON) {
		LOG_ERR("Invalid SINC3 SEL value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_update(dev, base_reg + 1, MAX86178_MEAS_CFG1_SINC3_SEL_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set MEAS%d SINC3 SEL: %d", meas_idx + 1, ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ppg_sinc3_sel(const struct device *dev, enum sensor_channel chan,
				      struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	uint8_t reg_val;
	int ret;

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_read(dev, base_reg + 1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read MEAS%d SINC3 SEL: %d", meas_idx + 1, ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_MEAS_CFG1_SINC3_SEL_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_ppg_filt_sel(const struct device *dev, enum sensor_channel chan,
				     const struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	int ret;

	if (val->val1 < MAX86178_PPG_FILT_SEL_CDM || val->val1 > MAX86178_PPG_FILT_SEL_FDM) {
		LOG_ERR("Invalid FILT SEL value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_update(dev, base_reg + 1, MAX86178_MEAS_CFG1_FILT_SEL_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set MEAS%d FILT SEL: %d", meas_idx + 1, ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ppg_filt_sel(const struct device *dev, enum sensor_channel chan,
				     struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	uint8_t reg_val;
	int ret;

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_read(dev, base_reg + 1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read MEAS%d FILT SEL: %d", meas_idx + 1, ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_MEAS_CFG1_FILT_SEL_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_ppg_filt2_sel(const struct device *dev, enum sensor_channel chan,
				      const struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	int ret;

	if (val->val1 < MAX86178_PPG_FILT2_3RD_ORDER || val->val1 > MAX86178_PPG_FILT2_2ND_ORDER) {
		LOG_ERR("Invalid FILT2 SEL value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_update(dev, base_reg + 1, MAX86178_MEAS_CFG1_FILT2_SEL_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set MEAS%d FILT2 SEL: %d", meas_idx + 1, ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ppg_filt2_sel(const struct device *dev, enum sensor_channel chan,
				      struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	uint8_t reg_val;
	int ret;

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_read(dev, base_reg + 1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read MEAS%d FILT2 SEL: %d", meas_idx + 1, ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_MEAS_CFG1_FILT2_SEL_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_ppg_tint(const struct device *dev, enum sensor_channel chan,
				 const struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	int ret;

	if (val->val1 < MAX86178_PPG_TINT_14_6us || val->val1 > MAX86178_PPG_TINT_117_0us) {
		LOG_ERR("Invalid TINT value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_update(dev, base_reg + 1, MAX86178_MEAS_CFG1_TINT_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set MEAS%d TINT: %d", meas_idx + 1, ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ppg_tint(const struct device *dev, enum sensor_channel chan,
				 struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	uint8_t reg_val;
	int ret;

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_read(dev, base_reg + 1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read MEAS%d TINT: %d", meas_idx + 1, ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_MEAS_CFG1_TINT_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_ppg1_adc_rge(const struct device *dev, enum sensor_channel chan,
				     const struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	uint8_t enum_val;
	int ret;

	/* Convert µA value (in val2) to enum */
	ret = ppg_adc_range_to_enum(val->val2, &enum_val);
	if (ret < 0) {
		LOG_ERR("Invalid PPG1 ADC range: %d µA (valid: 4, 8, 16, 32)", val->val2);
		return -EINVAL;
	}

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_update(dev, base_reg + 2, MAX86178_MEAS_CFG2_PPG1_ADC_RGE_MSK, enum_val);
	if (ret < 0) {
		LOG_ERR("Failed to set MEAS%d PPG1 ADC RGE: %d", meas_idx + 1, ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ppg1_adc_rge(const struct device *dev, enum sensor_channel chan,
				     struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	uint8_t reg_val;
	int32_t microamps;
	int ret;

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_read(dev, base_reg + 2, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read MEAS%d PPG1 ADC RGE: %d", meas_idx + 1, ret);
		return ret;
	}

	ret = ppg_adc_range_from_enum(FIELD_GET(MAX86178_MEAS_CFG2_PPG1_ADC_RGE_MSK, reg_val),
				      &microamps);
	if (ret < 0) {
		return ret;
	}

	val->val1 = 0;
	val->val2 = microamps;
	return 0;
}

static int max86178_set_ppg2_adc_rge(const struct device *dev, enum sensor_channel chan,
				     const struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	uint8_t enum_val;
	int ret;

	ret = ppg_adc_range_to_enum(val->val2, &enum_val);
	if (ret < 0) {
		LOG_ERR("Invalid PPG2 ADC range: %d µA (valid: 4, 8, 16, 32)", val->val2);
		return -EINVAL;
	}

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_update(dev, base_reg + 2, MAX86178_MEAS_CFG2_PPG2_ADC_RGE_MSK, enum_val);
	if (ret < 0) {
		LOG_ERR("Failed to set MEAS%d PPG2 ADC RGE: %d", meas_idx + 1, ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ppg2_adc_rge(const struct device *dev, enum sensor_channel chan,
				     struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	uint8_t reg_val;
	int32_t microamps;
	int ret;

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_read(dev, base_reg + 2, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read MEAS%d PPG2 ADC RGE: %d", meas_idx + 1, ret);
		return ret;
	}

	ret = ppg_adc_range_from_enum(FIELD_GET(MAX86178_MEAS_CFG2_PPG2_ADC_RGE_MSK, reg_val),
				      &microamps);
	if (ret < 0) {
		return ret;
	}

	val->val1 = 0;
	val->val2 = microamps;
	return 0;
}

static int max86178_set_ppg1_dac_off(const struct device *dev, enum sensor_channel chan,
				     const struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	int ret;

	if (val->val1 < MAX86178_PPG_DAC_OFF_0uA || val->val1 > MAX86178_PPG_DAC_OFF_30uA) {
		LOG_ERR("Invalid PPG1 DAC OFF value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_update(dev, base_reg + 3, MAX86178_MEAS_CFG3_PPG1_DACOFF_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set MEAS%d PPG1 DAC OFF: %d", meas_idx + 1, ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ppg1_dac_off(const struct device *dev, enum sensor_channel chan,
				     struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	uint8_t reg_val;
	int ret;

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_read(dev, base_reg + 3, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read MEAS%d PPG1 DAC OFF: %d", meas_idx + 1, ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_MEAS_CFG3_PPG1_DACOFF_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_ppg2_dac_off(const struct device *dev, enum sensor_channel chan,
				     const struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	int ret;

	if (val->val1 < MAX86178_PPG_DAC_OFF_0uA || val->val1 > MAX86178_PPG_DAC_OFF_30uA) {
		LOG_ERR("Invalid PPG2 DAC OFF value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_update(dev, base_reg + 3, MAX86178_MEAS_CFG3_PPG2_DACOFF_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set MEAS%d PPG2 DAC OFF: %d", meas_idx + 1, ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ppg2_dac_off(const struct device *dev, enum sensor_channel chan,
				     struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	uint8_t reg_val;
	int ret;

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_read(dev, base_reg + 3, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read MEAS%d PPG2 DAC OFF: %d", meas_idx + 1, ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_MEAS_CFG3_PPG2_DACOFF_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_ppg_led_rge(const struct device *dev, enum sensor_channel chan,
				    const struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	uint8_t enum_val;
	int ret;

	ret = ppg_led_range_to_enum(val->val2, &enum_val);
	if (ret < 0) {
		LOG_ERR("Invalid LED range: %d µA (valid: 32000, 64000, 96000, 128000)", val->val2);
		return -EINVAL;
	}

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_update(dev, base_reg + 4, MAX86178_MEAS_CFG4_LED_RGE_MSK, enum_val);
	if (ret < 0) {
		LOG_ERR("Failed to set MEAS%d LED RGE: %d", meas_idx + 1, ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ppg_led_rge(const struct device *dev, enum sensor_channel chan,
				    struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	uint8_t reg_val;
	int32_t microamps;
	int ret;

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_read(dev, base_reg + 4, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read MEAS%d LED RGE: %d", meas_idx + 1, ret);
		return ret;
	}

	ret = ppg_led_range_from_enum(FIELD_GET(MAX86178_MEAS_CFG4_LED_RGE_MSK, reg_val),
				      &microamps);
	if (ret < 0) {
		return ret;
	}

	val->val1 = 0;
	val->val2 = microamps;
	return 0;
}

static int max86178_set_ppg_led_setlng(const struct device *dev, enum sensor_channel chan,
				       const struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	int ret;

	if (val->val1 < MAX86178_PPG_LED_SETLNG_7_7us ||
	    val->val1 > MAX86178_PPG_LED_SETLNG_23_7us) {
		LOG_ERR("Invalid LED SETLNG value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_update(dev, base_reg + 4, MAX86178_MEAS_CFG4_LED_SETLNG_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set MEAS%d LED SETLNG: %d", meas_idx + 1, ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ppg_led_setlng(const struct device *dev, enum sensor_channel chan,
				       struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	uint8_t reg_val;
	int ret;

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_read(dev, base_reg + 4, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read MEAS%d LED SETLNG: %d", meas_idx + 1, ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_MEAS_CFG4_LED_SETLNG_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_ppg_pd_setlng(const struct device *dev, enum sensor_channel chan,
				      const struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	int ret;

	if (val->val1 < MAX86178_PPG_PD_SETLNG_7_8us || val->val1 > MAX86178_PPG_PD_SETLNG_23_8us) {
		LOG_ERR("Invalid PD SETLNG value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_update(dev, base_reg + 4, MAX86178_MEAS_CFG4_PD_SETLNG_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set MEAS%d PD SETLNG: %d", meas_idx + 1, ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ppg_pd_setlng(const struct device *dev, enum sensor_channel chan,
				      struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	uint8_t reg_val;
	int ret;

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_read(dev, base_reg + 4, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read MEAS%d PD SETLNG: %d", meas_idx + 1, ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_MEAS_CFG4_PD_SETLNG_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_ppg_pd1_sel(const struct device *dev, enum sensor_channel chan,
				    const struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	int ret;

	if (val->val1 < MAX86178_PD_NOT_SEL || val->val1 > MAX86178_PD_CONN_TO_PPG2) {
		LOG_ERR("Invalid PD1 SEL value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_update(dev, base_reg + 5, MAX86178_MEAS_CFG5_PD1_SEL_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set MEAS%d PD1 SEL: %d", meas_idx + 1, ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ppg_pd1_sel(const struct device *dev, enum sensor_channel chan,
				    struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	uint8_t reg_val;
	int ret;

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_read(dev, base_reg + 5, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read MEAS%d PD1 SEL: %d", meas_idx + 1, ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_MEAS_CFG5_PD1_SEL_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_ppg_pd2_sel(const struct device *dev, enum sensor_channel chan,
				    const struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	int ret;

	if (val->val1 < MAX86178_PD_NOT_SEL || val->val1 > MAX86178_PD_CONN_TO_PPG2) {
		LOG_ERR("Invalid PD2 SEL value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_update(dev, base_reg + 5, MAX86178_MEAS_CFG5_PD2_SEL_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set MEAS%d PD2 SEL: %d", meas_idx + 1, ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ppg_pd2_sel(const struct device *dev, enum sensor_channel chan,
				    struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	uint8_t reg_val;
	int ret;

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_read(dev, base_reg + 5, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read MEAS%d PD2 SEL: %d", meas_idx + 1, ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_MEAS_CFG5_PD2_SEL_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_ppg_pd3_sel(const struct device *dev, enum sensor_channel chan,
				    const struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	int ret;

	if (val->val1 < MAX86178_PD_NOT_SEL || val->val1 > MAX86178_PD_CONN_TO_PPG2) {
		LOG_ERR("Invalid PD3 SEL value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_update(dev, base_reg + 5, MAX86178_MEAS_CFG5_PD3_SEL_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set MEAS%d PD3 SEL: %d", meas_idx + 1, ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ppg_pd3_sel(const struct device *dev, enum sensor_channel chan,
				    struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	uint8_t reg_val;
	int ret;

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_read(dev, base_reg + 5, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read MEAS%d PD3 SEL: %d", meas_idx + 1, ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_MEAS_CFG5_PD3_SEL_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_ppg_pd4_sel(const struct device *dev, enum sensor_channel chan,
				    const struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	int ret;

	if (val->val1 < MAX86178_PD_NOT_SEL || val->val1 > MAX86178_PD_CONN_TO_PPG2) {
		LOG_ERR("Invalid PD4 SEL value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_update(dev, base_reg + 5, MAX86178_MEAS_CFG5_PD4_SEL_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set MEAS%d PD4 SEL: %d", meas_idx + 1, ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ppg_pd4_sel(const struct device *dev, enum sensor_channel chan,
				    struct sensor_value *val)
{
	uint8_t meas_idx;
	uint8_t base_reg;
	uint8_t reg_val;
	int ret;

	ret = max86178_chan_to_meas_idx(chan, &meas_idx);
	if (ret < 0) {
		return ret;
	}

	base_reg = max86178_meas_base_reg(meas_idx);
	ret = max86178_reg_read(dev, base_reg + 5, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read MEAS%d PD4 SEL: %d", meas_idx + 1, ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_MEAS_CFG5_PD4_SEL_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_ecg_input_pol(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	if (val->val1 < MAX86178_ECG_INPUT_POL_NORMAL ||
	    val->val1 > MAX86178_ECG_INPUT_POL_INVERTED) {
		LOG_ERR("Invalid ECG input polarity value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_ECG_CFG2, MAX86178_ECG_CFG2_ECG_IPOL_MASK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG input polarity: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ecg_input_pol(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_ECG_CFG2, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read ECG input polarity: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_ECG_CFG2_ECG_IPOL_MASK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_ecg_pga_gain(const struct device *dev, const struct sensor_value *val)
{
	uint8_t enum_val;
	int ret;

	/* Convert gain value to enum */
	ret = ecg_pga_gain_to_enum(val->val1, &enum_val);
	if (ret < 0) {
		LOG_ERR("Invalid ECG PGA gain value: %d (valid: 1, 2, 4, 8)", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_ECG_CFG2, MAX86178_ECG_CFG2_ECG_PGA_GAIN_MSK,
				  enum_val);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG PGA gain: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ecg_pga_gain(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int32_t gain;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_ECG_CFG2, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read ECG PGA gain: %d", ret);
		return ret;
	}

	ret = ecg_pga_gain_from_enum(FIELD_GET(MAX86178_ECG_CFG2_ECG_PGA_GAIN_MSK, reg_val), &gain);
	if (ret < 0) {
		return ret;
	}

	val->val1 = gain;
	val->val2 = 0;
	return 0;
}

static int max86178_set_ecg_ina_rge(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	if (val->val1 < 0 || val->val1 > 3) {
		LOG_ERR("Invalid ECG INA range value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_ECG_CFG2, MAX86178_ECG_CFG2_ECG_INA_RGE_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG INA range: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ecg_ina_rge(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_ECG_CFG2, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read ECG INA range: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_ECG_CFG2_ECG_INA_RGE_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_ecg_ina_gain(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	if (val->val1 < 0 || val->val1 > 7) {
		LOG_ERR("Invalid ECG INA gain value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_ECG_CFG2, MAX86178_ECG_CFG2_ECG_INA_GAIN_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG INA gain: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ecg_ina_gain(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_ECG_CFG2, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read ECG INA gain: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_ECG_CFG2_ECG_INA_GAIN_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_ecg_imp_hi(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	if (val->val1 < 0 || val->val1 > 1) {
		LOG_ERR("Invalid ECG impedance mode value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_ECG_CFG3, MAX86178_ECG_CFG3_ECG_IMP_HI_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG impedance mode: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ecg_imp_hi(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_ECG_CFG3, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read ECG impedance mode: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_ECG_CFG3_ECG_IMP_HI_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_ecg_auto_rec(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	if (val->val1 < MAX86178_ECG_AUTO_REC_DISABLED ||
	    val->val1 > MAX86178_ECG_AUTO_REC_ENABLED) {
		LOG_ERR("Invalid ECG auto recovery value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_ECG_CFG3, MAX86178_ECG_CFG3_ECG_AUTO_REC_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG auto recovery: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ecg_auto_rec(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_ECG_CFG3, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read ECG auto recovery: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_ECG_CFG3_ECG_AUTO_REC_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_ecg_mux_sel(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	if (val->val1 < 0 || val->val1 > 3) {
		LOG_ERR("Invalid ECG mux select value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_ECG_CFG3, MAX86178_ECG_CFG3_ECG_MUX_SEL_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG mux select: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ecg_mux_sel(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_ECG_CFG3, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read ECG mux select: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_ECG_CFG3_ECG_MUX_SEL_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_en_ecg_fast_rec(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	if (val->val1 < MAX86178_EN_ECG_FAST_REC_NORMAL_MODE ||
	    val->val1 > MAX86178_EN_ECG_FAST_REC_AUTO_FAST_REC_MODE) {
		LOG_ERR("Invalid ECG fast recovery mode value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_ECG_CFG4, MAX86178_ECG_CFG4_EN_ECG_FAST_REC_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG fast recovery mode: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_en_ecg_fast_rec(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_ECG_CFG4, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read ECG fast recovery mode: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_ECG_CFG4_EN_ECG_FAST_REC_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_ecg_fast_rec_thres(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	if (val->val1 < 0 || val->val1 > 63) {
		LOG_ERR("Invalid ECG fast recovery threshold value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_ECG_CFG4,
				  MAX86178_ECG_CFG4_ECG_FAST_REC_THRESHOLD_MSK, (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG fast recovery threshold: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ecg_fast_rec_thres(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_ECG_CFG4, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read ECG fast recovery threshold: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_ECG_CFG4_ECG_FAST_REC_THRESHOLD_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_ecg_cal_freq(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	if (val->val1 < MAX86178_ECG_CAL_FREQ_DIV_128 ||
	    val->val1 > MAX86178_ECG_CAL_FREQ_DIV_2097152) {
		LOG_ERR("Invalid ECG calibration frequency value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_ECG_CAL_CFG1,
				  MAX86178_ECG_CAL_CFG1_ECG_CAL_FREQ_MSK, (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG calibration frequency: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ecg_cal_freq(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_ECG_CAL_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read ECG calibration frequency: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_ECG_CAL_CFG1_ECG_CAL_FREQ_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_ecg_cal_duty(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	if (val->val1 < MAX86178_ECG_CAL_DUTY_CAL_HIGH ||
	    val->val1 > MAX86178_ECG_CAL_DUTY_CAL_50) {
		LOG_ERR("Invalid ECG calibration duty value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_ECG_CAL_CFG1,
				  MAX86178_ECG_CAL_CFG1_ECG_CAL_DUTY_MSK, (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG calibration duty: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ecg_cal_duty(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_ECG_CAL_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read ECG calibration duty: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_ECG_CAL_CFG1_ECG_CAL_DUTY_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_ecg_cal_en(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	if (val->val1 < MAX86178_ECG_CAL_DISABLED || val->val1 > MAX86178_ECG_CAL_ENABLED) {
		LOG_ERR("Invalid ECG calibration enable value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_ECG_CAL_CFG1, MAX86178_ECG_CAL_CFG1_ECG_CAL_EN_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG calibration enable: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ecg_cal_en(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_ECG_CAL_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read ECG calibration enable: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_ECG_CAL_CFG1_ECG_CAL_EN_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_ecg_cal_high(const struct device *dev, const struct sensor_value *val)
{
	int ret;
	uint8_t msb;
	uint8_t lsb;

	if (val->val1 < 0 || val->val1 > 2047) {
		LOG_ERR("Invalid ECG calibration high value: %d", val->val1);
		return -EINVAL;
	}

	msb = (uint8_t)((val->val1 >> 8) & 0x07);
	lsb = (uint8_t)(val->val1 & 0xFF);

	ret = max86178_reg_update(dev, MAX86178_ECG_CAL_CFG1,
				  MAX86178_ECG_CAL_CFG1_ECG_CAL_HIGH_MSB_MSK, msb);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG calibration high MSB: %d", ret);
		return ret;
	}

	ret = max86178_reg_write(dev, MAX86178_ECG_CAL_CFG2, &lsb, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG calibration high LSB: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ecg_cal_high(const struct device *dev, struct sensor_value *val)
{
	uint8_t cfg1_val;
	uint8_t lsb;
	uint8_t msb;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_ECG_CAL_CFG1, &cfg1_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read ECG calibration high MSB: %d", ret);
		return ret;
	}
	msb = FIELD_GET(MAX86178_ECG_CAL_CFG1_ECG_CAL_HIGH_MSB_MSK, cfg1_val);

	ret = max86178_reg_read(dev, MAX86178_ECG_CAL_CFG2, &lsb, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read ECG calibration high LSB: %d", ret);
		return ret;
	}

	val->val1 = ((uint16_t)msb << 8) | lsb;
	val->val2 = 0;
	return 0;
}

static int max86178_set_ecg_open_p(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	if (val->val1 < MAX86178_ECG_PIN_INTERNALLY_CONNECTED ||
	    val->val1 > MAX86178_ECG_PIN_INTERNALLY_ISOLATED) {
		LOG_ERR("Invalid ECG open P value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_ECG_CAL_CFG3, MAX86178_ECG_CAL_CFG3_ECG_OPEN_P_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG open P: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ecg_open_p(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_ECG_CAL_CFG3, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read ECG open P: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_ECG_CAL_CFG3_ECG_OPEN_P_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_ecg_open_n(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	if (val->val1 < MAX86178_ECG_PIN_INTERNALLY_CONNECTED ||
	    val->val1 > MAX86178_ECG_PIN_INTERNALLY_ISOLATED) {
		LOG_ERR("Invalid ECG open N value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_ECG_CAL_CFG3, MAX86178_ECG_CAL_CFG3_ECG_OPEN_N_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG open N: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ecg_open_n(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_ECG_CAL_CFG3, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read ECG open N: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_ECG_CAL_CFG3_ECG_OPEN_N_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_ecg_cal_mode(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	if (val->val1 < MAX86178_ECG_CAL_MODE_UNIPOLAR ||
	    val->val1 > MAX86178_ECG_CAL_MODE_BIPOLAR) {
		LOG_ERR("Invalid ECG calibration mode value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_ECG_CAL_CFG3,
				  MAX86178_ECG_CAL_CFG3_ECG_CAL_MODE_MSK, (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG calibration mode: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ecg_cal_mode(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_ECG_CAL_CFG3, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read ECG calibration mode: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_ECG_CAL_CFG3_ECG_CAL_MODE_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* ECG_CAL_MAG setter/getter */
static int max86178_set_ecg_cal_mag(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate enum range */
	if (val->val1 < MAX86178_ECG_CAL_MAG_0_5mV || val->val1 > MAX86178_ECG_CAL_MAG_1mV) {
		LOG_ERR("Invalid ECG calibration magnitude value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_ECG_CAL_CFG3,
				  MAX86178_ECG_CAL_CFG3_ECG_CAL_MAG_MASK, (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG calibration magnitude: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ecg_cal_mag(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_ECG_CAL_CFG3, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read ECG calibration magnitude: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_ECG_CAL_CFG3_ECG_CAL_MAG_MASK, reg_val);
	val->val2 = 0;
	return 0;
}

/* ECG_CAL_P_SEL setter/getter */
static int max86178_set_ecg_cal_p_sel(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate enum range */
	if (val->val1 < MAX86178_ECG_CAL_SEL_NO_CAL ||
	    val->val1 > MAX86178_ECG_CAL_SEL_CONN_VCALN) {
		LOG_ERR("Invalid ECG calibration P select value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_ECG_CAL_CFG3,
				  MAX86178_ECG_CAL_CFG3_ECG_CAL_P_SEL_MSK, (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG calibration P select: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ecg_cal_p_sel(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_ECG_CAL_CFG3, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read ECG calibration P select: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_ECG_CAL_CFG3_ECG_CAL_P_SEL_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* ECG_CAL_N_SEL setter/getter */
static int max86178_set_ecg_cal_n_sel(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate enum range */
	if (val->val1 < MAX86178_ECG_CAL_SEL_NO_CAL ||
	    val->val1 > MAX86178_ECG_CAL_SEL_CONN_VCALN) {
		LOG_ERR("Invalid ECG calibration N select value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_ECG_CAL_CFG3,
				  MAX86178_ECG_CAL_CFG3_ECG_CAL_N_SEL_MSK, (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG calibration N select: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ecg_cal_n_sel(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_ECG_CAL_CFG3, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read ECG calibration N select: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_ECG_CAL_CFG3_ECG_CAL_N_SEL_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* EN_ECG_LON setter/getter */
static int max86178_set_en_ecg_lon(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate enum range */
	if (val->val1 < MAX86178_ECG_LON_DISABLED || val->val1 > MAX86178_ECG_LON_ENABLED) {
		LOG_ERR("Invalid ECG lead on enable value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_ECG_LD_CFG1, MAX86178_ECG_LEAD_CFG1_EN_ECG_LON_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG lead on enable: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_en_ecg_lon(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_ECG_LD_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read ECG lead on enable: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_ECG_LEAD_CFG1_EN_ECG_LON_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* EN_ECG_LOFF setter/getter */
static int max86178_set_en_ecg_loff(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate enum range */
	if (val->val1 < MAX86178_ECG_LOFF_DISABLED || val->val1 > MAX86178_ECG_LOFF_ENABLED) {
		LOG_ERR("Invalid ECG lead off enable value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_ECG_LD_CFG1, MAX86178_ECG_LEAD_CFG1_EN_ECG_LOFF_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG lead off enable: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_en_ecg_loff(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_ECG_LD_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read ECG lead off enable: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_ECG_LEAD_CFG1_EN_ECG_LOFF_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* ECG_LOFF_MODE setter/getter */
static int max86178_set_ecg_loff_mode(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate enum range */
	if (val->val1 < MAX86178_ECG_DC_LEAD_OFF_DETECTION_MODE ||
	    val->val1 > MAX86178_ECG_AC_LEAD_OFF_DETECTION_MODE) {
		LOG_ERR("Invalid ECG lead off mode value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_ECG_LD_CFG1,
				  MAX86178_ECG_LEAD_CFG1_ECG_LOFF_MODE_MSK, (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG lead off mode: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ecg_loff_mode(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_ECG_LD_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read ECG lead off mode: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_ECG_LEAD_CFG1_ECG_LOFF_MODE_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* ECG_LOFF_FREQ setter/getter */
/* ECG_LOFF_FREQ setter/getter - accepts numeric frequency in Hz (0, 4, 8, 16, 32, 64, 128, 256) */
static int max86178_set_ecg_loff_freq(const struct device *dev, const struct sensor_value *val)
{
	uint8_t enum_val;
	int ret;

	/* Convert Hz value to enum */
	ret = ecg_loff_freq_to_enum(val->val1, &enum_val);
	if (ret < 0) {
		LOG_ERR("Invalid ECG lead off frequency: %d Hz (valid: 0, 4, 8, 16, 32, 64, 128, "
			"256)",
			val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_ECG_LD_CFG1,
				  MAX86178_ECG_LEAD_CFG1_ECG_LOFF_FREQ_MSK, enum_val);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG lead off frequency: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ecg_loff_freq(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int32_t hz;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_ECG_LD_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read ECG lead off frequency: %d", ret);
		return ret;
	}

	/* Convert enum to Hz value */
	ret = ecg_loff_freq_from_enum(FIELD_GET(MAX86178_ECG_LEAD_CFG1_ECG_LOFF_FREQ_MSK, reg_val),
				      &hz);
	if (ret < 0) {
		return ret;
	}

	val->val1 = hz;
	val->val2 = 0;
	return 0;
}

/* ECG_LOFF_IPOL setter/getter */
static int max86178_set_ecg_loff_ipol(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate enum range */
	if (val->val1 < MAX86178_ECG_LOFF_IPOL_NON_INVERTED ||
	    val->val1 > MAX86178_ECG_LOFF_IPOL_INVERTED) {
		LOG_ERR("Invalid ECG lead off current polarity value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_ECG_LD_CFG2,
				  MAX86178_ECG_LEAD_CFG2_ECG_LOFF_IPOL_MSK, (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG lead off current polarity: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ecg_loff_ipol(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_ECG_LD_CFG2, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read ECG lead off polarity: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_ECG_LEAD_CFG2_ECG_LOFF_IPOL_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* ECG_LOFF_IMAG setter/getter - accepts numeric current in nA (0, 5, 10, 20, 50, 100, 200, 400) */
static int max86178_set_ecg_loff_imag(const struct device *dev, const struct sensor_value *val)
{
	uint8_t enum_val;
	int ret;

	/* Convert nA value to enum */
	ret = ecg_loff_imag_to_enum(val->val1, &enum_val);
	if (ret < 0) {
		LOG_ERR("Invalid ECG lead off current: %d nA (valid: 0, 5, 10, 20, 50, 100, 200, "
			"400)",
			val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_ECG_LD_CFG2,
				  MAX86178_ECG_LEAD_CFG2_ECG_LOFF_IMAG_MSK, enum_val);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG lead off current magnitude: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ecg_loff_imag(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int32_t nanoamps;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_ECG_LD_CFG2, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read ECG lead off current magnitude: %d", ret);
		return ret;
	}

	/* Convert enum to nA value */
	ret = ecg_loff_imag_from_enum(FIELD_GET(MAX86178_ECG_LEAD_CFG2_ECG_LOFF_IMAG_MSK, reg_val),
				      &nanoamps);
	if (ret < 0) {
		return ret;
	}

	val->val1 = nanoamps;
	val->val2 = 0;
	return 0;
}

/* ECG_LOFF_THRESH setter/getter */
static int max86178_set_ecg_loff_thresh(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate range (4-bit field, 0-15) */
	if (val->val1 < 0 || val->val1 > 15) {
		LOG_ERR("Invalid ECG lead off threshold value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_ECG_LD_CFG2,
				  MAX86178_ECG_LEAD_CFG2_ECG_LOFF_THRESH_MSK, (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG lead off threshold: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ecg_loff_thresh(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_ECG_LD_CFG2, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read ECG lead off threshold: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_ECG_LEAD_CFG2_ECG_LOFF_THRESH_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* ECG_RBIAS_VALUE setter/getter */
/* ECG_RBIAS_VALUE setter/getter - accepts resistance in megaohms (50, 100, 200) */
static int max86178_set_ecg_rbias_value(const struct device *dev, const struct sensor_value *val)
{
	uint8_t enum_val;
	int ret;

	/* Convert megaohm value to enum */
	ret = ecg_rbias_value_to_enum(val->val1, &enum_val);
	if (ret < 0) {
		LOG_ERR("Invalid ECG bias resistor value: %d MΩ (valid: 50, 100, 200)", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_ECG_LB_CFG1,
				  MAX86178_ECG_LEAD_BIAS_ECG_RBIAS_VAL_MSK, enum_val);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG bias resistor value: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_ecg_rbias_value(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int32_t megaohms;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_ECG_LB_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read ECG bias resistor value: %d", ret);
		return ret;
	}

	/* Convert enum to megaohm value */
	ret = ecg_rbias_value_from_enum(
		FIELD_GET(MAX86178_ECG_LEAD_BIAS_ECG_RBIAS_VAL_MSK, reg_val), &megaohms);
	if (ret < 0) {
		return ret;
	}

	val->val1 = megaohms;
	val->val2 = 0;
	return 0;
}

/* EN_ECG_RBIAS_P setter/getter */
static int max86178_set_en_ecg_rbias_p(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate enum range */
	if (val->val1 < MAX86178_EN_ECG_RBIAS_DISCONNECTED ||
	    val->val1 > MAX86178_EN_ECG_RBIAS_CONNECTED) {
		LOG_ERR("Invalid ECG bias resistor P enable value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_ECG_LB_CFG1,
				  MAX86178_ECG_LEAD_BIAS_EN_ECG_RBIAS_P_MSK, (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG bias resistor P enable: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_en_ecg_rbias_p(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_ECG_LB_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read ECG bias resistor P enable: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_ECG_LEAD_BIAS_EN_ECG_RBIAS_P_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* EN_ECG_RBIAS_N setter/getter */
static int max86178_set_en_ecg_rbias_n(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate enum range */
	if (val->val1 < MAX86178_EN_ECG_RBIAS_DISCONNECTED ||
	    val->val1 > MAX86178_EN_ECG_RBIAS_CONNECTED) {
		LOG_ERR("Invalid ECG bias resistor N enable value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_ECG_LB_CFG1,
				  MAX86178_ECG_LEAD_BIAS_EN_ECG_RBIAS_N_MSK, (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG bias resistor N enable: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_en_ecg_rbias_n(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_ECG_LB_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read ECG bias resistor N enable: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_ECG_LEAD_BIAS_EN_ECG_RBIAS_N_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_rld_en(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	if (val->val1 < MAX86178_RLD_DISABLED || val->val1 > MAX86178_RLD_ENABLED) {
		LOG_ERR("Invalid RLD enable value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_RLD_CFG1, MAX86178_RLD_CFG1_RLD_EN_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set RLD enable: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_rld_en(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_RLD_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read RLD enable: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_RLD_CFG1_RLD_EN_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

static int max86178_set_rld_mode(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	if (val->val1 < MAX86178_RLD_MODE_OPEN_LOOP || val->val1 > MAX86178_RLD_MODE_CLOSED_LOOP) {
		LOG_ERR("Invalid RLD mode value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_RLD_CFG1, MAX86178_RLD_CFG1_RLD_MODE_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set RLD mode: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_rld_mode(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_RLD_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read RLD mode: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_RLD_CFG1_RLD_MODE_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* RLD_RBIAS setter/getter */
static int max86178_set_rld_rbias(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate enum range */
	if (val->val1 < MAX86178_RLD_RBIAS_VMID || val->val1 > MAX86178_RLD_RBIAS_VRLD) {
		LOG_ERR("Invalid RLD RBIAS value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_RLD_CFG1, MAX86178_RLD_CFG1_RLD_BIAS_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set RLD RBIAS: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_rld_rbias(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_RLD_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read RLD RBIAS: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_RLD_CFG1_RLD_BIAS_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* EN_RLD_OOR setter/getter */
static int max86178_set_en_rld_oor(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate enum range */
	if (val->val1 < MAX86178_EN_RLD_OOR_DISABLED || val->val1 > MAX86178_EN_RLD_OOR_ENABLED) {
		LOG_ERR("Invalid RLD OOR enable value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_RLD_CFG1, MAX86178_RLD_CFG1_EN_RLD_OOR_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set RLD OOR enable: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_en_rld_oor(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_RLD_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read RLD OOR enable: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_RLD_CFG1_EN_RLD_OOR_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* ACTV_CM_P setter/getter */
static int max86178_set_actv_cm_p(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate enum range */
	if (val->val1 < MAX86178_ECG_ACTV_CM_DISABLED || val->val1 > MAX86178_ECG_ACTV_CM_ENABLED) {
		LOG_ERR("Invalid ACTV_CM_P value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_RLD_CFG1, MAX86178_RLD_CFG1_ACTV_CM_P_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set ACTV_CM_P: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_actv_cm_p(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_RLD_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read ACTV_CM_P: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_RLD_CFG1_ACTV_CM_P_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* ACTV_CM_N setter/getter */
static int max86178_set_actv_cm_n(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate enum range */
	if (val->val1 < MAX86178_ECG_ACTV_CM_DISABLED || val->val1 > MAX86178_ECG_ACTV_CM_ENABLED) {
		LOG_ERR("Invalid ACTV_CM_N value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_RLD_CFG1, MAX86178_RLD_CFG1_ACTV_CM_N_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set ACTV_CM_N: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_actv_cm_n(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_RLD_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read ACTV_CM_N: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_RLD_CFG1_ACTV_CM_N_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* RLD_GAIN setter/getter - accepts numeric gain value (12, 24, 48, 97) */
static int max86178_set_rld_gain(const struct device *dev, const struct sensor_value *val)
{
	uint8_t enum_val;
	int ret;

	/* Convert gain value to enum */
	ret = rld_gain_to_enum(val->val1, &enum_val);
	if (ret < 0) {
		LOG_ERR("Invalid RLD gain value: %d (valid: 12, 24, 48, 97)", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_RLD_CFG1, MAX86178_RLD_CFG1_RLD_GAIN_MSK, enum_val);
	if (ret < 0) {
		LOG_ERR("Failed to set RLD gain: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_rld_gain(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int32_t gain;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_RLD_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read RLD gain: %d", ret);
		return ret;
	}

	/* Convert enum to gain value */
	ret = rld_gain_from_enum(FIELD_GET(MAX86178_RLD_CFG1_RLD_GAIN_MSK, reg_val), &gain);
	if (ret < 0) {
		return ret;
	}

	val->val1 = gain;
	val->val2 = 0;
	return 0;
}

/* RLD_EXT_RES setter/getter */
static int max86178_set_rld_ext_res(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate enum range */
	if (val->val1 < MAX86178_RLD_EXT_RES_INTERNAL ||
	    val->val1 > MAX86178_RLD_EXT_RES_EXTERNAL) {
		LOG_ERR("Invalid RLD external resistor value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_RLD_CFG2, MAX86178_RLD_CFG2_RLD_EXT_RES_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set RLD external resistor: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_rld_ext_res(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_RLD_CFG2, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read RLD external resistor: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_RLD_CFG2_RLD_EXT_RES_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* RLD_SEL_ECG setter/getter */
static int max86178_set_rld_sel_ecg(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate boolean value */
	if (val->val1 != 0 && val->val1 != 1) {
		LOG_ERR("Invalid RLD select ECG value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_RLD_CFG2, MAX86178_RLD_CFG2_RLD_SEL_ECG_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set RLD select ECG: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_rld_sel_ecg(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_RLD_CFG2, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read RLD select ECG: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_RLD_CFG2_RLD_SEL_ECG_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* RLD_BW setter/getter */
static int max86178_set_rld_bw(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate 2-bit field range */
	if (val->val1 < 0 || val->val1 > 3) {
		LOG_ERR("Invalid RLD bandwidth value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_RLD_CFG2, MAX86178_RLD_CFG2_RLD_BW_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set RLD bandwidth: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_rld_bw(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_RLD_CFG2, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read RLD bandwidth: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_RLD_CFG2_RLD_BW_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BODY_BIAS_DAC setter/getter */
static int max86178_set_body_bias_dac(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate 4-bit field range */
	if (val->val1 < 0 || val->val1 > 15) {
		LOG_ERR("Invalid body bias DAC value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_RLD_CFG2, MAX86178_RLD_CFG2_BODY_BIAS_DAC_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set body bias DAC: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_get_body_bias_dac(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_RLD_CFG2, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read body bias DAC: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_RLD_CFG2_BODY_BIAS_DAC_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BioZ Setup Configuration setter/getter functions */

/* BIOZ_ADC_OSR setter/getter - accepts numeric OSR value (8, 16, 32, ..., 1024) */
static int max86178_attr_set_bioz_adc_osr(const struct device *dev, const struct sensor_value *val)
{
	uint8_t enum_val;
	int ret;

	/* Convert numeric OSR value to enum */
	ret = bioz_adc_osr_to_enum(val->val1, &enum_val);
	if (ret < 0) {
		LOG_ERR("Invalid BioZ ADC OSR value: %d (valid: 8, 16, 32, 64, 128, 256, 512, "
			"1024)",
			val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_CFG1, MAX86178_BIOZ_CFG1_BIOZ_ADC_OSR_MSK,
				  enum_val);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ ADC OSR: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bioz_adc_osr(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int32_t osr;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ ADC OSR: %d", ret);
		return ret;
	}

	/* Convert enum to numeric OSR value */
	ret = bioz_adc_osr_from_enum(FIELD_GET(MAX86178_BIOZ_CFG1_BIOZ_ADC_OSR_MSK, reg_val), &osr);
	if (ret < 0) {
		return ret;
	}

	val->val1 = osr;
	val->val2 = 0;
	return 0;
}

/* BIOZ_DAC_OSR setter/getter - accepts numeric OSR value (32, 64, 128, 256) */
static int max86178_attr_set_bioz_dac_osr(const struct device *dev, const struct sensor_value *val)
{
	uint8_t enum_val;
	int ret;

	/* Convert numeric OSR value to enum */
	ret = bioz_dac_osr_to_enum(val->val1, &enum_val);
	if (ret < 0) {
		LOG_ERR("Invalid BioZ DAC OSR value: %d (valid: 32, 64, 128, 256)", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_CFG1, MAX86178_BIOZ_CFG1_BIOZ_DAC_OSR_MSK,
				  enum_val);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ DAC OSR: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bioz_dac_osr(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int32_t osr;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ DAC OSR: %d", ret);
		return ret;
	}

	/* Convert enum to numeric OSR value */
	ret = bioz_dac_osr_from_enum(FIELD_GET(MAX86178_BIOZ_CFG1_BIOZ_DAC_OSR_MSK, reg_val), &osr);
	if (ret < 0) {
		return ret;
	}

	val->val1 = osr;
	val->val2 = 0;
	return 0;
}

/* EN_BIOZ_THRESH setter/getter */
static int max86178_attr_set_en_bioz_thresh(const struct device *dev,
					    const struct sensor_value *val)
{
	int ret;

	/* Validate enum range */
	if (val->val1 < MAX86178_EN_BIOZ_THRESH_DISABLED ||
	    val->val1 > MAX86178_EN_BIOZ_THRESH_ENABLED) {
		LOG_ERR("Invalid BioZ threshold enable value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_CFG2, MAX86178_BIOZ_CFG2_EN_BIOZ_THRESH_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ threshold enable: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_en_bioz_thresh(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_CFG2, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ threshold enable: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_CFG2_EN_BIOZ_THRESH_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BIOZ_DLPF setter/getter */
static int max86178_attr_set_bioz_dlpf(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate enum range (0-4) */
	if (val->val1 < MAX86178_BIOZ_DLPF_BYPASS || val->val1 > MAX86178_BIOZ_DLPF_0_25) {
		LOG_ERR("Invalid BioZ DLPF value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_CFG2, MAX86178_BIOZ_CFG2_BIOZ_DLPF_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ DLPF: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bioz_dlpf(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_CFG2, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ DLPF: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_CFG2_BIOZ_DLPF_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BIOZ_DHPF setter/getter */
static int max86178_attr_set_bioz_dhpf(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate enum range (0-2) */
	if (val->val1 < MAX86178_BIOZ_DHPF_BYPASS || val->val1 > MAX86178_BIOZ_DHPF_0_002) {
		LOG_ERR("Invalid BioZ DHPF value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_CFG2, MAX86178_BIOZ_CFG2_BIOZ_DHPF_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ DHPF: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bioz_dhpf(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_CFG2, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ DHPF: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_CFG2_BIOZ_DHPF_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BIOZ_DRV_MODE setter/getter */
static int max86178_attr_set_bioz_drv_mode(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate enum range (0-3) */
	if (val->val1 < MAX86178_BIOZ_DRV_MODE_CURRENT ||
	    val->val1 > MAX86178_BIOZ_DRV_MODE_STANDBY) {
		LOG_ERR("Invalid BioZ drive mode value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_CFG3, MAX86178_BIOZ_CFG3_BIOZ_DRV_MODE_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ drive mode: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bioz_drv_mode(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_CFG3, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ drive mode: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_CFG3_BIOZ_DRV_MODE_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BIOZ_IDRV_RGE setter/getter */
static int max86178_attr_set_bioz_idrv_rge(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate enum range (0-3) */
	if (val->val1 < MAX86178_BIOZ_IDRV_RGE_552_5K ||
	    val->val1 > MAX86178_BIOZ_IDRV_RGE_276_25) {
		LOG_ERR("Invalid BioZ current drive range value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_CFG3, MAX86178_BIOZ_CFG3_BIOZ_IDRV_RGE_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ current drive range: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bioz_idrv_rge(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_CFG3, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ current drive range: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_CFG3_BIOZ_IDRV_RGE_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BIOZ_VDRV_MAG setter/getter */
/* BIOZ_VDRV_MAG setter/getter - accepts voltage in µV (50000, 100000, 250000, 500000) as volts in
 * val2 */
static int max86178_attr_set_bioz_vdrv_mag(const struct device *dev, const struct sensor_value *val)
{
	uint8_t enum_val;
	int ret;

	/* Convert µV value (in val2) to enum */
	ret = bioz_vdrv_mag_to_enum(val->val2, &enum_val);
	if (ret < 0) {
		LOG_ERR("Invalid BioZ voltage drive magnitude: %d µV (valid: 50000, 100000, "
			"250000, 500000)",
			val->val2);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_CFG3, MAX86178_BIOZ_CFG3_BIOZ_VDRV_MAG_MSK,
				  enum_val);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ voltage drive magnitude: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bioz_vdrv_mag(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int32_t microvolts;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_CFG3, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ voltage drive magnitude: %d", ret);
		return ret;
	}

	/* Convert enum to µV value */
	ret = bioz_vdrv_mag_from_enum(FIELD_GET(MAX86178_BIOZ_CFG3_BIOZ_VDRV_MAG_MSK, reg_val),
				      &microvolts);
	if (ret < 0) {
		return ret;
	}

	/* Return as volts: µV goes in val2 (fractional micro-units) */
	val->val1 = 0;
	val->val2 = microvolts;
	return 0;
}

/* BIOZ_EXT_RES setter/getter */
static int max86178_attr_set_bioz_ext_res(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate enum range */
	if (val->val1 < MAX86178_BIOZ_EXT_RES_INTERNAL ||
	    val->val1 > MAX86178_BIOZ_EXT_RES_EXTERNAL) {
		LOG_ERR("Invalid BioZ external resistor value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_CFG3, MAX86178_BIOZ_CFG3_BIOZ_EXT_RES_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ external resistor: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bioz_ext_res(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_CFG3, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ external resistor: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_CFG3_BIOZ_EXT_RES_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* EN_UTIL_MODE setter/getter */
static int max86178_attr_set_en_util_mode(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate range (0-1) */
	if (val->val1 < 0 || val->val1 > 1) {
		LOG_ERR("Invalid utility mode enable value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_CFG4, MAX86178_BIOZ_CFG4_EN_UTIL_MODE_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set utility mode enable: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_en_util_mode(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_CFG4, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read utility mode enable: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_CFG4_EN_UTIL_MODE_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BIOZ_DC_DAC_CODE setter/getter */
static int max86178_attr_set_bioz_dc_dac_code(const struct device *dev,
					      const struct sensor_value *val)
{
	int ret;

	/* Validate 7-bit field range */
	if (val->val1 < 0 || val->val1 > 127) {
		LOG_ERR("Invalid BioZ DC DAC code value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_CFG5, MAX86178_BIOZ_CFG5_BIOZ_DC_DAC_CODE_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ DC DAC code: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bioz_dc_dac_code(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_CFG5, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ DC DAC code: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_CFG5_BIOZ_DC_DAC_CODE_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BIOZ_DC_CODE_SEL setter/getter */
static int max86178_attr_set_bioz_dc_code_sel(const struct device *dev,
					      const struct sensor_value *val)
{
	int ret;

	/* Validate enum range */
	if (val->val1 < MAX86178_BIOZ_DC_CODE_SEL_DDS ||
	    val->val1 > MAX86178_BIOZ_DC_CODE_SEL_DC_CODE) {
		LOG_ERR("Invalid BioZ DC code select value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_CFG5, MAX86178_BIOZ_CFG5_BIOZ_DC_CODE_SEL_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ DC code select: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bioz_dc_code_sel(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_CFG5, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ DC code select: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_CFG5_BIOZ_DC_CODE_SEL_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BIOZ_GAIN setter/getter */
/* BIOZ_GAIN setter/getter - accepts gain value (1, 2.5, 5, 10) */
static int max86178_attr_set_bioz_gain(const struct device *dev, const struct sensor_value *val)
{
	uint8_t enum_val;
	int ret;

	/* Convert gain value to enum (supports fractional 2.5 via val2) */
	ret = bioz_gain_to_enum(val, &enum_val);
	if (ret < 0) {
		LOG_ERR("Invalid BioZ gain value: %d.%d (valid: 1, 2.5, 5, 10)", val->val1,
			val->val2);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_CFG6, MAX86178_BIOZ_CFG6_BIOZ_GAIN_MSK,
				  enum_val);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ gain: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bioz_gain(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_CFG6, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ gain: %d", ret);
		return ret;
	}

	/* Convert enum to gain value */
	ret = bioz_gain_from_enum(FIELD_GET(MAX86178_BIOZ_CFG6_BIOZ_GAIN_MSK, reg_val), val);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

/* DM_DIS setter/getter */
static int max86178_attr_set_dm_dis(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate enum range */
	if (val->val1 < MAX86178_BIOZ_DM_ENABLED || val->val1 > MAX86178_BIOZ_DM_DISABLED) {
		LOG_ERR("Invalid DM disable value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_CFG6, MAX86178_BIOZ_CFG6_DM_DIS_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set DM disable: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_dm_dis(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_CFG6, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read DM disable: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_CFG6_DM_DIS_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BIOZ_INA_MODE setter/getter */
static int max86178_attr_set_bioz_ina_mode(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate enum range */
	if (val->val1 < MAX86178_BIOZ_INA_MODE_HIGH_POWER ||
	    val->val1 > MAX86178_BIOZ_INA_MODE_LOW_POWER) {
		LOG_ERR("Invalid BioZ INA mode value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_CFG6, MAX86178_BIOZ_CFG6_BIOZ_INA_MODE_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ INA mode: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bioz_ina_mode(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_CFG6, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ INA mode: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_CFG6_BIOZ_INA_MODE_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BIOZ_AHPF setter/getter */
static int max86178_attr_set_bioz_ahpf(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate 4-bit field range (0-15) */
	if (val->val1 < MAX86178_BIOZ_AHPF_100Hz || val->val1 > MAX86178_BIOZ_AHPF_BYPASS_2) {
		LOG_ERR("Invalid BioZ AHPF value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_CFG6, MAX86178_BIOZ_CFG6_BIOZ_AHPF_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ AHPF: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bioz_ahpf(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_CFG6, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ AHPF: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_CFG6_BIOZ_AHPF_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BIOZ_AMP_BW setter/getter */
static int max86178_attr_set_bioz_amp_bw(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate enum range (0-3) */
	if (val->val1 < MAX86178_BIOZ_AMP_BW_LOW || val->val1 > MAX86178_BIOZ_AMP_BW_HIGH) {
		LOG_ERR("Invalid BioZ amp bandwidth value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_CFG7, MAX86178_BIOZ_CFG7_BIOZ_AMP_BW_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ amp bandwidth: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bioz_amp_bw(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_CFG7, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ amp bandwidth: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_CFG7_BIOZ_AMP_BW_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BIOZ_AMP_RGE setter/getter */
static int max86178_attr_set_bioz_amp_rge(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate enum range (0-3) */
	if (val->val1 < MAX86178_BIOZ_AMP_RGE_LOW || val->val1 > MAX86178_BIOZ_AMP_RGE_HIGH) {
		LOG_ERR("Invalid BioZ amp range value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_CFG7, MAX86178_BIOZ_CFG7_BIOZ_AMP_RGE_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ amp range: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bioz_amp_rge(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_CFG7, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ amp range: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_CFG7_BIOZ_AMP_RGE_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BIOZ_DAC_RESET setter/getter */
static int max86178_attr_set_bioz_dac_reset(const struct device *dev,
					    const struct sensor_value *val)
{
	int ret;

	/* Validate enum range */
	if (val->val1 < MAX86178_BIOZ_DRV_RESET_SWITCH_OPEN ||
	    val->val1 > MAX86178_BIOZ_DRV_RESET_SWITCH_CLOSED) {
		LOG_ERR("Invalid BioZ DAC reset value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_CFG7, MAX86178_BIOZ_CFG7_BIOZ_DAC_RESET_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ DAC reset: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bioz_dac_reset(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_CFG7, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ DAC reset: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_CFG7_BIOZ_DAC_RESET_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BIOZ_DRV_RESET setter/getter */
static int max86178_attr_set_bioz_drv_reset(const struct device *dev,
					    const struct sensor_value *val)
{
	int ret;

	/* Validate enum range */
	if (val->val1 < MAX86178_BIOZ_DRV_RESET_SWITCH_OPEN ||
	    val->val1 > MAX86178_BIOZ_DRV_RESET_SWITCH_CLOSED) {
		LOG_ERR("Invalid BioZ drive reset value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_CFG7, MAX86178_BIOZ_CFG7_BIOZ_DRV_RESET_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ drive reset: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bioz_drv_reset(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_CFG7, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ drive reset: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_CFG7_BIOZ_DRV_RESET_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BIOZ_DC_RESTORE setter/getter */
static int max86178_attr_set_bioz_dc_restore(const struct device *dev,
					     const struct sensor_value *val)
{
	int ret;

	/* Validate enum range */
	if (val->val1 < MAX86178_BIOZ_DC_RESTORE_SWITCH_OPEN ||
	    val->val1 > MAX86178_BIOZ_DC_RESTORE_SWITCH_CLOSED) {
		LOG_ERR("Invalid BioZ DC restore value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_CFG7, MAX86178_BIOZ_CFG7_BIOZ_DC_RESTORE_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ DC restore: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bioz_dc_restore(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_CFG7, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ DC restore: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_CFG7_BIOZ_DC_RESTORE_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BIOZ_EXT_CAP setter/getter */
static int max86178_attr_set_bioz_ext_cap(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate enum range */
	if (val->val1 < MAX86178_BIOZ_EXT_CAP_INTERNAL ||
	    val->val1 > MAX86178_BIOZ_EXT_CAP_EXTERNAL) {
		LOG_ERR("Invalid BioZ external cap value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_CFG7, MAX86178_BIOZ_CFG7_BIOZ_EXT_CAP_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ external cap: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bioz_ext_cap(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_CFG7, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ external cap: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_CFG7_BIOZ_EXT_CAP_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BIOZ_CH_FSEL setter/getter */
static int max86178_attr_set_bioz_ch_fsel(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate enum range */
	if (val->val1 < MAX86178_BIOZ_CH_FSEL_50KHZ || val->val1 > MAX86178_BIOZ_CH_FSEL_25KHZ) {
		LOG_ERR("Invalid BioZ chopper frequency select value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_CFG8, MAX86178_BIOZ_CFG8_BIOZ_CH_FSEL_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ chopper frequency select: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bioz_ch_fsel(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_CFG8, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ chopper frequency select: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_CFG8_BIOZ_CH_FSEL_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BIOZ_INA_CHOP_EN setter/getter */
static int max86178_attr_set_bioz_ina_chop_en(const struct device *dev,
					      const struct sensor_value *val)
{
	int ret;

	/* Validate enum range */
	if (val->val1 < MAX86178_BIOZ_INA_CHOP_DISABLED ||
	    val->val1 > MAX86178_BIOZ_INA_CHOP_ENABLED) {
		LOG_ERR("Invalid BioZ INA chopper enable value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_CFG8, MAX86178_BIOZ_CFG8_BIOZ_INA_CHOP_EN_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ INA chopper enable: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bioz_ina_chop_en(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_CFG8, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ INA chopper enable: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_CFG8_BIOZ_INA_CHOP_EN_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BIOZ_FAST setter/getter */
static int max86178_attr_set_bioz_fast(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate enum range */
	if (val->val1 < MAX86178_BIOZ_FAST_NORMAL || val->val1 > MAX86178_BIOZ_FAST_START_ENABLED) {
		LOG_ERR("Invalid BioZ fast start value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_CFG8, MAX86178_BIOZ_CFG8_BIOZ_FAST_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ fast start: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bioz_fast(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_CFG8, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ fast start: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_CFG8_BIOZ_FAST_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BIOZ_IPOL setter/getter */
static int max86178_attr_set_bioz_ipol(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate enum range */
	if (val->val1 < MAX86178_BIOZ_IPOL_NON_INVERTED ||
	    val->val1 > MAX86178_BIOZ_IPOL_INVERTED) {
		LOG_ERR("Invalid BioZ input polarity value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_CFG8, MAX86178_BIOZ_CFG8_BIOZ_IPOL_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ input polarity: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bioz_ipol(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_CFG8, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ input polarity: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_CFG8_BIOZ_IPOL_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BIOZ_STBYON setter/getter */
static int max86178_attr_set_bioz_stbyon(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate enum range */
	if (val->val1 < MAX86178_BIOZ_STBYON_DISABLED || val->val1 > MAX86178_BIOZ_STBYON_ENABLED) {
		LOG_ERR("Invalid BioZ standby on value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_CFG8, MAX86178_BIOZ_CFG8_BIOZ_STBYON_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ standby on: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bioz_stbyon(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_CFG8, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ standby on: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_CFG8_BIOZ_STBYON_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BIOZ_CMRES_DIS setter/getter */
static int max86178_attr_set_bioz_cmres_dis(const struct device *dev,
					    const struct sensor_value *val)
{
	int ret;

	/* Validate enum range */
	if (val->val1 < MAX86178_BIOZ_CMRES_DIS_100M || val->val1 > MAX86178_BIOZ_CMRES_DIS_0) {
		LOG_ERR("Invalid BioZ CM resistor disable value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_CFG8, MAX86178_BIOZ_CFG8_BIOZ_CMRES_DIS_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ CM resistor disable: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bioz_cmres_dis(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_CFG8, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ CM resistor disable: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_CFG8_BIOZ_CMRES_DIS_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BIOZ_RLD_DRV setter/getter */
static int max86178_attr_set_bioz_rld_drv(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate enum range */
	if (val->val1 < MAX86178_BIOZ_RLD_DRV_VMID_TX || val->val1 > MAX86178_BIOZ_RLD_DRV_VRLD) {
		LOG_ERR("Invalid BioZ RLD drive value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_CFG8, MAX86178_BIOZ_CFG8_BIOZ_RLD_DRV_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ RLD drive: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bioz_rld_drv(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_CFG8, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ RLD drive: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_CFG8_BIOZ_RLD_DRV_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BIOZ_RLD_SEL_BIOZ setter/getter */
static int max86178_attr_set_bioz_rld_sel_bioz(const struct device *dev,
					       const struct sensor_value *val)
{
	int ret;

	/* Validate range (0-1) */
	if (val->val1 < 0 || val->val1 > 1) {
		LOG_ERR("Invalid BioZ RLD select value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_CFG8, MAX86178_BIOZ_CFG8_BIOZ_RLD_SEK_BIOZ_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ RLD select: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bioz_rld_sel_bioz(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_CFG8, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ RLD select: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_CFG8_BIOZ_RLD_SEK_BIOZ_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BIOZ_LO_THRESH setter/getter */
static int max86178_attr_set_bioz_lo_thresh(const struct device *dev,
					    const struct sensor_value *val)
{
	int ret;
	uint8_t reg_vals[2];

	/* Validate 16-bit field range */
	if (val->val1 < 0 || val->val1 > 65535) {
		LOG_ERR("Invalid BioZ low threshold value: %d", val->val1);
		return -EINVAL;
	}

	/* Write 16-bit value (big-endian) */
	reg_vals[0] = (uint8_t)((val->val1 >> 8) & 0xFF);
	reg_vals[1] = (uint8_t)(val->val1 & 0xFF);
	ret = max86178_reg_write(dev, MAX86178_BIOZ_LO_THRESH, reg_vals, 2);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ low threshold: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bioz_lo_thresh(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_vals[2];
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_LO_THRESH, reg_vals, 2);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ low threshold: %d", ret);
		return ret;
	}

	/* Read 16-bit value (big-endian) */
	val->val1 = ((uint16_t)reg_vals[0] << 8) | reg_vals[1];
	val->val2 = 0;
	return 0;
}

/* BIOZ_HI_THRESH setter/getter */
static int max86178_attr_set_bioz_hi_thresh(const struct device *dev,
					    const struct sensor_value *val)
{
	int ret;
	uint8_t reg_vals[2];

	/* Validate 16-bit field range */
	if (val->val1 < 0 || val->val1 > 65535) {
		LOG_ERR("Invalid BioZ high threshold value: %d", val->val1);
		return -EINVAL;
	}

	/* Write 16-bit value (big-endian) */
	reg_vals[0] = (uint8_t)((val->val1 >> 8) & 0xFF);
	reg_vals[1] = (uint8_t)(val->val1 & 0xFF);
	ret = max86178_reg_write(dev, MAX86178_BIOZ_HI_THRESH, reg_vals, 2);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ high threshold: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bioz_hi_thresh(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_vals[2];
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_HI_THRESH, reg_vals, 2);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ high threshold: %d", ret);
		return ret;
	}

	/* Read 16-bit value (big-endian) */
	val->val1 = ((uint16_t)reg_vals[0] << 8) | reg_vals[1];
	val->val2 = 0;
	return 0;
}

/* BioZ Calibration Configuration setter/getter functions */

/* BIOZ_CAL_EN setter/getter */
static int max86178_attr_set_bioz_cal_en(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate range (0-1) */
	if (val->val1 < 0 || val->val1 > 1) {
		LOG_ERR("Invalid BioZ calibration enable value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_MUX_CFG1,
				  MAX86178_BIOZ_MUX_CFG1_BIOZ_CAL_EN_MSK, (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ calibration enable: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bioz_cal_en(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_MUX_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ calibration enable: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_MUX_CFG1_BIOZ_CAL_EN_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BIOZ_MUX_EN setter/getter */
static int max86178_attr_set_bioz_mux_en(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate range (0-1) */
	if (val->val1 < 0 || val->val1 > 1) {
		LOG_ERR("Invalid BioZ mux enable value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_MUX_CFG1,
				  MAX86178_BIOZ_MUX_CFG1_BIOZ_MUX_EN_MSK, (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ mux enable: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bioz_mux_en(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_MUX_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ mux enable: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_MUX_CFG1_BIOZ_MUX_EN_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BIOZ_CONNECT_CAL_ONLY setter/getter */
static int max86178_attr_set_bioz_connect_cal_only(const struct device *dev,
						   const struct sensor_value *val)
{
	int ret;

	/* Validate range (0-1) */
	if (val->val1 < 0 || val->val1 > 1) {
		LOG_ERR("Invalid BioZ connect cal only value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_MUX_CFG1,
				  MAX86178_BIOZ_MUX_CFG1_BIOZ_CONNECT_CAL_ONLY_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ connect cal only: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bioz_connect_cal_only(const struct device *dev,
						   struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_MUX_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ connect cal only: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_MUX_CFG1_BIOZ_CONNECT_CAL_ONLY_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BMUX_BIST_EN setter/getter */
static int max86178_attr_set_bmux_bist_en(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate range (0-1) */
	if (val->val1 < 0 || val->val1 > 1) {
		LOG_ERR("Invalid BioZ mux BIST enable value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_MUX_CFG1,
				  MAX86178_BIOZ_MUX_CFG1_BMUX_BIST_EN_MSK, (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ mux BIST enable: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bmux_bist_en(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_MUX_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ mux BIST enable: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_MUX_CFG1_BMUX_BIST_EN_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BMUX_RSEL setter/getter */
/* BMUX_RSEL setter/getter - accepts resistance in ohms (200, 500, 800, 5000) */
static int max86178_attr_set_bmux_rsel(const struct device *dev, const struct sensor_value *val)
{
	uint8_t enum_val;
	int ret;

	/* Convert ohm value to enum */
	ret = bmux_rsel_to_enum(val->val1, &enum_val);
	if (ret < 0) {
		LOG_ERR("Invalid BioZ mux resistor value: %d Ω (valid: 200, 500, 800, 5000)",
			val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_MUX_CFG1, MAX86178_BIOZ_MUX_CFG1_BMUX_RSEL_MSK,
				  enum_val);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ mux resistor select: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bmux_rsel(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int32_t ohms;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_MUX_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ mux resistor select: %d", ret);
		return ret;
	}

	/* Convert enum to ohm value */
	ret = bmux_rsel_from_enum(FIELD_GET(MAX86178_BIOZ_MUX_CFG1_BMUX_RSEL_MSK, reg_val), &ohms);
	if (ret < 0) {
		return ret;
	}

	val->val1 = ohms;
	val->val2 = 0;
	return 0;
}

/* EN_INT_INLOAD setter/getter */
static int max86178_attr_set_en_int_inload(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate range (0-1) */
	if (val->val1 < 0 || val->val1 > 1) {
		LOG_ERR("Invalid internal load enable value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_MUX_CFG2,
				  MAX86178_BIOZ_MUX_CFG2_EN_INT_INLOAD_MSK, (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set internal load enable: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_en_int_inload(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_MUX_CFG2, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read internal load enable: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_MUX_CFG2_EN_INT_INLOAD_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* EN_EXT_INLOAD setter/getter */
static int max86178_attr_set_en_ext_inload(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate range (0-1) */
	if (val->val1 < 0 || val->val1 > 1) {
		LOG_ERR("Invalid external load enable value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_MUX_CFG2,
				  MAX86178_BIOZ_MUX_CFG2_EN_EXT_INLOAD_MSK, (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set external load enable: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_en_ext_inload(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_MUX_CFG2, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read external load enable: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_MUX_CFG2_EN_EXT_INLOAD_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* GSR_LOAD_EN setter/getter */
static int max86178_attr_set_gsr_load_en(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate range (0-1) */
	if (val->val1 < 0 || val->val1 > 1) {
		LOG_ERR("Invalid GSR load enable value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_MUX_CFG2,
				  MAX86178_BIOZ_MUX_CFG2_GSR_LOAD_EN_MSK, (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set GSR load enable: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_gsr_load_en(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_MUX_CFG2, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read GSR load enable: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_MUX_CFG2_GSR_LOAD_EN_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BMUX_GSR_RSEL setter/getter - accepts resistance in kiloohms (25, 100, 500, 1000) */
static int max86178_attr_set_bmux_gsr_rsel(const struct device *dev, const struct sensor_value *val)
{
	uint8_t enum_val;
	int ret;

	/* Convert kilohm value to enum */
	ret = bmux_gsr_rsel_to_enum(val->val1, &enum_val);
	if (ret < 0) {
		LOG_ERR("Invalid BioZ GSR resistor value: %d kΩ (valid: 25, 100, 500, 1000)",
			val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_MUX_CFG2,
				  MAX86178_BIOZ_MUX_CFG2_BMUX_GSR_RSEL_MSK, enum_val);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ mux GSR resistor select: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bmux_gsr_rsel(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int32_t kiloohms;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_MUX_CFG2, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ mux GSR resistor select: %d", ret);
		return ret;
	}

	/* Convert enum to kilohm value */
	ret = bmux_gsr_rsel_from_enum(FIELD_GET(MAX86178_BIOZ_MUX_CFG2_BMUX_GSR_RSEL_MSK, reg_val),
				      &kiloohms);
	if (ret < 0) {
		return ret;
	}

	val->val1 = kiloohms;
	val->val2 = 0;
	return 0;
}

/* DRVN_ASSIGN setter/getter */
static int max86178_attr_set_drvn_assign(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate 2-bit field range */
	if (val->val1 < 0 || val->val1 > 3) {
		LOG_ERR("Invalid DRVN assign value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_MUX_CFG3,
				  MAX86178_BIOZ_MUX_CFG3_DRVN_ASSIGN_MSK, (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set DRVN assign: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_drvn_assign(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_MUX_CFG3, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read DRVN assign: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_MUX_CFG3_DRVN_ASSIGN_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* DRVP_ASSIGN setter/getter */
static int max86178_attr_set_drvp_assign(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate 2-bit field range */
	if (val->val1 < 0 || val->val1 > 3) {
		LOG_ERR("Invalid DRVP assign value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_MUX_CFG3,
				  MAX86178_BIOZ_MUX_CFG3_DRVP_ASSIGN_MSK, (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set DRVP assign: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_drvp_assign(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_MUX_CFG3, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read DRVP assign: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_MUX_CFG3_DRVP_ASSIGN_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BIN_ASSIGN setter/getter */
static int max86178_attr_set_bin_assign(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate enum range */
	if (val->val1 < MAX86178_BIOZ_BIN_ASSIGN_EL4 ||
	    val->val1 > MAX86178_BIOZ_BIN_ASSIGN_UTILITY_ADC) {
		LOG_ERR("Invalid BIN assign value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_MUX_CFG3,
				  MAX86178_BIOZ_MUX_CFG3_BIN_ASSIGN_MSK, (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BIN assign: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bin_assign(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_MUX_CFG3, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BIN assign: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_MUX_CFG3_BIN_ASSIGN_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BIP_ASSIGN setter/getter */
static int max86178_attr_set_bip_assign(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate enum range */
	if (val->val1 < MAX86178_BIOZ_BIP_ASSIGN_EL1 ||
	    val->val1 > MAX86178_BIOZ_BIP_ASSIGN_UTILITY_ADC) {
		LOG_ERR("Invalid BIP assign value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_MUX_CFG3,
				  MAX86178_BIOZ_MUX_CFG3_BIP_ASSIGN_MSK, (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BIP assign: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bip_assign(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_MUX_CFG3, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BIP assign: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_MUX_CFG3_BIP_ASSIGN_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BioZ Lead Detection Configuration setter/getter functions */

/* EN_BIOZ_LON setter/getter */
static int max86178_attr_set_en_bioz_lon(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate enum range */
	if (val->val1 < MAX86178_EN_BIOZ_LON_DISABLED || val->val1 > MAX86178_EN_BIOZ_LON_ENABLED) {
		LOG_ERR("Invalid BioZ lead on enable value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_LD_CFG1, MAX86178_BIOZ_LD_CFG1_EN_BIOZ_LON_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ lead on enable: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_en_bioz_lon(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_LD_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ lead on enable: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_LD_CFG1_EN_BIOZ_LON_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* EN_BIOZ_LOFF setter/getter */
static int max86178_attr_set_en_bioz_loff(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate enum range */
	if (val->val1 < MAX86178_EN_BIOZ_LOFF_DISABLED ||
	    val->val1 > MAX86178_EN_BIOZ_LOFF_ENABLED) {
		LOG_ERR("Invalid BioZ lead off enable value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_LD_CFG1,
				  MAX86178_BIOZ_LD_CFG1_EN_BIOZ_LOFF_MSK, (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ lead off enable: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_en_bioz_loff(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_LD_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ lead off enable: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_LD_CFG1_EN_BIOZ_LOFF_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* EN_EXT_BIOZ_LOFF setter/getter */
static int max86178_attr_set_en_ext_bioz_loff(const struct device *dev,
					      const struct sensor_value *val)
{
	int ret;

	/* Validate enum range */
	if (val->val1 < MAX86178_EN_EXT_BIOZ_LOFF_INTERNAL ||
	    val->val1 > MAX86178_EN_EXT_BIOZ_LOFF_EXTERNAL) {
		LOG_ERR("Invalid BioZ external lead off enable value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_LD_CFG1,
				  MAX86178_BIOZ_LD_CFG1_EN_EXT_BIOZ_LOFF_MSK, (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ external lead off enable: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_en_ext_bioz_loff(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_LD_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ external lead off enable: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_LD_CFG1_EN_EXT_BIOZ_LOFF_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* EN_BIOZ_DRV_OOR setter/getter */
static int max86178_attr_set_en_bioz_drv_oor(const struct device *dev,
					     const struct sensor_value *val)
{
	int ret;

	/* Validate enum range */
	if (val->val1 < MAX86178_EN_BIOZ_DRV_OOR_DISABLED ||
	    val->val1 > MAX86178_EN_BIOZ_DRV_OOR_ENABLED) {
		LOG_ERR("Invalid BioZ drive out-of-range enable value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_LD_CFG1,
				  MAX86178_BIOZ_LD_CFG1_EN_BIOZ_DRV_OOR_MSK, (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ drive out-of-range enable: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_en_bioz_drv_oor(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_LD_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ drive out-of-range enable: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_LD_CFG1_EN_BIOZ_DRV_OOR_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BIOZ_LOFF_IPOL setter/getter */
static int max86178_attr_set_bioz_loff_ipol(const struct device *dev,
					    const struct sensor_value *val)
{
	int ret;

	/* Validate range (0-1) */
	if (val->val1 < 0 || val->val1 > 1) {
		LOG_ERR("Invalid BioZ lead off current polarity value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_LD_CFG1,
				  MAX86178_BIOZ_LD_CFG1_BIOZ_LOFF_IPOL_MSK, (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ lead off current polarity: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bioz_loff_ipol(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_LD_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ lead off current polarity: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_LD_CFG1_BIOZ_LOFF_IPOL_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BIOZ_LOFF_IMAG setter/getter */
static int max86178_attr_set_bioz_loff_imag(const struct device *dev,
					    const struct sensor_value *val)
{
	int ret;

	/* Validate 3-bit field range */
	if (val->val1 < 0 || val->val1 > 7) {
		LOG_ERR("Invalid BioZ lead off current magnitude value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_LD_CFG1,
				  MAX86178_BIOZ_LD_CFG1_BIOZ_LOFF_IMAG_MSK, (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ lead off current magnitude: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bioz_loff_imag(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_LD_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ lead off current magnitude: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_LD_CFG1_BIOZ_LOFF_IMAG_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BIOZ_LOFF_THRESH setter/getter */
static int max86178_attr_set_bioz_loff_thresh(const struct device *dev,
					      const struct sensor_value *val)
{
	int ret;

	/* Validate 4-bit field range */
	if (val->val1 < 0 || val->val1 > 15) {
		LOG_ERR("Invalid BioZ lead off threshold value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_LOFF_THRESH,
				  MAX86178_BIOZ_LOFF_THRESH_BIOZ_LOFF_THRESH_MSK,
				  (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ lead off threshold: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bioz_loff_thresh(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_LOFF_THRESH, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ lead off threshold: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_LOFF_THRESH_BIOZ_LOFF_THRESH_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* RESP_CG_MAG setter/getter */
static int max86178_attr_set_resp_cg_mag(const struct device *dev, const struct sensor_value *val)
{
	int ret;

	/* Validate 3-bit field range */
	if (val->val1 < 0 || val->val1 > 7) {
		LOG_ERR("Invalid respiration charge magnitude value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_LOFF_THRESH,
				  MAX86178_BIOZ_LOFF_THRESH_RESP_CG_MAG_MSK, (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set respiration charge magnitude: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_resp_cg_mag(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_LOFF_THRESH, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read respiration charge magnitude: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_LOFF_THRESH_RESP_CG_MAG_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* RESP_CG_MAG_4X setter/getter */
static int max86178_attr_set_resp_cg_mag_4x(const struct device *dev,
					    const struct sensor_value *val)
{
	int ret;

	/* Validate range (0-1) */
	if (val->val1 < 0 || val->val1 > 1) {
		LOG_ERR("Invalid respiration charge magnitude 4x value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_LOFF_THRESH,
				  MAX86178_BIOZ_LOFF_THRESH_RESP_CG_MAG_4X_MSK, (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set respiration charge magnitude 4x: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_resp_cg_mag_4x(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_LOFF_THRESH, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read respiration charge magnitude 4x: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_LOFF_THRESH_RESP_CG_MAG_4X_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* BioZ Lead Bias Configuration setter/getter functions */

/* BIOZ_RBIAS_VALUE setter/getter - accepts resistance in megaohms (50, 100, 200) */
static int max86178_attr_set_bioz_rbias_value(const struct device *dev,
					      const struct sensor_value *val)
{
	uint8_t enum_val;
	int ret;

	/* Convert megaohm value to enum */
	ret = bioz_rbias_value_to_enum(val->val1, &enum_val);
	if (ret < 0) {
		LOG_ERR("Invalid BioZ bias resistor value: %d MΩ (valid: 50, 100, 200)", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_LB_CFG1,
				  MAX86178_BIOZ_LB_CFG1_BIOZ_RBIAS_VALUE_MSK, enum_val);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ bias resistor value: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_bioz_rbias_value(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int32_t megaohms;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_LB_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ bias resistor value: %d", ret);
		return ret;
	}

	/* Convert enum to megaohm value */
	ret = bioz_rbias_value_from_enum(
		FIELD_GET(MAX86178_BIOZ_LB_CFG1_BIOZ_RBIAS_VALUE_MSK, reg_val), &megaohms);
	if (ret < 0) {
		return ret;
	}

	val->val1 = megaohms;
	val->val2 = 0;
	return 0;
}

/* EN_BIOZ_RBIAS_P setter/getter */
static int max86178_attr_set_en_bioz_rbias_p(const struct device *dev,
					     const struct sensor_value *val)
{
	int ret;

	/* Validate range (0-1) */
	if (val->val1 < 0 || val->val1 > 1) {
		LOG_ERR("Invalid BioZ bias resistor P enable value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_LB_CFG1,
				  MAX86178_BIOZ_LB_CFG1_EN_BIOZ_RBIAS_P_MSK, (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ bias resistor P enable: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_en_bioz_rbias_p(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_LB_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ bias resistor P enable: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_LB_CFG1_EN_BIOZ_RBIAS_P_MSK, reg_val);
	val->val2 = 0;
	return 0;
}

/* EN_BIOZ_RBIAS_N setter/getter */
static int max86178_attr_set_en_bioz_rbias_n(const struct device *dev,
					     const struct sensor_value *val)
{
	int ret;

	/* Validate range (0-1) */
	if (val->val1 < 0 || val->val1 > 1) {
		LOG_ERR("Invalid BioZ bias resistor N enable value: %d", val->val1);
		return -EINVAL;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_LB_CFG1,
				  MAX86178_BIOZ_LB_CFG1_EN_BIOZ_RBIAS_N_MSK, (uint8_t)val->val1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ bias resistor N enable: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_attr_get_en_bioz_rbias_n(const struct device *dev, struct sensor_value *val)
{
	uint8_t reg_val;
	int ret;

	ret = max86178_reg_read(dev, MAX86178_BIOZ_LB_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read BioZ bias resistor N enable: %d", ret);
		return ret;
	}

	val->val1 = FIELD_GET(MAX86178_BIOZ_LB_CFG1_EN_BIOZ_RBIAS_N_MSK, reg_val);
	val->val2 = 0;
	return 0;
}
static int max86178_attr_set(const struct device *dev, enum sensor_channel chan,
			     enum sensor_attribute attr, const struct sensor_value *val)
{
	int ret = 0;

	if (val == NULL) {
		LOG_ERR("Null pointer passed for sensor value");
		return -EINVAL;
	}

	switch ((int)attr) {
	case SENSOR_ATTR_MAX86178_PPG_THRESH1_HI:
		ret = max86178_set_ppg_thresh1_hi(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_PPG_THRESH1_LO:
		ret = max86178_set_ppg_thresh1_lo(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_PPG_THRESH2_HI:
		ret = max86178_set_ppg_thresh2_hi(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_PPG_THRESH2_LO:
		ret = max86178_set_ppg_thresh2_lo(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_PPG_THRESH1_MEAS_SEL:
		ret = max86178_set_ppg_thresh1_meas_sel(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_PPG_THRESH2_MEAS_SEL:
		ret = max86178_set_ppg_thresh2_meas_sel(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_PPG_THRESH1_CHAN_SEL:
		ret = max86178_set_ppg_thresh1_chan_sel(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_PPG_THRESH2_CHAN_SEL:
		ret = max86178_set_ppg_thresh2_chan_sel(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_PPG_TIME_HYST:
		ret = max86178_set_ppg_time_hyst(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_PPG_LEVEL_HYST:
		ret = max86178_set_ppg_level_hyst(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_PPG1_PWRDN:
		ret = max86178_set_ppg1_pwrdn(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_PPG2_PWRDN:
		ret = max86178_set_ppg2_pwrdn(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_PPG_SYNC_MODE:
		ret = max86178_set_ppg_sync_mode(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_PROX_DATA_EN:
		ret = max86178_set_prox_data_en(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_PROX_AUTO_EN:
		ret = max86178_set_prox_auto_en(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_ALC_DISABLE:
		ret = max86178_set_alc_disable(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_COLLECT_RAW_DATA:
		ret = max86178_set_collect_raw_data(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_MEAS1_CONFIG_SEL:
		ret = max86178_set_meas1_config_sel(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_PD1_BIAS:
		ret = max86178_set_pd_bias(dev, val, 1);
		break;
	case SENSOR_ATTR_MAX86178_PD2_BIAS:
		ret = max86178_set_pd_bias(dev, val, 2);
		break;
	case SENSOR_ATTR_MAX86178_PD3_BIAS:
		ret = max86178_set_pd_bias(dev, val, 3);
		break;
	case SENSOR_ATTR_MAX86178_PD4_BIAS:
		ret = max86178_set_pd_bias(dev, val, 4);
		break;
	case SENSOR_ATTR_MAX86178_SMP_AVE:
		ret = max86178_set_smp_ave(dev, val);
		break;
	/* PPG Measurement Config Attributes (per-channel) */
	case SENSOR_ATTR_MAX86178_PPG_DRVA:
		ret = max86178_set_ppg_drva(dev, chan, val);
		break;
	case SENSOR_ATTR_MAX86178_PPG_DRVB:
		ret = max86178_set_ppg_drvb(dev, chan, val);
		break;
	case SENSOR_ATTR_MAX86178_PPG_DRVA_PA:
		ret = max86178_set_ppg_drva_pa(dev, chan, val);
		break;
	case SENSOR_ATTR_MAX86178_PPG_DRVB_PA:
		ret = max86178_set_ppg_drvb_pa(dev, chan, val);
		break;
	case SENSOR_ATTR_MAX86178_PPG_AMB_MODE:
		ret = max86178_set_ppg_amb_mode(dev, chan, val);
		break;
	case SENSOR_ATTR_MAX86178_PPG_AVG_NUM:
		ret = max86178_set_ppg_avg_num(dev, chan, val);
		break;
	case SENSOR_ATTR_MAX86178_PPG_SINC3_SEL:
		ret = max86178_set_ppg_sinc3_sel(dev, chan, val);
		break;
	case SENSOR_ATTR_MAX86178_PPG_FILT_SEL:
		ret = max86178_set_ppg_filt_sel(dev, chan, val);
		break;
	case SENSOR_ATTR_MAX86178_PPG_FILT2_SEL:
		ret = max86178_set_ppg_filt2_sel(dev, chan, val);
		break;
	case SENSOR_ATTR_MAX86178_PPG_TINT:
		ret = max86178_set_ppg_tint(dev, chan, val);
		break;
	case SENSOR_ATTR_MAX86178_PPG1_ADC_RGE:
		ret = max86178_set_ppg1_adc_rge(dev, chan, val);
		break;
	case SENSOR_ATTR_MAX86178_PPG2_ADC_RGE:
		ret = max86178_set_ppg2_adc_rge(dev, chan, val);
		break;
	case SENSOR_ATTR_MAX86178_PPG1_DAC_OFF:
		ret = max86178_set_ppg1_dac_off(dev, chan, val);
		break;
	case SENSOR_ATTR_MAX86178_PPG2_DAC_OFF:
		ret = max86178_set_ppg2_dac_off(dev, chan, val);
		break;
	case SENSOR_ATTR_MAX86178_PPG_LED_RGE:
		ret = max86178_set_ppg_led_rge(dev, chan, val);
		break;
	case SENSOR_ATTR_MAX86178_PPG_LED_SETLNG:
		ret = max86178_set_ppg_led_setlng(dev, chan, val);
		break;
	case SENSOR_ATTR_MAX86178_PPG_PD_SETLNG:
		ret = max86178_set_ppg_pd_setlng(dev, chan, val);
		break;
	case SENSOR_ATTR_MAX86178_PPG_PD1_SEL:
		ret = max86178_set_ppg_pd1_sel(dev, chan, val);
		break;
	case SENSOR_ATTR_MAX86178_PPG_PD2_SEL:
		ret = max86178_set_ppg_pd2_sel(dev, chan, val);
		break;
	case SENSOR_ATTR_MAX86178_PPG_PD3_SEL:
		ret = max86178_set_ppg_pd3_sel(dev, chan, val);
		break;
	case SENSOR_ATTR_MAX86178_PPG_PD4_SEL:
		ret = max86178_set_ppg_pd4_sel(dev, chan, val);
		break;
	/* ECG Setup Configuration */
	case SENSOR_ATTR_MAX86178_ECG_INPUT_POL:
		ret = max86178_set_ecg_input_pol(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_ECG_PGA_GAIN:
		ret = max86178_set_ecg_pga_gain(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_ECG_INA_RGE:
		ret = max86178_set_ecg_ina_rge(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_ECG_INA_GAIN:
		ret = max86178_set_ecg_ina_gain(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_ECG_IMP_HI:
		ret = max86178_set_ecg_imp_hi(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_ECG_AUTO_REC:
		ret = max86178_set_ecg_auto_rec(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_ECG_MUX_SEL:
		ret = max86178_set_ecg_mux_sel(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_EN_ECG_FAST_REC:
		ret = max86178_set_en_ecg_fast_rec(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_ECG_FAST_REC_THRES:
		ret = max86178_set_ecg_fast_rec_thres(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_ECG_CAL_FREQ:
		ret = max86178_set_ecg_cal_freq(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_ECG_CAL_DUTY:
		ret = max86178_set_ecg_cal_duty(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_ECG_CAL_EN:
		ret = max86178_set_ecg_cal_en(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_ECG_CAL_HIGH:
		ret = max86178_set_ecg_cal_high(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_ECG_OPEN_P:
		ret = max86178_set_ecg_open_p(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_ECG_OPEN_N:
		ret = max86178_set_ecg_open_n(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_ECG_CAL_MODE:
		ret = max86178_set_ecg_cal_mode(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_ECG_CAL_MAG:
		ret = max86178_set_ecg_cal_mag(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_ECG_CAL_P_SEL:
		ret = max86178_set_ecg_cal_p_sel(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_ECG_CAL_N_SEL:
		ret = max86178_set_ecg_cal_n_sel(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_EN_ECG_LON:
		ret = max86178_set_en_ecg_lon(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_EN_ECG_LOFF:
		ret = max86178_set_en_ecg_loff(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_ECG_LOFF_MODE:
		ret = max86178_set_ecg_loff_mode(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_ECG_LOFF_FREQ:
		ret = max86178_set_ecg_loff_freq(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_ECG_LOFF_IPOL:
		ret = max86178_set_ecg_loff_ipol(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_ECG_LOFF_IMAG:
		ret = max86178_set_ecg_loff_imag(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_ECG_LOFF_THRESH:
		ret = max86178_set_ecg_loff_thresh(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_ECG_RBIAS_VALUE:
		ret = max86178_set_ecg_rbias_value(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_EN_ECG_RBIAS_P:
		ret = max86178_set_en_ecg_rbias_p(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_EN_ECG_RBIAS_N:
		ret = max86178_set_en_ecg_rbias_n(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_RLD_EN:
		ret = max86178_set_rld_en(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_RLD_MODE:
		ret = max86178_set_rld_mode(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_RLD_RBIAS:
		ret = max86178_set_rld_rbias(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_EN_RLD_OOR:
		ret = max86178_set_en_rld_oor(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_ACTV_CM_P:
		ret = max86178_set_actv_cm_p(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_ACTV_CM_N:
		ret = max86178_set_actv_cm_n(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_RLD_GAIN:
		ret = max86178_set_rld_gain(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_RLD_EXT_RES:
		ret = max86178_set_rld_ext_res(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_RLD_SEL_ECG:
		ret = max86178_set_rld_sel_ecg(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_RLD_BW:
		ret = max86178_set_rld_bw(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BODY_BIAS_DAC:
		ret = max86178_set_body_bias_dac(dev, val);
		break;
	/* BioZ Setup Configuration */
	case SENSOR_ATTR_MAX86178_BIOZ_ADC_OSR:
		ret = max86178_attr_set_bioz_adc_osr(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BIOZ_DAC_OSR:
		ret = max86178_attr_set_bioz_dac_osr(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_EN_BIOZ_THRESH:
		ret = max86178_attr_set_en_bioz_thresh(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BIOZ_DLPF:
		ret = max86178_attr_set_bioz_dlpf(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BIOZ_DHPF:
		ret = max86178_attr_set_bioz_dhpf(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BIOZ_DRV_MODE:
		ret = max86178_attr_set_bioz_drv_mode(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BIOZ_IDRV_RGE:
		ret = max86178_attr_set_bioz_idrv_rge(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BIOZ_VDRV_MAG:
		ret = max86178_attr_set_bioz_vdrv_mag(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BIOZ_EXT_RES:
		ret = max86178_attr_set_bioz_ext_res(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_EN_UTIL_MODE:
		ret = max86178_attr_set_en_util_mode(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BIOZ_DC_DAC_CODE:
		ret = max86178_attr_set_bioz_dc_dac_code(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BIOZ_DC_CODE_SEL:
		ret = max86178_attr_set_bioz_dc_code_sel(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BIOZ_GAIN:
		ret = max86178_attr_set_bioz_gain(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_DM_DIS:
		ret = max86178_attr_set_dm_dis(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BIOZ_INA_MODE:
		ret = max86178_attr_set_bioz_ina_mode(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BIOZ_AHPF:
		ret = max86178_attr_set_bioz_ahpf(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BIOZ_AMP_BW:
		ret = max86178_attr_set_bioz_amp_bw(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BIOZ_AMP_RGE:
		ret = max86178_attr_set_bioz_amp_rge(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BIOZ_DAC_RESET:
		ret = max86178_attr_set_bioz_dac_reset(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BIOZ_DRV_RESET:
		ret = max86178_attr_set_bioz_drv_reset(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BIOZ_DC_RESTORE:
		ret = max86178_attr_set_bioz_dc_restore(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BIOZ_EXT_CAP:
		ret = max86178_attr_set_bioz_ext_cap(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BIOZ_CH_FSEL:
		ret = max86178_attr_set_bioz_ch_fsel(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BIOZ_INA_CHOP_EN:
		ret = max86178_attr_set_bioz_ina_chop_en(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BIOZ_FAST:
		ret = max86178_attr_set_bioz_fast(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BIOZ_IPOL:
		ret = max86178_attr_set_bioz_ipol(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BIOZ_STBYON:
		ret = max86178_attr_set_bioz_stbyon(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BIOZ_CMRES_DIS:
		ret = max86178_attr_set_bioz_cmres_dis(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BIOZ_RLD_DRV:
		ret = max86178_attr_set_bioz_rld_drv(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BIOZ_RLD_SEL_BIOZ:
		ret = max86178_attr_set_bioz_rld_sel_bioz(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BIOZ_LO_THRESH:
		ret = max86178_attr_set_bioz_lo_thresh(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BIOZ_HI_THRESH:
		ret = max86178_attr_set_bioz_hi_thresh(dev, val);
		break;
	/* BioZ Calibration Configuration */
	case SENSOR_ATTR_MAX86178_BIOZ_CAL_EN:
		ret = max86178_attr_set_bioz_cal_en(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BIOZ_MUX_EN:
		ret = max86178_attr_set_bioz_mux_en(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BIOZ_CONNECT_CAL_ONLY:
		ret = max86178_attr_set_bioz_connect_cal_only(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BMUX_BIST_EN:
		ret = max86178_attr_set_bmux_bist_en(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BMUX_RSEL:
		ret = max86178_attr_set_bmux_rsel(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_EN_INT_INLOAD:
		ret = max86178_attr_set_en_int_inload(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_EN_EXT_INLOAD:
		ret = max86178_attr_set_en_ext_inload(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_GSR_LOAD_EN:
		ret = max86178_attr_set_gsr_load_en(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BMUX_GSR_RSEL:
		ret = max86178_attr_set_bmux_gsr_rsel(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_DRVN_ASSIGN:
		ret = max86178_attr_set_drvn_assign(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_DRVP_ASSIGN:
		ret = max86178_attr_set_drvp_assign(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BIN_ASSIGN:
		ret = max86178_attr_set_bin_assign(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BIP_ASSIGN:
		ret = max86178_attr_set_bip_assign(dev, val);
		break;
	/* BioZ Lead Detection Configuration */
	case SENSOR_ATTR_MAX86178_EN_BIOZ_LON:
		ret = max86178_attr_set_en_bioz_lon(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_EN_BIOZ_LOFF:
		ret = max86178_attr_set_en_bioz_loff(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_EN_EXT_BIOZ_LOFF:
		ret = max86178_attr_set_en_ext_bioz_loff(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_EN_BIOZ_DRV_OOR:
		ret = max86178_attr_set_en_bioz_drv_oor(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BIOZ_LOFF_IPOL:
		ret = max86178_attr_set_bioz_loff_ipol(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BIOZ_LOFF_IMAG:
		ret = max86178_attr_set_bioz_loff_imag(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_BIOZ_LOFF_THRESH:
		ret = max86178_attr_set_bioz_loff_thresh(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_RESP_CG_MAG:
		ret = max86178_attr_set_resp_cg_mag(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_RESP_CG_MAG_4X:
		ret = max86178_attr_set_resp_cg_mag_4x(dev, val);
		break;
	/* BioZ Lead Bias Configuration */
	case SENSOR_ATTR_MAX86178_BIOZ_RBIAS_VALUE:
		ret = max86178_attr_set_bioz_rbias_value(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_EN_BIOZ_RBIAS_P:
		ret = max86178_attr_set_en_bioz_rbias_p(dev, val);
		break;
	case SENSOR_ATTR_MAX86178_EN_BIOZ_RBIAS_N:
		ret = max86178_attr_set_en_bioz_rbias_n(dev, val);
		break;
	/* TODO: Implement channel enable attributes with special considerations */
	case SENSOR_ATTR_MAX86178_MEAS1_EN:
	case SENSOR_ATTR_MAX86178_MEAS2_EN:
	case SENSOR_ATTR_MAX86178_MEAS3_EN:
	case SENSOR_ATTR_MAX86178_MEAS4_EN:
	case SENSOR_ATTR_MAX86178_MEAS5_EN:
	case SENSOR_ATTR_MAX86178_MEAS6_EN:
	case SENSOR_ATTR_MAX86178_ECG_EN:
	case SENSOR_ATTR_MAX86178_BIOZ_EN:
	case SENSOR_ATTR_MAX86178_ECG_BIOZ_BG_EN:
	case SENSOR_ATTR_MAX86178_OSC_EN:
		LOG_WRN("Channel enable attribute not yet implemented");
		return -ENOTSUP;
	default:
		LOG_ERR("Attribute not supported");
		return -ENOTSUP;
	}

	if (ret != 0) {
		LOG_ERR("Failed to set attribute");
		return ret;
	}

	return 0;
}

static int max86178_attr_get(const struct device *dev, enum sensor_channel chan,
			     enum sensor_attribute attr, struct sensor_value *val)
{
	if (val == NULL) {
		LOG_ERR("Null pointer passed for sensor value");
		return -EINVAL;
	}

	switch ((int)attr) {
	case SENSOR_ATTR_MAX86178_PPG_THRESH1_HI:
		return max86178_get_ppg_thresh1_hi(dev, val);
	case SENSOR_ATTR_MAX86178_PPG_THRESH1_LO:
		return max86178_get_ppg_thresh1_lo(dev, val);
	case SENSOR_ATTR_MAX86178_PPG_THRESH2_HI:
		return max86178_get_ppg_thresh2_hi(dev, val);
	case SENSOR_ATTR_MAX86178_PPG_THRESH2_LO:
		return max86178_get_ppg_thresh2_lo(dev, val);
	case SENSOR_ATTR_MAX86178_PPG_THRESH1_MEAS_SEL:
		return max86178_get_ppg_thresh1_meas_sel(dev, val);
	case SENSOR_ATTR_MAX86178_PPG_THRESH2_MEAS_SEL:
		return max86178_get_ppg_thresh2_meas_sel(dev, val);
	case SENSOR_ATTR_MAX86178_PPG_THRESH1_CHAN_SEL:
		return max86178_get_ppg_thresh1_chan_sel(dev, val);
	case SENSOR_ATTR_MAX86178_PPG_THRESH2_CHAN_SEL:
		return max86178_get_ppg_thresh2_chan_sel(dev, val);
	case SENSOR_ATTR_MAX86178_PPG_TIME_HYST:
		return max86178_get_ppg_time_hyst(dev, val);
	case SENSOR_ATTR_MAX86178_PPG_LEVEL_HYST:
		return max86178_get_ppg_level_hyst(dev, val);
	case SENSOR_ATTR_MAX86178_PPG1_PWRDN:
		return max86178_get_ppg1_pwrdn(dev, val);
	case SENSOR_ATTR_MAX86178_PPG2_PWRDN:
		return max86178_get_ppg2_pwrdn(dev, val);
	case SENSOR_ATTR_MAX86178_PPG_SYNC_MODE:
		return max86178_get_ppg_sync_mode(dev, val);
	case SENSOR_ATTR_MAX86178_PROX_DATA_EN:
		return max86178_get_prox_data_en(dev, val);
	case SENSOR_ATTR_MAX86178_PROX_AUTO_EN:
		return max86178_get_prox_auto_en(dev, val);
	case SENSOR_ATTR_MAX86178_ALC_DISABLE:
		return max86178_get_alc_disable(dev, val);
	case SENSOR_ATTR_MAX86178_COLLECT_RAW_DATA:
		return max86178_get_collect_raw_data(dev, val);
	case SENSOR_ATTR_MAX86178_MEAS1_CONFIG_SEL:
		return max86178_get_meas1_config_sel(dev, val);
	case SENSOR_ATTR_MAX86178_PD1_BIAS:
		return max86178_get_pd_bias(dev, val, 1);
	case SENSOR_ATTR_MAX86178_PD2_BIAS:
		return max86178_get_pd_bias(dev, val, 2);
	case SENSOR_ATTR_MAX86178_PD3_BIAS:
		return max86178_get_pd_bias(dev, val, 3);
	case SENSOR_ATTR_MAX86178_PD4_BIAS:
		return max86178_get_pd_bias(dev, val, 4);
	case SENSOR_ATTR_MAX86178_SMP_AVE:
		return max86178_get_smp_ave(dev, val);
	/* PPG Measurement Config Attributes (per-channel) */
	case SENSOR_ATTR_MAX86178_PPG_DRVA:
		return max86178_get_ppg_drva(dev, chan, val);
	case SENSOR_ATTR_MAX86178_PPG_DRVB:
		return max86178_get_ppg_drvb(dev, chan, val);
	case SENSOR_ATTR_MAX86178_PPG_DRVA_PA:
		return max86178_get_ppg_drva_pa(dev, chan, val);
	case SENSOR_ATTR_MAX86178_PPG_DRVB_PA:
		return max86178_get_ppg_drvb_pa(dev, chan, val);
	case SENSOR_ATTR_MAX86178_PPG_AMB_MODE:
		return max86178_get_ppg_amb_mode(dev, chan, val);
	case SENSOR_ATTR_MAX86178_PPG_AVG_NUM:
		return max86178_get_ppg_avg_num(dev, chan, val);
	case SENSOR_ATTR_MAX86178_PPG_SINC3_SEL:
		return max86178_get_ppg_sinc3_sel(dev, chan, val);
	case SENSOR_ATTR_MAX86178_PPG_FILT_SEL:
		return max86178_get_ppg_filt_sel(dev, chan, val);
	case SENSOR_ATTR_MAX86178_PPG_FILT2_SEL:
		return max86178_get_ppg_filt2_sel(dev, chan, val);
	case SENSOR_ATTR_MAX86178_PPG_TINT:
		return max86178_get_ppg_tint(dev, chan, val);
	case SENSOR_ATTR_MAX86178_PPG1_ADC_RGE:
		return max86178_get_ppg1_adc_rge(dev, chan, val);
	case SENSOR_ATTR_MAX86178_PPG2_ADC_RGE:
		return max86178_get_ppg2_adc_rge(dev, chan, val);
	case SENSOR_ATTR_MAX86178_PPG1_DAC_OFF:
		return max86178_get_ppg1_dac_off(dev, chan, val);
	case SENSOR_ATTR_MAX86178_PPG2_DAC_OFF:
		return max86178_get_ppg2_dac_off(dev, chan, val);
	case SENSOR_ATTR_MAX86178_PPG_LED_RGE:
		return max86178_get_ppg_led_rge(dev, chan, val);
	case SENSOR_ATTR_MAX86178_PPG_LED_SETLNG:
		return max86178_get_ppg_led_setlng(dev, chan, val);
	case SENSOR_ATTR_MAX86178_PPG_PD_SETLNG:
		return max86178_get_ppg_pd_setlng(dev, chan, val);
	case SENSOR_ATTR_MAX86178_PPG_PD1_SEL:
		return max86178_get_ppg_pd1_sel(dev, chan, val);
	case SENSOR_ATTR_MAX86178_PPG_PD2_SEL:
		return max86178_get_ppg_pd2_sel(dev, chan, val);
	case SENSOR_ATTR_MAX86178_PPG_PD3_SEL:
		return max86178_get_ppg_pd3_sel(dev, chan, val);
	case SENSOR_ATTR_MAX86178_PPG_PD4_SEL:
		return max86178_get_ppg_pd4_sel(dev, chan, val);
	/* ECG Setup Configuration */
	case SENSOR_ATTR_MAX86178_ECG_INPUT_POL:
		return max86178_get_ecg_input_pol(dev, val);
	case SENSOR_ATTR_MAX86178_ECG_PGA_GAIN:
		return max86178_get_ecg_pga_gain(dev, val);
	case SENSOR_ATTR_MAX86178_ECG_INA_RGE:
		return max86178_get_ecg_ina_rge(dev, val);
	case SENSOR_ATTR_MAX86178_ECG_INA_GAIN:
		return max86178_get_ecg_ina_gain(dev, val);
	case SENSOR_ATTR_MAX86178_ECG_IMP_HI:
		return max86178_get_ecg_imp_hi(dev, val);
	case SENSOR_ATTR_MAX86178_ECG_AUTO_REC:
		return max86178_get_ecg_auto_rec(dev, val);
	case SENSOR_ATTR_MAX86178_ECG_MUX_SEL:
		return max86178_get_ecg_mux_sel(dev, val);
	case SENSOR_ATTR_MAX86178_EN_ECG_FAST_REC:
		return max86178_get_en_ecg_fast_rec(dev, val);
	case SENSOR_ATTR_MAX86178_ECG_FAST_REC_THRES:
		return max86178_get_ecg_fast_rec_thres(dev, val);
	case SENSOR_ATTR_MAX86178_ECG_CAL_FREQ:
		return max86178_get_ecg_cal_freq(dev, val);
	case SENSOR_ATTR_MAX86178_ECG_CAL_DUTY:
		return max86178_get_ecg_cal_duty(dev, val);
	case SENSOR_ATTR_MAX86178_ECG_CAL_EN:
		return max86178_get_ecg_cal_en(dev, val);
	case SENSOR_ATTR_MAX86178_ECG_CAL_HIGH:
		return max86178_get_ecg_cal_high(dev, val);
	case SENSOR_ATTR_MAX86178_ECG_OPEN_P:
		return max86178_get_ecg_open_p(dev, val);
	case SENSOR_ATTR_MAX86178_ECG_OPEN_N:
		return max86178_get_ecg_open_n(dev, val);
	case SENSOR_ATTR_MAX86178_ECG_CAL_MODE:
		return max86178_get_ecg_cal_mode(dev, val);
	case SENSOR_ATTR_MAX86178_ECG_CAL_MAG:
		return max86178_get_ecg_cal_mag(dev, val);
	case SENSOR_ATTR_MAX86178_ECG_CAL_P_SEL:
		return max86178_get_ecg_cal_p_sel(dev, val);
	case SENSOR_ATTR_MAX86178_ECG_CAL_N_SEL:
		return max86178_get_ecg_cal_n_sel(dev, val);
	case SENSOR_ATTR_MAX86178_EN_ECG_LON:
		return max86178_get_en_ecg_lon(dev, val);
	case SENSOR_ATTR_MAX86178_EN_ECG_LOFF:
		return max86178_get_en_ecg_loff(dev, val);
	case SENSOR_ATTR_MAX86178_ECG_LOFF_MODE:
		return max86178_get_ecg_loff_mode(dev, val);
	case SENSOR_ATTR_MAX86178_ECG_LOFF_FREQ:
		return max86178_get_ecg_loff_freq(dev, val);
	case SENSOR_ATTR_MAX86178_ECG_LOFF_IPOL:
		return max86178_get_ecg_loff_ipol(dev, val);
	case SENSOR_ATTR_MAX86178_ECG_LOFF_IMAG:
		return max86178_get_ecg_loff_imag(dev, val);
	case SENSOR_ATTR_MAX86178_ECG_LOFF_THRESH:
		return max86178_get_ecg_loff_thresh(dev, val);
	case SENSOR_ATTR_MAX86178_ECG_RBIAS_VALUE:
		return max86178_get_ecg_rbias_value(dev, val);
	case SENSOR_ATTR_MAX86178_EN_ECG_RBIAS_P:
		return max86178_get_en_ecg_rbias_p(dev, val);
	case SENSOR_ATTR_MAX86178_EN_ECG_RBIAS_N:
		return max86178_get_en_ecg_rbias_n(dev, val);
	case SENSOR_ATTR_MAX86178_RLD_EN:
		return max86178_get_rld_en(dev, val);
	case SENSOR_ATTR_MAX86178_RLD_MODE:
		return max86178_get_rld_mode(dev, val);
	case SENSOR_ATTR_MAX86178_RLD_RBIAS:
		return max86178_get_rld_rbias(dev, val);
	case SENSOR_ATTR_MAX86178_EN_RLD_OOR:
		return max86178_get_en_rld_oor(dev, val);
	case SENSOR_ATTR_MAX86178_ACTV_CM_P:
		return max86178_get_actv_cm_p(dev, val);
	case SENSOR_ATTR_MAX86178_ACTV_CM_N:
		return max86178_get_actv_cm_n(dev, val);
	case SENSOR_ATTR_MAX86178_RLD_GAIN:
		return max86178_get_rld_gain(dev, val);
	case SENSOR_ATTR_MAX86178_RLD_EXT_RES:
		return max86178_get_rld_ext_res(dev, val);
	case SENSOR_ATTR_MAX86178_RLD_SEL_ECG:
		return max86178_get_rld_sel_ecg(dev, val);
	case SENSOR_ATTR_MAX86178_RLD_BW:
		return max86178_get_rld_bw(dev, val);
	case SENSOR_ATTR_MAX86178_BODY_BIAS_DAC:
		return max86178_get_body_bias_dac(dev, val);
	/* BioZ Setup Configuration */
	case SENSOR_ATTR_MAX86178_BIOZ_ADC_OSR:
		return max86178_attr_get_bioz_adc_osr(dev, val);
	case SENSOR_ATTR_MAX86178_BIOZ_DAC_OSR:
		return max86178_attr_get_bioz_dac_osr(dev, val);
	case SENSOR_ATTR_MAX86178_EN_BIOZ_THRESH:
		return max86178_attr_get_en_bioz_thresh(dev, val);
	case SENSOR_ATTR_MAX86178_BIOZ_DLPF:
		return max86178_attr_get_bioz_dlpf(dev, val);
	case SENSOR_ATTR_MAX86178_BIOZ_DHPF:
		return max86178_attr_get_bioz_dhpf(dev, val);
	case SENSOR_ATTR_MAX86178_BIOZ_DRV_MODE:
		return max86178_attr_get_bioz_drv_mode(dev, val);
	case SENSOR_ATTR_MAX86178_BIOZ_IDRV_RGE:
		return max86178_attr_get_bioz_idrv_rge(dev, val);
	case SENSOR_ATTR_MAX86178_BIOZ_VDRV_MAG:
		return max86178_attr_get_bioz_vdrv_mag(dev, val);
	case SENSOR_ATTR_MAX86178_BIOZ_EXT_RES:
		return max86178_attr_get_bioz_ext_res(dev, val);
	case SENSOR_ATTR_MAX86178_EN_UTIL_MODE:
		return max86178_attr_get_en_util_mode(dev, val);
	case SENSOR_ATTR_MAX86178_BIOZ_DC_DAC_CODE:
		return max86178_attr_get_bioz_dc_dac_code(dev, val);
	case SENSOR_ATTR_MAX86178_BIOZ_DC_CODE_SEL:
		return max86178_attr_get_bioz_dc_code_sel(dev, val);
	case SENSOR_ATTR_MAX86178_BIOZ_GAIN:
		return max86178_attr_get_bioz_gain(dev, val);
	case SENSOR_ATTR_MAX86178_DM_DIS:
		return max86178_attr_get_dm_dis(dev, val);
	case SENSOR_ATTR_MAX86178_BIOZ_INA_MODE:
		return max86178_attr_get_bioz_ina_mode(dev, val);
	case SENSOR_ATTR_MAX86178_BIOZ_AHPF:
		return max86178_attr_get_bioz_ahpf(dev, val);
	case SENSOR_ATTR_MAX86178_BIOZ_AMP_BW:
		return max86178_attr_get_bioz_amp_bw(dev, val);
	case SENSOR_ATTR_MAX86178_BIOZ_AMP_RGE:
		return max86178_attr_get_bioz_amp_rge(dev, val);
	case SENSOR_ATTR_MAX86178_BIOZ_DAC_RESET:
		return max86178_attr_get_bioz_dac_reset(dev, val);
	case SENSOR_ATTR_MAX86178_BIOZ_DRV_RESET:
		return max86178_attr_get_bioz_drv_reset(dev, val);
	case SENSOR_ATTR_MAX86178_BIOZ_DC_RESTORE:
		return max86178_attr_get_bioz_dc_restore(dev, val);
	case SENSOR_ATTR_MAX86178_BIOZ_EXT_CAP:
		return max86178_attr_get_bioz_ext_cap(dev, val);
	case SENSOR_ATTR_MAX86178_BIOZ_CH_FSEL:
		return max86178_attr_get_bioz_ch_fsel(dev, val);
	case SENSOR_ATTR_MAX86178_BIOZ_INA_CHOP_EN:
		return max86178_attr_get_bioz_ina_chop_en(dev, val);
	case SENSOR_ATTR_MAX86178_BIOZ_FAST:
		return max86178_attr_get_bioz_fast(dev, val);
	case SENSOR_ATTR_MAX86178_BIOZ_IPOL:
		return max86178_attr_get_bioz_ipol(dev, val);
	case SENSOR_ATTR_MAX86178_BIOZ_STBYON:
		return max86178_attr_get_bioz_stbyon(dev, val);
	case SENSOR_ATTR_MAX86178_BIOZ_CMRES_DIS:
		return max86178_attr_get_bioz_cmres_dis(dev, val);
	case SENSOR_ATTR_MAX86178_BIOZ_RLD_DRV:
		return max86178_attr_get_bioz_rld_drv(dev, val);
	case SENSOR_ATTR_MAX86178_BIOZ_RLD_SEL_BIOZ:
		return max86178_attr_get_bioz_rld_sel_bioz(dev, val);
	case SENSOR_ATTR_MAX86178_BIOZ_LO_THRESH:
		return max86178_attr_get_bioz_lo_thresh(dev, val);
	case SENSOR_ATTR_MAX86178_BIOZ_HI_THRESH:
		return max86178_attr_get_bioz_hi_thresh(dev, val);
	/* BioZ Calibration Configuration */
	case SENSOR_ATTR_MAX86178_BIOZ_CAL_EN:
		return max86178_attr_get_bioz_cal_en(dev, val);
	case SENSOR_ATTR_MAX86178_BIOZ_MUX_EN:
		return max86178_attr_get_bioz_mux_en(dev, val);
	case SENSOR_ATTR_MAX86178_BIOZ_CONNECT_CAL_ONLY:
		return max86178_attr_get_bioz_connect_cal_only(dev, val);
	case SENSOR_ATTR_MAX86178_BMUX_BIST_EN:
		return max86178_attr_get_bmux_bist_en(dev, val);
	case SENSOR_ATTR_MAX86178_BMUX_RSEL:
		return max86178_attr_get_bmux_rsel(dev, val);
	case SENSOR_ATTR_MAX86178_EN_INT_INLOAD:
		return max86178_attr_get_en_int_inload(dev, val);
	case SENSOR_ATTR_MAX86178_EN_EXT_INLOAD:
		return max86178_attr_get_en_ext_inload(dev, val);
	case SENSOR_ATTR_MAX86178_GSR_LOAD_EN:
		return max86178_attr_get_gsr_load_en(dev, val);
	case SENSOR_ATTR_MAX86178_BMUX_GSR_RSEL:
		return max86178_attr_get_bmux_gsr_rsel(dev, val);
	case SENSOR_ATTR_MAX86178_DRVN_ASSIGN:
		return max86178_attr_get_drvn_assign(dev, val);
	case SENSOR_ATTR_MAX86178_DRVP_ASSIGN:
		return max86178_attr_get_drvp_assign(dev, val);
	case SENSOR_ATTR_MAX86178_BIN_ASSIGN:
		return max86178_attr_get_bin_assign(dev, val);
	case SENSOR_ATTR_MAX86178_BIP_ASSIGN:
		return max86178_attr_get_bip_assign(dev, val);
	/* BioZ Lead Detection Configuration */
	case SENSOR_ATTR_MAX86178_EN_BIOZ_LON:
		return max86178_attr_get_en_bioz_lon(dev, val);
	case SENSOR_ATTR_MAX86178_EN_BIOZ_LOFF:
		return max86178_attr_get_en_bioz_loff(dev, val);
	case SENSOR_ATTR_MAX86178_EN_EXT_BIOZ_LOFF:
		return max86178_attr_get_en_ext_bioz_loff(dev, val);
	case SENSOR_ATTR_MAX86178_EN_BIOZ_DRV_OOR:
		return max86178_attr_get_en_bioz_drv_oor(dev, val);
	case SENSOR_ATTR_MAX86178_BIOZ_LOFF_IPOL:
		return max86178_attr_get_bioz_loff_ipol(dev, val);
	case SENSOR_ATTR_MAX86178_BIOZ_LOFF_IMAG:
		return max86178_attr_get_bioz_loff_imag(dev, val);
	case SENSOR_ATTR_MAX86178_BIOZ_LOFF_THRESH:
		return max86178_attr_get_bioz_loff_thresh(dev, val);
	case SENSOR_ATTR_MAX86178_RESP_CG_MAG:
		return max86178_attr_get_resp_cg_mag(dev, val);
	case SENSOR_ATTR_MAX86178_RESP_CG_MAG_4X:
		return max86178_attr_get_resp_cg_mag_4x(dev, val);
	/* BioZ Lead Bias Configuration */
	case SENSOR_ATTR_MAX86178_BIOZ_RBIAS_VALUE:
		return max86178_attr_get_bioz_rbias_value(dev, val);
	case SENSOR_ATTR_MAX86178_EN_BIOZ_RBIAS_P:
		return max86178_attr_get_en_bioz_rbias_p(dev, val);
	case SENSOR_ATTR_MAX86178_EN_BIOZ_RBIAS_N:
		return max86178_attr_get_en_bioz_rbias_n(dev, val);
	/* TODO: Implement channel enable attributes with special considerations */
	case SENSOR_ATTR_MAX86178_MEAS1_EN:
	case SENSOR_ATTR_MAX86178_MEAS2_EN:
	case SENSOR_ATTR_MAX86178_MEAS3_EN:
	case SENSOR_ATTR_MAX86178_MEAS4_EN:
	case SENSOR_ATTR_MAX86178_MEAS5_EN:
	case SENSOR_ATTR_MAX86178_MEAS6_EN:
	case SENSOR_ATTR_MAX86178_ECG_EN:
	case SENSOR_ATTR_MAX86178_BIOZ_EN:
	case SENSOR_ATTR_MAX86178_ECG_BIOZ_BG_EN:
	case SENSOR_ATTR_MAX86178_OSC_EN:
		LOG_WRN("Channel enable attribute not yet implemented");
		return -ENOTSUP;
	default:
		LOG_ERR("Attribute not supported");
		return -ENOTSUP;
	}
}

static const struct sensor_driver_api max86178_api = {
	.attr_set = max86178_attr_set,
	.attr_get = max86178_attr_get,
#ifdef CONFIG_MAX86178_TRIGGER
	.trigger_set = max86178_trigger_set,
#endif /* CONFIG_MAX86178_TRIGGER */
#ifdef CONFIG_SENSOR_ASYNC_API
	.get_decoder = max86178_get_decoder,
	.submit = max86178_submit,
#endif /* CONFIG_SENSOR_ASYNC_API */
};

static int max86178_osc_enable(const struct device *dev, bool enable)
{
	int ret = 0;
	uint8_t reg_val = 0;
	ret = max86178_reg_update(dev, MAX86178_PLL_CFG1, MAX86178_PLL_CFG1_PLL_EN_MSK,
				  enable ? 1 : 0);
	if (ret < 0) {
		LOG_ERR("Failed to %s internal oscillator: %d", enable ? "enable" : "disable", ret);
		return ret;
	}

	if (enable) {
		while (FIELD_GET(MAX86178_STATUS3_PHASE_LOCK_MSK, reg_val) == 0) {
			ret = max86178_reg_read(dev, MAX86178_STATUS3, &reg_val, 1);
			if (ret < 0) {
				LOG_ERR("Failed to read PLL lock status: %d", ret);
				return ret;
			}
		}
	}
	return 0;
}

static int max86178_set_pll_cfg6(const struct device *dev, enum ref_clk_sel ref_clk,
				 enum clk_ref_sel clk_freq_sel,
				 enum max86178_clk_fine_tune clk_fine_tune)
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

	if (*ecg_ndiv < 16) {
		LOG_WRN("ECG NDIV value %d is less than minimum of 16. Using 16 instead.",
			*ecg_ndiv);
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
	mdiv |= FIELD_GET(MAX86178_PLL_CFG1_MDIV_MSB_MSK, reg_val) << 8;
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

	if (ndiv < 16) {
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
	uint8_t bioz_kdiv_reg_val;

	ret = max86178_get_pll_clk(dev, &pll_clk);
	if (ret < 0) {
		LOG_ERR("Failed to get PLL clock: %d", ret);
		return ret;
	}

	ret = max86178_reg_read(dev, MAX86178_PLL_CFG3, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read PLL CFG3: %d", ret);
		return ret;
	}
	bioz_kdiv_reg_val = FIELD_GET(MAX86178_PLL_CFG3_BIOZ_KDIV_MSK, reg_val);

	if (bioz_kdiv_reg_val > 13 || bioz_kdiv_reg_val < 16) {
		bioz_kdiv = 8192; /* For register values 13-15, BIOZ KDIV is 8192 */
	} else {
		bioz_kdiv = 1 << bioz_kdiv_reg_val; /* BIOZ KDIV is 2^reg_value */
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
	case 2:
	case 3:
	case 4:
		ecg_f_div = 1 << (ecg_f_div_reg_val -
				  1); /* ECG FDIV is 2^(reg_value-1) for values 1-4 */
		break;
	case 5:
	case 6:
	case 7:
		ecg_f_div = 16; /* For register values 5-7, ECG FDIV is 16 */
		break;
	default:
		LOG_ERR("Unknown ECG FDIV register value: %d", ecg_f_div_reg_val);
		return -EINVAL;
	}
	ret = max86178_reg_read(dev, MAX86178_RESP_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read RESP_CFG1: %d", ret);
		return ret;
	}

	resp_en = FIELD_GET(MAX86178_RESP_CFG1_RESP_EN_MSK, reg_val);
	if (resp_en == 0) {
		*bioz_synth_clk = pll_clk / bioz_kdiv;
	} else {
		*bioz_synth_clk = pll_clk / (ecg_f_div * bioz_kdiv);
	}
	return 0;
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

	ret = max86178_reg_update(dev, MAX86178_PLL_CFG4, MAX86178_PLL_CFG4_ECG_FDIV_MSK, fdiv);
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

	ret = max86178_reg_update(dev, MAX86178_PLL_CFG4, MAX86178_PLL_CFG4_ECG_NDIV_MSB_MSK,
				  reg_val);
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

static int validate_ecg_fdiv_ndiv(const struct device *dev, uint32_t pll_clk, uint8_t fdiv_reg_val,
				  uint16_t ndiv_reg_val)
{
	uint32_t ecg_adc_clk;

	uint8_t fdiv;
	switch (fdiv_reg_val) {
	case 0:
		fdiv = 0;
		LOG_WRN("ECG FDIV is set to 0, which will disable the ECG ADC clock. ECG "
			"measurements will not be possible.");
		return 0;
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

	if (ndiv_reg_val < 16) {
		LOG_WRN("ECG NDIV value %d is less than minimum of 16. Using 16 instead.",
			ndiv_reg_val);
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
	ret = max86178_reg_update(dev, MAX86178_ECG_CFG1, MAX86178_ECG_CFG1_ECG_DEC_RATE_MSK,
				  dec_rate);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG Decimation Rate: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_set_ecg_clk_cfg(const struct device *dev, uint8_t fdiv_reg_val,
				    uint16_t ndiv_reg_val, enum max86178_ecg_dec_rate dec_rate)
{
	int ret;
	uint32_t pll_clk;

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

static bool validate_bioz_ndiv(const struct device *dev, uint32_t pll_clk, uint8_t resp_en,
			       uint8_t ecg_fdiv_reg_val, uint8_t bioz_ndiv_reg_val)
{
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
		LOG_INF("Calculated BIOZ ADC Clock: %d Hz (PLL Clock: %d Hz, ECG FDIV: %d, BIOZ "
			"NDIV: %d)",
			bioz_adc_clk, pll_clk, ecg_fdiv_reg_val, bioz_ndiv);
		return false;
	}
	return true;
}

static int max86178_set_resp_en(const struct device *dev, bool enable)
{
	return max86178_reg_update(dev, MAX86178_RESP_CFG1, MAX86178_RESP_CFG1_RESP_EN_MSK,
				   enable ? 1 : 0);
}

static int max86178_set_bioz_ndiv(const struct device *dev, uint8_t bioz_ndiv_reg_val)
{
	return max86178_reg_update(dev, MAX86178_PLL_CFG3, MAX86178_PLL_CFG3_BIOZ_NDIV_MSK,
				   bioz_ndiv_reg_val);
}

static int max86178_set_bioz_adc_osr(const struct device *dev, uint8_t bioz_adc_osr_reg_val)
{
	return max86178_reg_update(dev, MAX86178_BIOZ_CFG1, MAX86178_BIOZ_CFG1_BIOZ_ADC_OSR_MSK,
				   bioz_adc_osr_reg_val);
}

static int max86178_set_bioz_adc_clk_cfg(const struct device *dev, uint8_t ecg_fdiv_reg_val,
					 uint8_t bioz_ndiv_reg_val, bool resp_en,
					 uint8_t bioz_adc_osr_reg_val)
{
	int ret = 0;
	uint32_t pll_clk;

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

static bool validate_bioz_kdiv(const struct device *dev, uint32_t pll_clk, bool resp_en,
			       uint8_t ecg_fdiv_reg_val, uint8_t bioz_kdiv_reg_val)
{
	uint32_t bioz_synth_clk;
	uint16_t bioz_kdiv = 0;
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

	if (resp_en == 0) {
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

	if (bioz_synth_clk < MAX86178_BIOZ_SYNTH_CLK_MIN ||
	    bioz_synth_clk > MAX86178_BIOZ_SYNTH_CLK_MAX) {
		LOG_ERR("Invalid BIOZ Synth clock frequency: %d Hz. Must be between %d and %d Hz",
			bioz_synth_clk, MAX86178_BIOZ_SYNTH_CLK_MIN, MAX86178_BIOZ_SYNTH_CLK_MAX);
		return false;
	}
	return true;
}

static int max86178_set_bioz_kdiv(const struct device *dev, uint8_t bioz_kdiv_reg_val)
{
	return max86178_reg_update(dev, MAX86178_PLL_CFG3, MAX86178_PLL_CFG3_BIOZ_KDIV_MSK,
				   bioz_kdiv_reg_val);
}

static int max86178_set_bioz_dac_osr(const struct device *dev, uint8_t bioz_dac_osr_reg_val)
{
	return max86178_reg_update(dev, MAX86178_BIOZ_CFG1, MAX86178_BIOZ_CFG1_BIOZ_DAC_OSR_MSK,
				   bioz_dac_osr_reg_val);
}

static int max86178_bioz_synth_clk_cfg(const struct device *dev, bool resp_en,
				       uint8_t ecg_fdiv_reg_val, uint8_t bioz_kdiv_reg_val,
				       uint8_t bioz_dac_osr_reg_val)
{
	int ret = 0;
	uint32_t pll_clk;

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
	/* Set PLL Configuration 6 - Reference Clock, Clock Frequency Selection, and Clock Fine Tune
	 */
	ret = max86178_set_pll_cfg6(dev, config->clk_cfg.osc_cfg.ref_clk,
				    config->clk_cfg.osc_cfg.clk_freq_sel,
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
	ret = max86178_set_ecg_clk_cfg(dev, config->clk_cfg.ecg_cfg.ecg_fdiv,
				       config->clk_cfg.ecg_cfg.ecg_ndiv,
				       config->clk_cfg.ecg_cfg.ecg_dec_rate);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG clock configuration: %d", ret);
		return ret;
	}

	/* Set BIOZ ADC Clock Configuration */
	ret = max86178_set_bioz_adc_clk_cfg(
		dev, config->clk_cfg.ecg_cfg.ecg_fdiv, config->clk_cfg.bioz_cfg.bioz_ndiv,
		config->resp_cfg.resp_en, config->clk_cfg.bioz_cfg.bioz_adc_osr);
	if (ret < 0) {
		LOG_ERR("Failed to set BIOZ ADC clock configuration: %d", ret);
		return ret;
	}

	/* Set BIOZ Synth Clock Configuration */
	ret = max86178_bioz_synth_clk_cfg(
		dev, config->resp_cfg.resp_en, config->clk_cfg.ecg_cfg.ecg_fdiv,
		config->clk_cfg.bioz_cfg.bioz_kdiv, config->clk_cfg.bioz_cfg.bioz_dac_osr);
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
		LOG_ERR("Invalid FIFO watermark level: %d. Must be between 1 and 256.",
			config->fifo_cfg.fifo_watermark);
		return -EINVAL;
	}

	/* Flush Fifo */
	ret = max86178_reg_update(dev, MAX86178_FIFO_CFG2, MAX86178_FIFO_CFG2_FLUSH_FIFO_MSK, 1);
	if (ret < 0) {
		LOG_ERR("Failed to flush FIFO: %d", ret);
		return ret;
	}

	/* Set FIFO Watermark */
	reg_val = 256 - config->fifo_cfg.fifo_watermark; /* FIFO counts down, so watermark is set as
							    (256 - desired level) */
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
#ifdef CONFIG_MAX86178_TRIGGER
	/* Set Interrupt Enable */
	ret = max86178_reg_update(dev,
				  config->route_to_int2 ? MAX86178_INT2_EN1 : MAX86178_INT1_EN1,
				  MAX86178_INT_EN1_A_FULL_MSK, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set FIFO interrupt enable: %d", ret);
		return ret;
	}
#endif /* CONFIG_MAX86178_TRIGGER */
	return 0;
}

static int max86178_fr_clk_div_set(const struct device *dev, uint16_t fr_clk_div)
{
	int ret = 0;
	uint8_t reg_val;

	reg_val = FIELD_GET(MAX86178_FR_CLK_DIV_MSB_MSK, fr_clk_div);

	ret = max86178_reg_update(dev, MAX86178_FR_CLK_DIV_MSB, MAX86178_FR_CLK_DIV_MSB_MSK,
				  reg_val);
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

static int max86178_set_ppg_meas_en(const struct device *dev)
{
	int ret = 0;
	uint8_t reg_val;
	const struct max86178_dev_config *config = dev->config;

	reg_val = FIELD_PREP(MAX86178_PPG_CFG1_MEAS1_EN_MSK, config->ppg_cfg.meas_en[0] ? 1 : 0) |
		  FIELD_PREP(MAX86178_PPG_CFG1_MEAS2_EN_MSK, config->ppg_cfg.meas_en[1] ? 1 : 0) |
		  FIELD_PREP(MAX86178_PPG_CFG1_MEAS3_EN_MSK, config->ppg_cfg.meas_en[2] ? 1 : 0) |
		  FIELD_PREP(MAX86178_PPG_CFG1_MEAS4_EN_MSK, config->ppg_cfg.meas_en[3] ? 1 : 0) |
		  FIELD_PREP(MAX86178_PPG_CFG1_MEAS5_EN_MSK, config->ppg_cfg.meas_en[4] ? 1 : 0) |
		  FIELD_PREP(MAX86178_PPG_CFG1_MEAS6_EN_MSK, config->ppg_cfg.meas_en[5] ? 1 : 0);
	ret = max86178_reg_write(dev, MAX86178_PPG_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set PPG measurement enable: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_set_ppg_pwrdn_sync_mode(const struct device *dev)
{
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;
	uint8_t reg_val;

	reg_val = FIELD_PREP(MAX86178_PPG_CFG2_PPG1_PWRDN_MSK, config->ppg_cfg.ppg1_pwrdn) |
		  FIELD_PREP(MAX86178_PPG_CFG2_PPG2_PWRDN_MSK, config->ppg_cfg.ppg2_pwrdn) |
		  FIELD_PREP(MAX86178_PPG_CFG2_PPG_SYNC_MODE_MSK, config->ppg_cfg.ppg_sync_mode);

	ret = max86178_reg_write(dev, MAX86178_PPG_CFG2, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set PPG power down and sync mode: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_set_ppg_cfg3(const struct device *dev)
{
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;
	uint8_t reg_val;

	reg_val = FIELD_PREP(MAX86178_PPG_CFG3_MEAS1_CONFIG_SEL_MSK,
			     config->ppg_cfg.meas1_config_sel) |
		  FIELD_PREP(MAX86178_PPG_CFG3_COLLECT_RAW_DATA_MSK,
			     config->ppg_cfg.collect_raw_data) |
		  FIELD_PREP(MAX86178_PPG_CFG3_ALC_DISABLE_MSK, config->ppg_cfg.alc_disable) |
		  FIELD_PREP(MAX86178_PPG_CFG3_SMP_AVE_MSK, config->ppg_cfg.smp_ave);

	ret = max86178_reg_write(dev, MAX86178_PPG_CFG3, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set PPG CFG3: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_set_ppg_cfg4(const struct device *dev)
{
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;
	uint8_t reg_val;

	reg_val = FIELD_PREP(MAX86178_PPG_CFG4_PROX_AUTO_MSK, config->ppg_cfg.prox_auto_en) |
		  FIELD_PREP(MAX86178_PPG_CFG4_PROX_DATA_EN_MSK, config->ppg_cfg.prox_data_en);
	ret = max86178_reg_write(dev, MAX86178_PPG_CFG4, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set PPG CFG4: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_set_pdbias(const struct device *dev)
{
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;
	uint8_t reg_val;

	reg_val = FIELD_PREP(MAX86178_PD_BIAS_PD1_MSK, config->ppg_cfg.pd1_bias) |
		  FIELD_PREP(MAX86178_PD_BIAS_PD2_MSK, config->ppg_cfg.pd2_bias) |
		  FIELD_PREP(MAX86178_PD_BIAS_PD3_MSK, config->ppg_cfg.pd3_bias) |
		  FIELD_PREP(MAX86178_PD_BIAS_PD4_MSK, config->ppg_cfg.pd4_bias);
	ret = max86178_reg_write(dev, MAX86178_PD_BIAS, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set PD Bias: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_set_sequencer_meas_sel(const struct device *dev, uint8_t seq_num)
{
	int ret = 0;
	uint8_t reg_val;
	const struct max86178_dev_config *config = dev->config;
	uint8_t seq_reg_addr = MAX86178_MEAS1_SEL + ((seq_num - 1) * 8);

	reg_val =
		FIELD_PREP(MAX86178_MEAS_SEL_DRVA_MSK, config->ppg_cfg.meas_cfg[seq_num - 1].drva) |
		FIELD_PREP(MAX86178_MEAS_SEL_DRVB_MSK, config->ppg_cfg.meas_cfg[seq_num - 1].drvb) |
		FIELD_PREP(MAX86178_MEAS_SEL_AMB_MSK,
			   config->ppg_cfg.meas_cfg[seq_num - 1].amb_mode);
	ret = max86178_reg_write(dev, seq_reg_addr, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set Sequencer %d measurement selection: %d", seq_num, ret);
		return ret;
	}
	return 0;
}

static int max86178_set_sequencer_meas_cfg1(const struct device *dev, uint8_t seq_num)
{
	int ret = 0;
	uint8_t reg_val;
	const struct max86178_dev_config *config = dev->config;
	uint8_t seq_reg_addr = MAX86178_MEAS1_CFG1 + ((seq_num - 1) * 8);

	reg_val = FIELD_PREP(MAX86178_MEAS_CFG1_AVER_MSK,
			     config->ppg_cfg.meas_cfg[seq_num - 1].avg_num) |
		  FIELD_PREP(MAX86178_MEAS_CFG1_TINT_MSK,
			     config->ppg_cfg.meas_cfg[seq_num - 1].tint) |
		  FIELD_PREP(MAX86178_MEAS_CFG1_FILT_SEL_MSK,
			     config->ppg_cfg.meas_cfg[seq_num - 1].filt_sel) |
		  FIELD_PREP(MAX86178_MEAS_CFG1_FILT2_SEL_MSK,
			     config->ppg_cfg.meas_cfg[seq_num - 1].filt2_sel) |
		  FIELD_PREP(MAX86178_MEAS_CFG1_SINC3_SEL_MSK,
			     config->ppg_cfg.meas_cfg[seq_num - 1].sinc3_sel);
	ret = max86178_reg_write(dev, seq_reg_addr, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set Sequencer %d measurement configuration: %d", seq_num, ret);
		return ret;
	}
	return 0;
}

static int max86178_set_sequencer_meas_cfg2(const struct device *dev, uint8_t seq_num)
{
	int ret = 0;
	uint8_t reg_val;
	const struct max86178_dev_config *config = dev->config;
	uint8_t seq_reg_addr = MAX86178_MEAS1_CFG2 + ((seq_num - 1) * 8);

	reg_val = FIELD_PREP(MAX86178_MEAS_CFG2_PPG1_ADC_RGE_MSK,
			     config->ppg_cfg.meas_cfg[seq_num - 1].ppg1_adc_rge) |
		  FIELD_PREP(MAX86178_MEAS_CFG2_PPG2_ADC_RGE_MSK,
			     config->ppg_cfg.meas_cfg[seq_num - 1].ppg2_adc_rge);
	ret = max86178_reg_write(dev, seq_reg_addr, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set Sequencer %d measurement configuration: %d", seq_num, ret);
		return ret;
	}
	return 0;
}

static int max86178_set_sequencer_meas_cfg3(const struct device *dev, uint8_t seq_num)
{
	int ret = 0;
	uint8_t reg_val;
	const struct max86178_dev_config *config = dev->config;
	uint8_t seq_reg_addr = MAX86178_MEAS1_CFG3 + ((seq_num - 1) * 8);

	reg_val = FIELD_PREP(MAX86178_MEAS_CFG3_PPG1_DACOFF_MSK,
			     config->ppg_cfg.meas_cfg[seq_num - 1].ppg1_dac_off) |
		  FIELD_PREP(MAX86178_MEAS_CFG3_PPG2_DACOFF_MSK,
			     config->ppg_cfg.meas_cfg[seq_num - 1].ppg2_dac_off);
	ret = max86178_reg_write(dev, seq_reg_addr, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set Sequencer %d measurement configuration: %d", seq_num, ret);
		return ret;
	}
	return 0;
}

static int max86178_set_sequencer_meas_cfg4(const struct device *dev, uint8_t seq_num)
{
	int ret = 0;
	uint8_t reg_val;
	const struct max86178_dev_config *config = dev->config;
	uint8_t seq_reg_addr = MAX86178_MEAS1_CFG4 + ((seq_num - 1) * 8);

	reg_val = FIELD_PREP(MAX86178_MEAS_CFG4_LED_RGE_MSK,
			     config->ppg_cfg.meas_cfg[seq_num - 1].led_rge) |
		  FIELD_PREP(MAX86178_MEAS_CFG4_LED_SETLNG_MSK,
			     config->ppg_cfg.meas_cfg[seq_num - 1].led_setlng) |
		  FIELD_PREP(MAX86178_MEAS_CFG4_PD_SETLNG_MSK,
			     config->ppg_cfg.meas_cfg[seq_num - 1].pd_setlng);
	ret = max86178_reg_write(dev, seq_reg_addr, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set Sequencer %d measurement configuration: %d", seq_num, ret);
		return ret;
	}
	return 0;
}

static int max86178_set_sequencer_meas_cfg5(const struct device *dev, uint8_t seq_num)
{
	int ret = 0;
	uint8_t reg_val;
	const struct max86178_dev_config *config = dev->config;
	uint8_t seq_reg_addr = MAX86178_MEAS1_CFG5 + ((seq_num - 1) * 8);

	reg_val = FIELD_PREP(MAX86178_MEAS_CFG5_PD1_SEL_MSK,
			     config->ppg_cfg.meas_cfg[seq_num - 1].pd1_sel) |
		  FIELD_PREP(MAX86178_MEAS_CFG5_PD2_SEL_MSK,
			     config->ppg_cfg.meas_cfg[seq_num - 1].pd2_sel) |
		  FIELD_PREP(MAX86178_MEAS_CFG5_PD3_SEL_MSK,
			     config->ppg_cfg.meas_cfg[seq_num - 1].pd3_sel) |
		  FIELD_PREP(MAX86178_MEAS_CFG5_PD4_SEL_MSK,
			     config->ppg_cfg.meas_cfg[seq_num - 1].pd4_sel);
	ret = max86178_reg_write(dev, seq_reg_addr, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set Sequencer %d measurement configuration: %d", seq_num, ret);
		return ret;
	}
	return 0;
}

static int max86178_set_sequencer_leda_current(const struct device *dev, uint8_t seq_num)
{
	int ret = 0;
	uint8_t reg_val;
	const struct max86178_dev_config *config = dev->config;
	uint8_t seq_reg_addr = MAX86178_MEAS1_LEDA_PA + ((seq_num - 1) * 8);

	reg_val = config->ppg_cfg.meas_cfg[seq_num - 1].drva_pa;
	ret = max86178_reg_write(dev, seq_reg_addr, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set Sequencer %d measurement configuration: %d", seq_num, ret);
		return ret;
	}
	return 0;
}

static int max86178_set_sequencer_ledb_current(const struct device *dev, uint8_t seq_num)
{
	int ret = 0;
	uint8_t reg_val;
	const struct max86178_dev_config *config = dev->config;
	uint8_t seq_reg_addr = MAX86178_MEAS1_LEDB_PA + ((seq_num - 1) * 8);

	reg_val = config->ppg_cfg.meas_cfg[seq_num - 1].drvb_pa;
	ret = max86178_reg_write(dev, seq_reg_addr, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set Sequencer %d measurement configuration: %d", seq_num, ret);
		return ret;
	}
	return 0;
}

static int max86178_set_sequencer_cfg(const struct device *dev, uint8_t seq_num)
{
	int ret = 0;

	if (seq_num < 1 || seq_num > 6) {
		LOG_ERR("Invalid sequencer number: %d. Must be between 1 and 6.", seq_num);
		return -EINVAL;
	}

	/* Set measurement selection for the sequencer */
	ret = max86178_set_sequencer_meas_sel(dev, seq_num);
	if (ret < 0) {
		LOG_ERR("Failed to set sequencer %d measurement selection: %d", seq_num, ret);
		return ret;
	}
	/* Set measurement configurations for the sequencer */
	ret = max86178_set_sequencer_meas_cfg1(dev, seq_num);
	if (ret < 0) {
		LOG_ERR("Failed to set sequencer %d measurement configuration 1: %d", seq_num, ret);
		return ret;
	}

	ret = max86178_set_sequencer_meas_cfg2(dev, seq_num);
	if (ret < 0) {
		LOG_ERR("Failed to set sequencer %d measurement configuration 2: %d", seq_num, ret);
		return ret;
	}

	ret = max86178_set_sequencer_meas_cfg3(dev, seq_num);
	if (ret < 0) {
		LOG_ERR("Failed to set sequencer %d measurement configuration 3: %d", seq_num, ret);
		return ret;
	}

	ret = max86178_set_sequencer_meas_cfg4(dev, seq_num);
	if (ret < 0) {
		LOG_ERR("Failed to set sequencer %d measurement configuration 4: %d", seq_num, ret);
		return ret;
	}

	ret = max86178_set_sequencer_meas_cfg5(dev, seq_num);
	if (ret < 0) {
		LOG_ERR("Failed to set sequencer %d measurement configuration 5: %d", seq_num, ret);
		return ret;
	}

	ret = max86178_set_sequencer_leda_current(dev, seq_num);
	if (ret < 0) {
		LOG_ERR("Failed to set sequencer %d LEDA current: %d", seq_num, ret);
		return ret;
	}

	ret = max86178_set_sequencer_ledb_current(dev, seq_num);
	if (ret < 0) {
		LOG_ERR("Failed to set sequencer %d LEDB current: %d", seq_num, ret);
		return ret;
	}
	return 0;
}

static int max86178_set_ppg_threshold_meas_sel(const struct device *dev)
{
	int ret = 0;
	uint8_t reg_val;
	const struct max86178_dev_config *config = dev->config;

	reg_val = FIELD_PREP(MAX86178_THRESH_MEAS_SEL_THRESH1_MEAS_SEL_MSK,
			     config->ppg_cfg.threshold_cfg.thresh1_meas_sel) |
		  FIELD_PREP(MAX86178_THRESH_MEAS_SEL_THRESH2_MEAS_SEL_MSK,
			     config->ppg_cfg.threshold_cfg.thresh2_meas_sel);

	ret = max86178_reg_write(dev, MAX86178_THRESH_MEAS_SEL, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set PPG threshold measurement selection: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_set_ppg_threshold_hyst(const struct device *dev)
{
	int ret = 0;
	uint8_t reg_val;
	const struct max86178_dev_config *config = dev->config;

	reg_val = FIELD_PREP(MAX86178_THRESH_HYST_LEVEL_HYST_MSK,
			     config->ppg_cfg.threshold_cfg.level_hyst) |
		  FIELD_PREP(MAX86178_THRESH_HYST_TIME_HYST_MSK,
			     config->ppg_cfg.threshold_cfg.time_hyst) |
		  FIELD_PREP(MAX86178_THRESH_HYST_THRESH1_PPG_SEL_MSK,
			     config->ppg_cfg.threshold_cfg.thresh1_chan_sel) |
		  FIELD_PREP(MAX86178_THRESH_HYST_THRESH2_PPG_SEL_MSK,
			     config->ppg_cfg.threshold_cfg.thresh2_chan_sel);

	ret = max86178_reg_write(dev, MAX86178_THRESH_HYST, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set PPG threshold hysteresis: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_set_ppg_threshold_level(const struct device *dev)
{
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;
	uint8_t reg_val = 0;

	reg_val = config->ppg_cfg.threshold_cfg.thresh1_hi;
	ret = max86178_reg_write(dev, MAX86178_PPG_HI_THRESH1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set PPG HI threshold 1 level: %d", ret);
		return ret;
	}

	reg_val = config->ppg_cfg.threshold_cfg.thresh1_lo;
	ret = max86178_reg_write(dev, MAX86178_PPG_LO_THRESH1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set PPG LOW threshold 1 level: %d", ret);
		return ret;
	}

	reg_val = config->ppg_cfg.threshold_cfg.thresh2_hi;
	ret = max86178_reg_write(dev, MAX86178_PPG_HI_THRESH2, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set PPG HI threshold 2 level: %d", ret);
		return ret;
	}

	reg_val = config->ppg_cfg.threshold_cfg.thresh2_lo;
	ret = max86178_reg_write(dev, MAX86178_PPG_LO_THRESH2, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set PPG LOW threshold 2 level: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_set_ppg_threshold(const struct device *dev)
{
	int ret = 0;

	/* Set threshold measurement selection */
	ret = max86178_set_ppg_threshold_meas_sel(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set PPG threshold measurement selection: %d", ret);
		return ret;
	}

	ret = max86178_set_ppg_threshold_hyst(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set PPG threshold hysteresis: %d", ret);
		return ret;
	}

	ret = max86178_set_ppg_threshold_level(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set PPG threshold levels: %d", ret);
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
	/* Set PPG PWRDN and sync mode */
	ret = max86178_set_ppg_pwrdn_sync_mode(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set PPG power down and sync mode: %d", ret);
		return ret;
	}

	/* Set PPG CFG3 settings */
	ret = max86178_set_ppg_cfg3(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set PPG CFG3: %d", ret);
		return ret;
	}

	/* Set PPG CFG4 settings */
	ret = max86178_set_ppg_cfg4(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set PPG CFG4: %d", ret);
		return ret;
	}

	/* Set PD Bias */
	ret = max86178_set_pdbias(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set PD Bias: %d", ret);
		return ret;
	}

	/* Set Sequencer Configs */
	for (uint8_t seq_num = 1; seq_num <= 6; seq_num++) {
		ret = max86178_set_sequencer_cfg(dev, seq_num);
		if (ret < 0) {
			LOG_ERR("Failed to set sequencer %d configuration: %d", seq_num, ret);
			return ret;
		}
	}

	ret = max86178_set_ppg_threshold(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set PPG thresholds: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_set_ecg_cfg2(const struct device *dev)
{
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;
	uint8_t reg_val;

	reg_val =
		FIELD_PREP(MAX86178_ECG_CFG2_ECG_INA_GAIN_MSK, config->ecg_cfg.setup.ecg_ina_gain) |
		FIELD_PREP(MAX86178_ECG_CFG2_ECG_INA_RGE_MSK, config->ecg_cfg.setup.ecg_ina_rge) |
		FIELD_PREP(MAX86178_ECG_CFG2_ECG_PGA_GAIN_MSK, config->ecg_cfg.setup.ecg_pga_gain) |
		FIELD_PREP(MAX86178_ECG_CFG2_ECG_IPOL_MASK, config->ecg_cfg.setup.ecg_input_pol);
	ret = max86178_reg_write(dev, MAX86178_ECG_CFG2, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG INA, PGA, and input polarity configuration: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_set_ecg_cfg3(const struct device *dev)
{
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;
	uint8_t reg_val;

	reg_val =
		FIELD_PREP(MAX86178_ECG_CFG3_ECG_MUX_SEL_MSK, config->ecg_cfg.setup.ecg_mux_sel) |
		FIELD_PREP(MAX86178_ECG_CFG3_ECG_AUTO_REC_MSK, config->ecg_cfg.setup.ecg_auto_rec) |
		FIELD_PREP(MAX86178_ECG_CFG3_ECG_IMP_HI_MSK, config->ecg_cfg.setup.ecg_imp_hi);
	ret = max86178_reg_write(dev, MAX86178_ECG_CFG3, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG MUX, auto recovery, and impedance configuration: %d",
			ret);
		return ret;
	}
	return 0;
}

static int max86178_set_ecg_cfg4(const struct device *dev)
{
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;
	uint8_t reg_val;

	reg_val = FIELD_PREP(MAX86178_ECG_CFG4_ECG_FAST_REC_THRESHOLD_MSK,
			     config->ecg_cfg.setup.ecg_fast_rec_thres) |
		  FIELD_PREP(MAX86178_ECG_CFG4_EN_ECG_FAST_REC_MSK,
			     config->ecg_cfg.setup.en_ecg_fast_rec);
	ret = max86178_reg_write(dev, MAX86178_ECG_CFG4, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG fast recovery configuration: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_set_ecg_setup(const struct device *dev)
{
	int ret = 0;

	/* Set ECG CFG2 settings */
	ret = max86178_set_ecg_cfg2(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG CFG2: %d", ret);
		return ret;
	}

	/* Set ECG CFG3 settings */
	ret = max86178_set_ecg_cfg3(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG CFG3: %d", ret);
		return ret;
	}

	/* Set ECG CFG4 settings */
	ret = max86178_set_ecg_cfg4(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG CFG4: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_set_ecg_cal_cfg1(const struct device *dev)
{
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;
	uint8_t reg_val;

	reg_val = FIELD_PREP(MAX86178_ECG_CAL_CFG1_ECG_CAL_EN_MSK,
			     config->ecg_cfg.calibration.ecg_cal_en) |
		  FIELD_PREP(MAX86178_ECG_CAL_CFG1_ECG_CAL_DUTY_MSK,
			     config->ecg_cfg.calibration.ecg_cal_duty) |
		  FIELD_PREP(MAX86178_ECG_CAL_CFG1_ECG_CAL_FREQ_MSK,
			     config->ecg_cfg.calibration.ecg_freq) |
		  FIELD_PREP(MAX86178_ECG_CAL_CFG1_ECG_CAL_HIGH_MSB_MSK,
			     (config->ecg_cfg.calibration.ecg_cal_high >> 8) & 0x07);
	ret = max86178_reg_write(dev, MAX86178_ECG_CAL_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG calibration configuration: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_set_ecg_cal_cfg2(const struct device *dev)
{
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;
	uint8_t reg_val;

	reg_val = config->ecg_cfg.calibration.ecg_cal_high & 0xFF;
	ret = max86178_reg_write(dev, MAX86178_ECG_CAL_CFG2, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG calibration high byte: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_set_ecg_cal_cfg3(const struct device *dev)
{
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;
	uint8_t reg_val;

	reg_val = FIELD_PREP(MAX86178_ECG_CAL_CFG3_ECG_CAL_N_SEL_MSK,
			     config->ecg_cfg.calibration.ecg_cal_n_sel) |
		  FIELD_PREP(MAX86178_ECG_CAL_CFG3_ECG_CAL_P_SEL_MSK,
			     config->ecg_cfg.calibration.ecg_cal_p_sel) |
		  FIELD_PREP(MAX86178_ECG_CAL_CFG3_ECG_CAL_MAG_MASK,
			     config->ecg_cfg.calibration.ecg_cal_mag) |
		  FIELD_PREP(MAX86178_ECG_CAL_CFG3_ECG_CAL_MODE_MSK,
			     config->ecg_cfg.calibration.ecg_cal_mode) |
		  FIELD_PREP(MAX86178_ECG_CAL_CFG3_ECG_OPEN_P_MSK,
			     config->ecg_cfg.calibration.ecg_open_p) |
		  FIELD_PREP(MAX86178_ECG_CAL_CFG3_ECG_OPEN_N_MSK,
			     config->ecg_cfg.calibration.ecg_open_n);
	ret = max86178_reg_write(dev, MAX86178_ECG_CAL_CFG3, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG calibration configuration: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_set_ecg_calibration(const struct device *dev)
{
	int ret = 0;

	/* Set ECG calibration CFG1 settings */
	ret = max86178_set_ecg_cal_cfg1(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG calibration CFG1: %d", ret);
		return ret;
	}

	/* Set ECG calibration CFG2 settings */
	ret = max86178_set_ecg_cal_cfg2(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG calibration CFG2: %d", ret);
		return ret;
	}

	/* Set ECG calibration CFG3 settings */
	ret = max86178_set_ecg_cal_cfg3(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG calibration CFG3: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_set_ecg_lead_detect_cfg1(const struct device *dev)
{
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;
	uint8_t reg_val;

	reg_val = FIELD_PREP(MAX86178_ECG_LEAD_CFG1_ECG_LOFF_FREQ_MSK,
			     config->ecg_cfg.lead_detect.ecg_loff_freq) |
		  FIELD_PREP(MAX86178_ECG_LEAD_CFG1_ECG_LOFF_MODE_MSK,
			     config->ecg_cfg.lead_detect.ecg_loff_mode) |
		  FIELD_PREP(MAX86178_ECG_LEAD_CFG1_EN_ECG_LOFF_MSK,
			     config->ecg_cfg.lead_detect.en_ecg_loff) |
		  FIELD_PREP(MAX86178_ECG_LEAD_CFG1_EN_ECG_LON_MSK,
			     config->ecg_cfg.lead_detect.en_ecg_lon);
	ret = max86178_reg_write(dev, MAX86178_ECG_LD_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG lead detection configuration: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_set_ecg_lead_detect_cfg2(const struct device *dev)
{
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;
	uint8_t reg_val;

	reg_val = FIELD_PREP(MAX86178_ECG_LEAD_CFG2_ECG_LOFF_THRESH_MSK,
			     config->ecg_cfg.lead_detect.ecg_loff_thresh) |
		  FIELD_PREP(MAX86178_ECG_LEAD_CFG2_ECG_LOFF_IMAG_MSK,
			     config->ecg_cfg.lead_detect.ecg_loff_current_mag) |
		  FIELD_PREP(MAX86178_ECG_LEAD_CFG2_ECG_LOFF_IPOL_MSK,
			     config->ecg_cfg.lead_detect.ecg_loff_ipol);
	ret = max86178_reg_write(dev, MAX86178_ECG_LD_CFG2, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG lead detection configuration: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_set_ecg_lead_detection(const struct device *dev)
{
	int ret = 0;

	/* Set ECG lead detection CFG1 settings */
	ret = max86178_set_ecg_lead_detect_cfg1(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG lead detection CFG1: %d", ret);
		return ret;
	}

	/* Set ECG lead detection CFG2 settings */
	ret = max86178_set_ecg_lead_detect_cfg2(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG lead detection CFG2: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_set_ecg_lead_bias(const struct device *dev)
{
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;
	uint8_t reg_val;

	reg_val = FIELD_PREP(MAX86178_ECG_LEAD_BIAS_EN_ECG_RBIAS_N_MSK,
			     config->ecg_cfg.lead_bias.en_ecg_rbias_n) |
		  FIELD_PREP(MAX86178_ECG_LEAD_BIAS_EN_ECG_RBIAS_P_MSK,
			     config->ecg_cfg.lead_bias.en_ecg_rbias_p) |
		  FIELD_PREP(MAX86178_ECG_LEAD_BIAS_ECG_RBIAS_VAL_MSK,
			     config->ecg_cfg.lead_bias.ecg_rbias_value);
	ret = max86178_reg_write(dev, MAX86178_ECG_LB_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG lead bias configuration: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_set_ecg_rld_cfg1(const struct device *dev)
{
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;
	uint8_t reg_val;

	reg_val = FIELD_PREP(MAX86178_RLD_CFG1_RLD_GAIN_MSK, config->ecg_cfg.rld_cfg.rld_gain) |
		  FIELD_PREP(MAX86178_RLD_CFG1_ACTV_CM_N_MSK, config->ecg_cfg.rld_cfg.actv_cm_n) |
		  FIELD_PREP(MAX86178_RLD_CFG1_ACTV_CM_P_MSK, config->ecg_cfg.rld_cfg.actv_cm_p) |
		  FIELD_PREP(MAX86178_RLD_CFG1_EN_RLD_OOR_MSK, config->ecg_cfg.rld_cfg.en_rld_oor) |
		  FIELD_PREP(MAX86178_RLD_CFG1_RLD_BIAS_MSK, config->ecg_cfg.rld_cfg.rld_rbias) |
		  FIELD_PREP(MAX86178_RLD_CFG1_RLD_MODE_MSK, config->ecg_cfg.rld_cfg.rld_mode) |
		  FIELD_PREP(MAX86178_RLD_CFG1_RLD_EN_MSK, config->ecg_cfg.rld_cfg.rld_en);
	ret = max86178_reg_write(dev, MAX86178_RLD_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG RLD configuration 1: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_set_ecg_rld_cfg2(const struct device *dev)
{
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;
	uint8_t reg_val;

	reg_val =
		FIELD_PREP(MAX86178_RLD_CFG2_BODY_BIAS_DAC_MSK,
			   config->ecg_cfg.rld_cfg.body_bias_dac) |
		FIELD_PREP(MAX86178_RLD_CFG2_RLD_BW_MSK, config->ecg_cfg.rld_cfg.rld_bw) |
		FIELD_PREP(MAX86178_RLD_CFG2_RLD_SEL_ECG_MSK, config->ecg_cfg.rld_cfg.rld_sel_ecg) |
		FIELD_PREP(MAX86178_RLD_CFG2_RLD_EXT_RES_MSK, config->ecg_cfg.rld_cfg.rld_ext_res);
	ret = max86178_reg_write(dev, MAX86178_RLD_CFG2, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG RLD configuration 2: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_set_ecg_rld_cfg(const struct device *dev)
{
	int ret = 0;

	/* Set ECG RLD configuration 1 */
	ret = max86178_set_ecg_rld_cfg1(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG RLD configuration 1: %d", ret);
		return ret;
	}

	/* Set ECG RLD configuration 2 */
	ret = max86178_set_ecg_rld_cfg2(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG RLD configuration 2: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_ecg_init(const struct device *dev)
{
	int ret = 0;

	/* Set ECG Configs */
	ret = max86178_set_ecg_setup(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG configuration: %d", ret);
		return ret;
	}

	/* Set ECG calibration settings */
	ret = max86178_set_ecg_calibration(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG calibration: %d", ret);
		return ret;
	}

	/* Set ECG lead detection settings */
	ret = max86178_set_ecg_lead_detection(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG lead detection configuration: %d", ret);
		return ret;
	}

	/* Set ECG lead bias settings */
	ret = max86178_set_ecg_lead_bias(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG lead bias configuration: %d", ret);
		return ret;
	}

	/* Set ECG RLD settings */
	ret = max86178_set_ecg_rld_cfg(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set ECG RLD configuration: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_set_bioz_cfg2(const struct device *dev)
{
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;
	uint8_t reg_val;

	reg_val = FIELD_PREP(MAX86178_BIOZ_CFG2_EN_BIOZ_THRESH_MSK,
			     config->bioz_cfg.setup.en_bioz_thresh) |
		  FIELD_PREP(MAX86178_BIOZ_CFG2_BIOZ_DLPF_MSK, config->bioz_cfg.setup.bioz_dlpf) |
		  FIELD_PREP(MAX86178_BIOZ_CFG2_BIOZ_DHPF_MSK, config->bioz_cfg.setup.bioz_dhpf);
	ret = max86178_reg_write(dev, MAX86178_BIOZ_CFG2, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ configuration 2: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_set_bioz_cfg3(const struct device *dev)
{
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;
	uint8_t reg_val;

	reg_val = FIELD_PREP(MAX86178_BIOZ_CFG3_BIOZ_DRV_MODE_MSK,
			     config->bioz_cfg.setup.bioz_drv_mode) |
		  FIELD_PREP(MAX86178_BIOZ_CFG3_BIOZ_IDRV_RGE_MSK,
			     config->bioz_cfg.setup.bioz_idrv_rge) |
		  FIELD_PREP(MAX86178_BIOZ_CFG3_BIOZ_VDRV_MAG_MSK,
			     config->bioz_cfg.setup.bioz_vdrv_mag) |
		  FIELD_PREP(MAX86178_BIOZ_CFG3_BIOZ_EXT_RES_MSK,
			     config->bioz_cfg.setup.bioz_ext_res);
	ret = max86178_reg_write(dev, MAX86178_BIOZ_CFG3, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ configuration 3: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_set_bioz_cfg4(const struct device *dev)
{
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;
	uint8_t reg_val;

	reg_val = FIELD_PREP(MAX86178_BIOZ_CFG4_EN_UTIL_MODE_MSK,
			     config->bioz_cfg.setup.en_util_mode);
	ret = max86178_reg_write(dev, MAX86178_BIOZ_CFG4, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ configuration 4: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_set_bioz_cfg5(const struct device *dev)
{
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;
	uint8_t reg_val;

	reg_val = FIELD_PREP(MAX86178_BIOZ_CFG5_BIOZ_DC_DAC_CODE_MSK,
			     config->bioz_cfg.setup.bioz_dc_dac_code) |
		  FIELD_PREP(MAX86178_BIOZ_CFG5_BIOZ_DC_CODE_SEL_MSK,
			     config->bioz_cfg.setup.bioz_dc_code_sel);
	ret = max86178_reg_write(dev, MAX86178_BIOZ_CFG5, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ configuration 5: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_set_bioz_cfg6(const struct device *dev)
{
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;
	uint8_t reg_val;

	reg_val = FIELD_PREP(MAX86178_BIOZ_CFG6_BIOZ_GAIN_MSK, config->bioz_cfg.setup.bioz_gain) |
		  FIELD_PREP(MAX86178_BIOZ_CFG6_DM_DIS_MSK, config->bioz_cfg.setup.bioz_dm_dis) |
		  FIELD_PREP(MAX86178_BIOZ_CFG6_BIOZ_INA_MODE_MSK,
			     config->bioz_cfg.setup.bioz_ina_mode) |
		  FIELD_PREP(MAX86178_BIOZ_CFG6_BIOZ_AHPF_MSK, config->bioz_cfg.setup.bioz_ahpf);
	ret = max86178_reg_write(dev, MAX86178_BIOZ_CFG6, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ configuration 6: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_set_bioz_cfg7(const struct device *dev)
{
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;
	uint8_t reg_val;

	reg_val =
		FIELD_PREP(MAX86178_BIOZ_CFG7_BIOZ_AMP_BW_MSK, config->bioz_cfg.setup.bioz_amp_bw) |
		FIELD_PREP(MAX86178_BIOZ_CFG7_BIOZ_AMP_RGE_MSK,
			   config->bioz_cfg.setup.bioz_amp_rge) |
		FIELD_PREP(MAX86178_BIOZ_CFG7_BIOZ_DAC_RESET_MSK,
			   config->bioz_cfg.setup.bioz_dac_reset) |
		FIELD_PREP(MAX86178_BIOZ_CFG7_BIOZ_DRV_RESET_MSK,
			   config->bioz_cfg.setup.bioz_drv_reset) |
		FIELD_PREP(MAX86178_BIOZ_CFG7_BIOZ_DC_RESTORE_MSK,
			   config->bioz_cfg.setup.bioz_dc_restore) |
		FIELD_PREP(MAX86178_BIOZ_CFG7_BIOZ_EXT_CAP_MSK,
			   config->bioz_cfg.setup.bioz_ext_cap);
	ret = max86178_reg_write(dev, MAX86178_BIOZ_CFG7, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ configuration 7: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_set_bioz_cfg8(const struct device *dev)
{
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;
	uint8_t reg_val;

	reg_val =
		FIELD_PREP(MAX86178_BIOZ_CFG8_BIOZ_CH_FSEL_MSK,
			   config->bioz_cfg.setup.bioz_ch_fsel) |
		FIELD_PREP(MAX86178_BIOZ_CFG8_BIOZ_INA_CHOP_EN_MSK,
			   config->bioz_cfg.setup.bioz_ina_chop_en) |
		FIELD_PREP(MAX86178_BIOZ_CFG8_BIOZ_FAST_MSK, config->bioz_cfg.setup.bioz_fast) |
		FIELD_PREP(MAX86178_BIOZ_CFG8_BIOZ_IPOL_MSK, config->bioz_cfg.setup.bioz_ipol) |
		FIELD_PREP(MAX86178_BIOZ_CFG8_BIOZ_STBYON_MSK, config->bioz_cfg.setup.bioz_stbyon) |
		FIELD_PREP(MAX86178_BIOZ_CFG8_BIOZ_CMRES_DIS_MSK,
			   config->bioz_cfg.setup.bioz_cmres_dis) |
		FIELD_PREP(MAX86178_BIOZ_CFG8_BIOZ_RLD_DRV_MSK, config->bioz_cfg.setup.rld_drv) |
		FIELD_PREP(MAX86178_BIOZ_CFG8_BIOZ_RLD_SEK_BIOZ_MSK,
			   config->bioz_cfg.setup.rld_sel_bioz);
	ret = max86178_reg_write(dev, MAX86178_BIOZ_CFG8, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ configuration 8: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_init_bioz_lo_thresh(const struct device *dev)
{
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;
	uint8_t reg_val = config->bioz_cfg.setup.bioz_lo_thresh;

	ret = max86178_reg_write(dev, MAX86178_BIOZ_LO_THRESH, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ low threshold: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_init_bioz_hi_thresh(const struct device *dev)
{
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;
	uint8_t reg_val = config->bioz_cfg.setup.bioz_hi_thresh;

	ret = max86178_reg_write(dev, MAX86178_BIOZ_HI_THRESH, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ high threshold: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_set_bioz_setup(const struct device *dev)
{
	int ret = 0;

	/* Set BioZ configuration 2 */
	ret = max86178_set_bioz_cfg2(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ configuration 2: %d", ret);
		return ret;
	}

	/* Set BioZ configuration 3 */
	ret = max86178_set_bioz_cfg3(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ configuration 3: %d", ret);
		return ret;
	}

	/* Set BioZ configuration 4 */
	ret = max86178_set_bioz_cfg4(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ configuration 4: %d", ret);
		return ret;
	}

	/* Set BioZ configuration 5 */
	ret = max86178_set_bioz_cfg5(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ configuration 5: %d", ret);
		return ret;
	}

	/* Set BioZ configuration 6 */
	ret = max86178_set_bioz_cfg6(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ configuration 6: %d", ret);
		return ret;
	}

	/* Set BioZ configuration 7 */
	ret = max86178_set_bioz_cfg7(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ configuration 7: %d", ret);
		return ret;
	}

	/* Set BioZ configuration 8 */
	ret = max86178_set_bioz_cfg8(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ configuration 8: %d", ret);
		return ret;
	}

	/* Set BioZ low threshold */
	ret = max86178_init_bioz_lo_thresh(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ low threshold: %d", ret);
		return ret;
	}

	/* Set BioZ high threshold */
	ret = max86178_init_bioz_hi_thresh(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ high threshold: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_set_bioz_mux1(const struct device *dev)
{
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;
	uint8_t reg_val;

	reg_val = FIELD_PREP(MAX86178_BIOZ_MUX_CFG1_BIOZ_CAL_EN_MSK,
			     config->bioz_cfg.calibration.bioz_cal_en) |
		  FIELD_PREP(MAX86178_BIOZ_MUX_CFG1_BIOZ_MUX_EN_MSK,
			     config->bioz_cfg.calibration.bioz_mux_en) |
		  FIELD_PREP(MAX86178_BIOZ_MUX_CFG1_BIOZ_CONNECT_CAL_ONLY_MSK,
			     config->bioz_cfg.calibration.connect_cal_only) |
		  FIELD_PREP(MAX86178_BIOZ_MUX_CFG1_BMUX_BIST_EN_MSK,
			     config->bioz_cfg.calibration.bmux_bist_en) |
		  FIELD_PREP(MAX86178_BIOZ_MUX_CFG1_BMUX_RSEL_MSK,
			     config->bioz_cfg.calibration.bmux_rsel);
	ret = max86178_reg_write(dev, MAX86178_BIOZ_MUX_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ MUX configuration: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_set_bioz_mux2(const struct device *dev)
{
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;
	uint8_t reg_val;

	reg_val = FIELD_PREP(MAX86178_BIOZ_MUX_CFG2_EN_INT_INLOAD_MSK,
			     config->bioz_cfg.calibration.en_int_inload) |
		  FIELD_PREP(MAX86178_BIOZ_MUX_CFG2_EN_EXT_INLOAD_MSK,
			     config->bioz_cfg.calibration.en_ext_inload) |
		  FIELD_PREP(MAX86178_BIOZ_MUX_CFG2_GSR_LOAD_EN_MSK,
			     config->bioz_cfg.calibration.gsr_load_en) |
		  FIELD_PREP(MAX86178_BIOZ_MUX_CFG2_BMUX_GSR_RSEL_MSK,
			     config->bioz_cfg.calibration.bmux_gsr_rsel);
	ret = max86178_reg_write(dev, MAX86178_BIOZ_MUX_CFG2, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ MUX configuration 2: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_set_bioz_mux3(const struct device *dev)
{
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;
	uint8_t reg_val;

	reg_val = FIELD_PREP(MAX86178_BIOZ_MUX_CFG3_DRVN_ASSIGN_MSK,
			     config->bioz_cfg.calibration.drvn_assign) |
		  FIELD_PREP(MAX86178_BIOZ_MUX_CFG3_DRVP_ASSIGN_MSK,
			     config->bioz_cfg.calibration.drvp_assign) |
		  FIELD_PREP(MAX86178_BIOZ_MUX_CFG3_BIN_ASSIGN_MSK,
			     config->bioz_cfg.calibration.bin_assign) |
		  FIELD_PREP(MAX86178_BIOZ_MUX_CFG3_BIP_ASSIGN_MSK,
			     config->bioz_cfg.calibration.bip_assign);
	ret = max86178_reg_write(dev, MAX86178_BIOZ_MUX_CFG3, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ MUX configuration 3: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_set_bioz_mux4(const struct device *dev)
{
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;
	uint8_t reg_val = config->bioz_cfg.calibration.bist_r_err;

	ret = max86178_reg_write(dev, MAX86178_BIOZ_MUX_CFG4, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ MUX configuration 4: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_set_bioz_calibration(const struct device *dev)
{
	int ret = 0;

	/* Set BioZ MUX configuration 1 */
	ret = max86178_set_bioz_mux1(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ MUX configuration 1: %d", ret);
		return ret;
	}

	/* Set BioZ MUX configuration 2 */
	ret = max86178_set_bioz_mux2(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ MUX configuration 2: %d", ret);
		return ret;
	}

	/* Set BioZ MUX configuration 3 */
	ret = max86178_set_bioz_mux3(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ MUX configuration 3: %d", ret);
		return ret;
	}

	/* Set BioZ MUX configuration 4 */
	ret = max86178_set_bioz_mux4(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ MUX configuration 4: %d", ret);
		return ret;
	}

	return 0;
}

static int max86178_bioz_set_bioz_lead_detect_cfg1(const struct device *dev)
{
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;
	uint8_t reg_val;

	reg_val = FIELD_PREP(MAX86178_BIOZ_LD_CFG1_BIOZ_LOFF_IMAG_MSK,
			     config->bioz_cfg.lead_detect.bioz_loff_current_mag) |
		  FIELD_PREP(MAX86178_BIOZ_LD_CFG1_BIOZ_LOFF_IPOL_MSK,
			     config->bioz_cfg.lead_detect.bioz_loff_ipol) |
		  FIELD_PREP(MAX86178_BIOZ_LD_CFG1_EN_BIOZ_DRV_OOR_MSK,
			     config->bioz_cfg.lead_detect.en_bioz_drv_oor) |
		  FIELD_PREP(MAX86178_BIOZ_LD_CFG1_EN_EXT_BIOZ_LOFF_MSK,
			     config->bioz_cfg.lead_detect.en_ext_bioz_loff) |
		  FIELD_PREP(MAX86178_BIOZ_LD_CFG1_EN_BIOZ_LOFF_MSK,
			     config->bioz_cfg.lead_detect.en_bioz_loff) |
		  FIELD_PREP(MAX86178_BIOZ_LD_CFG1_EN_BIOZ_LON_MSK,
			     config->bioz_cfg.lead_detect.en_bioz_lon);
	ret = max86178_reg_write(dev, MAX86178_BIOZ_LD_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ lead detection configuration 1: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_bioz_set_bioz_lead_off_thresh(const struct device *dev)
{
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;
	uint8_t reg_val;

	reg_val = FIELD_PREP(MAX86178_BIOZ_LOFF_THRESH_BIOZ_LOFF_THRESH_MSK,
			     config->bioz_cfg.lead_detect.bioz_loff_thresh) |
		  FIELD_PREP(MAX86178_BIOZ_LOFF_THRESH_RESP_CG_MAG_MSK,
			     config->bioz_cfg.lead_detect.resp_cg_mag) |
		  FIELD_PREP(MAX86178_BIOZ_LOFF_THRESH_RESP_CG_MAG_4X_MSK,
			     config->bioz_cfg.lead_detect.resp_cg_mag4x);
	ret = max86178_reg_write(dev, MAX86178_BIOZ_LOFF_THRESH, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ lead off threshold: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_set_bioz_lead_detect(const struct device *dev)
{
	int ret = 0;

	/* Set BioZ lead detection configuration 1 */
	ret = max86178_bioz_set_bioz_lead_detect_cfg1(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ lead detection configuration 1: %d", ret);
		return ret;
	}

	/* Set BioZ lead off threshold */
	ret = max86178_bioz_set_bioz_lead_off_thresh(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ lead off threshold: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_set_bioz_lead_bias(const struct device *dev)
{
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;
	uint8_t reg_val;

	reg_val = FIELD_PREP(MAX86178_BIOZ_LB_CFG1_EN_BIOZ_RBIAS_N_MSK,
			     config->bioz_cfg.lead_bias.en_bioz_rbias_n) |
		  FIELD_PREP(MAX86178_BIOZ_LB_CFG1_EN_BIOZ_RBIAS_P_MSK,
			     config->bioz_cfg.lead_bias.en_bioz_rbias_p) |
		  FIELD_PREP(MAX86178_BIOZ_LB_CFG1_BIOZ_RBIAS_VALUE_MSK,
			     config->bioz_cfg.lead_bias.bioz_rbias_value);
	ret = max86178_reg_write(dev, MAX86178_BIOZ_LB_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ lead bias configuration: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_set_resp_cfg(const struct device *dev)
{
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;
	uint8_t reg_val;

	reg_val = FIELD_PREP(MAX86178_RESP_CFG1_RESP_EN_MSK, config->resp_cfg.resp_en) |
		  FIELD_PREP(MAX86178_RESP_CFG1_CG_MODE, config->resp_cfg.cg_mode) |
		  FIELD_PREP(MAX86178_RESP_CFG1_CG_CHOP_CLK, config->resp_cfg.cg_chop_clk) |
		  FIELD_PREP(MAX86178_RESP_CFG1_CG_LPF_DUTY, config->resp_cfg.cg_lpf_duty);
	ret = max86178_reg_write(dev, MAX86178_RESP_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set Respiration configuration: %d", ret);
		return ret;
	}
	return 0;
}

static int max86178_bioz_init(const struct device *dev)
{
	int ret = 0;

	/* Set BioZ setup */
	ret = max86178_set_bioz_setup(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ setup: %d", ret);
		return ret;
	}

	/* Set BioZ calibration settings */
	ret = max86178_set_bioz_calibration(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ calibration: %d", ret);
		return ret;
	}

	/* Set BioZ lead detection settings */
	ret = max86178_set_bioz_lead_detect(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ lead detection configuration: %d", ret);
		return ret;
	}

	/* Set BioZ lead bias settings */
	ret = max86178_set_bioz_lead_bias(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set BioZ lead bias configuration: %d", ret);
		return ret;
	}

	/* Set Respiration settings */
	ret = max86178_set_resp_cfg(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set Respiration configuration: %d", ret);
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

static int max86178_ecg_en(const struct device *dev, bool enable)
{
	int ret;
	ret = max86178_reg_update(dev, MAX86178_ECG_CFG1, MAX86178_ECG_CFG1_ECG_EN_MSK,
				  enable ? 1 : 0);
	if (ret < 0) {
		LOG_ERR("Failed to %s ECG channel: %d", enable ? "enable" : "disable", ret);
		return ret;
	}
	return 0;
}

static int max86178_bioz_en(const struct device *dev, enum max86178_bioz_en bioz_channel_en,
			    enum max86178_ecg_bioz_bg_en ecg_bioz_bg_en)
{
	int ret;
	ret = max86178_reg_update(dev, MAX86178_BIOZ_CFG1, MAX86178_BIOZ_CFG1_ECG_BIOZ_BG_EN_MSK,
				  ecg_bioz_bg_en);
	if (ret < 0) {
		LOG_ERR("Failed to %s ECG BioZ Bandgap Reference: %d",
			ecg_bioz_bg_en ? "enable" : "disable", ret);
		return ret;
	}

	ret = max86178_reg_update(dev, MAX86178_BIOZ_CFG1, MAX86178_BIOZ_CFG1_BIOZ_EN_MSK,
				  bioz_channel_en);
	if (ret < 0) {
		LOG_ERR("Failed to %s BioZ channel: %d", bioz_channel_en ? "enable" : "disable",
			ret);
		return ret;
	}
	return 0;
}

static int max86178_sys_cfg_init(const struct device *dev)
{
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;
	uint8_t reg_val;

	reg_val = FIELD_PREP(MAX86178_SYS_CFG1_ECG_BIOZ_TIMING_DATA_MSK,
			     config->sys_cfg.ecg_bioz_timing_data_en) |
		  FIELD_PREP(MAX86178_SYS_CFG1_BIOZ_PPG_TIMING_DATA_MSK,
			     config->sys_cfg.bioz_ppg_timing_data_en) |
		  FIELD_PREP(MAX86178_SYS_CFG1_ECG_PPG_TIMING_DATA_MSK,
			     config->sys_cfg.ecg_ppg_timing_data_en);
	ret = max86178_reg_write(dev, MAX86178_SYS_CFG1, &reg_val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set system configuration: %d", ret);
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

	/* Initialize ECG */
	ret = max86178_ecg_init(dev);
	if (ret < 0) {
		LOG_ERR("ECG initialization failed");
		return ret;
	}

	/* Initialize BioZ */
	ret = max86178_bioz_init(dev);
	if (ret < 0) {
		LOG_ERR("BioZ initialization failed");
		return ret;
	}

	/* Initialize system configuration */
	ret = max86178_sys_cfg_init(dev);
	if (ret < 0) {
		LOG_ERR("System configuration initialization failed");
		return ret;
	}

	ret = max86178_osc_enable(dev, true);
	if (ret < 0) {
		LOG_ERR("Failed to disable internal oscillator: %d", ret);
		return ret;
	}

	/* Enable Channels */
	ret = max86178_set_ppg_meas_en(dev);
	if (ret < 0) {
		LOG_ERR("Failed to set PPG measurement enable: %d", ret);
		return ret;
	}

	ret = max86178_ecg_en(dev, config->ecg_cfg.setup.ecg_en);
	if (ret < 0) {
		LOG_ERR("Failed to enable ECG channel: %d", ret);
		return ret;
	}

	ret = max86178_bioz_en(dev, config->bioz_cfg.setup.bioz_en,
			       config->bioz_cfg.setup.ecg_bioz_bg_en);
	if (ret < 0) {
		LOG_ERR("Failed to enable BioZ channel: %d", ret);
		return ret;
	}

	/* Store configuration values in data structure for runtime access */
#ifdef CONFIG_MAX86178_TRIGGER
	ret = max86178_init_interrupt(dev);
	if (ret < 0) {
		LOG_ERR("Failed to initialize triggers: %d", ret);
		return ret;
	}
#endif /* CONFIG_MAX86178_TRIGGER */
	return 0;
}
/*******************************************************************************
 * CLOCK CONFIGURATION PARSING MACROS
 ******************************************************************************/

/* Clock Oscillator Configuration */
#define MAX86178_CLK_OSC_CFG(inst)                                                                 \
	{                                                                                          \
		.ref_clk = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), clk), osc),             \
				      ref_clk_sel, 0),                                             \
		.clk_freq_sel = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), clk), osc),        \
					   clk_freq_sel, 0),                                       \
		.clk_fine_tune = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), clk), osc),       \
					    clk_fine_tune, 0),                                     \
		.mdiv = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), clk), osc), mdiv, 0),      \
	}

/* PPG Clock Configuration */
#define MAX86178_CLK_PPG_CFG(inst)                                                                 \
	{                                                                                          \
		.ppg_fr_clk_div = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), clk), ppg),      \
					     fr_clk_div, 0),                                       \
	}

/* ECG Clock Configuration */
#define MAX86178_CLK_ECG_CFG(inst)                                                                 \
	{                                                                                          \
		.ecg_fdiv =                                                                        \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), clk), ecg), ecg_fdiv, 0),  \
		.ecg_ndiv =                                                                        \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), clk), ecg), ecg_ndiv, 0),  \
		.ecg_dec_rate = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), clk), ecg),        \
					   ecg_dec_rate, 0),                                       \
	}

/* BioZ Clock Configuration */
#define MAX86178_CLK_BIOZ_CFG(inst)                                                                \
	{                                                                                          \
		.bioz_kdiv = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), clk), bioz),          \
					bioz_kdiv, 0),                                             \
		.bioz_ndiv = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), clk), bioz),          \
					bioz_ndiv, 0),                                             \
		.bioz_adc_osr = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), clk), bioz),       \
					   bioz_adc_osr, 0),                                       \
		.bioz_dac_osr = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), clk), bioz),       \
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
		.fifo_watermark =                                                                  \
			DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), fifo), fifo_watermark, 256),        \
		.fifo_rollover_en =                                                                \
			DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), fifo), fifo_rollover_en, 0),        \
		.fifo_a_full_type =                                                                \
			DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), fifo), fifo_a_full_type, 0),        \
	}

/*******************************************************************************
 * PPG CONFIGURATION PARSING MACROS
 ******************************************************************************/

/* PPG Measurement Slot Configuration - for a specific measurement index */
#define MAX86178_PPG_MEAS_CFG_BY_IDX(inst, idx)                                                    \
	{                                                                                          \
		.drva = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx), drva,   \
				   0),                                                             \
		.drvb = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx), drvb,   \
				   0),                                                             \
		.amb_mode = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx),     \
				       amb_mode, 0),                                               \
		.avg_num = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx),      \
				      avg_num, 0),                                                 \
		.sinc3_sel = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx),    \
					sinc3_sel, 0),                                             \
		.filt_sel = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx),     \
				       filt_sel, 0),                                               \
		.filt2_sel = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx),    \
					filt2_sel, 0),                                             \
		.ppg1_dac_off = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx), \
					   ppg1_dac_off, 0),                                       \
		.ppg1_adc_rge = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx), \
					   ppg1_adc_rge, 0),                                       \
		.ppg2_dac_off = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx), \
					   ppg2_dac_off, 0),                                       \
		.ppg2_adc_rge = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx), \
					   ppg2_adc_rge, 0),                                       \
		.tint = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx), tint,   \
				   0),                                                             \
		.pd_setlng = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx),    \
					pd_setlng, 0),                                             \
		.led_setlng = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx),   \
					 led_setlng, 0),                                           \
		.led_rge = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx),      \
				      led_rge, 0),                                                 \
		.pd1_sel = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx),      \
				      pd1_sel, 1),                                                 \
		.pd2_sel = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx),      \
				      pd2_sel, 1),                                                 \
		.pd3_sel = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx),      \
				      pd3_sel, 1),                                                 \
		.pd4_sel = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx),      \
				      pd4_sel, 1),                                                 \
		.drva_pa = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx),      \
				      drva_pa, 0),                                                 \
		.drvb_pa = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), meas_##idx),      \
				      drvb_pa, 0),                                                 \
	}

/* PPG Threshold Configuration */
#define MAX86178_PPG_THRESHOLD_CFG(inst)                                                           \
	{                                                                                          \
		.thresh1_meas_sel =                                                                \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), threshold),          \
				   thresh1_meas_sel, 0),                                           \
		.thresh2_meas_sel =                                                                \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), threshold),          \
				   thresh2_meas_sel, 0),                                           \
		.thresh1_chan_sel =                                                                \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), threshold),          \
				   thresh1_chan_sel, 0),                                           \
		.thresh2_chan_sel =                                                                \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), threshold),          \
				   thresh2_chan_sel, 0),                                           \
		.time_hyst = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), threshold),     \
					time_hyst, 0),                                             \
		.level_hyst = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), threshold),    \
					 level_hyst, 0),                                           \
		.thresh1_hi = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), threshold),    \
					 thresh1_hi, 0),                                           \
		.thresh1_lo = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), threshold),    \
					 thresh1_lo, 0),                                           \
		.thresh2_hi = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), threshold),    \
					 thresh2_hi, 0),                                           \
		.thresh2_lo = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ppg), threshold),    \
					 thresh2_lo, 0),                                           \
	}

/* Complete PPG Configuration */
#define MAX86178_PPG_CFG(inst)                                                                     \
	{                                                                                          \
		.meas_en =                                                                         \
			{                                                                          \
				DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), meas1_en, false),     \
				DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), meas2_en, false),     \
				DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), meas3_en, false),     \
				DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), meas4_en, false),     \
				DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), meas5_en, false),     \
				DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), meas6_en, false),     \
			},                                                                         \
		.ppg1_pwrdn = DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), ppg1_pwrdn, 0),         \
		.ppg2_pwrdn = DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), ppg2_pwrdn, 0),         \
		.ppg_sync_mode = DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), ppg_sync_mode, 0),   \
		.prox_data_en = DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), prox_data_en, false), \
		.prox_auto_en = DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), prox_auto_en, false), \
		.alc_disable = DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), alc_disable, false),   \
		.collect_raw_data =                                                                \
			DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), collect_raw_data, false),     \
		.meas1_config_sel =                                                                \
			DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), meas1_config_sel, false),     \
		.smp_ave = DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), smp_ave, 0),               \
		.pd1_bias = DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), pd1_bias, 1),             \
		.pd2_bias = DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), pd2_bias, 1),             \
		.pd3_bias = DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), pd3_bias, 1),             \
		.pd4_bias = DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), ppg), pd4_bias, 1),             \
		.meas_cfg =                                                                        \
			{                                                                          \
				MAX86178_PPG_MEAS_CFG_BY_IDX(inst, 1),                             \
				MAX86178_PPG_MEAS_CFG_BY_IDX(inst, 2),                             \
				MAX86178_PPG_MEAS_CFG_BY_IDX(inst, 3),                             \
				MAX86178_PPG_MEAS_CFG_BY_IDX(inst, 4),                             \
				MAX86178_PPG_MEAS_CFG_BY_IDX(inst, 5),                             \
				MAX86178_PPG_MEAS_CFG_BY_IDX(inst, 6),                             \
			},                                                                         \
		.threshold_cfg = MAX86178_PPG_THRESHOLD_CFG(inst),                                 \
	}

/*******************************************************************************
 * ECG CONFIGURATION PARSING MACROS
 ******************************************************************************/

/* ECG Setup Configuration */
#define MAX86178_ECG_SETUP_CFG(inst)                                                               \
	{                                                                                          \
		.ecg_en = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), setup), ecg_en,    \
				     false),                                                       \
		.ecg_input_pol = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), setup),     \
					    ecg_input_pol, 0),                                     \
		.ecg_pga_gain = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), setup),      \
					   ecg_pga_gain, 0),                                       \
		.ecg_ina_rge = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), setup),       \
					  ecg_ina_rge, 0),                                         \
		.ecg_ina_gain = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), setup),      \
					   ecg_ina_gain, 0),                                       \
		.ecg_imp_hi = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), setup),        \
					 ecg_imp_hi, false),                                       \
		.ecg_auto_rec = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), setup),      \
					   ecg_auto_rec, 0),                                       \
		.ecg_mux_sel = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), setup),       \
					  ecg_mux_sel, 0),                                         \
		.en_ecg_fast_rec = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), setup),   \
					      en_ecg_fast_rec, 0),                                 \
		.ecg_fast_rec_thres = DT_PROP_OR(                                                  \
			DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), setup), ecg_fast_rec_thres, 0), \
	}

/* ECG Calibration Configuration */
#define MAX86178_ECG_CAL_CFG(inst)                                                                 \
	{                                                                                          \
		.ecg_cal_high = DT_PROP_OR(                                                        \
			DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), calibration), ecg_cal_high, 0), \
		.ecg_freq = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), calibration),    \
				       ecg_cal_freq, 0),                                           \
		.ecg_cal_duty = DT_PROP_OR(                                                        \
			DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), calibration), ecg_cal_duty, 0), \
		.ecg_cal_en = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), calibration),  \
					 ecg_cal_en, 0),                                           \
		.ecg_open_p = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), calibration),  \
					 ecg_open_p, 0),                                           \
		.ecg_open_n = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), calibration),  \
					 ecg_open_n, 0),                                           \
		.ecg_cal_mode = DT_PROP_OR(                                                        \
			DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), calibration), ecg_cal_mode, 0), \
		.ecg_cal_mag = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), calibration), \
					  ecg_cal_mag, 0),                                         \
		.ecg_cal_p_sel =                                                                   \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), calibration),        \
				   ecg_cal_p_sel, 0),                                              \
		.ecg_cal_n_sel =                                                                   \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), calibration),        \
				   ecg_cal_n_sel, 0),                                              \
	}

/* ECG Lead Detect Configuration */
#define MAX86178_ECG_LEAD_DETECT_CFG(inst)                                                         \
	{                                                                                          \
		.en_ecg_lon = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), lead_detect),  \
					 en_ecg_lon, 0),                                           \
		.en_ecg_loff = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), lead_detect), \
					  en_ecg_loff, 0),                                         \
		.ecg_loff_mode =                                                                   \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), lead_detect),        \
				   ecg_loff_mode, 0),                                              \
		.ecg_loff_freq =                                                                   \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), lead_detect),        \
				   ecg_loff_freq, 0),                                              \
		.ecg_loff_ipol =                                                                   \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), lead_detect),        \
				   ecg_loff_ipol, 0),                                              \
		.ecg_loff_current_mag =                                                            \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), lead_detect),        \
				   ecg_loff_current_mag, 0),                                       \
		.ecg_loff_thresh =                                                                 \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), lead_detect),        \
				   ecg_loff_thresh, 0),                                            \
	}

/* ECG Lead Bias Configuration */
#define MAX86178_ECG_LEAD_BIAS_CFG(inst)                                                           \
	{                                                                                          \
		.ecg_rbias_value =                                                                 \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), lead_bias),          \
				   ecg_rbias_value, 0),                                            \
		.en_ecg_rbias_p = DT_PROP_OR(                                                      \
			DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), lead_bias), en_ecg_rbias_p, 0), \
		.en_ecg_rbias_n = DT_PROP_OR(                                                      \
			DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), lead_bias), en_ecg_rbias_n, 0), \
	}

/* ECG RLD Configuration */
#define MAX86178_ECG_RLD_CFG(inst)                                                                 \
	{                                                                                          \
		.rld_en = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), rld), rld_en, 0),  \
		.rld_mode =                                                                        \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), rld), rld_mode, 0),  \
		.rld_rbias =                                                                       \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), rld), rld_rbias, 0), \
		.en_rld_oor = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), rld),          \
					 en_rld_oor, 0),                                           \
		.actv_cm_p =                                                                       \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), rld), actv_cm_p, 0), \
		.actv_cm_n =                                                                       \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), rld), actv_cm_n, 0), \
		.rld_gain =                                                                        \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), rld), rld_gain, 0),  \
		.rld_ext_res = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), rld),         \
					  rld_ext_res, false),                                     \
		.rld_sel_ecg = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), rld),         \
					  rld_sel_ecg, false),                                     \
		.rld_bw = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), rld), rld_bw, 0),  \
		.body_bias_dac = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), ecg), rld),       \
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
		.ecg_bioz_bg_en = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),   \
					     ecg_bioz_bg_en, 0),                                   \
		.bioz_en = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup), bioz_en, \
				      0),                                                          \
		.bioz_dhpf = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),        \
					bioz_dhpf, 0),                                             \
		.bioz_dlpf = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),        \
					bioz_dlpf, 0),                                             \
		.en_bioz_thresh = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),   \
					     en_bioz_thresh, 0),                                   \
		.bioz_ext_res = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),     \
					   bioz_ext_res, 0),                                       \
		.bioz_vdrv_mag = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),    \
					    bioz_vdrv_mag, 0),                                     \
		.bioz_idrv_rge = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),    \
					    bioz_idrv_rge, 0),                                     \
		.bioz_drv_mode = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),    \
					    bioz_drv_mode, 0),                                     \
		.en_util_mode = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),     \
					   en_util_mode, false),                                   \
		.bioz_dc_code_sel = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup), \
					       bioz_dc_code_sel, 0),                               \
		.bioz_dc_dac_code = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup), \
					       bioz_dc_dac_code, 0),                               \
		.bioz_ahpf = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),        \
					bioz_ahpf, 0),                                             \
		.bioz_ina_mode = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),    \
					    bioz_ina_mode, 0),                                     \
		.bioz_dm_dis = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),      \
					  bioz_dm_dis, 0),                                         \
		.bioz_gain = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),        \
					bioz_gain, 0),                                             \
		.bioz_ext_cap = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),     \
					   bioz_ext_cap, 0),                                       \
		.bioz_dc_restore = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),  \
					      bioz_dc_restore, 0),                                 \
		.bioz_drv_reset = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),   \
					     bioz_drv_reset, 0),                                   \
		.bioz_dac_reset = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),   \
					     bioz_dac_reset, false),                               \
		.bioz_amp_rge = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),     \
					   bioz_amp_rge, 0),                                       \
		.bioz_amp_bw = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),      \
					  bioz_amp_bw, 0),                                         \
		.rld_sel_bioz = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),     \
					   rld_sel_bioz, false),                                   \
		.rld_drv = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup), rld_drv, \
				      0),                                                          \
		.bioz_cmres_dis = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),   \
					     bioz_cmres_dis, 0),                                   \
		.bioz_stbyon = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),      \
					  bioz_stbyon, 0),                                         \
		.bioz_ipol = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),        \
					bioz_ipol, 0),                                             \
		.bioz_fast = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),        \
					bioz_fast, 0),                                             \
		.bioz_ina_chop_en = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup), \
					       bioz_ina_chop_en, 0),                               \
		.bioz_ch_fsel = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),     \
					   bioz_ch_fsel, 0),                                       \
		.bioz_lo_thresh = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),   \
					     bioz_lo_thresh, 0),                                   \
		.bioz_hi_thresh = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), setup),   \
					     bioz_hi_thresh, 0),                                   \
	}

/* BioZ Calibration Configuration */
#define MAX86178_BIOZ_CAL_CFG(inst)                                                                \
	{                                                                                          \
		.bmux_rsel = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), calibration),  \
					bmux_rsel, 0),                                             \
		.bmux_bist_en =                                                                    \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), calibration),       \
				   bmux_bist_en, 0),                                               \
		.connect_cal_only =                                                                \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), calibration),       \
				   connect_cal_only, false),                                       \
		.bioz_mux_en =                                                                     \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), calibration),       \
				   bioz_mux_en, false),                                            \
		.bioz_cal_en =                                                                     \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), calibration),       \
				   bioz_cal_en, false),                                            \
		.bmux_gsr_rsel =                                                                   \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), calibration),       \
				   bmux_gsr_rsel, 0),                                              \
		.gsr_load_en = DT_PROP_OR(                                                         \
			DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), calibration), gsr_load_en, 0), \
		.en_ext_inload =                                                                   \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), calibration),       \
				   en_ext_inload, 0),                                              \
		.en_int_inload =                                                                   \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), calibration),       \
				   en_int_inload, 0),                                              \
		.bip_assign = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), calibration), \
					 bip_assign, 0),                                           \
		.bin_assign = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), calibration), \
					 bin_assign, 0),                                           \
		.drvp_assign = DT_PROP_OR(                                                         \
			DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), calibration), drvp_assign, 0), \
		.drvn_assign = DT_PROP_OR(                                                         \
			DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), calibration), drvn_assign, 0), \
		.bist_r_err = DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), calibration), \
					 bist_r_err, 0),                                           \
	}

/* BioZ Lead Detect Configuration */
#define MAX86178_BIOZ_LEAD_DETECT_CFG(inst)                                                        \
	{                                                                                          \
		.en_bioz_lon = DT_PROP_OR(                                                         \
			DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), lead_detect), en_bioz_lon, 0), \
		.en_bioz_loff =                                                                    \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), lead_detect),       \
				   en_bioz_loff, 0),                                               \
		.en_ext_bioz_loff =                                                                \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), lead_detect),       \
				   en_ext_bioz_loff, 0),                                           \
		.en_bioz_drv_oor =                                                                 \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), lead_detect),       \
				   en_bioz_drv_oor, 0),                                            \
		.bioz_loff_ipol =                                                                  \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), lead_detect),       \
				   bioz_loff_ipol, false),                                         \
		.bioz_loff_current_mag =                                                           \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), lead_detect),       \
				   bioz_loff_current_mag, 0),                                      \
		.resp_cg_mag4x =                                                                   \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), lead_detect),       \
				   resp_cg_mag4x, false),                                          \
		.resp_cg_mag = DT_PROP_OR(                                                         \
			DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), lead_detect), resp_cg_mag, 0), \
		.bioz_loff_thresh =                                                                \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), lead_detect),       \
				   bioz_loff_thresh, 0),                                           \
	}

/* BioZ Lead Bias Configuration */
#define MAX86178_BIOZ_LEAD_BIAS_CFG(inst)                                                          \
	{                                                                                          \
		.en_bioz_rbias_p =                                                                 \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), lead_bias),         \
				   en_bioz_rbias_p, false),                                        \
		.en_bioz_rbias_n =                                                                 \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), lead_bias),         \
				   en_bioz_rbias_n, false),                                        \
		.bioz_rbias_value =                                                                \
			DT_PROP_OR(DT_CHILD(DT_CHILD(DT_DRV_INST(inst), bioz), lead_bias),         \
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
		.resp_en = DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), resp), resp_en, false),          \
		.cg_mode = DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), resp), cg_mode, 0),              \
		.cg_chop_clk = DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), resp), cg_chop_clk, 0),      \
		.cg_lpf_duty = DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), resp), cg_lpf_duty, 0),      \
	}

/*******************************************************************************
 * SYSTEM CONFIGURATION PARSING MACROS
 ******************************************************************************/
#define MAX86178_SYS_CFG(inst)                                                                     \
	{                                                                                          \
		.ecg_ppg_timing_data_en = DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), sys_cfg),         \
						     ecg_ppg_timing_data_en, false),               \
		.bioz_ppg_timing_data_en = DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), sys_cfg),        \
						      bioz_ppg_timing_data_en, false),             \
		.ecg_bioz_timing_data_en = DT_PROP_OR(DT_CHILD(DT_DRV_INST(inst), sys_cfg),        \
						      ecg_bioz_timing_data_en, false),             \
	}

/*******************************************************************************
 * IRQ CONFIGURATION PARSING MACROS
 ******************************************************************************/
#ifdef CONFIG_MAX86178_TRIGGER
#define MAX86178_CFG_IRQ(inst)                                                                     \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, int1_gpios),	\
		(	\
			.interrupt_gpio = GPIO_DT_SPEC_INST_GET(inst, int1_gpios),	\
			.route_to_int2 = false,	\
		),	\
		(	\
			.interrupt_gpio = GPIO_DT_SPEC_INST_GET(inst, int2_gpios),	\
			.route_to_int2 = true,	\
		))
#else
#define MAX86178_CFG_IRQ(inst)
#endif /* CONFIG_MAX86178_TRIGGER */
/*******************************************************************************
 * DEVICE INSTANTIATION MACROS
 ******************************************************************************/

/* Device instantiation macros */
#define MAX86178_CONFIG_I2C(inst)                                                                  \
	{.bus = {.i2c = I2C_DT_SPEC_INST_GET(inst)},                                               \
	 .bus_is_ready = max86178_bus_check_i2c,                                                   \
	 .reg_access = max86178_reg_access_i2c,                                                    \
	 .clk_cfg = MAX86178_CLK_CFG(inst),                                                        \
	 .ppg_cfg = MAX86178_PPG_CFG(inst),                                                        \
	 .ecg_cfg = MAX86178_ECG_CFG(inst),                                                        \
	 .bioz_cfg = MAX86178_BIOZ_CFG(inst),                                                      \
	 .fifo_cfg = MAX86178_FIFO_CFG(inst),                                                      \
	 .resp_cfg = MAX86178_RESP_CFG(inst),                                                      \
	 .sys_cfg = MAX86178_SYS_CFG(inst),                                                        \
	 COND_CODE_1(UTIL_OR(DT_INST_NODE_HAS_PROP(inst, int1_gpios), DT_INST_NODE_HAS_PROP(inst, int2_gpios)), (MAX86178_CFG_IRQ(inst)), ()) }

#define MAX86178_CONFIG_SPI(inst)                                                                  \
	{.bus = {.spi = SPI_DT_SPEC_INST_GET(                                                      \
			 inst, SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0)},       \
	 .bus_is_ready = max86178_bus_check_spi,                                                   \
	 .reg_access = max86178_reg_access_spi,                                                    \
	 .clk_cfg = MAX86178_CLK_CFG(inst),                                                        \
	 .ppg_cfg = MAX86178_PPG_CFG(inst),                                                        \
	 .ecg_cfg = MAX86178_ECG_CFG(inst),                                                        \
	 .bioz_cfg = MAX86178_BIOZ_CFG(inst),                                                      \
	 .fifo_cfg = MAX86178_FIFO_CFG(inst),                                                      \
	 .resp_cfg = MAX86178_RESP_CFG(inst),                                                      \
	 .sys_cfg = MAX86178_SYS_CFG(inst),                                                        \
	 COND_CODE_1(UTIL_OR(DT_INST_NODE_HAS_PROP(inst, int1_gpios), DT_INST_NODE_HAS_PROP(inst, int2_gpios)), (MAX86178_CFG_IRQ(inst)), ()) }

#ifdef CONFIG_MAX86178_STREAM
#define MAX86178_RTIO_SPI_DEFINE(inst)                                                             \
	COND_CODE_1(CONFIG_SPI_RTIO,                                                               \
		    (SPI_DT_IODEV_DEFINE(max86178_iodev_##inst, DT_DRV_INST(inst),                \
					 SPI_OP_MODE_MASTER | SPI_WORD_SET(8) |                   \
						 SPI_TRANSFER_MSB, 0);),                           \
		    ())

#define MAX86178_RTIO_I2C_DEFINE(inst)                                                             \
	COND_CODE_1(CONFIG_I2C_RTIO, (I2C_DT_IODEV_DEFINE(max86178_iodev_##inst,                  \
							   DT_DRV_INST(inst));),                   \
		    ())

/** RTIO SQE/CQE pool size depends on the fifo-watermark */
#define MAX86178_RTIO_DEFINE(inst)                                                                 \
	/* Conditionally include SPI and/or I2C parts based on their presence */                   \
	COND_CODE_1(DT_INST_ON_BUS(inst, spi), (MAX86178_RTIO_SPI_DEFINE(inst)), ())                                                                                \
	COND_CODE_1(DT_INST_ON_BUS(inst, i2c), (MAX86178_RTIO_I2C_DEFINE(inst)), ())                                                                                \
	RTIO_DEFINE(max86178_rtio_ctx_##inst, 8, 8);
#else
#define MAX86178_RTIO_DEFINE(inst)
#endif /* CONFIG_MAX86178_STREAM */

#define MAX86178_DEFINE(inst)                                                                      \
	MAX86178_RTIO_DEFINE(inst)                                                                 \
	static struct max86178_data max86178_data_##inst = {                                       \
		IF_ENABLED(CONFIG_MAX86178_STREAM,                                                 \
			   (.rtio_ctx = &max86178_rtio_ctx_##inst,                                 \
			    .iodev = &max86178_iodev_##inst, ))};              \
	static const struct max86178_dev_config max86178_config_##inst = COND_CODE_1(DT_INST_ON_BUS(inst, i2c), (MAX86178_CONFIG_I2C(inst)),                \
			    (MAX86178_CONFIG_SPI(inst))); \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, max86178_init, NULL, &max86178_data_##inst,             \
				     &max86178_config_##inst, POST_KERNEL,                         \
				     CONFIG_SENSOR_INIT_PRIORITY, &max86178_api);

DT_INST_FOREACH_STATUS_OKAY(MAX86178_DEFINE)
