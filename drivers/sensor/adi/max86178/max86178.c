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

static int max86178_ppg_init(const struct device *dev)
{
	/* Implement PPG sensor initialization */
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
