/*
 * Copyright (c) 2025 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_max40109

#include <zephyr/device.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include "max40109.h"
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor/max40109.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(max40109, CONFIG_SENSOR_LOG_LEVEL);

static int max40109_reg_access(const struct device *dev, uint8_t addr_reg, uint8_t *data, bool read,
			       uint8_t length)
{

	const struct max40109_config *config = dev->config;
	int ret;

	if (read) {
		ret = i2c_burst_read_dt(&config->i2c, addr_reg, data, length);
	} else {
		ret = i2c_burst_write_dt(&config->i2c, addr_reg, data, length);
	}

	return ret < 0 ? ret : 0;
}

int max40109_reg_read(const struct device *dev, uint8_t reg_addr, uint8_t *val, uint8_t length)
{
	return max40109_reg_access(dev, reg_addr, val, true, length);
}

int max40109_reg_write(const struct device *dev, uint8_t reg_addr, uint8_t val)
{
	return max40109_reg_access(dev, reg_addr, &val, false, 1);
}

int max40109_reg_write_multiple(const struct device *dev, uint8_t reg_addr, const uint8_t *val,
				uint8_t length)
{
	const struct max40109_config *config = dev->config;
	int ret;
	if (length < 2) {
		return -EINVAL; // Invalid length
	}
	return max40109_reg_access(dev, reg_addr, val, false, length);
}

int max40109_reg_update(const struct device *dev, uint8_t reg_addr, uint8_t mask, uint8_t val)
{
	uint8_t reg_val = 0;
	int ret;

	ret = max40109_reg_read(dev, reg_addr, &reg_val, 1);
	if (ret < 0) {
		return ret;
	}

	reg_val &= ~mask;                 // Clear the bits specified by the mask
	reg_val |= FIELD_PREP(mask, val); // Set the bits specified by val

	return max40109_reg_write(dev, reg_addr, reg_val);
}

int max40109_mtp_read(const struct device *dev, uint8_t mtp_addr, uint8_t *val)
{
	const struct max40109_config *config = dev->config;
	int ret = 0;

	ret = max40109_reg_write(dev, MAX40109_MTP_CONTROL, 0x10);
	if (ret < 0) {
		return ret; // Error writing MTP control register
	}
	ret = max40109_reg_write(dev, MAX40109_MTP_PROT_ADDR, mtp_addr);
	if (ret < 0) {
		return ret; // Error reading MTP protection address
	}

	return max40109_reg_read(dev, MAX40109_MTP_PROT_RDATA_MSB, val, MAX40109_MTP_DATA_LENGTH);
}

int max40109_mtp_write(const struct device *dev, uint8_t mtp_addr, const uint8_t *val)
{
	const struct max40109_config *config = dev->config;
	int ret = 0;

	ret = max40109_reg_write(dev, MAX40109_MTP_CONTROL, 0x10);
	if (ret < 0) {
		return ret; // Error writing MTP control register
	}
	ret = max40109_reg_write(dev, MAX40109_MTP_PROT_ADDR, mtp_addr);
	if (ret < 0) {
		return ret; // Error writing MTP protection address
	}

	return max40109_reg_write_multiple(dev, MAX40109_MTP_PROT_WDATA_MSB, val,
					   MAX40109_MTP_DATA_LENGTH);
}

int max40109_mtp_update(const struct device *dev, uint8_t mtp_addr, uint16_t mask, uint16_t val)
{
	uint8_t reg_val[2] = {0};
	int ret;

	ret = max40109_mtp_read(dev, mtp_addr, reg_val);
	if (ret < 0) {
		return ret; // Error reading MTP data
	}

	uint16_t reg = (reg_val[0] << 8) | reg_val[1]; // Combine MSB and LSB into a 16-bit value
	reg &= ~mask;                                  // Clear the bits specified by the mask
	reg |= FIELD_PREP(mask, val);                  // Set the bits specified by val
	reg_val[0] = (reg >> 8) & 0xFF;                // MSB
	reg_val[1] = reg & 0xFF;                       // LSB

	return max40109_mtp_write(dev, mtp_addr, reg_val);
}

static void coeff_hex_to_bytes(const uint32_t hex, uint8_t *buf_msb, uint8_t *buf_lsb)
{
	// Convert a 32-bit hex value to 2 bytes
	buf_msb[0] = (hex >> 24) & 0xFF; // MSB
	buf_msb[1] = (hex >> 16) & 0xFF;
	buf_lsb[0] = (hex >> 8) & 0xFF;
	buf_lsb[1] = hex & 0xFF; // LSB
}

int max40109_mtp_initialize(const struct device *dev)
{
	const struct max40109_config *config = dev->config;
	int ret = 0;

	ret = max40109_reg_write(dev, MAX40109_CP_CONTROL_1, 0x80);
	if (ret) {
		return ret;
	}

	ret = max40109_reg_write(dev, MAX40109_CP_CONTROL_2, 0x1B);
	if (ret) {
		return ret;
	}

	uint8_t slp_mr[2] = {0x03, 0x02};
	ret = max40109_reg_write_multiple(dev, MAX40109_SLP_MR, slp_mr, 2);
	if (ret < 0) {
		return ret; // Error writing SLP_MR register
	}

	uint8_t slp_mref[2] = {0x02, 0x00};
	ret = max40109_reg_write_multiple(dev, MAX40109_SLP_MREF, slp_mref, 2);
	if (ret < 0) {
		return ret; // Error writing SLP_MREF register
	}

	uint8_t slp_mrv[2] = {0x03, 0x00};
	ret = max40109_reg_write_multiple(dev, MAX40109_SLP_MRV_MSB, slp_mrv, 2);
	if (ret < 0) {
		return ret; // Error writing SLP_MRV register
	}

	uint8_t slp_mrefv[2] = {0x04, 0x01};
	ret = max40109_reg_write_multiple(dev, MAX40109_SLP_MREFV_MSB, slp_mrefv, 2);
	if (ret < 0) {
		return ret; // Error writing SLP_MREFV register
	}

	return 0;
}

static int max40109_is_burn_successful(const struct device *dev)
{
	const struct max40109_config *config = dev->config;
	uint8_t mtp_status = 0;
	int ret;

	ret = max40109_reg_read(dev, MAX40109_MTP_STATUS, &mtp_status, 1);
	if (ret) {
		return ret; // Error reading MTP status
	} else {

		if (mtp_status &
		    (MTP_ECC_ERR_2_BIT_MASK | MTP_ECC_ERR_1_BIT_MASK | MTP_VPP_INIT_FAIL_MASK |
		     MTP_FULL_MASK | MTP_VERIFICATION_FAIL_MASK | MTP_VPP_ACT_MASK)) {
			LOG_ERR("MTP ECC error detected");
			return -EIO; // MTP ECC error detected
		} else if (mtp_status & MTP_BURN_DONE_MASK) {
			LOG_INF("MTP burn successful");
			return 0; // MTP burn completed successfully
		} else {
			LOG_ERR("MTP burn not completed successfully");
			return -EIO; // MTP burn not completed successfully
		}
	}

	return 0;
}

static const struct calibration_coeff_map cal_table[] = {
	{MAX40109_CALIBRATION_K0, CAL_DATA0},  {MAX40109_CALIBRATION_K1, CAL_DATA2},
	{MAX40109_CALIBRATION_K2, CAL_DATA4},  {MAX40109_CALIBRATION_K3, CAL_DATA6},
	{MAX40109_CALIBRATION_H0, CAL_DATA8},  {MAX40109_CALIBRATION_H1, CAL_DATA10},
	{MAX40109_CALIBRATION_H2, CAL_DATA12}, {MAX40109_CALIBRATION_H3, CAL_DATA14},
	{MAX40109_CALIBRATION_G0, CAL_DATA16}, {MAX40109_CALIBRATION_G1, CAL_DATA18},
	{MAX40109_CALIBRATION_G2, CAL_DATA20}, {MAX40109_CALIBRATION_G3, CAL_DATA22},
	{MAX40109_CALIBRATION_N0, CAL_DATA24}, {MAX40109_CALIBRATION_N1, CAL_DATA26},
	{MAX40109_CALIBRATION_N2, CAL_DATA28}, {MAX40109_CALIBRATION_N3, CAL_DATA30},
	{MAX40109_CALIBRATION_M0, CAL_DATA32}, {MAX40109_CALIBRATION_M1, CAL_DATA34},
	{MAX40109_CALIBRATION_M2, CAL_DATA36}, {MAX40109_CALIBRATION_M3, CAL_DATA38},
};

static int get_calibration_address(uint8_t cal_coeff, uint8_t *mtp_addr)
{
	for (size_t i = 0; i < sizeof(cal_table) / sizeof(cal_table[0]); i++) {
		if (cal_table[i].cal_coeff == cal_coeff) {
			*mtp_addr = cal_table[i].cal_start_addr;
			return 0;
		}
	}
	return -EINVAL; // Invalid calibration ID
}

int max40109_mtp_calibration(const struct device *dev, enum max40109_calibration_coefficients coeff,
			     float value, bool burn)
{
	const struct max40109_config *config = dev->config;
	uint8_t mtp_addr = 0;
	uint32_t hex = FLOAT_TO_HEX(value);
	uint8_t buf_msb[2] = {0, 0};
	uint8_t buf_lsb[2] = {0, 0};
	int ret = 0;

	if (coeff < MAX40109_CALIBRATION_K0 || coeff > MAX40109_CALIBRATION_M3) {
		LOG_ERR("Invalid calibration coefficient");
		return -EINVAL; // Invalid coefficient
	}
	coeff_hex_to_bytes(hex, buf_msb, buf_lsb);

	ret = max40109_mtp_initialize(dev);
	if (ret < 0) {
		return ret; // Error initializing MTP
	}

	ret = get_calibration_address(coeff, &mtp_addr);
	if (ret < 0) {
		LOG_ERR("Failed to get MTP address for coefficient %d", coeff);
		return ret; // Error getting MTP address
	}

	if (burn) {
		LOG_INF("Burning MTP data for coefficient %d", coeff);
		ret = max40109_mtp_burn(dev, mtp_addr, buf_lsb);
		if (ret < 0) {
			LOG_ERR("Failed to burn MTP data for coefficient %d", coeff);
			return ret; // Error burning MTP data
		}
		ret = max40109_mtp_burn(dev, mtp_addr + 1, buf_msb);
		if (ret < 0) {
			LOG_ERR("Failed to burn MTP data for coefficient %d", coeff);
			return ret; // Error burning MTP data
		}
	} else {
		LOG_INF("Writing MTP data for coefficient %d without burning", coeff);
		ret = max40109_mtp_write(dev, mtp_addr, buf_lsb);
		if (ret < 0) {
			LOG_ERR("Failed to write MTP data for coefficient %d", coeff);
			return ret; // Error writing MTP data
		}
		ret = max40109_mtp_write(dev, mtp_addr + 1, buf_msb);
		if (ret < 0) {
			LOG_ERR("Failed to write MTP data for coefficient %d", coeff);
			return ret; // Error writing MTP data
		}
		LOG_INF("MTP data for coefficient %d written successfully", coeff);
	}

	return 0; // Success
}

static const struct temp_gain_map temp_gain_table[] = {
	{1, 5, SENSOR_MAX40109_PGA_TEMPERATURE_GAIN_1_5},
	{2, 0, SENSOR_MAX40109_PGA_TEMPERATURE_GAIN_2},
	{3, 0, SENSOR_MAX40109_PGA_TEMPERATURE_GAIN_3},
	{5, 0, SENSOR_MAX40109_PGA_TEMPERATURE_GAIN_5},
	{6, 0, SENSOR_MAX40109_PGA_TEMPERATURE_GAIN_6},
	{10, 0, SENSOR_MAX40109_PGA_TEMPERATURE_GAIN_10},
	{15, 0, SENSOR_MAX40109_PGA_TEMPERATURE_GAIN_15},
	{20, 0, SENSOR_MAX40109_PGA_TEMPERATURE_GAIN_20},
	{24, 0, SENSOR_MAX40109_PGA_TEMPERATURE_GAIN_24},
	{30, 0, SENSOR_MAX40109_PGA_TEMPERATURE_GAIN_30},
	{36, 0, SENSOR_MAX40109_PGA_TEMPERATURE_GAIN_36},
	{40, 0, SENSOR_MAX40109_PGA_TEMPERATURE_GAIN_40},
	{45, 0, SENSOR_MAX40109_PGA_TEMPERATURE_GAIN_45},
	{60, 0, SENSOR_MAX40109_PGA_TEMPERATURE_GAIN_60},
	{72, 0, SENSOR_MAX40109_PGA_TEMPERATURE_GAIN_72},
	{90, 0, SENSOR_MAX40109_PGA_TEMPERATURE_GAIN_90},
};

static const struct pressure_gain_map pressure_gain_table[] = {
	{5, SENSOR_MAX40109_PGA_PRESSURE_GAIN_5},
	{10, SENSOR_MAX40109_PGA_PRESSURE_GAIN_10},
	{15, SENSOR_MAX40109_PGA_PRESSURE_GAIN_15},
	{20, SENSOR_MAX40109_PGA_PRESSURE_GAIN_20},
	{24, SENSOR_MAX40109_PGA_PRESSURE_GAIN_24},
	{40, SENSOR_MAX40109_PGA_PRESSURE_GAIN_40},
	{60, SENSOR_MAX40109_PGA_PRESSURE_GAIN_60},
	{72, SENSOR_MAX40109_PGA_PRESSURE_GAIN_72},
	{90, SENSOR_MAX40109_PGA_PRESSURE_GAIN_90},
	{108, SENSOR_MAX40109_PGA_PRESSURE_GAIN_108},
	{126, SENSOR_MAX40109_PGA_PRESSURE_GAIN_126},
	{144, SENSOR_MAX40109_PGA_PRESSURE_GAIN_144},
	{160, SENSOR_MAX40109_PGA_PRESSURE_GAIN_160},
	{180, SENSOR_MAX40109_PGA_PRESSURE_GAIN_180},
	{200, SENSOR_MAX40109_PGA_PRESSURE_GAIN_200},
	{252, SENSOR_MAX40109_PGA_PRESSURE_GAIN_252},
	{540, SENSOR_MAX40109_PGA_PRESSURE_GAIN_540},
	{1080, SENSOR_MAX40109_PGA_PRESSURE_GAIN_1080},
	{1440, SENSOR_MAX40109_PGA_PRESSURE_GAIN_1440},
	{2520, SENSOR_MAX40109_PGA_PRESSURE_GAIN_2520},
};

static const struct sampling_rate_entry sampling_rate_table[] = {
	{1000, 1, SENSOR_SAMPLING_RATE_MAX40109_1000HZ_1HZ},
	{1000, 10, SENSOR_SAMPLING_RATE_MAX40109_1000HZ_10HZ},
	{2000, 1, SENSOR_SAMPLING_RATE_MAX40109_2000HZ_1HZ},
	{2000, 10, SENSOR_SAMPLING_RATE_MAX40109_2000HZ_10HZ},
	{4000, 1, SENSOR_SAMPLING_RATE_MAX40109_4000HZ_1HZ},
	{4000, 10, SENSOR_SAMPLING_RATE_MAX40109_4000HZ_10HZ},
	{8000, 1, SENSOR_SAMPLING_RATE_MAX40109_8000HZ_1HZ},
	{8000, 10, SENSOR_SAMPLING_RATE_MAX40109_8000HZ_10HZ},
	{16000, 1, SENSOR_SAMPLING_RATE_MAX40109_16000HZ_1HZ},
	{16000, 10, SENSOR_SAMPLING_RATE_MAX40109_16000HZ_10HZ},
};

static const struct drv_scale_entry drv_scale_table[] = {
	{1, 0, MAX40109_DRV_SCALE_1_0},
	{0, 500000, MAX40109_DRV_SCALE_0_5},
	{0, 375000, MAX40109_DRV_SCALE_0_375},
	{0, 250000, MAX40109_DRV_SCALE_0_25},
};

static const struct calib_map calib_map_table[] = {
	{SENSOR_CHAN_AMBIENT_TEMP_CALIB_COEFF_K0, MAX40109_CALIBRATION_K0},
	{SENSOR_CHAN_AMBIENT_TEMP_CALIB_COEFF_K1, MAX40109_CALIBRATION_K1},
	{SENSOR_CHAN_AMBIENT_TEMP_CALIB_COEFF_K2, MAX40109_CALIBRATION_K2},
	{SENSOR_CHAN_AMBIENT_TEMP_CALIB_COEFF_K3, MAX40109_CALIBRATION_K3},
	{SENSOR_CHAN_PRESSURE_CALIB_COEFF_H0, MAX40109_CALIBRATION_H0},
	{SENSOR_CHAN_PRESSURE_CALIB_COEFF_H1, MAX40109_CALIBRATION_H1},
	{SENSOR_CHAN_PRESSURE_CALIB_COEFF_H2, MAX40109_CALIBRATION_H2},
	{SENSOR_CHAN_PRESSURE_CALIB_COEFF_H3, MAX40109_CALIBRATION_H3},
	{SENSOR_CHAN_PRESSURE_CALIB_COEFF_G0, MAX40109_CALIBRATION_G0},
	{SENSOR_CHAN_PRESSURE_CALIB_COEFF_G1, MAX40109_CALIBRATION_G1},
	{SENSOR_CHAN_PRESSURE_CALIB_COEFF_G2, MAX40109_CALIBRATION_G2},
	{SENSOR_CHAN_PRESSURE_CALIB_COEFF_G3, MAX40109_CALIBRATION_G3},
	{SENSOR_CHAN_PRESSURE_CALIB_COEFF_N0, MAX40109_CALIBRATION_N0},
	{SENSOR_CHAN_PRESSURE_CALIB_COEFF_N1, MAX40109_CALIBRATION_N1},
	{SENSOR_CHAN_PRESSURE_CALIB_COEFF_N2, MAX40109_CALIBRATION_N2},
	{SENSOR_CHAN_PRESSURE_CALIB_COEFF_N3, MAX40109_CALIBRATION_N3},
	{SENSOR_CHAN_PRESSURE_CALIB_COEFF_M0, MAX40109_CALIBRATION_M0},
	{SENSOR_CHAN_PRESSURE_CALIB_COEFF_M1, MAX40109_CALIBRATION_M1},
	{SENSOR_CHAN_PRESSURE_CALIB_COEFF_M2, MAX40109_CALIBRATION_M2},
	{SENSOR_CHAN_PRESSURE_CALIB_COEFF_M3, MAX40109_CALIBRATION_M3},
};

static const struct analog_bw_map bw_table[] = {
	{1200, MAX40109_ANALOG_FILTER_BW_1200HZ},
	{900, MAX40109_ANALOG_FILTER_BW_900HZ},
	{37000, MAX40109_ANALOG_FILTER_BW_37000HZ},
	{45000, MAX40109_ANALOG_FILTER_BW_45000HZ},
};

static const struct digital_filter_map filter_table[] = {
	{0, MAX40109_DIGITAL_FILTER_SETUP_AVERAGE_NONE},
	{4, MAX40109_DIGITAL_FILTER_SETUP_AVERAGE_4},
	{8, MAX40109_DIGITAL_FILTER_SETUP_AVERAGE_8},
	{16, MAX40109_DIGITAL_FILTER_SETUP_AVERAGE_16},
	{32, MAX40109_DIGITAL_FILTER_SETUP_AVERAGE_32},
	{64, MAX40109_DIGITAL_FILTER_SETUP_AVERAGE_64},
	{128, MAX40109_DIGITAL_FILTER_SETUP_AVERAGE_128},
};

static int max40109_set_sampling_rate(const struct device *dev, int pressure_rate,
				      int temperature_rate)
{
	for (size_t i = 0; i < ARRAY_SIZE(sampling_rate_table); i++) {
		if (sampling_rate_table[i].pressure_rate == pressure_rate &&
		    sampling_rate_table[i].temperature_rate == temperature_rate) {
			return max40109_reg_write(dev, MAX40109_ADC_SAMPLE_RATE,
						  sampling_rate_table[i].reg_value);
		}
	}
	return -EINVAL;
}

static int max40109_set_pressure_gain(const struct device *dev, int val1)
{
	for (size_t i = 0; i < ARRAY_SIZE(pressure_gain_table); i++) {
		if (pressure_gain_table[i].val1 == val1) {
			return max40109_reg_write(dev, MAX40109_PGA_PRESSURE_GAIN,
						  pressure_gain_table[i].reg_value);
		}
	}
	return -EINVAL;
}

static int max40109_set_temperature_gain(const struct device *dev, int val1, int val2)
{
	for (size_t i = 0; i < ARRAY_SIZE(temp_gain_table); i++) {
		if (temp_gain_table[i].val1 == val1 && temp_gain_table[i].val2 == val2) {
			return max40109_reg_write(dev, MAX40109_PGA_TEMPERATURE_GAIN,
						  temp_gain_table[i].reg_value);
		}
	}
	return -EINVAL;
}

static int max40109_set_alert_mode(const struct device *dev, uint8_t mode)
{
	if (mode < MAX40109_ALERT_ISSUES_INTERRUPT ||
	    mode > MAX40109_ALERT_ISSUES_PRESSURE_DIGITAL_OUTPUT_CASE_4) {
		return -EINVAL; // Invalid alert mode
	}
	return max40109_reg_update(dev, MAX40109_REG_CONFIG_LSB, ALERT_MODE_MASK, mode);
}

static int max40109_set_drv_scale(const struct device *dev, int val1, int val2)
{
	for (size_t i = 0; i < ARRAY_SIZE(drv_scale_table); i++) {
		if (drv_scale_table[i].val1 == val1 && drv_scale_table[i].val2 == val2) {
			return max40109_reg_update(dev, MAX40109_TEMP_MODE, DRV_SCALE_MASK,
						   drv_scale_table[i].reg_value);
		}
	}
	return -EINVAL;
}

static int set_analog_filter_bw(struct device *dev, int analog_filter_bw)
{
	for (size_t i = 0; i < sizeof(bw_table) / sizeof(bw_table[0]); i++) {
		if (bw_table[i].bw == analog_filter_bw) {
			return max40109_reg_write(dev, MAX40109_ANALOG_FILTER_BW,
						  bw_table[i].reg_val);
		}
	}
	return -EINVAL; // Invalid bandwidth value
}

static int set_digital_filter_setup(struct device *dev, int digital_filter_setup)
{
	for (size_t i = 0; i < sizeof(filter_table) / sizeof(filter_table[0]); i++) {
		if (filter_table[i].setup == digital_filter_setup) {
			return max40109_reg_update(dev, MAX40109_REG_CONFIG_MSB,
						   DIGITAL_FILTER_MASK, filter_table[i].reg_val);
		}
	}
	return -EINVAL; // Invalid digital filter setup value
}

int max40109_set_calibration_coeff(const struct device *dev, enum sensor_channel chan,
				   const struct sensor_value *val)
{
	uint8_t coeff = 0;
	bool found = false;

	for (size_t i = 0; i < ARRAY_SIZE(calib_map_table); i++) {
		if (calib_map_table[i].chan == chan) {
			coeff = calib_map_table[i].coeff;
			found = true;
			break;
		}
	}

	if (!found) {
		return -ENOTSUP; // Unsupported calibration channel
	}

	float calibration_value = val->val1 + (val->val2 / 1000000.0f);
	int ret = max40109_mtp_calibration(dev, coeff, calibration_value, false);
	if (ret < 0) {
		return ret; // Error applying calibration
	}

	return 0; // Success
}

int max40109_mtp_calibration_read(const struct device *dev,
				  enum max40109_calibration_coefficients coeff, float *value)
{
	const struct max40109_config *config = dev->config;
	uint8_t mtp_addr = 0;
	uint8_t buf_msb[2] = {0, 0};
	uint8_t buf_lsb[2] = {0, 0};
	int ret = 0;

	if (coeff < MAX40109_CALIBRATION_K0 || coeff > MAX40109_CALIBRATION_M3) {
		LOG_ERR("Invalid calibration coefficient");
		return -EINVAL; // Invalid coefficient
	}

	ret = get_calibration_address(coeff, &mtp_addr);
	if (ret < 0) {
		LOG_ERR("Failed to get MTP address for coefficient %d", coeff);
		return ret; // Error getting MTP address
	}

	ret = max40109_mtp_read(dev, mtp_addr, buf_lsb);
	if (ret < 0) {
		LOG_ERR("Failed to read MTP data for coefficient %d", coeff);
		return ret; // Error reading MTP data
	}
	ret = max40109_mtp_read(dev, mtp_addr + 1, buf_msb);
	if (ret < 0) {
		LOG_ERR("Failed to read MTP data for coefficient %d", coeff);
		return ret; // Error reading MTP data
	}
	uint32_t hex = (buf_msb[0] << 24) | (buf_msb[1] << 16) | (buf_lsb[0] << 8) | buf_lsb[1];

	*value = *((float *)&hex);
	return 0; // Success
}

int max40109_mtp_burn(const struct device *dev, uint8_t mtp_addr, const uint8_t *val)
{
	const struct max40109_config *config = dev->config;
	int ret = 0;

	ret = max40109_mtp_initialize(dev);
	if (ret < 0) {
		return ret; // Error initializing MTP
	}

	ret = max40109_reg_write(dev, MAX40109_MTP_ADDR, mtp_addr);

	if (ret) {
		return ret;
	}

	ret = max40109_reg_write_multiple(dev, MAX40109_MTP_DATA0_MSB, val,
					  MAX40109_MTP_DATA_LENGTH);
	if (ret < 0) {
		return ret; // Error writing MTP data
	}

	ret = max40109_reg_write(dev, MAX40109_MTP_CONTROL, 0x01);

	if (ret) {
		return ret; // Error writing MTP control register
	}

	ret = max40109_is_burn_successful(dev);
	if (ret < 0) {
		LOG_ERR("MTP burn failed, check MTP status");
		return -EIO; // MTP burn not successful
	}

	LOG_INF("MTP burn successful");
	return 0;
}

static int max40109_init(const struct device *dev)
{
	const struct max40109_config *config = dev->config;

	if (!i2c_is_ready_dt(&config->i2c)) {
		return -ENODEV;
	}

	/* Reset the STATUS Register */
	int ret = 0;
	uint8_t status_reg[2];

	ret = max40109_reg_read(dev, MAX40109_REG_STATUS_MSB, status_reg, 2);
	if (ret < 0) {
		return ret;
	}

	ret = max40109_reg_write_multiple(dev, MAX40109_REG_STATUS_MSB, status_reg, 2);
	if (ret < 0) {
		return ret;
	}

#ifdef CONFIG_MAX40109_TRIGGER
	if (config->interrupt_gpio.port) {
		if (max40109_init_interrupt(dev) < 0) {
			LOG_ERR("Failed to initialize interrupts");
			return -EIO;
		}
	}
#endif

	return 0;
}

static int max40109_attr_set(const struct device *dev, enum sensor_channel chan,
			     enum sensor_attribute attr, const struct sensor_value *val)
{
	const struct max40109_config *config = dev->config;
	struct max40109_data *data = dev->data;

	int ret = 0;

	switch (attr) {
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		if (val->val1 < 0) {
			return -EINVAL;
		}

		int pressure_rate = val->val1;
		if (pressure_rate < 1000 || pressure_rate > 16000) {
			return -EINVAL; // Invalid pressure rate value
		}

		int temperature_rate = val->val2;
		if (temperature_rate < 1 || temperature_rate > 10) {
			return -EINVAL; // Invalid temperature rate value
		}

		ret = max40109_set_sampling_rate(dev, pressure_rate, temperature_rate);
		break;
	case SENSOR_ATTR_GAIN:
		if (chan == SENSOR_CHAN_PRESS) {
			int pressure_gain = val->val1;
			if (pressure_gain < 5) {
				return -EINVAL; // Invalid gain value
			}
			ret = max40109_set_pressure_gain(dev, pressure_gain);
		}
		if (chan == SENSOR_CHAN_AMBIENT_TEMP) {
			int temperature_gain = val->val1;
			int temperature_gain_decimal = val->val2 / 1000000;

			if (temperature_gain < 1) {
				return -EINVAL;
			}
			ret = max40109_set_temperature_gain(dev, temperature_gain,
							    temperature_gain_decimal);
			if (ret < 0) {
				return ret;
			}
		}
		break;

	case MAX40109_ANALOG_FILTER_BW_SETUP:
		if (val->val1 < 0) {
			return -EINVAL; // Invalid bandwidth value
		}

		int analog_filter_bw = val->val1;

		ret = set_analog_filter_bw((struct device *)dev, analog_filter_bw);
		if (ret < 0) {
			return ret;
		}
		break;

	case MAX40109_DIGITAL_FILTER_SETUP:
		if (val->val1 < 0) {
			return -EINVAL; // Invalid digital filter setup value
		}
		int digital_filter_setup = val->val1;

		ret = set_digital_filter_setup((struct device *)dev, digital_filter_setup);
		if (ret < 0) {
			return ret;
		}
		break;

	case MAX40109_ALERT_MODE:
		if (val->val1 < 0) {
			return -EINVAL; // Invalid alert mode value
		}
		int alert_mode = val->val1;

		ret = max40109_set_alert_mode(dev, alert_mode);
		if (ret < 0) {
			return ret;
		}
		break;

	case MAX40109_DRV_SCALE:
		if (val == NULL) {
			return -EINVAL; // Invalid value
		}
		if (val->val1 < 0 && val->val2 == 0) {
			return -EINVAL; // Invalid scale value
		}
		if (val->val1 == 1 && val->val2 != 0) {
			return -EINVAL; // Scale value must be 1.0
		}

		ret = max40109_set_drv_scale(dev, val->val1, val->val2);
		if (ret < 0) {
			return ret;
		}

		break;

	case SENSOR_ATTR_CALIB_TARGET:

		ret = max40109_set_calibration_coeff(dev, chan, val);
		if (ret < 0) {
			return ret; // Error applying calibration
		}

		break;

	case MAX40109_OVER_PRESSURE_THRESHOLD_POSITIVE:

		if (chan != SENSOR_CHAN_PRESS) {
			return -ENOTSUP;
		}

		if (val->val1 < 0 || val->val1 > 0xFF) {
			return -EINVAL; // Invalid threshold value
		}
		uint16_t threshold_over_pos = val->val1;

		ret = max40109_mtp_update(dev, DIAG_DATA0, OVER_PRESSURE_POSITIVE_MASK,
					  threshold_over_pos);
		if (ret < 0) {
			return ret; // Error setting over-pressure positive threshold
		}
		break;

	case MAX40109_UNDER_PRESSURE_THRESHOLD_POSITIVE:
		if (chan != SENSOR_CHAN_PRESS) {
			return -ENOTSUP;
		}

		if (val->val1 < 0 || val->val1 > 0xFF) {
			return -EINVAL; // Invalid threshold value
		}
		uint16_t threshold_under_pos = val->val1;

		ret = max40109_mtp_update(dev, DIAG_DATA0, UNDER_PRESSURE_POSITIVE_MASK,
					  threshold_under_pos);
		if (ret < 0) {
			return ret; // Error setting under-pressure threshold
		}
		break;

	case MAX40109_OVER_PRESSURE_THRESHOLD_NEGATIVE:

		if (chan != SENSOR_CHAN_PRESS) {
			return -ENOTSUP;
		}

		if (val->val1 < 0 || val->val1 > 0xFF) {
			return -EINVAL; // Invalid threshold value
		}
		uint16_t threshold_over_neg = val->val1;

		ret = max40109_mtp_update(dev, DIAG_DATA1, OVER_PRESSURE_NEGATIVE_MASK,
					  threshold_over_neg);
		if (ret < 0) {
			return ret; // Error setting over-pressure negative threshold
		}
		break;

	case MAX40109_UNDER_PRESSURE_THRESHOLD_NEGATIVE:

		if (chan != SENSOR_CHAN_PRESS) {
			return -ENOTSUP;
		}

		if (val->val1 < 0 || val->val1 > 0xFF) {
			return -EINVAL; // Invalid threshold value
		}
		uint16_t threshold_under_neg = val->val1;

		ret = max40109_mtp_update(dev, DIAG_DATA1, UNDER_PRESSURE_NEGATIVE_MASK,
					  threshold_under_neg);
		if (ret < 0) {
			return ret; // Error setting under-pressure negative threshold
		}
		break;

	case MAX40109_OVER_TEMPERATURE_VOLTAGE_THRESHOLD:
		if (chan != SENSOR_CHAN_AMBIENT_TEMP) {
			return -ENOTSUP;
		}
		if (val->val1 < 0 || val->val1 > 0xFF) {
			return -EINVAL; // Invalid threshold value
		}
		uint16_t threshold_over_temp = val->val1;
		ret = max40109_mtp_update(dev, DIAG_DATA2, OVER_TEMPERATURE_VOLTAGE_MASK,
					  threshold_over_temp);
		if (ret < 0) {
			return ret; // Error setting over-temperature threshold
		}

		break;

	case MAX40109_UNDER_TEMPERATURE_VOLTAGE_THRESHOLD:
		if (chan != SENSOR_CHAN_AMBIENT_TEMP) {
			return -ENOTSUP;
		}
		if (val->val1 < 0 || val->val1 > 0xFF) {
			return -EINVAL; // Invalid threshold value
		}
		uint16_t threshold_under_temp = val->val1;
		ret = max40109_mtp_update(dev, DIAG_DATA2, UNDER_TEMPERATURE_VOLTAGE_MASK,
					  threshold_under_temp);
		if (ret < 0) {
			return ret; // Error setting under-temperature threshold
		}

		break;

	case MAX40109_OVER_VOLTAGE_DRIVE_THRESHOLD:
		if (val->val1 < 0 || val->val1 > 0xFF) {
			return -EINVAL; // Invalid threshold value
		}

		uint16_t threshold_over_drv = val->val1;
		ret = max40109_mtp_update(dev, DIAG_DATA3, OVER_VOLTAGE_DRIVE_MASK,
					  threshold_over_drv);
		if (ret < 0) {
			return ret; // Error setting over-voltage drive threshold
		}
		break;

	case MAX40109_UNDER_VOLTAGE_DRIVE_THRESHOLD:
		if (val->val1 < 0 || val->val1 > 0xFF) {
			return -EINVAL; // Invalid threshold value
		}
		uint16_t threshold_under_drv = val->val1;
		ret = max40109_mtp_update(dev, DIAG_DATA3, UNDER_VOLTAGE_DRIVE_MASK,
					  threshold_under_drv);
		if (ret < 0) {
			return ret; // Error setting under-voltage drive threshold
		}

		break;

	case MAX40109_PRIMARY_THRESHOLD_PRESSURE_VALUE:
		if (chan != SENSOR_CHAN_PRESS) {
			return -ENOTSUP;
		}
		if (val->val1 < 0 || val->val1 > 0xFF) {
			return -EINVAL; // Invalid threshold value
		}
		uint16_t threshold_primary = val->val1;
		ret = max40109_mtp_update(dev, PRESSURE_THRESHOLD,
					  PRIMARY_THRESHOLD_PRESSURE_VALUE_MASK, threshold_primary);
		if (ret < 0) {
			return ret; // Error setting primary threshold pressure value
		}
		break;

	case MAX40109_HYSTERESIS_THRESHOLD_PRESSURE_VALUE:
		if (chan != SENSOR_CHAN_PRESS) {
			return -ENOTSUP;
		}
		if (val->val1 < 0 || val->val1 > 0xFF) {
			return -EINVAL; // Invalid threshold value
		}
		uint16_t threshold_hysteresis = val->val1;
		ret = max40109_mtp_update(dev, PRESSURE_THRESHOLD,
					  HYSTERESIS_THRESHOLD_PRESSURE_VALUE_MASK,
					  threshold_hysteresis);
		if (ret < 0) {
			return ret; // Error setting hysteresis threshold pressure value
		}

		break;

	default:
		ret = -ENOTSUP; // Unsupported attribute
		break;
	}

	return ret < 0 ? ret : 0;
}

static int max40109_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	const struct max40109_config *config = dev->config;
	struct max40109_data *data = dev->data;
	int ret;

	if (chan != SENSOR_CHAN_PRESS || chan != SENSOR_CHAN_AMBIENT_TEMP ||
	    chan != SENSOR_CHAN_ALL) {
		return -ENOTSUP; // Unsupported channel
	}

	uint8_t uncalibrated_data[2];
	uint8_t calibrated_data[2];

	if (chan == SENSOR_CHAN_PRESS || chan == SENSOR_CHAN_ALL) {
		ret = max40109_reg_read(dev, MAX40109_UNCALIBRATED_PRESSURE_MSB, uncalibrated_data,
					2);
		if (ret < 0) {
			return ret; // Error reading pressure data
		}
		data->uncalibrated_pressure = (uncalibrated_data[0] << 8) | uncalibrated_data[1];

		ret = max40109_reg_read(dev, MAX40109_CALIBRATED_PRESSURE_MSB, calibrated_data, 2);
		if (ret < 0) {
			return ret; // Error reading calibrated pressure data
		}
		data->calibrated_pressure = (calibrated_data[0] << 8) | calibrated_data[1];
	}

	else if (chan == SENSOR_CHAN_AMBIENT_TEMP || chan == SENSOR_CHAN_ALL) {
		ret = max40109_reg_read(dev, MAX40109_UNCALBIRATED_TEMPERATURE_MSB,
					uncalibrated_data, 2);
		if (ret < 0) {
			return ret; // Error reading temperature data
		}
		data->uncalibrated_temperature = (uncalibrated_data[0] << 8) | uncalibrated_data[1];

		ret = max40109_reg_read(dev, MAX40109_CALIBRATED_TEMPERATURE_MSB, calibrated_data,
					2);
		if (ret < 0) {
			return ret; // Error reading calibrated temperature data
		}
		data->calibrated_temperature = (calibrated_data[0] << 8) | calibrated_data[1];
	}

	return 0;
}

static DEVICE_API(sensor, max40109_driver_api) = {
	.attr_set = max40109_attr_set,
	// 	.attr_get = max40109_attr_get,
	.sample_fetch = max40109_sample_fetch,
// 	.channel_get = max40109_channel_get,
#ifdef CONFIG_MAX40109_TRIGGER
	.trigger_set = max40109_trigger_set,
#endif
};

#define MAX40109_CONFIG(inst)                                                                      \
	.digital_filter_setup = DT_INST_PROP(inst, digital_filter),                                \
	.alert_mode = DT_INST_PROP(inst, alert_mode),                                              \
	.temp_current = DT_INST_PROP(inst, temp_current),                                          \
	.pga_pressure_gain = DT_INST_PROP(inst, pga_pressure_gain),                                \
	.current_source = DT_INST_PROP(inst, current_source),                                      \
	.adc_sample_rate = DT_INST_PROP(inst, adc_sample_rate),                                    \
	.pga_temperature_gain = DT_INST_PROP(inst, pga_temperature_gain),                          \
	.bridge_drive = DT_INST_PROP(inst, bridge_drive),                                          \
	.temp_mode = DT_INST_PROP(inst, temp_mode), .drv_scale = DT_INST_PROP(inst, drv_scale),    \
	.analog_filter_bw_setup = DT_INST_PROP(inst, analog_filter_bw),                            \
	.analog_output_stage = DT_INST_PROP(inst, analog_output_stage),                            \
	.k0 = (float)DT_INST_PROP(inst, k0) / 1000.0f,                                             \
	.k1 = (float)DT_INST_PROP(inst, k1) / 1000.0f,                                             \
	.k2 = (float)DT_INST_PROP(inst, k2) / 1000.0f,                                             \
	.k3 = (float)DT_INST_PROP(inst, k3) / 1000.0f,                                             \
	.h0 = (float)DT_INST_PROP(inst, h0) / 1000.0f,                                             \
	.h1 = (float)DT_INST_PROP(inst, h1) / 1000.0f,                                             \
	.h2 = (float)DT_INST_PROP(inst, h2) / 1000.0f,                                             \
	.h3 = (float)DT_INST_PROP(inst, h3) / 1000.0f,                                             \
	.g0 = (float)DT_INST_PROP(inst, g0) / 1000.0f,                                             \
	.g1 = (float)DT_INST_PROP(inst, g1) / 1000.0f,                                             \
	.g2 = (float)DT_INST_PROP(inst, g2) / 1000.0f,                                             \
	.g3 = (float)DT_INST_PROP(inst, g3) / 1000.0f,                                             \
	.n0 = (float)DT_INST_PROP(inst, n0) / 1000.0f,                                             \
	.n1 = (float)DT_INST_PROP(inst, n1) / 1000.0f,                                             \
	.n2 = (float)DT_INST_PROP(inst, n2) / 1000.0f,                                             \
	.n3 = (float)DT_INST_PROP(inst, n3) / 1000.0f,                                             \
	.m0 = (float)DT_INST_PROP(inst, m0) / 1000.0f,                                             \
	.m1 = (float)DT_INST_PROP(inst, m1) / 1000.0f,                                             \
	.m2 = (float)DT_INST_PROP(inst, m2) / 1000.0f,                                             \
	.m3 = (float)DT_INST_PROP(inst, m3) / 1000.0f,                                             \
	.overpressure_threshold_positive = DT_INST_PROP(inst, overpressure_threshold_pos),         \
	.underpressure_threshold_positive = DT_INST_PROP(inst, underpressure_threshold_pos),       \
	.overpressure_threshold_negative = DT_INST_PROP(inst, overpressure_threshold_neg),         \
	.underpressure_threshold_negative = DT_INST_PROP(inst, underpressure_threshold_neg),       \
	.overvoltage_temperature = DT_INST_PROP(inst, overtemp_threshold),                         \
	.undervoltage_temperature = DT_INST_PROP(inst, undertemp_threshold),                       \
	.overvoltage_drive = DT_INST_PROP(inst, overvoltage_drv),                                  \
	.undervoltage_drive = DT_INST_PROP(inst, undervoltage_drv),                                \
	.primary_threshold_pressure_value = DT_INST_PROP(inst, primary_threshold_pressure_value),  \
	.hysteresis_threshold_pressure_value =                                                     \
		DT_INST_PROP(inst, hysteresis_threshold_pressure_value)

#define MAX40109_DEFINE(inst)                                                                      \
	static struct max40109_data max40109_prv_data_##inst;                                      \
	static const struct max40109_config max40109_config_##inst = {                             \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
		MAX40109_CONFIG(inst),                                                             \
		IF_ENABLED(CONFIG_MAX40109_TRIGGER, (.interrupt_gpio = GPIO_DT_SPEC_INST_GET_OR(        \
			inst, interrupt_gpios, {0}),)) };        \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, &max40109_init, NULL, &max40109_prv_data_##inst,        \
				     &max40109_config_##inst, POST_KERNEL,                         \
				     CONFIG_SENSOR_INIT_PRIORITY, &max40109_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MAX40109_DEFINE)
