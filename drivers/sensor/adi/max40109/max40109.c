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

	switch (coeff) {
	case MAX40109_CALIBRATION_K0:
		mtp_addr = CAL_DATA0;
		break;
	case MAX40109_CALIBRATION_K1:
		mtp_addr = CAL_DATA2;
		break;
	case MAX40109_CALIBRATION_K2:
		mtp_addr = CAL_DATA4;
		break;
	case MAX40109_CALIBRATION_K3:
		mtp_addr = CAL_DATA6;
		break;
	case MAX40109_CALIBRATION_H0:
		mtp_addr = CAL_DATA8;
		break;
	case MAX40109_CALIBRATION_H1:
		mtp_addr = CAL_DATA10;
		break;
	case MAX40109_CALIBRATION_H2:
		mtp_addr = CAL_DATA12;
		break;
	case MAX40109_CALIBRATION_H3:
		mtp_addr = CAL_DATA14;
		break;
	case MAX40109_CALIBRATION_G0:
		mtp_addr = CAL_DATA16;
		break;
	case MAX40109_CALIBRATION_G1:
		mtp_addr = CAL_DATA18;
		break;
	case MAX40109_CALIBRATION_G2:
		mtp_addr = CAL_DATA20;
		break;
	case MAX40109_CALIBRATION_G3:
		mtp_addr = CAL_DATA22;
		break;
	case MAX40109_CALIBRATION_N0:
		mtp_addr = CAL_DATA24;
		break;
	case MAX40109_CALIBRATION_N1:
		mtp_addr = CAL_DATA26;
		break;
	case MAX40109_CALIBRATION_N2:
		mtp_addr = CAL_DATA28;
		break;
	case MAX40109_CALIBRATION_N3:
		mtp_addr = CAL_DATA30;
		break;
	case MAX40109_CALIBRATION_M0:
		mtp_addr = CAL_DATA32;
		break;
	case MAX40109_CALIBRATION_M1:
		mtp_addr = CAL_DATA34;
		break;
	case MAX40109_CALIBRATION_M2:
		mtp_addr = CAL_DATA36;
		break;
	case MAX40109_CALIBRATION_M3:
		mtp_addr = CAL_DATA38;
		break;
	default:
		LOG_ERR("Invalid calibration coefficient");
		return -EINVAL; // Invalid coefficient
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

	switch (coeff) {
	case MAX40109_CALIBRATION_K0:
		mtp_addr = CAL_DATA0;
		break;
	case MAX40109_CALIBRATION_K1:
		mtp_addr = CAL_DATA2;
		break;
	case MAX40109_CALIBRATION_K2:
		mtp_addr = CAL_DATA4;
		break;
	case MAX40109_CALIBRATION_K3:
		mtp_addr = CAL_DATA6;
		break;
	case MAX40109_CALIBRATION_H0:
		mtp_addr = CAL_DATA8;
		break;
	case MAX40109_CALIBRATION_H1:
		mtp_addr = CAL_DATA10;
		break;
	case MAX40109_CALIBRATION_H2:
		mtp_addr = CAL_DATA12;
		break;
	case MAX40109_CALIBRATION_H3:
		mtp_addr = CAL_DATA14;
		break;
	case MAX40109_CALIBRATION_G0:
		mtp_addr = CAL_DATA16;
		break;
	case MAX40109_CALIBRATION_G1:
		mtp_addr = CAL_DATA18;
		break;
	case MAX40109_CALIBRATION_G2:
		mtp_addr = CAL_DATA20;
		break;
	case MAX40109_CALIBRATION_G3:
		mtp_addr = CAL_DATA22;
		break;
	case MAX40109_CALIBRATION_N0:
		mtp_addr = CAL_DATA24;
		break;
	case MAX40109_CALIBRATION_N1:
		mtp_addr = CAL_DATA26;
		break;
	case MAX40109_CALIBRATION_N2:
		mtp_addr = CAL_DATA28;
		break;
	case MAX40109_CALIBRATION_N3:
		mtp_addr = CAL_DATA30;
		break;
	case MAX40109_CALIBRATION_M0:
		mtp_addr = CAL_DATA32;
		break;
	case MAX40109_CALIBRATION_M1:
		mtp_addr = CAL_DATA34;
		break;
	case MAX40109_CALIBRATION_M2:
		mtp_addr = CAL_DATA36;
		break;
	case MAX40109_CALIBRATION_M3:
		mtp_addr = CAL_DATA38;
		break;
	default:
		LOG_ERR("Invalid calibration coefficient");
		return -EINVAL; // Invalid coefficient
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

		if (pressure_rate == 1000 && temperature_rate == 1) {
			ret = max40109_reg_write(dev, MAX40109_ADC_SAMPLE_RATE,
						 SENSOR_SAMPLING_RATE_MAX40109_1000HZ_1HZ);
		}

		if (pressure_rate == 1000 && temperature_rate == 10) {
			ret = max40109_reg_write(dev, MAX40109_ADC_SAMPLE_RATE,
						 SENSOR_SAMPLING_RATE_MAX40109_1000HZ_10HZ);
		}

		else if (pressure_rate == 2000 && temperature_rate == 1) {
			ret = max40109_reg_write(dev, MAX40109_ADC_SAMPLE_RATE,
						 SENSOR_SAMPLING_RATE_MAX40109_2000HZ_1HZ);
		}

		else if (pressure_rate == 2000 && temperature_rate == 10) {
			ret = max40109_reg_write(dev, MAX40109_ADC_SAMPLE_RATE,
						 SENSOR_SAMPLING_RATE_MAX40109_2000HZ_10HZ);
		}

		else if (pressure_rate == 4000 && temperature_rate == 1) {
			ret = max40109_reg_write(dev, MAX40109_ADC_SAMPLE_RATE,
						 SENSOR_SAMPLING_RATE_MAX40109_4000HZ_1HZ);
		}

		else if (pressure_rate == 4000 && temperature_rate == 10) {
			ret = max40109_reg_write(dev, MAX40109_ADC_SAMPLE_RATE,
						 SENSOR_SAMPLING_RATE_MAX40109_4000HZ_10HZ);
		}

		else if (pressure_rate == 8000 && temperature_rate == 1) {
			ret = max40109_reg_write(dev, MAX40109_ADC_SAMPLE_RATE,
						 SENSOR_SAMPLING_RATE_MAX40109_8000HZ_1HZ);
		}

		else if (pressure_rate == 8000 && temperature_rate == 10) {
			ret = max40109_reg_write(dev, MAX40109_ADC_SAMPLE_RATE,
						 SENSOR_SAMPLING_RATE_MAX40109_8000HZ_10HZ);
		}

		else if (pressure_rate == 16000 && temperature_rate == 1) {
			ret = max40109_reg_write(dev, MAX40109_ADC_SAMPLE_RATE,
						 SENSOR_SAMPLING_RATE_MAX40109_16000HZ_1HZ);
		}

		else if (pressure_rate == 16000 && temperature_rate == 10) {
			ret = max40109_reg_write(dev, MAX40109_ADC_SAMPLE_RATE,
						 SENSOR_SAMPLING_RATE_MAX40109_16000HZ_10HZ);
		}

		else {
			return -EINVAL;
		}
		break;
	case SENSOR_ATTR_GAIN:
		if (chan == SENSOR_CHAN_PRESS) {
			int pressure_gain = val->val1;
			if (pressure_gain < 5) {
				return -EINVAL; // Invalid gain value
			}

			if (pressure_gain == 5) {
				ret = max40109_reg_write(dev, MAX40109_PGA_PRESSURE_GAIN,
							 SENSOR_MAX40109_PGA_PRESSURE_GAIN_5);
			}

			else if (pressure_gain == 10) {
				ret = max40109_reg_write(dev, MAX40109_PGA_PRESSURE_GAIN,
							 SENSOR_MAX40109_PGA_PRESSURE_GAIN_10);
			}

			else if (pressure_gain == 15) {
				ret = max40109_reg_write(dev, MAX40109_PGA_PRESSURE_GAIN,
							 SENSOR_MAX40109_PGA_PRESSURE_GAIN_15);
			}

			else if (pressure_gain == 20) {
				ret = max40109_reg_write(dev, MAX40109_PGA_PRESSURE_GAIN,
							 SENSOR_MAX40109_PGA_PRESSURE_GAIN_20);
			}

			else if (pressure_gain == 24) {
				ret = max40109_reg_write(dev, MAX40109_PGA_PRESSURE_GAIN,
							 SENSOR_MAX40109_PGA_PRESSURE_GAIN_24);
			}

			else if (pressure_gain == 40) {
				ret = max40109_reg_write(dev, MAX40109_PGA_PRESSURE_GAIN,
							 SENSOR_MAX40109_PGA_PRESSURE_GAIN_40);
			}

			else if (pressure_gain == 60) {
				ret = max40109_reg_write(dev, MAX40109_PGA_PRESSURE_GAIN,
							 SENSOR_MAX40109_PGA_PRESSURE_GAIN_60);
			}

			else if (pressure_gain == 72) {
				ret = max40109_reg_write(dev, MAX40109_PGA_PRESSURE_GAIN,
							 SENSOR_MAX40109_PGA_PRESSURE_GAIN_72);
			}

			else if (pressure_gain == 90) {
				ret = max40109_reg_write(dev, MAX40109_PGA_PRESSURE_GAIN,
							 SENSOR_MAX40109_PGA_PRESSURE_GAIN_90);
			}

			else if (pressure_gain == 108) {
				ret = max40109_reg_write(dev, MAX40109_PGA_PRESSURE_GAIN,
							 SENSOR_MAX40109_PGA_PRESSURE_GAIN_108);
			}

			else if (pressure_gain == 126) {
				ret = max40109_reg_write(dev, MAX40109_PGA_PRESSURE_GAIN,
							 SENSOR_MAX40109_PGA_PRESSURE_GAIN_126);
			}

			else if (pressure_gain == 144) {
				ret = max40109_reg_write(dev, MAX40109_PGA_PRESSURE_GAIN,
							 SENSOR_MAX40109_PGA_PRESSURE_GAIN_144);
			}

			else if (pressure_gain == 160) {
				ret = max40109_reg_write(dev, MAX40109_PGA_PRESSURE_GAIN,
							 SENSOR_MAX40109_PGA_PRESSURE_GAIN_160);
			}

			else if (pressure_gain == 180) {
				ret = max40109_reg_write(dev, MAX40109_PGA_PRESSURE_GAIN,
							 SENSOR_MAX40109_PGA_PRESSURE_GAIN_180);
			}

			else if (pressure_gain == 200) {
				ret = max40109_reg_write(dev, MAX40109_PGA_PRESSURE_GAIN,
							 SENSOR_MAX40109_PGA_PRESSURE_GAIN_200);
			}

			else if (pressure_gain == 252) {
				ret = max40109_reg_write(dev, MAX40109_PGA_PRESSURE_GAIN,
							 SENSOR_MAX40109_PGA_PRESSURE_GAIN_252);
			}

			else if (pressure_gain == 540) {
				ret = max40109_reg_write(dev, MAX40109_PGA_PRESSURE_GAIN,
							 SENSOR_MAX40109_PGA_PRESSURE_GAIN_540);
			}

			else if (pressure_gain == 1080) {
				ret = max40109_reg_write(dev, MAX40109_PGA_PRESSURE_GAIN,
							 SENSOR_MAX40109_PGA_PRESSURE_GAIN_1080);
			}

			else if (pressure_gain == 1440) {
				ret = max40109_reg_write(dev, MAX40109_PGA_PRESSURE_GAIN,
							 SENSOR_MAX40109_PGA_PRESSURE_GAIN_1440);
			}

			else if (pressure_gain == 2520) {
				ret = max40109_reg_write(dev, MAX40109_PGA_PRESSURE_GAIN,
							 SENSOR_MAX40109_PGA_PRESSURE_GAIN_2520);
			}

			else {
				return -EINVAL; // Invalid pressure gain value
			}
		}
		if (chan == SENSOR_CHAN_AMBIENT_TEMP) {
			int temperature_gain = val->val1;
			int temperature_gain_decimal = val->val2;

			if (temperature_gain < 1) {
				return -EINVAL;
			}

			if (temperature_gain == 1 && temperature_gain_decimal == 5) {
				ret = max40109_reg_write(dev, MAX40109_PGA_TEMPERATURE_GAIN,
							 SENSOR_MAX40109_PGA_TEMPERATURE_GAIN_1_5);
			}

			else if (temperature_gain == 2 && temperature_gain_decimal == 0) {
				ret = max40109_reg_write(dev, MAX40109_PGA_TEMPERATURE_GAIN,
							 SENSOR_MAX40109_PGA_TEMPERATURE_GAIN_2);
			}

			else if (temperature_gain == 3 && temperature_gain_decimal == 0) {
				ret = max40109_reg_write(dev, MAX40109_PGA_TEMPERATURE_GAIN,
							 SENSOR_MAX40109_PGA_TEMPERATURE_GAIN_3);
			}

			else if (temperature_gain == 5 && temperature_gain_decimal == 0) {
				ret = max40109_reg_write(dev, MAX40109_PGA_TEMPERATURE_GAIN,
							 SENSOR_MAX40109_PGA_TEMPERATURE_GAIN_5);
			}

			else if (temperature_gain == 6 && temperature_gain_decimal == 0) {
				ret = max40109_reg_write(dev, MAX40109_PGA_TEMPERATURE_GAIN,
							 SENSOR_MAX40109_PGA_TEMPERATURE_GAIN_6);
			}

			else if (temperature_gain == 10 && temperature_gain_decimal == 0) {
				ret = max40109_reg_write(dev, MAX40109_PGA_TEMPERATURE_GAIN,
							 SENSOR_MAX40109_PGA_TEMPERATURE_GAIN_10);
			}

			else if (temperature_gain == 15 && temperature_gain_decimal == 0) {
				ret = max40109_reg_write(dev, MAX40109_PGA_TEMPERATURE_GAIN,
							 SENSOR_MAX40109_PGA_TEMPERATURE_GAIN_15);
			}

			else if (temperature_gain == 20 && temperature_gain_decimal == 0) {
				ret = max40109_reg_write(dev, MAX40109_PGA_TEMPERATURE_GAIN,
							 SENSOR_MAX40109_PGA_TEMPERATURE_GAIN_20);
			}

			else if (temperature_gain == 24 && temperature_gain_decimal == 0) {
				ret = max40109_reg_write(dev, MAX40109_PGA_TEMPERATURE_GAIN,
							 SENSOR_MAX40109_PGA_TEMPERATURE_GAIN_24);
			}

			else if (temperature_gain == 30 && temperature_gain_decimal == 0) {
				ret = max40109_reg_write(dev, MAX40109_PGA_TEMPERATURE_GAIN,
							 SENSOR_MAX40109_PGA_TEMPERATURE_GAIN_30);
			}

			else if (temperature_gain == 36 && temperature_gain_decimal == 0) {
				ret = max40109_reg_write(dev, MAX40109_PGA_TEMPERATURE_GAIN,
							 SENSOR_MAX40109_PGA_TEMPERATURE_GAIN_36);
			}

			else if (temperature_gain == 40 && temperature_gain_decimal == 0) {
				ret = max40109_reg_write(dev, MAX40109_PGA_TEMPERATURE_GAIN,
							 SENSOR_MAX40109_PGA_TEMPERATURE_GAIN_40);
			}

			else if (temperature_gain == 45 && temperature_gain_decimal == 0) {
				ret = max40109_reg_write(dev, MAX40109_PGA_TEMPERATURE_GAIN,
							 SENSOR_MAX40109_PGA_TEMPERATURE_GAIN_45);
			}

			else if (temperature_gain == 60 && temperature_gain_decimal == 0) {
				ret = max40109_reg_write(dev, MAX40109_PGA_TEMPERATURE_GAIN,
							 SENSOR_MAX40109_PGA_TEMPERATURE_GAIN_60);
			}

			else if (temperature_gain == 72 && temperature_gain_decimal == 0) {
				ret = max40109_reg_write(dev, MAX40109_PGA_TEMPERATURE_GAIN,
							 SENSOR_MAX40109_PGA_TEMPERATURE_GAIN_72);
			}

			else if (temperature_gain == 90 && temperature_gain_decimal == 0) {
				ret = max40109_reg_write(dev, MAX40109_PGA_TEMPERATURE_GAIN,
							 SENSOR_MAX40109_PGA_TEMPERATURE_GAIN_90);
			}

			else {
				return -EINVAL; // Invalid temperature gain value
			}
		}
		break;

	case MAX40109_ANALOG_FILTER_BW_SETUP:
		if (val->val1 < 0) {
			return -EINVAL; // Invalid bandwidth value
		}

		int analog_filter_bw = val->val1;

		if (analog_filter_bw == 1200) {
			ret = max40109_reg_write(dev, MAX40109_ANALOG_FILTER_BW,
						 MAX40109_ANALOG_FILTER_BW_1200HZ);
		} else if (analog_filter_bw == 900) {
			ret = max40109_reg_write(dev, MAX40109_ANALOG_FILTER_BW,
						 MAX40109_ANALOG_FILTER_BW_900HZ);
		} else if (analog_filter_bw == 37000) {
			ret = max40109_reg_write(dev, MAX40109_ANALOG_FILTER_BW,
						 MAX40109_ANALOG_FILTER_BW_37000HZ);
		} else if (analog_filter_bw == 45000) {
			ret = max40109_reg_write(dev, MAX40109_ANALOG_FILTER_BW,
						 MAX40109_ANALOG_FILTER_BW_45000HZ);
		} else {
			return -EINVAL; // Invalid bandwidth value
		}
		break;

	case MAX40109_DIGITAL_FILTER_SETUP:
		if (val->val1 < 0) {
			return -EINVAL; // Invalid digital filter setup value
		}
		int digital_filter_setup = val->val1;

		if (digital_filter_setup == 0) {
			ret = max40109_reg_update(dev, MAX40109_REG_CONFIG_MSB, DIGITAL_FILTER_MASK,
						  MAX40109_DIGITAL_FILTER_SETUP_AVERAGE_NONE);
		} else if (digital_filter_setup == 4) {
			ret = max40109_reg_update(dev, MAX40109_REG_CONFIG_MSB, DIGITAL_FILTER_MASK,
						  MAX40109_DIGITAL_FILTER_SETUP_AVERAGE_4);
		} else if (digital_filter_setup == 8) {
			ret = max40109_reg_update(dev, MAX40109_REG_CONFIG_MSB, DIGITAL_FILTER_MASK,
						  MAX40109_DIGITAL_FILTER_SETUP_AVERAGE_8);
		} else if (digital_filter_setup == 16) {
			ret = max40109_reg_update(dev, MAX40109_REG_CONFIG_MSB, DIGITAL_FILTER_MASK,
						  MAX40109_DIGITAL_FILTER_SETUP_AVERAGE_16);
		} else if (digital_filter_setup == 32) {
			ret = max40109_reg_update(dev, MAX40109_REG_CONFIG_MSB, DIGITAL_FILTER_MASK,
						  MAX40109_DIGITAL_FILTER_SETUP_AVERAGE_32);
		} else if (digital_filter_setup == 64) {
			ret = max40109_reg_update(dev, MAX40109_REG_CONFIG_MSB, DIGITAL_FILTER_MASK,
						  MAX40109_DIGITAL_FILTER_SETUP_AVERAGE_64);
		} else if (digital_filter_setup == 128) {
			ret = max40109_reg_update(dev, MAX40109_REG_CONFIG_MSB, DIGITAL_FILTER_MASK,
						  MAX40109_DIGITAL_FILTER_SETUP_AVERAGE_128);
		} else {
			return -EINVAL; // Invalid digital filter setup value
		}
		break;

	case MAX40109_ALERT_MODE:
		if (val->val1 < 0) {
			return -EINVAL; // Invalid alert mode value
		}
		int alert_mode = val->val1;

		ret = max40109_reg_update(dev, MAX40109_REG_CONFIG_LSB, ALERT_MODE_MASK,
					  alert_mode);
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

		if (val->val1 == 1) {
			ret = max40109_reg_update(dev, MAX40109_TEMP_MODE, DRV_SCALE_MASK,
						  MAX40109_DRV_SCALE_1_0);

			if (ret) {
				return ret;
			}
		}

		else {
			if (val->val2 == 500000) {
				ret = max40109_reg_update(dev, MAX40109_TEMP_MODE, DRV_SCALE_MASK,
							  MAX40109_DRV_SCALE_0_5);
			} else if (val->val2 == 375000) {
				ret = max40109_reg_update(dev, MAX40109_TEMP_MODE, DRV_SCALE_MASK,
							  MAX40109_DRV_SCALE_0_375);
			} else if (val->val2 == 250000) {
				ret = max40109_reg_update(dev, MAX40109_TEMP_MODE, DRV_SCALE_MASK,
							  MAX40109_DRV_SCALE_0_25);
			} else {
				return -EINVAL; // Invalid scale value
			}
		}

		break;

	case SENSOR_ATTR_CALIB_TARGET:
		int coeff = 0;
		switch (chan) {
		case SENSOR_CHAN_AMBIENT_TEMP_CALIB_COEFF_K0:
			coeff = MAX40109_CALIBRATION_K0;
			break;
		case SENSOR_CHAN_AMBIENT_TEMP_CALIB_COEFF_K1:
			coeff = MAX40109_CALIBRATION_K1;
			break;
		case SENSOR_CHAN_AMBIENT_TEMP_CALIB_COEFF_K2:
			coeff = MAX40109_CALIBRATION_K2;
			break;
		case SENSOR_CHAN_AMBIENT_TEMP_CALIB_COEFF_K3:
			coeff = MAX40109_CALIBRATION_K3;
			break;
		case SENSOR_CHAN_PRESSURE_CALIB_COEFF_H0:
			coeff = MAX40109_CALIBRATION_H0;
			break;
		case SENSOR_CHAN_PRESSURE_CALIB_COEFF_H1:
			coeff = MAX40109_CALIBRATION_H1;
			break;
		case SENSOR_CHAN_PRESSURE_CALIB_COEFF_H2:
			coeff = MAX40109_CALIBRATION_H2;
			break;
		case SENSOR_CHAN_PRESSURE_CALIB_COEFF_H3:
			coeff = MAX40109_CALIBRATION_H3;
			break;
		case SENSOR_CHAN_PRESSURE_CALIB_COEFF_G0:
			coeff = MAX40109_CALIBRATION_G0;
			break;
		case SENSOR_CHAN_PRESSURE_CALIB_COEFF_G1:
			coeff = MAX40109_CALIBRATION_G1;
			break;
		case SENSOR_CHAN_PRESSURE_CALIB_COEFF_G2:
			coeff = MAX40109_CALIBRATION_G2;
			break;
		case SENSOR_CHAN_PRESSURE_CALIB_COEFF_G3:
			coeff = MAX40109_CALIBRATION_G3;
			break;
		case SENSOR_CHAN_PRESSURE_CALIB_COEFF_N0:
			coeff = MAX40109_CALIBRATION_N0;
			break;
		case SENSOR_CHAN_PRESSURE_CALIB_COEFF_N1:
			coeff = MAX40109_CALIBRATION_N1;
			break;
		case SENSOR_CHAN_PRESSURE_CALIB_COEFF_N2:
			coeff = MAX40109_CALIBRATION_N2;
			break;
		case SENSOR_CHAN_PRESSURE_CALIB_COEFF_N3:
			coeff = MAX40109_CALIBRATION_N3;
			break;
		case SENSOR_CHAN_PRESSURE_CALIB_COEFF_M0:
			coeff = MAX40109_CALIBRATION_M0;
			break;
		case SENSOR_CHAN_PRESSURE_CALIB_COEFF_M1:
			coeff = MAX40109_CALIBRATION_M1;
			break;
		case SENSOR_CHAN_PRESSURE_CALIB_COEFF_M2:
			coeff = MAX40109_CALIBRATION_M2;
			break;
		case SENSOR_CHAN_PRESSURE_CALIB_COEFF_M3:
			coeff = MAX40109_CALIBRATION_M3;
			break;
		default:
			return -ENOTSUP; // Unsupported channel for calibration
			break;
		}
		float calibration_value = val->val1 + (val->val2 / 1000000.0f);

		ret = max40109_mtp_calibration(dev, coeff, calibration_value, false);
		if (ret < 0) {
			return ret; // Error setting calibration coefficient
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
	.undervoltage_drive = DT_INST_PROP(inst, undervoltage_drv), \
	.primary_threshold_pressure_value = DT_INST_PROP(inst, primary_threshold_pressure_value),       \
	.hysteresis_threshold_pressure_value = DT_INST_PROP(inst, hysteresis_threshold_pressure_value)


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
