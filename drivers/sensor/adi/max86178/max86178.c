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

/* Device instantiation macros */
#define MAX86178_CONFIG_I2C(inst)                                                                  \
	{                                                                                          \
		.bus = {.i2c = I2C_DT_SPEC_INST_GET(inst)},                                        \
		.bus_is_ready = max86178_bus_check_i2c,                                            \
		.reg_access = max86178_reg_access_i2c,                                             \
	}

#define MAX86178_CONFIG_SPI(inst)                                                                  \
	{                                                                                          \
		.bus = {.spi = SPI_DT_SPEC_INST_GET(                                               \
				inst, SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB,    \
				0)},                                                               \
		.bus_is_ready = max86178_bus_check_spi,                                            \
		.reg_access = max86178_reg_access_spi,                                             \
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
