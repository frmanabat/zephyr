/*
 * Copyright (c) 2026 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "max86178.h"
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor_clock.h>

LOG_MODULE_DECLARE(MAX86178);

void max86178_flush_fifo(const struct device *dev)
{
	int ret = 0;
	const struct max86178_dev_config *config = dev->config;
	uint8_t fifo_config =
		FIELD_PREP(MAX86178_FIFO_CFG2_FLUSH_FIFO_MSK, 1) |
		FIELD_PREP(MAX86178_FIFO_CFG2_FIFO_STAT_CLR_MSK, 1) |
		FIELD_PREP(MAX86178_FIFO_CFG2_FIFO_RO_MSK, config->fifo_cfg.fifo_rollover_en) |
		FIELD_PREP(MAX86178_FIFO_CFG2_A_FULL_TYPE_MSK, config->fifo_cfg.fifo_a_full_type);

	ret = max86178_reg_write(dev, MAX86178_FIFO_CFG2, &fifo_config, 1);
	if (ret < 0) {
		LOG_ERR("Failed to flush FIFO: %d", ret);
		return;
	}
	return;
}

void max86178_submit_stream(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	const struct sensor_read_config *cfg =
		(const struct sensor_read_config *)iodev_sqe->sqe.iodev->data;

	struct max86178_data *data = (struct max86178_data *)dev->data;
	const struct max86178_dev_config *cfg_178 = dev->config;
	uint8_t fifo_watermark_irq = 0;
	int ret = 0;

	ret = gpio_pin_interrupt_configure_dt(&cfg_178->interrupt_gpio, GPIO_INT_DISABLE);
	if (ret < 0) {
		LOG_ERR("Failed to disable interrupt: %d", ret);
		rtio_iodev_sqe_err(iodev_sqe, ret);
		return;
	}

	for (size_t i = 0; i < cfg->count; i++) {
		if (cfg->triggers[i].trigger == SENSOR_TRIG_FIFO_WATERMARK) {
			fifo_watermark_irq = 1;
			break;
		}
	}

	if (fifo_watermark_irq != data->fifo_watermark_irq) {
		data->fifo_watermark_irq = fifo_watermark_irq;

		ret = max86178_reg_update(
			dev, cfg_178->route_to_int2 ? MAX86178_INT2_EN1 : MAX86178_INT1_EN1,
			MAX86178_INT_EN1_A_FULL_MSK, fifo_watermark_irq);
		if (ret < 0) {
			LOG_ERR("Failed to update interrupt map: %d", ret);
			rtio_iodev_sqe_err(iodev_sqe, ret);
			return;
		}

		max86178_flush_fifo(dev);

		ret = max86178_reg_read(dev, MAX86178_STATUS1, &data->status1, 1);
		if (ret < 0) {
			LOG_ERR("Failed to read status register: %d", ret);
			rtio_iodev_sqe_err(iodev_sqe, ret);
		}
	}
	ret = gpio_pin_interrupt_configure_dt(&cfg_178->interrupt_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to enable interrupt: %d", ret);
		rtio_iodev_sqe_err(iodev_sqe, ret);
		return;
	}
	data->sqe = iodev_sqe;
}

static void max86178_irq_en_cb(struct rtio *r, const struct rtio_sqe *sqe, int result, void *arg)
{
	ARG_UNUSED(result);
	const struct device *dev = (const struct device *)arg;
	const struct max86178_dev_config *config = (const struct max86178_dev_config *)dev->config;

	gpio_pin_interrupt_configure_dt(&config->interrupt_gpio, GPIO_INT_EDGE_TO_ACTIVE);
}

static void max86178_flush_rtio(const struct device *dev)
{
	struct max86178_data *data = (struct max86178_data *)dev->data;
	const struct max86178_dev_config *config = (const struct max86178_dev_config *)dev->config;
	uint8_t fifo_config =
		FIELD_PREP(MAX86178_FIFO_CFG2_FLUSH_FIFO_MSK, 1) |
		FIELD_PREP(MAX86178_FIFO_CFG2_FIFO_STAT_CLR_MSK, 1) |
		FIELD_PREP(MAX86178_FIFO_CFG2_FIFO_RO_MSK, config->fifo_cfg.fifo_rollover_en) |
		FIELD_PREP(MAX86178_FIFO_CFG2_A_FULL_TYPE_MSK, config->fifo_cfg.fifo_a_full_type);
	struct rtio_sqe *sqe = rtio_sqe_acquire(data->rtio_ctx);

#ifdef CONFIG_SPI_RTIO
	uint8_t reg_addr_w[3] = {MAX86178_FIFO_CFG2, 0, fifo_config};
	rtio_sqe_prep_tiny_write(sqe, data->iodev, RTIO_PRIO_NORM, reg_addr_w, 3, NULL);
#else
	uint8_t reg_addr_w[2] = {MAX86178_FIFO_CFG2, fifo_config};
	rtio_sqe_prep_tiny_write(sqe, data->iodev, RTIO_PRIO_NORM, reg_addr_w, 2, NULL);
#endif

	struct rtio_sqe *complete_op = rtio_sqe_acquire(data->rtio_ctx);
	rtio_sqe_prep_callback(complete_op, max86178_irq_en_cb, (void *)dev, NULL);
	rtio_submit(data->rtio_ctx, 0);
}

static void max86178_complete_op_cb(struct rtio *r, const struct rtio_sqe *sqe, int result,
				    void *arg)
{
	ARG_UNUSED(result);
	const struct device *dev = (const struct device *)arg;
	const struct max86178_dev_config *config = (const struct max86178_dev_config *)dev->config;
	struct rtio_iodev_sqe *current_sqe = (struct rtio_iodev_sqe *)sqe->userdata;

	rtio_iodev_sqe_ok(current_sqe, 0);
	gpio_pin_interrupt_configure_dt(&config->interrupt_gpio, GPIO_INT_EDGE_TO_ACTIVE);
}

static void max86178_process_fifo_samples_cb(struct rtio *r, const struct rtio_sqe *sqe, int result,
					     void *arg)
{
	ARG_UNUSED(result);
	const struct device *dev = (const struct device *)arg;
	struct max86178_data *data = (struct max86178_data *)dev->data;
	const struct max86178_dev_config *config = (const struct max86178_dev_config *)dev->config;
	struct rtio_iodev_sqe *current_sqe = data->sqe;
	uint16_t fifo_sample_count =
		(FIELD_GET(MAX86178_FIFO_COUNTER1_DATA_COUNT_MSB, data->fifo_counter[0]) << 8) |
		data->fifo_counter[1];
	uint16_t fifo_bytes = fifo_sample_count * MAX86178_SAMPLE_SIZE_BYTES;

	data->sqe = NULL;

	if (current_sqe == NULL) {
		LOG_ERR("No pending SQE");
		gpio_pin_interrupt_configure_dt(&config->interrupt_gpio, GPIO_INT_EDGE_TO_ACTIVE);
		return;
	}

	const size_t min_read_size = sizeof(struct max86178_fifo_data) + MAX86178_SAMPLE_SIZE_BYTES;
	const size_t ideal_read_size = sizeof(struct max86178_fifo_data) + fifo_bytes;

	uint8_t *buf;
	uint32_t buf_len;

	if (rtio_sqe_rx_buf(current_sqe, min_read_size, ideal_read_size, &buf, &buf_len) != 0) {
		LOG_ERR("Failed to get buffer");
		rtio_iodev_sqe_err(current_sqe, -ENOMEM);
		gpio_pin_interrupt_configure_dt(&config->interrupt_gpio, GPIO_INT_EDGE_TO_ACTIVE);
		return;
	}

	LOG_DBG("Requesting buffer [%u, %u] got %u", (unsigned int)min_read_size,
		(unsigned int)ideal_read_size, buf_len);

	struct max86178_fifo_data *hdr = (struct max86178_fifo_data *)buf;

	hdr->is_fifo = 1;
	hdr->timestamp = data->timestamp;
	hdr->status1 = data->status1;
	hdr->fifo_samples = fifo_sample_count;
	hdr->fifo_byte_count = fifo_bytes;
	hdr->sample_set_size = MAX86178_SAMPLE_SIZE_BYTES;

	uint32_t buf_avail = buf_len - sizeof(*hdr);

	memcpy(buf, hdr, sizeof(*hdr));

	uint8_t *read_buf = buf + sizeof(*hdr);
	uint32_t read_len = MIN(fifo_bytes, buf_avail);

	((struct max86178_fifo_data *)buf)->fifo_byte_count = read_len;
	__ASSERT_NO_MSG(read_len % MAX86178_SAMPLE_SIZE_BYTES == 0);

	/* Flush Completions */
	struct rtio_cqe *cqe;

	int res = 0;
	do {
		cqe = rtio_cqe_consume(data->rtio_ctx);
		if (cqe != NULL) {
			if ((cqe->result < 0) && (res == 0)) {
				LOG_ERR("Bus error: %d", cqe->result);
				res = cqe->result;
			}
			rtio_cqe_release(data->rtio_ctx, cqe);
		}
	} while (cqe != NULL);

	/* Bail/cancel attempt to read sensor on any error */
	if (res != 0) {
		rtio_iodev_sqe_err(current_sqe, res);
		return;
	}

	struct rtio_sqe *write_fifo_addr = rtio_sqe_acquire(data->rtio_ctx);
	struct rtio_sqe *read_fifo_addr = rtio_sqe_acquire(data->rtio_ctx);
	struct rtio_sqe *complete_op = rtio_sqe_acquire(data->rtio_ctx);
#ifdef CONFIG_SPI_RTIO
	uint8_t reg[] = {MAX86178_FIFO_DATA, 0x80};
	rtio_sqe_prep_tiny_write(write_fifo_addr, data->iodev, RTIO_PRIO_NORM, reg, 2, NULL);
#elif defined(CONFIG_I2C_RTIO)
	uint8_t reg = MAX86178_FIFO_DATA;
	rtio_sqe_prep_tiny_write(write_fifo_addr, data->iodev, RTIO_PRIO_NORM, &reg, 1, NULL);
#endif
	write_fifo_addr->flags |= RTIO_SQE_TRANSACTION;
	rtio_sqe_prep_read(read_fifo_addr, data->iodev, RTIO_PRIO_NORM, read_buf, read_len, NULL);
	read_fifo_addr->flags |= RTIO_SQE_CHAINED;
#ifdef CONFIG_I2C_RTIO
	read_fifo_addr->iodev_flags |= RTIO_IODEV_I2C_STOP | RTIO_IODEV_I2C_RESTART;
#endif
	rtio_sqe_prep_callback(complete_op, max86178_complete_op_cb, (void *)dev,
			       (void *)current_sqe);
	rtio_submit(data->rtio_ctx, 0);
}

static void max86178_process_status1_cb(struct rtio *r, const struct rtio_sqe *sqe, int result,
					void *arg)
{
	ARG_UNUSED(result);
	const struct device *dev = (const struct device *)arg;
	struct max86178_data *data = (struct max86178_data *)dev->data;
	const struct max86178_dev_config *config = (const struct max86178_dev_config *)dev->config;
	struct rtio_iodev_sqe *current_sqe = data->sqe;
	struct sensor_read_config *read_config;
	uint8_t status1 = data->status1;

	if (data->sqe == NULL) {
		return;
	}

	read_config = (struct sensor_read_config *)data->sqe->sqe.iodev->data;

	if (read_config == NULL) {
		return;
	}

	if (read_config->is_streaming == false) {
		return;
	}

	gpio_pin_interrupt_configure_dt(&config->interrupt_gpio, GPIO_INT_DISABLE);

	struct sensor_stream_trigger *fifo_wmark_cfg = NULL;

	for (int i = 0; i < read_config->count; ++i) {
		if (read_config->triggers[i].trigger == SENSOR_TRIG_FIFO_WATERMARK) {
			fifo_wmark_cfg = &read_config->triggers[i];
			continue;
		}
	}

	bool fifo_watermark_irq = false;

	if ((fifo_wmark_cfg != NULL) && FIELD_GET(MAX86178_STATUS1_A_FULL_MSK, status1)) {
		fifo_watermark_irq = true;
	}

	if (!fifo_watermark_irq) {
		gpio_pin_interrupt_configure_dt(&config->interrupt_gpio, GPIO_INT_EDGE_TO_ACTIVE);
		return;
	}

	/* Flush completions */
	struct rtio_cqe *cqe;
	int res = 0;

	do {
		cqe = rtio_cqe_consume(data->rtio_ctx);
		if (cqe != NULL) {
			if ((cqe->result < 0) && (res == 0)) {
				LOG_ERR("Bus error: %d", cqe->result);
				res = cqe->result;
			}
			rtio_cqe_release(data->rtio_ctx, cqe);
		}
	} while (cqe != NULL);

	/* Bail/cancel attempt to read sensor on any error */
	if (res != 0) {
		rtio_iodev_sqe_err(data->sqe, res);
		return;
	}

	__ASSERT_NO_MSG(fifo_wmark_cfg != NULL);

	enum sensor_stream_data_opt data_opt = fifo_wmark_cfg->opt;

	if (data_opt == SENSOR_STREAM_DATA_NOP || data_opt == SENSOR_STREAM_DATA_DROP) {
		uint8_t *buf;
		uint32_t buf_len;

		/* Clear Streaming_sqe since we're done with the call */
		data->sqe = NULL;
		if (rtio_sqe_rx_buf(current_sqe, sizeof(struct max86178_fifo_data),
				    sizeof(struct max86178_fifo_data), &buf, &buf_len) != 0) {
			rtio_iodev_sqe_err(current_sqe, -ENOMEM);
			gpio_pin_interrupt_configure_dt(&config->interrupt_gpio,
							GPIO_INT_EDGE_TO_ACTIVE);
			return;
		}

		struct max86178_fifo_data *rx_data = (struct max86178_fifo_data *)buf;
		memset(buf, 0, buf_len);
		rx_data->is_fifo = 1;
		rx_data->timestamp = data->timestamp;
		rx_data->status1 = status1;
		rx_data->fifo_samples = 0;
		rtio_iodev_sqe_ok(current_sqe, 0);

		if (data_opt == SENSOR_STREAM_DATA_DROP) {
			/* Flush the FIFO by disabling it. Save current mode for after the reset. */
			max86178_flush_rtio(dev);
		}

		gpio_pin_interrupt_configure_dt(&config->interrupt_gpio, GPIO_INT_EDGE_TO_ACTIVE);
		return;
	}

	struct rtio_sqe *write_fifo_addr = rtio_sqe_acquire(data->rtio_ctx);
	struct rtio_sqe *read_fifo_addr = rtio_sqe_acquire(data->rtio_ctx);
	struct rtio_sqe *complete_op = rtio_sqe_acquire(data->rtio_ctx);
#ifdef CONFIG_SPI_RTIO
	uint8_t reg[] = {MAX86178_FIFO_COUNTER1, 0x80};
	rtio_sqe_prep_tiny_write(write_fifo_addr, data->iodev, RTIO_PRIO_NORM, reg, 2, NULL);
#elif defined(CONFIG_I2C_RTIO)
	uint8_t reg = MAX86178_FIFO_COUNTER1;
	rtio_sqe_prep_tiny_write(write_fifo_addr, data->iodev, RTIO_PRIO_NORM, &reg, 1, NULL);
#endif
	write_fifo_addr->flags |= RTIO_SQE_TRANSACTION;
	rtio_sqe_prep_read(read_fifo_addr, data->iodev, RTIO_PRIO_NORM, data->fifo_counter, 2,
			   NULL);
	read_fifo_addr->flags |= RTIO_SQE_CHAINED;
#ifdef CONFIG_I2C_RTIO
	read_fifo_addr->iodev_flags |= RTIO_IODEV_I2C_STOP | RTIO_IODEV_I2C_RESTART;
#endif
	rtio_sqe_prep_callback(complete_op, max86178_process_fifo_samples_cb, (void *)dev, NULL);
	rtio_submit(data->rtio_ctx, 0);
}

void max86178_stream_irq_handler(const struct device *dev)
{
	int ret = 0;
	struct max86178_data *data = (struct max86178_data *)dev->data;
	uint64_t cycles;

	if (data->sqe == NULL) {
		return;
	}

	ret = sensor_clock_get_cycles(&cycles);
	if (ret != 0) {
		LOG_ERR("Failed to get sensor clock cycles: %d", ret);
		rtio_iodev_sqe_err(data->sqe, ret);
		return;
	}

	data->timestamp = sensor_clock_cycles_to_ns(cycles);
	struct rtio_sqe *write_status_addr = rtio_sqe_acquire(data->rtio_ctx);
	struct rtio_sqe *read_status_reg = rtio_sqe_acquire(data->rtio_ctx);
	struct rtio_sqe *check_status_reg = rtio_sqe_acquire(data->rtio_ctx);
#ifdef CONFIG_SPI_RTIO
	uint8_t reg[] = {MAX86178_STATUS1, 0x80};
	rtio_sqe_prep_tiny_write(write_status_addr, data->iodev, RTIO_PRIO_NORM, reg, 2, NULL);
#elif defined(CONFIG_I2C_RTIO)
	uint8_t reg = MAX86178_STATUS1;
	rtio_sqe_prep_tiny_write(write_status_addr, data->iodev, RTIO_PRIO_NORM, &reg, 1, NULL);
#endif
	write_status_addr->flags |= RTIO_SQE_TRANSACTION;
	rtio_sqe_prep_read(read_status_reg, data->iodev, RTIO_PRIO_NORM, &data->status1, 1, NULL);
	read_status_reg->flags |= RTIO_SQE_CHAINED;
#ifdef CONFIG_I2C_RTIO
	read_status_reg->iodev_flags |= RTIO_IODEV_I2C_STOP | RTIO_IODEV_I2C_RESTART;
#endif
	rtio_sqe_prep_callback(check_status_reg, max86178_process_status1_cb, (void *)dev, NULL);
	rtio_submit(data->rtio_ctx, 0);
}
