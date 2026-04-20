/*
 * Copyright (c) 2026 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "max86178.h"
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_DECLARE(MAX86178);

/* FIFO tag field is in bits 23:20 */
#define MAX86178_FIFO_TAG_MASK GENMASK(23, 20)
#define MAX86178_FIFO_TIMING_TAG_MASK GENMASK(19, 18)
#define MAX86178_FIFO_DATA_FIELD GENMASK(19, 0)
#define MAX86178_FIFO_TIMING_PPG_ECG_SAMPLE_FIELD GENMASK(9, 0)
#define MAX86178_FIFO_TIMING_PPG_BIOZ_SAMPLE_FIELD GENMASK(13, 0)

static bool count_instance_from_fifo_data(uint32_t fifo_data, uint16_t channel_enabled_mask)
{
	uint8_t tag = FIELD_GET(MAX86178_FIFO_TAG_MASK, fifo_data);
	uint8_t tag_timing = FIELD_GET(MAX86178_FIFO_TIMING_TAG_MASK, fifo_data);
	uint16_t tag_bit_mask = 0;

	/* Map tag value to corresponding bit position */
	switch (tag) {
	case MAX86178_PPG_MEAS1_TAG:
		tag_bit_mask = BIT(0);
		break;
	case MAX86178_PPG_MEAS2_TAG:
		tag_bit_mask = BIT(1);
		break;
	case MAX86178_PPG_MEAS3_TAG:
		tag_bit_mask = BIT(2);
		break;
	case MAX86178_PPG_MEAS4_TAG:
		tag_bit_mask = BIT(3);
		break;
	case MAX86178_PPG_MEAS5_TAG:
		tag_bit_mask = BIT(4);
		break;
	case MAX86178_PPG_MEAS6_TAG:
		tag_bit_mask = BIT(5);
		break;
	case MAX86178_PPG_DARK_TAG:
		tag_bit_mask = BIT(6);
		break;
	case MAX86178_PPG_ALC_OVF_TAG:
		tag_bit_mask = BIT(7);
		break;
	case MAX86178_PPG_EXP_OVF_TAG:
		tag_bit_mask = BIT(8);
		break;
	case MAX86178_BIOZ_I_TAG:
		tag_bit_mask = BIT(9);
		break;
	case MAX86178_BIOZ_Q_TAG:
		tag_bit_mask = BIT(10);
		break;
	case MAX86178_ECG_AND_FAST_RECOVERY_TAG:
		tag_bit_mask = BIT(11);
		break;
	case MAX86178_ECGP_ECGN_TAG:
		tag_bit_mask = BIT(12);
		break;
	case MAX86178_CAPP_CAPN_TAG:
		tag_bit_mask = BIT(13);
		break;
	case MAX86178_TIMING_TAG:
		if (tag_timing == 0) {
			tag_bit_mask = BIT(14); /* ECG/PPG timing */
		} else if (tag_timing == 1) {
			tag_bit_mask = BIT(15); /* BIOZ/PPG timing */
		}
		break;
	default:
		break;
	}

	/* Check if this tag's bit is enabled in the channel mask */
	return (channel_enabled_mask & tag_bit_mask) != 0;
}

static uint16_t get_channel_mask(enum sensor_channel chan_type)
{
	uint16_t channel_enabled_mask = 0;

	/* Channel Check */
	switch ((int)chan_type) {
	case SENSOR_CHAN_PPG_MEAS1:
		channel_enabled_mask = BIT(0);
		break;
	case SENSOR_CHAN_PPG_MEAS2:
		channel_enabled_mask = BIT(1);
		break;
	case SENSOR_CHAN_PPG_MEAS3:
		channel_enabled_mask = BIT(2);
		break;
	case SENSOR_CHAN_PPG_MEAS4:
		channel_enabled_mask = BIT(3);
		break;
	case SENSOR_CHAN_PPG_MEAS5:
		channel_enabled_mask = BIT(4);
		break;
	case SENSOR_CHAN_PPG_MEAS6:
		channel_enabled_mask = BIT(5);
		break;
	case SENSOR_CHAN_PPG_DARK:
		channel_enabled_mask = BIT(6);
		break;
	case SENSOR_CHAN_PPG_ALC_OVF:
		channel_enabled_mask = BIT(7);
		break;
	case SENSOR_CHAN_PPG_EXP_OVF:
		channel_enabled_mask = BIT(8);
		break;
	case SENSOR_CHAN_BIOZ_I:
		channel_enabled_mask = BIT(9);
		break;
	case SENSOR_CHAN_BIOZ_Q:
		channel_enabled_mask = BIT(10);
		break;
	case SENSOR_CHAN_ECG_AND_FAST_REC:
		channel_enabled_mask = BIT(11);
		break;
	case SENSOR_CHAN_ECGP_ECGN:
		channel_enabled_mask = BIT(12);
		break;
	case SENSOR_CHAN_CAPP_CAPN:
		channel_enabled_mask = BIT(13);
		break;
	case SENSOR_CHAN_TIMING_ECG_PPG:
		channel_enabled_mask = BIT(14);
		break;
	case SENSOR_CHAN_TIMING_BIOZ_PPG:
		channel_enabled_mask = BIT(15);
		break;
	case SENSOR_CHAN_ALL:
		channel_enabled_mask = GENMASK(15, 0);
		break;
	default:
		break;
	}

	return channel_enabled_mask;
}

static int max86178_decoder_get_frame_count(const uint8_t *buffer, struct sensor_chan_spec channel,
					    uint16_t *frame_count)
{
#ifndef CONFIG_MAX86178_STREAM
	return -ENOTSUP;
#endif
	const struct max86178_fifo_data *data = (const struct max86178_fifo_data *)buffer;
	uint16_t channel_enabled_mask = get_channel_mask(channel.chan_type);

	if (channel_enabled_mask == 0) {
		return -ENOTSUP;
	}

	buffer += sizeof(struct max86178_fifo_data);
	const uint8_t *buffer_end = buffer + data->fifo_byte_count;
	uint16_t count = 0;

	while (buffer < buffer_end) {
		uint32_t fifo_data = sys_get_be24(buffer);

		if (count_instance_from_fifo_data(fifo_data, channel_enabled_mask)) {
			count++;
		}
		buffer += MAX86178_SAMPLE_SIZE_BYTES;
	}
	*frame_count = count;
	return 0;
}

static int max86178_decoder_decode(const uint8_t *buffer, struct sensor_chan_spec channel,
				   uint32_t *fit, uint16_t max_count, void *data_out)
{
#ifndef CONFIG_MAX86178_STREAM
	return -ENOTSUP;
#endif
	const struct max86178_fifo_data *data = (const struct max86178_fifo_data *)buffer;
	struct sensor_q31_data *out = (struct sensor_q31_data *)data_out;
	uint16_t channel_enabled_mask = get_channel_mask(channel.chan_type);

	if (channel_enabled_mask == 0) {
		LOG_ERR("Unsupported channel type %d for MAX86178 decoder", channel.chan_type);
		return -EINVAL;
	}

	buffer += sizeof(struct max86178_fifo_data);
	const uint8_t *buffer_end = buffer + data->fifo_byte_count;
	int count = 0;
	uint32_t samples_seen = 0;    /* Track how many matching samples we've seen */
	uint32_t start_offset = *fit; /* Number of samples to skip (already decoded) */

	while (count < max_count && buffer < buffer_end) {
		uint32_t fifo_data = sys_get_be24(buffer);
		uint32_t sample_value = 0;
		if (count_instance_from_fifo_data(fifo_data, channel_enabled_mask)) {
			/* Only decode if we've skipped past already-decoded samples */
			if (samples_seen >= start_offset) {
				if (channel.chan_type == SENSOR_CHAN_TIMING_ECG_PPG) {
					/* For timing channel, decode the timing tag bits instead of sample value */
					sample_value = FIELD_GET(MAX86178_FIFO_TIMING_PPG_ECG_SAMPLE_FIELD, fifo_data);
					out[count].readings[0].value = sample_value;
				} else if (channel.chan_type == SENSOR_CHAN_TIMING_BIOZ_PPG) {
					sample_value = FIELD_GET(MAX86178_FIFO_TIMING_PPG_BIOZ_SAMPLE_FIELD, fifo_data);
					out[count].readings[0].value = sample_value;
				}
				else {
					sample_value =
						FIELD_GET(MAX86178_FIFO_DATA_FIELD, fifo_data);

					out[count].readings[0].value = sample_value;
				}
				count++;
			}
			samples_seen++;
		}
		buffer += MAX86178_SAMPLE_SIZE_BYTES;
	}

	*fit += count; /* Update fit with number of newly decoded samples */
	return count;
}

static int max86178_decoder_get_size_info(struct sensor_chan_spec channel, size_t *base_size,
					  size_t *frame_size)
{
	__ASSERT_NO_MSG(base_size != NULL);
	__ASSERT_NO_MSG(frame_size != NULL);

	switch ((int)channel.chan_type) {
	case SENSOR_CHAN_PPG_MEAS1:
	case SENSOR_CHAN_PPG_MEAS2:
	case SENSOR_CHAN_PPG_MEAS3:
	case SENSOR_CHAN_PPG_MEAS4:
	case SENSOR_CHAN_PPG_MEAS5:
	case SENSOR_CHAN_PPG_MEAS6:
	case SENSOR_CHAN_PPG_DARK:
	case SENSOR_CHAN_PPG_ALC_OVF:
	case SENSOR_CHAN_PPG_EXP_OVF:
	case SENSOR_CHAN_BIOZ_I:
	case SENSOR_CHAN_BIOZ_Q:
	case SENSOR_CHAN_ECG_AND_FAST_REC:
	case SENSOR_CHAN_ECGP_ECGN:
	case SENSOR_CHAN_CAPP_CAPN:
	case SENSOR_CHAN_TIMING_ECG_PPG:
	case SENSOR_CHAN_TIMING_BIOZ_PPG:
		*base_size = sizeof(struct sensor_q31_data);
		*frame_size = sizeof(struct sensor_q31_sample_data);
		return 0;
	default:
		LOG_ERR("Unsupported channel type: %d", channel.chan_type);
		return -ENOTSUP;
	}
}

static bool max86178_decoder_has_trigger(const uint8_t *buffer, enum sensor_trigger_type trigger)
{
#ifdef CONFIG_MAX86178_STREAM
	return false;
#endif
	const struct max86178_fifo_data *fifo_data = (const struct max86178_fifo_data *)buffer;

	switch (trigger) {
	case SENSOR_TRIG_FIFO_FULL:
	case SENSOR_TRIG_FIFO_WATERMARK:
		return FIELD_GET(MAX86178_STATUS1_A_FULL_MSK, fifo_data->status1);
	default:
		return false;
	}
}

SENSOR_DECODER_API_DT_DEFINE() = {
	.get_frame_count = max86178_decoder_get_frame_count,
	.get_size_info = max86178_decoder_get_size_info,
	.decode = max86178_decoder_decode,
	.has_trigger = max86178_decoder_has_trigger,
};

/**
 * @brief Get the sensor decoder API for MAX86178
 *
 * @param dev Device pointer
 * @param decoder Decoder API pointer
 * @return int 0 on success, negative error code otherwise
 */
int max86178_get_decoder(const struct device *dev, const struct sensor_decoder_api **decoder)
{
	ARG_UNUSED(dev);
	*decoder = &SENSOR_DECODER_NAME();
	return 0;
}
