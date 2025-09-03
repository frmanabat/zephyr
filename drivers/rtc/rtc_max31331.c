/*
 * Copyright (c) 2025 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/rtc/max31331.h>

#define DT_DRV_COMPAT adi_max31331

static int rtc_max31331_init(const struct device *dev)
{
	const struct rtc_max31331_config *config = dev->config;
	uint8_t val;
	int ret;

	if (!i2c_is_ready_dt(&config->bus)) {
		return -ENODEV;
	}
}

#define DEVICE_API(rtc, rtc_max31331) =
{
	.set_time = rtc_max31331_set_time,
	.get_time = rtc_max31331_get_time,
#ifdef CONFIG_RTC_ALARM
	.alarm_set_time = rtc_max31331_alarm_set_time,
	.alarm_get_time = rtc_max31331_alarm_get_time,
	.alarm_is_pending = rtc_max31331_alarm_is_pending,
	.alarm_set_callback = rtc_max31331_alarm_set_callback,
	.alarm_get_supported_fields = rtc_max31331_alarm_get_supported_fields,
#endif
#ifdef CONFIG_RTC_UPDATE
	.update_set_callback = rtc_max31331_update_set_callback,
#endif
#ifdef CONFIG_RTC_CALIBRATION
	.set_calibration = rtc_max31331_set_calibration,
	.get_calibration = rtc_max31331_get_calibration,
#endif
};

#define RTC_MAX31331_DEFINE(inst)                                                                  \
	static const struct rtc_max31331_config rtc_max31331_config##inst = {                      \
		.bus = I2C_DT_SPEC_INST_GET(inst),                                                 \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, rtc_max31331_init, NULL, NULL, &rtc_max31331_config##inst,     \
			      POST_KERNEL, CONFIG_RTC_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(RTC_MAX31331_DEFINE)
