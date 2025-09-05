/*
 * Copyright (c) 2025 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/rtc/rtc_max31331.h>
#include <zephyr/drivers/rtc.h>

#define DT_DRV_COMPAT adi_max31331

LOG_MODULE_REGISTER(rtc_max31331, CONFIG_RTC_LOG_LEVEL);

static int max31331_reg_access(const struct device *dev, uint8_t addr_reg, uint8_t *data, bool read,
			       uint8_t length)
{

	const struct rtc_max31331_config *config = dev->config;
	int ret;

	if (read) {
		ret = i2c_burst_read_dt(&config->i2c, addr_reg, data, length);
	} else {
		ret = i2c_burst_write_dt(&config->i2c, addr_reg, data, length);
	}

	return ret < 0 ? ret : 0;
}

int max31331_reg_read(const struct device *dev, uint8_t reg_addr, uint8_t *val, uint8_t length)
{
	return max31331_reg_access(dev, reg_addr, val, true, length);
}

int max31331_reg_write(const struct device *dev, uint8_t reg_addr, uint8_t val)
{
	return max31331_reg_access(dev, reg_addr, &val, false, 1);
}

int max31331_reg_write_multiple(const struct device *dev, uint8_t reg_addr, const uint8_t *val,
				uint8_t length)
{
	const struct max31331_config *config = dev->config;
	int ret;
	if (length < 2) {
		return -EINVAL; // Invalid length
	}
	return max31331_reg_access(dev, reg_addr, val, false, length);
}

int max31331_reg_update(const struct device *dev, uint8_t reg_addr, uint8_t mask, uint8_t val)
{
	uint8_t reg_val = 0;
	int ret;

	ret = max31331_reg_read(dev, reg_addr, &reg_val, 1);
	if (ret < 0) {
		return ret;
	}

	reg_val &= ~mask;                 // Clear the bits specified by the mask
	reg_val |= FIELD_PREP(mask, val); // Set the bits specified by val

	return max31331_reg_write(dev, reg_addr, reg_val);
}

static int rtc_max31331_get_time (const struct device *dev, struct rtc_time *timeptr)
{
	const struct rtc_max31331_config *config = dev->config;
	int ret;
	uint8_t raw_time[7];

	ret = max31331_reg_read(dev, MAX31331_SECONDS, raw_time, sizeof(raw_time));
	if (ret) {
		LOG_ERR("Unable to get time. Err: %i", ret);
		return ret;
	}

	timeptr->tm_sec  = bcd2bin(raw_time[0] & SECONDS_FIELD_MASK);
	timeptr->tm_min  = bcd2bin(raw_time[1] & MINUTES_FIELD_MASK);
	timeptr->tm_hour = bcd2bin(raw_time[2] & HOURS_FIELD_MASK); /* Support 24 hour format to make it compatible with BCD MACRO*/
	timeptr->tm_wday = bcd2bin(raw_time[3] & DAY_FIELD_MASK) + MAX31331_DAY_OFFSET;
	timeptr->tm_mday = bcd2bin(raw_time[4] & DATE_FIELD_MASK);
	timeptr->tm_mon  = bcd2bin(raw_time[5] & MONTH_FIELD_MASK);
	if (raw_time[5] & BIT(7)) {
		/* Century bit is set, so we are in 21st century */
		timeptr->tm_year = bcd2bin(raw_time[6] & YEAR_FIELD_MASK) + MAX31331_YEAR_2100;
	} else {
		timeptr->tm_year = bcd2bin(raw_time[6] & YEAR_FIELD_MASK);
	}

	LOG_DBG("Get time: year: %d, month: %d, month day: %d, week day: %d, hour: %d, "
			"minute: %d, second: %d",
			timeptr->tm_year, timeptr->tm_mon, timeptr->tm_mday, timeptr->tm_wday,
			timeptr->tm_hour, timeptr->tm_min, timeptr->tm_sec);

	return 0;
}

static int rtc_max31331_set_time (const struct device *dev, const struct rtc_time *timeptr)
{
	const struct rtc_max31331_config *config = dev->config;
	int ret;
	uint8_t raw_time[7];

	if((timeptr == NULL) || !rtc_utils_validate_rtc_time(timeptr, MAX31331_RTC_TIME_MASK)) {
		LOG_ERR("invalid time");
		return -EINVAL;
	}

	raw_time[0] = bin2bcd(timeptr->tm_sec) & SECONDS_FIELD_MASK;
	raw_time[1] = bin2bcd(timeptr->tm_min) & MINUTES_FIELD_MASK;
	raw_time[2] = bin2bcd(timeptr->tm_hour) & HOURS_FIELD_MASK; /* Support 24 hour format to make it compatible with BCD MACRO*/
	raw_time[3] = bin2bcd(timeptr->tm_wday - MAX31331_DAY_OFFSET) & DAY_FIELD_MASK;
	raw_time[4] = bin2bcd(timeptr->tm_mday) & DATE_FIELD_MASK;
	raw_time[5] = bin2bcd(timeptr->tm_mon) & MONTH_FIELD_MASK;

	if(timeptr->tm_year >= MAX31331_YEAR_2100){
		raw_time[5] |= BIT(7); // Set century bit for 21st century
		raw_time[6] = bin2bcd(timeptr->tm_year - MAX31331_YEAR_2100) & YEAR_FIELD_MASK;
	} 
	else{
		raw_time[6] = bin2bcd(timeptr->tm_year) & YEAR_FIELD_MASK;
	}

	ret = max31331_reg_write_multiple(dev, MAX31331_SECONDS, raw_time, sizeof(raw_time));
	if (ret) {
		LOG_ERR("Error when setting time: %i", ret);
		return ret;
	}
	return 0;
}

#ifdef CONFIG_RTC_ALARM
static int rtc_max31331_alarm_set_time(const struct device *dev, uint16_t id, uint16_t mask,
				  const struct rtc_time *timeptr)
{
	const struct rtc_max31331_config *config = dev->config;
	int ret = 0;
	uint8_t raw_time [6];
	if (id <= 0) {
		LOG_ERR("invalid ID %d", id);
		return -EINVAL;
	}

	if ((mask & RTC_ALARM_TIME_MASK_MONTHDAY) && (mask & RTC_ALARM_TIME_MASK_WEEKDAY)) {
		LOG_ERR("Both day and date are set. Not Supported");
		return -EINVAL;
	}

	if((timeptr == NULL) || !rtc_utils_validate_rtc_time(timeptr, mask)){
		LOG_ERR("invalid alarm time");
		return -EINVAL;
	}

	// if ((timeptr->tm_wday != NULL) && (timeptr->tm_mday != NULL)) {
	// 	LOG_ERR("Both weekday and month day are set. Not supported");
	// 	return -EINVAL;
	// }

	if (id == 1){

	raw_time[0] = (mask & RTC_ALARM_TIME_MASK_SECOND) ? (bin2bcd(timeptr->tm_sec) & ALARM_1_SECONDS_FIELD_MASK) & ~ALARM_1_SECONDS_ENABLE_MASK : (bin2bcd(timeptr->tm_sec) & ALARM_1_SECONDS_FIELD_MASK) | ALARM_1_SECONDS_ENABLE_MASK;
	raw_time[1] = (mask & RTC_ALARM_TIME_MASK_MINUTE) ? (bin2bcd(timeptr->tm_min) & ALARM_1_MINUTES_FIELD_MASK) & ~ALARM_1_MINUTES_ENABLE_MASK : (bin2bcd(timeptr->tm_min) & ALARM_1_MINUTES_FIELD_MASK) | ALARM_1_MINUTES_ENABLE_MASK;
	raw_time[2] = (mask & RTC_ALARM_TIME_MASK_HOUR)   ? (bin2bcd(timeptr->tm_hour) & ALARM_1_HOURS_FIELD_MASK) & ~ALARM_1_HOURS_ENABLE_MASK     : (bin2bcd(timeptr->tm_hour) & ALARM_1_HOURS_FIELD_MASK) | ALARM_1_HOURS_ENABLE_MASK;
	
	if(timeptr->tm_wday != NULL){
		raw_time[3] = (mask & RTC_ALARM_TIME_MASK_WEEKDAY) ? ((bin2bcd(timeptr->tm_wday) & ALARM_1_DAY_DATE_MASK) | ALARM_1_DAY_DATE_OP_MASK) & ~ALARM_1_DAY_DATE_ENABLE_MASK : (bin2bcd(timeptr->tm_wday) & ALARM_1_DAY_DATE_MASK) | ALARM_1_DAY_DATE_OP_MASK | ALARM_1_DAY_DATE_ENABLE_MASK;
	}
	else if(timeptr->tm_mday != NULL){
		raw_time[3] = (mask & RTC_ALARM_TIME_MASK_MONTHDAY) ? (bin2bcd(timeptr->tm_mday) & ALARM_1_DAY_DATE_FIELD_MASK &~ ALARM_1_DAY_DATE_OP_MASK) & ~ALARM_1_DAY_DATE_ENABLE_MASK : (bin2bcd(timeptr->tm_mday) & ALARM_1_DAY_DATE_FIELD_MASK &~ ALARM_1_DAY_DATE_OP_MASK) | ALARM_1_DAY_DATE_ENABLE_MASK;
	}
	else{
		raw_time[3] = 0x80; // Disable day/date alarm if neither is set
	}

	raw_time[4] = (mask & RTC_ALARM_TIME_MASK_MONTH) ? (bin2bcd(timeptr->tm_mon) & ALARM_1_MONTH_FIELD_MASK) & ~ALARM_1_MONTH_ENABLE_MASK : (bin2bcd(timeptr->tm_mon) & ALARM_1_MONTH_FIELD_MASK | ALARM_1_MONTH_ENABLE_MASK);
	
	if (mask & RTC_ALARM_TIME_MASK_YEAR) {
			raw_time[5] = (bin2bcd(timeptr->tm_year) & ALARM_1_YEAR_FIELD_MASK);
		} 
	else{
		raw_time[4] |= ALARM_1_YEAR_ENABLE_MASK; // Set year bit for 21st century
		raw_time[5] = (bin2bcd(timeptr->tm_year) & ALARM_1_YEAR_FIELD_MASK);
	}
	ret = max31331_reg_write_multiple(dev, MAX31331_ALARM_1_SECONDS, raw_time, sizeof(raw_time));
	if (ret) {
		LOG_ERR("Error when setting alarm: %i", ret);
		return ret;
	}
	if (mask != 0) {
		// Enable the alarm interrupt
		ret = max31331_reg_update(dev, MAX31331_INTERRUPT_ENABLE, ALARM_1_INTERRUPT_ENABLE_MASK, 1);
		if (ret) {
			LOG_ERR("Error enabling alarm interrupt: %i", ret);
			return ret;
		}
	} else {
		// Disable the alarm interrupt if no mask is set
		ret = max31331_reg_update(dev, MAX31331_INTERRUPT_ENABLE, ALARM_1_INTERRUPT_ENABLE_MASK, 0);
		if (ret) {
			LOG_ERR("Error disabling alarm interrupt: %i", ret);
			return ret;
		}
	}
	}
	else if (id == 2){
		raw_time[0] = (mask & RTC_ALARM_TIME_MASK_MINUTE) ? (bin2bcd(timeptr->tm_min) & ALARM_2_MINUTES_FIELD_MASK) & ~ALARM_2_MINUTES_ENABLE_MASK : (bin2bcd(timeptr->tm_min) & ALARM_2_MINUTES_FIELD_MASK) | ALARM_2_MINUTES_ENABLE_MASK;
		raw_time[1] = (mask & RTC_ALARM_TIME_MASK_HOUR)   ? (bin2bcd(timeptr->tm_hour) & ALARM_2_HOURS_FIELD_MASK) & ~ALARM_2_HOURS_ENABLE_MASK     : (bin2bcd(timeptr->tm_hour) & ALARM_2_HOURS_FIELD_MASK) | ALARM_2_HOURS_ENABLE_MASK;
		
		if(timeptr->tm_wday != NULL){
			raw_time[2] = (mask & RTC_ALARM_TIME_MASK_WEEKDAY) ? ((bin2bcd(timeptr->tm_wday) & ALARM_2_DAY_DATE_MASK) | ALARM_2_DAY_DATE_OP_MASK) & ~ALARM_2_DAY_DATE_ENABLE_MASK : (bin2bcd(timeptr->tm_wday) & ALARM_2_DAY_DATE_MASK) | ALARM_2_DAY_DATE_OP_MASK | ALARM_2_DAY_DATE_ENABLE_MASK;
		}
		else if(timeptr->tm_mday != NULL){
			raw_time[2] = (mask & RTC_ALARM_TIME_MASK_MONTHDAY) ? (bin2bcd(timeptr->tm_mday) & ALARM_2_DAY_DATE_FIELD_MASK &~ ALARM_2_DAY_DATE_OP_MASK) & ~ALARM_2_DAY_DATE_ENABLE_MASK : (bin2bcd(timeptr->tm_mday) & ALARM_2_DAY_DATE_FIELD_MASK &~ ALARM_2_DAY_DATE_OP_MASK) | ALARM_2_DAY_DATE_ENABLE_MASK;
		}
		else{
			raw_time[2] = 0x80; // Disable day/date alarm if neither is set
		}

		ret = max31331_reg_write_multiple(dev, MAX31331_ALARM_2_MINUTES, raw_time, 3);
		if (ret) {
			LOG_ERR("Error when setting alarm: %i", ret);
			return ret;
		}
		ret = max31331_reg_update(dev, MAX31331_INTERRUPT_ENABLE, ALARM_2_INTERRUPT_ENABLE_MASK, 1);
		if (ret) {
			LOG_ERR("Error enabling alarm interrupt: %i", ret);
			return ret;
		}
	}
	else{
		LOG_ERR("Invalid Alarm ID: %d", id);
		return -EINVAL;
	}
	return 0;
}

static int rtc_max31331_alarm_get_time(const struct device *dev, uint16_t id, uint16_t *mask, struct rtc_time *timeptr)
{
	const struct rtc_max31331_config *config = dev->config;
	int ret;
	uint8_t raw_time[6];

	if (id <= 0 || id > 2) {
		LOG_ERR("invalid ID %d", id);
		return -EINVAL;
	}

	if (id == 1){
		ret = max31331_reg_read(dev, MAX31331_ALARM_1_SECONDS, raw_time, sizeof(raw_time));
		if (ret) {
			LOG_ERR("Error when getting alarm time: %i", ret);
			return ret;
		}

		*mask = 0;
		if (!(raw_time[0] & ALARM_1_SECONDS_ENABLE_MASK)) {
			timeptr->tm_sec = bcd2bin(raw_time[0] & ALARM_1_SECONDS_FIELD_MASK);
			*mask |= RTC_ALARM_TIME_MASK_SECOND;
		}
		if (!(raw_time[1] & ALARM_1_MINUTES_ENABLE_MASK)) {
			timeptr->tm_min = bcd2bin(raw_time[1] & ALARM_1_MINUTES_FIELD_MASK);
			*mask |= RTC_ALARM_TIME_MASK_MINUTE;
		}
		if (!(raw_time[2] & ALARM_1_HOURS_ENABLE_MASK)) {
			timeptr->tm_hour = bcd2bin(raw_time[2] & ALARM_1_HOURS_FIELD_MASK);
			*mask |= RTC_ALARM_TIME_MASK_HOUR;
		}
		if (!(raw_time[3] & ALARM_1_DAY_DATE_ENABLE_MASK)) {
			if (raw_time[3] & ALARM_1_DAY_DATE_OP_MASK) {
				timeptr->tm_wday = bcd2bin(raw_time[3] & ALARM_1_DAY_DATE_MASK) + MAX31331_DAY_OFFSET;
				*mask |= RTC_ALARM_TIME_MASK_WEEKDAY;
			} else {
				timeptr->tm_mday = bcd2bin(raw_time[3] & ALARM_1_DAY_DATE_MASK);
				*mask |= RTC_ALARM_TIME_MASK_MONTHDAY;
			}
		}
		if (!(raw_time[4] & ALARM_1_MONTH_ENABLE_MASK)) {
			timeptr->tm_mon = bcd2bin(raw_time[4] & ALARM_1_MONTH_FIELD_MASK);
			*mask |= RTC_ALARM_TIME_MASK_MONTH;
		}
		if (!(raw_time[4] & ALARM_1_YEAR_ENABLE_MASK)) {
			timeptr->tm_year = bcd2bin(raw_time[5] & ALARM_1_YEAR_FIELD_MASK);
			*mask |= RTC_ALARM_TIME_MASK_YEAR;
		}
	}
	else if (id == 2){
		ret = max31331_reg_read(dev, MAX31331_ALARM_2_MINUTES, raw_time, 3);
		if (ret) {
			LOG_ERR("Error when getting alarm time: %i", ret);
			return ret;
		}

		*mask = 0;
		if (!(raw_time[0] & ALARM_2_MINUTES_ENABLE_MASK)) {
			timeptr->tm_min = bcd2bin(raw_time[0] & ALARM_2_MINUTES_FIELD_MASK);
			*mask |= RTC_ALARM_TIME_MASK_MINUTE;
		}
		if (!(raw_time[1] & ALARM_2_HOURS_ENABLE_MASK)) {
			timeptr->tm_hour = bcd2bin(raw_time[1] & ALARM_2_HOURS_FIELD_MASK);
			*mask |= RTC_ALARM_TIME_MASK_HOUR;
		}
		if (!(raw_time[2] & ALARM_2_DAY_DATE_ENABLE_MASK)) {
			if (raw_time[2] & ALARM_2_DAY_DATE_OP_MASK) {
				timeptr->tm_wday = bcd2bin(raw_time[2] & ALARM_2_DAY_DATE_MASK) + MAX31331_DAY_OFFSET;
				*mask |= RTC_ALARM_TIME_MASK_WEEKDAY;
			} else {
				timeptr->tm_mday = bcd2bin(raw_time[2] & ALARM_2_DAY_DATE_MASK);
				*mask |= RTC_ALARM_TIME_MASK_MONTHDAY;
			}
		}
	}
	else{
		LOG_ERR("Invalid Alarm ID: %d", id);
		return -EINVAL;
	}
	return 0;
}

#endif

static int rtc_max31331_init(const struct device *dev)
{
	const struct rtc_max31331_config *config = dev->config;
	uint8_t val;
	int ret;

	if (!i2c_is_ready_dt(&config->i2c)) {
		printk("I2C bus not ready");
		return -ENODEV;
	}

	/** RESET REGISTERS */
	ret = max31331_reg_write(dev, MAX31331_RTC_RESET, SW_RESET_MASK);
	if (ret) {
		LOG_ERR("Failed to reset the device");
		return ret;
	}

	ret = max31331_reg_write(dev, MAX31331_RTC_RESET, 0);
	if (ret) {
		LOG_ERR("Failed to reset the device");
		return ret;
	}


	/** Enable Oscillator */
	ret = max31331_reg_update(dev, MAX31331_RTC_CONFIG1, ENABLE_OSCILLATOR_MASK , 1);
	if (ret) {
		LOG_ERR("Failed to enable oscillator");
		return ret;
	}

	return 0;
}

static DEVICE_API(rtc, rtc_max31331) =
{
	.set_time = rtc_max31331_set_time,
	.get_time = rtc_max31331_get_time,
#ifdef CONFIG_RTC_ALARM
	.alarm_set_time = rtc_max31331_alarm_set_time,
	.alarm_get_time = rtc_max31331_alarm_get_time,
	// .alarm_is_pending = rtc_max31331_alarm_is_pending,
	// .alarm_set_callback = rtc_max31331_alarm_set_callback,
	// .alarm_get_supported_fields = rtc_max31331_alarm_get_supported_fields,
#endif
#ifdef CONFIG_RTC_UPDATE
	// .update_set_callback = rtc_max31331_update_set_callback,
#endif
#ifdef CONFIG_RTC_CALIBRATION
	// .set_calibration = rtc_max31331_set_calibration,
	// .get_calibration = rtc_max31331_get_calibration,
#endif
};

#define RTC_MAX31331_DEFINE(inst)                                                                  \
	static const struct rtc_max31331_config rtc_max31331_config##inst = {                      \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, rtc_max31331_init, NULL, NULL, &rtc_max31331_config##inst,     \
			      POST_KERNEL, CONFIG_RTC_INIT_PRIORITY, &rtc_max31331);

DT_INST_FOREACH_STATUS_OKAY(RTC_MAX31331_DEFINE)
