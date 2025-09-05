/*
 * Copyright (c) 2024, Muhammad Waleed Badar
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/sys/util.h>

#define RTC_DEVICE_NODE DT_NODELABEL(rtc_max31331)
static const struct device *rtc_max31331 = DEVICE_DT_GET(RTC_DEVICE_NODE);

int main(void)
{
	/* Check if the RTC is ready */
	if (!device_is_ready(rtc_max31331)) {
		printk("Device is not ready\n");
		return 0;
	}
	printk("Device is ready: RTC_MAX31331\n");
	int ret = 0;
	uint8_t val = 0;
	ret = max31331_reg_read(rtc_max31331, 0x02, &val, 1);

	printk("Reset Register: 0x%02X\n", val);
	k_sleep(K_MSEC(2000));
	
	struct rtc_time time;

	//Set time to 2024-06-01 12:00:00
	time.tm_year = 2024 - 2000; // Year since 1900
	time.tm_mon = 6 - 1;        // Month [0-11]
	time.tm_mday = 1;          // Day of the month [1-31]
	time.tm_hour = 12;         // Hours [0-23]
	time.tm_min = 0;           // Minutes [0-59]
	time.tm_sec = 40;           // Seconds [0-59]
	time.tm_wday = 6;          // Day of the week [0-6] (0 = Sunday)
	time.tm_yday = -1;         // Day of the year [0-365] (not used)
	time.tm_isdst = -1;        // Daylight saving time flag (not used)
	time.tm_nsec = 0;          // Nanoseconds (not used)
	ret = rtc_set_time(rtc_max31331, &time);
	if (ret) {
		printk("Error setting time: %d\n", ret);
		return ret;
	}
	printk("Time set to: %04d-%02d-%02d %02d:%02d:%02d\n",
	       time.tm_year + 2000, time.tm_mon + 1, time.tm_mday,
	       time.tm_hour, time.tm_min, time.tm_sec);
	k_sleep(K_MSEC(1000));
	// /* Function to get and print the current date and time from the RTC */
	ret = rtc_get_time(rtc_max31331, &time);
	if (ret) {
		printk("Error getting time: %d\n", ret);
		return ret;
	}
	printk("Current Date and Time: %04d-%02d-%02d %02d:%02d:%02d\n",
	       time.tm_year + 2000, time.tm_mon + 1, time.tm_mday,
	       time.tm_hour, time.tm_min, time.tm_sec);

	struct rtc_time alarm_time;
	alarm_time.tm_year = 2024 - 2000; // Year since 1900
	alarm_time.tm_mon = 6 - 1;        // Month [0-11]
	alarm_time.tm_mday = 1;          // Day of the month [1-31]
	alarm_time.tm_hour = 12;         // Hours [0-23]
	alarm_time.tm_wday = NULL;
	alarm_time.tm_min = 0;           // Minutes [0-59]
	alarm_time.tm_sec = 50;
	ret = rtc_alarm_set_time(rtc_max31331, 1, RTC_ALARM_TIME_MASK_SECOND,
					       &alarm_time);

	if (ret) {
		printk("Error setting alarm: %d\n", ret);
		return ret;
	}

	ret =  max31331_reg_read(rtc_max31331, 0x01, &val, 1);
	if (ret) {
		printk("Error reading control register: %d\n", ret);
		return ret;
	}

	printk("Control Register after setting alarm: 0x%02X\n", val);
	
	ret = max31331_reg_read(rtc_max31331, 0x0F, &val, 1); // Enable the alarm interrupt
	if (ret) {
		printk("Error reading interrupt register: %d\n", ret);
		return ret;
	}
	printk("Seconds alarm1 reg: 0x%02X\n", val);
	uint16_t mask;

	ret = rtc_alarm_get_time(rtc_max31331, 1, &mask, &alarm_time);
	if (ret) {
		printk("Error getting alarm time: %d\n", ret);
		return ret;
	}

	printk("Alarm set to trigger at: %04d-%02d-%02d %02d:%02d:%02d\n",
	       alarm_time.tm_year + 2000, alarm_time.tm_mon + 1, alarm_time.tm_mday,
	       alarm_time.tm_hour, alarm_time.tm_min, alarm_time.tm_sec);

	// ret = rtc_alarm_get_time(rtc_max31331, 1, RTC_ALARM_TIME_MASK_SECOND, &alarm_time);
	// if (ret) {
	// 	printk("Error getting alarm time: %d\n", ret);
	// 	return ret;
	// }
	// printk("Alarm time retrieved: %04d-%02d-%02d %02d:%02d:%02d\n",
	//        alarm_time.tm_year + 2000, alarm_time.tm_mon + 1, alarm_time.tm_mday,
	//        alarm_time.tm_hour, alarm_time.tm_min, alarm_time.tm_sec);
	// /* Continuously read the current date and time from the RTC */
	// while (get_date_time(rtc) == 0) {
	// 	k_sleep(K_MSEC(1000));
	// };

	while (1) {
	ret = rtc_get_time(rtc_max31331, &time);
	if (ret) {
		printk("Error getting time: %d\n", ret);
		return ret;
	}
	printk("Current Date and Time: %04d-%02d-%02d %02d:%02d:%02d\n",
	       time.tm_year + 2000, time.tm_mon + 1, time.tm_mday,
	       time.tm_hour, time.tm_min, time.tm_sec);
		k_sleep(K_MSEC(1000));
	}
	return 0;
}
