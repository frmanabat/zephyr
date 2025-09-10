/*
 * Copyright (c) 2025, Francis Roi Manabat
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/drivers/rtc/rtc_max31331.h>
#include <zephyr/sys/util.h>

#define RTC_DEVICE_NODE DT_NODELABEL(rtc_max31331)
#define PBUTTON_NODE    DT_NODELABEL(user_button)
#define TEST_OUT        DT_ALIAS(outgpio)
static const struct device *rtc_max31331 = DEVICE_DT_GET(RTC_DEVICE_NODE);
static const struct gpio_dt_spec pbutton = GPIO_DT_SPEC_GET(PBUTTON_NODE, gpios);
static const struct gpio_dt_spec test_out = GPIO_DT_SPEC_GET(TEST_OUT, gpios);

#define TASK1_STACK_SIZE 1024ul
static K_THREAD_STACK_DEFINE(thread_stack, TASK1_STACK_SIZE);
static struct k_thread thread_id;

#define TASK2_STACK_SIZE 1024ul
static K_THREAD_STACK_DEFINE(din_stack, TASK2_STACK_SIZE);
static struct k_thread din_thread_id;
int count = 0;

struct rtc_time timestamps[4];

static void alarm_callback(const struct device *dev, int alarm_id, void *user_data)
{
	printk("Alarm %d triggered!\n", alarm_id);
}

static void thread_entry(void *p1, void *p2, void *p3)
{
	while (1) {
		k_sleep(K_MSEC(1000));
		int ret = 0;
		struct rtc_time time;
		ret = rtc_get_time(rtc_max31331, &time);
		if (ret) {
			printk("Error getting time: %d\n", ret);
			continue;
		}
		printk("Thread: Current Date and Time: %04d-%02d-%02d %02d:%02d:%02d\n",
		       time.tm_year + 2000, time.tm_mon + 1, time.tm_mday, time.tm_hour,
		       time.tm_min, time.tm_sec);
	}
}

static void din_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	printk("DIN Event Detected!\n");
}

static void din_test(void *p1, void *p2, void *p3)
{
	int ret = 0;
	while (1) {
		static int last_state = 0;
		static int debounce_counter = 0;
		static int count = 0;

		int current_state = gpio_pin_get_dt(&pbutton);

		if (current_state != last_state) {
			debounce_counter++;
			if (debounce_counter >= 5) { // Debounce threshold
				last_state = current_state;
				debounce_counter = 0;

				if (current_state) {
					gpio_pin_set_dt(&test_out, 0);
					count++;
					printk("Button pressed %d times\n", count);
				} else {
					gpio_pin_set_dt(&test_out, 1);
				}
			}
		} else {
			debounce_counter = 0;
		}

		if (count >= 4) {
			for (int i = 0; i < 4; i++) {
				uint8_t flags = 0;
				struct rtc_time ts;
				ret = rtc_max31331_get_timestamps(rtc_max31331, &ts, i, &flags);
				if (ret) {
					printk("Error getting timestamp %d: %d\n", i, ret);
					continue;
				}
				timestamps[i] = ts;
			}
			printk("Timestamps retrieved\n");
			printk("Timestamps 0: %04d-%02d-%02d %02d:%02d:%02d\n",
			       timestamps[0].tm_year + 2000, timestamps[0].tm_mon + 1,
			       timestamps[0].tm_mday, timestamps[0].tm_hour, timestamps[0].tm_min,
			       timestamps[0].tm_sec);
			printk("Timestamps 1: %04d-%02d-%02d %02d:%02d:%02d\n",
			       timestamps[1].tm_year + 2000, timestamps[1].tm_mon + 1,
			       timestamps[1].tm_mday, timestamps[1].tm_hour, timestamps[1].tm_min,
			       timestamps[1].tm_sec);
			printk("Timestamps 2: %04d-%02d-%02d %02d:%02d:%02d\n",
			       timestamps[2].tm_year + 2000, timestamps[2].tm_mon + 1,
			       timestamps[2].tm_mday, timestamps[2].tm_hour, timestamps[2].tm_min,
			       timestamps[2].tm_sec);
			printk("Timestamps 3: %04d-%02d-%02d %02d:%02d:%02d\n",
			       timestamps[3].tm_year + 2000, timestamps[3].tm_mon + 1,
			       timestamps[3].tm_mday, timestamps[3].tm_hour, timestamps[3].tm_min,
			       timestamps[3].tm_sec);
			count = 0;
		}
		k_sleep(K_MSEC(50));
	}
}

int main(void)
{
	/* Check if the RTC is ready */
	if (!device_is_ready(rtc_max31331)) {
		printk("Device is not ready\n");
		return 0;
	}

	if (!gpio_is_ready_dt(&pbutton)) {
		printk("Button device not ready\n");
		return 0;
	}

	if (!gpio_is_ready_dt(&test_out)) {
		printk("Test output device not ready\n");
		return 0;
	}

	printk("Device is ready: RTC_MAX31331\n");
	int ret = 0;
	uint8_t val = 0;

	ret = gpio_pin_configure_dt(&pbutton, GPIO_INPUT);
	if (ret) {
		printk("Error configuring button pin: %d\n", ret);
		return ret;
	}

	ret = gpio_pin_configure_dt(&test_out, GPIO_OUTPUT_HIGH);
	if (ret) {
		printk("Error configuring test output pin: %d\n", ret);
		return ret;
	}

	ret = max31331_reg_read(rtc_max31331, 0x02, &val, 1);

	printk("Reset Register: 0x%02X\n", val);
	k_sleep(K_MSEC(2000));

	struct rtc_time time;

	// Set time to 2024-06-01 12:00:00
	time.tm_year = 2024 - 2000; // Year since 1900
	time.tm_mon = 6 - 1;        // Month [0-11]
	time.tm_mday = 1;           // Day of the month [1-31]
	time.tm_hour = 12;          // Hours [0-23]
	time.tm_min = 0;            // Minutes [0-59]
	time.tm_sec = 40;           // Seconds [0-59]
	time.tm_wday = 6;           // Day of the week [0-6] (0 = Sunday)
	time.tm_yday = -1;          // Day of the year [0-365] (not used)
	time.tm_isdst = -1;         // Daylight saving time flag (not used)
	time.tm_nsec = 0;           // Nanoseconds (not used)
	ret = rtc_set_time(rtc_max31331, &time);
	if (ret) {
		printk("Error setting time: %d\n", ret);
		return ret;
	}
	printk("Time set to: %04d-%02d-%02d %02d:%02d:%02d\n", time.tm_year + 2000, time.tm_mon + 1,
	       time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec);
	k_sleep(K_MSEC(1000));
	// /* Function to get and print the current date and time from the RTC */
	ret = rtc_get_time(rtc_max31331, &time);
	if (ret) {
		printk("Error getting time: %d\n", ret);
		return ret;
	}
	printk("Current Date and Time: %04d-%02d-%02d %02d:%02d:%02d\n", time.tm_year + 2000,
	       time.tm_mon + 1, time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec);

	ret = rtc_alarm_set_callback(rtc_max31331, 1, alarm_callback, NULL);
	if (ret) {
		printk("Error setting alarm callback: %d\n", ret);
		return ret;
	}

	struct rtc_time alarm_time;
	alarm_time.tm_year = 2024 - 2000; // Year since 1900
	alarm_time.tm_mon = 6 - 1;        // Month [0-11]
	alarm_time.tm_mday = 1;           // Day of the month [1-31]
	alarm_time.tm_hour = 12;          // Hours [0-23]
	alarm_time.tm_wday = NULL;
	alarm_time.tm_min = 0; // Minutes [0-59]
	alarm_time.tm_sec = 50;
	ret = rtc_alarm_set_time(rtc_max31331, 1, RTC_ALARM_TIME_MASK_SECOND, &alarm_time);

	if (ret) {
		printk("Error setting alarm: %d\n", ret);
		return ret;
	}

	ret = max31331_reg_read(rtc_max31331, 0x01, &val, 1);
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

	ret = rtc_alarm_set_callback(rtc_max31331, 1, alarm_callback, NULL);

	ret = rtc_max31331_set_timestamp_callback(rtc_max31331, din_callback, NULL);
	if (ret) {
		printk("Error setting DIN callback: %d\n", ret);
		return ret;
	}

	k_thread_create(&thread_id, thread_stack, K_THREAD_STACK_SIZEOF(thread_stack), thread_entry,
			NULL, NULL, NULL, 7, 0, K_NO_WAIT);

	// DIN test thread
	k_thread_create(&din_thread_id, din_stack, K_THREAD_STACK_SIZEOF(din_stack), din_test, NULL,
			NULL, NULL, 7, 0, K_NO_WAIT);

	k_thread_start(&thread_id);
	k_thread_start(&din_thread_id);
	return 0;
}
