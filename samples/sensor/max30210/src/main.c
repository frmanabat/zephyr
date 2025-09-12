/*
 * Copyright (c) 2025 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// #include <zephyr/drivers/sensor/max30011.h>
#include <stdio.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/max30210.h>
#include <zephyr/drivers/i2c.h>

#define MAX30210_NODE    DT_NODELABEL(max30210)
#define TRIGGER_SET_TEST 1

static const struct device *max30210_dev = DEVICE_DT_GET(MAX30210_NODE);


#define TASK_STACK_SIZE 1024
K_THREAD_STACK_DEFINE(thread_stack, TASK_STACK_SIZE);
struct k_thread thread_id;

void handler(const struct device *dev, const struct sensor_trigger *trig)
{
	struct sensor_value temp_value;
	printf("Interrupt triggered: %s\n", dev->name);
	int ret = sensor_sample_fetch(dev);
	if (ret < 0) {
		printf("Failed to fetch sample: %d\n", ret);
		return;
	}
	ret = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp_value);
	if (ret < 0) {
		printf("Failed to get temperature channel: %d\n", ret);
		return;
	}
	uint8_t interrupt_status = 0;
	// ret = max30210_reg_read(dev, 0x00, &interrupt_status, 1); // Read interrupt status
	// register
	printf("Temperature is too High: %f C \n", sensor_value_to_float(&temp_value));
}

static void thread_sim(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	while (1) {
	printf("Simulated thread running...\n");
	k_sleep(K_MSEC(500));
	}
}

void new_temp_handler(const struct device *dev, const struct sensor_trigger *trig)
{
	struct sensor_value temp_value;
	printf("New temperature handler triggered: %s\n", dev->name);
	int ret = sensor_sample_fetch(dev);
	if (ret < 0) {
		printf("Failed to fetch sample: %d\n", ret);
		return;
	}
	ret = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp_value);
	if (ret < 0) {
		printf("Failed to get temperature channel: %d\n", ret);
		return;
	}
	printf("New Temperature is: %f C \n", sensor_value_to_float(&temp_value));
	// Read interrupt status register
}

int main(void)
{
	printf("Starting MAX30210 sample application\n");
	if (!device_is_ready(max30210_dev)) {
		printf("MAX30210 device not ready\n");
		return -ENODEV;
	}
	printf("MAX30210 device is ready: %s\n", max30210_dev->name);

	int ret;

	struct sensor_trigger trig = {.type = SENSOR_TRIG_THRESHOLD,
				      .chan = SENSOR_CHAN_AMBIENT_TEMP};

	ret = sensor_trigger_set(max30210_dev, &trig, handler);
	if (ret < 0) {
		printf("Failed to set trigger: %d\n", ret);
		return ret;
	}

	struct sensor_trigger new_temp_trig = {.type = SENSOR_TRIG_DATA_READY,
					       .chan = SENSOR_CHAN_AMBIENT_TEMP};

	ret = sensor_trigger_set(max30210_dev, &new_temp_trig, new_temp_handler);
	if (ret < 0) {
		printf("Failed to set trigger: %d\n", ret);
		return ret;
	}
	uint8_t interrupt_enable_setup = 0x00; // Initialize to 0x00
	ret = max30210_reg_read(max30210_dev, 0x02, &interrupt_enable_setup, 1);

	printf("Interrupt Enable Setup: 0x%02X\n", interrupt_enable_setup);
	
	k_thread_create(&thread_id, thread_stack, TASK_STACK_SIZE,
			thread_sim, NULL, NULL, NULL,
			7, 0, K_NO_WAIT);

	k_thread_name_set(&thread_id, "simulated_thread");

	k_thread_start(&thread_id);
	
	return 0;

}
