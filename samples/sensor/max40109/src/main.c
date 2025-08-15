/*
 * Copyright (c) 2025 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/max40109.h>
#include <zephyr/drivers/i2c.h>

#define MAX40109_NODE    DT_NODELABEL(max40109)
#define TRIGGER_SET_TEST 1

static const struct device *max40109_dev = DEVICE_DT_GET(MAX40109_NODE);

int main(void)
{
	printf("Starting MAX40109 sample application\n");
	if (!device_is_ready(max40109_dev)) {
		printf("MAX40109 device not ready\n");
		return -ENODEV;
	}
	printf("MAX40109 device is ready: %s\n", max40109_dev->name);

    uint8_t interrupt_enable_reg [2];
    uint16_t int_setup;

    int ret = 0;

    ret = max40109_reg_read(max40109_dev, 0x0B, interrupt_enable_reg, 2);
    if (ret < 0) {
        printf("Failed to read interrupt enable register: %d\n", ret);
        return ret;
    }
    int_setup = (interrupt_enable_reg[0] << 8) | interrupt_enable_reg[1];
    printf("Interrupt Enable Register: 0x%04X\n", int_setup);
    
    uint8_t calibrated_pressure[2];
    ret = max40109_reg_read(max40109_dev, 0x0F, calibrated_pressure, 2);
    
    if (ret < 0) {
        printf("Failed to read calibrated pressure: %d\n", ret);
        return ret;
    }


    uint16_t pressure = (calibrated_pressure[0] << 8) | calibrated_pressure[1];
    printf("Calibrated Pressure: %d\n", pressure);

    uint8_t uncalibrated_pressure[2];

    ret = max40109_reg_read(max40109_dev, 0x06, uncalibrated_pressure, 2);
    if (ret < 0) {
        printf("Failed to read uncalibrated pressure: %d\n", ret);
        return ret;
    }
    uint16_t uncalibrated_pressure_value = (uncalibrated_pressure[0] << 8) | uncalibrated_pressure[1];
    printf("Uncalibrated Pressure: %d\n", uncalibrated_pressure_value);

}