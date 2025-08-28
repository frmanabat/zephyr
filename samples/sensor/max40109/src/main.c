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
    int ret = 0;
#if 0
    uint8_t interrupt_enable_reg [2];
    uint16_t int_setup;



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

    uint8_t mtp_control_reg = 0x10;
    ret = max40109_reg_write(max40109_dev, 0xA2, mtp_control_reg);
    uint8_t diag_data0 [2];
    ret = max40109_mtp_read(max40109_dev, 0x38, diag_data0);
    if (ret < 0) {
        printf("Failed to read diagnostic data: %d\n", ret);
        return ret;
    }


    uint16_t diag_data0_value = (diag_data0[0] << 8) | diag_data0[1];
    printf("Diagnostic Data 0: 0x%04X\n", diag_data0_value);
#endif
    // ret = max40109_mtp_initialize(max40109_dev);
    // if (ret < 0) {
    //     printf("Failed to initialize MTP: %d\n", ret);
    //     return ret;
    // }
    // printf("MTP initialized successfully\n");

#if 0 
    uint8_t mtp_data[2] = {0x12, 0x34};
    ret = max40109_mtp_write(max40109_dev, SP_DATA3, mtp_data);
    if (ret < 0) {
        printf("Failed to write MTP data: %d\n", ret);
        return ret;
    }
    printf("MTP data written successfully\n");
    uint8_t mtp_data_read [2] = {0,0};
    ret = max40109_mtp_read(max40109_dev, SP_DATA3, mtp_data_read);
    if (ret < 0) {
        printf("Failed to read MTP data: %d\n", ret);
        return ret;
    }
    printf("MTP Data Read: 0x%02X 0x%02X\n", mtp_data_read[0], mtp_data_read[1]);
#endif
#if 0
    float k0_value = 1.234;
    ret = max40109_mtp_calibration(max40109_dev, MAX40109_CALIBRATION_K0, k0_value, false);
    if (ret < 0) {
        printf("Failed to set calibration coefficient K0: %d\n", ret);
        return ret;
    }
    printf("Calibration coefficient K0 set successfully to %f\n", k0_value);

    float k0_read_value = 0.0;
    ret = max40109_mtp_calibration_read(max40109_dev, MAX40109_CALIBRATION_K0, &k0_read_value);
    if (ret < 0) {
        printf("Failed to read calibration coefficient K0: %d\n", ret);
        return ret;
    }
    printf("Calibration coefficient K0 read successfully: %f\n", k0_read_value);
    return 0;
#endif

// // #if 0
//     uint8_t data_burn[2] = {0x56, 0x78};
//     ret = max40109_mtp_burn (max40109_dev, SP_DATA4, data_burn);
//     if (ret < 0) {
//         printf("Failed to burn MTP data: %d\n", ret);
//         return ret;
//     }
//     printf("MTP data burned successfully\n");
//     return 0;
// // #endif
#if 0 
    uint8_t buff_burn [2] = {0,0};
    ret = max40109_mtp_read(max40109_dev, SP_DATA4, buff_burn);
    if (ret < 0) {
        printf("Failed to read MTP data: %d\n", ret);
        return ret;
    }
    printf("MTP Data Read After Burn: 0x%02X 0x%02X\n", buff_burn[0], buff_burn[1]);
    return 0;
#endif
    uint8_t val [2] = {0xFF, 0xCD};
    ret = max40109_mtp_write(max40109_dev, SP_DATA5, val);
    if (ret < 0) {
        printf("Failed to write MTP data: %d\n", ret);
        return ret;
    }
    printf("MTP data written successfully\n");
    uint8_t val_read [2] = {0,0};
    ret = max40109_mtp_read(max40109_dev, SP_DATA5, val_read);
    if (ret < 0) {
        printf("Failed to read MTP data: %d\n", ret);
        return ret;
    }
    printf("MTP Data Read: 0x%02X 0x%02X\n", val_read[0], val_read[1]);

    ret = max40109_mtp_update(max40109_dev, SP_DATA5, GENMASK(15,8), 0xAB);
    if (ret < 0) {
        printf("Failed to update MTP data: %d\n", ret);
        return ret;
    }
    printf("MTP data updated successfully\n");

    uint8_t val_updated [2] = {0,0};
    ret = max40109_mtp_read(max40109_dev, SP_DATA5, val_updated);
    if (ret < 0) {
        printf("Failed to read MTP data: %d\n", ret);
        return ret;
    }
    printf("MTP Data Read After Update: 0x%02X 0x%02X\n", val_updated[0], val_updated[1]);

    ret = max40109_mtp_calibration(max40109_dev, MAX40109_CALIBRATION_H0, 3.14159, false);
    if (ret < 0) {
        printf("Failed to set calibration coefficient H0: %d\n", ret);
        return ret;
    }
    printf("Calibration coefficient H0 set successfully to %f\n", 3.14159);

    float h0_value = 0.0;

    ret = max40109_mtp_calibration_read(max40109_dev, MAX40109_CALIBRATION_H0, &h0_value);
    if (ret < 0) {
        printf("Failed to read calibration coefficient H0: %d\n", ret);
        return ret;
    }
    printf("Calibration coefficient H0 read successfully: %f\n", h0_value);
    return 0;
    
}