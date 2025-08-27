/*
 * Copyright (c) 2025 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ADI_MAX40109_MAX40109_H_
#define ZEPHYR_DRIVERS_SENSOR_ADI_MAX40109_MAX40109_H_

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor/max40109.h>
#include <zephyr/types.h>
#include <zephyr/sys/byteorder.h>

/** Register addresses for MAX40109 */

#define MAX40109_REG_CONFIG_MSB                        0x00u
#define MAX40109_REG_CONFIG_LSB                        0x01u
#define MAX40109_REG_STATUS_MSB                        0x02u
#define MAX40109_REG_STATUS_LSB                        0x03u
#define MAX40109_PGA_PRESSURE_GAIN                     0x04u
#define MAX40109_CURRENT_SOURCE                        0x05u
#define MAX40109_UNCALIBRATED_PRESSURE_MSB             0x06u
#define MAX40109_UNCALIBRATED_PRESSURE_LSB             0x07u
#define MAX40109_UNCALBIRATED_TEMPERATURE_MSB          0x08u
#define MAX40109_UNCALBIRATED_TEMPERATURE_LSB          0x09u
#define MAX40109_ADC_SAMPLE_RATE                       0x0Au
#define MAX40109_INTERRUPT_ENABLE_MSB                  0x0Bu
#define MAX40109_INTERRUPT_ENABLE_LSB                  0x0Cu
#define MAX40109_BRIDGE_DRIVE                          0x0Du
#define MAX40109_PGA_TEMPERATURE_GAIN                  0x0Eu
#define MAX40109_CALIBRATED_PRESSURE_MSB               0x0Fu
#define MAX40109_CALIBRATED_PRESSURE_LSB               0x10u
#define MAX40109_CALIBRATED_TEMPERATURE_MSB            0x11u
#define MAX40109_CALIBRATED_TEMPERATURE_LSB            0x12u
#define MAX40109_TEMP_MODE                             0x13u
#define MAX40109_SENSOR_OFFSET_CAL_CONFIG              0x14u
#define MAX40109_ANALOG_FILTER_BW                      0x15u
#define MAX40109_ZERO_PRESSURE_OFFSET_RAM_OVERRIDE_MSB 0x1Au
#define MAX40109_ZERO_PRESSURE_OFFSET_RAM_OVERRIDE_LSB 0x1Bu
#define MAX40109_ZERO_PRESSURE_OFFSET_SELECT           0x1Cu
#define MAX40109_ANALOG_OUTPUT_STAGE                   0x1Eu
#define MAX40109_SLP_MR                                0x9Bu
#define MAX40109_SLP_MREF                              0x9Du
#define MAX40109_CP_CONTROL_1                          0x9Fu
#define MAX40109_CP_CONTROL_2                          0xA0u
#define MAX40109_MTP_CONTROL                           0xA2u
#define MAX40109_MTP_STATUS                            0xA3u
#define MAX40109_MTP_PROT_ADDR                         0xA4u
#define MAX40109_MTP_PROT_WDATA_MSB                    0xA5u
#define MAX40109_MTP_PROT_WDATA_LSB                    0xA6u
#define MAX40109_MTP_PROT_RDATA_MSB                    0xA7u
#define MAX40109_MTP_PROT_RDATA_LSB                    0xA8u
#define MAX40109_MTP_LEVEL_MSB                         0xA9u
#define MAX40109_MTP_LEVEL_LSB                         0xAAu
#define MAX40109_SLP_MRV_MSB                           0xABu
#define MAX40109_SLP_MRV_LSB                           0xACu
#define MAX40109_SLP_MREFV_MSB                         0xADu
#define MAX40109_SLP_MREFV_LSB                         0xAEu
#define MAX40109_MTP_DATA0_MSB                         0xAFu
#define MAX40109_MTP_DATA0_LSB                         0xB0u
#define MAX40109_MTP_ADDR                              0xB1u

/** CONFIG Register Masks */

#define MTP_ENABLE_MASK                        BIT(7)
#define TEMP_CAL_BYPASS_MASK                   BIT(6)
#define PRESSURE_CAL_BYPASS_MASK               BIT(5)
#define DIGITAL_FILTER_MASK                    GENMASK(4, 2)
#define REFIN_INTERNAL_EXTERNAL_SEL_MASK       BIT(1)
#define CURRENT_SOURCE_REFERENCE_RESISTOR_MASK BIT(0)
#define SHUTDOWN_MASK                          BIT(7)
#define TEMP_CURRENT_MASK                      GENMASK(5, 4)
#define ALERT_MODE_MASK                        GENMASK(3, 1)
#define PGA_INPUT_MASK                         BIT(0)

/** STATUS Register Masks */
#define DRV_FAULT_MASK           BIT(3)
#define INT_FAULT_MASK           BIT(2)
#define TEMP_DATA_READY_MASK     BIT(1)
#define PRESSURE_DATA_READY_MASK BIT(0)
#define UV_DRV_MASK              BIT(7)
#define OV_DRV_MASK              BIT(6)
#define UV_INT_MASK              BIT(5)
#define OV_INT_MASK              BIT(4)
#define UV_INP_MINUS_MASK        BIT(3)
#define OV_INP_MINUS_MASK        BIT(2)
#define UV_INP_PLUS_MASK         BIT(1)
#define OV_INP_PLUS_MASK         BIT(0)

/** INTERRUPT ENABLE Register Masks */
#define INT_TEMP_DATA_READY_EN_MASK     BIT(1)
#define INT_PRESSURE_DATA_READY_EN_MASK BIT(0)
#define INT_UV_DRV_EN_MASK              BIT(7)
#define INT_OV_DRV_EN_MASK              BIT(6)
#define INT_UV_INT_EN_MASK              BIT(5)
#define INT_OV_INT_EN_MASK              BIT(4)
#define INT_UV_INP_MINUS_EN_MASK        BIT(3)
#define INT_OV_INP_MINUS_EN_MASK        BIT(2)
#define INT_UV_INP_PLUS_EN_MASK         BIT(1)
#define INT_OV_INP_PLUS_EN_MASK         BIT(0)

/** MTP Status Register Masks */
#define MTP_BURN_DONE_MASK         BIT(7)
#define MTP_ECC_ERR_2_BIT_MASK     BIT(6)
#define MTP_ECC_ERR_1_BIT_MASK     BIT(5)
#define MTP_VPP_INIT_FAIL_MASK     BIT(3)
#define MTP_FULL_MASK              BIT(2)
#define MTP_VERIFICATION_FAIL_MASK BIT(1)
#define MTP_VPP_ACT_MASK           BIT(0)

/** TEMP MODE Register Masks */
#define DRV_SCALE_MASK GENMASK(4, 3)
#define TEMP_MODE_MASK GENMASK(2, 0)

/** SENSOR OFFSET CAL CONFIG Register Masks */
#define CONNECT_TRIM_RESISTOR_MASK              BIT(2)
#define CONNECT_OFFSET_CALIBRATION_CURRENT_MASK BIT(1)
#define PGA_FUNCTIONALITY_MASK                  BIT(0)

/** MTP DATA LENGTH */
#define MAX40109_MTP_DATA_LENGTH 2

struct max40109_config {
	struct i2c_dt_spec i2c;

#if defined(CONFIG_MAX40109_TRIGGER)
	struct gpio_dt_spec interrupt_gpio;
#endif
	uint8_t digital_filter_setup;
	uint8_t alert_mode;
	uint8_t temp_current;
	uint8_t pga_pressure_gain;
	uint8_t current_source;
	uint8_t adc_sample_rate;
	uint8_t pga_temperature_gain;
	uint8_t analog_filter_bw_setup;
	uint8_t bridge_drive;
	uint8_t temp_mode;
	uint8_t drv_scale;
	uint8_t analog_filter_bw;
	uint8_t analog_output_stage;
	uint8_t pga_input_mux;

	/** Calibration Coefficients */

	/** Temperature Calibration Coefficients */
	float k0;
	float k1;
	float k2;
	float k3;

	/** Zeroth-Order Pressure Calibration Coefficient */
	float h0;
	float h1;
	float h2;
	float h3;

	/**	First-Order Pressure Calibration Coefficient */
	float g0;
	float g1;
	float g2;
	float g3;

	/** Second-Order Pressure Calibration Coefficient */
	float n0;
	float n1;
	float n2;
	float n3;

	/** Third-Order Pressure Calibration Coefficient */
	float m0;
	float m1;
	float m2;
	float m3;

	/** Overpressure Threshold Positive */
	uint8_t overpressure_threshold_positive;

	/** Underpressure Threshold Positive */
	uint8_t underpressure_threshold_positive;

	/** Overpressure Threshold Negative */
	uint8_t overpressure_threshold_negative;

	/** Underpressure Threshold Negative */
	uint8_t underpressure_threshold_negative;

	/** Overvoltage Temperature */
	uint8_t overvoltage_temperature;

	/** Undervoltage Temperature */
	uint8_t undervoltage_temperature;

	/** Overvoltage Drive */
	uint8_t overvoltage_drive;

	/** Undervoltage Drive */
	uint8_t undervoltage_drive;

	/** Primary Threshold Pressure Value */
	uint8_t primary_threshold_pressure_value;

	/** Hysteresis Threshold Pressure Value */
	uint8_t hysteresis_threshold_pressure_value;
};

struct max40109_data {
	uint8_t status_msb;
	uint8_t status_lsb;
	bool temp_cal_bypass;
	bool pressure_cal_bypass;
	uint16_t uncalibrated_pressure;
	uint16_t uncalibrated_temperature;
	uint16_t calibrated_pressure;
	uint16_t calibrated_temperature;

#ifdef CONFIG_MAX40109_TRIGGER
	const struct device *dev;
	struct gpio_callback gpio_cb;
	struct k_sem gpio_sem;

	sensor_trigger_handler_t drv_fault_handler;
	sensor_trigger_handler_t int_fault_handler;
	sensor_trigger_handler_t temp_data_rdy_handler;
	sensor_trigger_handler_t pressure_data_rdy_handler;
	sensor_trigger_handler_t uv_drv_handler;
	sensor_trigger_handler_t ov_drv_handler;
	sensor_trigger_handler_t uv_int_handler;
	sensor_trigger_handler_t ov_int_handler;
	sensor_trigger_handler_t uv_inp_minus_handler;
	sensor_trigger_handler_t ov_inp_minus_handler;
	sensor_trigger_handler_t uv_inp_plus_handler;
	sensor_trigger_handler_t ov_inp_plus_handler;

#if defined(CONFIG_MAX40109_TRIGGER_OWN_THREAD)
	struct k_thread thread;
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_MAX40109_THREAD_STACK_SIZE);
#elif defined(CONFIG_MAX40109_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
#endif
#endif
};

#endif
