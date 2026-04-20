/*
 * Copyright (c) 2026 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Devicetree bindings for the Analog Devices MAX86178 multi-sensor AFE
 *
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_SENSOR_MAX86178_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_SENSOR_MAX86178_H_

/**
 * @defgroup max86178 MAX86178 DT Options
 * @ingroup sensor_interface
 * @{
 */

/*******************************************************************************
 * CLOCK CONFIGURATION
 ******************************************************************************/

/**
 * @defgroup max86178_clk_ref Reference Clock Frequency
 * @{
 */
/** Reference clock: 32.000 kHz */
#define MAX86178_DT_REF_CLK_32000 0
/** Reference clock: 32.768 kHz */
#define MAX86178_DT_REF_CLK_32768 1
/** @} */

/**
 * @defgroup max86178_ref_clk_sel Reference Clock Source
 * @{
 */
/** Internal oscillator */
#define MAX86178_DT_REF_CLK_SEL_INTERNAL 0
/** External clock via CLKIN pin */
#define MAX86178_DT_REF_CLK_SEL_EXTERNAL 1
/** @} */

/**
 * @defgroup max86178_clk_fine_tune Clock Fine Tuning
 * @{
 */
#define MAX86178_DT_CLK_FINE_TUNE_SHIFT_0_0     0
#define MAX86178_DT_CLK_FINE_TUNE_SHIFT_0_2     1
#define MAX86178_DT_CLK_FINE_TUNE_SHIFT_0_4     2
#define MAX86178_DT_CLK_FINE_TUNE_SHIFT_0_6     3
#define MAX86178_DT_CLK_FINE_TUNE_SHIFT_0_8     4
#define MAX86178_DT_CLK_FINE_TUNE_SHIFT_1_0     5
#define MAX86178_DT_CLK_FINE_TUNE_SHIFT_1_2     6
#define MAX86178_DT_CLK_FINE_TUNE_SHIFT_1_4     7
#define MAX86178_DT_CLK_FINE_TUNE_SHIFT_1_6     8
#define MAX86178_DT_CLK_FINE_TUNE_SHIFT_1_8     9
#define MAX86178_DT_CLK_FINE_TUNE_SHIFT_2_0     10
#define MAX86178_DT_CLK_FINE_TUNE_SHIFT_2_2     11
#define MAX86178_DT_CLK_FINE_TUNE_SHIFT_2_4     12
#define MAX86178_DT_CLK_FINE_TUNE_SHIFT_2_6     13
#define MAX86178_DT_CLK_FINE_TUNE_SHIFT_2_8     14
#define MAX86178_DT_CLK_FINE_TUNE_SHIFT_3_0     15
#define MAX86178_DT_CLK_FINE_TUNE_SHIFT_NEG_3_2 16
#define MAX86178_DT_CLK_FINE_TUNE_SHIFT_NEG_3_0 17
#define MAX86178_DT_CLK_FINE_TUNE_SHIFT_NEG_2_8 18
#define MAX86178_DT_CLK_FINE_TUNE_SHIFT_NEG_2_6 19
#define MAX86178_DT_CLK_FINE_TUNE_SHIFT_NEG_2_4 20
#define MAX86178_DT_CLK_FINE_TUNE_SHIFT_NEG_2_2 21
#define MAX86178_DT_CLK_FINE_TUNE_SHIFT_NEG_2_0 22
#define MAX86178_DT_CLK_FINE_TUNE_SHIFT_NEG_1_8 23
#define MAX86178_DT_CLK_FINE_TUNE_SHIFT_NEG_1_6 24
#define MAX86178_DT_CLK_FINE_TUNE_SHIFT_NEG_1_4 25
#define MAX86178_DT_CLK_FINE_TUNE_SHIFT_NEG_1_2 26
#define MAX86178_DT_CLK_FINE_TUNE_SHIFT_NEG_1_0 27
#define MAX86178_DT_CLK_FINE_TUNE_SHIFT_NEG_0_8 28
#define MAX86178_DT_CLK_FINE_TUNE_SHIFT_NEG_0_6 29
#define MAX86178_DT_CLK_FINE_TUNE_SHIFT_NEG_0_4 30
#define MAX86178_DT_CLK_FINE_TUNE_SHIFT_NEG_0_2 31
/** @} */

/*******************************************************************************
 * ECG CLOCK CONFIGURATION
 ******************************************************************************/

/**
 * @defgroup max86178_ecg_dec_rate ECG Decimation Rate
 * @{
 */
/** ECG decimation: 16 */
#define MAX86178_DT_ECG_DEC_RATE_16  0
/** ECG decimation: 32 */
#define MAX86178_DT_ECG_DEC_RATE_32  1
/** ECG decimation: 64 */
#define MAX86178_DT_ECG_DEC_RATE_64  2
/** ECG decimation: 128 */
#define MAX86178_DT_ECG_DEC_RATE_128 3
/** ECG decimation: 256 */
#define MAX86178_DT_ECG_DEC_RATE_256 4
/** ECG decimation: 512 */
#define MAX86178_DT_ECG_DEC_RATE_512 5
/** @} */

/*******************************************************************************
 * BIOZ CLOCK CONFIGURATION
 ******************************************************************************/

/**
 * @defgroup max86178_bioz_ndiv BioZ N Divider
 * @{
 */
/** BioZ N divider: 256 */
#define MAX86178_DT_BIOZ_NDIV_256  0
/** BioZ N divider: 512 */
#define MAX86178_DT_BIOZ_NDIV_512  1
/** BioZ N divider: 1024 */
#define MAX86178_DT_BIOZ_NDIV_1024 2
/** @} */

/**
 * @defgroup max86178_bioz_kdiv BioZ K Divider
 * @{
 */
/** BioZ K divider: 1 */
#define MAX86178_DT_BIOZ_KDIV_1    0
/** BioZ K divider: 2 */
#define MAX86178_DT_BIOZ_KDIV_2    1
/** BioZ K divider: 4 */
#define MAX86178_DT_BIOZ_KDIV_4    2
/** BioZ K divider: 8 */
#define MAX86178_DT_BIOZ_KDIV_8    3
/** BioZ K divider: 16 */
#define MAX86178_DT_BIOZ_KDIV_16   4
/** BioZ K divider: 32 */
#define MAX86178_DT_BIOZ_KDIV_32   5
/** BioZ K divider: 64 */
#define MAX86178_DT_BIOZ_KDIV_64   6
/** BioZ K divider: 128 */
#define MAX86178_DT_BIOZ_KDIV_128  7
/** BioZ K divider: 256 */
#define MAX86178_DT_BIOZ_KDIV_256  8
/** BioZ K divider: 512 */
#define MAX86178_DT_BIOZ_KDIV_512  9
/** BioZ K divider: 1024 */
#define MAX86178_DT_BIOZ_KDIV_1024 10
/** BioZ K divider: 2048 */
#define MAX86178_DT_BIOZ_KDIV_2048 11
/** BioZ K divider: 4096 */
#define MAX86178_DT_BIOZ_KDIV_4096 12
/** BioZ K divider: 8192 */
#define MAX86178_DT_BIOZ_KDIV_8192 13
/** @} */

/**
 * @defgroup max86178_bioz_adc_osr BioZ ADC Oversampling
 * @{
 */
/** BioZ ADC OSR: 8 */
#define MAX86178_DT_BIOZ_ADC_OSR_8    0
/** BioZ ADC OSR: 16 */
#define MAX86178_DT_BIOZ_ADC_OSR_16   1
/** BioZ ADC OSR: 32 */
#define MAX86178_DT_BIOZ_ADC_OSR_32   2
/** BioZ ADC OSR: 64 */
#define MAX86178_DT_BIOZ_ADC_OSR_64   3
/** BioZ ADC OSR: 128 */
#define MAX86178_DT_BIOZ_ADC_OSR_128  4
/** BioZ ADC OSR: 256 */
#define MAX86178_DT_BIOZ_ADC_OSR_256  5
/** BioZ ADC OSR: 512 */
#define MAX86178_DT_BIOZ_ADC_OSR_512  6
/** BioZ ADC OSR: 1024 */
#define MAX86178_DT_BIOZ_ADC_OSR_1024 7
/** @} */

/**
 * @defgroup max86178_bioz_dac_osr BioZ DAC Oversampling
 * @{
 */
/** BioZ DAC OSR: 32 */
#define MAX86178_DT_BIOZ_DAC_OSR_32  0
/** BioZ DAC OSR: 64 */
#define MAX86178_DT_BIOZ_DAC_OSR_64  1
/** BioZ DAC OSR: 128 */
#define MAX86178_DT_BIOZ_DAC_OSR_128 2
/** BioZ DAC OSR: 256 */
#define MAX86178_DT_BIOZ_DAC_OSR_256 3
/** @} */

/*******************************************************************************
 * PPG CONFIGURATION
 ******************************************************************************/

/**
 * @defgroup max86178_ppg_drva PPG LED Driver A Selection
 * @{
 */
/** LED Driver A: LED1 */
#define MAX86178_DT_PPG_DRVA_LED1_DRV 0
/** LED Driver A: LED2 */
#define MAX86178_DT_PPG_DRVA_LED2_DRV 1
/** LED Driver A: LED3 */
#define MAX86178_DT_PPG_DRVA_LED3_DRV 2
/** LED Driver A: LED4 */
#define MAX86178_DT_PPG_DRVA_LED4_DRV 3
/** LED Driver A: LED5 */
#define MAX86178_DT_PPG_DRVA_LED5_DRV 4
/** LED Driver A: LED6 */
#define MAX86178_DT_PPG_DRVA_LED6_DRV 5
/** @} */

/**
 * @defgroup max86178_ppg_drvb PPG LED Driver B Selection
 * @{
 */
/** LED Driver B: LED1 */
#define MAX86178_DT_PPG_DRVB_LED1_DRV 0
/** LED Driver B: LED2 */
#define MAX86178_DT_PPG_DRVB_LED2_DRV 1
/** LED Driver B: LED3 */
#define MAX86178_DT_PPG_DRVB_LED3_DRV 2
/** LED Driver B: LED4 */
#define MAX86178_DT_PPG_DRVB_LED4_DRV 3
/** LED Driver B: LED5 */
#define MAX86178_DT_PPG_DRVB_LED5_DRV 4
/** LED Driver B: LED6 */
#define MAX86178_DT_PPG_DRVB_LED6_DRV 5
/** @} */

/**
 * @defgroup max86178_ppg_amb PPG Ambient Mode
 * @{
 */
/** PPG ambient: Normal mode */
#define MAX86178_DT_PPG_AMB_NORMAL_MODE     0
/** PPG ambient: Direct ambient conversion */
#define MAX86178_DT_PPG_AMB_DIRECT_AMB_CONV 1
/** @} */

/**
 * @defgroup max86178_ppg_sinc3_sel PPG SINC3 Filter
 * @{
 */
/** SINC3 filter: Off */
#define MAX86178_DT_PPG_SINC3_OFF 0
/** SINC3 filter: On */
#define MAX86178_DT_PPG_SINC3_ON  1
/** @} */

/**
 * @defgroup max86178_ppg_avg_num PPG Averaging
 * @{
 */
/** PPG average: 1 sample */
#define MAX86178_DT_PPG_AVG_NUM_1   0
/** PPG average: 2 samples */
#define MAX86178_DT_PPG_AVG_NUM_2   1
/** PPG average: 4 samples */
#define MAX86178_DT_PPG_AVG_NUM_4   2
/** PPG average: 8 samples */
#define MAX86178_DT_PPG_AVG_NUM_8   3
/** PPG average: 16 samples */
#define MAX86178_DT_PPG_AVG_NUM_16  4
/** PPG average: 32 samples */
#define MAX86178_DT_PPG_AVG_NUM_32  5
/** PPG average: 64 samples */
#define MAX86178_DT_PPG_AVG_NUM_64  6
/** PPG average: 128 samples */
#define MAX86178_DT_PPG_AVG_NUM_128 7
/** @} */

/**
 * @defgroup max86178_ppg_filt_sel PPG Filter Selection
 * @{
 */
/** PPG filter: Coded division multiplexing */
#define MAX86178_DT_PPG_FILT_SEL_CDM 0
/** PPG filter: Frequency division multiplexing */
#define MAX86178_DT_PPG_FILT_SEL_FDM 1
/** @} */

/**
 * @defgroup max86178_ppg_filt2_sel PPG Second Filter Order
 * @{
 */
/** Second filter: 3rd order */
#define MAX86178_DT_PPG_FILT2_3RD_ORDER 0
/** Second filter: 2nd order */
#define MAX86178_DT_PPG_FILT2_2ND_ORDER 1
/** @} */

/**
 * @defgroup max86178_ppg_adc_rge PPG ADC Range
 * @{
 */
/** PPG ADC range: 4µA full scale */
#define MAX86178_DT_PPG_ADC_RGE_4uA  0
/** PPG ADC range: 8µA full scale */
#define MAX86178_DT_PPG_ADC_RGE_8uA  1
/** PPG ADC range: 16µA full scale */
#define MAX86178_DT_PPG_ADC_RGE_16uA 2
/** PPG ADC range: 32µA full scale */
#define MAX86178_DT_PPG_ADC_RGE_32uA 3
/** @} */

/**
 * @defgroup max86178_ppg_tint PPG Integration Time
 * @{
 */
/** LED integration time: 14.6µs */
#define MAX86178_DT_PPG_TINT_14_6us  0
/** LED integration time: 29.2µs */
#define MAX86178_DT_PPG_TINT_29_2us  1
/** LED integration time: 58.6µs */
#define MAX86178_DT_PPG_TINT_58_6us  2
/** LED integration time: 117.0µs */
#define MAX86178_DT_PPG_TINT_117_0us 3
/** @} */

/**
 * @defgroup max86178_ppg_pd_setlng PPG Photodiode Settling Time
 * @{
 */
/** PD settling time: 7.8µs */
#define MAX86178_DT_PPG_PD_SETLNG_7_8us  0
/** PD settling time: 11.8µs */
#define MAX86178_DT_PPG_PD_SETLNG_11_8us 1
/** PD settling time: 15.8µs */
#define MAX86178_DT_PPG_PD_SETLNG_15_8us 2
/** PD settling time: 23.8µs */
#define MAX86178_DT_PPG_PD_SETLNG_23_8us 3
/** @} */

/**
 * @defgroup max86178_ppg_led_setlng PPG LED Settling Time
 * @{
 */
/** LED settling time: 7.7µs */
#define MAX86178_DT_PPG_LED_SETLNG_7_7us  0
/** LED settling time: 11.7µs */
#define MAX86178_DT_PPG_LED_SETLNG_11_7us 1
/** LED settling time: 15.7µs */
#define MAX86178_DT_PPG_LED_SETLNG_15_7us 2
/** LED settling time: 23.7µs */
#define MAX86178_DT_PPG_LED_SETLNG_23_7us 3
/** @} */

/**
 * @defgroup max86178_ppg_dac_off PPG DAC Offset Current
 * @{
 */
/** DAC offset: 0µA */
#define MAX86178_DT_PPG_DAC_OFF_0uA  0
/** DAC offset: 2µA */
#define MAX86178_DT_PPG_DAC_OFF_2uA  1
/** DAC offset: 4µA */
#define MAX86178_DT_PPG_DAC_OFF_4uA  2
/** DAC offset: 6µA */
#define MAX86178_DT_PPG_DAC_OFF_6uA  3
/** DAC offset: 8µA */
#define MAX86178_DT_PPG_DAC_OFF_8uA  4
/** DAC offset: 10µA */
#define MAX86178_DT_PPG_DAC_OFF_10uA 5
/** DAC offset: 12µA */
#define MAX86178_DT_PPG_DAC_OFF_12uA 6
/** DAC offset: 14µA */
#define MAX86178_DT_PPG_DAC_OFF_14uA 7
/** DAC offset: 16µA */
#define MAX86178_DT_PPG_DAC_OFF_16uA 8
/** DAC offset: 18µA */
#define MAX86178_DT_PPG_DAC_OFF_18uA 9
/** DAC offset: 20µA */
#define MAX86178_DT_PPG_DAC_OFF_20uA 10
/** DAC offset: 22µA */
#define MAX86178_DT_PPG_DAC_OFF_22uA 11
/** DAC offset: 24µA */
#define MAX86178_DT_PPG_DAC_OFF_24uA 12
/** DAC offset: 26µA */
#define MAX86178_DT_PPG_DAC_OFF_26uA 13
/** DAC offset: 28µA */
#define MAX86178_DT_PPG_DAC_OFF_28uA 14
/** DAC offset: 30µA */
#define MAX86178_DT_PPG_DAC_OFF_30uA 15
/** @} */

/**
 * @defgroup max86178_ppg_led_rge PPG LED Current Range
 * @{
 */
/** LED range: 32mA max */
#define MAX86178_DT_PPG_LED_RGE_32mA  0
/** LED range: 64mA max */
#define MAX86178_DT_PPG_LED_RGE_64mA  1
/** LED range: 96mA max */
#define MAX86178_DT_PPG_LED_RGE_96mA  2
/** LED range: 128mA max */
#define MAX86178_DT_PPG_LED_RGE_128mA 3
/** @} */

/**
 * @defgroup max86178_ppg_pd_sel PPG Photodiode Selection
 * @{
 */
/** Photodiode: Not selected */
#define MAX86178_DT_PD_NOT_SEL      1
/** Photodiode: Connected to PPG1 */
#define MAX86178_DT_PD_CONN_TO_PPG1 2
/** Photodiode: Connected to PPG2 */
#define MAX86178_DT_PD_CONN_TO_PPG2 3
/** @} */

/**
 * @defgroup max86178_ppg_pwrdn PPG Power Control
 * @{
 */
/** PPG channel: Enabled */
#define MAX86178_DT_PPG_CHAN_ENABLED 0
/** PPG channel: Powered down */
#define MAX86178_DT_PPG_CHAN_PWRDN   1
/** @} */

/**
 * @defgroup max86178_ppg_sync_mode PPG Sync Mode
 * @{
 */
/** PPG sync: Internal */
#define MAX86178_DT_PPG_INTERNAL_SYNC 0
/** PPG sync: External via TRIG pin */
#define MAX86178_DT_PPG_EXTERNAL_SYNC 1
/** @} */

/**
 * @defgroup max86178_smp_ave Sample Averaging
 * @{
 */
/** Sample average: 1 */
#define MAX86178_DT_SMP_AVE_1  0
/** Sample average: 2 */
#define MAX86178_DT_SMP_AVE_2  1
/** Sample average: 4 */
#define MAX86178_DT_SMP_AVE_4  2
/** Sample average: 8 */
#define MAX86178_DT_SMP_AVE_8  3
/** Sample average: 16 */
#define MAX86178_DT_SMP_AVE_16 4
/** @} */

/**
 * @defgroup max86178_ppg_thresh_meas_sel PPG Threshold Measurement Selection
 * @{
 */
/** Threshold: Disabled */
#define MAX86178_DT_PPG_THRESH_DISABLED 0
/** Threshold: MEAS1 */
#define MAX86178_DT_PPG_THRESH_MEAS1    1
/** Threshold: MEAS2 */
#define MAX86178_DT_PPG_THRESH_MEAS2    2
/** Threshold: MEAS3 */
#define MAX86178_DT_PPG_THRESH_MEAS3    3
/** Threshold: MEAS4 */
#define MAX86178_DT_PPG_THRESH_MEAS4    4
/** Threshold: MEAS5 */
#define MAX86178_DT_PPG_THRESH_MEAS5    5
/** Threshold: MEAS6 */
#define MAX86178_DT_PPG_THRESH_MEAS6    6
/** @} */

/**
 * @defgroup max86178_ppg_thresh_chan_sel PPG Threshold Channel Selection
 * @{
 */
/** Threshold channel: PPG1 */
#define MAX86178_DT_PPG_THRESH_PPG_CHAN1 0
/** Threshold channel: PPG2 */
#define MAX86178_DT_PPG_THRESH_PPG_CHAN2 1
/** @} */

/**
 * @defgroup max86178_ppg_time_hyst PPG Time Hysteresis
 * @{
 */
/** Time hysteresis: Disabled */
#define MAX86178_DT_PPG_TIME_HYST_DISABLED  0
/** Time hysteresis: 2 samples */
#define MAX86178_DT_PPG_TIME_HYST_2_SAMPLES 1
/** Time hysteresis: 4 samples */
#define MAX86178_DT_PPG_TIME_HYST_4_SAMPLES 2
/** Time hysteresis: 8 samples */
#define MAX86178_DT_PPG_TIME_HYST_8_SAMPLES 3
/** @} */

/**
 * @defgroup max86178_ppg_level_hyst PPG Level Hysteresis
 * @{
 */
/** Level hysteresis: Disabled */
#define MAX86178_DT_PPG_LEVEL_HYST_DISABLED    0
/** Level hysteresis: 2 samples */
#define MAX86178_DT_PPG_LEVEL_HYST_2_SAMPLES   1
/** Level hysteresis: 4 samples */
#define MAX86178_DT_PPG_LEVEL_HYST_4_SAMPLES   2
/** Level hysteresis: 8 samples */
#define MAX86178_DT_PPG_LEVEL_HYST_8_SAMPLES   3
/** Level hysteresis: 16 samples */
#define MAX86178_DT_PPG_LEVEL_HYST_16_SAMPLES  4
/** Level hysteresis: 32 samples */
#define MAX86178_DT_PPG_LEVEL_HYST_32_SAMPLES  5
/** Level hysteresis: 64 samples */
#define MAX86178_DT_PPG_LEVEL_HYST_64_SAMPLES  6
/** Level hysteresis: 128 samples */
#define MAX86178_DT_PPG_LEVEL_HYST_128_SAMPLES 7
/** @} */

/**
 * @defgroup max86178_ppg_pd_bias PPG Photodiode Bias
 * @{
 */
/** PD bias: Not recommended */
#define MAX86178_DT_PPG_PD_BIAS_NOT_RECOMMENDED 0
/** PD bias: 0 to 125pF */
#define MAX86178_DT_PPG_PD_BIAS_0_TO_125pF      1
/** PD bias: 125pF to 250pF */
#define MAX86178_DT_PPG_PD_BIAS_125pF_TO_250pF  2
/** PD bias: 250pF to 500pF */
#define MAX86178_DT_PPG_PD_BIAS_250pF_TO_500pF  3
/** @} */

/*******************************************************************************
 * ECG CONFIGURATION
 ******************************************************************************/

/**
 * @defgroup max86178_ecg_input_pol ECG Input Polarity
 * @{
 */
/** ECG input: Normal polarity */
#define MAX86178_DT_ECG_INPUT_POL_NORMAL   0
/** ECG input: Inverted polarity */
#define MAX86178_DT_ECG_INPUT_POL_INVERTED 1
/** @} */

/**
 * @defgroup max86178_ecg_pga_gain ECG PGA Gain
 * @{
 */
/** ECG PGA gain: 1 */
#define MAX86178_DT_ECG_PGA_GAIN_1 0
/** ECG PGA gain: 2 */
#define MAX86178_DT_ECG_PGA_GAIN_2 1
/** ECG PGA gain: 4 */
#define MAX86178_DT_ECG_PGA_GAIN_4 2
/** ECG PGA gain: 8 */
#define MAX86178_DT_ECG_PGA_GAIN_8 3
/** @} */

/**
 * @defgroup max86178_ecg_auto_rec ECG Auto Recovery
 * @{
 */
/** ECG auto recovery: Disabled */
#define MAX86178_DT_ECG_AUTO_REC_DISABLED 0
/** ECG auto recovery: Enabled */
#define MAX86178_DT_ECG_AUTO_REC_ENABLED  1
/** @} */

/**
 * @defgroup max86178_en_ecg_fast_rec ECG Fast Recovery
 * @{
 */
/** ECG fast recovery: Normal mode */
#define MAX86178_DT_EN_ECG_FAST_REC_NORMAL_MODE          0
/** ECG fast recovery: Manual fast recovery mode */
#define MAX86178_DT_EN_ECG_FAST_REC_MANUAL_FAST_REC_MODE 1
/** ECG fast recovery: Auto fast recovery mode */
#define MAX86178_DT_EN_ECG_FAST_REC_AUTO_FAST_REC_MODE   2
/** @} */

/**
 * @defgroup max86178_ecg_cal_freq ECG Calibration Frequency
 * @{
 */
/** ECG cal freq: Divider 128 */
#define MAX86178_DT_ECG_CAL_FREQ_DIV_128     0
/** ECG cal freq: Divider 512 */
#define MAX86178_DT_ECG_CAL_FREQ_DIV_512     1
/** ECG cal freq: Divider 2048 */
#define MAX86178_DT_ECG_CAL_FREQ_DIV_2048    2
/** ECG cal freq: Divider 8192 */
#define MAX86178_DT_ECG_CAL_FREQ_DIV_8192    3
/** ECG cal freq: Divider 32768 */
#define MAX86178_DT_ECG_CAL_FREQ_DIV_32768   4
/** ECG cal freq: Divider 131072 */
#define MAX86178_DT_ECG_CAL_FREQ_DIV_131072  5
/** ECG cal freq: Divider 524288 */
#define MAX86178_DT_ECG_CAL_FREQ_DIV_524288  6
/** ECG cal freq: Divider 2097152 */
#define MAX86178_DT_ECG_CAL_FREQ_DIV_2097152 7
/** @} */

/**
 * @defgroup max86178_ecg_cal_duty ECG Calibration Duty Cycle
 * @{
 */
/** ECG cal duty: Always high */
#define MAX86178_DT_ECG_CAL_DUTY_CAL_HIGH 0
/** ECG cal duty: 50% */
#define MAX86178_DT_ECG_CAL_DUTY_CAL_50   1
/** @} */

/**
 * @defgroup max86178_ecg_cal_en ECG Calibration Enable
 * @{
 */
/** ECG calibration: Disabled */
#define MAX86178_DT_ECG_CAL_DISABLED 0
/** ECG calibration: Enabled */
#define MAX86178_DT_ECG_CAL_ENABLED  1
/** @} */

/**
 * @defgroup max86178_ecg_open ECG Pin Isolation
 * @{
 */
/** ECG pin: Internally connected */
#define MAX86178_DT_ECG_PIN_INTERNALLY_CONNECTED 0
/** ECG pin: Internally isolated */
#define MAX86178_DT_ECG_PIN_INTERNALLY_ISOLATED  1
/** @} */

/**
 * @defgroup max86178_ecg_cal_mode ECG Calibration Mode
 * @{
 */
/** ECG cal mode: Unipolar */
#define MAX86178_DT_ECG_CAL_MODE_UNIPOLAR 0
/** ECG cal mode: Bipolar */
#define MAX86178_DT_ECG_CAL_MODE_BIPOLAR  1
/** @} */

/**
 * @defgroup max86178_ecg_cal_mag ECG Calibration Magnitude
 * @{
 */
/** ECG cal magnitude: 0.5mV */
#define MAX86178_DT_ECG_CAL_MAG_0_5mV 0
/** ECG cal magnitude: 1.0mV */
#define MAX86178_DT_ECG_CAL_MAG_1mV   1
/** @} */

/**
 * @defgroup max86178_ecg_cal_sel ECG Calibration Connection
 * @{
 */
/** ECG cal: No calibration */
#define MAX86178_DT_ECG_CAL_SEL_NO_CAL        0
/** ECG cal: Connect to VMID_ECG */
#define MAX86178_DT_ECG_CAL_SEL_CONN_VMID_ECG 1
/** ECG cal: Connect to VCALP */
#define MAX86178_DT_ECG_CAL_SEL_CONN_VCALP    2
/** ECG cal: Connect to VCALN */
#define MAX86178_DT_ECG_CAL_SEL_CONN_VCALN    3
/** @} */

/**
 * @defgroup max86178_ecg_lon ECG Lead-On Detection
 * @{
 */
/** ECG lead-on: Disabled */
#define MAX86178_DT_ECG_LON_DISABLED 0
/** ECG lead-on: Enabled */
#define MAX86178_DT_ECG_LON_ENABLED  1
/** @} */

/**
 * @defgroup max86178_ecg_loff_en ECG Lead-Off Detection
 * @{
 */
/** ECG lead-off: Disabled */
#define MAX86178_DT_ECG_LOFF_DISABLED 0
/** ECG lead-off: Enabled */
#define MAX86178_DT_ECG_LOFF_ENABLED  1
/** @} */

/**
 * @defgroup max86178_ecg_loff_mode ECG Lead-Off Mode
 * @{
 */
/** ECG lead-off mode: DC */
#define MAX86178_DT_ECG_DC_LEAD_OFF_DETECTION_MODE 0
/** ECG lead-off mode: AC */
#define MAX86178_DT_ECG_AC_LEAD_OFF_DETECTION_MODE 1
/** @} */

/**
 * @defgroup max86178_ecg_loff_freq ECG Lead-Off Frequency
 * @{
 */
/** ECG lead-off freq: Disabled */
#define MAX86178_DT_ECG_LOFF_FREQ_DISABLED 0
/** ECG lead-off freq: 4Hz */
#define MAX86178_DT_ECG_LOFF_FREQ_4Hz      1
/** ECG lead-off freq: 8Hz */
#define MAX86178_DT_ECG_LOFF_FREQ_8Hz      2
/** ECG lead-off freq: 16Hz */
#define MAX86178_DT_ECG_LOFF_FREQ_16Hz     3
/** ECG lead-off freq: 32Hz */
#define MAX86178_DT_ECG_LOFF_FREQ_32Hz     4
/** ECG lead-off freq: 64Hz */
#define MAX86178_DT_ECG_LOFF_FREQ_64Hz     5
/** ECG lead-off freq: 128Hz */
#define MAX86178_DT_ECG_LOFF_FREQ_128Hz    6
/** ECG lead-off freq: 256Hz */
#define MAX86178_DT_ECG_LOFF_FREQ_256Hz    7
/** @} */

/**
 * @defgroup max86178_ecg_loff_ipol ECG Lead-Off Current Polarity
 * @{
 */
/** ECG lead-off polarity: Non-inverted */
#define MAX86178_DT_ECG_LOFF_IPOL_NON_INVERTED 0
/** ECG lead-off polarity: Inverted */
#define MAX86178_DT_ECG_LOFF_IPOL_INVERTED     1
/** @} */

/**
 * @defgroup max86178_ecg_loff_imag ECG Lead-Off Current Magnitude
 * @{
 */
/** ECG lead-off current: 0nA */
#define MAX86178_DT_ECG_LOFF_IMAG_0nA   0
/** ECG lead-off current: 5nA */
#define MAX86178_DT_ECG_LOFF_IMAG_5nA   1
/** ECG lead-off current: 10nA */
#define MAX86178_DT_ECG_LOFF_IMAG_10nA  2
/** ECG lead-off current: 20nA */
#define MAX86178_DT_ECG_LOFF_IMAG_20nA  3
/** ECG lead-off current: 50nA */
#define MAX86178_DT_ECG_LOFF_IMAG_50nA  4
/** ECG lead-off current: 100nA */
#define MAX86178_DT_ECG_LOFF_IMAG_100nA 5
/** ECG lead-off current: 200nA */
#define MAX86178_DT_ECG_LOFF_IMAG_200nA 6
/** ECG lead-off current: 400nA */
#define MAX86178_DT_ECG_LOFF_IMAG_400nA 7
/** @} */

/**
 * @defgroup max86178_ecg_rbias_value ECG Bias Resistor Value
 * @{
 */
/** ECG bias resistor: 50MΩ */
#define MAX86178_DT_ECG_RBIAS_50M  0
/** ECG bias resistor: 100MΩ */
#define MAX86178_DT_ECG_RBIAS_100M 1
/** ECG bias resistor: 200MΩ */
#define MAX86178_DT_ECG_RBIAS_200M 2
/** @} */

/**
 * @defgroup max86178_en_ecg_rbias ECG Bias Resistor Enable
 * @{
 */
/** ECG bias resistor: Disconnected */
#define MAX86178_DT_EN_ECG_RBIAS_DISCONNECTED 0
/** ECG bias resistor: Connected */
#define MAX86178_DT_EN_ECG_RBIAS_CONNECTED    1
/** @} */

/**
 * @defgroup max86178_ecg_rld_en ECG Right Leg Drive Enable
 * @{
 */
/** RLD: Disabled */
#define MAX86178_DT_RLD_DISABLED 0
/** RLD: Enabled */
#define MAX86178_DT_RLD_ENABLED  1
/** @} */

/**
 * @defgroup max86178_ecg_rld_mode ECG RLD Mode
 * @{
 */
/** RLD mode: Open loop */
#define MAX86178_DT_RLD_MODE_OPEN_LOOP   0
/** RLD mode: Closed loop */
#define MAX86178_DT_RLD_MODE_CLOSED_LOOP 1
/** @} */

/**
 * @defgroup max86178_ecg_rld_rbias ECG RLD Bias Source
 * @{
 */
/** RLD bias: VMID */
#define MAX86178_DT_RLD_RBIAS_VMID 0
/** RLD bias: VRLD */
#define MAX86178_DT_RLD_RBIAS_VRLD 1
/** @} */

/**
 * @defgroup max86178_ecg_en_rld_oor ECG RLD Out-of-Range
 * @{
 */
/** RLD out-of-range: Disabled */
#define MAX86178_DT_EN_RLD_OOR_DISABLED 0
/** RLD out-of-range: Enabled */
#define MAX86178_DT_EN_RLD_OOR_ENABLED  1
/** @} */

/**
 * @defgroup max86178_ecg_actv_cm ECG Active Common Mode
 * @{
 */
/** ECG active CM: Disabled */
#define MAX86178_DT_ECG_ACTV_CM_DISABLED 0
/** ECG active CM: Enabled */
#define MAX86178_DT_ECG_ACTV_CM_ENABLED  1
/** @} */

/**
 * @defgroup max86178_ecg_rld_gain ECG RLD Gain
 * @{
 */
/** RLD gain: 12 */
#define MAX86178_DT_RLD_GAIN_12 0
/** RLD gain: 24 */
#define MAX86178_DT_RLD_GAIN_24 1
/** RLD gain: 48 */
#define MAX86178_DT_RLD_GAIN_48 2
/** RLD gain: 97 */
#define MAX86178_DT_RLD_GAIN_97 3
/** @} */

/**
 * @defgroup max86178_rld_ext_res RLD External Resistor
 * @{
 */
/** RLD external resistor: Internal */
#define MAX86178_DT_RLD_EXT_RES_INTERNAL 0
/** RLD external resistor: External */
#define MAX86178_DT_RLD_EXT_RES_EXTERNAL 1
/** @} */

/*******************************************************************************
 * BIOZ CONFIGURATION
 ******************************************************************************/

/**
 * @defgroup max86178_ecg_bioz_bg_en ECG/BioZ Bandgap Enable
 * @{
 */
/** ECG/BioZ bandgap: Disabled */
#define MAX86178_DT_ECG_BIOZ_BG_EN_DISABLED 0
/** ECG/BioZ bandgap: Enabled */
#define MAX86178_DT_ECG_BIOZ_BG_EN_ENABLED  1
/** @} */

/**
 * @defgroup max86178_bioz_en BioZ Channel Enable
 * @{
 */
/** BioZ: Disabled */
#define MAX86178_DT_BIOZ_EN_DISABLED  0
/** BioZ: I channel only */
#define MAX86178_DT_BIOZ_EN_I_CHANNEL 1
/** BioZ: Q channel only */
#define MAX86178_DT_BIOZ_EN_Q_CHANNEL 2
/** BioZ: Utility mode */
#define MAX86178_DT_BIOZ_EN_UTILITY   3
/** @} */

/**
 * @defgroup max86178_bioz_dhpf BioZ Digital High-Pass Filter
 * @{
 */
/** BioZ digital HPF: Bypass */
#define MAX86178_DT_BIOZ_DHPF_BYPASS  0
/** BioZ digital HPF: 0.00025 */
#define MAX86178_DT_BIOZ_DHPF_0_00025 1
/** BioZ digital HPF: 0.002 */
#define MAX86178_DT_BIOZ_DHPF_0_002   2
/** @} */

/**
 * @defgroup max86178_bioz_dlpf BioZ Digital Low-Pass Filter
 * @{
 */
/** BioZ digital LPF: Bypass */
#define MAX86178_DT_BIOZ_DLPF_BYPASS 0
/** BioZ digital LPF: 0.005 */
#define MAX86178_DT_BIOZ_DLPF_0_005  1
/** BioZ digital LPF: 0.02 */
#define MAX86178_DT_BIOZ_DLPF_0_02   2
/** BioZ digital LPF: 0.08 */
#define MAX86178_DT_BIOZ_DLPF_0_08   3
/** BioZ digital LPF: 0.25 */
#define MAX86178_DT_BIOZ_DLPF_0_25   4
/** @} */

/**
 * @defgroup max86178_en_bioz_thresh BioZ Threshold Enable
 * @{
 */
/** BioZ threshold: Disabled */
#define MAX86178_DT_EN_BIOZ_THRESH_DISABLED 0
/** BioZ threshold: Enabled */
#define MAX86178_DT_EN_BIOZ_THRESH_ENABLED  1
/** @} */

/**
 * @defgroup max86178_bioz_ext_res BioZ External Resistor
 * @{
 */
/** BioZ external resistor: Internal */
#define MAX86178_DT_BIOZ_EXT_RES_INTERNAL 0
/** BioZ external resistor: External */
#define MAX86178_DT_BIOZ_EXT_RES_EXTERNAL 1
/** @} */

/**
 * @defgroup max86178_bioz_vdrv_mag BioZ Voltage Drive Magnitude
 * @{
 */
/** BioZ voltage drive: 50mV */
#define MAX86178_DT_BIOZ_VDRV_MAG_50mV  0
/** BioZ voltage drive: 100mV */
#define MAX86178_DT_BIOZ_VDRV_MAG_100mV 1
/** BioZ voltage drive: 250mV */
#define MAX86178_DT_BIOZ_VDRV_MAG_250mV 2
/** BioZ voltage drive: 500mV */
#define MAX86178_DT_BIOZ_VDRV_MAG_500mV 3
/** @} */

/**
 * @defgroup max86178_bioz_idrv_rge BioZ Current Drive Range
 * @{
 */
/** BioZ current range: 552.5kΩ */
#define MAX86178_DT_BIOZ_IDRV_RGE_552_5K 0
/** BioZ current range: 110.5kΩ */
#define MAX86178_DT_BIOZ_IDRV_RGE_110_5K 1
/** BioZ current range: 5.525kΩ */
#define MAX86178_DT_BIOZ_IDRV_RGE_5_525K 2
/** BioZ current range: 276.25Ω */
#define MAX86178_DT_BIOZ_IDRV_RGE_276_25 3
/** @} */

/**
 * @defgroup max86178_bioz_drv_mode BioZ Drive Mode
 * @{
 */
/** BioZ drive: Current sine-wave */
#define MAX86178_DT_BIOZ_DRV_MODE_CURRENT  0
/** BioZ drive: Voltage sine-wave */
#define MAX86178_DT_BIOZ_DRV_MODE_VOLTAGE  1
/** BioZ drive: H-Bridge alternating */
#define MAX86178_DT_BIOZ_DRV_MODE_H_BRIDGE 2
/** BioZ drive: Standby low-power */
#define MAX86178_DT_BIOZ_DRV_MODE_STANDBY  3
/** @} */

/**
 * @defgroup max86178_bioz_dc_code_sel BioZ DC Code Selection
 * @{
 */
/** BioZ DC code: DDS DAC code */
#define MAX86178_DT_BIOZ_DC_CODE_SEL_DDS     0
/** BioZ DC code: DC DAC code */
#define MAX86178_DT_BIOZ_DC_CODE_SEL_DC_CODE 1
/** @} */

/**
 * @defgroup max86178_bioz_ahpf BioZ Analog High-Pass Filter
 * @{
 */
/** BioZ analog HPF: 100Hz */
#define MAX86178_DT_BIOZ_AHPF_100Hz    0
/** BioZ analog HPF: 200Hz */
#define MAX86178_DT_BIOZ_AHPF_200Hz    1
/** BioZ analog HPF: 500Hz */
#define MAX86178_DT_BIOZ_AHPF_500Hz    2
/** BioZ analog HPF: 1000Hz */
#define MAX86178_DT_BIOZ_AHPF_1000Hz   3
/** BioZ analog HPF: 2000Hz */
#define MAX86178_DT_BIOZ_AHPF_2000Hz   4
/** BioZ analog HPF: 5000Hz */
#define MAX86178_DT_BIOZ_AHPF_5000Hz   5
/** BioZ analog HPF: 10000Hz */
#define MAX86178_DT_BIOZ_AHPF_10000Hz  6
/** BioZ analog HPF: Bypass */
#define MAX86178_DT_BIOZ_AHPF_BYPASS   7
/** BioZ analog HPF: 42.4MΩ */
#define MAX86178_DT_BIOZ_AHPF_42_4M    8
/** BioZ analog HPF: 21.2MΩ */
#define MAX86178_DT_BIOZ_AHPF_21_2M    9
/** BioZ analog HPF: 8.4MΩ */
#define MAX86178_DT_BIOZ_AHPF_8_4M     10
/** BioZ analog HPF: 4.2MΩ */
#define MAX86178_DT_BIOZ_AHPF_4_2M     11
/** BioZ analog HPF: 2.2MΩ */
#define MAX86178_DT_BIOZ_AHPF_2_2M     12
/** BioZ analog HPF: 848kΩ */
#define MAX86178_DT_BIOZ_AHPF_848K     13
/** BioZ analog HPF: 848kΩ (alt) */
#define MAX86178_DT_BIOZ_AHPF_848K_2   14
/** BioZ analog HPF: Bypass (alt) */
#define MAX86178_DT_BIOZ_AHPF_BYPASS_2 15
/** @} */

/**
 * @defgroup max86178_bioz_ina_mode BioZ INA Power Mode
 * @{
 */
/** BioZ INA: High-power low-noise */
#define MAX86178_DT_BIOZ_INA_MODE_HIGH_POWER 0
/** BioZ INA: Low-power */
#define MAX86178_DT_BIOZ_INA_MODE_LOW_POWER  1
/** @} */

/**
 * @defgroup max86178_bioz_dm_dis BioZ Demodulation
 * @{
 */
/** BioZ demodulation: Enabled */
#define MAX86178_DT_BIOZ_DM_ENABLED  0
/** BioZ demodulation: Disabled */
#define MAX86178_DT_BIOZ_DM_DISABLED 1
/** @} */

/**
 * @defgroup max86178_bioz_gain BioZ Gain
 * @{
 */
/** BioZ gain: 1 */
#define MAX86178_DT_BIOZ_GAIN_1  0
/** BioZ gain: 2 */
#define MAX86178_DT_BIOZ_GAIN_2  1
/** BioZ gain: 5 */
#define MAX86178_DT_BIOZ_GAIN_5  2
/** BioZ gain: 10 */
#define MAX86178_DT_BIOZ_GAIN_10 3
/** @} */

/**
 * @defgroup max86178_bioz_ext_cap BioZ External Capacitor
 * @{
 */
/** BioZ external cap: Internal switch shorts pins */
#define MAX86178_DT_BIOZ_EXT_CAP_INTERNAL 0
/** BioZ external cap: External cap, switch open */
#define MAX86178_DT_BIOZ_EXT_CAP_EXTERNAL 1
/** @} */

/**
 * @defgroup max86178_bioz_dc_restore BioZ DC Restore
 * @{
 */
/** BioZ DC restore: Switch open */
#define MAX86178_DT_BIOZ_DC_RESTORE_SWITCH_OPEN   0
/** BioZ DC restore: Switch closed */
#define MAX86178_DT_BIOZ_DC_RESTORE_SWITCH_CLOSED 1
/** @} */

/**
 * @defgroup max86178_bioz_drv_reset BioZ Drive Reset
 * @{
 */
/** BioZ drive reset: Switch open */
#define MAX86178_DT_BIOZ_DRV_RESET_SWITCH_OPEN   0
/** BioZ drive reset: Switch closed */
#define MAX86178_DT_BIOZ_DRV_RESET_SWITCH_CLOSED 1
/** @} */

/**
 * @defgroup max86178_bioz_amp_rge BioZ Amplifier Range
 * @{
 */
/** BioZ amp range: Low */
#define MAX86178_DT_BIOZ_AMP_RGE_LOW         0
/** BioZ amp range: Medium-low */
#define MAX86178_DT_BIOZ_AMP_RGE_MEDIUM_LOW  1
/** BioZ amp range: Medium-high */
#define MAX86178_DT_BIOZ_AMP_RGE_MEDIUM_HIGH 2
/** BioZ amp range: High */
#define MAX86178_DT_BIOZ_AMP_RGE_HIGH        3
/** @} */

/**
 * @defgroup max86178_bioz_amp_bw BioZ Amplifier Bandwidth
 * @{
 */
/** BioZ amp BW: Low */
#define MAX86178_DT_BIOZ_AMP_BW_LOW         0
/** BioZ amp BW: Medium-low */
#define MAX86178_DT_BIOZ_AMP_BW_MEDIUM_LOW  1
/** BioZ amp BW: Medium-high */
#define MAX86178_DT_BIOZ_AMP_BW_MEDIUM_HIGH 2
/** BioZ amp BW: High */
#define MAX86178_DT_BIOZ_AMP_BW_HIGH        3
/** @} */

/**
 * @defgroup max86178_bioz_rld_drv BioZ RLD Drive Source
 * @{
 */
/** BioZ RLD drive: VMID_TX */
#define MAX86178_DT_BIOZ_RLD_DRV_VMID_TX 0
/** BioZ RLD drive: VRLD */
#define MAX86178_DT_BIOZ_RLD_DRV_VRLD    1
/** @} */

/**
 * @defgroup max86178_bioz_cmres_dis BioZ Common-Mode Resistor
 * @{
 */
/** BioZ common-mode resistor: 100MΩ */
#define MAX86178_DT_BIOZ_CMRES_DIS_100M 0
/** BioZ common-mode resistor: Disabled */
#define MAX86178_DT_BIOZ_CMRES_DIS_0    1
/** @} */

/**
 * @defgroup max86178_bioz_stbyon BioZ Standby Enable
 * @{
 */
/** BioZ standby: Disabled */
#define MAX86178_DT_BIOZ_STBYON_DISABLED 0
/** BioZ standby: Enabled */
#define MAX86178_DT_BIOZ_STBYON_ENABLED  1
/** @} */

/**
 * @defgroup max86178_bioz_ipol BioZ Current Polarity
 * @{
 */
/** BioZ polarity: Non-inverted */
#define MAX86178_DT_BIOZ_IPOL_NON_INVERTED 0
/** BioZ polarity: Inverted */
#define MAX86178_DT_BIOZ_IPOL_INVERTED     1
/** @} */

/**
 * @defgroup max86178_bioz_fast BioZ Fast Start Mode
 * @{
 */
/** BioZ fast start: Normal */
#define MAX86178_DT_BIOZ_FAST_NORMAL        0
/** BioZ fast start: Enabled */
#define MAX86178_DT_BIOZ_FAST_START_ENABLED 1
/** @} */

/**
 * @defgroup max86178_bioz_ina_chop_en BioZ INA Chopper
 * @{
 */
/** BioZ INA chopper: Disabled */
#define MAX86178_DT_BIOZ_INA_CHOP_DISABLED 0
/** BioZ INA chopper: Enabled */
#define MAX86178_DT_BIOZ_INA_CHOP_ENABLED  1
/** @} */

/**
 * @defgroup max86178_bioz_ch_fsel BioZ Chopper Frequency
 * @{
 */
/** BioZ chopper freq: 50kHz */
#define MAX86178_DT_BIOZ_CH_FSEL_50KHZ 0
/** BioZ chopper freq: 25kHz */
#define MAX86178_DT_BIOZ_CH_FSEL_25KHZ 1
/** @} */

/**
 * @defgroup max86178_bmux_rsel BioZ Mux Resistor
 * @{
 */
/** BioZ mux resistor: 5000Ω */
#define MAX86178_DT_BMUX_RSEL_5000_OHM 0
/** BioZ mux resistor: 800Ω */
#define MAX86178_DT_BMUX_RSEL_800_OHM  1
/** BioZ mux resistor: 500Ω */
#define MAX86178_DT_BMUX_RSEL_500_OHM  2
/** BioZ mux resistor: 200Ω */
#define MAX86178_DT_BMUX_RSEL_200_OHM  3
/** @} */

/**
 * @defgroup max86178_bmux_bist_en BioZ Built-In Self-Test
 * @{
 */
/** BioZ BIST: Disabled */
#define MAX86178_DT_BMUX_BIST_DISABLED 0
/** BioZ BIST: Enabled */
#define MAX86178_DT_BMUX_BIST_ENABLED  1
/** @} */

/**
 * @defgroup max86178_bmux_gsr_rsel BioZ GSR Resistor
 * @{
 */
/** BioZ GSR resistor: 25kΩ */
#define MAX86178_DT_BMUX_GSR_RSEL_25K   0
/** BioZ GSR resistor: 100kΩ */
#define MAX86178_DT_BMUX_GSR_RSEL_100K  1
/** BioZ GSR resistor: 500kΩ */
#define MAX86178_DT_BMUX_GSR_RSEL_500K  2
/** BioZ GSR resistor: 1000kΩ */
#define MAX86178_DT_BMUX_GSR_RSEL_1000K 3
/** @} */

/**
 * @defgroup max86178_gsr_load_en BioZ GSR Load Enable
 * @{
 */
/** GSR load: Disabled */
#define MAX86178_DT_GSR_LOAD_DISABLED 0
/** GSR load: Enabled */
#define MAX86178_DT_GSR_LOAD_ENABLED  1
/** @} */

/**
 * @defgroup max86178_en_ext_inload BioZ External Input Load
 * @{
 */
/** External input load: Disabled */
#define MAX86178_DT_EN_EXT_INLOAD_DISABLED 0
/** External input load: Enabled */
#define MAX86178_DT_EN_EXT_INLOAD_ENABLED  1
/** @} */

/**
 * @defgroup max86178_en_int_inload BioZ Internal Input Load
 * @{
 */
/** Internal input load: Disabled */
#define MAX86178_DT_EN_INT_INLOAD_DISABLED 0
/** Internal input load: Enabled */
#define MAX86178_DT_EN_INT_INLOAD_ENABLED  1
/** @} */

/**
 * @defgroup max86178_bioz_bip_assign BioZ Positive Input Assignment
 * @{
 */
/** BioZ BIP: EL1 */
#define MAX86178_DT_BIOZ_BIP_ASSIGN_EL1         0
/** BioZ BIP: EL2A */
#define MAX86178_DT_BIOZ_BIP_ASSIGN_EL2A        1
/** BioZ BIP: EL2B */
#define MAX86178_DT_BIOZ_BIP_ASSIGN_EL2B        2
/** BioZ BIP: Utility ADC */
#define MAX86178_DT_BIOZ_BIP_ASSIGN_UTILITY_ADC 3
/** @} */

/**
 * @defgroup max86178_bioz_bin_assign BioZ Negative Input Assignment
 * @{
 */
/** BioZ BIN: EL4 */
#define MAX86178_DT_BIOZ_BIN_ASSIGN_EL4         0
/** BioZ BIN: EL3A */
#define MAX86178_DT_BIOZ_BIN_ASSIGN_EL3A        1
/** BioZ BIN: EL3B */
#define MAX86178_DT_BIOZ_BIN_ASSIGN_EL3B        2
/** BioZ BIN: Utility ADC */
#define MAX86178_DT_BIOZ_BIN_ASSIGN_UTILITY_ADC 3
/** @} */

/**
 * @defgroup max86178_en_bioz_lon BioZ Lead-On Detection
 * @{
 */
/** BioZ lead-on: Disabled */
#define MAX86178_DT_EN_BIOZ_LON_DISABLED 0
/** BioZ lead-on: Enabled */
#define MAX86178_DT_EN_BIOZ_LON_ENABLED  1
/** @} */

/**
 * @defgroup max86178_en_bioz_loff BioZ Lead-Off Detection
 * @{
 */
/** BioZ lead-off: Disabled */
#define MAX86178_DT_EN_BIOZ_LOFF_DISABLED 0
/** BioZ lead-off: Enabled */
#define MAX86178_DT_EN_BIOZ_LOFF_ENABLED  1
/** @} */

/**
 * @defgroup max86178_en_ext_bioz_loff BioZ External Lead-Off
 * @{
 */
/** BioZ external lead-off: Internal */
#define MAX86178_DT_EN_EXT_BIOZ_LOFF_INTERNAL 0
/** BioZ external lead-off: External */
#define MAX86178_DT_EN_EXT_BIOZ_LOFF_EXTERNAL 1
/** @} */

/**
 * @defgroup max86178_en_bioz_drv_oor BioZ Drive Out-of-Range
 * @{
 */
/** BioZ drive OOR: Disabled */
#define MAX86178_DT_EN_BIOZ_DRV_OOR_DISABLED 0
/** BioZ drive OOR: Enabled */
#define MAX86178_DT_EN_BIOZ_DRV_OOR_ENABLED  1
/** @} */

/**
 * @defgroup max86178_bioz_rbias_value BioZ Bias Resistor Value
 * @{
 */
/** BioZ bias resistor: 50MΩ */
#define MAX86178_DT_BIOZ_RBIAS_50M  0
/** BioZ bias resistor: 100MΩ */
#define MAX86178_DT_BIOZ_RBIAS_100M 1
/** BioZ bias resistor: 200MΩ */
#define MAX86178_DT_BIOZ_RBIAS_200M 2
/** @} */

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_SENSOR_MAX86178_H_ */
