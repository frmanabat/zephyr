/*
 * Copyright (c) 2026 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_MAX86178_MAX86178_H_
#define ZEPHYR_DRIVERS_SENSOR_MAX86178_MAX86178_H_

#include <zephyr/drivers/sensor.h>
#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#define DT_DRV_COMPAT adi_max86178
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
#define MAX86178_BUS_SPI
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(spi) */
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
#define MAX86178_BUS_I2C
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c) */

#ifdef MAX86178_BUS_SPI
#include <zephyr/drivers/spi.h>
#endif /* MAX86178_BUS_SPI */

#ifdef MAX86178_BUS_I2C
#include <zephyr/drivers/i2c.h>
#endif /* MAX86178_BUS_I2C */

/*
 * MAX86178 registers definition
 */

/* Status Registers */
#define MAX86178_STATUS1          0x00u /* Status Register 1 */
#define MAX86178_STATUS2          0x01u /* Status Register 2 */
#define MAX86178_STATUS3          0x02u /* Status Register 3 */
#define MAX86178_STATUS4          0x03u /* Status Register 4 (ECG Status) */
#define MAX86178_STATUS5          0x04u /* Status Register 5 (BioZ Status) */

/* FIFO Registers */
#define MAX86178_FIFO_WR_PTR      0x08u /* FIFO Write Pointer */
#define MAX86178_FIFO_RD_PTR      0x09u /* FIFO Read Pointer */
#define MAX86178_FIFO_COUNTER1    0x0Au /* FIFO Counter 1 */
#define MAX86178_FIFO_COUNTER2    0x0Bu /* FIFO Counter 2 */
#define MAX86178_FIFO_DATA        0x0Cu /* FIFO Data Register */
#define MAX86178_FIFO_CFG1        0x0Du /* FIFO Configuration 1 */
#define MAX86178_FIFO_CFG2        0x0Eu /* FIFO Configuration 2 */

/* System Control Registers */
#define MAX86178_SYS_SYNC         0x10u /* System Sync */
#define MAX86178_SYS_CFG1         0x11u /* System Configuration 1 */
#define MAX86178_SYS_CFG2         0x12u /* System Configuration 2 */
#define MAX86178_PIN_FUNC_CFG     0x13u /* Pin Functional Configuration */
#define MAX86178_OUT_PIN_CFG      0x14u /* Output Pin Configuration */
#define MAX86178_I2C_BCAST_ADDR   0x15u /* I2C Broadcast Address */

/* PLL Configuration Registers */
#define MAX86178_PLL_CFG1         0x18u /* PLL Configuration 1 */
#define MAX86178_PLL_CFG2         0x19u /* PLL Configuration 2 */
#define MAX86178_PLL_CFG3         0x1Au /* PLL Configuration 3 */
#define MAX86178_PLL_CFG4         0x1Bu /* PLL Configuration 4 */
#define MAX86178_PLL_CFG5         0x1Cu /* PLL Configuration 5 */
#define MAX86178_PLL_CFG6         0x1Du /* PLL Configuration 6 */

/* PPG Setup Registers */
#define MAX86178_PPG_CFG1         0x20u /* PPG Configuration 1 */
#define MAX86178_PPG_CFG2         0x21u /* PPG Configuration 2 */
#define MAX86178_PPG_CFG3         0x22u /* PPG Configuration 3 */
#define MAX86178_PPG_CFG4         0x23u /* PPG Configuration 4 */
#define MAX86178_PD_BIAS          0x24u /* Photodiode Bias */

/* PPG Frame Rate Clock */
#define MAX86178_FR_CLK_DIV_MSB   0x28u /* Frame Clock Divider MSB */
#define MAX86178_FR_CLK_DIV_LSB   0x29u /* Frame Clock Divider LSB */

/* PPG MEAS1 Setup (0x30-0x37) */
#define MAX86178_MEAS1_SEL        0x30u /* MEAS1 Selects */
#define MAX86178_MEAS1_CFG1       0x31u /* MEAS1 Configuration 1 */
#define MAX86178_MEAS1_CFG2       0x32u /* MEAS1 Configuration 2 */
#define MAX86178_MEAS1_CFG3       0x33u /* MEAS1 Configuration 3 */
#define MAX86178_MEAS1_CFG4       0x34u /* MEAS1 Configuration 4 */
#define MAX86178_MEAS1_CFG5       0x35u /* MEAS1 Configuration 5 */
#define MAX86178_MEAS1_LEDA_PA    0x36u /* MEAS1 LEDA Current */
#define MAX86178_MEAS1_LEDB_PA    0x37u /* MEAS1 LEDB Current */

/* PPG MEAS2 Setup (0x38-0x3F) */
#define MAX86178_MEAS2_SEL        0x38u /* MEAS2 Selects */
#define MAX86178_MEAS2_CFG1       0x39u /* MEAS2 Configuration 1 */
#define MAX86178_MEAS2_CFG2       0x3Au /* MEAS2 Configuration 2 */
#define MAX86178_MEAS2_CFG3       0x3Bu /* MEAS2 Configuration 3 */
#define MAX86178_MEAS2_CFG4       0x3Cu /* MEAS2 Configuration 4 */
#define MAX86178_MEAS2_CFG5       0x3Du /* MEAS2 Configuration 5 */
#define MAX86178_MEAS2_LEDA_PA    0x3Eu /* MEAS2 LEDA Current */
#define MAX86178_MEAS2_LEDB_PA    0x3Fu /* MEAS2 LEDB Current */

/* PPG MEAS3 Setup (0x40-0x47) */
#define MAX86178_MEAS3_SEL        0x40u /* MEAS3 Selects */
#define MAX86178_MEAS3_CFG1       0x41u /* MEAS3 Configuration 1 */
#define MAX86178_MEAS3_CFG2       0x42u /* MEAS3 Configuration 2 */
#define MAX86178_MEAS3_CFG3       0x43u /* MEAS3 Configuration 3 */
#define MAX86178_MEAS3_CFG4       0x44u /* MEAS3 Configuration 4 */
#define MAX86178_MEAS3_CFG5       0x45u /* MEAS3 Configuration 5 */
#define MAX86178_MEAS3_LEDA_PA    0x46u /* MEAS3 LEDA Current */
#define MAX86178_MEAS3_LEDB_PA    0x47u /* MEAS3 LEDB Current */

/* PPG MEAS4 Setup (0x48-0x4F) */
#define MAX86178_MEAS4_SEL        0x48u /* MEAS4 Selects */
#define MAX86178_MEAS4_CFG1       0x49u /* MEAS4 Configuration 1 */
#define MAX86178_MEAS4_CFG2       0x4Au /* MEAS4 Configuration 2 */
#define MAX86178_MEAS4_CFG3       0x4Bu /* MEAS4 Configuration 3 */
#define MAX86178_MEAS4_CFG4       0x4Cu /* MEAS4 Configuration 4 */
#define MAX86178_MEAS4_CFG5       0x4Du /* MEAS4 Configuration 5 */
#define MAX86178_MEAS4_LEDA_PA    0x4Eu /* MEAS4 LEDA Current */
#define MAX86178_MEAS4_LEDB_PA    0x4Fu /* MEAS4 LEDB Current */

/* PPG MEAS5 Setup (0x50-0x57) */
#define MAX86178_MEAS5_SEL        0x50u /* MEAS5 Selects */
#define MAX86178_MEAS5_CFG1       0x51u /* MEAS5 Configuration 1 */
#define MAX86178_MEAS5_CFG2       0x52u /* MEAS5 Configuration 2 */
#define MAX86178_MEAS5_CFG3       0x53u /* MEAS5 Configuration 3 */
#define MAX86178_MEAS5_CFG4       0x54u /* MEAS5 Configuration 4 */
#define MAX86178_MEAS5_CFG5       0x55u /* MEAS5 Configuration 5 */
#define MAX86178_MEAS5_LEDA_PA    0x56u /* MEAS5 LEDA Current */
#define MAX86178_MEAS5_LEDB_PA    0x57u /* MEAS5 LEDB Current */

/* PPG MEAS6 Setup (0x58-0x5F) */
#define MAX86178_MEAS6_SEL        0x58u /* MEAS6 Selects */
#define MAX86178_MEAS6_CFG1       0x59u /* MEAS6 Configuration 1 */
#define MAX86178_MEAS6_CFG2       0x5Au /* MEAS6 Configuration 2 */
#define MAX86178_MEAS6_CFG3       0x5Bu /* MEAS6 Configuration 3 */
#define MAX86178_MEAS6_CFG4       0x5Cu /* MEAS6 Configuration 4 */
#define MAX86178_MEAS6_CFG5       0x5Du /* MEAS6 Configuration 5 */
#define MAX86178_MEAS6_LEDA_PA    0x5Eu /* MEAS6 LEDA Current */
#define MAX86178_MEAS6_LEDB_PA    0x5Fu /* MEAS6 LEDB Current */

/* PPG Threshold Interrupt Registers (0x70-0x75) */
#define MAX86178_THRESH_MEAS_SEL  0x70u /* Threshold Measurement Select */
#define MAX86178_THRESH_HYST      0x71u /* Threshold Hysteresis */
#define MAX86178_PPG_HI_THRESH1   0x72u /* PPG High Threshold 1 */
#define MAX86178_PPG_LO_THRESH1   0x73u /* PPG Low Threshold 1 */
#define MAX86178_PPG_HI_THRESH2   0x74u /* PPG High Threshold 2 */
#define MAX86178_PPG_LO_THRESH2   0x75u /* PPG Low Threshold 2 */

/* ECG Configuration and Control Registers (0x80-0x8F) */
#define MAX86178_ECG_CFG1         0x80u /* ECG Configuration 1 */
#define MAX86178_ECG_CFG2         0x81u /* ECG Configuration 2 */
#define MAX86178_ECG_CFG3         0x82u /* ECG Configuration 3 */
#define MAX86178_ECG_CFG4         0x83u /* ECG Configuration 4 */
#define MAX86178_ECG_CAL_CFG1     0x84u /* ECG Calibration Configuration 1 */
#define MAX86178_ECG_CAL_CFG2     0x85u /* ECG Calibration Configuration 2 */
#define MAX86178_ECG_CAL_CFG3     0x86u /* ECG Calibration Configuration 3 */
#define MAX86178_ECG_LD_CFG1      0x88u /* ECG Lead Detect Configuration 1 */
#define MAX86178_ECG_LD_CFG2      0x89u /* ECG Lead Detect Configuration 2 */
#define MAX86178_ECG_LB_CFG1      0x90u /* ECG Lead Bias Configuration 1 */
#define MAX86178_RLD_CFG1         0x92u /* RLD Configuration 1 */
#define MAX86178_RLD_CFG2         0x93u /* RLD Configuration 2 */

/* BioZ Configuration and Control Registers */
#define MAX86178_BIOZ_CFG1        0xA0u /* BioZ Configuration 1 */
#define MAX86178_BIOZ_CFG2        0xA1u /* BioZ Configuration 2 */
#define MAX86178_BIOZ_CFG3        0xA2u /* BioZ Configuration 3 */
#define MAX86178_BIOZ_CFG4        0xA3u /* BioZ Configuration 4 */
#define MAX86178_BIOZ_CFG5        0xA4u /* BioZ Configuration 5 */
#define MAX86178_BIOZ_CFG6        0xA5u /* BioZ Configuration 6 */
#define MAX86178_BIOZ_CFG7        0xA6u /* BioZ Configuration 7 */
#define MAX86178_BIOZ_CFG8        0xA7u /* BioZ Configuration 8 */
#define MAX86178_BIOZ_LO_THRESH   0xA8u /* BioZ Low Threshold */
#define MAX86178_BIOZ_HI_THRESH   0xA9u /* BioZ High Threshold */

/* BioZ Calibration Registers (0xAA-0xAD) */
#define MAX86178_BIOZ_MUX_CFG1    0xAAu /* BioZ Mux Configuration 1 */
#define MAX86178_BIOZ_MUX_CFG2    0xABu /* BioZ Mux Configuration 2 */
#define MAX86178_BIOZ_MUX_CFG3    0xACu /* BioZ Mux Configuration 3 */
#define MAX86178_BIOZ_MUX_CFG4    0xADu /* BioZ Mux Configuration 4 */

/* BioZ Lead Detect/Bias and Respiration Registers */
#define MAX86178_BIOZ_LD_CFG1     0xB0u /* BioZ Lead Detect Configuration 1 */
#define MAX86178_BIOZ_LOFF_THRESH 0xB1u /* BioZ Lead-Off Threshold */
#define MAX86178_BIOZ_LB_CFG1     0xB4u /* BioZ Lead Bias Configuration 1 */
#define MAX86178_RESP_CFG1        0xB6u /* Respiration Configuration 1 */

/* Interrupt Enable Registers */
#define MAX86178_INT1_EN1         0xC0u /* Interrupt1 Enable 1 */
#define MAX86178_INT1_EN2         0xC1u /* Interrupt1 Enable 2 */
#define MAX86178_INT1_EN3         0xC2u /* Interrupt1 Enable 3 */
#define MAX86178_INT1_EN4         0xC3u /* Interrupt1 Enable 4 */
#define MAX86178_INT1_EN5         0xC4u /* Interrupt1 Enable 5 */
#define MAX86178_INT2_EN1         0xC5u /* Interrupt2 Enable 1 */
#define MAX86178_INT2_EN2         0xC6u /* Interrupt2 Enable 2 */
#define MAX86178_INT2_EN3         0xC7u /* Interrupt2 Enable 3 */
#define MAX86178_INT2_EN4         0xC8u /* Interrupt2 Enable 4 */
#define MAX86178_INT2_EN5         0xC9u /* Interrupt2 Enable 5 */

/* Device Identification Registers */
#define MAX86178_PART_ID          0xFFu /* Part ID Register */
#define MAX86178_PART_ID_MSK      GENMASK(7, 0)
#define MAX86178_PART_ID_VAL      0x43u /* Reset value of PART_ID[7:0] */

/* REGISTER BIT MASKS */

/* STATUS1 (0x00) */
#define MAX86178_STATUS1_PWR_RDY_MSK         BIT(0)
#define MAX86178_STATUS1_PPG_THRESH1_HILO_MSK BIT(1)
#define MAX86178_STATUS1_PPG_THRESH2_HILO_MSK BIT(2)
#define MAX86178_STATUS1_EXP_OVF_MSK         BIT(3)
#define MAX86178_STATUS1_ALC_OVF_MSK         BIT(4)
#define MAX86178_STATUS1_FIFO_DATA_RDY_MSK   BIT(5)
#define MAX86178_STATUS1_PPG_FRAME_RDY_MSK   BIT(6)
#define MAX86178_STATUS1_A_FULL_MSK          BIT(7)

/* STATUS2 (0x01) */
#define MAX86178_STATUS2_LED1_COMPB_MSK      BIT(0)
#define MAX86178_STATUS2_LED2_COMPB_MSK      BIT(1)
#define MAX86178_STATUS2_LED3_COMPB_MSK      BIT(2)
#define MAX86178_STATUS2_LED4_COMPB_MSK      BIT(3)
#define MAX86178_STATUS2_LED5_COMPB_MSK      BIT(4)
#define MAX86178_STATUS2_LED6_COMPB_MSK      BIT(5)
#define MAX86178_STATUS2_INVALID_PPG_CFG_MSK BIT(7)

/* STATUS3 (0x02) */
#define MAX86178_STATUS3_PHASE_LOCK_MSK      BIT(0)
#define MAX86178_STATUS3_PHASE_UNLOCK_MSK    BIT(1)
#define MAX86178_STATUS3_FREQ_LOCK_MSK       BIT(2)
#define MAX86178_STATUS3_FREQ_UNLOCK_MSK     BIT(3)

/* STATUS4 (0x03) - ECG Status */
#define MAX86178_STATUS4_ECG_LOFF_NL_MSK     BIT(0)
#define MAX86178_STATUS4_ECG_LOFF_NH_MSK     BIT(1)
#define MAX86178_STATUS4_ECG_LOFF_PL_MSK     BIT(2)
#define MAX86178_STATUS4_ECG_LOFF_PH_MSK     BIT(3)
#define MAX86178_STATUS4_RLD_OOR_MSK         BIT(4)
#define MAX86178_STATUS4_ECG_FAST_REC_MSK    BIT(5)
#define MAX86178_STATUS4_ECG_LON_MSK         BIT(7)

/* STATUS5 (0x04) - BioZ Status */
#define MAX86178_STATUS5_BIOZ_LOFF_NL_MSK    BIT(0)
#define MAX86178_STATUS5_BIOZ_LOFF_NH_MSK    BIT(1)
#define MAX86178_STATUS5_BIOZ_LOFF_PL_MSK    BIT(2)
#define MAX86178_STATUS5_BIOZ_LOFF_PH_MSK    BIT(3)
#define MAX86178_STATUS5_BIOZ_DRV_OOR_MSK    BIT(4)
#define MAX86178_STATUS5_BIOZ_UNDR_MSK       BIT(5)
#define MAX86178_STATUS5_BIOZ_OVER_MSK       BIT(6)
#define MAX86178_STATUS5_BIOZ_LON_MSK        BIT(7)

/* Common Interrupt Enable Masks (shared by INT1_ENx and INT2_ENx) */
/* INTx_EN1 layout (x = 1,2) */
#define MAX86178_INT_EN1_PPG_THRESH1_HILO_MSK BIT(1)
#define MAX86178_INT_EN1_PPG_THRESH2_HILO_MSK BIT(2)
#define MAX86178_INT_EN1_EXP_OVF_MSK          BIT(3)
#define MAX86178_INT_EN1_ALC_OVF_MSK          BIT(4)
#define MAX86178_INT_EN1_FIFO_DATA_RDY_MSK    BIT(5)
#define MAX86178_INT_EN1_PPG_FRAME_RDY_MSK    BIT(6)
#define MAX86178_INT_EN1_A_FULL_MSK           BIT(7)

/* INTx_EN2 layout (x = 1,2) */
#define MAX86178_INT_EN2_LED1_COMPB_MSK       BIT(0)
#define MAX86178_INT_EN2_LED2_COMPB_MSK       BIT(1)
#define MAX86178_INT_EN2_LED3_COMPB_MSK       BIT(2)
#define MAX86178_INT_EN2_LED4_COMPB_MSK       BIT(3)
#define MAX86178_INT_EN2_LED5_COMPB_MSK       BIT(4)
#define MAX86178_INT_EN2_LED6_COMPB_MSK       BIT(5)
#define MAX86178_INT_EN2_INVALID_PPG_CFG_MSK  BIT(7)

/* INTx_EN3 layout (x = 1,2) */
#define MAX86178_INT_EN3_PHASE_LOCK_MSK       BIT(0)
#define MAX86178_INT_EN3_PHASE_UNLOCK_MSK     BIT(1)
#define MAX86178_INT_EN3_FREQ_LOCK_MSK        BIT(2)
#define MAX86178_INT_EN3_FREQ_UNLOCK_MSK      BIT(3)

/* INTx_EN4 layout (x = 1,2) */
#define MAX86178_INT_EN4_ECG_LOFF_NL_MSK      BIT(0)
#define MAX86178_INT_EN4_ECG_LOFF_NH_MSK      BIT(1)
#define MAX86178_INT_EN4_ECG_LOFF_PL_MSK      BIT(2)
#define MAX86178_INT_EN4_ECG_LOFF_PH_MSK      BIT(3)
#define MAX86178_INT_EN4_RLD_OOR_MSK          BIT(4)
#define MAX86178_INT_EN4_ECG_FAST_REC_MSK     BIT(5)
#define MAX86178_INT_EN4_ECG_LON_MSK          BIT(7)

/* INTx_EN5 layout (x = 1,2) */
#define MAX86178_INT_EN5_BIOZ_LOFF_NL_MSK     BIT(0)
#define MAX86178_INT_EN5_BIOZ_LOFF_NH_MSK     BIT(1)
#define MAX86178_INT_EN5_BIOZ_LOFF_PL_MSK     BIT(2)
#define MAX86178_INT_EN5_BIOZ_LOFF_PH_MSK     BIT(3)
#define MAX86178_INT_EN5_BIOZ_DRVP_OFF_MSK    BIT(4)
#define MAX86178_INT_EN5_BIOZ_UNDR_MSK        BIT(5)
#define MAX86178_INT_EN5_BIOZ_OVER_MSK        BIT(6)
#define MAX86178_INT_EN5_BIOZ_LON_MSK         BIT(7)

/* FIFO_COUNTER1 (0x0A) */
#define MAX86178_FIFO_COUNTER1_OVF_COUNTER_MSK GENMASK(6, 0)

/* FIFO_COUNTER2 (0x0B) */
#define MAX86178_FIFO_COUNTER2_DATA_COUNT_MSK GENMASK(7, 0)

/* FIFO_CFG2 (0x0E) */
#define MAX86178_FIFO_CFG2_FIFO_RO_MSK       BIT(1)
#define MAX86178_FIFO_CFG2_A_FULL_TYPE_MSK   BIT(2)
#define MAX86178_FIFO_CFG2_FIFO_STAT_CLR_MSK BIT(3)
#define MAX86178_FIFO_CFG2_FLUSH_FIFO_MSK    BIT(4)
#define MAX86178_FIFO_CFG2_FIFO_MARK_MSK     BIT(5)

/* SYS_SYNC (0x10) */
#define MAX86178_SYS_SYNC_TIMING_SYS_RESET_MSK BIT(7)

/* SYS_CFG1 (0x11) */
#define MAX86178_SYS_CFG1_RESET_MSK          BIT(0)
#define MAX86178_SYS_CFG1_SHDN_MSK           BIT(1)
#define MAX86178_SYS_CFG1_ECG_BIOZ_TIMING_DATA_MSK BIT(3)
#define MAX86178_SYS_CFG1_BIOZ_PPG_TIMING_DATA_MSK BIT(4)
#define MAX86178_SYS_CFG1_ECG_PPG_TIMING_DATA_MSK  BIT(5)
#define MAX86178_SYS_CFG1_DISABLE_I2C_MSK    BIT(6)

/* SYS_CFG2 (0x12) */
#define MAX86178_SYS_CFG2_BYP_DLY_MSK        BIT(7)

/* PIN_FUNC_CFG (0x13) */
#define MAX86178_PIN_FUNC_CFG_TRIG_FCFG_MSK  GENMASK(7, 5)
#define MAX86178_PIN_FUNC_CFG_ECG_SAMP_SYNC_FREQ_MSK GENMASK(4, 0)

/* OUT_PIN_CFG (0x14) */
#define MAX86178_OUT_PIN_CFG_TRIG_ICFG_MSK   BIT(7)
#define MAX86178_OUT_PIN_CFG_TRIG_OCFG_MSK   GENMASK(6, 5)
#define MAX86178_OUT_PIN_CFG_INT1_FCFG_MSK   GENMASK(4, 3)
#define MAX86178_OUT_PIN_CFG_INT2_FCFG_MSK   GENMASK(2, 1)
#define MAX86178_OUT_PIN_CFG_INT1_OCFG_MSK   GENMASK(1, 0)
#define MAX86178_OUT_PIN_CFG_INT2_OCFG_MSK   GENMASK(1, 0)

/* I2C_BCAST_ADDR (0x15) */
#define MAX86178_I2C_BCAST_EN_MSK            BIT(7)
#define MAX86178_I2C_BCAST_ADDR_MSK          GENMASK(6, 0)

/* PLL_CFG1 (0x18) */
#define MAX86178_PLL_CFG1_PLL_EN_MSK         BIT(0)
#define MAX86178_PLL_CFG1_PLL_LOCK_WNDW_MSK  BIT(1)
#define MAX86178_PLL_CFG1_BIOZ_NDIV_MSK      GENMASK(3, 2)
#define MAX86178_PLL_CFG1_MDIV_MSK           GENMASK(9, 8)

/* PLL_CFG3 (0x1A) */
#define MAX86178_PLL_CFG3_ECG_FDIV_MSK       GENMASK(2, 0)
#define MAX86178_PLL_CFG3_BIOZ_KDIV_MSK      GENMASK(7, 4)

/* PLL_CFG4 (0x1B) */
#define MAX86178_PLL_CFG4_ECG_NDIV_MSK       GENMASK(10, 8)

/* PLL_CFG6 (0x1D) */
#define MAX86178_PLL_CFG6_CLK_FINE_TUNE_MSK  GENMASK(4, 0)
#define MAX86178_PLL_CFG6_CLK_FREQ_SEL_MSK   GENMASK(6, 5)
#define MAX86178_PLL_CFG6_REF_CLK_SEL_MSK    BIT(7)

/* PPG_CFG1 (0x20) */
#define MAX86178_PPG_CFG1_MEAS1_EN_MSK       BIT(0)
#define MAX86178_PPG_CFG1_MEAS2_EN_MSK       BIT(1)
#define MAX86178_PPG_CFG1_MEAS3_EN_MSK       BIT(2)
#define MAX86178_PPG_CFG1_MEAS4_EN_MSK       BIT(3)
#define MAX86178_PPG_CFG1_MEAS5_EN_MSK       BIT(4)
#define MAX86178_PPG_CFG1_MEAS6_EN_MSK       BIT(5)

/* PPG_CFG2 (0x21) */
#define MAX86178_PPG_CFG2_PPG1_PWRDN_MSK     BIT(2)
#define MAX86178_PPG_CFG2_PPG2_PWRDN_MSK     BIT(3)
#define MAX86178_PPG_CFG2_PPG_SYNC_MODE_MSK  BIT(5)

/* PPG_CFG3 (0x22) */
#define MAX86178_PPG_CFG3_MEAS1_CONFIG_SEL_MSK BIT(0)
#define MAX86178_PPG_CFG3_COLLECT_RAW_DATA_MSK BIT(2)
#define MAX86178_PPG_CFG3_ALC_DISABLE_MSK    BIT(5)
#define MAX86178_PPG_CFG3_SMP_AVE_MSK        GENMASK(6, 4)

/* PPG_CFG4 (0x23) */
#define MAX86178_PPG_CFG4_PROX_DATA_EN_MSK   BIT(1)
#define MAX86178_PPG_CFG4_PROX_AUTO_MSK      BIT(6)

/* PD_BIAS (0x24) */
#define MAX86178_PD_BIAS_PD1_MSK             GENMASK(1, 0)
#define MAX86178_PD_BIAS_PD2_MSK             GENMASK(3, 2)
#define MAX86178_PD_BIAS_PD3_MSK             GENMASK(5, 4)
#define MAX86178_PD_BIAS_PD4_MSK             GENMASK(7, 6)

/* FR_CLK_DIV (0x28-0x29) */
#define MAX86178_FR_CLK_DIV_MSK              GENMASK(14, 0)

/* MEAS Configuration Bit Masks (applies to MEAS1-6) */
#define MAX86178_MEAS_SEL_DRVA_MSK           GENMASK(2, 0)
#define MAX86178_MEAS_SEL_DRVB_MSK           GENMASK(5, 3)
#define MAX86178_MEAS_SEL_AMB_MSK            BIT(6)

#define MAX86178_MEAS_CFG1_AVER_MSK          GENMASK(2, 0)
#define MAX86178_MEAS_CFG1_FILT_SEL_MSK      BIT(3)
#define MAX86178_MEAS_CFG1_FILT2_SEL_MSK     BIT(5)
#define MAX86178_MEAS_CFG1_SINC3_SEL_MSK     BIT(6)

#define MAX86178_MEAS_CFG2_PPG1_DACOFF_MSK   GENMASK(3, 0)
#define MAX86178_MEAS_CFG2_PPG1_ADC_RGE_MSK  GENMASK(5, 4)

#define MAX86178_MEAS_CFG3_PPG2_DACOFF_MSK   GENMASK(3, 0)
#define MAX86178_MEAS_CFG3_PPG2_ADC_RGE_MSK  GENMASK(5, 4)
#define MAX86178_MEAS_CFG3_TINT_MSK          GENMASK(7, 6)

#define MAX86178_MEAS_CFG4_PD_SETLNG_MSK     GENMASK(1, 0)

#define MAX86178_MEAS_CFG5_PD1_SEL_MSK       GENMASK(1, 0)
#define MAX86178_MEAS_CFG5_PD2_SEL_MSK       GENMASK(3, 2)
#define MAX86178_MEAS_CFG5_LED_RGE_MSK       GENMASK(5, 4)
#define MAX86178_MEAS_CFG5_LED_SETLNG_MSK    GENMASK(7, 6)

#define MAX86178_MEAS_CFG5_PD3_SEL_MSK       GENMASK(1, 0)
#define MAX86178_MEAS_CFG5_PD4_SEL_MSK       GENMASK(3, 2)

/* ECG_CFG1 (0x80) */
#define MAX86178_ECG_CFG1_ECG_EN_MSK         BIT(0)

enum clk_ref_sel {
	MAX86178_REF_CLK_32000 = 0,
	MAX86178_REF_CLK_32768,
	MAX86178_REF_CLK_COUNT,
};

enum ref_clk_sel {
	MAX86178_REF_CLK_SEL_INTERNAL = 0,
	MAX86178_REF_CLK_SEL_EXTERNAL,
	MAX86178_REF_CLK_SEL_COUNT,
};

enum max86178_clk_fine_tune {
	MAX86178_CLK_FINE_TUNE_SHIFT_0_0 = 0,
	MAX86178_CLK_FINE_TUNE_SHIFT_0_2,
	MAX86178_CLK_FINE_TUNE_SHIFT_0_4,
	MAX86178_CLK_FINE_TUNE_SHIFT_0_6,
	MAX86178_CLK_FINE_TUNE_SHIFT_0_8,
	MAX86178_CLK_FINE_TUNE_SHIFT_1_0,
	MAX86178_CLK_FINE_TUNE_SHIFT_1_2,
	MAX86178_CLK_FINE_TUNE_SHIFT_1_4,
	MAX86178_CLK_FINE_TUNE_SHIFT_1_6,
	MAX86178_CLK_FINE_TUNE_SHIFT_1_8,
	MAX86178_CLK_FINE_TUNE_SHIFT_2_0,
	MAX86178_CLK_FINE_TUNE_SHIFT_2_2,
	MAX86178_CLK_FINE_TUNE_SHIFT_2_4,
	MAX86178_CLK_FINE_TUNE_SHIFT_2_6,
	MAX86178_CLK_FINE_TUNE_SHIFT_2_8,
	MAX86178_CLK_FINE_TUNE_SHIFT_3_0,
	MAX86178_CLK_FINE_TUNE_SHIFT_NEG_3_2,
	MAX86178_CLK_FINE_TUNE_SHIFT_NEG_3_0,
	MAX86178_CLK_FINE_TUNE_SHIFT_NEG_2_8,
	MAX86178_CLK_FINE_TUNE_SHIFT_NEG_2_6,
	MAX86178_CLK_FINE_TUNE_SHIFT_NEG_2_4,
	MAX86178_CLK_FINE_TUNE_SHIFT_NEG_2_2,
	MAX86178_CLK_FINE_TUNE_SHIFT_NEG_2_0,
	MAX86178_CLK_FINE_TUNE_SHIFT_NEG_1_8,
	MAX86178_CLK_FINE_TUNE_SHIFT_NEG_1_6,
	MAX86178_CLK_FINE_TUNE_SHIFT_NEG_1_4,
	MAX86178_CLK_FINE_TUNE_SHIFT_NEG_1_2,
	MAX86178_CLK_FINE_TUNE_SHIFT_NEG_1_0,
	MAX86178_CLK_FINE_TUNE_SHIFT_NEG_0_8,
	MAX86178_CLK_FINE_TUNE_SHIFT_NEG_0_6,
	MAX86178_CLK_FINE_TUNE_SHIFT_NEG_0_4,
	MAX86178_CLK_FINE_TUNE_SHIFT_NEG_0_2,
	MAX86178_CLK_FINE_TUNE_SHIFT_COUNT,
}

enum max86178_bioz_ndiv {
	MAX86178_BIOZ_NDIV_256 = 0,
	MAX86178_BIOZ_NDIV_512,
	MAX86178_BIOZ_NDIV_1024,
	MAX86178_BIOZ_NDIV_1024_2,
	MAX86178_BIOZ_NDIV_COUNT,
}

enum max86178_bioz_kdiv {
	MAX86178_BIOZ_KDIV_1 = 0,
	MAX86178_BIOZ_KDIV_2,
	MAX86178_BIOZ_KDIV_4,
	MAX86178_BIOZ_KDIV_8,
	MAX86178_BIOZ_KDIV_16,
	MAX86178_BIOZ_KDIV_32,
	MAX86178_BIOZ_KDIV_64,
	MAX86178_BIOZ_KDIV_128,
	MAX86178_BIOZ_KDIV_256,
	MAX86178_BIOZ_KDIV_512,
	MAX86178_BIOZ_KDIV_1024,
	MAX86178_BIOZ_KDIV_2048,
	MAX86178_BIOZ_KDIV_4096,
	MAX86178_BIOZ_KDIV_8192,
	MAX86178_BIOZ_KDIV_8192_2,
	MAX86178_BIOZ_KDIV_8192_3,
	MAX86178_BIOZ_KDIV_COUNT,
};

enum max86178_bioz_adc_osr {
	MAX86178_BIOZ_ADC_OSR_8 = 0,
	MAX86178_BIOZ_ADC_OSR_16,
	MAX86178_BIOZ_ADC_OSR_32,
	MAX86178_BIOZ_ADC_OSR_64,
	MAX86178_BIOZ_ADC_OSR_128,
	MAX86178_BIOZ_ADC_OSR_256,
	MAX86178_BIOZ_ADC_OSR_512,
	MAX86178_BIOZ_ADC_OSR_1024,
	MAX86178_BIOZ_ADC_OSR_COUNT,	
};

enum max86178_bioz_dac_osr {
	MAX86178_BIOZ_DAC_OSR_32 = 0,
	MAX86178_BIOZ_DAC_OSR_64,
	MAX86178_BIOZ_DAC_OSR_128,
	MAX86178_BIOZ_DAC_OSR_256,
	MAX86178_BIOZ_DAC_OSR_COUNT,	
};

enum max86178_ecg_dec_rate {
	MAX86178_ECG_DEC_RATE_16 = 0,
	MAX86178_ECG_DEC_RATE_32,
	MAX86178_ECG_DEC_RATE_64,
	MAX86178_ECG_DEC_RATE_128,
	MAX86178_ECG_DEC_RATE_256,
	MAX86178_ECG_DEC_RATE_512,
	MAX86178_ECG_DEC_RATE_512_2,
	MAX86178_ECG_DEC_RATE_512_3,
	MAX86178_ECG_DEC_RATE_COUNT,
};

enum max86178_ppg_drva {
	MAX86178_PPG_DRVA_LED1_DRV = 0,
	MAX86178_PPG_DRVA_LED2_DRV,
	MAX86178_PPG_DRVA_LED3_DRV,
	MAX86178_PPG_DRVA_LED4_DRV,
	MAX86178_PPG_DRVA_LED5_DRV,
	MAX86178_PPG_DRVA_LED6_DRV,
	MAX86178_PPG_DRVA_COUNT,
};

enum max86178_ppg_drvb {
	MAX86178_PPG_DRVB_LED1_DRV = 0,
	MAX86178_PPG_DRVB_LED2_DRV,
	MAX86178_PPG_DRVB_LED3_DRV,
	MAX86178_PPG_DRVB_LED4_DRV,
	MAX86178_PPG_DRVB_LED5_DRV,
	MAX86178_PPG_DRVB_LED6_DRV,
	MAX86178_PPG_DRVB_COUNT,
};

enum max86178_ppg_amb {
	MAX86178_PPG_AMB_NORMAL_MODE = 0,
	MAX86178_PPG_AMB_DIRECT_AMB_CONV,
};

enum max86178_ppg_sinc3_sel {
	MAX86178_PPG_SINC3_OFF = 0,
	MAX86178_PPG_SINC3_ON,
};

enum max86178_ppg_avg_num {
	MAX86178_PPG_AVG_NUM_1 = 0,
	MAX86178_PPG_AVG_NUM_2,
	MAX86178_PPG_AVG_NUM_4,
	MAX86178_PPG_AVG_NUM_8,
	MAX86178_PPG_AVG_NUM_16,
	MAX86178_PPG_AVG_NUM_32,
	MAX86178_PPG_AVG_NUM_64,
	MAX86178_PPG_AVG_NUM_128,
	MAX86178_PPG_AVG_NUM_COUNT,
};

enum max86178_ppg_filt_sel {
	MAX86178_PPG_FILT_SEL_CDM = 0,
	MAX86178_PPG_FILT_SEL_FDM,
};

enum max86178_ppg_filt2_sel {
	MAX86178_PPG_FILT2_3RD_ORDER = 0,
	MAX86178_PPG_FILT2_2ND_ORDER,
};

enum max86178_ppg_adc_rge {
	MAX86178_PPG_ADC_RGE_4uA = 0,
	MAX86178_PPG_ADC_RGE_8uA,
	MAX86178_PPG_ADC_RGE_16uA,
	MAX86178_PPG_ADC_RGE_32uA,
};

enum max86178_ppg_tint {
	MAX86178_PPG_TINT_14_6us = 0,
	MAX86178_PPG_TINT_29_2us,
	MAX86178_PPG_TINT_58_6us,
	MAX86178_PPG_TINT_117_0us,
};

enum max86178_ppg_pd_setlng {
	MAX86178_PPG_PD_SETLNG_7_8us = 0,
	MAX86178_PPG_PD_SETLNG_11_8us,
	MAX86178_PPG_PD_SETLNG_15_8us,
	MAX86178_PPG_PD_SETLNG_23_8us,
};

enum max86178_led_setlng {
	MAX86178_PPG_LED_SETLNG_7_7us = 0,
	MAX86178_PPG_LED_SETLNG_11_7us,
	MAX86178_PPG_LED_SETLNG_15_7us,
	MAX86178_PPG_LED_SETLNG_23_7us,
};

enum max86178_ppg_dac_off {
	MAX86178_PPG_DAC_OFF_0uA = 0,
	MAX86178_PPG_DAC_OFF_2uA,
	MAX86178_PPG_DAC_OFF_4uA,
	MAX86178_PPG_DAC_OFF_6uA,
	MAX86178_PPG_DAC_OFF_8uA,
	MAX86178_PPG_DAC_OFF_10uA,
	MAX86178_PPG_DAC_OFF_12uA,
	MAX86178_PPG_DAC_OFF_14uA,
	MAX86178_PPG_DAC_OFF_16uA,
	MAX86178_PPG_DAC_OFF_18uA,
	MAX86178_PPG_DAC_OFF_20uA,
	MAX86178_PPG_DAC_OFF_22uA,
	MAX86178_PPG_DAC_OFF_24uA,
	MAX86178_PPG_DAC_OFF_26uA,
	MAX86178_PPG_DAC_OFF_28uA,
	MAX86178_PPG_DAC_OFF_30uA,
};

enum max86178_ppg_led_rge {
	MAX86178_PPG_LED_RGE_32mA = 0,
	MAX86178_PPG_LED_RGE_64mA,
	MAX86178_PPG_LED_RGE_96mA,
	MAX86178_PPG_LED_RGE_128mA,
};

enum max86178_ppg_pd_sel {
	MAX86178_PD_NOT_SEL = 1,
	MAX86178_PD_CONN_TO_PPG1,
	MAX86178_PD_CONN_TO_PPG2,
};

enum max86178_ppg_pwrdn {
	MAX86178_PPG_CHAN_ENABLED = 0,
	MAX86178_PPG_CHAN_PWRDN,
};

enum max86178_ppg_sync_mode {
	MAX86178_PPG_INTERNAL_SYNC = 0,
	MAX86178_PPG_EXTERNAL_SYNC,
};

enum max86178_smp_ave {
	MAX86178_SMP_AVE_1 = 0,
	MAX86178_SMP_AVE_2,
	MAX86178_SMP_AVE_4,
	MAX86178_SMP_AVE_8,
	MAX86178_SMP_AVE_16,
};

enum max86178_ppg_thresh_meas_sel {
	MAX86178_PPG_THRESH_DISABLED = 0,
	MAX86178_PPG_THRESH_MEAS1,
	MAX86178_PPG_THRESH_MEAS2,
	MAX86178_PPG_THRESH_MEAS3,
	MAX86178_PPG_THRESH_MEAS4,
	MAX86178_PPG_THRESH_MEAS5,
	MAX86178_PPG_THRESH_MEAS6,
};

enum max86178_ppg_thresh_chan_sel {
	MAX86178_PPG_THRESH_PPG_CHAN1 = 0,
	MAX86178_PPG_THRESH_PPG_CHAN2,
};

enum max86178_ppg_time_hyst {
	MAX86178_PPG_TIME_HYST_DISABLED = 0,
	MAX86178_PPG_TIME_HYST_2_SAMPLES,
	MAX86178_PPG_TIME_HYST_4_SAMPLES,
	MAX86178_PPG_TIME_HYST_8_SAMPLES,
};

enum max86178_ppg_level_hyst {
	MAX86178_PPG_LEVEL_HYST_DISABLED = 0,
	MAX86178_PPG_LEVEL_HYST_2_SAMPLES,
	MAX86178_PPG_LEVEL_HYST_4_SAMPLES,
	MAX86178_PPG_LEVEL_HYST_8_SAMPLES,
	MAX86178_PPG_LEVEL_HYST_16_SAMPLES,
	MAX86178_PPG_LEVEL_HYST_32_SAMPLES,
	MAX86178_PPG_LEVEL_HYST_64_SAMPLES,
	MAX86178_PPG_LEVEL_HYST_128_SAMPLES,
};

enum max86178_ecg_input_pol {
	MAX86178_ECG_INPUT_POL_NORMAL = 0,
	MAX86178_ECG_INPUT_POL_INVERTED,
};

enum max86178_ecg_pga_gain {
	MAX86178_ECG_PGA_GAIN_1 = 0,
	MAX86178_ECG_PGA_GAIN_2,
	MAX86178_ECG_PGA_GAIN_4,
	MAX86178_ECG_PGA_GAIN_8,
};

enum max86178_ecg_auto_rec {
	MAX86178_ECG_AUTO_REC_DISABLED = 0,
	MAX86178_ECG_AUTO_REC_ENABLED,
};

enum max86178_en_ecg_fast_rec {
	MAX86178_EN_ECG_FAST_REC_NORMAL_MODE = 0,
	MAX86178_EN_ECG_FAST_REC_MANUAL_FAST_REC_MODE,
	MAX86178_EN_ECG_FAST_REC_AUTO_FAST_REC_MODE,
};

enum max86178_ecg_cal_freq {
	MAX86178_ECG_CAL_FREQ_DIV_128 = 0,
	MAX86178_ECG_CAL_FREQ_DIV_512,
	MAX86178_ECG_CAL_FREQ_DIV_2048,
	MAX86178_ECG_CAL_FREQ_DIV_8192,
	MAX86178_ECG_CAL_FREQ_DIV_32768,
	MAX86178_ECG_CAL_FREQ_DIV_131072,
	MAX86178_ECG_CAL_FREQ_DIV_524288,
	MAX86178_ECG_CAL_FREQ_DIV_2097152,
};

enum max86178_ecg_cal_duty {
	MAX86178_ECG_CAL_DUTY_CAL_HIGH = 0,
	MAX86178_ECG_CAL_DUTY_CAL_50,
};

enum max86178_ecg_cal_en {
	MAX86178_ECG_CAL_DISABLED = 0,
	MAX86178_ECG_CAL_ENABLED,
};

enum max86178_ecg_open {
	MAX86178_ECG_PIN_INTERNALLY_CONNECTED = 0,
	MAX86178_ECG_PIN_INTERNALLY_ISOLATED,
};

enum max86178_ecg_cal_mode {
	MAX86178_ECG_CAL_MODE_UNIPOLAR = 0,
	MAX86178_ECG_CAL_MODE_BIPOLAR,
};

enum max86178_ecg_cal_mag {
	MAX86178_ECG_CAL_MAG_0_5mV = 0,
	MAX86178_ECG_CAL_MAG_1mV,
};

enum max86178_ecg_cal_sel{
	MAX86178_ECG_CAL_SEL_NO_CAL = 0,
	MAX86178_ECG_CAL_SEL_CONN_VMID_ECG,
	MAX86178_ECG_CAL_SEL_CONN_VCALP,
	MAX86178_ECG_CAL_SEL_CONN_VCALN,
};

enum max86178_ecg_lon {
	MAX86178_ECG_LON_DISABLED = 0,
	MAX86178_ECG_LON_ENABLED,
};

enum max86178_ecg_loff_en {
	MAX86178_ECG_LOFF_DISABLED = 0,
	MAX86178_ECG_LOFF_ENABLED,
};

enum max86178_ecg_loff_mode {
	MAX86178_ECG_DC_LEAD_OFF_DETECTION_MODE = 0,
	MAX86178_ECG_AC_LEAD_OFF_DETECTION_MODE,
};

enum max86178_ecg_loff_freq {
	MAX86178_ECG_LOFF_FREQ_DISABLED = 0,
	MAX86178_ECG_LOFF_FREQ_4Hz,
	MAX86178_ECG_LOFF_FREQ_8Hz,
	MAX86178_ECG_LOFF_FREQ_16Hz,
	MAX86178_ECG_LOFF_FREQ_32Hz,
	MAX86178_ECG_LOFF_FREQ_64Hz,
	MAX86178_ECG_LOFF_FREQ_128Hz,
	MAX86178_ECG_LOFF_FREQ_256Hz,
};

enum max86178_ecg_loff_ipol {
	MAX86178_ECG_LOFF_IPOL_NON_INVERTED = 0,
	MAX86178_ECG_LOFF_IPOL_INVERTED,
};

enum max86178_ecg_loff_imag {
	MAX86178_ECG_LOFF_IMAG_0nA = 0,
	MAX86178_ECG_LOFF_IMAG_5nA,
	MAX86178_ECG_LOFF_IMAG_10nA,
	MAX86178_ECG_LOFF_IMAG_20nA,
	MAX86178_ECG_LOFF_IMAG_50nA,
	MAX86178_ECG_LOFF_IMAG_100nA,
	MAX86178_ECG_LOFF_IMAG_200nA,
	MAX86178_ECG_LOFF_IMAG_400nA,
};

enum max86178_ecg_rbias_value {
	MAX86178_ECG_RBIAS_50M = 0,
	MAX86178_ECG_RBIAS_100M,
	MAX86178_ECG_RBIAS_200M,
};

enum max86178_en_ecg_rbias{
	MAX86178_EN_ECG_RBIAS_DISCONNECTED = 0,
	MAX86178_EN_ECG_RBIAS_CONNECTED,
};

enum max86178_ecg_rld_en {
	MAX86178_RLD_DISABLED = 0,
	MAX86178_RLD_ENABLED,
};

enum max86178_ecg_rld_mode {
	MAX86178_RLD_MODE_OPEN_LOOP = 0,
	MAX86178_RLD_MODE_CLOSED_LOOP,
};

enum max86178_ecg_rld_rbias {
	MAX86178_RLD_RBIAS_VMID = 0,
	MAX86178_RLD_RBIAS_VRLD,
};

enum max86178_ecg_en_rld_oor {
	MAX86178_EN_RLD_OOR_DISABLED = 0,
	MAX86178_EN_RLD_OOR_ENABLED,
};

enum max86178_ecg_actv_cm {
	MAX86178_ECG_ACTV_CM_DISABLED = 0,
	MAX86178_ECG_ACTV_CM_ENABLED,
};

enum max86178_ecg_rld_gain {
	MAX86178_RLD_GAIN_12 = 0,
	MAX86178_RLD_GAIN_24,
	MAX86178_RLD_GAIN_48,
	MAX86178_RLD_GAIN_97,
};

enum max86178_rld_ext_res {
	MAX86178_RLD_EXT_RES_INTERNAL = 0,
	MAX86178_RLD_EXT_RES_EXTERNAL,
};

enum max86178_bioz_dac_osr {
	MAX86178_BIOZ_DAC_OSR_32 = 0,
	MAX86178_BIOZ_DAC_OSR_64,
	MAX86178_BIOZ_DAC_OSR_128,
	MAX86178_BIOZ_DAC_OSR_256,
};

enum max86178_bioz_adc_osr {
	MAX86178_BIOZ_ADC_OSR_8 = 0,
	MAX86178_BIOZ_ADC_OSR_16,
	MAX86178_BIOZ_ADC_OSR_32,
	MAX86178_BIOZ_ADC_OSR_64,
	MAX86178_BIOZ_ADC_OSR_128,
	MAX86178_BIOZ_ADC_OSR_256,
	MAX86178_BIOZ_ADC_OSR_512,
	MAX86178_BIOZ_ADC_OSR_1024,
};

enum max86178_ecg_bioz_bg_en {
	MAX86178_ECG_BIOZ_BG_EN_DISABLED = 0,
	MAX86178_ECG_BIOZ_BG_EN_ENABLED,
};

enum max86178_bioz_en {
	MAX86178_BIOZ_EN_DISABLED = 0,
	MAX86178_BIOZ_EN_I_CHANNEL,
	MAX86178_BIOZ_EN_Q_CHANNEL,
	MAX86178_BIOZ_EN_UTILITY,
};

enum max86178_bioz_dhpf {
	MAX86178_BIOZ_DHPF_BYPASS = 0,
	MAX86178_BIOZ_DHPF_0_00025,
	MAX86178_BIOZ_DHPF_0_002,
};

enum max86178_bioz_dlpf {
	MAX86178_BIOZ_DLPF_BYPASS = 0,
	MAX86178_BIOZ_DLPF_0_005,
	MAX86178_BIOZ_DLPF_0_02,
	MAX86178_BIOZ_DLPF_0_08,
	MAX86178_BIOZ_DLPF_0_25,
};

enum max86178_en_bioz_thresh {
	MAX86178_EN_BIOZ_THRESH_DISABLED = 0,
	MAX86178_EN_BIOZ_THRESH_ENABLED,
};

enum max86178_bioz_ext_res {
	MAX86178_BIOZ_EXT_RES_INTERNAL = 0,
	MAX86178_BIOZ_EXT_RES_EXTERNAL,
};

enum max86178_bioz_vdrv_mag {
	MAX86178_BIOZ_VDRV_MAG_50mV = 0,
	MAX86178_BIOZ_VDRV_MAG_100mV,
	MAX86178_BIOZ_VDRV_MAG_250mV,
	MAX86178_BIOZ_VDRV_MAG_500mV,
};

/**
 * 
 * @brief MAX86178 bus configuration
 *
 */
union max86178_bus {
#if defined(MAX86178_BUS_I2C)
	struct i2c_dt_spec i2c;
#endif /* MAX86178_BUS_I2C */
#if defined(MAX86178_BUS_SPI)
	struct spi_dt_spec spi;
#endif /* MAX86178_BUS_SPI */
	uint8_t dummy;
};

/**
 * @brief Function pointer to check if bus is ready
 *
 */
typedef int (*max86178_bus_is_ready_fn)(const union max86178_bus *bus);

/**
 * @brief Function pointer for register access
 *
 */
typedef int (*max86178_reg_access_fn)(const struct device *dev, bool read, uint8_t reg_addr,
				      uint8_t *data, size_t length);

/* Clock Configuration Structs */
struct max86178_clk_ppg_cfg {
	uint16_t ppg_fr_clk_div : 15;
};

struct max86178_clk_ecg_cfg {
	uint8_t ecg_fdiv : 3;
	uint16_t ecg_ndiv : 11;
	enum max86178_ecg_dec_rate ecg_dec_rate;
};

struct max86178_clk_bioz_cfg {
	enum max86178_bioz_kdiv bioz_kdiv;
	enum max86178_bioz_ndiv bioz_ndiv;
	enum max86178_bioz_adc_osr bioz_adc_osr;
	enum max86178_bioz_dac_osr bioz_dac_osr;
};

struct max86190_clk_osc_cfg {
	enum ref_clk_sel ref_clk;
	enum clk_ref_sel clk_freq_sel;
	enum max86178_clk_fine_tune clk_fine_tune;
	uint16_t mdiv;
};

struct max86178_clk_cfg {
	struct max86190_clk_osc_cfg osc_cfg;
	struct max86178_clk_ppg_cfg ppg_cfg;
	struct max86178_clk_ecg_cfg ecg_cfg;
	struct max86178_clk_bioz_cfg bioz_cfg;
};

/* PPG Structs */
struct max86178_ppg_meas_cfg {
	enum max86178_ppg_drva drva;
	enum max86178_ppg_drvb drvb;
	enum max86178_ppg_amb amb_mode;
	enum max86178_ppg_avg_num avg_num;
	enum max86178_ppg_sinc3_sel sinc3_sel;
	enum max86178_ppg_filt_sel filt_sel;
	enum max86178_ppg_filt2_sel filt2_sel;
	enum max86178_ppg_dac_off ppg1_dac_off;
	enum max86178_ppg_adc_rge ppg1_adc_rge;
	enum max86178_ppg_dac_off ppg2_dac_off;
	enum max86178_ppg_adc_rge ppg2_adc_rge;
	enum max86178_ppg_tint tint;
	enum max86178_ppg_pd_setlng pd_setlng;
	enum max86178_led_setlng led_setlng;
	enum max86178_ppg_led_rge led_rge;
	enum max86178_ppg_pd_sel pd1_sel;
	enum max86178_ppg_pd_sel pd2_sel;
	enum max86178_ppg_pd_sel pd3_sel;
	enum max86178_ppg_pd_sel pd4_sel;
	uint8_t drva_pa;
	uint8_t drvb_pa;
};

struct max86178_ppg_threshold_cfg {
	enum max86178_ppg_thresh_meas_sel thresh1_meas_sel;
	enum max86178_ppg_thresh_meas_sel thresh2_meas_sel;
	enum max86178_ppg_thresh_chan_sel thresh1_chan_sel;
	enum max86178_ppg_thresh_chan_sel thresh2_chan_sel;
	enum max86178_ppg_time_hyst time_hyst;
	enum max86178_ppg_level_hyst level_hyst;
	uint16_t thresh1_hi;
	uint16_t thresh1_lo;
	uint16_t thresh2_hi;
	uint16_t thresh2_lo;
};

struct max86178_ppg_cfg {
	bool meas_en[6];
	enum max86178_ppg_pwrdn ppg1_pwrdn;
	enum max86178_ppg_pwrdn ppg2_pwrdn;
	enum max86178_ppg_sync_mode ppg_sync_mode;
	bool prox_data_en;
	bool prox_auto_en;
	bool alc_disable;
	bool collect_raw_data;
	bool meas1_config_sel;
	enum max86178_smp_ave smp_ave;
	struct max86178_ppg_meas_cfg meas_cfg[6];
	struct max86178_ppg_threshold_cfg threshold_cfg;
};

/* ECG Structs */
struct max86178_ecg_setup {
	bool ecg_en;
	enum max86178_ecg_input_pol ecg_input_pol;
	enum max86178_ecg_pga_gain ecg_pga_gain;
	uint8_t ecg_ina_rge : 2;
	uint8_t ecg_ina_gain : 3;
	bool ecg_imp_hi;
	enum max86178_ecg_auto_rec ecg_auto_rec;
	uint8_t ecg_mux_sel : 2;
	enum max86178_en_ecg_fast_rec en_ecg_fast_rec;
	uint8_t ecg_fast_rec_thres : 6;
};

struct max86178_ecg_calibration {
	uint16_t ecg_cal_high : 11;
	enum max86178_ecg_cal_freq ecg_freq;
	enum max86178_ecg_cal_duty ecg_cal_duty;
	enum max86178_ecg_cal_en ecg_cal_en;
	enum max86178_ecg_open ecg_open_p;
	enum max86178_ecg_open ecg_open_n;
	enum max86178_ecg_cal_mode ecg_cal_mode;
	enum max86178_ecg_cal_mag ecg_cal_mag;
	enum max86178_ecg_cal_sel ecg_cal_p_sel;
	enum max86178_ecg_cal_sel ecg_cal_n_sel;
};

struct max86178_ecg_lead_detect {
	enum max86178_ecg_lon en_ecg_lon;
	enum max86178_ecg_loff_en en_ecg_loff;
	enum max86178_ecg_loff_mode ecg_loff_mode;
	enum max86178_ecg_loff_freq ecg_loff_freq;
	enum max86178_ecg_loff_ipol ecg_loff_ipol;
	enum max86178_ecg_loff_imag ecg_loff_current_mag;
	uint8_t ecg_loff_thresh : 4;
};

struct max86178_lead_bias {
	enum max86178_ecg_rbias_value ecg_rbias_value;
	enum max86178_en_ecg_rbias en_ecg_rbias_p;
	enum max86178_en_ecg_rbias en_ecg_rbias_n;
};

struct max86178_rld_cfg {
	enum max86178_ecg_rld_en rld_en;
	enum max86178_ecg_rld_mode rld_mode;
	enum max86178_ecg_rld_rbias rld_rbias;
	enum max86178_ecg_en_rld_oor en_rld_oor;
	enum max86178_ecg_actv_cm actv_cm_p;
	enum max86178_ecg_actv_cm actv_cm_n;
	enum max86178_ecg_rld_gain rld_gain;
	bool rld_ext_res;
	bool rld_sel_ecg;
	uint8_t rld_bw : 2;
	uint8_t body_bias_dac : 4;
};

struct max86178_ecg_cfg {
	struct max86178_ecg_setup setup;
	struct max86178_ecg_calibration calibration;
	struct max86178_ecg_lead_detect lead_detect;
	struct max86178_lead_bias lead_bias;
	struct max86178_rld_cfg rld_cfg;
};


struct max86178_bioz_setup {
	enum max86178_bioz_dac_osr bioz_dac_osr;
	enum max86178_bioz_adc_osr bioz_adc_osr;
	enum max86178_ecg_bioz_bg_en ecg_bioz_bg_en;
	enum max86178_bioz_en bioz_en;
	enum max86178_bioz_dhpf bioz_dhpf;
	enum max86178_bioz_dlpf bioz_dlpf;
	enum max86178_en_bioz_thresh en_bioz_thresh;
	enum max86178_bioz_ext_res bioz_ext_res;
	uint8_t bioz_vdrv_mag : 2;
	uint8_t bioz_idrv_rge : 2;
	uint8_t bioz_drv_mode : 2;
	bool en_util_mode;
	bool bioz_dc_code_sel;
	uint8_t bioz_dc_dac_code : 7;
	uint8_t bioz_ahpf : 4;
	bool bioz_ina_mode;
	bool bioz_dm_dis;
	bool bioz_gain : 2;
	bool bioz_ext_cap;
	bool bioz_dc_restore;
	bool bioz_drv_reset;
	bool bioz_dac_reset;
	uint8_t bioz_amp_rge : 2;
	uint8_t bioz_amp_bw : 2;
	bool rld_sel_bioz;
	bool rld_drv;
	bool bioz_cmres_dis;
	bool bioz_stbyon;
	bool bioz_ipol;
	bool bioz_fast;
	bool bioz_ina_chop_en;
	bool bioz_ch_fsel;
	uint8_t bioz_lo_thresh;
	uint8_t bioz_hi_thresh;
};

struct max86178_bioz_calibration {
	uint8_t bmux_rsel : 2;
	bool bmux_bist_en;
	bool connect_cal_only;
	bool bioz_mux_en;
	bool bioz_cal_en;
	uint8_t bmux_gsr_rsel : 2;
	bool gsr_load_en;
	bool en_ext_inload;
	bool en_int_inload;
	uint8_t bip_assign : 2;
	uint8_t bin_assign : 2;
	uint8_t drvp_assign : 2;
	uint8_t drvn_assign : 2;
};

struct max86178_bioz_lead_detect {
	bool en_bioz_lon;
	bool en_bioz_loff;
	bool en_ext_bioz_loff;
	bool bioz_loff_ipol;
	uint8_t bioz_loff_current_mag : 3;
	bool resp_cg_mag4x;
	uint8_t resp_cg_mag : 3;
	uint8_t bioz_loff_thresh : 4;
};

struct max86178_lead_bias {
	bool en_bioz_rbias_p;
	bool en_bioz_rbias_n;
	uint8_t bioz_rbias_value : 2;
};

struct max86178_bioz_cfg {
	struct max86178_bioz_setup setup;
	struct max86178_bioz_calibration calibration;
	struct max86178_bioz_lead_detect lead_detect;
	struct max86178_lead_bias lead_bias;
};

struct max86178_resp_setup {
	bool resp_en;
	uint8_t cg_mode : 2;
	uint8_t cg_chop_clk : 2;
	uint8_t cg_lpf_duty : 3;
};


/**
 * @brief MAX86178 driver data structure
 *
 */
struct max86178_data {
	/* Add driver runtime data here */
};

/**
 * @brief MAX86178 device configuration structure
 *
 */
struct max86178_dev_config {
	const union max86178_bus bus;
	max86178_bus_is_ready_fn bus_is_ready;
	max86178_reg_access_fn reg_access;
	/* Add device configuration parameters here */
};

/**
 * @brief MAX86178 register read function
 *
 * @param dev Device pointer
 * @param reg_addr Register address
 * @param data Data buffer
 * @param length Number of bytes to read
 * @return int 0 on success, negative error code on failure
 */
int max86178_reg_read(const struct device *dev, uint8_t reg_addr, uint8_t *data, size_t length);

/**
 * @brief MAX86178 register write function
 *
 * @param dev Device pointer
 * @param reg_addr Register address
 * @param data Data buffer
 * @param length Number of bytes to write
 * @return int 0 on success, negative error code on failure
 */
int max86178_reg_write(const struct device *dev, uint8_t reg_addr, uint8_t *data, size_t length);

/**
 * @brief MAX86178 register update function
 *
 * @param dev Device pointer
 * @param reg_addr Register address
 * @param mask Bit mask
 * @param value Value to set
 * @return int 0 on success, negative error code on failure
 */
int max86178_reg_update(const struct device *dev, uint8_t reg_addr, uint8_t mask, uint8_t value);

#endif /* ZEPHYR_DRIVERS_SENSOR_MAX86178_MAX86178_H_ */
