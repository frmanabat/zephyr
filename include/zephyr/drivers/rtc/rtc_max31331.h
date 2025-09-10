/*
 * Copyright (c) 2025 Analog Devices Inc.
 * @author Francis Roi Manabat <francisroi.manabat@analog.com>
 * SPDX-License-Identifier: Apache-2.0
 */


#include <zephyr/drivers/i2c.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <errno.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/drivers/gpio.h>

#define MAX31331_STATUS_REG       0x00u
#define MAX31331_INTERRUPT_ENABLE 0x01u
#define MAX31331_RTC_RESET        0x02u
#define MAX31331_RTC_CONFIG1      0x03u
#define MAX31331_RTC_CONFIG2      0x04u
#define MAX31331_TIMESTAMP_CONFIG 0x05u
#define MAX31331_TIMER_CONFIG     0x06u
#define MAX31331_SECONDS_1_128    0x07u
#define MAX31331_SECONDS          0x08u
#define MAX31331_MINUTES          0x09u
#define MAX31331_HOURS            0x0Au
#define MAX31331_DAY              0x0Bu
#define MAX31331_DATE             0x0Cu
#define MAX31331_MONTH            0x0Du
#define MAX31331_YEAR             0x0Eu
#define MAX31331_ALARM_1_SECONDS  0x0Fu
#define MAX31331_ALARM_1_MINUTES  0x10u
#define MAX31331_ALARM_1_HOURS    0x11u
#define MAX31331_ALARM_1_DAY_DATE 0x12u
#define MAX31331_ALARM_1_MONTH    0x13u
#define MAX31331_ALARM_1_YEAR     0x14u
#define MAX31331_ALARM_2_MINUTES  0x15u
#define MAX31331_ALARM_2_HOURS    0x16u
#define MAX31331_ALARM_2_DAY_DATE 0x17u
#define MAX31331_TIMER_COUNT      0x18u
#define MAX31331_TIMER_INIT       0x19u
#define MAX31331_POWER_MANAGEMENT 0x1Au
#define MAX31331_TRICKLE_REG      0x1Bu
#define MAX31331_OFFSET_HIGH      0x1Du
#define MAX31331_OFFSET_LOW       0x1Eu

/** TS RAM REGISTERS */
#define MAX31331_TS0_SEC_1_128 0x20u
#define MAX31331_TS0_SEC       0x21u
#define MAX31331_TS0_MIN       0x22u
#define MAX31331_TS0_HOUR      0x23u
#define MAX31331_TS0_DATE      0x24u
#define MAX31331_TS0_MONTH     0x25u
#define MAX31331_TS0_YEAR      0x26u
#define MAX31331_TS0_FLAGS     0x27u
#define MAX31331_TS1_SEC_1_128 0x28u
#define MAX31331_TS1_SEC       0x29u
#define MAX31331_TS1_MIN       0x2Au
#define MAX31331_TS1_HOUR      0x2Bu
#define MAX31331_TS1_DATE      0x2Cu
#define MAX31331_TS1_MONTH     0x2Du
#define MAX31331_TS1_YEAR      0x2Eu
#define MAX31331_TS1_FLAGS     0x2Fu
#define MAX31331_TS2_SEC_1_128 0x30u
#define MAX31331_TS2_SEC       0x31u
#define MAX31331_TS2_MIN       0x32u
#define MAX31331_TS2_HOUR      0x33u
#define MAX31331_TS2_DATE      0x34u
#define MAX31331_TS2_MONTH     0x35u
#define MAX31331_TS2_YEAR      0x36u
#define MAX31331_TS2_FLAGS     0x37u
#define MAX31331_TS3_SEC_1_128 0x38u
#define MAX31331_TS3_SEC       0x39u
#define MAX31331_TS3_MIN       0x3Au
#define MAX31331_TS3_HOUR      0x3Bu
#define MAX31331_TS3_DATE      0x3Cu
#define MAX31331_TS3_MONTH     0x3Du
#define MAX31331_TS3_YEAR      0x3Eu
#define MAX31331_TS3_FLAGS     0x3Fu

/** REGISTER MASKS */

/** STATUS REGISTERS MASKS*/
#define POWER_SOURCE_MASK         BIT(7)
#define OSCILLATOR_STOP_FLAG_MASK BIT(6)
#define POWER_FAIL_MASK           BIT(5)
#define VBATLOW_MASK              BIT(4)
#define DIGITAL_INTERRUPT_MASK    BIT(3)
#define TIMER_FLAG_MASK           BIT(2)
#define ALARM_2_FLAG_MASK         BIT(1)
#define ALARM_1_FLAG_MASK         BIT(0)

/** INTERRUPT ENABLE MASKS */
#define DISABLE_OSCILLATR_FLAG_MASK      BIT(6)
#define POWER_FAIL_INTERRUPT_ENABLE_MASK BIT(5)
#define VBATLOW_INTERRUPT_ENABLE_MASK    BIT(4)
#define DIGITAL_INTERRUPT_ENABLE_MASK    BIT(3)
#define TIMER_INTERRUPT_ENABLE_MASK      BIT(2)
#define ALARM_2_INTERRUPT_ENABLE_MASK    BIT(1)
#define ALARM_1_INTERRUPT_ENABLE_MASK    BIT(0)

/** RTC RESET MASKS */
#define SW_RESET_MASK BIT(0)

/** RTC CONFIG1 MASKS */
#define EN_IOUTPUT_MASK             BIT(6)
#define ALARM1_AUTO_CLEAR_MASK      GENMASK(5, 4)
#define DIGITAL_INPUT_POLARITY_MASK BIT(3)
#define ENABLE_I2C_TIMEOUT_MASK     BIT(1)
#define ENABLE_OSCILLATOR_MASK      BIT(0)

/** RTC CONFIG2 MASKS */
#define CLKOUT_ENABLE_MASK BIT(2)
#define CLKO_HZ_MASK       GENMASK(1, 0)

/** RTC TIMESTAMP CONFIG MASKS */
#define TS_VBAT_LOW_EN_MASK         BIT(5)
#define TS_POWER_SUPPLY_SWITCH_MASK BIT(4)
#define TS_DIN_MASK                 BIT(3)
#define TS_OVERWRITE_MASK           BIT(2)
#define TS_REG_RESET_MASK           BIT(1)
#define TS_ENABLE_MASK              BIT(0)

/** RTC TIMER CONFIG MASKS */
#define TIMER_ENABLE_MASK         BIT(4)
#define TIMER_PAUSE_MASK          BIT(3)
#define TIMER_REPEAT_MODE_MASK    BIT(2)
#define TIMER_FREQUENCY_SELECTION GENMASK(1, 0)

/** RTC SECONDS 1_128 MASKS */
#define SECONDS_1_2_MASK   BIT(6)
#define SECONDS_1_4_MASK   BIT(5)
#define SECONDS_1_8_MASK   BIT(4)
#define SECONDS_1_16_MASK  BIT(3)
#define SECONDS_1_32_MASK  BIT(2)
#define SECONDS_1_64_MASK  BIT(1)
#define SECONDS_1_128_MASK BIT(0)

/** RTC SECONDS MASKS */
#define SECONDS_10_MASK    GENMASK(6, 4)
#define SECONDS_MASK       GENMASK(3, 0)
#define SECONDS_FIELD_MASK (SECONDS_10_MASK | SECONDS_MASK)

/** RTC MINUTES MASKS */
#define MINUTES_10_MASK    GENMASK(6, 4)
#define MINUTES_MASK       GENMASK(3, 0)
#define MINUTES_FIELD_MASK (MINUTES_10_MASK | MINUTES_MASK)

/** RTC HOURS MASKS */
#define F24_12_MASK        BIT(6)
#define HOUR_20_AM_PM_MASK BIT(5)
#define HOURS_10_MASK      BIT(4)
#define HOURS_MASK         GENMASK(3, 0)
#define HOURS_FIELD_MASK   (HOUR_20_AM_PM_MASK | HOURS_10_MASK | HOURS_MASK)

/** RTC DAY MASKS */
#define DAY_FIELD_MASK      GENMASK(2, 0)
#define MAX31331_DAY_OFFSET -1

/** RTC DATE MASKS */
#define DATE_10_MASK    GENMASK(5, 4)
#define DATE_MASK       GENMASK(3, 0)
#define DATE_FIELD_MASK (DATE_10_MASK | DATE_MASK)

/** RTC MONTH MASKS */
#define MONTH_10_MASK    BIT(4)
#define MONTH_MASK       GENMASK(3, 0)
#define MONTH_FIELD_MASK (MONTH_10_MASK | MONTH_MASK)

/** RTC YEAR MASKS */
#define YEAR_10_MASK    GENMASK(7, 4)
#define YEAR_MASK       GENMASK(3, 0)
#define YEAR_FIELD_MASK (YEAR_10_MASK | YEAR_MASK)

/** CENTURY DETERMINANT*/
#define MAX31331_YEAR_2100 (2100 - 2000)

/** ALARM 1 MASKS */

/** ALARM 1 SECONDS MASKS */
#define ALARM_1_SECONDS_ENABLE_MASK BIT(7)
#define ALARM_1_SECONDS_10_MASK     GENMASK(6, 4)
#define ALARM_1_SECONDS_MASK        GENMASK(3, 0)
#define ALARM_1_SECONDS_FIELD_MASK  (ALARM_1_SECONDS_10_MASK | ALARM_1_SECONDS_MASK)

/** ALARM 1 MINUTES MASKS */
#define ALARM_1_MINUTES_ENABLE_MASK BIT(7)
#define ALARM_1_MINUTES_10_MASK     GENMASK(6, 4)
#define ALARM_1_MINUTES_MASK        GENMASK(3, 0)
#define ALARM_1_MINUTES_FIELD_MASK  (ALARM_1_MINUTES_10_MASK | ALARM_1_MINUTES_MASK)

/** ALARM 1 HOURS MASKS */
#define ALARM_1_HOURS_ENABLE_MASK BIT(7)
#define ALARM_1_HR_20_AM_PM_MASK  BIT(5)
#define ALARM_1_HOURS_10_MASK     BIT(4)
#define ALARM_1_HOURS_MASK        GENMASK(3, 0)
#define ALARM_1_HOURS_FIELD_MASK                                                                   \
	(ALARM_1_HOURS_10_MASK | ALARM_1_HOURS_MASK | ALARM_1_HR_20_AM_PM_MASK)

/** ALARM 1 DAY/DATE MASKS */
#define ALARM_1_DAY_DATE_ENABLE_MASK BIT(7)
#define ALARM_1_DAY_DATE_OP_MASK     BIT(6)
#define ALARM_1_DATE_10_MASK         GENMASK(5, 4)
#define ALARM_1_DAY_DATE_MASK        GENMASK(3, 0)
#define ALARM_1_DAY_DATE_FIELD_MASK  (ALARM_1_DATE_10_MASK | ALARM_1_DAY_DATE_MASK)

/** ALARM 1 MONTH MASKS */
#define ALARM_1_MONTH_ENABLE_MASK BIT(7)
#define ALARM_1_YEAR_ENABLE_MASK  BIT(6)
#define ALARM_1_MONTH_10_MASK     BIT(4)
#define ALARM_1_MONTH_MASK        GENMASK(3, 0)
#define ALARM_1_MONTH_FIELD_MASK  (ALARM_1_MONTH_10_MASK | ALARM_1_MONTH_MASK)

/** ALARM 1 YEAR MASKS */
#define ALARM_1_YEAR_10_MASK    GENMASK(7, 4)
#define ALARM_1_YEAR_MASK       GENMASK(3, 0)
#define ALARM_1_YEAR_FIELD_MASK (ALARM_1_YEAR_10_MASK | ALARM_1_YEAR_MASK)
/** ALARM 2 MASKS */

/** ALARM 2 MINUTES MASKS */
#define ALARM_2_MINUTES_ENABLE_MASK BIT(7)
#define ALARM_2_MINUTES_10_MASK     GENMASK(6, 4)
#define ALARM_2_MINUTES_MASK        GENMASK(3, 0)
#define ALARM_2_MINUTES_FIELD_MASK  (ALARM_2_MINUTES_10_MASK | ALARM_2_MINUTES_MASK)

/** ALARM 2 HOURS MASKS */
#define ALARM_2_HOURS_ENABLE_MASK BIT(7)
#define ALARM_2_HR_20_AM_PM_MASK  BIT(5)
#define ALARM_2_HOURS_10_MASK     BIT(4)
#define ALARM_2_HOURS_MASK        GENMASK(3, 0)
#define ALARM_2_HOURS_FIELD_MASK                                                                   \
	(ALARM_2_HOURS_10_MASK | ALARM_2_HOURS_MASK | ALARM_2_HR_20_AM_PM_MASK)

/** ALARM 2 DAY/DATE MASKS */
#define ALARM_2_DAY_DATE_ENABLE_MASK BIT(7)
#define ALARM_2_DAY_DATE_OP_MASK     BIT(6)
#define ALARM_2_DATE_10_MASK         GENMASK(5, 4)
#define ALARM_2_DAY_DATE_MASK        GENMASK(3, 0)
#define ALARM_2_DAY_DATE_FIELD_MASK  (ALARM_2_DATE_10_MASK | ALARM_2_DAY_DATE_MASK)

/** TIMER COUNT MASKS */
#define TIMER_COUNT_MASK GENMASK(7, 0)

/** TIMER INITIAL MASKS */
#define TIMER_INIT_MASK GENMASK(7, 0)

/** POWER MANAGEMENT MASKS */
#define BACKUP_BATTERY_SELECT_MASK BIT(1)
#define MANUAL_SEL_MASK            BIT(0)

/** TRICKLE REG MASKS */
#define TRICKLE_MASKS       GENMASK(3, 1)
#define TRICKLE_ENABLE_MASK BIT(0)

/** OFFSET HIGH */
#define COMPWORD_MSB_MASK GENMASK(7, 0)

/** OFFSET LOW */
#define COMPWORD_LSB_MASK GENMASK(7, 0)

/** TIMESTAMP 0 MASKS */

/** TIMESTAMP 0 SECONDS 1_128 MASKS */
#define TS0_SECONDS_1_2_MASK   BIT(6)
#define TS0_SECONDS_1_4_MASK   BIT(5)
#define TS0_SECONDS_1_8_MASK   BIT(4)
#define TS0_SECONDS_1_16_MASK  BIT(3)
#define TS0_SECONDS_1_32_MASK  BIT(2)
#define TS0_SECONDS_1_64_MASK  BIT(1)
#define TS0_SECONDS_1_128_MASK BIT(0)

/** TIMESTAMP 0 SECONDS MASKS */
#define TS0_SECONDS_10_MASK GENMASK(6, 4)
#define TS0_SECONDS_MASK    GENMASK(3, 0)

/** TIMESTAMP 0 MINUTES MASKS */
#define TS0_MINUTES_10_MASK GENMASK(6, 4)
#define TS0_MINUTES_MASK    GENMASK(3, 0)

/** TIMESTAMP 0 HOURS MASKS */
#define TS0_F24_12_MASK        BIT(6)
#define TS0_HOUR_20_AM_PM_MASK BIT(5)
#define TS0_HOURS_10_MASK      BIT(4)
#define TS0_HOURS_MASK         GENMASK(3, 0)

/** TIMESTAMP 0 DATE MASKS */
#define TS0_DATE_10_MASK GENMASK(5, 4)
#define TS0_DATE_MASK    GENMASK(3, 0)

/** TIMESTAMP 0 MONTH MASKS */
#define CENTURY_MASK      BIT(7)
#define TS0_MONTH_10_MASK BIT(4)
#define TS0_MONTH_MASK    GENMASK(3, 0)

/** TIMESTAMP 0 YEAR MASKS */
#define TS0_YEAR_10_MASK GENMASK(7, 4)
#define TS0_YEAR_MASK    GENMASK(3, 0)

/** TIMESTAMP 0 FLAGS MASKS */
#define TS0_V_LOW_MASK               BIT(3)
#define TS0_V_BAT_TO_VCC_SWITCH_MASK BIT(2)
#define TS0_VCC_TO_V_BAT_SWITCH_MASK BIT(1)
#define TS0_DIN_MASK                 BIT(0)

/** TIMESTAMP 1 MASKS */

/** TIMESTAMP 1 SECONDS 1_128 MASKS */
#define TS1_SECONDS_1_2_MASK   BIT(6)
#define TS1_SECONDS_1_4_MASK   BIT(5)
#define TS1_SECONDS_1_8_MASK   BIT(4)
#define TS1_SECONDS_1_16_MASK  BIT(3)
#define TS1_SECONDS_1_32_MASK  BIT(2)
#define TS1_SECONDS_1_64_MASK  BIT(1)
#define TS1_SECONDS_1_128_MASK BIT(0)

/** TIMESTAMP 1 SECONDS MASKS */
#define TS1_SECONDS_10_MASK GENMASK(6, 4)
#define TS1_SECONDS_MASK    GENMASK(3, 0)

/** TIMESTAMP 1 MINUTES MASKS */
#define TS1_MINUTES_10_MASK GENMASK(6, 4)
#define TS1_MINUTES_MASK    GENMASK(3, 0)

/** TIMESTAMP 1 HOURS MASKS */
#define TS1_F24_12_MASK        BIT(6)
#define TS1_HOUR_20_AM_PM_MASK BIT(5)
#define TS1_HOURS_10_MASK      BIT(4)
#define TS1_HOURS_MASK         GENMASK(3, 0)

/** TIMESTAMP 1 DATE MASKS */
#define TS1_DATE_10_MASK GENMASK(5, 4)
#define TS1_DATE_MASK    GENMASK(3, 0)

/** TIMESTAMP 1 MONTH MASKS */
#define TS1_CENTURY_MASK  BIT(7)
#define TS1_MONTH_10_MASK BIT(4)
#define TS1_MONTH_MASK    GENMASK(3, 0)

/** TIMESTAMP 1 YEAR MASKS */
#define TS1_YEAR_10_MASK GENMASK(7, 4)
#define TS1_YEAR_MASK    GENMASK(3, 0)

/** TIMESTAMP 1 FLAGS MASKS */
#define TS1_V_LOW_MASK               BIT(3)
#define TS1_V_BAT_TO_VCC_SWITCH_MASK BIT(2)
#define TS1_VCC_TO_V_BAT_SWITCH_MASK BIT(1)
#define TS1_DIN_MASK                 BIT(0)

/** TIMESTAMP 2 MASKS */
/** TIMESTAMP 2 SECONDS 1_128 MASKS */
#define TS2_SECONDS_1_2_MASK   BIT(6)
#define TS2_SECONDS_1_4_MASK   BIT(5)
#define TS2_SECONDS_1_8_MASK   BIT(4)
#define TS2_SECONDS_1_16_MASK  BIT(3)
#define TS2_SECONDS_1_32_MASK  BIT(2)
#define TS2_SECONDS_1_64_MASK  BIT(1)
#define TS2_SECONDS_1_128_MASK BIT(0)

/** TIMESTAMP 2 SECONDS MASKS */
#define TS2_SECONDS_10_MASK GENMASK(6, 4)
#define TS2_SECONDS_MASK    GENMASK(3, 0)

/** TIMESTAMP 2 MINUTES MASKS */
#define TS2_MINUTES_10_MASK GENMASK(6, 4)
#define TS2_MINUTES_MASK    GENMASK(3, 0)

/** TIMESTAMP 2 HOURS MASKS */
#define TS2_F24_12_MASK        BIT(6)
#define TS2_HOUR_20_AM_PM_MASK BIT(5)
#define TS2_HOURS_10_MASK      BIT(4)
#define TS2_HOURS_MASK         GENMASK(3, 0)

/** TIMESTAMP 2 DATE MASKS */
#define TS2_DATE_10_MASK GENMASK(5, 4)
#define TS2_DATE_MASK    GENMASK(3, 0)

/** TIMESTAMP 2 MONTH MASKS */
#define TS2_CENTURY_MASK  BIT(7)
#define TS2_MONTH_10_MASK BIT(4)
#define TS2_MONTH_MASK    GENMASK(3, 0)

/** TIMESTAMP 2 YEAR MASKS */
#define TS2_YEAR_10_MASK GENMASK(7, 4)
#define TS2_YEAR_MASK    GENMASK(3, 0)

/** TIMESTAMP 2 FLAGS MASKS */
#define TS2_V_LOW_MASK               BIT(3)
#define TS2_V_BAT_TO_VCC_SWITCH_MASK BIT(2)
#define TS2_VCC_TO_V_BAT_SWITCH_MASK BIT(1)
#define TS2_DIN_MASK                 BIT(0)

/** TIMESTAMP 3 MASKS */
/** TIMESTAMP 3 SECONDS 1_128 MASKS */
#define TS3_SECONDS_1_2_MASK   BIT(6)
#define TS3_SECONDS_1_4_MASK   BIT(5)
#define TS3_SECONDS_1_8_MASK   BIT(4)
#define TS3_SECONDS_1_16_MASK  BIT(3)
#define TS3_SECONDS_1_32_MASK  BIT(2)
#define TS3_SECONDS_1_64_MASK  BIT(1)
#define TS3_SECONDS_1_128_MASK BIT(0)

/** TIMESTAMP 3 SECONDS MASKS */
#define TS3_SECONDS_10_MASK GENMASK(6, 4)
#define TS3_SECONDS_MASK    GENMASK(3, 0)

/** TIMESTAMP 3 MINUTES MASKS */
#define TS3_MINUTES_10_MASK GENMASK(6, 4)
#define TS3_MINUTES_MASK    GENMASK(3, 0)

/** TIMESTAMP 3 HOURS MASKS */
#define TS3_F24_12_MASK        BIT(6)
#define TS3_HOUR_20_AM_PM_MASK BIT(5)
#define TS3_HOURS_10_MASK      BIT(4)
#define TS3_HOURS_MASK         GENMASK(3, 0)

/** TIMESTAMP 3 DATE MASKS */
#define TS3_DATE_10_MASK GENMASK(5, 4)
#define TS3_DATE_MASK    GENMASK(3, 0)

/** TIMESTAMP 3 MONTH MASKS */
#define TS3_CENTURY_MASK  BIT(7)
#define TS3_MONTH_10_MASK BIT(4)
#define TS3_MONTH_MASK    GENMASK(3, 0)

/** TIMESTAMP 3 YEAR MASKS */
#define TS3_YEAR_10_MASK GENMASK(7, 4)
#define TS3_YEAR_MASK    GENMASK(3, 0)

/** TIMESTAMP 3 FLAGS MASKS */
#define TS3_V_LOW_MASK               BIT(3)
#define TS3_V_BAT_TO_VCC_SWITCH_MASK BIT(2)
#define TS3_VCC_TO_V_BAT_SWITCH_MASK BIT(1)
#define TS3_DIN_MASK                 BIT(0)

#define MAX31331_RTC_TIME_MASK                                                                     \
	(RTC_ALARM_TIME_MASK_SECOND | RTC_ALARM_TIME_MASK_MINUTE | RTC_ALARM_TIME_MASK_HOUR |      \
	 RTC_ALARM_TIME_MASK_MONTH | RTC_ALARM_TIME_MASK_MONTHDAY | RTC_ALARM_TIME_MASK_YEAR |     \
	 RTC_ALARM_TIME_MASK_WEEKDAY)

#define ALARM_COUNT 2

#ifdef CONFIG_RTC_ALARM
struct rtc_max31331_alarm {
	rtc_alarm_callback callback;
	void *user_data;
};
#endif
typedef void (*rtc_max31331_timestamp_callback)(const struct device *dev, void *user_data);
struct rtc_max31331_data {
#ifdef CONFIG_RTC_ALARM
	struct rtc_max31331_alarm alarms[ALARM_COUNT];
#endif
#ifdef CONFIG_RTC_UPDATE
	struct rtc_max31331_update update;
#endif

#ifdef CONFIG_RTC_MAX31331_TIMESTAMPING
	struct rtc_time timestamp_buffer[4];
	rtc_max31331_timestamp_callback ts_callback;
	void *ts_user_data;
#endif
	struct gpio_callback int_callback;
#if defined(CONFIG_RTC_MAX31331_INTERRUPT_GLOBAL_THREAD)
	struct k_work work;
#elif defined(CONFIG_RTC_MAX31331_INTERRUPT_OWN_THREAD)
	struct k_thread thread;
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_RTC_MAX31331_THREAD_STACK_SIZE);
	struct k_sem sem;
#endif
	const struct device *dev;
};

struct rtc_max31331_config {
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec inta_gpios;

#ifdef CONFIG_RTC_MAX31331_TIMESTAMPING
	bool ts_enable;
	bool ts_vbat_enable;
	bool ts_din;
	bool ts_overwrite;
	bool ts_power_supply_switch;
	bool din_polarity;
	bool din_en_io;
#endif
};

int max31331_reg_read(const struct device *dev, uint8_t reg_addr, uint8_t *val, uint8_t length);
int max31331_reg_write(const struct device *dev, uint8_t reg_addr, uint8_t val);
int max31331_reg_write_multiple(const struct device *dev, uint8_t reg_addr, const uint8_t *val,
				uint8_t length);
int max31331_reg_update(const struct device *dev, uint8_t reg_addr, uint8_t mask, uint8_t val);

#ifdef CONFIG_RTC_MAX31331_TIMESTAMPING
int rtc_max31331_get_timestamps(const struct device *dev, struct rtc_time *timeptr, uint8_t index,
				uint8_t *flags);
int rtc_max31331_set_timestamp_callback(const struct device *dev,
					rtc_max31331_timestamp_callback cb, void *user_data);
#endif
