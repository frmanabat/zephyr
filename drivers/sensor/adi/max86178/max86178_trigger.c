/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "max86178.h"
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(MAX86178);


#if defined(CONFIG_MAX86178_TRIGGER_OWN_THREAD) || defined(CONFIG_MAX86178_TRIGGER_GLOBAL_THREAD)
static void max86178_thread_cb(const struct device *dev)
{
	const struct max86178_dev_config *cfg = dev->config;
	struct max86178_data *data = dev->data;
	uint8_t status [5];
	int ret;

	/* Clear the status */
	ret = max86178_reg_read(dev, MAX86178_STATUS1, &status[0], 1);
    if (ret) {
        LOG_ERR("Failed to read STATUS1: %d", ret);
        return;
    }

    ret = max86178_reg_read(dev, MAX86178_STATUS2, &status[1], 1);
    if (ret) {
        LOG_ERR("Failed to read STATUS2: %d", ret);
        return;
    }

    ret = max86178_reg_read(dev, MAX86178_STATUS3, &status[2], 1);
    if (ret) {
        LOG_ERR("Failed to read STATUS3: %d", ret);
        return;
    }

    ret = max86178_reg_read(dev, MAX86178_STATUS4, &status[3], 1);
    if (ret) {
        LOG_ERR("Failed to read STATUS4: %d", ret);
        return;
    }

    ret = max86178_reg_read(dev, MAX86178_STATUS5, &status[4], 1);
    if (ret) {
        LOG_ERR("Failed to read STATUS5: %d", ret);
        return;
    }

	ret = gpio_pin_interrupt_configure_dt(&cfg->interrupt_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret) {
		LOG_ERR("Failed to enable interrupt: %d", ret);
		return;
	}
}
#endif

static void max86178_gpio_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	int ret;
	struct max86178_data *data = CONTAINER_OF(cb, struct max86178_data, gpio_cb);
	const struct max86178_dev_config *cfg = data->dev->config;

	ret = gpio_pin_interrupt_configure_dt(&cfg->interrupt_gpio, GPIO_INT_DISABLE);
	if (ret) {
		LOG_ERR("Failed to disable interrupt: %d", ret);
		return;
	}
#ifdef CONFIG_MAX86178_TRIGGER_OWN_THREAD
#ifdef CONFIG_MAX86178_STREAM
	max86178_stream_irq_handler(data->dev);
#endif /* CONFIG_MAX86178_STREAM */
#endif /* CONFIG_MAX86178_TRIGGER_OWN_THREAD */

#if defined(CONFIG_MAX86178_TRIGGER_OWN_THREAD)
	k_sem_give(&data->gpio_sem);
#elif defined(CONFIG_MAX86178_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&data->work);
#endif /* CONFIG_MAX86178_TRIGGER_OWN_THREAD || CONFIG_MAX86178_TRIGGER_GLOBAL_THREAD */
}

int max86178_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
                            sensor_trigger_handler_t handler)
{
    struct max86178_data *data = dev->data;
    const struct max86178_dev_config *config = dev->config;
    uint8_t int_mask;
    uint8_t int_en = 1;
    uint8_t offset_addr = MAX86178_INT1_EN1;
    int ret;

    if (handler == NULL) {
        LOG_INF("Disabling trigger for type %d", trig->type);
        int_en = 0;
    }

    ret = gpio_pin_interrupt_configure_dt(&config->interrupt_gpio, GPIO_INT_DISABLE);
    if (ret) {
        LOG_ERR("Failed to disable interrupt: %d", ret);
        return ret;
    }

    switch ((int)trig->type) {
        case SENSOR_TRIG_FIFO_WATERMARK:
            data->a_full_handler = handler;
            data->a_full_trigger = trig;
            int_mask = MAX86178_INT_EN1_A_FULL_MSK;
            offset_addr = 0;
            break;
        case SENSOR_TRIG_MAX86178_PPG_THRESH1_HILO:
            data->ppg_thresh1_hilo_handler = handler;
            data->ppg_thresh1_hilo_trigger = trig;
            int_mask = MAX86178_INT_EN1_PPG_THRESH1_HILO_MSK;
            offset_addr = 0;
             break;
            break;
        case SENSOR_TRIG_MAX86178_PPG_THRESH2_HILO:
            data->ppg_thresh2_hilo_handler = handler;
            data->ppg_thresh2_hilo_trigger = trig;
            int_mask = MAX86178_INT_EN1_PPG_THRESH2_HILO_MSK;
            offset_addr = 0;
            break;
        case SENSOR_TRIG_MAX86178_EXP_OVF:
            data->exp_ovf_handler = handler;
            data->exp_ovf_trigger = trig;
            int_mask = MAX86178_INT_EN1_EXP_OVF_MSK;
            offset_addr = 0;
            break;
        case SENSOR_TRIG_MAX86178_ALC_OVF:
            data->alc_ovf_handler = handler;
            data->alc_ovf_trigger = trig;
            int_mask = MAX86178_INT_EN1_ALC_OVF_MSK;
            offset_addr = 0;
            break;
        case SENSOR_TRIG_MAX86178_FIFO_DATA_RDY:
            data->fifo_data_rdy_handler = handler;
            data->fifo_data_rdy_trigger = trig;
            int_mask = MAX86178_INT_EN1_FIFO_DATA_RDY_MSK;
            offset_addr = 0;
            break;
        case SENSOR_TRIG_MAX86178_PPG_FRAME_RDY:
            data->ppg_frame_rdy_handler = handler;
            data->ppg_frame_rdy_trigger = trig;
            int_mask = MAX86178_INT_EN1_PPG_FRAME_RDY_MSK;
            offset_addr = 0;
            break;
        case SENSOR_TRIG_MAX86178_LED1_COMPB:
            data->led1_compb_handler = handler;
            data->led1_compb_trigger = trig;
            int_mask = MAX86178_INT_EN2_LED1_COMPB_MSK;
            offset_addr = 1;
            break;
        case SENSOR_TRIG_MAX86178_LED2_COMPB:
            data->led2_compb_handler = handler;
            data->led2_compb_trigger = trig;
            int_mask = MAX86178_INT_EN2_LED2_COMPB_MSK;
            offset_addr = 1;
            break;
        case SENSOR_TRIG_MAX86178_LED3_COMPB:
            data->led3_compb_handler = handler;
            data->led3_compb_trigger = trig;
            int_mask = MAX86178_INT_EN2_LED3_COMPB_MSK;
            offset_addr = 1;
            break;
        case SENSOR_TRIG_MAX86178_LED4_COMPB:
            data->led4_compb_handler = handler;
            data->led4_compb_trigger = trig;
            int_mask = MAX86178_INT_EN2_LED4_COMPB_MSK;
            offset_addr = 1;
            break;
        case SENSOR_TRIG_MAX86178_LED5_COMPB:
            data->led5_compb_handler = handler;
            data->led5_compb_trigger = trig;
            int_mask = MAX86178_INT_EN2_LED5_COMPB_MSK;
            offset_addr = 1;
            break;
        case SENSOR_TRIG_MAX86178_LED6_COMPB:
            data->led6_compb_handler = handler;
            data->led6_compb_trigger = trig;
            int_mask = MAX86178_INT_EN2_LED6_COMPB_MSK;
            offset_addr = 1;
            break;
        case SENSOR_TRIG_MAX86178_INVALID_PPG_CFG:
            data->invalid_ppg_cfg_handler = handler;
            data->invalid_ppg_cfg_trigger = trig;
            int_mask = MAX86178_INT_EN2_INVALID_PPG_CFG_MSK;
            offset_addr = 1;
            break;
        case SENSOR_TRIG_MAX86178_PHASE_LOCK:
            data->phase_lock_handler = handler;
            data->phase_lock_trigger = trig;
            int_mask = MAX86178_INT_EN3_PHASE_LOCK_MSK;
            offset_addr = 2;
            break;
        case SENSOR_TRIG_MAX86178_PHASE_UNLOCK:
            data->phase_unlock_handler = handler;
            data->phase_unlock_trigger = trig;
            int_mask = MAX86178_INT_EN3_PHASE_UNLOCK_MSK;
            offset_addr = 2;
             break;
            break;
        case SENSOR_TRIG_MAX86178_FREQ_LOCK:
            data->freq_lock_handler = handler;
            data->freq_lock_trigger = trig;
            int_mask = MAX86178_INT_EN3_FREQ_LOCK_MSK;
            offset_addr = 2;
            break;
        case SENSOR_TRIG_MAX86178_FREQ_UNLOCK:
            data->freq_unlock_handler = handler;
            data->freq_unlock_trigger = trig;
            int_mask = MAX86178_INT_EN3_FREQ_UNLOCK_MSK;
            offset_addr = 2;
            break;
        case SENSOR_TRIG_MAX86178_ECG_LOFF_NL:
            data->ecg_loff_nl_handler = handler;
            data->ecg_loff_nl_trigger = trig;
            int_mask = MAX86178_INT_EN4_ECG_LOFF_NL_MSK;
            offset_addr = 3;
            break;
        case SENSOR_TRIG_MAX86178_ECG_LOFF_NH:
            data->ecg_loff_nh_handler = handler;
            data->ecg_loff_nh_trigger = trig;
            int_mask = MAX86178_INT_EN4_ECG_LOFF_NH_MSK;
            offset_addr = 3;
            break;
        case SENSOR_TRIG_MAX86178_ECG_LOFF_PL:
            data->ecg_loff_pl_handler = handler;
            data->ecg_loff_pl_trigger = trig;
            int_mask = MAX86178_INT_EN4_ECG_LOFF_PL_MSK;
            offset_addr = 3;
            break;
        case SENSOR_TRIG_MAX86178_ECG_LOFF_PH:
            data->ecg_loff_ph_handler = handler;
            data->ecg_loff_ph_trigger = trig;
            int_mask = MAX86178_INT_EN4_ECG_LOFF_PH_MSK;
            offset_addr = 3;
            break;
        case SENSOR_TRIG_MAX86178_RLD_OOR:
            data->rld_oor_handler = handler;
            data->rld_oor_trigger = trig;
            int_mask = MAX86178_INT_EN4_RLD_OOR_MSK;
            offset_addr = 3;
            break;
        case SENSOR_TRIG_MAX86178_ECG_FAST_REC:
            data->ecg_fast_rec_handler = handler;
            data->ecg_fast_rec_trigger = trig;
            int_mask = MAX86178_INT_EN4_ECG_FAST_REC_MSK;
            offset_addr = 3;
            break;
        case SENSOR_TRIG_MAX86178_ECG_LON:
            data->ecg_lon_handler = handler;
            data->ecg_lon_trigger = trig;
            int_mask = MAX86178_INT_EN4_ECG_LON_MSK;
            offset_addr = 3;
            break;
         case SENSOR_TRIG_MAX86178_BIOZ_LOFF_NL:
            data->bioz_loff_nl_handler = handler;
            data->bioz_loff_nl_trigger = trig;
            int_mask = MAX86178_INT_EN5_BIOZ_LOFF_NL_MSK;
            offset_addr = 4;
            break;
        case SENSOR_TRIG_MAX86178_BIOZ_LOFF_NH:
            data->bioz_loff_nh_handler = handler;
            data->bioz_loff_nh_trigger = trig;
            int_mask = MAX86178_INT_EN5_BIOZ_LOFF_NH_MSK;
            offset_addr = 4;
            break;
        case SENSOR_TRIG_MAX86178_BIOZ_LOFF_PL:
            data->bioz_loff_pl_handler = handler;
            data->bioz_loff_pl_trigger = trig;
            int_mask = MAX86178_INT_EN5_BIOZ_LOFF_PL_MSK;
            offset_addr = 4;
            break;
        case SENSOR_TRIG_MAX86178_BIOZ_LOFF_PH:
            data->bioz_loff_ph_handler = handler;
            data->bioz_loff_ph_trigger = trig;
            int_mask = MAX86178_INT_EN5_BIOZ_LOFF_PH_MSK;
            offset_addr = 4;
            break;
        case SENSOR_TRIG_MAX86178_BIOZ_DRVP_OFF:
            data->bioz_drvp_off_handler = handler;
            data->bioz_drvp_off_trigger = trig;
            int_mask = MAX86178_INT_EN5_BIOZ_DRVP_OFF_MSK;
            offset_addr = 4;
            break;
        case SENSOR_TRIG_MAX86178_BIOZ_UNDR:
            data->bioz_undr_handler = handler;
            data->bioz_undr_trigger = trig;
            int_mask = MAX86178_INT_EN5_BIOZ_UNDR_MSK;
            offset_addr = 4;
            break;
        case SENSOR_TRIG_MAX86178_BIOZ_OVER:
            data->bioz_over_handler = handler;
            data->bioz_over_trigger = trig;
            int_mask = MAX86178_INT_EN5_BIOZ_OVER_MSK;
            offset_addr = 4;
            break;
        case SENSOR_TRIG_MAX86178_BIOZ_LON:
            data->bioz_lon_handler = handler;
            data->bioz_lon_trigger = trig;
            int_mask = MAX86178_INT_EN5_BIOZ_LON_MSK;
            offset_addr = 4;
            break;
        default:
            LOG_ERR("Unsupported trigger type");
            return -ENOTSUP;
    }

    if (config->route_to_int2) {
        offset_addr += MAX86178_INT1_INT2_OFFSET;
    }

    ret = max86178_reg_update(dev, offset_addr, int_mask, int_en);
    if (ret) {
        LOG_ERR("Failed to enable interrupt: %d", ret);
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&config->interrupt_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret) {
        LOG_ERR("Failed to enable interrupt: %d", ret);
        return ret;
    }

    return 0;
}

int max86178_init_interrupt(const struct device *dev)
{
    int ret = 0;
    const struct max86178_dev_config *config = dev->config;
    struct max86178_data *data = dev->data;
    
    if (!gpio_is_ready_dt(&config->interrupt_gpio)) {
        LOG_ERR("GPIO device not ready");
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&config->interrupt_gpio, GPIO_INPUT);
    if (ret) {
        LOG_ERR("Failed to configure GPIO pin");
        return ret;
    }

    gpio_init_callback(&data->gpio_cb, max86178_gpio_callback, BIT(config->interrupt_gpio.pin));

    ret = gpio_add_callback(config->interrupt_gpio.port, &data->gpio_cb);
    if (ret) {
        LOG_ERR("Failed to add GPIO callback");
        return ret;
    }

    data->dev = dev;
#if defined(CONFIG_MAX86178_STREAM)
    ret = max86178_reg_update(dev, config->route_to_int2 ? MAX86178_INT2_EN1 : MAX86178_INT1_EN1, MAX86178_INT_EN1_A_FULL_MSK, 1);
    if (ret) {
        LOG_ERR("Failed to enable FIFO A Full interrupt: %d", ret);
        return ret;
    }
#endif /* CONFIG_MAX86178_STREAM */

#if defined(CONFIG_MAX86178_TRIGGER_OWN_THREAD)
    k_sem_init(&data->gpio_sem, 0, K_SEM_MAX_LIMIT);
    k_thread_create(&data->thread, data->thread_stack, CONFIG_MAX86178_THREAD_STACK_SIZE,
                    (k_thread_entry_t)max86178_thread_cb, data, NULL, NULL,
                    CONFIG_MAX86178_THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&data->thread, dev->name);
#elif defined(CONFIG_MAX86178_TRIGGER_GLOBAL_THREAD)
    data->work.handler = max86178_work_cb;
#endif /* CONFIG_MAX86178_TRIGGER_OWN_THREAD || CONFIG_MAX86178_TRIGGER_GLOBAL_THREAD */
    ret = gpio_pin_interrupt_configure_dt(&config->interrupt_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret) {
        LOG_ERR("Failed to enable GPIO interrupt");
        return ret;
    }
    LOG_INF("MAX86178 interrupt initialized");
    return 0;

}