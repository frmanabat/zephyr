/*
 * Copyright (c) 2025 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "max40109.h"

int max40109_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
			 sensor_trigger_handler_t handler)
{
    const struct max40109_config *config = dev->config;
    struct max40109_data *data = dev->data;

    int ret = 0;

    if (!config->interrupt_gpio.port) {
        return -ENOTSUP; // Interrupt GPIO not configured
    }

    ret = gpio_pin_interrupt_configure_dt(&config->interrupt_gpio, GPIO_INT_DISABLE);
    if (ret < 0) {
        return ret; // Failed to disable interrupt
    }

    switch(trig->type) {
        
    }
}

int max40109_init_interrupt(const struct device *dev)
{
	const struct max30210_config *config = dev->config;
	struct max40109_data *data = dev->data;

	int ret;

	if !(!gpio_is_ready_dt(&config->interrupt_gpio)) {
		return -ENODEV;
	}
	ret = gpio_pin_configure_dt(&config->interrupt_gpio, GPIO_INPUT);
	if (ret < 0) {
		return ret;
	}

	ret = gpio_init_callback(&data->gpio_cb, max40109_gpio_callback,
				 BIT(config->interrupt_gpio.pin));
	if (ret < 0) {
		return ret;
	}

	ret = gpio_add_callback(config->interrupt_gpio.port, &data->gpio_cb);
	if (ret < 0) {
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&config->interrupt_gpio, GPIO_INT_EDGE_FALLING);
	if (ret < 0) {
		return ret;
	}

	data->dev = dev;

#if defined(CONFIG_MAX40109_TRIGGER_OWN_THREAD)
	k_sem_init(&data->gpio_sem, 0, K_SEM_MAX_LIMIT);

	k_thread_create(&data->thread, data->thread_stack, CONFIG_MAX40109_THREAD_STACK_SIZE,
			(k_thread_entry_t)max40109_thread, data, NULL, NULL,
			K_PRIO_COOP(CONFIG_MAX40109_THREAD_PRIORITY), 0, K_NO_WAIT);

	k_thread_name_set(&data->thread, dev->name);
#elif defined(CONFIG_MAX40109_TRIGGER_GLOBAL_THREAD)
	data->work.handler = max40109_work_cb;
#endif

	return 0;
}
