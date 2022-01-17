/*
 * Copyright (c) 2021, Wiifor SAS. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_LR1110_GENERAL_H_
#define ZEPHYR_DRIVERS_LR1110_GENERAL_H_

#include <zephyr/types.h>
#include <drivers/gpio.h>
#include <drivers/lora.h>
#include <device.h>

int __lr1110_configure_pin(const struct device * *dev, const char *controller,
			   gpio_pin_t pin, gpio_flags_t flags);

#define lr1110_configure_pin(_name, _flags)				\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(0, _name##_gpios),		\
		    (__lr1110_configure_pin(&LR1110._name,		\
				DT_INST_GPIO_LABEL(0, _name##_gpios),	\
				DT_INST_GPIO_PIN(0, _name##_gpios),	\
				DT_INST_GPIO_FLAGS(0, _name##_gpios) |	\
						      _flags)),		\
		    (0))

int lr1110_lora_send(const struct device *dev, uint8_t *data,
		     uint32_t data_len);

int lr1110_lora_recv(const struct device *dev, uint8_t *data, uint8_t size,
		     k_timeout_t timeout, int16_t *rssi, int8_t *snr);

int lr1110_lora_config(const struct device *dev,
		       struct lora_modem_config *config);

int lr1110_lora_test_cw(const struct device *dev, uint32_t frequency,
			int8_t tx_power,
			uint16_t duration);

int lr1110_init(const struct device *dev);

void lr1110_sleep();

#endif /* ZEPHYR_DRIVERS_LR1110_GENERAL_H_ */
