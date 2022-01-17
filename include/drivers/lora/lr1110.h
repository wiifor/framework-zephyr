/*
 * Copyright (c) 2021 Wiifor SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Extended public API for Semtech LR1110 transceiver
 *
 * Some capabilities and operational requirements for this sensor
 * cannot be expressed within the lora driver abstraction.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_LORA_LR1110_H_
#define ZEPHYR_INCLUDE_DRIVERS_LORA_LR1110_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <device.h>
#include <drivers/sensor.h>

#define LR1110_UID_LENGTH ( 8 )
#define LR1110_JOIN_EUI_LENGTH ( 8 )

typedef uint8_t lr1110_uid_t[LR1110_UID_LENGTH];
typedef uint8_t lr1110_join_eui_t[LR1110_JOIN_EUI_LENGTH];


/**
 * @brief Fetch LR1110 uid.
 *
 * @param dev Pointer to the sensor device
 *
 * @param uid Pointer to where the returned information should be stored
 */
void lr1110_read_uid(const struct device *dev,
			   lr1110_uid_t uid);

/**
 * @brief Fetch LR1110 join eui.
 *
 * @param dev Pointer to the sensor device
 *
 * @param uid Pointer to where the returned information should be stored
 */
void lr1110_read_join_eui(const struct device *dev, lr1110_join_eui_t join_eui);

/**
 * @brief Activates the sleep mode for the LR1110.
 */
void lr1110_sw_sleep();

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_LORA_LR1110_H_ */
