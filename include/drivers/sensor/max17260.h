/*
 * Copyright (c) 2021 Wiifor
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file drivers/sensor/max17260.h
 *
 * @brief Public APIs for the maxim battery gauge driver.
 *
 * Following types require to be visible by user application and can't be
 * defined in max17260 driver header.
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_MAX17260_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_MAX17260_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <drivers/sensor.h>

/**
 * @brief MAX17260 attribute types.
 */
enum max17260_sensor_attribute {
	SENSOR_ATTR_MAX17260_PRIV_START = SENSOR_ATTR_PRIV_START,
	SENSOR_ATTR_MAX17260_HIBERNATE,
	SENSOR_ATTR_MAX17260_PRIV_END,
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_MAX17260_H_ */