/*
 * Copyright (c) 2021 Wiifor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file drivers/sensor/bmi160.h
 *
 * @brief Public APIs for the Bosch inertial measurement unit driver.
 *
 * Following types require to be visible by user application and can't be
 * defined in bmi160 driver header.
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_BMI160_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_BMI160_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <drivers/sensor.h>

/**
 * @brief BMI160 attribute types.
 */
enum bmi610_sensor_attribute {
	SENSOR_ATTR_BMI160_PRIV_START = SENSOR_ATTR_PRIV_START,
	/**
	 * Manage power mode. This will be used by the internal chip's algorithms
	 * to enable, disable or put in low power certain axis, or all of them.
	 */
	SENSOR_ATTR_BMI160_PMU_ACTIVE,
	SENSOR_ATTR_BMI160_PMU_LOW_POWER,
	SENSOR_ATTR_BMI160_PMU_OFF,
	SENSOR_ATTR_BMI160_PRIV_END,
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_BMI160_H_ */
