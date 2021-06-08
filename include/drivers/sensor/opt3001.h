/*
 * Copyright (c) 2021 Wiifor
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file drivers/sensor/opt3001.h
 *
 * @brief Public APIs for the maxim battery gauge driver.
 *
 * Following types require to be visible by user application and can't be
 * defined in opt3001 driver header.
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_OPT3001_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_OPT3001_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <drivers/sensor.h>

/**
 * @brief OPT3001 attribute types.
 */
enum opt3001_sensor_attribute {
	SENSOR_ATTR_OPT3001_INIT,
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_OPT3001_H_ */