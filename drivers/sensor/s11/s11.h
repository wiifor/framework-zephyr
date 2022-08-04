/**
 * BSD 3-Clause License
 * 
 * Copyright (c) 2021, Jhonatan Napadow
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/

/**
 * @brief Function headers for Senseair Sunrise CO2 sensor (S11) driver
 * @file s11.h
 * @date 2021-03-11
 * @author Jhonatan Napadow
 */

/** @defgroup core Driver core
 * API functions for interacting with the driver
 * 
 * Setup sensor:
 * - Instantiate a s11_dev struct
 * - Add callbacks to s11_dev
 * - Write I2C address to s11_dev.i2c_address
 * - Call s11_init()
 * 
 * 
 * The driver is platform-agnostic so the user must adapt it by implementing read/write/delay wrappers
 * that are platform-specific.
 * 
 * The following callbacks must be implemented:
 * - Register a I2C read callback, s11_rw_fptr_t to s11_dev.read
 * - Register a I2C write callback, s11_rw_fptr_t to s11_dev.read
 * - Register a delay callback, s11_delay_fptr_t to s11_dev.delay
 *
 *
 * **Read callback**
 * A platform-specific read callback must be implemented to handle the I2C-communication
 * The callback must have the same format as s11_rw_fptr_t
 *
 *
 * The read callback must implement the follwoing I2C sequence: 
 * - START + Slave address + R/W=0
 * - STOP
 * 
 * - START + Slave address + R/W=0
 * - Write: Register address
 * - STOP
 * 
 * - START + Slave address + R/W=1
 * - Read: Data (n)
 * - Read: Data (n+1)
 * - ...
 * - Read: Data (n+len)
 * - STOP
 *  
 * **Write callback**
 * The write callback must implement the follwoing I2C sequence:
 * The read callback must implement the follwoing I2C sequence:
 * - START + Slave address + R/W=0
 * - STOP
 * 
 * - START + Slave address + R/W=0
 * - Write: Register address 
 * - Write: Data (n)
 * - Write: Data (n+1)
 * - ...
 * - Write: Data (n+len)
 * - STOP 
 */

/** @defgroup calibration Calibration
 * API for (re-)calibration of sensor
 * 
 * Usage:
 * - Select calibration method by setting bits in s11_cal.cal_cmd register
 * - Set calibration target value in ppm in s11_cal.cal_target register (if performing a target calibration)
 * - Call s11_calibrate() to perform calibration
 * - Call s11_get_cal_status() to retrieve calibration status
 * - Read calibration results from s11_cal.cal_status register
 */

/** @defgroup measurement Measurement
 * API for measuring CO2
 * 
 * Single measurement mode:
 * - Call s11_start_meas()
 * - Wait for nREADY pin to go hi
 * - Call s11_get_state_data()
 * - Power off/disable sensor (to save power)
 * - Wait
 * - Power on/enable sensor
 * - Repeat
 *
 * Continuous measurement mode:
 * - Call s11_get_meas_data()
 * - Read results from s11_dev.s11_meas_res
 * - Repeat
 */

/** @defgroup settings Settings
 * API for changing sensors internal settings
 *
 * Writing settings:
 * - Set registers in s11_dev.s11_dev_sett to desired values
 * - Call s11_set_dev_settings()
 *  
 * Reading settings:
 * - Call s11_get_dev_settings()
 * - Read current settings from register s11_dev.s11_dev_sett
 */

#ifndef S11_H_
#define S11_H_

#include "s11_defs.h"

// C++
#ifdef __cplusplus
extern "C" {
#endif

// Function prototype declarations

/**
 * @brief Initialize the device,
 * reads configuration and device id from the sensor.
 * @param[in,out] dev : Instance of S11 structure
 * @return Execution status, defined in s11_defs.h, S11_E_...
 * @ingroup core
 */
int8_t s11_init(struct s11_dev *dev);

/**
 * @brief Reads error status from the sensor.
 * Stores it in the device cluster.
 * @param[in,out] dev : Instance of S11 structure
 * @ingroup core
 */
int8_t s11_get_status(struct s11_dev *dev);

/**
 * @brief Reads firmware rev and Sensor ID from the sensor.
 * Stores it in the device cluster.
 * @param[in,out] dev : Instance of S11 structure
 */
int8_t s11_get_sensor_info(struct s11_dev *dev);

/** 
 * @brief Read the sensors calibration status
 * @param[in,out] dev : Instance of S11 structure
 * @ingroup calibration
 */
int8_t s11_get_cal_status(struct s11_dev *dev);

/** 
 * @brief Perform the sensors built-in calibration routine
 * and retrieve calibration status afterwards
 * @param[in,out] dev : Instance of S11 structure
 * @ingroup calibration
 */
int8_t s11_calibrate(struct s11_dev *dev);

/**
 *  @brief Start a measurement cycle,
 *  if applicable also write ABC+IIR filter data
 *  @param[in,out] dev : Instance of S11 structure
  * @ingroup measurement
 */
int8_t s11_start_meas(struct s11_dev *dev);

/**
 * @brief Read back measurement data,
 * @param[in,out] dev : Instance of S11 structure
 * @ingroup measurement
*/
int8_t s11_get_meas_data(struct s11_dev *dev);

/**
 * @brief Read back ABC+IIR data,
 * @param[in,out] dev : Instance of S11 structure
 * @ingroup measurement
*/
int8_t s11_get_state_data(struct s11_dev *dev);

/** 
 * @brief Read sensor settings from EEPROM
 * - Read settings from sensor to s11_dev_sett struct
 * 
 * @param[in,out] dev : Instance of S11 structure
 * @ingroup settings
 */
int8_t s11_get_dev_settings(struct s11_dev *dev);

/** 
 * @brief Apply sensor settings to EEPROM and restart sensor
 * - Applies settings in s11_dev_sett struct to sensor
 * - Performs a limit check of parameters before any write
 * - Performs a read before write and only writes if needed as to save EEPROM life
 * - Function is blocking during restart
 *
 * @param[in,out] dev : Instance of S11 structure
 * @ingroup settings
 */
int8_t s11_set_dev_settings(struct s11_dev *dev);

/** 
 * @brief Restart sensor
 * @param[in,out] dev : Instance of S11 structure
 * @ingroup core
 */
int8_t s11_restart(struct s11_dev *dev);

#ifdef __cplusplus
}
#endif

#endif // S11_H_