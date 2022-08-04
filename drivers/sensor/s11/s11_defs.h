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
 * @brief Definitions & constants for Senseair Sunrise CO2 sensor (S11) driver
 * @file s11_defs.h
 * @date 2021-03-11
 * @author Jhonatan Napadow
 *
 */

/**
 * Copyright (c) 2022 Wiifor SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 **/

/*
 * @brief Source code for Senseair Sunrise HVAC (s11) sensor driver
 * adapted for Framework Zephyr
 * @file s11_defs.h
 * @date 2022-08-08
 * @author Nicolas Pelissier
 *
 */

#ifndef S11_DEFS_H_17
#define S11_DEFS_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include <device.h>
#include <zephyr/types.h>

// Default
#define S11_DEF_I2C_ADDR (0x68) ///< Default I2C address

/// @name Error Status register (ERRSTAT)
/// @{
#define S11_ADDR_ERRSTAT_MSB (0x00) ///< Register address
#define S11_ADDR_ERRSTAT_LSB (0x01) ///< Register address
/// @}

/// @name Info register range (INF)
/// @{
#define S11_ADDR_INF_BUF (0x38) ///< First register address
#define S11_ADDR_INF_BUF_LEN (6) ///< Length [bytes]
// Offsets
#define S11_ADDR_INF_FW_REV_MSB (0) ///< Offset - Firmware revision MSB
#define S11_ADDR_INF_FW_REV_LSB (1) ///< Offset - Firmware revision LSB
#define S11_ADDR_INF_ID_MMSB (2) ///< Offset - Device ID MMSB
#define S11_ADDR_INF_ID_MLSB (3) ///< Offset - Device ID MLSB
#define S11_ADDR_INF_ID_LMSB (4) ///< Offset - Device ID LMSB
#define S11_ADDR_INF_ID_LLSB (5) ///< Offset - Device ID LLSB
/// @}

/// @name Measurement data register range (MD)
/// @{
// Reg addr - measurement data array (R)
#define S11_ADDR_MD_BUF (0x06) ///< First register address
#define S11_ADDR_MD_BUF_LEN (16) ///< Length [bytes]
// Offsets
#define S11_ADDR_MD_CO2_FP_MSB (0) ///< Offset - CO2 value filtered & pressure compensated MSB
#define S11_ADDR_MD_CO2_FP_LSB (1) ///< Offset - CO2 value filtered & pressure compensated LSB
#define S11_ADDR_MD_TEMP_MSB (2) ///< Offset - PCB temperature MSB
#define S11_ADDR_MD_TEMP_LSB (3) ///< Offset - PCB temperature LSB
#define S11_ADDR_MD_COUNT (7) ///< Offset - Measurement counts
#define S11_ADDR_MD_CYCLE_TIME_MSB (8) ///< Offset - Measurement cycle time MSB
#define S11_ADDR_MD_CYCLE_TIME_LSB (9) ///< Offset - Measurememnt cycle time LSB
#define S11_ADDR_MD_CO2_P_MSB (10) ///< Offset - CO2 value - pressure compensated
#define S11_ADDR_MD_CO2_P_LSB (11) ///< Offset - CO2 value - pressure compensated
#define S11_ADDR_MD_CO2_F_MSB (12) ///< Offset - CO2 value - filtered MSB
#define S11_ADDR_MD_CO2_F_LSB (13) ///< Offset - CO2 value - filtered LSB
#define S11_ADDR_MD_CO2_MSB (14) ///< Offset - CO2 value MSB
#define S11_ADDR_MD_CO2_LSB (15) ///< Offset - CO2 value LSB
/// @}

/// @name Calibration data register range (CAL)
/// @{
#define S11_ADDR_CAL_BUF (0x81)
#define S11_ADDR_CAL_BUF_LEN (5)
// Offsets
#define S11_ADDR_CAL_STATUS (0)
#define S11_ADDR_CAL_CMD_MSB (1)
#define S11_ADDR_CAL_CMD_LSB (2)
#define S11_ADDR_CAL_TGT_MSB (3)
#define S11_ADDR_CAL_TGT_LSB (4)
/// @}

/// @name CO2 override register (CO2)
/// @{
#define S11_ADDR_CO2_OVERRIDE (0x86) ///< CO2 Override value [ppm]
/// @}

/// @name CO2 override disable command
/// @{
#define S11_CO2_CMD_OVERRIDE_DIS                                                                   \
	(32762) ///< Write this value to CO2 override register to disable override
/// @}

/// @name Device configuration register range
/// EE = EEPROM register, limited no of write cycles
/// @{
#define S11_ADDR_DEV_MEAS_MODE (0x95) ///< Measurement mode register (EE)
#define S11_ADDR_DEV_MEAS_PER_MSB (0x96) ///< Measurement period register MSB (EE)
#define S11_ADDR_DEV_MEAS_PER_LSB (0x97) ///< Measurement period register LSB (EE)
#define S11_ADDR_DEV_NB_SAMP_MSB (0x98) ///< Number of samples register MSB (EE)
#define S11_ADDR_DEV_NB_SAMP_LSB (0x99) ///< Number of samples register LSB (EE)
#define S11_ADDR_ABC_PER_MSB (0x9A) ///< ABC period register MSB (EE)
#define S11_ADDR_ABC_PER_LSB (0x9B) ///< ABC period register LSB (EE)
#define S11_ADDR_ABC_TARGET_MSB (0x9E) ///< ABC target register MSB (EE)
#define S11_ADDR_ABC_TARGET_LSB (0x9F) ///< ABC target register LSB (EE)
#define S11_ADDR_IIR_STAT_PAR (0xA1) ///< Static IIR filter parameter register (EE)
#define S11_ADDR_DEV_METER_CTL (0xA5) ///< Meter control register (EE)
#define S11_ADDR_DEV_ADDR (0xA7) ///< Device address register, I2C/Modbus (EE)
/// @}

/// @name Device reset
/// @{
#define S11_ADDR_RESET (0xA3) ///< Register address
#define S11_RESET (0xFF) ///< Command

/// @}

/// @name Clear error status
/// @{
#define S11_ADDR_CLR_ERR_STATUS (0x9D) ///< Register address
#define S11_CLEAR_ERRSTAT_CMD (0x01) ///< Command

/// @}

/// @name Measurement command register range
/// @{
#define S11_ADDR_MC_BUF (0xC3)
#define S11_ADDR_MC_BUF_LEN (26)
// Offsets
#define S11_ADDR_MC_START (0)
#define S11_ADDR_MC_PRESS_MSB (25)
#define S11_ADDR_MC_PRESS_LSB (26)
/// @}

/// @name Measurement command
/// @{
#define S11_MC_CMD_START_MEAS (0x01) ///< Start measurement command
/// @}

/// @name Error Status Register (ERRSTAT) - Bitmask
/// @{
#define S11_ERRSTAT_MSK_LO_V (0x0100) ///< Low internal regulated voltage
#define S11_ERRSTAT_MSK_ME_TMT (0x0200) ///< Measurement timeout
#define S11_ERRSTAT_MSK_FATAL (0x0001) ///< Fatal error
#define S11_ERRSTAT_MSK_I2C (0x0002) ///< I2C error
#define S11_ERRSTAT_MSK_ALG (0x0004) ///< Algorithm error
#define S11_ERRSTAT_MSK_CAL (0x0008) ///< Calibration error
#define S11_ERRSTAT_MSK_SLF_DIA (0x0010) ///< Self diagnostics error
#define S11_ERRSTAT_MSK_OUT_RNG (0x0020) ///< Out of range
#define S11_ERRSTAT_MSK_MEM (0x0040) ///< Memory error
#define S11_ERRSTAT_MSK_NO_MEAS (0x0080) ///< No measurement completed
/// @}

/// @addtogroup calibration
///  @{
/// @name Calibration Status Register (CAL_STAT) - Bitmask
/// @{
#define S11_CAL_STAT_MSK_FACT_CAL (0x02) ///< Factory calibration completed
#define S11_CAL_STAT_MSK_ABC_CAL (0x04) ///< ABC calibration completed
#define S11_CAL_STAT_MSK_TARGET_CAL (0x08) ///< Target calibration completed
#define S11_CAL_STAT_MSK_BKG_CAL (0x10) ///< Background calibration completed
#define S11_CAL_STAT_MSK_ZERO_CAL (0x20) ///< Calibration status - Zero calibration completed
///@}

/// @name Calibration Command Register (CAL_CMD) - Calibration types
/// @brief Defines which calibration routine to perform
/// @{
#define S11_CAL_CMD_FACT_CAL (0x7C02) ///< Restore factory calibration
#define S11_CAL_CMD_ABC_CAL (0x7C03) ///< Forced ABC calibration
#define S11_CAL_CMD_TARGET_CAL (0x7C05) ///< Target value calibration
#define S11_CAL_CMD_BKG_CAL (0x7C06) ///< Background calibration [400ppm]
#define S11_CAL_CMD_ZERO_CAL (0x7C07) ///< Restore factory calibration [0ppm]
/// @} */
///  @} */

///  @name Meter control Register (MET_CTL)(EE) - Bitmask
///  @{
#define S11_MET_CTL_MSK_NRDY_EN (0x01) ///< Meter controls - Enable nReady pin
#define S11_MET_CTL_MSK_ABC_EN (0x02) ///< Meter controls - Enable ABC calibration
#define S11_MET_CTL_MSK_SIIR_EN (0x04) ///< Meter controls - Enable static IIR filter
#define S11_MET_CTL_MSK_DIIR_EN (0x08) ///< Meter controls - Enable dynamic IIR filter
#define S11_MET_CTL_MSK_PCOMP_EN (0x10) ///< Meter controls - Enable pressure compensation
///  @}

/// @name Measurement Mode Register (MM_STATE) - Values
/// @{
#define S11_MM_STATE_CONT_MEAS (0) ///< Continuous measurements
#define S11_MM_STATE_SGL_MEAS (1) ///< Single measurement
/// @}

/// @name Driver Error Codes
/// @{
#define S11_E_OK (0x00) ///< Function executed with no error
#define S11_E_NULL_PTR (0x01) ///< One or more function pointers are invalid
#define S11_E_I2C (0x02) ///< I2C communication error
#define S11_E_VAL_OUT_OF_LIMIT (0x03) ///< One or more input parameters are outside valid range
#define S11_E_CALIBRATION (0x04) ///< Calibration error
/// @}

// Shared I2C buffer length
#define S11_BUF_LEN (32)

/// @name Time constants
/// @{
/* Use 107 ms for 006-0-0008 article (Sunrise HVAC)
 * & 25 ms for 006-0-0002 & 006-0-0007 article (Sunrise)
 */
#define S11_EE_WR_DELAY_MS (107) ///< Wait time between EEPROM writes
#define S11_RESTART_DELAY_MS (35) ///< Wait time for sensor to boot
/// @}

/// @name Function pointer typedefs
/// @{

/// Type definition for read/write function pointer
typedef int8_t (*s11_rw_fptr_t)(const struct device *i2c_master, uint16_t i2c_slave,
				uint8_t reg_addr, uint8_t *data, uint8_t len);

/// Type definition for delay function pointer
typedef int8_t (*s11_delay_fptr_t)(uint32_t period);
///@}

/**
* @brief This structure contains measurement variables and data
* @ingroup measurement
* @details
* R = readable variable\n
* W = writeable variable\n
* P = private variable, internal use, do not read/write
*/
struct s11_meas_res {
	/// CO2 value filtered and pressure compensated [ppm] (R)
	uint16_t co2_fp;
	/// Chip temperature [0.01 deg C] (R)
	int16_t temp;
	/// Measurement count (R)
	uint8_t meas_count;
	/// Measurement cycle time [2 seconds] (R)
	uint16_t meas_cycle_time;
	/// CO2 value unfiltered & pressure compensated [ppm] (R)
	uint16_t co2_p;
	/// CO2 value filtered [ppm] (R)
	uint16_t co2_f;
	/// CO2 value unfiltered [ppm] (R)
	uint16_t co2;
	/// First measurement without state data completed (P)
	bool first_meas_completed;
	/// State data measurement read flag (P)
	bool state_data_exists;
	/// Measurement data buffer (P)
	uint8_t data[S11_ADDR_MD_BUF_LEN]; // #todo remove
	/// Measurement command buffer (P)
	uint8_t cmd[S11_ADDR_MC_BUF_LEN];
};

/**
* @brief This structure contains sensor device information
* @ingroup core
* @details
* R = readable variable\n
* W = writeable variable\n
* P = private variable, internal use, do not read/write
*/
struct s11_info {
	/// Firmware revision, MSB=main, LSB=sub (R)
	uint16_t firmware_rev;
	/// Sensor ID (R)
	uint32_t sensor_id;
};

/**
* @brief Structure containing calibration data
* @ingroup calibration
* @details
* R = readable variable\n
* W = writeable variable\n
* P = private variable, internal use, do not read/write
*/
struct s11_cal {
	/// Calibration status (R)
	uint8_t cal_status;
	/// Calibration command (W)
	uint16_t cal_cmd;
	/// Calibration target [ppm] (W)
	uint16_t cal_target;
};

/**
* @brief Structure containing device settings
* @ingroup settings
* @details
* EE = eeprom variable, restricted no of writes
* R = readable variable\n
* W = writeable variable\n
* P = private variable, internal use, do not read/write
*/
struct s11_dev_sett {
	/// Measurement mode (EE) (R/W)
	uint8_t meas_mode;
	/// Measurement Period [seconds] (EE) (R/W)
	uint16_t meas_period;
	/// Number of samples (EE) (R/W
	uint16_t meas_nb_samples;
	/// ABC period [hours] (EE) (R/W
	uint16_t abc_period;
	/// ABC target [ppm co2] (EE) (R/W
	uint16_t abc_target;
	/// Static IIR filter parameter (EE) (R/W)
	uint8_t iir_static_parameter;
	/// Meter control (EE) (R/W
	uint8_t meter_control;
	/// MB/I2c address (EE) (R/W
	uint8_t address;
	/// Counter for write operations from this API (R)
	uint32_t write_count;
};

/**
* @brief Main structure containing device state information and settings
* @ingroup core
* @details
* R = readable variable\n
* W = writeable variable\n
* P = private variable, internal use, do not read/write
*/
struct s11_dev {
	/// I2C master to use for communication (W)
	const struct device *i2c_master;
	/// I2C slave address to use for communication (W)
	uint16_t i2c_slave_addr;
	/// Error status (R)
	uint16_t error_status;
	/// Measurement results
	struct s11_meas_res meas_res;
	/// Sensor information
	struct s11_info info;
	/// Calibration settings
	struct s11_cal cal_sett;
	/// CO2 value override (W)
	uint16_t co2_val_override;
	/// ABC Time [h] (W)
	int16_t abc_time;
	/// Barometric air pressure value [0.1 hPA] (W)
	int16_t air_pressure;
	/// Device settings
	struct s11_dev_sett dev_sett;
	/// I2C read function pointer (W)
	s11_rw_fptr_t read;
	/// I2C write function pointer (W)
	s11_rw_fptr_t write;
	/// I2C delay function pointer (W)
	s11_delay_fptr_t delay;
	/// Shared data buffer for (P)
	uint8_t data[S11_BUF_LEN];
};

#endif // S11_DEFS_H_