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

/*
 * @brief Source code for Sensair Sunrise CO2 sensor (S11) driver
 * @file s11.c
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
 * @file s11.c
 * @date 2022-08-08
 * @author Nicolas Pelissier
 *
 */

#define DT_DRV_COMPAT senseair_s11

#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include <logging/log.h>

#include "s11.h"

LOG_MODULE_REGISTER(s11, CONFIG_SENSOR_LOG_LEVEL);

/*******************************************************************************/
/*                        Private function declarations                        */
/*******************************************************************************/
int8_t s11_null_ptr_check(struct s11_dev *dev);
uint16_t s11_u8_to_u16(uint8_t msb, uint8_t lsb);
uint8_t s11_u16_to_msb(uint16_t val);
uint8_t s11_u16_to_lsb(uint16_t val);
int8_t s11_read(struct s11_dev *dev, uint8_t reg_addr, uint8_t *data, uint8_t len);
int8_t s11_write(struct s11_dev *dev, uint8_t reg_addr, uint8_t *data, uint8_t len);
int8_t s11_delay(struct s11_dev *dev, uint32_t period);

/*******************************************************************************/
/*                        Private function definitions                         */
/*******************************************************************************/

/**
 * @brief Private helper function that checks that all pointers in device
 * structure are non-null.
 **/
int8_t s11_null_ptr_check(struct s11_dev *dev)
{
	int8_t res;
	if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay == NULL)) {
		res = S11_E_NULL_PTR;
	} else {
		res = S11_E_OK;
	}
	return res;
}

/**
 * @brief Private helper function that concatenates two
 * uint8_t to uint16_t
 **/
uint16_t s11_u8_to_u16(uint8_t msb, uint8_t lsb)
{
	return ((uint16_t)(msb) << 8) | (uint16_t)lsb;
}

/**
 * @brief Private helper function that concatenates two
 * uint8_t to uint16_t
 **/
uint32_t s11_u8_to_u32(uint8_t mmsb, uint8_t mlsb, uint8_t lmsb, uint8_t llsb)
{
	return (((uint32_t)mmsb << 24) | (uint32_t)mlsb << 16 | (uint32_t)lmsb << 8 |
		(uint32_t)llsb);
}

/**
 * @brief Private helper function that extract msb from u16
 **/
uint8_t s11_u16_to_msb(uint16_t val)
{
	return (uint8_t)((val >> 8) & 0x00FF);
}

/**
 * @brief Private helper function that extract lsb from u16
 **/
uint8_t s11_u16_to_lsb(uint16_t val)
{
	return (uint8_t)(val & 0x00FF);
}

/**
 * @brief Private helper function for reading I2C data
 **/
int8_t s11_read(struct s11_dev *dev, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
	int8_t res;
	res = s11_null_ptr_check(dev);
	if (res == S11_E_OK) {
		// Call the sensors read method
		res = dev->read(dev->i2c_master, dev->i2c_slave_addr, reg_addr, data, len);
	}
	return res;
}

/**
 * @brief Private helper function for writing I2C data
 **/
int8_t s11_write(struct s11_dev *dev, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
	int8_t res;
	res = s11_null_ptr_check(dev);
	if (res == S11_E_OK) {
		// Write Byte by Byte with a chip restart beetween each Byte
		// to write values properly
		for (uint8_t i = 0; i < len; i++) {
			// Call the sensors write method
			res = dev->write(dev->i2c_master, dev->i2c_slave_addr, reg_addr + i,
					 data + i, 1);
			s11_delay(dev, S11_EE_WR_DELAY_MS);
			s11_restart(dev);
		}
	}
	return res;
}

/**
 * @brief Private helper function for writing I2C data
 **/
int8_t s11_delay(struct s11_dev *dev, uint32_t period)
{
	int8_t res;
	res = s11_null_ptr_check(dev);
	if (res == S11_E_OK) {
		// Call the sensors delay method
		res = dev->delay(period);
	}
	return res;
}

/*******************************************************************************/
/*                        Public function definitions                          */
/*******************************************************************************/

int8_t s11_init(struct s11_dev *dev)
{
	int8_t res;
	// Check valid pointer
	res = s11_null_ptr_check(dev);
	if (res == S11_E_OK) {
		dev->meas_res.first_meas_completed = false;
		dev->meas_res.state_data_exists = false;
		// Read dev settings from sensor
		res = s11_get_dev_settings(dev);
	}

	return res;
}

int8_t s11_start_meas(struct s11_dev *dev)
{
	int8_t res;
	// Check valid pointer
	res = s11_null_ptr_check(dev);

	if (res == S11_E_OK) {
		if (dev->meas_res.state_data_exists) {
			// Write start meas cmd + iir, abc, press parameters
			dev->meas_res.cmd[S11_ADDR_MC_START] = S11_MC_CMD_START_MEAS;
			dev->meas_res.cmd[S11_ADDR_MC_PRESS_MSB] =
				s11_u16_to_msb(dev->air_pressure);
			dev->meas_res.cmd[S11_ADDR_MC_PRESS_LSB] =
				s11_u16_to_lsb(dev->air_pressure);
			res = s11_write(dev, S11_ADDR_MC_BUF, dev->meas_res.cmd,
					S11_ADDR_MC_BUF_LEN);
		} else {
			// Write start meas cmd (no state data)
			dev->meas_res.cmd[S11_ADDR_MC_START] = S11_MC_CMD_START_MEAS;
			res = s11_write(dev, S11_ADDR_MC_BUF + S11_ADDR_MC_START, dev->meas_res.cmd,
					1);
			if (res == S11_E_OK) {
				dev->meas_res.first_meas_completed = true;
			}
		}
	}
	return res;
}

int8_t s11_get_state_data(struct s11_dev *dev)
{
	int8_t res;
	// Check valid pointer
	res = s11_null_ptr_check(dev);

	if (res == S11_E_OK) {
		// Read ABC+IIR data
		res = s11_read(dev, S11_ADDR_MC_BUF, dev->meas_res.cmd, S11_ADDR_MD_BUF_LEN);
		if ((res == S11_E_OK) & (dev->meas_res.first_meas_completed)) {
			// set flag that state data exists
			dev->meas_res.state_data_exists = true;
		}
	}
	return res;
}

int8_t s11_restart(struct s11_dev *dev)
{
	int8_t res;
	// Check valid pointer
	res = s11_null_ptr_check(dev);

	if (res == S11_E_OK) {
		// Read dev settings from sensor
		dev->data[0] = S11_RESET;
		/* Original s11_restart use s11_write function.
		 * To avoid I2C issues, s11_restart is used in s11_write function.
		 * The 'write' callback is used directly to avoid loop between s11_write & s11_restart
		 */
		dev->write(dev->i2c_master, dev->i2c_slave_addr, S11_ADDR_RESET, dev->data, 1);
		s11_delay(dev, S11_RESTART_DELAY_MS);
	}
	return res;
}

int8_t s11_get_status(struct s11_dev *dev)
{
	int8_t res;
	// Check valid pointer
	res = s11_null_ptr_check(dev);

	if (res == S11_E_OK) {
		// Read dev settings from sensor
		res = s11_read(dev, S11_ADDR_ERRSTAT_MSB, dev->data, 2);
		if (res == S11_E_OK) {
			dev->error_status = s11_u8_to_u16(dev->data[0], dev->data[1]);
		}
	}
	return res;
}

int8_t s11_get_sensor_info(struct s11_dev *dev)
{
	int8_t res;
	// Check valid pointer
	res = s11_null_ptr_check(dev);

	if (res == S11_E_OK) {
		// Read sensor info registers
		res = s11_read(dev, S11_ADDR_INF_BUF, dev->data, S11_ADDR_INF_BUF_LEN);
		if (res == S11_E_OK) {
			dev->info.sensor_id = s11_u8_to_u32(dev->data[S11_ADDR_INF_ID_MMSB],
							    dev->data[S11_ADDR_INF_ID_MLSB],
							    dev->data[S11_ADDR_INF_ID_LMSB],
							    dev->data[S11_ADDR_INF_ID_LLSB]);
			dev->info.firmware_rev = s11_u8_to_u16(dev->data[S11_ADDR_INF_FW_REV_MSB],
							       dev->data[S11_ADDR_INF_FW_REV_LSB]);
		}
	}
	return res;
}

int8_t s11_get_cal_status(struct s11_dev *dev)
{
	int8_t res;
	// Check valid pointer
	res = s11_null_ptr_check(dev);
	if (res != S11_E_OK) {
		return res;
	}

	// Read calibration status
	res = s11_read(dev, S11_ADDR_CAL_BUF + S11_ADDR_CAL_STATUS, dev->data, 1);
	dev->cal_sett.cal_status = dev->data[0];

	return res;
}

int8_t s11_calibrate(struct s11_dev *dev)
{
	int8_t res;
	// Check valid pointer
	res = s11_null_ptr_check(dev);
	if (res != S11_E_OK) {
		return res;
	}

	// Check calibration command is valid
	if ((dev->cal_sett.cal_cmd < S11_CAL_CMD_FACT_CAL) |
	    (dev->cal_sett.cal_cmd > S11_CAL_CMD_ZERO_CAL)) {
		return S11_E_VAL_OUT_OF_LIMIT;
	}

	// Send calibration command (and clear calibration status)
	dev->data[S11_ADDR_CAL_STATUS] = 0x00;
	dev->data[S11_ADDR_CAL_CMD_MSB] = s11_u16_to_msb(dev->cal_sett.cal_cmd);
	dev->data[S11_ADDR_CAL_CMD_LSB] = s11_u16_to_lsb(dev->cal_sett.cal_cmd);
	dev->data[S11_ADDR_CAL_TGT_MSB] = s11_u16_to_msb(dev->cal_sett.cal_target);
	dev->data[S11_ADDR_CAL_TGT_LSB] = s11_u16_to_lsb(dev->cal_sett.cal_target);
	res = s11_write(dev, S11_ADDR_CAL_BUF, dev->data, S11_ADDR_CAL_BUF_LEN);
	if (res != S11_E_OK) {
		return res;
	}

	return res;
}

int8_t s11_get_meas_data(struct s11_dev *dev)
{
	int8_t res;
	// Check valid pointer
	res = s11_null_ptr_check(dev);

	if (res == S11_E_OK) {
		// Read sensor measurement data registers
		res = s11_read(dev, S11_ADDR_MD_BUF, dev->meas_res.data, S11_ADDR_MD_BUF_LEN);
		if (res == S11_E_OK) {
			dev->meas_res.co2_fp =
				s11_u8_to_u16(dev->meas_res.data[S11_ADDR_MD_CO2_FP_MSB],
					      dev->meas_res.data[S11_ADDR_MD_CO2_FP_LSB]);
			dev->meas_res.temp =
				s11_u8_to_u16(dev->meas_res.data[S11_ADDR_MD_TEMP_MSB],
					      dev->meas_res.data[S11_ADDR_MD_TEMP_LSB]);
			dev->meas_res.meas_count = dev->meas_res.data[S11_ADDR_MD_COUNT];
			dev->meas_res.meas_cycle_time =
				s11_u8_to_u16(dev->meas_res.data[S11_ADDR_MD_CYCLE_TIME_MSB],
					      dev->meas_res.data[S11_ADDR_MD_CYCLE_TIME_LSB]);
			dev->meas_res.co2_p =
				s11_u8_to_u16(dev->meas_res.data[S11_ADDR_MD_CO2_P_MSB],
					      dev->meas_res.data[S11_ADDR_MD_CO2_P_LSB]);
			dev->meas_res.co2_f =
				s11_u8_to_u16(dev->meas_res.data[S11_ADDR_MD_CO2_F_MSB],
					      dev->meas_res.data[S11_ADDR_MD_CO2_F_LSB]);
			dev->meas_res.co2 = s11_u8_to_u16(dev->meas_res.data[S11_ADDR_MD_CO2_MSB],
							  dev->meas_res.data[S11_ADDR_MD_CO2_LSB]);
		}
	}
	return res;
}

int8_t s11_get_dev_settings(struct s11_dev *dev)
{
	int8_t res;
	// Check valid pointer
	res = s11_null_ptr_check(dev);
	if (res != S11_E_OK) {
		return res;
	}

	// Read info registers
	res = s11_read(dev, S11_ADDR_DEV_MEAS_MODE, dev->data, 1);
	if (res != S11_E_OK) {
		return res;
	}
	dev->dev_sett.meas_mode = dev->data[0];

	// Read measurement period
	res = s11_read(dev, S11_ADDR_DEV_MEAS_PER_MSB, dev->data, 2);
	if (res != S11_E_OK) {
		return res;
	}
	dev->dev_sett.meas_period = s11_u8_to_u16(dev->data[0], dev->data[1]);

	// Read number of samples
	res = s11_read(dev, S11_ADDR_DEV_NB_SAMP_MSB, dev->data, 2);
	if (res != S11_E_OK) {
		return res;
	}
	dev->dev_sett.meas_nb_samples = s11_u8_to_u16(dev->data[0], dev->data[1]);

	// Read ABC period
	res = s11_read(dev, S11_ADDR_ABC_PER_MSB, dev->data, 2);
	if (res != S11_E_OK) {
		return res;
	}
	dev->dev_sett.abc_period = s11_u8_to_u16(dev->data[0], dev->data[1]);

	// Read ABC target
	res = s11_read(dev, S11_ADDR_ABC_TARGET_MSB, dev->data, 2);
	if (res != S11_E_OK) {
		return res;
	}
	dev->dev_sett.abc_target = s11_u8_to_u16(dev->data[0], dev->data[1]);

	// Read IIR static parameter
	res = s11_read(dev, S11_ADDR_IIR_STAT_PAR, dev->data, 1);
	if (res != S11_E_OK) {
		return res;
	}
	dev->dev_sett.iir_static_parameter = dev->data[0];

	// Read meter control
	res = s11_read(dev, S11_ADDR_DEV_METER_CTL, dev->data, 1);
	if (res != S11_E_OK) {
		return res;
	}
	dev->dev_sett.meter_control = dev->data[0];

	// Read IIR static parameter
	res = s11_read(dev, S11_ADDR_DEV_ADDR, dev->data, 1);
	if (res != S11_E_OK) {
		return res;
	}
	dev->dev_sett.address = dev->data[0];
	return res;
}

int8_t s11_set_dev_settings(struct s11_dev *dev)
{
	int8_t res;
	// Check valid pointer
	res = s11_null_ptr_check(dev);
	if (res != S11_E_OK)
		return res;

	// Limit check for all variables before write
	if ((dev->dev_sett.meas_mode < 0) || (dev->dev_sett.meas_mode > 1)) {
		return S11_E_VAL_OUT_OF_LIMIT;
	}
	if ((dev->dev_sett.meas_period < 2) || (dev->dev_sett.meas_period > 65534)) {
		return S11_E_VAL_OUT_OF_LIMIT;
	}
	if ((dev->dev_sett.meas_nb_samples < 1) || (dev->dev_sett.meas_nb_samples > 1024)) {
		return S11_E_VAL_OUT_OF_LIMIT;
	}
	if ((dev->dev_sett.abc_period < 1) || (dev->dev_sett.abc_period > 65534)) {
		return S11_E_VAL_OUT_OF_LIMIT;
	}
	if ((dev->dev_sett.abc_target < 0) || (dev->dev_sett.abc_target > 65534)) {
		return S11_E_VAL_OUT_OF_LIMIT;
	}
	if ((dev->dev_sett.iir_static_parameter < 2) || (dev->dev_sett.iir_static_parameter > 10)) {
		return S11_E_VAL_OUT_OF_LIMIT;
	}
	/* The following check is disabled because it does not make sense
	 * with desired configuration.
	 */
	/*if ((dev->dev_sett.meter_control < 0) ||
	    (dev->dev_sett.meter_control > 255)) { //31= 0b00011111 // todo, wrong limits/values
		return S11_E_VAL_OUT_OF_LIMIT;
	}*/
	if ((dev->dev_sett.address < 1) || (dev->dev_sett.address > 127)) {
		return S11_E_VAL_OUT_OF_LIMIT;
	}

	// Measurement mode
	if (res == S11_E_OK) {
		// Read current value
		res = s11_read(dev, S11_ADDR_DEV_MEAS_MODE, dev->data, 1);
		if (res == S11_E_OK) {
			// Check if update needed
			if (dev->dev_sett.meas_mode != dev->data[0]) {
				dev->data[0] = dev->dev_sett.meas_mode;
				// Write new value
				res = s11_write(dev, S11_ADDR_DEV_MEAS_MODE, dev->data, 1);
				dev->dev_sett.write_count++;
			}
		}
	}

	// Measurement period
	if (res == S11_E_OK) {
		// Read current value
		res = s11_read(dev, S11_ADDR_DEV_MEAS_PER_MSB, dev->data, 2);
		if (res == S11_E_OK) {
			// Check if update needed
			if (dev->dev_sett.meas_period !=
			    s11_u8_to_u16(dev->data[0], dev->data[1])) {
				dev->data[0] = s11_u16_to_msb(dev->dev_sett.meas_period);
				dev->data[1] = s11_u16_to_lsb(dev->dev_sett.meas_period);
				// Write new value
				res = s11_write(dev, S11_ADDR_DEV_MEAS_PER_MSB, dev->data, 2);
				dev->dev_sett.write_count++;
			}
		}
	}

	// Number of samples
	if (res == S11_E_OK) {
		// Read current value
		res = s11_read(dev, S11_ADDR_DEV_NB_SAMP_MSB, dev->data, 2);
		if (res == S11_E_OK) {
			// Check if update needed
			if (dev->dev_sett.meas_nb_samples !=
			    s11_u8_to_u16(dev->data[0], dev->data[1])) {
				dev->data[0] = s11_u16_to_msb(dev->dev_sett.meas_nb_samples);
				dev->data[1] = s11_u16_to_lsb(dev->dev_sett.meas_nb_samples);
				// Write new value
				res = s11_write(dev, S11_ADDR_DEV_NB_SAMP_MSB, dev->data, 2);
				dev->dev_sett.write_count++;
			}
		}
	}

	// ABC period
	if (res == S11_E_OK) {
		// Read current value
		res = s11_read(dev, S11_ADDR_ABC_PER_MSB, dev->data, 2);
		if (res == S11_E_OK) {
			// Check if update needed
			if (dev->dev_sett.abc_period != s11_u8_to_u16(dev->data[0], dev->data[1])) {
				dev->data[0] = s11_u16_to_msb(dev->dev_sett.abc_period);
				dev->data[1] = s11_u16_to_lsb(dev->dev_sett.abc_period);
				// Write new value
				res = s11_write(dev, S11_ADDR_ABC_PER_MSB, dev->data, 2);
				dev->dev_sett.write_count++;
			}
		}
	}

	// ABC target
	if (res == S11_E_OK) {
		// Read current value
		res = s11_read(dev, S11_ADDR_ABC_TARGET_MSB, dev->data, 2);
		if (res == S11_E_OK) {
			// Check if update needed
			if (dev->dev_sett.abc_target != s11_u8_to_u16(dev->data[0], dev->data[1])) {
				dev->data[0] = s11_u16_to_msb(dev->dev_sett.abc_target);
				dev->data[1] = s11_u16_to_lsb(dev->dev_sett.abc_target);
				// Write new value
				res = s11_write(dev, S11_ADDR_ABC_TARGET_MSB, dev->data, 2);
				dev->dev_sett.write_count++;
			}
		}
	}

	// IIR static parameter
	if (res == S11_E_OK) {
		// Read current value
		res = s11_read(dev, S11_ADDR_IIR_STAT_PAR, dev->data, 1);
		if (res == S11_E_OK) {
			// Check if update needed
			if (dev->dev_sett.iir_static_parameter != dev->data[0]) {
				dev->data[0] = dev->dev_sett.iir_static_parameter;
				// Write new value
				res = s11_write(dev, S11_ADDR_IIR_STAT_PAR, dev->data, 1);
				dev->dev_sett.write_count++;
			}
		}
	}

	// Meter control
	if (res == S11_E_OK) {
		// Read current value
		res = s11_read(dev, S11_ADDR_DEV_METER_CTL, dev->data, 1);
		if (res == S11_E_OK) {
			// Check if update needed
			if (dev->dev_sett.meter_control != dev->data[0]) {
				dev->data[0] = dev->dev_sett.meter_control;
				// Write new value
				res = s11_write(dev, S11_ADDR_DEV_METER_CTL, dev->data, 1);
				dev->dev_sett.write_count++;
			}
		}
	}

	// Device address
	if (res == S11_E_OK) {
		// Read current value
		res = s11_read(dev, S11_ADDR_DEV_ADDR, dev->data, 1);
		if (res == S11_E_OK) {
			// Check if update needed
			if (dev->dev_sett.address != dev->data[0]) {
				dev->data[0] = dev->dev_sett.address;
				// Write new value
				res = s11_write(dev, S11_ADDR_DEV_ADDR, dev->data, 1);
				dev->dev_sett.write_count++;
			}
		}
	}

	return res;
}

static int s11_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct s11_dev *s11_device = dev->data;

	if (s11_get_meas_data(s11_device) != S11_E_OK) {
		return -EIO;
	}

	return 0;
}

static int s11_channel_get(const struct device *dev, enum sensor_channel chan,
			   struct sensor_value *val)
{
	struct s11_dev *s11_device = dev->data;

	switch (chan) {
	case SENSOR_CHAN_CO2:
		val->val1 = s11_device->meas_res.co2_fp;
		val->val2 = 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int8_t sensor_read(const struct device *i2c_master, uint16_t i2c_slave, uint8_t reg_addr,
		   uint8_t *data, uint8_t len)
{
	if (i2c_burst_read(i2c_master, i2c_slave, reg_addr, data, len)) {
		return S11_E_I2C;
	}

	return S11_E_OK;
}

int8_t sensor_write(const struct device *i2c_master, uint16_t i2c_slave, uint8_t reg_addr,
		    uint8_t *data, uint8_t len)
{
	if (i2c_burst_write(i2c_master, i2c_slave, reg_addr, data, len)) {
		return S11_E_I2C;
	}

	return S11_E_OK;
}

int8_t sensor_delay_ms(uint32_t period)
{
	k_msleep(period);

	return S11_E_OK;
}

static int s11_chip_init(const struct device *dev)
{
	struct s11_dev *s11_device = dev->data;
	int8_t res;

	s11_device->i2c_master = device_get_binding(DT_INST_BUS_LABEL(0));

	if (s11_device->i2c_master == NULL) {
		LOG_ERR("Failed to get pointer to %s device!", DT_INST_BUS_LABEL(0));
		return -EINVAL;
	}

	s11_device->i2c_slave_addr = DT_INST_REG_ADDR(0);

	s11_device->write = sensor_write;
	s11_device->read = sensor_read;
	s11_device->delay = sensor_delay_ms;

	res = s11_init(s11_device);

	if (res == S11_E_OK) {
		/* Sensor is configured in continuous mode with measurement
		 * every 5 minutes and with 8 samples per measurement.
		 */
		s11_device->dev_sett.meas_mode = S11_MM_STATE_CONT_MEAS;
		s11_device->dev_sett.meas_period = 300;
		s11_device->dev_sett.meas_nb_samples = 8;
		res = s11_set_dev_settings(s11_device);
	}

	if (res != S11_E_OK) {
		return -EIO;
	}

	return 0;
}

static const struct sensor_driver_api s11_driver_api = {
	.sample_fetch = s11_sample_fetch,
	.channel_get = s11_channel_get,
};

static struct s11_dev s11_device;

DEVICE_DT_INST_DEFINE(0, s11_chip_init, NULL, &s11_device, NULL, POST_KERNEL,
		      CONFIG_SENSOR_INIT_PRIORITY, &s11_driver_api);