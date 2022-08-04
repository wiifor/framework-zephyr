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

#include "s11.h"

/*******************************************************************************/
/*                        Private function declarations                        */
/*******************************************************************************/
int8_t s11_null_ptr_check(struct s11_dev *dev);
uint16_t s11_u8_to_u16(uint8_t msb, uint8_t lsb);
uint8_t s11_u16_to_msb(uint16_t val);
uint8_t s11_u16_to_lsb(uint16_t val);
int8_t s11_read(struct s11_dev *dev, uint8_t reg_addr, uint8_t *data, uint8_t len);
int8_t s11_write(struct s11_dev *dev, uint8_t reg_addr, uint8_t *data, uint8_t len);

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
		res = dev->read(dev->i2c_address, reg_addr, data, len);
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
		// Call the sensors write method
		res = dev->write(dev->i2c_address, reg_addr, data, len);
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
		res = s11_write(dev, S11_ADDR_RESET, dev->data, 1);
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
	bool restart = false;
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
	if ((dev->dev_sett.meter_control < 0) ||
	    (dev->dev_sett.meter_control > 31)) { //31= 0b00011111 // todo, wrong limits/values
		return S11_E_VAL_OUT_OF_LIMIT;
	}
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
				s11_delay(dev, S11_EE_WR_DELAY_MS);
				dev->dev_sett.write_count++;
				restart = true;
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
				s11_delay(dev, S11_EE_WR_DELAY_MS);
				dev->dev_sett.write_count++;
				restart = true;
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
				s11_delay(dev, S11_EE_WR_DELAY_MS);
				dev->dev_sett.write_count++;
				restart = true;
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
				s11_delay(dev, S11_EE_WR_DELAY_MS);
				dev->dev_sett.write_count++;
				restart = true;
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
				s11_delay(dev, S11_EE_WR_DELAY_MS);
				dev->dev_sett.write_count++;
				restart = true;
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
				s11_delay(dev, S11_EE_WR_DELAY_MS);
				dev->dev_sett.write_count++;
				restart = true;
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
				s11_delay(dev, S11_EE_WR_DELAY_MS);
				dev->dev_sett.write_count++;
				restart = true;
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
				s11_delay(dev, S11_EE_WR_DELAY_MS);
				dev->dev_sett.write_count++;
				restart = true;
			}
		}
	}

	// Restart sensor if writing to EE has occured
	if (restart) {
		s11_restart(dev);
		s11_delay(dev, S11_RESTART_DELAY_MS);
	}
	return res;
}