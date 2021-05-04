/*
 * Copyright 2021 Matija Tudan
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include <drivers/sensor/max17260.h>
#include <logging/log.h>

#include "max17260.h"

LOG_MODULE_REGISTER(max17260, CONFIG_SENSOR_LOG_LEVEL);

#define DT_DRV_COMPAT maxim_max17260

/**
 * @brief Read a register value
 *
 * Registers have an address and a 16-bit value
 *
 * @param dev MAX17260 device to access
 * @param reg_addr Register address to read
 * @param valp Place to put the value on success
 * @return 0 if successful, or negative error code from I2C API
 */
static int max17260_reg_read(const struct device *dev, uint8_t reg_addr,
			     int16_t *valp)
{
	const struct max17260_config *cfg = dev->config;
	uint8_t i2c_data[2];
	int rc;

	rc = i2c_burst_read(cfg->i2c, cfg->i2c_addr, reg_addr, i2c_data, 2);
	if (rc < 0) {
		LOG_ERR("Unable to read register");
		return rc;
	}
	*valp = ((int16_t)i2c_data[1] << 8) | i2c_data[0];

	return 0;
}

/**
 * @brief Write a register value
 *
 * Registers have an address and a 16-bit value
 *
 * @param dev MAX17260 device to access
 * @param reg_addr Register address to write to
 * @param val Register value to write
 * @return 0 if successful, or negative error code from I2C API
 */
static int max17260_reg_write(const struct device *dev, uint8_t reg_addr,
			      int16_t val)
{
	const struct max17260_config *cfg = dev->config;
	uint8_t i2c_data[3] = { reg_addr, val & 0xFF, (uint16_t)val >> 8 };

	return i2c_write(cfg->i2c, i2c_data, sizeof(i2c_data), cfg->i2c_addr);
}

/**
 * @brief Convert current in MAX17055 units to milliamps
 *
 * @param rsense_mohms Value of Rsense in milliohms
 * @param val Value to convert (taken from a MAX17055 register)
 * @return corresponding value in milliamps
 */
static int current_to_ma(unsigned int rsense_mohms, int16_t val)
{
	return (val * 1.5625) / rsense_mohms;
}

/**
 * @brief Convert capacity in MAX17055 units to milliamps
 *
 * @param rsense_mohms Value of Rsense in milliohms
 * @param val Value to convert (taken from a MAX17055 register)
 * @return corresponding value in milliamps
 */
static int capacity_to_ma(unsigned int rsense_mohms, int16_t val)
{
	int lsb_units, rem;

	/* Get units for the LSB in uA */
	lsb_units = 5 * 1000 / rsense_mohms;
	/* Get remaining capacity in uA */
	rem = val * lsb_units;

	return rem;
}

/**
 * @brief Convert sensor value from millis
 *
 * @param val Where to store converted value in sensor_value format
 * @param val_millis Value in millis
 */
static void convert_millis(struct sensor_value *val, int32_t val_millis)
{
	val->val1 = val_millis / 1000;
	val->val2 = (val_millis % 1000) * 1000;
}

static int16_t ma_to_capacity(unsigned int rsense_mohms, uint16_t val)
{
	float lsb_units;
	int rem;

	/* Get units for the LSB in uA */
	lsb_units = rsense_mohms / 5. / 1000.;
	/* Get remaining uA in capacity */
	rem = (int)(val * 1000 * lsb_units);

	return rem;
}

/**
 * @brief Convert raw register values for specific channel
 *
 * @param dev MAX17260 device to access
 * @param chan Channel number to read
 * @param valp Returns the sensor value read on success
 * @return 0 if successful
 * @return -ENOTSUP for unsupported channels
 */
static int max17260_channel_get(const struct device *dev,
				enum sensor_channel chan,
				struct sensor_value *valp)
{
	const struct max17260_config *const config = dev->config;
	struct max17260_data *const data = dev->data;
	int32_t tmp;

	switch (chan) {
	case SENSOR_CHAN_GAUGE_VOLTAGE:
		/* Get voltage in uV */
		tmp = data->voltage * VOLTAGE_MULTIPLIER_UV;
		/* Convert to V */
		valp->val1 = tmp / 1000000;
		valp->val2 = tmp % 1000000;
		break;
	case SENSOR_CHAN_GAUGE_AVG_CURRENT: {
		int current;
		/* Get avg current in nA */
		current =
			current_to_ma(config->rsense_mohms, data->avg_current);
		//current = data->avg_current * CURRENT_MULTIPLIER_NA;
		/* Convert to mA */
		convert_millis(valp, current);
		break;
	}
	case SENSOR_CHAN_GAUGE_STATE_OF_CHARGE:
		valp->val1 = data->state_of_charge / 256;
		valp->val2 = data->state_of_charge % 256 * 1000000 / 256;
		break;
	case SENSOR_CHAN_GAUGE_TEMP:
		valp->val1 = data->internal_temp / 256;
		valp->val2 = data->internal_temp % 256 * 1000000 / 256;
		break;
	case SENSOR_CHAN_GAUGE_FULL_CHARGE_CAPACITY:
		tmp = capacity_to_ma(config->rsense_mohms, data->full_cap);
		convert_millis(valp, tmp);
		break;
	case SENSOR_CHAN_GAUGE_REMAINING_CHARGE_CAPACITY:
		tmp = capacity_to_ma(config->rsense_mohms, data->remaining_cap);
		convert_millis(valp, tmp);
		break;
	case SENSOR_CHAN_GAUGE_TIME_TO_EMPTY:
		/* Get time in ms */
		if (data->time_to_empty == 0xffff) {
			valp->val1 = 0;
			valp->val2 = 0;
		} else {
			tmp = data->time_to_empty * TIME_MULTIPLIER_MS;
			convert_millis(valp, tmp);
		}
		break;
	case SENSOR_CHAN_GAUGE_TIME_TO_FULL:
		/* Get time in ms */
		if (data->time_to_full == 0xffff) {
			valp->val1 = 0;
			valp->val2 = 0;
		} else {
			tmp = data->time_to_full * TIME_MULTIPLIER_MS;
			convert_millis(valp, tmp);
		}
		break;
	case SENSOR_CHAN_GAUGE_CYCLE_COUNT:
		valp->val1 = data->cycle_count / 100;
		valp->val2 = data->cycle_count % 100 * 10000;
		break;
	case SENSOR_CHAN_GAUGE_NOM_AVAIL_CAPACITY:
		tmp = capacity_to_ma(config->rsense_mohms, data->design_cap);
		convert_millis(valp, tmp);
		break;
	case SENSOR_CHAN_GAUGE_DESIGN_VOLTAGE:
		convert_millis(valp, config->design_voltage);
		break;
	case SENSOR_CHAN_GAUGE_DESIRED_VOLTAGE:
		convert_millis(valp, config->desired_voltage);
		break;
	case SENSOR_CHAN_GAUGE_DESIRED_CHARGING_CURRENT:
		valp->val1 = data->ichg_term;
		valp->val2 = 0;
		break;
	case MAX17260_COULOMB_COUNTER:
		/* Get spent capacity in mAh */
		data->coulomb_counter = 0xffff - data->coulomb_counter;
		valp->val1 = data->coulomb_counter / 2;
		valp->val2 = data->coulomb_counter % 2 * 10 / 2;
		break;
	default:
		LOG_ERR("Unsupported channel!");
		return -ENOTSUP;
	}

	return 0;
}

static int max17260_set_hibernate(const struct device *dev)
{
	/* Enable Hibernate */
	return max17260_reg_write(dev, HIBCFG, 0x801C);
}

static int max17260_config(const struct device *dev, enum sensor_channel chan,
			   enum sensor_attribute attr,
			   const struct sensor_value *val)
{
	/* Check MAX17260 private attribute types */
	switch ((enum max17260_sensor_attribute)attr) {
	case SENSOR_ATTR_MAX17260_HIBERNATE:
		return max17260_set_hibernate(dev);
	default:
		break;
	}
}

static int max17260_attr_set(struct device *dev, enum sensor_channel chan,
			     enum sensor_attribute attr,
			     const struct sensor_value *val)
{
	return max17260_config(dev, chan, attr, val);
}

/**
 * @brief Read register values for supported channels
 *
 * @param dev MAX17260 device to access
 * @return 0 if successful, or negative error code from I2C API
 */
static int max17260_sample_fetch(const struct device *dev,
				 enum sensor_channel chan)
{
	struct max17260_data *data = dev->data;
	struct {
		int reg_addr;
		int16_t *dest;
	} regs[] = {
		{ VCELL, &data->voltage },
		{ AVG_CURRENT, &data->avg_current },
		{ ICHG_TERM, &data->ichg_term },
		{ REP_SOC, &data->state_of_charge },
		{ INT_TEMP, &data->internal_temp },
		{ REP_CAP, &data->remaining_cap },
		{ FULL_CAP_REP, &data->full_cap },
		{ TTE, &data->time_to_empty },
		{ TTF, &data->time_to_full },
		{ CYCLES, &data->cycle_count },
		{ DESIGN_CAP, &data->design_cap },
		{ COULOMB_COUNTER, &data->coulomb_counter },
	};

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);
	for (size_t i = 0; i < ARRAY_SIZE(regs); i++) {
		int rc;

		rc = max17260_reg_read(dev, regs[i].reg_addr, regs[i].dest);
		if (rc != 0) {
			LOG_ERR("Failed to read channel %d", chan);
			return rc;
		}
	}

	return 0;
}

/**
 * @brief Initialise the fuel gauge
 *
 * @param dev MAX17260 device to access
 * @return 0 for success
 * @return -EINVAL if the I2C controller could not be found
 */
static int max17260_gauge_init(const struct device *dev)
{
	const struct max17260_config *const config = dev->config;
	int16_t tmp, hibcfg;

	if (!device_is_ready(config->i2c)) {
		LOG_ERR("Could not get pointer to %s device",
			config->i2c->name);
		return -EINVAL;
	}

	/* Read Status register */
	max17260_reg_read(dev, STATUS, &tmp);

	if (!(tmp & STATUS_POR)) {
		/*
		 * Status.POR bit is set to 1 when MAX17260 detects that
		 * a software or hardware POR event has occurred and
		 * therefore a custom configuration needs to be set...
		 * If POR event did not happen (Status.POR == 0), skip
		 * init and continue with measurements.
		 */
		LOG_DBG("No POR event detected - skip device configuration");
		return 0;
	}
	LOG_DBG("POR detected, setting custom device configuration...");

	/** STEP 1 */
	max17260_reg_read(dev, FSTAT, &tmp);

	/* Do not continue until FSTAT.DNR bit is cleared */
	while (tmp & FSTAT_DNR) {
		k_sleep(K_MSEC(10));
		max17260_reg_read(dev, FSTAT, &tmp);
	}

	/** STEP 2 */
	/* Store original HibCFG value */
	max17260_reg_read(dev, HIBCFG, &hibcfg);

	/* Exit Hibernate Mode step 1 */
	max17260_reg_write(dev, SOFT_WAKEUP, 0x0090);
	/* Exit Hibernate Mode step 2 */
	max17260_reg_write(dev, HIBCFG, 0x0000);
	/* Exit Hibernate Mode step 3 */
	max17260_reg_write(dev, SOFT_WAKEUP, 0x0000);

	/** STEP 2.1 --> OPTION 1 EZ Config (No INI file is needed) */
	/* Write DesignCap */
	max17260_reg_write(dev, DESIGN_CAP,
			   ma_to_capacity(config->rsense_mohms,
					  config->design_cap));

	/* Write IChgTerm */
	max17260_reg_write(dev, ICHG_TERM, config->desired_charging_current);

	/* Write VEmpty */
	max17260_reg_write(dev, VEMPTY,
			   ((config->empty_voltage / 10) << 7) |
				   ((config->recovery_voltage / 40) & 0x7F));

	/* Write ModelCFG */
	if (config->charge_voltage > 4275) {
		max17260_reg_write(dev, MODELCFG, 0x8400);
	} else {
		max17260_reg_write(dev, MODELCFG, 0x8000);
	}

	/*
	 * Read ModelCFG.Refresh (highest bit),
	 * proceed to Step 3 when ModelCFG.Refresh == 0
	 */
	max17260_reg_read(dev, MODELCFG, &tmp);

	/* Do not continue until ModelCFG.Refresh == 0 */
	while (tmp & MODELCFG_REFRESH) {
		k_sleep(K_MSEC(10));
		max17260_reg_read(dev, MODELCFG, &tmp);
	}

	/* Restore Original HibCFG value */
	max17260_reg_write(dev, HIBCFG, hibcfg);

	/** STEP 3 */
	/* Read Status register */
	max17260_reg_read(dev, STATUS, &tmp);

	/* Clear PowerOnReset bit */
	tmp &= ~STATUS_POR;
	max17260_reg_write(dev, STATUS, tmp);

	return 0;
}

static const struct sensor_driver_api max17260_battery_driver_api = {
	.attr_set = max17260_attr_set,
	.sample_fetch = max17260_sample_fetch,
	.channel_get = max17260_channel_get,
};

#define MAX17260_INIT(n)                                                       \
	static struct max17260_data max17260_data_##n;                         \
                                                                               \
	static const struct max17260_config max17260_config_##n = {            \
		.i2c = DEVICE_DT_GET(DT_BUS(DT_DRV_INST(n))),                  \
		.i2c_addr = DT_INST_REG_ADDR(n),                               \
		.rsense_mohms = DT_INST_PROP(n, rsense_mohms),                 \
		.design_voltage = DT_INST_PROP(n, design_voltage),             \
		.desired_voltage = DT_INST_PROP(n, desired_voltage),           \
		.desired_charging_current =                                    \
			DT_INST_PROP(n, desired_charging_current),             \
		.design_cap = DT_INST_PROP(n, design_cap),                     \
		.empty_voltage = DT_INST_PROP(n, empty_voltage),               \
		.recovery_voltage = DT_INST_PROP(n, recovery_voltage),         \
		.charge_voltage = DT_INST_PROP(n, charge_voltage),             \
	};                                                                     \
                                                                               \
	DEVICE_DT_INST_DEFINE(n, &max17260_gauge_init, device_pm_control_nop,  \
			      &max17260_data_##n, &max17260_config_##n,        \
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,        \
			      &max17260_battery_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MAX17260_INIT)
