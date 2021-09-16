/*
 * Copyright (c) 2021, Wiifor SAS. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/gpio.h>
#include <drivers/lora.h>
#include <drivers/spi.h>
#include <zephyr.h>

#include <lr1110/lr1110.h>

#include "lr1110_system.h"
#include "lr1110_radio.h"
#include "lr1110_general.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(lr1110, LOG_LEVEL_DBG);

#define DT_DRV_COMPAT semtech_lr1110

#define GPIO_CS_LABEL DT_INST_SPI_DEV_CS_GPIOS_LABEL(0)
#define GPIO_CS_PIN DT_INST_SPI_DEV_CS_GPIOS_PIN(0)
#define GPIO_CS_FLAGS DT_INST_SPI_DEV_CS_GPIOS_FLAGS(0)

#define GPIO_RESET_PIN DT_INST_GPIO_PIN(0, reset_gpios)
#define GPIO_BUSY_PIN DT_INST_GPIO_PIN(0, busy_gpios)
#define GPIO_DIO1_PIN DT_INST_GPIO_PIN(0, dio9_gpios)
#define GPIO_TX_ENABLE_PIN DT_INST_GPIO_PIN(0, tx_enable_gpios)
#define GPIO_RX_ENABLE_PIN DT_INST_GPIO_PIN(0, rx_enable_gpios)

#define HAVE_TCXO DT_INST_NODE_HAS_PROP(0, tcxo_voltage)
#if HAVE_TCXO
#define TCXO_VOLTAGE DT_INST_PROP(0, tcxo_voltage)
#endif

#if DT_INST_NODE_HAS_PROP(0, tcxo_power_startup_delay_ms)
#define TCXO_POWER_STARTUP_DELAY_MS DT_INST_PROP(0, tcxo_power_startup_delay_ms)
#else
#define TCXO_POWER_STARTUP_DELAY_MS 0
#endif

#define MODE(m) [LR1110_HAL_OP_MODE_##m] = #m
static const char *const mode_names[] = {
	MODE(SLEEP), MODE(STDBY_RC), MODE(STDBY_XOSC), MODE(FS),
	MODE(TX),    MODE(RX),	     MODE(RX_DC),      MODE(CAD),
};
#undef MODE

lr1110_t *context = &LR1110;

static void lr1110_board_init_tcxo_io(const void *context);

static const char *lr1110_mode_name(lr1110_hal_operating_mode_t m)
{
	static const char *unknown_mode = "unknown";

	if (m < ARRAY_SIZE(mode_names) && mode_names[m]) {
		return mode_names[m];
	} else {
		return unknown_mode;
	}
}

static void lr1110_set_tx_enable(const void *context, int value)
{
#if HAVE_GPIO_TX_ENABLE
	gpio_pin_set((lr1110_t *)context->tx_enable, GPIO_TX_ENABLE_PIN, value);
#endif
}

static void lr1110_set_rx_enable(const void *context, int value)
{
#if HAVE_GPIO_RX_ENABLE
	gpio_pin_set((lr1110_t *)context->rx_enable, GPIO_RX_ENABLE_PIN, value);
#endif
}

void lr1110_board_set_rf_tx_power(const void *context, int8_t power)
{
	LOG_DBG("power: %" PRIi8, power);
	if (power > 0) {
		if (power > 22) {
			power = 22;
		}
	} else {
		if (power < -9) {
			power = -9;
		}
	}
	lr1110_radio_set_tx_params(context, power, LR1110_RADIO_RAMP_TIME_40U);
}

uint32_t lr1110_board_get_tcxo_wakeup_time(const void *context)
{
	return TCXO_POWER_STARTUP_DELAY_MS + 42;
}

uint32_t lr1110_get_dio_1_pin_state(const void *context)
{
	return gpio_pin_get(((lr1110_t *)context)->dio9, GPIO_DIO1_PIN) > 0 ? 1U : 0U;
}

void lr1110_board_init(const void *context, lr1110_dio_irq_handler dio_irq)
{
	lr1110_system_reset(context);
	lr1110_hal_set_operating_mode(context, LR1110_HAL_OP_MODE_STDBY_RC);

	((lr1110_t *)context)->radio_dio_irq = dio_irq;

	lr1110_system_stat1_t stat1;
	lr1110_system_stat2_t stat2;
	uint32_t irq = 0;
	lr1110_system_get_status(context, &stat1, &stat2, &irq);
	lr1110_system_version_t version;
	lr1110_system_get_version(context, &version);
	lr1110_system_errors_t errors = { 0 };
	lr1110_system_get_errors(context, &errors);
	lr1110_system_clear_errors(context);

	// Initialize TCXO control
	lr1110_board_init_tcxo_io(context);

	// Initialize RF switch control
	lr1110_system_rfswitch_config_t rf_switch_configuration;
	rf_switch_configuration.enable = LR1110_SYSTEM_RFSW0_HIGH | LR1110_SYSTEM_RFSW1_HIGH;
	rf_switch_configuration.standby = 0;
	rf_switch_configuration.rx = LR1110_SYSTEM_RFSW0_HIGH;
	rf_switch_configuration.tx = LR1110_SYSTEM_RFSW0_HIGH | LR1110_SYSTEM_RFSW1_HIGH;
	rf_switch_configuration.wifi = 0;
	rf_switch_configuration.gnss = 0;

	lr1110_system_set_dio_as_rf_switch(context, &rf_switch_configuration);

	lr1110_radio_pa_config_t paConfig = {
		.pa_sel = LR1110_RADIO_PA_SEL_LP,
		.pa_reg_supply = LR1110_RADIO_PA_REG_SUPPLY_DCDC,
		.pa_dutycycle = 0x04,
		.pa_hp_sel = 0x00,
	};
	lr1110_radio_set_pa_config(context, &paConfig);

	// Set packet type
	lr1110_radio_packet_types_t packet_type = LR1110_RADIO_PACKET_LORA;
	lr1110_radio_set_packet_type(context, packet_type);
}

void lr1110_board_init_tcxo_io(const void *context)
{
#if HAVE_TCXO

	lr1110_system_set_tcxo_mode(context, TCXO_VOLTAGE,
				    (lr1110_board_get_tcxo_wakeup_time(context) * 1000) / 30.52);

	uint8_t calib_params =
		LR1110_SYSTEM_CALIBRATE_LF_RC_MASK | LR1110_SYSTEM_CALIBRATE_HF_RC_MASK |
		LR1110_SYSTEM_CALIBRATE_PLL_MASK | LR1110_SYSTEM_CALIBRATE_ADC_MASK |
		LR1110_SYSTEM_CALIBRATE_IMG_MASK | LR1110_SYSTEM_CALIBRATE_PLL_TX_MASK;
	lr1110_system_calibrate(context, calib_params);
#else
	LOG_DBG("No TCXO configured");
#endif
}

//
// lr1110_hal.h API implementation
//

static lr1110_hal_status_t lr1110_hal_wait_on_busy(const void *context);

lr1110_hal_status_t lr1110_hal_write(const void *context, const uint8_t *command,
				     const uint16_t command_length, const uint8_t *data,
				     const uint16_t data_length)
{
	if (lr1110_hal_wakeup(context) == LR1110_HAL_STATUS_OK) {
		const struct spi_buf tx_buf[] = { {
							  .buf = (uint8_t *)command,
							  .len = command_length,
						  },
						  {
							  .buf = (uint8_t *)data,
							  .len = data_length,
						  } };
		const struct spi_buf_set tx = {
			.buffers = tx_buf,
			.count = ARRAY_SIZE(tx_buf),
		};

		spi_write(((lr1110_t *)context)->spi, &((lr1110_t *)context)->spi_cfg, &tx);

		// 0x011B - LR1110_SYSTEM_SET_SLEEP_OC
		if (((command[0] << 8) | command[1]) != 0x011B) {
			return lr1110_hal_wait_on_busy(context);
		} else {
			return LR1110_HAL_STATUS_OK;
		}
	}
	return LR1110_HAL_STATUS_ERROR;
}

lr1110_hal_status_t lr1110_hal_read(const void *context, const uint8_t *command,
				    const uint16_t command_length, uint8_t *data,
				    const uint16_t data_length)
{
	if (lr1110_hal_wakeup(context) == LR1110_HAL_STATUS_OK) {
		uint8_t tmp_rx_buf[1] = { 0x00 };

		const struct spi_buf tx_buf[] = { {
			.buf = (uint8_t *)command,
			.len = command_length,
		} };
		const struct spi_buf_set tx = {
			.buffers = tx_buf,
			.count = ARRAY_SIZE(tx_buf),
		};

		const struct spi_buf rx_buf[] = { {
							  .buf = tmp_rx_buf,
							  .len = 1,
						  },
						  {
							  .buf = data,
							  .len = data_length,
						  } };

		const struct spi_buf_set rx = {
			.buffers = rx_buf,
			.count = ARRAY_SIZE(rx_buf),
		};

		spi_write(((lr1110_t *)context)->spi, &((lr1110_t *)context)->spi_cfg, &tx);
		lr1110_hal_wait_on_busy(context);

		spi_read(((lr1110_t *)context)->spi, &((lr1110_t *)context)->spi_cfg, &rx);

		return lr1110_hal_wait_on_busy(context);
	}
	return LR1110_HAL_STATUS_ERROR;
}

lr1110_hal_status_t lr1110_hal_write_read(const void *context, const uint8_t *command,
					  uint8_t *data, const uint16_t data_length)
{
	if (lr1110_hal_wakeup(context) == LR1110_HAL_STATUS_OK) {
		const struct spi_buf tx_buf[] = { {
			.buf = (uint8_t *)command,
			.len = data_length,
		} };
		const struct spi_buf_set tx = {
			.buffers = tx_buf,
			.count = ARRAY_SIZE(tx_buf),
		};

		const struct spi_buf rx_buf[] = { {
			.buf = data,
			.len = data_length,
		} };
		const struct spi_buf_set rx = {
			.buffers = rx_buf,
			.count = ARRAY_SIZE(rx_buf),
		};

		spi_transceive(((lr1110_t *)context)->spi, &((lr1110_t *)context)->spi_cfg, &tx,
			       &rx);

		lr1110_hal_wait_on_busy(context);

		// 0x011B - LR1110_SYSTEM_SET_SLEEP_OC
		if (((command[0] << 8) | command[1]) != 0x011B) {
			return lr1110_hal_wait_on_busy(context);
		} else {
			return LR1110_HAL_STATUS_OK;
		}
	}
	return LR1110_HAL_STATUS_ERROR;
}

void lr1110_hal_reset(const void *context)
{
	LOG_DBG("Resetting radio");
	gpio_pin_set(((lr1110_t *)context)->reset, GPIO_RESET_PIN, 1);
	k_sleep(K_MSEC(1));
	gpio_pin_set(((lr1110_t *)context)->reset, GPIO_RESET_PIN, 0);
	k_sleep(K_MSEC(50));
	((lr1110_t *)context)->op_mode = LR1110_HAL_OP_MODE_STDBY_RC;
}

lr1110_hal_status_t lr1110_hal_wakeup(const void *context)
{
	if ((lr1110_hal_get_operating_mode(context) == LR1110_HAL_OP_MODE_SLEEP) ||
	    (lr1110_hal_get_operating_mode(context) == LR1110_HAL_OP_MODE_RX_DC)) {
		// Send dummy byte to wake up the radio ready to accept commands
		uint8_t dummy_byte[] = { '\0' };

		const struct spi_buf tx_buf[] = { {
			.buf = dummy_byte,
			.len = sizeof(dummy_byte),
		} };

		const struct spi_buf_set tx = {
			.buffers = tx_buf,
			.count = ARRAY_SIZE(tx_buf),
		};

		spi_write(((lr1110_t *)context)->spi, &((lr1110_t *)context)->spi_cfg, &tx);

		// Radio is awake in STDBY_RC mode
		((lr1110_t *)context)->op_mode = LR1110_HAL_OP_MODE_STDBY_RC;
	}

	return lr1110_hal_wait_on_busy(context);
}

lr1110_hal_status_t lr1110_hal_wait_on_busy(const void *context)
{
	while (gpio_pin_get(((lr1110_t *)context)->busy, GPIO_BUSY_PIN)) {
		k_sleep(K_MSEC(1));
	}
	return LR1110_HAL_STATUS_OK;
}

lr1110_hal_operating_mode_t lr1110_hal_get_operating_mode(const void *context)
{
	return ((lr1110_t *)context)->op_mode;
}

void lr1110_hal_set_operating_mode(const void *context, lr1110_hal_operating_mode_t op_mode)
{
	LOG_DBG("SetOperatingMode: %s (%i)", lr1110_mode_name(op_mode), op_mode);

	((lr1110_t *)context)->op_mode = op_mode;

	/* To avoid inadvertently putting the RF switch in an
	 * undefined state, first disable the port we don't want to
	 * use and then enable the other one.
	 */
	switch (op_mode) {
	case LR1110_HAL_OP_MODE_TX:
		lr1110_set_rx_enable(context, 0);
		lr1110_set_tx_enable(context, 1);
		break;

	case LR1110_HAL_OP_MODE_RX:
	case LR1110_HAL_OP_MODE_RX_C:
	case LR1110_HAL_OP_MODE_RX_DC:
		lr1110_set_tx_enable(context, 0);
		lr1110_set_rx_enable(context, 1);
		break;
	default:
		lr1110_set_rx_enable(context, 0);
		lr1110_set_tx_enable(context, 0);
		break;
	}
}

static void lr1110_dio9_irq_work_handler(struct k_work *work)
{
	if (!(((lr1110_t *)context)->radio_dio_irq)) {
		LOG_WRN("DIO1 interrupt without valid HAL IRQ callback.");
		return;
	}

	((lr1110_t *)context)->radio_dio_irq(NULL);
	if (Radio.IrqProcess) {
		Radio.IrqProcess();
	}
}

static void lr1110_dio9_irq_callback(const struct device *dev, struct gpio_callback *cb,
				     uint32_t pins)
{
	if (pins & BIT(GPIO_DIO1_PIN)) {
		k_work_submit(&((lr1110_t *)context)->dio9_irq_work);
	}
}

static int lr1110_lora_init(const struct device *dev)
{
	LOG_DBG("Initializing %s", DT_INST_LABEL(0));

	if (lr1110_configure_pin(reset, GPIO_OUTPUT_ACTIVE) ||
	    lr1110_configure_pin(busy, GPIO_INPUT) ||
	    lr1110_configure_pin(dio9, (GPIO_INPUT | GPIO_INT_DEBOUNCE)) ||
	    lr1110_configure_pin(rx_enable, GPIO_OUTPUT_INACTIVE) ||
	    lr1110_configure_pin(tx_enable, GPIO_OUTPUT_INACTIVE)) {
		return -EIO;
	}

	int err = gpio_pin_interrupt_configure(((lr1110_t *)context)->dio9, GPIO_DIO1_PIN,
					       GPIO_INT_EDGE_RISING);
	if (err != 0)
		LOG_ERR("Cannot set interrupt on dev: err=%d", err);

	k_work_init(&((lr1110_t *)context)->dio9_irq_work, lr1110_dio9_irq_work_handler);
	gpio_init_callback(&((lr1110_t *)context)->dio9_irq_callback, lr1110_dio9_irq_callback,
			   BIT(GPIO_DIO1_PIN));
	if (gpio_add_callback(((lr1110_t *)context)->dio9,
			      &((lr1110_t *)context)->dio9_irq_callback) < 0) {
		LOG_ERR("Could not set GPIO callback for DIO1 interrupt.");
		return -EIO;
	}

	(((lr1110_t *)context)->spi) = device_get_binding(DT_INST_BUS_LABEL(0));
	if (!((lr1110_t *)context)->spi) {
		LOG_ERR("Cannot get pointer to %s device", DT_INST_BUS_LABEL(0));
		return -EINVAL;
	}

	((lr1110_t *)context)->spi_cs.gpio_dev = device_get_binding(GPIO_CS_LABEL);
	if (!((lr1110_t *)context)->spi_cs.gpio_dev) {
		LOG_ERR("Cannot get pointer to %s device", GPIO_CS_LABEL);
		return -EIO;
	}
	((lr1110_t *)context)->spi_cs.gpio_pin = GPIO_CS_PIN;
	((lr1110_t *)context)->spi_cs.gpio_dt_flags = GPIO_CS_FLAGS;
	((lr1110_t *)context)->spi_cs.delay = 0U;

	((lr1110_t *)context)->spi_cfg.cs = &((lr1110_t *)context)->spi_cs;

	((lr1110_t *)context)->spi_cfg.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB;
	((lr1110_t *)context)->spi_cfg.frequency = DT_INST_PROP(0, spi_max_frequency);
	((lr1110_t *)context)->spi_cfg.slave = DT_INST_REG_ADDR(0);

	// Send dummy byte to initialize SPI
	uint8_t dummy_byte[] = { '\0' };

	const struct spi_buf tx_buf[] = { {
		.buf = dummy_byte,
		.len = sizeof(dummy_byte),
	} };

	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = ARRAY_SIZE(tx_buf),
	};

	spi_write(((lr1110_t *)context)->spi, &((lr1110_t *)context)->spi_cfg, &tx);

	return 0;
}

static const struct lora_driver_api lr1110_lora_api = {
	.config = lr1110_lora_config,
	.send = lr1110_lora_send,
	.recv = lr1110_lora_recv,
	.test_cw = lr1110_lora_test_cw,
};

DEVICE_DT_INST_DEFINE(0, &lr1110_lora_init, NULL, NULL, NULL, POST_KERNEL,
		      CONFIG_LORA_INIT_PRIORITY, &lr1110_lora_api);