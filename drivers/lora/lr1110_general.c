/*
 * Copyright (c) 2021, Wiifor SAS. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/gpio.h>
#include <drivers/lora.h>
#include <logging/log.h>
#include <zephyr.h>

/* LoRaMac-node specific includes */
#include <radio.h>

#include "lr1110_general.h"

LOG_MODULE_REGISTER(lr1110_general, LOG_LEVEL_DBG);

static struct lr1110_data {
	struct k_sem data_sem;
	RadioEvents_t events;
	uint8_t *rx_buf;
	uint8_t rx_len;
	int8_t snr;
	int16_t rssi;
} dev_data;

int __lr1110_configure_pin(const struct device * *dev, const char *controller,
			   gpio_pin_t pin, gpio_flags_t flags)
{
	int err;

	*dev = device_get_binding(controller);
	if (!(*dev)) {
		LOG_ERR("Cannot get pointer to %s device", controller);
		return -EIO;
	}

	err = gpio_pin_configure(*dev, pin, flags);
	if (err) {
		LOG_ERR("Cannot configure gpio %s %d: %d", controller, pin,
			err);
		return err;
	}

	return 0;
}

static void lr1110_ev_rx_done(uint8_t *payload, uint16_t size, int16_t rssi,
			      int8_t snr)
{
	Radio.Sleep();

	dev_data.rx_buf = payload;
	dev_data.rx_len = size;
	dev_data.rssi = rssi;
	dev_data.snr = snr;

	k_sem_give(&dev_data.data_sem);
}

static void lr1110_ev_tx_done(void)
{
	Radio.Sleep();
}

int lr1110_lora_send(const struct device *dev, uint8_t *data,
		     uint32_t data_len)
{
	Radio.SetMaxPayloadLength(MODEM_LORA, data_len);

	Radio.Send(data, data_len);

	return 0;
}

int lr1110_lora_recv(const struct device *dev, uint8_t *data, uint8_t size,
		     k_timeout_t timeout, int16_t *rssi, int8_t *snr)
{
	int ret;

	Radio.SetMaxPayloadLength(MODEM_LORA, 255);
	Radio.Rx(0);

	ret = k_sem_take(&dev_data.data_sem, timeout);
	if (ret < 0) {
		LOG_ERR("Receive timeout!");
		return ret;
	}

	/* Only copy the bytes that can fit the buffer, drop the rest */
	if (dev_data.rx_len > size)
		dev_data.rx_len = size;

	/*
	 * FIXME: We are copying the global buffer here, so it might get
	 * overwritten inbetween when a new packet comes in. Use some
	 * wise method to fix this!
	 */
	memcpy(data, dev_data.rx_buf, dev_data.rx_len);

	if (rssi != NULL) {
		*rssi = dev_data.rssi;
	}

	if (snr != NULL) {
		*snr = dev_data.snr;
	}

	return dev_data.rx_len;
}

int lr1110_lora_config(const struct device *dev,
		       struct lora_modem_config *config)
{
	Radio.SetChannel(config->frequency);

	if (config->tx) {
		Radio.SetTxConfig(MODEM_LORA, config->tx_power, 0,
				  config->bandwidth, config->datarate,
				  config->coding_rate, config->preamble_len,
				  false, true, 0, 0, false, 4000);
	} else {
		/* TODO: Get symbol timeout value from config parameters */
		Radio.SetRxConfig(MODEM_LORA, config->bandwidth,
				  config->datarate, config->coding_rate,
				  0, config->preamble_len, 10, false, 0,
				  false, 0, 0, false, true);
	}

	return 0;
}

int lr1110_lora_test_cw(const struct device *dev, uint32_t frequency,
			int8_t tx_power,
			uint16_t duration)
{
	Radio.SetTxContinuousWave(frequency, tx_power, duration);
	return 0;
}

int lr1110_init(const struct device *dev)
{
	k_sem_init(&dev_data.data_sem, 0, K_SEM_MAX_LIMIT);

	dev_data.events.TxDone = lr1110_ev_tx_done;
	dev_data.events.RxDone = lr1110_ev_rx_done;
	Radio.Init(&dev_data.events);

	/*
	 * Automatically place the radio into sleep mode upon boot.
	 * The required `lora_config` call before transmission or reception
	 * will bring the radio out of sleep mode before it is used. The radio
	 * is automatically placed back into sleep mode upon TX or RX
	 * completion.
	 */
	Radio.Sleep();
	return 0;
}

void lr1110_sleep()
{
	Radio.Sleep();
}