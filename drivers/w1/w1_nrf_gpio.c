/*
 * Copyright (c) 2023 Hudson C. Dalpra
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT w1_nrf_gpio


#include <zephyr/drivers/w1.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <hal/nrf_gpio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(w1_nrf_gpio, CONFIG_W1_LOG_LEVEL);

/*
 * The time critical sections are used to ensure that the timing
 * between communication operations is correct.
 */
#if defined(CONFIG_W1_NRF_GPIO_TIME_CRITICAL)
#define W1_GPIO_ENTER_CRITICAL()   irq_lock()
#define W1_GPIO_EXIT_CRITICAL(key) irq_unlock(key)
#define W1_GPIO_WAIT_US(us)        k_busy_wait(us)
#else
#define W1_GPIO_ENTER_CRITICAL()   0u
#define W1_GPIO_EXIT_CRITICAL(key) (void)key
#define W1_GPIO_WAIT_US(us)        k_usleep(us)
#endif

/*
 * Standard timing between communication operations:
 */
#define W1_GPIO_TIMING_STD_A 6u
#define W1_GPIO_TIMING_STD_B 64u
#if defined(CONFIG_W1_NRF_GPIO_TIME_CRITICAL)
#define W1_GPIO_TIMING_STD_C 60u
#else
#define W1_GPIO_TIMING_STD_C 32u
#endif
#define W1_GPIO_TIMING_STD_D 10u
#define W1_GPIO_TIMING_STD_E 9u
#define W1_GPIO_TIMING_STD_F 55u
#define W1_GPIO_TIMING_STD_G 0u
#define W1_GPIO_TIMING_STD_H 480u
#define W1_GPIO_TIMING_STD_I 70u
#define W1_GPIO_TIMING_STD_J 410u

/*
 * Overdrive timing between communication operations:
 *
 * Not completely correct since the overdrive communication requires
 * delays of 2.5us, 7.5us and 8.5us.
 * The delays are approximated by flooring the values.
 */
#define W1_GPIO_TIMING_OD_A 1u
#define W1_GPIO_TIMING_OD_B 7u
#define W1_GPIO_TIMING_OD_C 7u
#define W1_GPIO_TIMING_OD_D 2u
#define W1_GPIO_TIMING_OD_E 1u
#define W1_GPIO_TIMING_OD_F 7u
#define W1_GPIO_TIMING_OD_G 2u
#define W1_GPIO_TIMING_OD_H 70u
#define W1_GPIO_TIMING_OD_I 8u
#define W1_GPIO_TIMING_OD_J 40u

struct w1_gpio_timing {
	uint16_t a;
	uint16_t b;
	uint16_t c;
	uint16_t d;
	uint16_t e;
	uint16_t f;
	uint16_t g;
	uint16_t h;
	uint16_t i;
	uint16_t j;
};

struct w1_gpio_config {
	/** w1 master config, common to all drivers */
	struct w1_master_config master_config;
	/** registers of the port the 1-Wire line belongs to */
	NRF_GPIO_Type *const port;
	/** mask of the 1-Wire line within its port */
	const uint32_t mask;
	/** absolute pin number, only needed to configure the pin */
	const uint32_t abs_pin;
};

struct w1_gpio_data {
	/** w1 master data, common to all drivers */
	struct w1_master_data master_data;
	/** timing parameters for 1-Wire communication */
	const struct w1_gpio_timing *timing;
	/** overdrive speed mode active */
	bool overdrive_active;
};

static const struct w1_gpio_timing std = {
	.a = W1_GPIO_TIMING_STD_A,
	.b = W1_GPIO_TIMING_STD_B,
	.c = W1_GPIO_TIMING_STD_C,
	.d = W1_GPIO_TIMING_STD_D,
	.e = W1_GPIO_TIMING_STD_E,
	.f = W1_GPIO_TIMING_STD_F,
	.g = W1_GPIO_TIMING_STD_G,
	.h = W1_GPIO_TIMING_STD_H,
	.i = W1_GPIO_TIMING_STD_I,
	.j = W1_GPIO_TIMING_STD_J,
};

static const struct w1_gpio_timing od = {
	.a = W1_GPIO_TIMING_OD_A,
	.b = W1_GPIO_TIMING_OD_B,
	.c = W1_GPIO_TIMING_OD_C,
	.d = W1_GPIO_TIMING_OD_D,
	.e = W1_GPIO_TIMING_OD_E,
	.f = W1_GPIO_TIMING_OD_F,
	.g = W1_GPIO_TIMING_OD_G,
	.h = W1_GPIO_TIMING_OD_H,
	.i = W1_GPIO_TIMING_OD_I,
	.j = W1_GPIO_TIMING_OD_J,
};

static int w1_gpio_reset_bus(const struct device *dev)
{
	const struct w1_gpio_config *cfg = dev->config;
	const struct w1_gpio_data *data = dev->data;

	const struct w1_gpio_timing *timing = data->timing;

	int ret = 0;
	unsigned int key = W1_GPIO_ENTER_CRITICAL();

	W1_GPIO_WAIT_US(timing->g);
	nrf_gpio_port_out_clear(cfg->port, cfg->mask);

	W1_GPIO_WAIT_US(timing->h);
	// k_busy_wait(timing->h);
	nrf_gpio_port_out_set(cfg->port, cfg->mask);

	#if defined(CONFIG_W1_NRF_GPIO_TIME_CRITICAL)
	W1_GPIO_WAIT_US(timing->i);
	#else
	k_busy_wait(timing->i);
	#endif

	ret = (nrf_gpio_port_in_read(cfg->port) & cfg->mask) != 0;

	ret ^= 0x01;

	W1_GPIO_WAIT_US(timing->j);

	W1_GPIO_EXIT_CRITICAL(key);
	return ret;
}

static int w1_gpio_read_bit(const struct device *dev)
{
	const struct w1_gpio_config *cfg = dev->config;
	const struct w1_gpio_data *data = dev->data;

	const struct w1_gpio_timing *timing = data->timing;

	int ret = 0;
	unsigned int key = W1_GPIO_ENTER_CRITICAL();

	nrf_gpio_port_out_clear(cfg->port, cfg->mask);

	#if defined(CONFIG_W1_NRF_GPIO_TIME_CRITICAL)
	W1_GPIO_WAIT_US(timing->a);
	#else
	k_busy_wait(timing->a);
	#endif

	nrf_gpio_port_out_set(cfg->port, cfg->mask);

	#if defined(CONFIG_W1_NRF_GPIO_TIME_CRITICAL)
	W1_GPIO_WAIT_US(timing->e);
	#else
	k_busy_wait(timing->e);
	#endif

	ret = (nrf_gpio_port_in_read(cfg->port) & cfg->mask) != 0;

	W1_GPIO_WAIT_US(timing->f);

	W1_GPIO_EXIT_CRITICAL(key);
	return ret;
}

static int w1_gpio_write_bit(const struct device *dev, const bool bit)
{
	const struct w1_gpio_config *cfg = dev->config;
	const struct w1_gpio_data *data = dev->data;

	const struct w1_gpio_timing *timing = data->timing;

	int ret = 0;
	unsigned int key = W1_GPIO_ENTER_CRITICAL();

	nrf_gpio_port_out_clear(cfg->port, cfg->mask);

	#if defined(CONFIG_W1_NRF_GPIO_TIME_CRITICAL)
	W1_GPIO_WAIT_US(bit ? timing->a : timing->c);
	#else
	bit ? k_busy_wait(timing->a) : W1_GPIO_WAIT_US(timing->c);
	#endif

	nrf_gpio_port_out_set(cfg->port, cfg->mask);

	#if defined(CONFIG_W1_NRF_GPIO_TIME_CRITICAL)
	W1_GPIO_WAIT_US(bit ? timing->b : timing->d);
	#else
	k_busy_wait(bit ? timing->b : timing->d);
	#endif

	W1_GPIO_EXIT_CRITICAL(key);

	return ret;
}

static int w1_gpio_read_byte(const struct device *dev)
{
	int ret = 0;
	int byte = 0x00;

	for (int i = 0; i < 8; i++) {
		ret = w1_gpio_read_bit(dev);
		if (ret < 0) {
			return ret;
		}

		byte >>= 1;
		if (ret) {
			byte |= 0x80;
		}
	}

	return byte;
}

static int w1_gpio_write_byte(const struct device *dev, const uint8_t byte)
{
	int ret = 0;
	uint8_t write = byte;

	for (int i = 0; i < 8; i++) {

		ret = w1_gpio_write_bit(dev, write & 0x01);

		if (ret < 0) {
			return ret;
		}
		write >>= 1;
	}

	return ret;
}

static int w1_gpio_configure(const struct device *dev, enum w1_settings_type type, uint32_t value)
{
	struct w1_gpio_data *data = dev->data;

	switch (type) {
	case W1_SETTING_SPEED:
		data->overdrive_active = (value != 0);
		data->timing = data->overdrive_active ? &od : &std;
		return 0;
	default:
		return -ENOTSUP;
	}
}

static int w1_gpio_init(const struct device *dev)
{
	const struct w1_gpio_config *cfg = dev->config;
	struct w1_gpio_data *data = dev->data;

	// equivalent of gpio_pin_configure_dt()
	// =========
	nrf_gpio_cfg(
		cfg->abs_pin,
		NRF_GPIO_PIN_DIR_OUTPUT,
		NRF_GPIO_PIN_INPUT_CONNECT,
		NRF_GPIO_PIN_NOPULL,
		NRF_GPIO_PIN_S0D1,
		NRF_GPIO_PIN_NOSENSE
	);
	nrf_gpio_port_out_set(cfg->port, cfg->mask);
	// =========

	data->timing = &std;
	data->overdrive_active = false;

	LOG_DBG("w1-gpio initialized, with %d slave devices", cfg->master_config.slave_count);
	return 0;
}

static DEVICE_API(w1, w1_nrf_gpio_driver_api) = {
	.reset_bus = w1_gpio_reset_bus,
	.read_bit = w1_gpio_read_bit,
	.write_bit = w1_gpio_write_bit,
	.read_byte = w1_gpio_read_byte,
	.write_byte = w1_gpio_write_byte,
	.configure = w1_gpio_configure,
};

/* Resolved at build time, so the hot path needs no runtime port decoding. */
#define W1_NRF_GPIO_PORT(inst)                                                                     \
	_CONCAT(NRF_P, DT_PROP(DT_GPIO_CTLR(DT_DRV_INST(inst), gpios), port))

#define W1_NRF_GPIO_INIT(inst)                                                                  \
	static const struct w1_gpio_config w1_gpio_cfg_##inst = {                                  \
		.master_config.slave_count = W1_INST_SLAVE_COUNT(inst),                            \
		.port = W1_NRF_GPIO_PORT(inst),                                                    \
		.mask = BIT(DT_INST_GPIO_PIN(inst, gpios)),                                        \
        .abs_pin = NRF_DT_GPIOS_TO_PSEL(DT_DRV_INST(inst), gpios)};                                       \
	static struct w1_gpio_data w1_gpio_data_##inst = {};                                       \
	DEVICE_DT_INST_DEFINE(inst, &w1_gpio_init, NULL, &w1_gpio_data_##inst,                     \
			      &w1_gpio_cfg_##inst, POST_KERNEL, CONFIG_W1_INIT_PRIORITY,           \
			      &w1_nrf_gpio_driver_api);

DT_INST_FOREACH_STATUS_OKAY(W1_NRF_GPIO_INIT)
