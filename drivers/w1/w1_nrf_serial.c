/*
 * Copyright (c) 2022 Thomas Stranger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

 #define DT_DRV_COMPAT nrf_w1_serial
 /**
 * @brief 1-Wire Bus Master driver using Zephyr serial interface.
 *
 * This driver implements the 1-Wire interface using an uart.
 * The driver uses a uart peripheral with a baudrate of 115.2 kBd to send
 * and receive data bits and a baurade of 9.6 kBd for slave reset and
 * presence detection as suggested for normal speed operating mode in:
 * https://www.analog.com/en/resources/technical-articles/using-a-uart-to-implement-a-1wire-bus-master.html
 * For overdrive speed communication baudrates of 1 MBd and 115.2 kBd
 * are used, respectively.
 */

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/drivers/w1.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>

#include <nrfx_uarte.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <hal/nrf_gpio.h>

#define PIN1_24 NRF_GPIO_PIN_MAP(1, 24)

LOG_MODULE_REGISTER(nrf_w1_serial, CONFIG_W1_LOG_LEVEL);

#define W1_SERIAL_READ_REQ_BYTE 0xFF
#define W1_SERIAL_BIT_1         0xFF
#define W1_SERIAL_BIT_0         0x00

#define CONFIG_W1_NRF_SERIAL_TIME_CRITICAL 0


#define W1_SERIAL_STD_RESET_BYTE 0xF0

/*
 * Overdrive speed signal parameters:
 * Unlike standard speed, RST and DATA timing defined in Devicetree
 */
#define W1_SERIAL_OD_RESET_BYTE 0xE0

#define TIME_CRITICAL 0

struct w1_serial_config {
	/** w1 master config, common to all drivers */
	struct w1_master_config master_config;
	/** Baud rate for overdrive speed data communication */
	uint32_t od_data_baud;
	/** Baud rate for overdrive speed reset/presence detection */
	uint32_t od_reset_baud;
    /** Pin control configuration */
    const struct pinctrl_dev_config *pcfg;
    /** IRQ configuration function */
    void (*irq_config_func)(void);
};

struct w1_serial_data {
	/** w1 master data, common to all drivers */
	struct w1_master_data master_data;
	bool overdrive_active;
    nrfx_uarte_t inst;
    struct k_sem tx_sem;
    struct k_sem rx_sem;
    uint8_t rx_byte;
};

/*
 * Concurrently transmits and receives one 1-Wire bit
 * by sending and receiving one uart byte
 */
static int serial_tx_rx(const struct device *dev, const uint8_t *tx_data, uint8_t *rx_data,
			size_t len, uint32_t timeout)
{
    struct w1_serial_data * data = dev->data;
    // nrf_gpio_cfg_output(PIN1_24);

    #if (CONFIG_W1_NRF_SERIAL_TIME_CRITICAL)
    int ret = 0;
    k_timepoint_t end;
    #endif

	__ASSERT_NO_MSG(tx_data != NULL);
	__ASSERT_NO_MSG(rx_data != NULL);

    #if (CONFIG_W1_NRF_SERIAL_TIME_CRITICAL)
    ret = nrfx_uarte_rx_buffer_set(&data->inst, &data->rx_byte, 1);

    if (ret != 0) {
        LOG_ERR("Failed to set rx buffer");
        return -EIO;
    }

    ret = nrfx_uarte_rx_enable(&data->inst, 0);

    if (ret != 0 && ret != -EBUSY) {
        LOG_ERR("Failed to enable uarte RX");
        return -EIO;
    }
    #endif

    for (int i = 0; i < len; i++)
    {
        #if (CONFIG_W1_NRF_SERIAL_TIME_CRITICAL)
        ret = nrfx_uarte_tx(&data->inst, &tx_data[i], 0x1, NRFX_UARTE_TX_BLOCKING);

        if (ret != 0){
            (void)nrfx_uarte_rx_abort(&data->inst, true, true);
            LOG_ERR("Failed to send data");
            return -EIO;
        }

        end = sys_timepoint_calc(K_USEC(timeout));

        do {
            ret = nrfx_uarte_rx_ready(&data->inst, NULL);
        }
        while(ret != 0 && !sys_timepoint_expired(end));

        if (ret != 0){
            (void)nrfx_uarte_rx_abort(&data->inst, true, true);
            LOG_ERR("Failed to receive data");
            return -EIO;
        }

        rx_data[i] = data->rx_byte;

        /*
         * Re-arm only between bits; skip after the last one - the next
         * serial_tx_rx() call re-arms at entry.
         */

        if (i + 1 < len) {
            if (nrfx_uarte_rx_buffer_set(&data->inst, &data->rx_byte, 1) != 0) {
                nrfx_uarte_rx_abort(&data->inst, true, true);
                LOG_ERR("Failed to re-arm rx buffer");
                return -EIO;
            }
        }
        #else
        if (nrfx_uarte_tx(&data->inst, &tx_data[i], 0x1, 0) != 0) {
            LOG_ERR("Failed to send data");
            return -EIO;
        }

        // if (k_sem_take(&data->tx_sem, K_USEC(timeout)) != 0)
        // {
        //     LOG_ERR("Failed to take tx semaphore");
        //     return -EIO;
        // }
        // nrf_gpio_pin_clear(PIN1_24);
        if (k_sem_take(&data->rx_sem, K_USEC(timeout)) != 0)
        {
            LOG_ERR("Failed to take rx semaphore");
            return -EIO;
        }
        rx_data[i] = data->rx_byte;

        #endif
    }
    return 0;
}

/* Concurretly transmits and receives one 1-Wire byte */
static int serial_tx_rx_byte(const struct device *dev, uint8_t tx_byte, uint8_t *rx_byte)
{
	__ASSERT_NO_MSG(rx_byte != NULL);
	uint8_t byte_representation[8];

	for (int i = 0; i < 8; ++i) {
		/*
		 * The uart start bit pulls the line low to indicate either
		 * write Bit 1, or read low time.
		 * Bit representations vary by speed mode.
		 */
		byte_representation[i] =
			((tx_byte & (1 << i)) != 0) ? W1_SERIAL_BIT_1 : W1_SERIAL_BIT_0;
	}

	if (serial_tx_rx(dev, &byte_representation[0], &byte_representation[0], 8,
			 CONFIG_NRF_W1_SERIAL_BIT_TIMEOUT) < 0) {
		return -EIO;
	}

	*rx_byte = 0;
	for (int i = 0; i < 8; ++i) {
		/*
		 * rx-byte different from 0xFF indicates that a slave has
		 * pulled line low to transmit a 0 bit, otherwise a 1 bit.
		 */
		*rx_byte |= (uint8_t)(byte_representation[i] == 0xFF) << i;
	}

	return 0;
}

/*
 * Maps a raw devicetree baud rate (Hz) to the nrf_uarte_baudrate_t enum required by nrfx.
 */
static nrf_uarte_baudrate_t w1_serial_baudrate_from_hz(uint32_t baud)
{
	switch (baud) {
	case 1200:    return NRF_UARTE_BAUDRATE_1200;
	case 2400:    return NRF_UARTE_BAUDRATE_2400;
	case 4800:    return NRF_UARTE_BAUDRATE_4800;
	case 9600:    return NRF_UARTE_BAUDRATE_9600;
	case 14400:   return NRF_UARTE_BAUDRATE_14400;
	case 19200:   return NRF_UARTE_BAUDRATE_19200;
	case 28800:   return NRF_UARTE_BAUDRATE_28800;
	case 31250:   return NRF_UARTE_BAUDRATE_31250;
	case 38400:   return NRF_UARTE_BAUDRATE_38400;
	case 56000:   return NRF_UARTE_BAUDRATE_56000;
	case 57600:   return NRF_UARTE_BAUDRATE_57600;
	case 76800:   return NRF_UARTE_BAUDRATE_76800;
	case 115200:  return NRF_UARTE_BAUDRATE_115200;
	case 230400:  return NRF_UARTE_BAUDRATE_230400;
	case 250000:  return NRF_UARTE_BAUDRATE_250000;
	case 460800:  return NRF_UARTE_BAUDRATE_460800;
	case 921600:  return NRF_UARTE_BAUDRATE_921600;
	case 1000000: return NRF_UARTE_BAUDRATE_1000000;
	default:
		LOG_WRN("Unsupported baud rate %u, falling back to 115200", baud);
		return NRF_UARTE_BAUDRATE_115200;
	}
}

static int w1_serial_reset_bus(const struct device *dev)
{
	const struct w1_serial_config *cfg = dev->config;
	struct w1_serial_data *data = dev->data;
	const uint8_t reset_byte_tx =
		data->overdrive_active ? W1_SERIAL_OD_RESET_BYTE : W1_SERIAL_STD_RESET_BYTE;
	uint8_t reset_byte_rx = 0;
	/* reset uses 115200/9600=12 slower baudrate,
	 * adjust timeout accordingly, also valid for overdrive speed.
	 */
	const uint32_t reset_timeout = CONFIG_NRF_W1_SERIAL_BIT_TIMEOUT * 12;

	nrf_uarte_baudrate_set(data->inst.p_reg, data->overdrive_active ?
		w1_serial_baudrate_from_hz(cfg->od_reset_baud) : NRF_UARTE_BAUDRATE_9600);

	if (serial_tx_rx(dev, &reset_byte_tx, &reset_byte_rx, 1, reset_timeout) < 0) {
		LOG_ERR("tx_rx_error reset_present");
		return -EIO;
	}

	nrf_uarte_baudrate_set(data->inst.p_reg, data->overdrive_active ?
		w1_serial_baudrate_from_hz(cfg->od_data_baud) : NRF_UARTE_BAUDRATE_115200);

	/* At least 1 device is present on bus if reset_byte_rx is different
	 * from reset_byte_tx (which varies depending on standard or overdrive mode).
	 * Bus is probably shorted if reset_byte_rx is 0x00.
	 */
	return (reset_byte_rx != reset_byte_tx) && (reset_byte_rx != 0x00);
}

static int w1_serial_read_bit(const struct device *dev)
{
	uint8_t tx_bit = W1_SERIAL_READ_REQ_BYTE;
	uint8_t rx_bit;

	if (serial_tx_rx(dev, &tx_bit, &rx_bit, 1, CONFIG_NRF_W1_SERIAL_BIT_TIMEOUT) != 0) {
		return -EIO;
	};

	return rx_bit == W1_SERIAL_READ_REQ_BYTE;
}

static int w1_serial_write_bit(const struct device *dev, const bool bit)
{
	uint8_t tx_bit;
	uint8_t rx_bit;

	tx_bit = bit ? W1_SERIAL_BIT_1 : W1_SERIAL_BIT_0;

	if (serial_tx_rx(dev, &tx_bit, &rx_bit, 1, CONFIG_NRF_W1_SERIAL_BIT_TIMEOUT) != 0) {
		return -EIO;
	}
	return 0;
}

static int w1_serial_read_byte(const struct device *dev)
{
	uint8_t tx_byte = 0xFF;
	uint8_t rx_byte;

	if (serial_tx_rx_byte(dev, tx_byte, &rx_byte) != 0) {
		return -EIO;
	}

	return rx_byte;
}

static int w1_serial_write_byte(const struct device *dev, const uint8_t byte)
{
	uint8_t rx_byte;

	return serial_tx_rx_byte(dev, byte, &rx_byte);
}

static int w1_serial_configure(const struct device *dev, enum w1_settings_type type, uint32_t value)
{
	const struct w1_serial_config *cfg = dev->config;
	struct w1_serial_data *data = dev->data;
	int ret = 0;
	bool temp;

	switch (type) {
	case W1_SETTING_SPEED:
		temp = (bool)value;
		if (temp == data->overdrive_active) {
			break;
		}

		data->overdrive_active = temp;
		nrf_uarte_baudrate_set(data->inst.p_reg, data->overdrive_active ?
			w1_serial_baudrate_from_hz(cfg->od_data_baud) : NRF_UARTE_BAUDRATE_115200);
		break;
	default:
		ret = -ENOTSUP;
	}
	return ret;
}
#if (!CONFIG_W1_NRF_SERIAL_TIME_CRITICAL)
static void w1_uarte_event_handler(nrfx_uarte_event_t const *p_event, void *p_context)
{
    struct w1_serial_data *data = p_context;

    switch (p_event->type){
        case NRFX_UARTE_EVT_TX_DONE:
            k_sem_give(&data->tx_sem);
            break;

        case NRFX_UARTE_EVT_RX_DONE:
            k_sem_give(&data->rx_sem);
            break;

        case NRFX_UARTE_EVT_RX_BUF_REQUEST:
            nrfx_uarte_rx_buffer_set(&data->inst, &data->rx_byte, 1);
            break;

        case NRFX_UARTE_EVT_ERROR:
            k_sem_give(&data->rx_sem);
            break;

        default:
            break;
    }
}
#endif
static int w1_serial_init(const struct device *dev)
{
	const struct w1_serial_config *cfg = dev->config;
	struct w1_serial_data *data = dev->data;
    int ret = 0;

    #if(!CONFIG_W1_NRF_SERIAL_TIME_CRITICAL)
    k_sem_init(&data->tx_sem, 0, 1);
    k_sem_init(&data->rx_sem, 0, 1);
    #endif

    nrfx_uarte_config_t config = NRFX_UARTE_DEFAULT_CONFIG(NRF_UARTE_PSEL_DISCONNECTED,
                                                           NRF_UARTE_PSEL_DISCONNECTED);

    config.skip_gpio_cfg = true;
    config.skip_psel_cfg = true;

    config.p_context = data;

    ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
    if (ret != 0) {
        LOG_ERR("Failed to apply pin control state");
        return ret;
    }
    #if(CONFIG_W1_NRF_SERIAL_TIME_CRITICAL)
    if (nrfx_uarte_init(&data->inst, &config, NULL) != 0) {
        LOG_ERR("Failed to initialize uarte");
        return -EIO;
    }

    if (nrfx_uarte_rx_buffer_set(&data->inst, &data->rx_byte, 1) != 0) {
        LOG_ERR("Failed to set rx buffer");
        return -EIO;
    }
    if (nrfx_uarte_rx_enable(&data->inst, 0) != 0) {
        LOG_ERR("Failed to enable uarte RX");
        return -EIO;
    }
    #else
    if (nrfx_uarte_init(&data->inst, &config, w1_uarte_event_handler) != 0) {
        LOG_ERR("Failed to initialize uarte");
        return -EIO;
    }

    cfg->irq_config_func();

    if (nrfx_uarte_rx_enable(&data->inst, 0) != 0) {
        LOG_ERR("Failed to enable uarte RX");
        return -EIO;
    }
    #endif

	data->overdrive_active = false;

	LOG_DBG("w1-serial initialized, with %d slave devices", cfg->master_config.slave_count);
	return 0;
}

static DEVICE_API(w1, w1_serial_driver_api) = {
	.reset_bus = w1_serial_reset_bus,
	.read_bit = w1_serial_read_bit,
	.write_bit = w1_serial_write_bit,
	.read_byte = w1_serial_read_byte,
	.write_byte = w1_serial_write_byte,
	.configure = w1_serial_configure,
};

#define NRF_W1_SERIAL_INIT(n)       \
    PINCTRL_DT_INST_DEFINE(n);                                                          \
    static struct w1_serial_data w1_serial_data_##n = {                                 \
        .inst = NRFX_UARTE_INSTANCE(DT_INST_REG_ADDR(n)),                               \
    };                                                                                  \
    static void w1_uarte_isr_##n(void)                                                  \
    {                                                                                   \
        nrfx_uarte_irq_handler(&w1_serial_data_##n.inst);                               \
    }                                                                                   \
    static void w1_uarte_irq_connect_##n(void)                                          \
    {                                                                                   \
        IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),                          \
                    w1_uarte_isr_##n, NULL, 0);                                         \
        irq_enable(DT_INST_IRQN(n));                                                     \
    }                                                                                   \
    static const struct w1_serial_config w1_serial_cfg_##n = {                         \
        .master_config.slave_count = W1_INST_SLAVE_COUNT(n),                           \
        .od_data_baud = DT_INST_PROP(n, overdrive_data_baud),                          \
        .od_reset_baud = DT_INST_PROP(n, overdrive_reset_baud),                        \
        .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                     \
        .irq_config_func = w1_uarte_irq_connect_##n,                                   \
    };                                                                                 \
    DEVICE_DT_INST_DEFINE(n, &w1_serial_init, NULL, &w1_serial_data_##n,               \
			      &w1_serial_cfg_##n, POST_KERNEL, CONFIG_W1_INIT_PRIORITY, \
			      &w1_serial_driver_api);
DT_INST_FOREACH_STATUS_OKAY(NRF_W1_SERIAL_INIT)