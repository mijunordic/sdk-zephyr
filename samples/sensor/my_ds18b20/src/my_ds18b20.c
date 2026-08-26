#include <stdbool.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include "zephyr/drivers/w1.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/ztest.h>

#include <hal/nrf_gpio.h>
#include <zephyr/debug/cpu_load.h>

LOG_MODULE_REGISTER(MY_DS18B20, CONFIG_SENSOR_LOG_LEVEL);

#define DS18B20_CMD_CONVERT_T         0x44
#define DS18B20_CMD_WRITE_SCRATCHPAD  0x4E
#define DS18B20_CMD_READ_SCRATCHPAD   0xBE
#define DS18B20_CMD_COPY_SCRATCHPAD   0x48
#define DS18B20_CMD_RECALL_EEPROM     0xB8
#define DS18B20_CMD_READ_POWER_SUPPLY 0xB4

/* resolution is set using bit 5 and 6 of configuration register
 * macro only valid for values 9 to 12
 */
#define DS18B20_RESOLUTION_POS		5
#define DS18B20_RESOLUTION_MASK		(BIT_MASK(2) << DS18B20_RESOLUTION_POS)
/* convert resolution in bits to scratchpad config format */
#define DS18B20_RESOLUTION(res)		((res - 9) << DS18B20_RESOLUTION_POS)
/* convert resolution in bits to array index (for resolution specific elements) */
#define DS18B20_RESOLUTION_INDEX(res)	(res - 9)

#define DS18B20_FAMILYCODE 0x28
#define DS18S20_FAMILYCODE 0x10

/*
 * If TESTS_INCLUDED equals 1 code includes tests, else it does not
 */

#define TESTS_INCLUDED 1
/*
 * If TEST_MODE equals 1 it's a time test, if equals 0 it's a CPU LOAD test
 */

#define TEST_MODE 1

#if (TESTS_INCLUDED)
#if (TEST_MODE)
#define PIN1_14 NRF_GPIO_PIN_MAP(1, 14)
#define PIN1_12 NRF_GPIO_PIN_MAP(1, 12)
#define PIN3_02 NRF_GPIO_PIN_MAP(3, 2)
#define PIN3_00 NRF_GPIO_PIN_MAP(3, 0)
#define PIN3_06 NRF_GPIO_PIN_MAP(3, 6)
#define PIN3_10 NRF_GPIO_PIN_MAP(3, 10)

void ds18b20_test_gpio_pins_setup(void)
{
#if (TEST_MODE)
	nrf_gpio_cfg_output(PIN1_12);
	nrf_gpio_cfg_output(PIN1_14);
	nrf_gpio_cfg_output(PIN3_02);
	nrf_gpio_cfg_output(PIN3_00);
	nrf_gpio_cfg_output(PIN3_06);
	nrf_gpio_cfg_output(PIN3_10);
#endif
}
#else
static int load;
#endif
#endif


enum chip_type {type_ds18b20, type_ds18s20};

struct ds18b20_scratchpad {
	int16_t temp;
	uint8_t alarm_temp_high;
	uint8_t alarm_temp_low;
	uint8_t config;
	uint8_t res[3];
	uint8_t crc;
} __packed;

struct ds18b20_config {
	const struct device *bus;
	uint8_t family;
	uint8_t resolution;
	enum chip_type chip;
};

struct ds18b20_data {
	struct w1_slave_config config;
	struct ds18b20_scratchpad scratchpad;
	bool lazy_loaded;
};

struct sensor_val {
	int32_t val1;
	int32_t val2;
};

int ds18b20_configure(const struct device *bus,
		      		  struct ds18b20_config *cfg,
		      		  struct ds18b20_data  *data);
int ds18b20_read_scratchpad(const struct device *bus,
							struct ds18b20_data *data,
							struct ds18b20_scratchpad *scratchpad);

/* measure wait time for 9-bit, 10-bit, 11-bit, 12-bit resolution respectively */
static const uint16_t measure_wait_ds18b20_ms[4] = { 94, 188, 376, 750 };

/* ds18s20 always needs 750ms */
static const uint16_t measure_wait_ds18s20_ms = { 750 };

void ds18b20_temperature_from_raw(const struct ds18b20_config *cfg,
								  uint8_t *temp_raw,
								  struct sensor_val *val)
{
	int16_t temp = sys_get_le16 (temp_raw);

	if (cfg->chip == type_ds18s20) {
		val->val1 = temp / 2;
		val->val2 = (temp % 2) * 500000;
	} else {
		val->val1 = temp / 16;
		val->val2 = (temp % 16) * 1000000 / 16;
	}
}

static inline bool slave_responded(uint8_t *rx_buf, size_t len)
{
	uint8_t cmp_byte = 0xff;

	for (int i = 0; i < len; i++) {
		cmp_byte &= rx_buf[i];
	}

	return (cmp_byte == 0xff) ? false : true;
}

/*
 * Write scratch pad, read back, then copy to eeprom
 */
static int ds18b20_write_scratchpad(const struct device *bus,
									struct ds18b20_data *data,
									struct ds18b20_scratchpad scratchpad)
{

	int ret;
	uint8_t sp_data[4] = {
		DS18B20_CMD_WRITE_SCRATCHPAD,
		scratchpad.alarm_temp_high,
		scratchpad.alarm_temp_low,
		scratchpad.config
	};
	#if(TESTS_INCLUDED)
	#if(TEST_MODE)
	nrf_gpio_pin_set(PIN1_12);
	#else
	printk("Inside ds18b20_write_scratchpad()\n\r");
	(void)cpu_load_get(true);
	#endif
	#endif

	ret = w1_write_read(bus, &data->config, sp_data, sizeof(sp_data), NULL, 0);

	#if(TESTS_INCLUDED)
	#if(TEST_MODE)
	nrf_gpio_pin_clear(PIN1_12);
	#else
	load = cpu_load_get(true);
	printk("w1_write_read in ds18b20_write_scratchpad load: %d \n\r", load);
	#endif
	#endif

	if (ret != 0) {
		return ret;
	}

	#if(TESTS_INCLUDED)
	#if(TEST_MODE)
	nrf_gpio_pin_set(PIN1_14);
	#else
	(void)cpu_load_get(true);
	#endif
	#endif

	ret = ds18b20_read_scratchpad(bus, data, &scratchpad);

	#if(TESTS_INCLUDED)
	#if(TEST_MODE)
	nrf_gpio_pin_clear(PIN1_14);
	#else
	load = cpu_load_get(true);
	printk("ds18b20_read_scratchpad load: %d \n\r", load);
	#endif
	#endif

	if (ret != 0) {
		return ret;
	}

	if ((sp_data[3] & DS18B20_RESOLUTION_MASK) !=
	    (scratchpad.config & DS18B20_RESOLUTION_MASK)) {
		return -EIO;
	}


	return 0;
}

int ds18b20_read_scratchpad(const struct device *bus,
							struct ds18b20_data *data,
							struct ds18b20_scratchpad *scratchpad)
{
	int ret;
	uint8_t cmd = DS18B20_CMD_READ_SCRATCHPAD;
	uint8_t crc;

	memset(scratchpad, 0, sizeof(*scratchpad));

	#if(TESTS_INCLUDED)
	#if(TEST_MODE)
	nrf_gpio_pin_set(PIN3_00);
	#else
	printk("Inside ds18b20_read_scratchpad()\n\r");
	(void)cpu_load_get(true);
	#endif
	#endif

	ret = w1_write_read(bus, &data->config, &cmd, 1,
			     (uint8_t *)scratchpad, sizeof(*scratchpad));

	#if(TESTS_INCLUDED)
	#if(TEST_MODE)
	nrf_gpio_pin_clear(PIN3_00);
	#else
	load = cpu_load_get(true);
	printk("w1_write_read() in ds18b20_read_scratchpad() load: %d \n\r", load);
	#endif
	#endif

	if (ret != 0) {
		return ret;
	}

	if (!slave_responded((uint8_t *)scratchpad, sizeof(*scratchpad))) {
		LOG_WRN("Slave not reachable");
		return -ENODEV;
	}

	crc = w1_crc8((uint8_t *)scratchpad, sizeof(*scratchpad) - 1);

	if (crc != scratchpad->crc) {
		LOG_WRN("CRC does not match");
		return -EIO;
	}

	return 0;
}

/* Starts sensor temperature conversion without waiting for completion. */
int ds18b20_temperature_convert(const struct device *bus,
								struct ds18b20_data *data)
{
	int ret;

	(void)w1_lock_bus(bus);
	ret = w1_reset_select(bus, &data->config);
	if (ret != 0) {
		goto out;
	}
	ret = w1_write_byte(bus, DS18B20_CMD_CONVERT_T);
out:
	(void)w1_unlock_bus(bus);
	return ret;
}

/*
 * Write resolution into configuration struct,
 * but don't write it to the sensor yet.
 */
static void ds18b20_set_resolution(struct ds18b20_data *data, uint8_t resolution)
{
	data->scratchpad.config &= ~DS18B20_RESOLUTION_MASK;
	data->scratchpad.config |= DS18B20_RESOLUTION(resolution);
}

uint16_t measure_wait_ms(const struct ds18b20_config *cfg)
{
	if (cfg->chip == type_ds18s20) {
		return measure_wait_ds18s20_ms;
	}

	return measure_wait_ds18b20_ms[DS18B20_RESOLUTION_INDEX(cfg->resolution)];
}

int ds18b20_configure(const struct device *bus,
		      struct ds18b20_config *cfg,
		      struct ds18b20_data  *data)
{
	int ret;

	#if(TESTS_INCLUDED)
	#if(TEST_MODE)
	nrf_gpio_pin_set(PIN3_06);
	#else
	(void)cpu_load_get(true);
	#endif
	#endif

	int stat = w1_reset_bus(bus);

	#if(TESTS_INCLUDED)
	#if(TEST_MODE)
	nrf_gpio_pin_clear(PIN3_06);
	#else
	load = cpu_load_get(true);
	printk("w1_reset_bus() in ds18b20_configure() load: %d \n\r", load);
	#endif
	#endif

	zassert_false(stat <= 0, "No 1-Wire slaves connected\n");

	/* In single-drop configurations the ROM can be read from the device */
	if (w1_get_slave_count(bus) == 1) {
		if (w1_rom_to_uint64(&data->config.rom) == 0ULL) {
			#if(TESTS_INCLUDED)
			#if(TEST_MODE)
			nrf_gpio_pin_set(PIN3_10);
			#else
			(void)cpu_load_get(true);
			#endif
			#endif

			(void)w1_read_rom(bus, &data->config.rom);

			#if(TESTS_INCLUDED)
			#if(TEST_MODE)
			nrf_gpio_pin_clear(PIN3_10);
			#else
			load = cpu_load_get(true);
			printk("w1_read_rom in ds18b20_configure() load: %d \n\r", load);
			#endif
			#endif
		}
	}
	zassert_false(CONFIG_COUNT_CONNECTED_DEVICES > 1ULL,
		"ROM required, because multiple slaves are on the bus\n");

	/* write default configuration */
	if (cfg->chip == type_ds18b20) {
		#if(TESTS_INCLUDED)
		#if(!TEST_MODE)
		(void)cpu_load_get(true);
		#endif
		#endif

		ds18b20_set_resolution(data, cfg->resolution);

		#if(TESTS_INCLUDED)
		#if(TEST_MODE)
		nrf_gpio_pin_set(PIN3_02);
		#else
		load = cpu_load_get(true);
		printk("ds18b20_set_resolution() load: %d \n\r", load);
		(void)cpu_load_get(true);
		#endif
		#endif

		ret = ds18b20_write_scratchpad(bus, data, data->scratchpad);

		#if(TESTS_INCLUDED)
		#if(TEST_MODE)
		nrf_gpio_pin_clear(PIN3_02);
		#else
		load = cpu_load_get(true);
		printk("ds18b20_write_scratchpad() load: %d \n\r", load);
		#endif
		#endif

		return ret;
	}

	LOG_DBG("Init: ROM=%016llx", w1_rom_to_uint64(&data->config.rom));

	return 0;
}
