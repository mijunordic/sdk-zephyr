/*
* Copyright (c) 2026, Nordic Semiconductor ASA
*
* SPDX-License-Identifier: Apache-2.0
*/
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include "my_ds18b20.c"
#include <zephyr/ztest.h>     

const struct device * bus;

static struct ds18b20_config cfg = {
    .resolution = 12,
    .family     = 0x28,
};
static struct ds18b20_data data = {0};  

static void *ds18b20_setup(void){
    bus = DEVICE_DT_GET(DT_NODELABEL(w1_uart));

    zassert_true(device_is_ready(bus), "\nError: Device \"%s\" is not ready.\n", bus->name);
}

ZTEST(test_w1, single_read_test)
{
	// const struct device *bus = get_bus_device();
	zassert_not_null(bus, "Error: Device \"%s\" is not available.\n", bus->name); 

    int status;

    if (!data.lazy_loaded) {
        status = ds18b20_configure(bus, &cfg, &data);
        zassert_true(status >= 0, "Error: I/O error.\n");
        data.lazy_loaded = true;
    }

    status = ds18b20_temperature_convert(bus, &data);
    zassert_true(status >= 0, "W1 convert error: %d\n", status);
   
    k_msleep(measure_wait_ms(&cfg));

    struct sensor_val temp;

    status = ds18b20_read_scratchpad(bus, &data, &data.scratchpad);
    zassert_true(status >= 0, "read_scratchpad() failed: %d\n", status);
   
    ds18b20_temperature_from_raw(&cfg, (uint8_t *)&data.scratchpad.temp, 
                                 &temp);

    zassert_within(temp.val1, 25, 5, "Error: Read temperature (%d.%06d) is out of range", temp.val1, temp.val2);
}

ZTEST(test_w1, multiple_read_test)
{
    // const struct device *bus = get_bus_device();
	zassert_not_null(bus, "Error: Device \"%s\" is not available.\n", bus->name); 

    int status;

    if (!data.lazy_loaded) {
        status = ds18b20_configure(bus, &cfg, &data);
        zassert_true(status >= 0, "Error: I/O error.\n");
        data.lazy_loaded = true;
    }
    for (int i = 0; i < 5; i++)
    {
        status = ds18b20_temperature_convert(bus, &data);
        zassert_true(status >= 0, "W1 convert error: %d\n", status);
    
        k_msleep(measure_wait_ms(&cfg));

        struct sensor_val temp;

        status = ds18b20_read_scratchpad(bus, &data, &data.scratchpad);
        zassert_true(status >= 0, "read_scratchpad() failed: %d\n", status);
    
        ds18b20_temperature_from_raw(&cfg, (uint8_t *)&data.scratchpad.temp, &temp);

        zassert_within(temp.val1, 25, 5, "Error: Read temperature (%d.%06d) is out of range", 
                       temp.val1, temp.val2);
    }
}

ZTEST_SUITE(test_w1, NULL, ds18b20_setup, NULL, NULL, NULL);