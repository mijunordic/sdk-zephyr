#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/w1_sensor.h>
#include <zephyr/drivers/w1.h>

#define MAX_SENSORS 8

/* Context passed to the search callback */
struct scan_ctx {
	struct w1_rom roms[MAX_SENSORS];
	size_t count;
};

/* Called once per discovered device */
static void search_cb(struct w1_rom rom, void *user_data)
{
	struct scan_ctx *ctx = user_data;

	if (ctx->count < MAX_SENSORS) {
		ctx->roms[ctx->count] = rom;

		printk("Found #%d: %02x%02x%02x%02x%02x%02x%02x%02x\n",
		       (int)ctx->count,
		       rom.family, rom.serial[0], rom.serial[1], rom.serial[2],
		       rom.serial[3], rom.serial[4], rom.serial[5], rom.crc);

		ctx->count++;
	}
}

/* Give a sensor device its ROM so the driver uses Match ROM */
static void set_rom(const struct device *dev, struct w1_rom rom)
{
	struct sensor_value val;

	w1_rom_to_sensor_value(&rom, &val);
	sensor_attr_set(dev, SENSOR_CHAN_ALL, SENSOR_ATTR_W1_ROM, &val);
}

int main(void)
{
	const struct device *bus = DEVICE_DT_GET(DT_NODELABEL(w1_uart));
	const struct device *a   = DEVICE_DT_GET(DT_NODELABEL(ds18b20_a));
	const struct device *b   = DEVICE_DT_GET(DT_NODELABEL(ds18b20_b));

	if (!device_is_ready(bus) || !device_is_ready(a) || !device_is_ready(b)) {
		printk("device(s) not ready\n");
		return 0;
	}

	/* Discover every device on the shared bus */
	struct scan_ctx ctx = { .count = 0 };
	int n = w1_search_rom(bus, search_cb, &ctx);

	printk("Total devices found: %d\n", n);

	if (ctx.count < 2) {
		printk("Expected 2 sensors, found %d\n", (int)ctx.count);
		return 0;
	}

	/* Assign discovered ROMs to the two sensor devices */
	set_rom(a, ctx.roms[0]);
	set_rom(b, ctx.roms[1]);


	while (1) {
		struct sensor_value ta, tb;
		int ra, rb;

		ra = sensor_sample_fetch(a);
		if (ra == 0) {
			sensor_channel_get(a, SENSOR_CHAN_AMBIENT_TEMP, &ta);
		}

		rb = sensor_sample_fetch(b);
		if (rb == 0) {
			sensor_channel_get(b, SENSOR_CHAN_AMBIENT_TEMP, &tb);
		}

		if (ra) {
			printk("A fetch failed: %d\n", ra);
		} else {
			printk("A: %d.%06d\n", ta.val1, ta.val2);
		}
		if (rb) {
			printk("B fetch failed: %d\n", rb);
		} else {
			printk("B: %d.%06d\n", tb.val1, tb.val2);
		}

		k_sleep(K_SECONDS(2));
	}

	return 0;
}