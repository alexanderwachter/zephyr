/*
 * Copyright (c) 2021 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <devicetree.h>
#include <sys/printk.h>

void main(void)
{
	const struct device *dev;
	struct sensor_value voltage, current, power, charge;

	dev = DEVICE_DT_GET(DT_NODELABEL(monitor));
	if (!device_is_ready(dev)) {
		printk("INA229 not ready");
		return;
	}

	while (1) {
		sensor_sample_fetch(dev);
		sensor_channel_get(dev, SENSOR_CHAN_VOLTAGE, &voltage);
		sensor_channel_get(dev, SENSOR_CHAN_CURRENT, &current);
		sensor_channel_get(dev, SENSOR_CHAN_POWER, &power);
		sensor_channel_get(dev, SENSOR_CHAN_CHARGE, &charge);
		printk("Voltage: %d.%06d V; Current: %d.%06d A; "
		       "Power: %d.%06d W; Charge: %d.%06dWs\n",
			voltage.val1, voltage.val2, current.val1, current.val2,
			power.val1, power.val2, charge.val1, charge.val2);

		k_sleep(K_MSEC(1000));
	}
}
