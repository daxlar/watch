/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

#include <sensor/drv2605/drv2605.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   500


#define LED0_NODE DT_ALIAS(led0)
#define SW0_NODE DT_ALIAS(sw0)
#define SW1_NODE DT_ALIAS(sw1)

#define BUTTON_THREAD_STACK_SIZE 500
#define BUTTON_THREAD_PRIORITY 5

K_THREAD_STACK_DEFINE(button_thread_stack_area, BUTTON_THREAD_STACK_SIZE);
struct k_thread button_thread_data;

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec sw0 = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
static const struct gpio_dt_spec sw1 = GPIO_DT_SPEC_GET(SW1_NODE, gpios);

static struct gpio_callback sw0_cb_data;
static struct gpio_callback sw1_cb_data;

static k_tid_t button_task_tid;

bool button_thread_active = false;
bool sw0_value = false;
bool sw1_value = false;


/// ADD DEBOUNCING HERE
void button_task(void* p1, void* p2, void* p3) {
	printk("i'm here at the button task");

	if(sw0_value) {
		sw0_value = false;
		gpio_pin_interrupt_configure_dt(&sw0, GPIO_INT_EDGE_TO_ACTIVE);
		printk("re-enabling sw0");
	}

	if(sw1_value) {
		sw1_value = false;
		gpio_pin_interrupt_configure_dt(&sw1, GPIO_INT_EDGE_TO_ACTIVE);
		printk("re-enabling sw1");
	}

	/// LOGIC
	/// check if both buttons have been held for 2 seconds
	/// if they have, enter time settings
	/// one button to increase, one button to decrease, press both to confirm the hours
	/// repeat this process for minutes

	button_thread_active = false;
}

void sw0_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
	printk("Button pressed at sw0: %" PRIu32 "\n", k_cycle_get_32());
	gpio_pin_interrupt_configure_dt(&sw0, GPIO_INT_DISABLE);

	if(!button_thread_active) {
		button_task_tid = k_thread_create(&button_thread_data, button_thread_stack_area,
										K_THREAD_STACK_SIZEOF(button_thread_stack_area),
										button_task,
										NULL, NULL, NULL,
										BUTTON_THREAD_PRIORITY, 0, K_NO_WAIT);
		button_thread_active = true;
	}
	sw0_value = true;
}

void sw1_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
	printk("Button pressed at sw1: %" PRIu32 "\n", k_cycle_get_32());
	gpio_pin_interrupt_configure_dt(&sw1, GPIO_INT_DISABLE);
	
	if(!button_thread_active) {
		button_task_tid = k_thread_create(&button_thread_data, button_thread_stack_area,
										K_THREAD_STACK_SIZEOF(button_thread_stack_area),
										button_task,
										NULL, NULL, NULL,
										BUTTON_THREAD_PRIORITY, 0, K_NO_WAIT);
		button_thread_active = true;
	}
	sw1_value = true;
}

int main(void)
{       
    const struct device* drv2605_dev = DEVICE_DT_GET_ANY(ti_drv2605);

	if (!device_is_ready(led.port)) {
        printk("failure at initializing led device\r\n");
		return 1;
	}

	int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
        printk("failure at configuring gpio device\r\n");
		return 1;
	}

	ret = gpio_pin_configure_dt(&sw0, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, sw0.port->name, sw0.pin);
		return ret;
	}

	ret = gpio_pin_configure_dt(&sw1, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, sw1.port->name, sw1.pin);
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&sw0, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, sw0.port->name, sw0.pin);
		return ret;
	}

	gpio_init_callback(&sw0_cb_data, sw0_pressed, BIT(sw0.pin));
	gpio_add_callback(sw0.port, &sw0_cb_data);

	ret = gpio_pin_interrupt_configure_dt(&sw1, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, sw1.port->name, sw1.pin);
		return ret;
	}

	gpio_init_callback(&sw1_cb_data, sw1_pressed, BIT(sw1.pin));
	gpio_add_callback(sw1.port, &sw1_cb_data);

    uint8_t index = 1;
    struct sensor_value sensor = {.val1 = index, .val2 = 0};
	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
            printk("failure at toggling led device\r\n");
			return 1;
		}
		
		int dev_ret = sensor_attr_set(drv2605_dev, 
										EMIT_WAVEFORM_NUM, 
										EMIT_WAVEFORM_START, 
										&sensor);

		if(dev_ret < 0) {
			printk("trouble trigger haptics driver");
		} else {
			//printk("beeping at index %d \r\n", sensor.val1);
            sensor.val1 = (sensor.val1 + 1) % 124;
		}

		k_msleep(SLEEP_TIME_MS);
	}

    return 0;
}