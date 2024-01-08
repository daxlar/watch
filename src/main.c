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

K_SEM_DEFINE(a_switch_was_pressed, 0, 1);
bool sw0_was_pressed = false;
bool sw1_was_pressed = false;
uint32_t sw0_pressed_time_stamp = 0;
uint32_t sw1_pressed_time_stamp = 0;


void sw0_pressed_task(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
	printk("Button pressed at sw0: %" PRIu32 "\n", k_cycle_get_32());
	gpio_pin_interrupt_configure_dt(&sw0, GPIO_INT_DISABLE);
	k_sem_give(&a_switch_was_pressed);
	sw0_was_pressed = true;
	sw0_pressed_time_stamp = k_cyc_to_ms_ceil32(k_cycle_get_32());
}

void sw1_pressed_task(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
	printk("Button pressed at sw1: %" PRIu32 "\n", k_cycle_get_32());
	gpio_pin_interrupt_configure_dt(&sw1, GPIO_INT_DISABLE);
	k_sem_give(&a_switch_was_pressed);
	sw1_was_pressed = true;
	sw1_pressed_time_stamp = k_cyc_to_ms_ceil32(k_cycle_get_32());
}

void initialize_switches() {
	int ret;

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

	gpio_init_callback(&sw0_cb_data, sw0_pressed_task, BIT(sw0.pin));
	gpio_add_callback(sw0.port, &sw0_cb_data);

	ret = gpio_pin_interrupt_configure_dt(&sw1, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, sw1.port->name, sw1.pin);
		return ret;
	}

	gpio_init_callback(&sw1_cb_data, sw1_pressed_task, BIT(sw1.pin));
	gpio_add_callback(sw1.port, &sw1_cb_data);

	return;
}

/// ADD DEBOUNCING HERE
/// any button to enter task
/// once task is entered, how do I differentiate between the different buttons ?
/// if I use a semaphore for each button
void button_task(void* p1, void* p2, void* p3) {

	printk("button_task is active \r\n");

	enum task_state_t {
		WAIT_FOR_DOUBLE,
		WAIT_FOR_ADJUSTMENTS,
	};

	#define SECOND_PRESS_BUFFER_MS 50

	enum task_state_t state = WAIT_FOR_DOUBLE;

	while(true) {
		switch(state) {
			case WAIT_FOR_DOUBLE: {

				k_sem_take(&a_switch_was_pressed, K_FOREVER);

				#define DOUBLE_PRESS_DURATION_MS 2000

				volatile uint32_t port_raw_value = 0;
				volatile uint32_t sw0_raw_value = 0;
				volatile uint32_t sw1_raw_value = 0;

				uint32_t cur_time = k_cyc_to_ms_ceil32(k_cycle_get_32());
				printk("cur_time: %d \r\n", cur_time);
				uint32_t buffer_end_time = cur_time + SECOND_PRESS_BUFFER_MS;

				while(buffer_end_time > cur_time) {
					cur_time = k_cyc_to_ms_ceil32(k_cycle_get_32());
				}

				uint32_t end_time = cur_time + DOUBLE_PRESS_DURATION_MS;
				bool failed_hold = false;
				
				while((!sw0_raw_value && !sw1_raw_value) && (cur_time < end_time)) {
					cur_time = k_cyc_to_ms_ceil32(k_cycle_get_32());
					gpio_port_get_raw(sw0.port, &port_raw_value);
					sw0_raw_value = (port_raw_value >> 11) & 0x1;
					sw1_raw_value = (port_raw_value >> 12) & 0x1;
					if(sw0_raw_value || sw1_raw_value) {
						printk("failed the hold");
						failed_hold = true;
						break;
					}
				}

				if(!failed_hold) {
					state = WAIT_FOR_ADJUSTMENTS;
					printk("got the hold, moving to adjustments \r\n");
				}

				sw0_was_pressed = false;
				sw1_was_pressed = false;
				gpio_pin_interrupt_configure_dt(&sw0, GPIO_INT_EDGE_TO_ACTIVE);
				gpio_pin_interrupt_configure_dt(&sw1, GPIO_INT_EDGE_TO_ACTIVE);
				k_sem_reset(&a_switch_was_pressed);
				
				break;
			}

			case WAIT_FOR_ADJUSTMENTS : {
				
				#define DURATION_BETWEEN_PRESSES_MS 2000

				/// get the presses needed for hours

				/// what needs to happen here ?
				/// either click the top button to increase time or click the bottom button to decrease the time
				/// between click there MUST be feedback to the user that a click happened
				///		this means that there has to be a delay between the clicks
				///		click -> feedback -> click
				///		if there is another click of any type during the feedback, it is ignored
				/// if both buttons are clicked, then we move onto the minutes ( hours -> minutes)

				uint8_t hour_count = 0;

				while(k_sem_take(&a_switch_was_pressed, K_MSEC(DURATION_BETWEEN_PRESSES_MS)) == 0) {
					
					uint32_t cur_time = k_cyc_to_ms_ceil32(k_cycle_get_32());
					uint32_t buffer_end_time = cur_time + SECOND_PRESS_BUFFER_MS;

					while(buffer_end_time > cur_time) {
						cur_time = k_cyc_to_ms_ceil32(k_cycle_get_32());
					}

					printk("hour switch delay finished \r\n");

					if(sw0_was_pressed && sw1_was_pressed) {
						/// reset the switches 
						sw0_was_pressed = false;
						sw1_was_pressed = false;
						gpio_pin_interrupt_configure_dt(&sw0, GPIO_INT_EDGE_TO_ACTIVE);
						gpio_pin_interrupt_configure_dt(&sw1, GPIO_INT_EDGE_TO_ACTIVE);

						/// check to see if the switches were pressed together
						int64_t sw0_pressed_time_stamp_64 = sw0_pressed_time_stamp;
						int64_t sw1_pressed_time_stamp_64 = sw1_pressed_time_stamp;
						if(abs(sw0_pressed_time_stamp_64 - sw1_pressed_time_stamp_64) < SECOND_PRESS_BUFFER_MS) {
							// done with hours, move on
							printk("hour double tap time diff: %d \r\n", abs(sw0_pressed_time_stamp_64 - sw1_pressed_time_stamp_64));
							break;
						} 
					} else if(sw0_was_pressed) {
						/// reset the switch
						sw0_was_pressed = false;
						gpio_pin_interrupt_configure_dt(&sw0, GPIO_INT_EDGE_TO_ACTIVE);

						/// adjust the hours
						hour_count = (hour_count + 1) % 13;
						printk("hour count: %d \r\n", hour_count);
					} else {
						/// reset the switch
						sw1_was_pressed = false;
						gpio_pin_interrupt_configure_dt(&sw1, GPIO_INT_EDGE_TO_ACTIVE);

						/// adjust the hours
						if(hour_count == 0) {
							hour_count = 12;
						} else {
							hour_count--;
						}
						printk("hour count: %d \r\n", hour_count);
					}
				}
				
				/// get the presses needed for minutes
				uint8_t minutes_count = 0;

				while(k_sem_take(&a_switch_was_pressed, K_MSEC(DURATION_BETWEEN_PRESSES_MS)) == 0) {
					
					uint32_t cur_time = k_cyc_to_ms_ceil32(k_cycle_get_32());
					uint32_t buffer_end_time = cur_time + SECOND_PRESS_BUFFER_MS;

					while(buffer_end_time > cur_time) {
						cur_time = k_cyc_to_ms_ceil32(k_cycle_get_32());
					}

					printk("minute delay finished \r\n");

					if(sw0_was_pressed && sw1_was_pressed) {
						/// reset the switches 
						sw0_was_pressed = false;
						sw1_was_pressed = false;
						gpio_pin_interrupt_configure_dt(&sw0, GPIO_INT_EDGE_TO_ACTIVE);
						gpio_pin_interrupt_configure_dt(&sw1, GPIO_INT_EDGE_TO_ACTIVE);

						/// check to see if the switches were pressed together
						int64_t sw0_pressed_time_stamp_64 = sw0_pressed_time_stamp;
						int64_t sw1_pressed_time_stamp_64 = sw1_pressed_time_stamp;
						if(abs(sw0_pressed_time_stamp_64 - sw1_pressed_time_stamp_64) < SECOND_PRESS_BUFFER_MS) {
							// done with minutes, move on
							printk("minute double tap time diff: %d \r\n", abs(sw0_pressed_time_stamp_64 - sw1_pressed_time_stamp_64));
							break;
						} 
					} else if(sw0_was_pressed) {
						/// reset the switch
						sw0_was_pressed = false;
						gpio_pin_interrupt_configure_dt(&sw0, GPIO_INT_EDGE_TO_ACTIVE);

						/// adjust the hours
						minutes_count = (minutes_count + 1) % 60;
						printk("minute count: %d \r\n", minutes_count);
					} else {
						/// reset the switch
						sw1_was_pressed = false;
						gpio_pin_interrupt_configure_dt(&sw1, GPIO_INT_EDGE_TO_ACTIVE);

						/// adjust the hours
						if(minutes_count == 0) {
							minutes_count = 59;
						} else {
							minutes_count--;
						}
						printk("minute count: %d \r\n", minutes_count);
					}
				}

				/// reset task, inputs are done
				sw0_was_pressed = false;
				sw1_was_pressed = false;
				gpio_pin_interrupt_configure_dt(&sw0, GPIO_INT_EDGE_TO_ACTIVE);
				gpio_pin_interrupt_configure_dt(&sw1, GPIO_INT_EDGE_TO_ACTIVE);

				state = WAIT_FOR_DOUBLE;
				break;
			}
		}
	}
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

	button_task_tid = k_thread_create(&button_thread_data, button_thread_stack_area,
										K_THREAD_STACK_SIZEOF(button_thread_stack_area),
										button_task,
										NULL, NULL, NULL,
										BUTTON_THREAD_PRIORITY, 0, K_NO_WAIT);

	initialize_switches();

	printk("switches are initialized \r\n");

    uint8_t index = 1;
    struct sensor_value sensor = {.val1 = index, .val2 = 0};
	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
            printk("failure at toggling led device\r\n");
			return 1;
		}
		/*
		int dev_ret = sensor_attr_set(drv2605_dev, 
									  EMIT_WAVEFORM_NUM, 
									  EMIT_WAVEFORM_START, 
									  &sensor);

		if(dev_ret < 0) {
			printk("trouble trigger haptics driver");
		} else {
            sensor.val1 = (sensor.val1 + 1) % 124;
		}
		*/
		k_msleep(SLEEP_TIME_MS);
	}

    return 0;
}