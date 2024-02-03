/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/timing/timing.h>

#include <sensor/adxl343/adxl343.h>
#include <sensor/drv2605/drv2605.h>
#include <sensor/pca9548/pca9548.h>
#include <sensor/rv8803/rv8803.h>

//#include <nrfx_timer.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   500


#define LED0_NODE DT_ALIAS(led0)
#define SW0_NODE DT_ALIAS(sw0)
#define SW1_NODE DT_ALIAS(sw1)

#define BUTTON_THREAD_STACK_SIZE 500
#define BUTTON_THREAD_PRIORITY 5
#define RTC_THREAD_STACK_SIZE 500
#define RTC_THREAD_PRIORITY 5

K_THREAD_STACK_DEFINE(button_thread_stack_area, BUTTON_THREAD_STACK_SIZE);
struct k_thread button_thread_data;

K_THREAD_STACK_DEFINE(rtc_thread_stack_area, RTC_THREAD_STACK_SIZE);
struct k_thread rtc_thread_data;

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec sw0 = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
static const struct gpio_dt_spec sw1 = GPIO_DT_SPEC_GET(SW1_NODE, gpios);
//static const struct device* adxl343_dev = DEVICE_DT_GET_ANY(adi_adxl343);
static const struct device* gpio0_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
static const struct device* rv8803_dev = DEVICE_DT_GET_ANY(mc_rv8803);

static struct gpio_callback sw0_cb_data;
static struct gpio_callback sw1_cb_data;
//static struct gpio_callback acc_cb_data;
static struct gpio_callback clk_cb_data;

static k_tid_t button_task_tid;
static k_tid_t rtc_task_tid;

K_SEM_DEFINE(a_switch_was_pressed, 0, 1);
bool sw0_was_pressed = false;
bool sw1_was_pressed = false;
uint32_t sw0_pressed_time_stamp = 0;
uint32_t sw1_pressed_time_stamp = 0;

K_SEM_DEFINE(final_time_stamp, 0, 1);

//nrfx_timer_t timer_t_inst = NRFX_TIMER_INSTANCE(0);

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

int initialize_switches() {
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

	return 0;
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

				uint32_t port_raw_value = 0;
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

/*
int initialize_haptic(const struct device* pca9548_dev, const struct device* drv2605_dev, enum pca9548_channel channel_number) {
	int open_channel = sensor_attr_set(pca9548_dev, channel_number, CHANNEL_OPEN, NULL);
	if(open_channel < 0) {
		printk("trouble setting channel i2cmux driver \r\n");
		return -ENODEV;
	}

	struct sensor_value initialization_values = {.val1 = 0, .val2 = 0};
	int trigger_waveform = sensor_attr_set(drv2605_dev, INITIALIZE, INITIALIZE_START, &initialization_values);
	if(trigger_waveform < 0) {
		printk("trouble initializing haptics driver \r\n");
		return -ENODEV;
	}

	return 0;
}

int activate_haptic(const struct device* pca9548_dev, const struct device* drv2605_dev, enum pca9548_channel channel_number) {

	int open_channel = sensor_attr_set(pca9548_dev, channel_number, CHANNEL_OPEN, NULL);
	if(open_channel < 0) {
		printk("trouble setting channel i2cmux driver \r\n");
		return -ENODEV;
	}

	struct sensor_value haptic_waveform_number = {.val1 = 14, .val2 = 0};
	int trigger_waveform = sensor_attr_set(drv2605_dev, EMIT_WAVEFORM, EMIT_WAVEFORM_START, &haptic_waveform_number);
	if(trigger_waveform < 0) {
		printk("trouble triggering haptics driver \r\n");
		return -ENODEV;
	}

	return 0;
}

int initialize_haptics(const struct device* pca9548_dev, const struct device* drv2605_dev) {
	for(uint8_t i = CHANNEL_ZERO; i < CHANNEL_SIX; i = i<<1) {
		printk("at: %d \r\n", i);
		int init_status = initialize_haptic(pca9548_dev, drv2605_dev, i);
		if(init_status < 0) {
			printk("trouble initializing haptics at: %d \r\n", i);
			return -ENODEV;
		}
	}

	return 0;

}

int initialize_accelerometer(const struct device* adxl343_dev) {

	struct sensor_value dummy_data = {.val1 = 0, .val2 = 0};
	return sensor_attr_set(adxl343_dev, ADXL343_INITIALIZE, ADXL343_INITIALIZE_START, &dummy_data);
}

int toggle_clear_int = 0;

void acc_trigger_task(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
	printk("accel double tap trigger! \r\n");
	toggle_clear_int = 1;
}
*/

int set_time(const struct device* rv8803_dev, int hours, int minutes) {

	struct sensor_value time = {.val1 = hours, .val2 = minutes};
	return sensor_attr_set(rv8803_dev, RV8803_SET_TIME, RV8803_SET_TIME_START, &time);
}

int get_time(const struct device* rv8803_dev, uint8_t* hours, uint8_t* minutes) {

	struct sensor_value time;
	int getting_time_error = sensor_channel_get(rv8803_dev, RV8803_GET_TIME, &time);
	if(getting_time_error) {
		printk("trouble getting time \r\n");
		return -ENODEV;
	}

	*hours = time.val1;
	*minutes = time.val2;
	return 0;
}


//timing_t start_time;
//timing_t end_time;

//uint32_t start_time_val;
//uint32_t end_time_val;
volatile bool time_stamp1_got = false;
volatile bool time_stamp2_got = false;

volatile bool found_first_inactive = false;

void clkout_edge_task(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {

	if(found_first_inactive == false) {
		found_first_inactive = true;
		int configure_gpio = gpio_pin_interrupt_configure(gpio0_dev, 4, GPIO_INT_EDGE_TO_ACTIVE);
		if (configure_gpio < 0) {
			printk("Failed to configure active interrupt for clkout pin \r\n");
		} else {
			printk("setting active interrupt \r\n");
		}
	} else if(time_stamp1_got == false) {
		//start_time = timing_counter_get();
		//start_time = k_cyc_to_ns_ceil32(k_cycle_get_32());
		//start_time_val = nrfx_timer_capture(&timer_t_inst, NRF_TIMER_CC_CHANNEL0);
		gpio_pin_toggle(gpio0_dev, 25);
		//printk("time stamp 1: %d \r\n", time_stamp1);
		time_stamp1_got = true;
	} else if(time_stamp2_got == false ){
		//end_time = timing_counter_get();
		//end_time = k_cyc_to_ns_ceil32(k_cycle_get_32());
		//end_time_val = nrfx_timer_capture(&timer_t_inst, NRF_TIMER_CC_CHANNEL1);
		gpio_pin_toggle(gpio0_dev, 25);
		//printk("time stamp 2: %d \r\n", time_stamp2);
		time_stamp2_got = true;
		k_sem_give(&final_time_stamp);
	} else {

	}
}

void initialize_rtc() {

	//timing_init();
	//timing_start();

	// use port 0 pin 24 for CLKOE
	// use port 0 pin 4 for CLKOUT
	struct sensor_value dummy;
	int start_offset_calc = sensor_attr_set(rv8803_dev, RV8803_INITIALIZE, RV8803_INITIALIZE_START, &dummy);
	if(start_offset_calc) {
		printk("having trouble starting offset calc! \r\n");
	}

	int configure_gpio = gpio_pin_configure(gpio0_dev, 24, GPIO_OUTPUT);
	if(configure_gpio < 0) {
		printk("can't configure clkoe pin \r\n");
	}

	gpio_pin_set(gpio0_dev, 24, 1);
	
	configure_gpio = gpio_pin_configure(gpio0_dev, 4, GPIO_INPUT);
	if(configure_gpio < 0) {
		printk("can't configure clkout pin \r\n");
	}

	configure_gpio = gpio_pin_interrupt_configure(gpio0_dev, 4, GPIO_INT_EDGE_TO_INACTIVE);
	if (configure_gpio < 0) {
		printk("Failed to configure interrupt for clkout pin \r\n");
	}

	gpio_init_callback(&clk_cb_data, clkout_edge_task, BIT(4));
	configure_gpio = gpio_add_callback(gpio0_dev, &clk_cb_data);
	if (configure_gpio < 0) {
		printk("Failed to add callback for clkout pin \r\n");
	}

}

void rtc_task(void* p1, void* p2, void* p3) {

	printk("rtc task is active! \r\n");

	k_sem_take(&final_time_stamp, K_FOREVER);

	gpio_pin_set(gpio0_dev, 24, 0);

	// WARNING THIS WILL NOT WORK WITHOUT AN ACCURATE AND HIGH SPEED TIMER!
	// max value of uint32_t is 4,294,967,295. nrf5240 has timer speed of 16Mhz, aka 62.5 ns per tick
	// theoretically, it could work, but experiments have shown that with software abstractions,
	// the typical calculated difference between active edge is roughly +/-0.5ms off of expected 1s period
	// e.g when tracking a 16Mhz square wave it actually shows 16008071Hz this is way off of 10ppm error
	// not sure why this is...
	// BUT a logic analyzer shows that the rv8803 clkout is actually around 56ns off of expected 1s, sometimes more
	// hence the it "could work" assertion.
	// furthermore, the nrf52840 timer has 10ppm error, while for the rv8803, 
	// 1.2ppm of compensation is roughly equivalent to offset of 5, where max offset is -32 -> +31
	// this means you need a more sensitive clock than the rv8803! (which is not likely :/ )
	
	//uint64_t total_cycles = timing_cycles_get(&start_time, &end_time);
	//uint64_t total_ns = timing_cycles_to_ns(total_cycles);
	//uint32_t total_timer_diff = end_time_val - start_time_val;
	//printk("timer diff: %d \r\n", total_timer_diff);
	/*
	double time_diff_hz = 1.0/((double)total_ns / 1000000000.0);
	double offset = ((time_diff_hz - 1) * 1000000.0)/0.2384;
	printk("offset: %d \r\n", (int)offset);

	struct sensor_value calib_offset = {.val1 = offset, .val2 = 0};
	int set_offset = sensor_attr_set(rv8803_dev, RV8803_INITIALIZE, RV8803_INITIALIZE_COMPUTE_OFFSET, &calib_offset);
	if(set_offset) {
		printk("having trouble starting offset calc! \r\n");
	} else {
		printk("set offset! \r\n");
	}
	*/

	int set_time_val = set_time(rv8803_dev, 21, 21);
	if(set_time_val) {
		printk("trouble setting the time \r\n");
	} else {
		printk("set the time! \r\n");
	}
	
	#define TIME_SLEEP_DURATION_MS 60 * 1000
	while(1) {
		uint8_t hours = 0;
		uint8_t minutes = 0;

		int get_time_error = get_time(rv8803_dev, &hours, &minutes);
		if(get_time_error) {
			printk("error at getting time \r\n");
		} else {
			printk("hours: %d, minutes %d \r\n", hours, minutes);
		}
		k_msleep(TIME_SLEEP_DURATION_MS);
	}
	
	while(1) {
		k_msleep(1000);
	}
}

int main(void)
{       
    //const struct device* drv2605_dev = DEVICE_DT_GET_ANY(ti_drv2605);
	//const struct device* pca9548_dev = DEVICE_DT_GET_ANY(ti_pca9548);

	// initialize a hardware timer here for the rtc calibration
	/*
	timer_t_inst.p_reg->PRESCALER = 0;
    uint32_t base_frequency = NRF_TIMER_BASE_FREQUENCY_GET(timer_t_inst.p_reg);
	printk("base frequency is: %d \r\n", base_frequency);
    nrfx_timer_config_t config = NRFX_TIMER_DEFAULT_CONFIG(base_frequency);
    config.bit_width = NRF_TIMER_BIT_WIDTH_32;
    config.p_context = &timer_t_inst;
    nrfx_err_t status = nrfx_timer_init(&timer_t_inst, &config, NULL);
    NRFX_ASSERT(status == NRFX_SUCCESS);
	nrfx_timer_clear(&timer_t_inst);
	nrfx_timer_enable(&timer_t_inst);
	*/

	// use port 0, pin 25 as a debug pin
	int configure_gpio = gpio_pin_configure(gpio0_dev, 25, GPIO_OUTPUT);
	if(configure_gpio < 0) {
		printk("can't configure debug pin 1\r\n");
	}

	gpio_pin_set(gpio0_dev, 25, 0);

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

	int init_switches = initialize_switches();
	if(init_switches < 0) {
		printk("problems initializing switches \r\n");
	} else {
		printk("switches are initialized \r\n");
	}

	rtc_task_tid = k_thread_create(&rtc_thread_data, rtc_thread_stack_area,
										K_THREAD_STACK_SIZEOF(rtc_thread_stack_area),
										rtc_task,
										NULL, NULL, NULL,
										RTC_THREAD_PRIORITY, 0, K_NO_WAIT);

	initialize_rtc();

	/*
	int init_accelerometer = initialize_accelerometer(adxl343_dev);
	if(init_accelerometer < 0) {
		printk("problems initializing accelerometer \r\n");
	} else {
		printk("accelerometer is initialized \r\n");
	}

	// pin 28 for accel int
	// should there be a pull-down here? Doesn't work if added
	int configure_gpio = gpio_pin_configure(gpio0_dev, 28, GPIO_INPUT);
	if(configure_gpio < 0) {
		printk("can't configure accel int pin \r\n");
	}

	configure_gpio = gpio_pin_interrupt_configure(gpio0_dev, 28, GPIO_INT_EDGE_TO_ACTIVE);
	if (configure_gpio < 0) {
		printk("Failed to configure interrupt for accel pin \r\n");
	}

	gpio_init_callback(&acc_cb_data, acc_trigger_task, BIT(28));
	configure_gpio = gpio_add_callback(gpio0_dev, &acc_cb_data);
	if (configure_gpio < 0) {
		printk("Failed to add callback for accel pin \r\n");
	}
	
	int init_haptics = initialize_haptics(pca9548_dev, drv2605_dev);
	while(init_haptics < 0) {
		if(init_haptics < 0) {
			printk("trying to initialize haptics again \r\n");
			k_msleep(SLEEP_TIME_MS);
			init_haptics = initialize_haptics(pca9548_dev, drv2605_dev);
		} 
	}
	*/
	
	#define SLEEP_DURATION_MS 1000
	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
            printk("failure at toggling led device\r\n");
			return 1;
		}

		/*
		if(toggle_clear_int == 1) {
			struct sensor_value dummy_data = {.val1 = 0, .val2 = 0};
			sensor_attr_set(adxl343_dev, INTERRUPT, INTERRUPT_CLEAR, &dummy_data);
			toggle_clear_int = 0;
		}
		*/
		/*
		for(uint8_t i = CHANNEL_ZERO; i < CHANNEL_SIX; i = i<<1) {
			int active_status = activate_haptic(pca9548_dev, drv2605_dev, i);
			if(active_status < 0) {
				printk("trouble triggering haptics at: %d \r\n", i);
			} else {
				printk("haptics at: %d is active \r\n", i);
			}
			k_msleep(SLEEP_TIME_MS);
		}
		
		k_msleep(SLEEP_TIME_MS);
		*/
		k_msleep(SLEEP_DURATION_MS);
	}

    return 0;
}