/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   500

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);


#define DRV2605_NODE DT_NODELABEL(drv2605)
static const struct i2c_dt_spec drv2605_device = I2C_DT_SPEC_GET(DRV2605_NODE);

/**
 * DRV2605 LRA Driver
 * 
 * Questions
 * 1). What other setups exist besides closed-loop bidirectional mode?
 * 
 * Factors
 * 1). Consider the power supply needed to power these motors!
 * 
 * Initialization 
 * 1). ERM_LRA bit in register 0x1A set to LRA
 * 2). Do I need to perform calibration ?
 * 3). STARTUP_BOOST to reduce start-time performance
 * 4). BRAKE_FACTOR to reduce break-time performance, but aggressive ratios reduce stability
 * 5). BRAKE_STABILIZER to improve stability issues at high ratios
 * 6). LOOP_GAIN to control the speed to match back-emf of motor
 * 7). Set MODE in register 0x01 to select interface mode ( use the internal memory! )
 * 	7a). interfaces include pwm, internal memory, rtp, analog input
 * 8). MAKE SURE REG PIN IS NOT SHORTED!
 * 
 * 
 * Operation
 * 1). Check failure detection register
 * 2). Make sure that the automatic level-calibration routine doesn't fail
 * 	2a). Check out the automatic-level calibration programming section
 * 3). Ensure that the actuator is present with DIAG_RESULT
 * 4). When using the internal memory interface, observe the input trigger options
 * 	4a). Use the GO bit of register 0x0C to launch/stop the waveform
 * 	4b). You can send GO command over I2C, or use the IN/TRIG pins to send GO with lower latency
 * 	4c). IN/TRIG pin can operate on edge/level trigger
 * 5). Check OVER_TEMP to make sure that the device is not overheating!
 * 6). Check OC_DETECT to ensure there isn't a short at the output
 * 7). Shutdown state is lowest power mode and registers are not reset
 * 	7a). A good strategy may be to maximize time in shutdown
 * 	7b). In order to play a waveform while at shutdown, have to go into standby, then into active
 * 8). If there is a short before waveform is played, must observe and perform cycling in Figure 16!
*/

// DRV2605 registers
uint8_t STATUS_REG = 0x00;
uint8_t MODE_REG = 0x01;
uint8_t LIBRARY_REG = 0x03;
uint8_t WAVEFORM_SEQ1_REG = 0x04;
uint8_t GO_REG = 0x0C;
uint8_t RATED_VOLTAGE_REG = 0x16;
uint8_t OVERDRIVE_CLAMP_VOLTAGE_REG = 0x17;
uint8_t FEEDBACK_CONTROL_REG = 0x1A;
uint8_t CONTROL1_REG = 0x1B;
uint8_t CONTROL2_REG = 0x1C;
uint8_t CONTROL4_REG = 0x1E;


bool DRV2605_Init() {

	// i2c operation result
	uint8_t i2c_operation_error = 0;
	
	// take the device out of standby mode
	i2c_operation_error = i2c_reg_write_byte_dt(&drv2605_device, MODE_REG, 0x00);
	if(i2c_operation_error) {
		printk("failure before starting auto calibration");
	}

	/**
	 * Perform auto-calibration
	 * Steps: 
	 * 0). Write 0x07 to register 0x01
	 * 1). Set ERM_LRA bit of reg 0x1A
	 * 2). Set FB_BRAKE_FACTOR bit field of reg 0x1A
	 * 3). Set LOOP_GAIN bit field of reg 0x1A 
	 * 	1-3). Value is 0b10101010
	 * 4). Set RATED_VOLTAGE bit field of reg 0x16
	 * 	4a). LRA used is the VG0832022D lra from Digikey
	 * 	4b). Voltage of 1.8VAC RMS, frequency of 235Hz
	 * 	4c). formula is Vrms = (20.71*10^-3 * RATED_VOLTAGE)/sqrt(1-(4*SAMPLE_TIME + 300 * 10^-6) * fLRA))
	 * 	4d). SAMPLE_TIME is recommended to be 300uS
	 * 	4e). RATED_VOLTAGE is 83.7524... which is 0x53
	 * 5). Set OD_CLAMP bit field of reg 0x17
	 * 	5a). Vclamp = 21.96 * 10^-3 * OD_CLAMP
	 * 	5b). 1.8 = 21.96 * 10^-3 * OD_CLAMP
	 * 	5c). OD_CLAMP = 81.967 = 0x52
	 * 6). Set AUTO_CAL_TIME bit field of reg 0x1E
	 * 	6a). 0b00110000
	 * 7). Set DRIVE_TIME bit field of reg 0x1B
	 * 	7a). drive time(ms) = 0.5 * LRA Period (0.00426s for 235 Hz) = 2.13ms
	 * 	7b). 2.13ms = DRIVE_TIME * 0.1 + 0.5ms, so DRIVE_TIME = 16
	 * 	7c). 0b10010000
	 * 8). Set SAMPLE_TIME bit field of 0x1C
	 * 9). Set BLANKING_TIME bit field of 0x1C
	 * 10). Set IDISS_TIME bit field of 0x1C
	 * 	8-10). 0b11110101 
	 * 11). Set GO bit by writing 0x01 to register 0x0C - wait for GO bit to clear
	 * 12). Check status of DIAG_RESULT of regiester 0x00 to ensure that calibration is complete without faults
	*/

	// step 0: put device in auto-calibration mode
	i2c_operation_error = i2c_reg_write_byte_dt(&drv2605_device, MODE_REG, 0x07);
	if(i2c_operation_error) {
		printk("failure at step 0 of auto calibration");
		return false;
	}

	// step 1-3: write to FEEDBACK_CONTROL_REG
	i2c_operation_error = i2c_reg_write_byte_dt(&drv2605_device, FEEDBACK_CONTROL_REG, 0b10101010);
	if(i2c_operation_error) {
		printk("failure at step 1-3 of auto calibration");
		return false;
	}

	// step 4: write to RATED_VOLTAGE_REG
	i2c_operation_error = i2c_reg_write_byte_dt(&drv2605_device, RATED_VOLTAGE_REG, 0x53);
	if(i2c_operation_error) {
		printk("failure at step 4 of auto calibration");
		return false;
	}

	// step 5: write to OVERDRIVE_CLAMP_VOLTAGE_REG
	i2c_operation_error = i2c_reg_write_byte_dt(&drv2605_device, OVERDRIVE_CLAMP_VOLTAGE_REG, 0x52);
	if(i2c_operation_error) {
		printk("failure at step 5 of auto calibration");
		return false;
	}

	// step 6: set AUTOCAL_TIME by writing to CONTROL4_REG
	i2c_operation_error = i2c_reg_write_byte_dt(&drv2605_device, CONTROL4_REG, 0b00110000);
	if(i2c_operation_error) {
		printk("failure at step 6 of auto calibration");
		return false;
	}

	// step 7: set DRIVE_TIME by writing to CONTROL1_REG
	i2c_operation_error = i2c_reg_write_byte_dt(&drv2605_device, CONTROL1_REG, 0b10010000);
	if(i2c_operation_error) {
		printk("failure at step 7 of auto calibration");
		return false;
	}

	// step 8-10: write to CONTROL2_REG
	i2c_operation_error = i2c_reg_write_byte_dt(&drv2605_device, CONTROL2_REG, 0b11110101);
	if(i2c_operation_error) {
		printk("failure at step 8-10 of auto calibration");
		return false;
	}

	// step 11: write to GO register to start auto-calibration and wait for bit to deassert
	i2c_operation_error = i2c_reg_write_byte_dt(&drv2605_device, GO_REG, 0x01);
	if(i2c_operation_error) {
		printk("failure at step 11 of auto calibration");
		return false;
	}

	uint8_t calibrating = 1;
	while(calibrating) {
		i2c_operation_error = i2c_reg_read_byte_dt(&drv2605_device, GO_REG, &calibrating);
		if(i2c_operation_error) {
			printk("failure at step 11 of auto calibration waiting for calibration to be finished");
			return false;
		}
	}

        printk("finished calibration \r\n");

	// step 12: check diag_result to make sure that the auto-calibration is finished correctly
	uint8_t autocalibration_result = 0;
	i2c_operation_error = i2c_reg_read_byte_dt(&drv2605_device, STATUS_REG, &autocalibration_result);
	if(i2c_operation_error) {
		printk("failure at step 12 of auto calibration");
		return false;
	}

	if(!(autocalibration_result & 0b00001000)) {
		printk("failure at step 12 of auto calibration - result did not converge");
		return false;
	}

        printk("autocalibration results look good \r\n");

	// select LRA waveform library
	i2c_operation_error = i2c_reg_write_byte_dt(&drv2605_device, LIBRARY_REG, 0b00000110);
	if(i2c_operation_error) {
		printk("failure at auto calibration waveform library selection");
		return false;
	}
		
	// put device is standby mode
	i2c_operation_error = i2c_reg_write_byte_dt(&drv2605_device, MODE_REG, 0b01000000);
	if(i2c_operation_error) {
		printk("failure at auto calibration waveform library selection");
		return false;
	}

	return true;
}

bool DRV2605_EmitWaveform(uint8_t waveform_index) {

	// check if waveform_index is within range
	if(waveform_index > 123) {
		return false;
	}

	// i2c operation error
	uint8_t i2c_operation_error = 0;

	// Deassert Standby and set to internal trigger
	i2c_operation_error = i2c_reg_write_byte_dt(&drv2605_device, MODE_REG, 0b00000000);
	if(i2c_operation_error) {
		printk("failure at EmitWaveform deasserting standby");
		return false;
	}
	
	// Select waveform index to be played and write it address 0x04
	i2c_operation_error = i2c_reg_write_byte_dt(&drv2605_device, WAVEFORM_SEQ1_REG, waveform_index);
	if(i2c_operation_error) {
		printk("failure at EmitWaveform selecting waveform");
		return false;
	}
	
	// Set Go bit to to fire the effect
	i2c_operation_error = i2c_reg_write_byte_dt(&drv2605_device, GO_REG, 0b00000001);
	if(i2c_operation_error) {
		printk("failure at EmitWaveform starting effect");
		return false;
	}

	uint8_t firing = 1;
	while(firing) {
		i2c_operation_error = i2c_reg_read_byte_dt(&drv2605_device, GO_REG, &firing);
		if(i2c_operation_error) {
			printk("failure at step EmitWaveform firing sequence");
			return false;
		}
	}

	// check for output shorts and over_temp here
	uint8_t physical_error = 0;
	i2c_operation_error = i2c_reg_read_byte_dt(&drv2605_device, STATUS_REG, &physical_error);
	if(i2c_operation_error) {
		printk("failure at step EmitWaveform reading physical errors");
		return false;
	}

	if((physical_error & 0b00000011)) {
		printk("failure at step EmitWaveform with physical errors");
		return false;
	}

	// Put device in low power mode by setting the STANDBY bit
	i2c_operation_error = i2c_reg_write_byte_dt(&drv2605_device, MODE_REG, 0b01000000);
	if(i2c_operation_error) {
		printk("failure at EmitWaveform going back to standby");
		return false;
	}
	
	return true;
}

int main(void)
{

	if (!device_is_ready(led.port)) {
                printk("failure at initializing led device\r\n");
		return 1;
	}


	if (!device_is_ready(drv2605_device.bus)) {
                printk("failure at initializing i2c device\r\n");
	        return 1;
	}

	int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
                printk("failure at configuring gpio device\r\n");
		return 1;
	}

        printk("starting haptics initialization \r\n");

	bool init_haptics = DRV2605_Init();
	if(!init_haptics) {
		printk("trouble initializing haptics driver\r\n");
                return 1;
	}
        
        uint8_t index = 1;
	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
                        printk("failure at toggling led device\r\n");
			return 1;
		}
		
		bool trigger_haptics = DRV2605_EmitWaveform(index);
		if(!trigger_haptics) {
			printk("trouble trigger haptics driver");
		} else {
			index = (index + 1) % 124;
			printk("beeping at index %d \r\n", index);
		}

		k_msleep(SLEEP_TIME_MS);
	}

        return 0;
}