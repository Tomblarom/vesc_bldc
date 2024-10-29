/*
	Copyright 2024 Kim Bogner   		kim@ebikerepair.eu
    Copyright 2020 Marcos Chaparro	        mchaparro@powerdesigns.ca
	Copyright 2012-2022 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "hw.h"
#include "bafang_display_serial.h"
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "utils.h"
#include <math.h>
#include "utils_math.h"
#include "mc_interface.h"
#include "drv8301.h"
#include "terminal.h"
#include "commands.h"
#include "stdio.h"
#include "app.h"
#include "mempools.h"

// Variables
static volatile bool i2c_running = false;

// I2C configuration
static const I2CConfig i2cfg = {
		OPMODE_I2C,
		100000,
		STD_DUTY_CYCLE
};

void hw_init_gpio(void) {
	// GPIO clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	// ENABLE_GATE
	palSetPadMode(GPIOC, 5, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	ENABLE_GATE();

	// GPIOA Configuration: Channel 1 to 3 as alternate function push-pull
	palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING);	// H1
	palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST	| PAL_STM32_PUDR_FLOATING); 	// H2
	palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST	| PAL_STM32_PUDR_FLOATING);		// H3

	palSetPadMode(GPIOB, 15, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING);	// L1
	palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING);	// L2
	palSetPadMode(GPIOB, 13, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING);	// L3

	// Hall sensors
	palSetPadMode(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3, PAL_MODE_INPUT_PULLUP);

	// AUX pins LIGHT1 LIGHT2
	AUX_OFF();
	palSetPadMode(AUX_GPIO, AUX_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	AUX2_OFF();
	palSetPadMode(AUX2_GPIO, AUX2_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	// Pin-Mapping
	palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG);		// VOLT1
	palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG);		// VOLT2
	palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);		// VOLT3
	palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG);		// TEMP_PCB
	
	palSetPadMode(GPIOB, 0, PAL_MODE_INPUT_PULLUP);		// BRAKE
	palSetPadMode(GPIOB, 5, PAL_MODE_INPUT_ANALOG);		// PAS1/CADENCE
	palSetPadMode(GPIOB, 6, PAL_MODE_INPUT_PULLUP);		// WHEELSPEED
	// palSetPadMode(GPIOB, 7, PAL_MODE_INPUT_PULLUP);		// FAULT (not connected)

	palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG);		// CURR3
	palSetPadMode(GPIOC, 1, PAL_MODE_INPUT_ANALOG);		// CURR1
	palSetPadMode(GPIOC, 2, PAL_MODE_INPUT_ANALOG);		// CURR2

	// palSetPadMode(GPIOA, 4, PAL_MODE_INPUT_ANALOG);		// WHEELSPEED2 (CAN only)
	palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_ANALOG);		// TORQUE
	palSetPadMode(GPIOA, 6, PAL_MODE_INPUT_ANALOG);		// THROTTLE
	palSetPadMode(GPIOB, 4, PAL_MODE_INPUT_ANALOG);		// PAS2
	palSetPadMode(GPIOB, 2, PAL_MODE_INPUT_ANALOG);		// PWR_BTN_HLD
	palSetPadMode(GPIOC, 3, PAL_MODE_INPUT_ANALOG);		// VIN_SENS
	palSetPadMode(GPIOC, 4, PAL_MODE_INPUT_ANALOG);		// TEMP_MOTOR
	palSetPadMode(GPIOC, 13, PAL_MODE_INPUT_ANALOG);	// -PWR_BTN

	drv8301_init();
	bafang_display_serial_start((int8_t) 3);
}

void hw_setup_adc_channels(void) {
	uint8_t t_samp = ADC_SampleTime_15Cycles;

	// ADC1 regular channels									           IDX	CH	NAME
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0,		1, t_samp);		//  0  	 0	VOLT1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10,		2, t_samp);		//  3  	10	CURR1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6,		3, t_samp);		//  6 	 6	THROTTLE [ADC_IND_EXT]
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9,		4, t_samp);		//  9	 9  GEARSENSOR [ADC_IND_EXT3]
	ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint,	5, t_samp);		//  12 	17	VREFINT
	// ADC2 regular channels		
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1,		1, t_samp);		//  1  	 1	VOLT2
	ADC_RegularChannelConfig(ADC2, ADC_Channel_11,		2, t_samp);		//	4  	11	CURR2
	ADC_RegularChannelConfig(ADC2, ADC_Channel_5,		3, t_samp);		//  7 	 5	TORQUE [ADC_IND_EXT2]
	ADC_RegularChannelConfig(ADC2, ADC_Channel_14,		4, t_samp);		// 10 	14	TEMP_MOTOR
	ADC_RegularChannelConfig(ADC2, ADC_Channel_3,		5, t_samp);		// 13 	13	UNUSED
	// ADC3 regular channels		
	ADC_RegularChannelConfig(ADC3, ADC_Channel_2,		1, t_samp);		//  2  	 2	VOLT3
	ADC_RegularChannelConfig(ADC3, ADC_Channel_12,		2, t_samp);		//  5  	12	CURR3
	ADC_RegularChannelConfig(ADC3, ADC_Channel_3,		3, t_samp);		//  8 	 3	TEMP_MOS (PCB)
	ADC_RegularChannelConfig(ADC3, ADC_Channel_13,		4, t_samp);		// 11 	13	VIN_SENS
	ADC_RegularChannelConfig(ADC3, ADC_Channel_3,		5, t_samp);		// 14 	13	UNUSED

	// Injected channels
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 1, t_samp);     // CURR1
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 1, t_samp);     // CURR2
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 1, t_samp);     // CURR3
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 2, t_samp);     // CURR1
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 2, t_samp);     // CURR2
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 2, t_samp);     // CURR3
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 3, t_samp);     // CURR1
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 3, t_samp);     // CURR2
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 3, t_samp);     // CURR3
}

void hw_start_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);

	if (!i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		i2cStart(&HW_I2C_DEV, &i2cfg);
		i2c_running = true;
	}

	i2cReleaseBus(&HW_I2C_DEV);
}

void hw_stop_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);

	if (i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN, PAL_MODE_INPUT);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN, PAL_MODE_INPUT);

		i2cStop(&HW_I2C_DEV);
		i2c_running = false;

	}

	i2cReleaseBus(&HW_I2C_DEV);
}

/**
 * Try to restore the i2c bus
 */
void hw_try_restore_i2c(void) {
	if (i2c_running) {
		i2cAcquireBus(&HW_I2C_DEV);

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		chThdSleep(1);

		for(int i = 0;i < 16;i++) {
			palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
			palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
		}

		// Generate start then stop condition
		palClearPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);
		chThdSleep(1);
		palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		HW_I2C_DEV.state = I2C_STOP;
		i2cStart(&HW_I2C_DEV, &i2cfg);

		i2cReleaseBus(&HW_I2C_DEV);
	}
}

volatile float wheel_rpm_filtered = 0;
static float trip_odometer = 0;
void hw_update_speed_sensor(void) {
	static float wheel_rpm = 0;
	static uint8_t sensor_state = 0;
	static uint8_t sensor_state_old = 0;
	static float last_sensor_event_time = 0;
	float current_time = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;

	sensor_state = palReadPad(HW_SPEED_SENSOR_PORT, HW_SPEED_SENSOR_PIN);

	if(sensor_state == 0 && sensor_state_old == 1 ) {
		float revolution_duration = current_time - last_sensor_event_time;

		if (revolution_duration > 0.05) {	//ignore periods <50ms
			last_sensor_event_time = current_time;
			wheel_rpm = 60.0 / revolution_duration;
			UTILS_LP_FAST(wheel_rpm_filtered, (float)wheel_rpm, 0.5);


			const volatile mc_configuration *conf = mc_interface_get_configuration();
			trip_odometer += conf->si_wheel_diameter * M_PI;
			//trip_odometer += mc_interface_get_configuration()->si_wheel_diameter * M_PI; test this
		}
	} else {
		// After 3 seconds without sensor signal, set RPM as zero
		if ( (current_time - last_sensor_event_time) > 3.0) {
			wheel_rpm_filtered = 0.0;
		}
	}
	sensor_state_old = sensor_state;
}

/* Get speed in m/s */
float hw_get_speed(void) {
	const volatile mc_configuration *conf = mc_interface_get_configuration();
	float speed = wheel_rpm_filtered * conf->si_wheel_diameter * M_PI / 60.0;
	return speed;
}

/* Get trip distance in meters */
float hw_get_distance(void) {
	return trip_odometer;
}

float hw_get_distance_abs(void) {
	return trip_odometer;
}

/* Gear Shift sensor support
 * Read the gear sensor and use it to override the brake adc signal to reduce motor 
 * power during shifting to extend gearing life.
 */
void hw_brake_override(float *brake_ptr) {
	float brake = *brake_ptr;

	// Track an independent gearshift sensor ramping to be multiplied by the brake signal
	static float gear = 0.0;
	gear = (float)palReadPad(HW_GEAR_SENSOR_PORT, HW_GEAR_SENSOR_PIN);

	// hardcoded ramps for now
	const float ramp_time_neg = 0.1;
	const float ramp_time_pos = 0.3;

	// Apply ramping
	static systime_t last_time = 0;
	static float gear_ramp = 0.0;
	float ramp_time = fabsf(gear) > fabsf(gear_ramp) ? ramp_time_pos : ramp_time_neg;

	if (ramp_time > 0.01) {
		const float ramp_step = (float)ST2MS(chVTTimeElapsedSinceX(last_time)) / (ramp_time * 1000.0);
		utils_step_towards(&gear_ramp, gear, ramp_step);
		last_time = chVTGetSystemTimeX();
		*brake_ptr = brake * gear_ramp;
	}

	static uint16_t delay_to_print = 0;
	if(delay_to_print ++ > 250){
		delay_to_print = 0;
		commands_printf("gear_ramp:%.2f",(double)gear_ramp);
		commands_printf("brake_output:%.2f",(double)brake);
	}
}