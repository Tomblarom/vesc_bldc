/*
	Copyright 2024 Kim Bogner   		    kim@ebikerepair.eu
    Copyright 2020 Marcos Chaparro	        mchaparro@powerdesigns.ca
    Copyright 2016 - 2022 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistrib50ute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef HW_X1_CORE_H_
#define HW_X1_CORE_H_

#ifdef HW_X1_IS_V1
#error "Not supported yet. Use X1_V2B_UART"
#define HW_NAME					"X1_V1"
#elif defined(HW_X1_IS_V2A)
#error "Not supported yet. Use X1_V2B_UART"
#define HW_NAME					"X1_V2A"
#elif defined(HW_X1_IS_V2B_CAN)
#error "Not supported yet. Use X1_V2B_UART"
#define HW_NAME					"X1_V2B_CAN"
#elif defined(HW_X1_IS_V2B_UART)
#define HW_NAME					"X1_V2B_UART"
#else
#error "Must include hardware type"
#endif

// HW properties
#define HW_HAS_DRV8301
#define HW_HAS_3_SHUNTS
#define HW_HAS_WHEEL_SPEED_SENSOR
#define HW_HAS_BAFANG_SERIAL_DISPLAY
// #define HW_HAS_PAS_TORQUE_SENSOR
// #define HW_USE_BRK

// Macros
#define ENABLE_GATE()			palSetPad(GPIOC, 5)
#define DISABLE_GATE()			palClearPad(GPIOC, 5)
#define DCCAL_ON()
#define DCCAL_OFF()
#define IS_DRV_FAULT()			(!palReadPad(GPIOB, 7))

#define AUX_GPIO				GPIOC
#define AUX_PIN					15      // LIGHT1 - OK
#define AUX_ON()				palSetPad(AUX_GPIO, AUX_PIN)
#define AUX_OFF()				palClearPad(AUX_GPIO, AUX_PIN)
#define AUX2_GPIO				GPIOC
#define AUX2_PIN				14      // LIGHT2 - OK
#define AUX2_ON()				palSetPad(AUX2_GPIO, AUX2_PIN)
#define AUX2_OFF()				palClearPad(AUX2_GPIO, AUX2_PIN)

// #define BRK_GPIO	            GPIOB
// #define BRK_PIN		            0

#define HW_ADC_INJ_CHANNELS		3
#define HW_ADC_NBR_CONV			4
#define HW_ADC_CHANNELS			(HW_ADC_NBR_CONV * 3)

// ADC Indexes                 IDX     NAME
#define ADC_IND_SENS1			0   // VOLT1
#define ADC_IND_SENS2			1   // VOLT2
#define ADC_IND_SENS3			2   // VOLT3
#define ADC_IND_CURR1			3   // CURR1
#define ADC_IND_CURR2			5   // CURR3
#define ADC_IND_CURR3			4   // CURR2
#define ADC_IND_EXT		        6   // THROTTLE     ADC1
#define ADC_IND_EXT2			7   // TORQUE       ADC2   0.8-3.3V
// #define ADC_IND_EXT3            9   // GEARSENSOR   ADC3
// #define ADC_IND_PWR_BTN         
#define ADC_IND_TEMP_MOS		8   // TEMP_MOS
#define ADC_IND_TEMP_MOTOR		10  // TEMP_MOTOR
#define ADC_IND_VIN_SENS		11  // VIN_SENS
#define ADC_IND_VREFINT			9  // VREFINT

// ADC macros and settings
#ifndef V_REG
#define V_REG					3.3
#endif
#ifndef VIN_R1
#define VIN_R1					39000.0
#endif
#ifndef VIN_R2
#define VIN_R2					2200.0
#endif
#ifndef CURRENT_AMP_GAIN
#define CURRENT_AMP_GAIN		20.0        // AD8418WBRMZ (Y4M)
#endif
#ifndef CURRENT_SHUNT_RES
#define CURRENT_SHUNT_RES		0.0005      // 0.5mΩ CSS2H-2512R-L500FE
#endif

// Input voltage
#define GET_INPUT_VOLTAGE()		((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] * ((VIN_R1 + VIN_R2) / VIN_R2))

// Temperature Sensors
#define NTC_RES(adc_val)        ((10000.0 * adc_val) / (4095.0 - adc_val))
#define NTC_TEMP(adc_ind)       (1.0 / ((logf(NTC_RES(ADC_Value[adc_ind]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15)
#define NTC_TEMP_MOTOR(beta)	(1.0 / ((logf(NTC_RES_MOTOR(ADC_Value[ADC_IND_TEMP_MOTOR]) / 10000.0) / beta) + (1.0 / 298.15)) - 273.15)
#define PTC_TEMP_MOTOR(res, con, tbase)			(((NTC_RES_MOTOR(ADC_Value[ADC_IND_TEMP_MOTOR]) - res) / NTC_RES_MOTOR(ADC_Value[ADC_IND_TEMP_MOTOR])) * 100.0 / con - 10.0)
#define PTC_TEMP_MOTOR_2(res, con, tbase)		0.0
#define MOTOR_TEMP_LPF	0.001
#define NTC_RES_MOTOR(adc_val)	(1000.0 / ((4095.0 / (float)adc_val) - 1.0))

// Voltage on ADC channel
#define ADC_VOLTS(ch)			((float)ADC_Value[ch] / 4096.0 * V_REG)

#define NTC_TEMP_MOS2()         ((float)ADC_VOLTS(ADC_IND_EXT))     // log throttle data
#define NTC_TEMP_MOS3()         ((float)ADC_VOLTS(ADC_IND_EXT2))    // log torque data

// ADC GPIOs
#define HW_ADC_EXT_GPIO 	GPIOA
#define HW_ADC_EXT_PIN		6               // THROTTLE
#define HW_ADC_EXT2_GPIO	GPIOA
#define HW_ADC_EXT2_PIN		5               // TORQUE

// UART Peripheral: Bafang Display
#define HW_UART_DEV				SD3
#define HW_UART_GPIO_AF			GPIO_AF_USART3
#define HW_UART_TX_PORT			GPIOB
#define HW_UART_TX_PIN			10
#define HW_UART_RX_PORT			GPIOB
#define HW_UART_RX_PIN			11

// Permanent UART Peripheral
// #define HW_UART_P_BAUD			9600                    //OK - Bafang Ultra Displays have 9600. BBSHD has 1200
// #define HW_UART_P_DEV			SD4                     //?
// #define HW_UART_P_DEV_TX		SD5                     //?     // UART for TX, due to mistake below
// #define HW_UART_P_GPIO_AF		GPIO_AF_UART4           //?
// #define HW_UART_P_TX_PORT		GPIOC
// // #define HW_UART_P_TX_PIN		12                      //?     // This is a mistake in the HW. We have to use a hack to use UART5.
// #define HW_UART_P_RX_PORT		GPIOC
// #define HW_UART_P_RX_PIN		11

// ICU Peripheral: PAS1/CADENCE
#define HW_USE_SERVO_TIM4
#define HW_ICU_TIMER			TIM4
#define HW_ICU_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE)
#define HW_ICU_DEV				ICUD4
#define HW_ICU_CHANNEL			ICU_CHANNEL_1
#define HW_ICU_GPIO_AF			GPIO_AF_TIM4
#define HW_ICU_GPIO				GPIOB
#define HW_ICU_PIN				5

// I2C Peripheral
#define HW_I2C_DEV				I2CD2                   //?
#define HW_I2C_GPIO_AF			GPIO_AF_I2C2            //?
#define HW_I2C_SCL_PORT			GPIOB
#define HW_I2C_SCL_PIN			10                      //?
#define HW_I2C_SDA_PORT			GPIOB
#define HW_I2C_SDA_PIN			11                      //?

// Hall/encoder pins - AS5047P
#define HW_HALL_ENC_GPIO1		GPIOC
#define HW_HALL_ENC_PIN1		6                       // SCK  SCLK (HALL_U_uC)
#define HW_HALL_ENC_GPIO2		GPIOC
#define HW_HALL_ENC_PIN2		7                       // MISO MISO (HALL_V_uC)
#define HW_HALL_ENC_GPIO3		GPIOC
#define HW_HALL_ENC_PIN3		8                       // NSS  HALL_CS_uC (HALL_W_uC)
#define HW_ENC_TIM				TIM3
#define HW_ENC_TIM_AF			GPIO_AF_TIM3
#define HW_ENC_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE)
#define HW_ENC_EXTI_PORTSRC		EXTI_PortSourceGPIOC
#define HW_ENC_EXTI_PINSRC		EXTI_PinSource8
#define HW_ENC_EXTI_CH			EXTI9_5_IRQn
#define HW_ENC_EXTI_LINE		EXTI_Line8
#define HW_ENC_EXTI_ISR_VEC		EXTI9_5_IRQHandler
#define HW_ENC_TIM_ISR_CH		TIM3_IRQn
#define HW_ENC_TIM_ISR_VEC		TIM3_IRQHandler

// SPI pins - OK
#define HW_SPI_DEV				SPID1
#define HW_SPI_GPIO_AF			GPIO_AF_SPI1
#define HW_SPI_PORT_MOSI		GPIOC
#define HW_SPI_PIN_MOSI			12
#define HW_SPI_PORT_MISO		GPIOC
#define HW_SPI_PIN_MISO			11
#define HW_SPI_PORT_SCK			GPIOC
#define HW_SPI_PIN_SCK			10
#define HW_SPI_PORT_NSS			GPIOC
#define HW_SPI_PIN_NSS			9

// SPI pins DRV8301 - OK
#define DRV8301_MOSI_GPIO		GPIOC
#define DRV8301_MOSI_PIN		12
#define DRV8301_MISO_GPIO		GPIOC
#define DRV8301_MISO_PIN		11
#define DRV8301_SCK_GPIO		GPIOC
#define DRV8301_SCK_PIN			10
#define DRV8301_CS_GPIO			GPIOC
#define DRV8301_CS_PIN			9

// Pedal Assist pins
#define HW_PAS1_PORT			GPIOB
#define HW_PAS1_PIN				5
#define HW_PAS2_PORT			GPIOB
#define HW_PAS2_PIN				4

// Ebike Sensors
#define HW_SPEED_SENSOR_PORT    GPIOB
#define HW_SPEED_SENSOR_PIN     6
#define HW_GEAR_SENSOR_PORT     GPIOB
#define HW_GEAR_SENSOR_PIN      1

// Measurement macros
#define ADC_V_L1				ADC_Value[ADC_IND_SENS1]
#define ADC_V_L2				ADC_Value[ADC_IND_SENS2]
#define ADC_V_L3				ADC_Value[ADC_IND_SENS3]
#define ADC_V_ZERO				(ADC_Value[ADC_IND_VIN_SENS] / 2)

// Macros
#define READ_HALL1()			palReadPad(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1)
#define READ_HALL2()			palReadPad(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2)
#define READ_HALL3()			palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)

// LEDs (empty since no LEDs present)
#define LED_GREEN_ON()
#define LED_GREEN_OFF()
#define LED_RED_ON()
#define LED_RED_OFF()

// Default setting overrides
#ifndef MCCONF_DEFAULT_MOTOR_TYPE
#define MCCONF_DEFAULT_MOTOR_TYPE		MOTOR_TYPE_FOC
#endif
#ifndef MCCONF_FOC_F_ZV                 //Zero Vector Frequency in Hz
#define MCCONF_FOC_F_ZV					30000.0 
#endif
#ifndef MCCONF_L_MAX_ABS_CURRENT
#define MCCONF_L_MAX_ABS_CURRENT		150.0
#endif
#ifndef MCCONF_FOC_SAMPLE_V0_V7
#define MCCONF_FOC_SAMPLE_V0_V7			false	
#endif

// Setting limits
#define HW_LIM_CURRENT			-100.0, 100.0
#define HW_LIM_CURRENT_IN		-10.0, 100.0
#define HW_LIM_CURRENT_ABS		0.0, 150.0
#define HW_LIM_VIN				8.0, 59.5
#define HW_LIM_ERPM				-15e3, 15e3
#define HW_LIM_DUTY_MIN			0.0, 0.005
#define HW_LIM_DUTY_MAX			0.0, 0.95
#define HW_LIM_TEMP_FET			-40.0, 110.0

// HW-specific functions
void hw_update_speed_sensor(void);
float hw_get_speed(void);
float hw_get_distance(void);
float hw_get_distance_abs(void);
void hw_brake_override(float *brake);

#endif /* HW_X1_CORE_H_ */
