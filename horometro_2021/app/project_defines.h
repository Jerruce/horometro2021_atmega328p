/*
 * project_defines.h
 *
 * Created: 23/04/2021 12:31:05
 *  Author: mtorres
 */ 


#ifndef PROJECT_DEFINES_H_
#define PROJECT_DEFINES_H_

/* CPU frequency */

#define F_CPU	128000UL

/*******************************************
 ************* PIN DEFINITION **************
 *******************************************/

/* Digital inputs */

#define DDR_ACCEL_SENSOR				DDRD
#define PIN_ACCEL_SENSOR				PIND
#define PORT_ACCEL_SENSOR				PORTD
#define ACCEL_SENSOR					2

#define DDR_PICK_UP_SENSOR				DDRD
#define PIN_PICK_UP_SENSOR				PIND
#define PORT_PICK_UP_SENSOR				PORTD
#define	PICK_UP_SENSOR					3

#define DDR_DIP_SW_CALIB_MODE			DDRD
#define PIN_DIP_SW_CALIB_MODE			PIND
#define PORT_DIP_SW_CALIB_MODE			PORTD
#define	DIP_SW_CALIB_MODE				4

#define DDR_WIFI_BUTTON					DDRD
#define PIN_WIFI_BUTTON					PIND
#define PORT_WIFI_BUTTON				PORTD
#define	WIFI_BUTTON						5

#define DDR_MODE_BUTTON					DDRD
#define PIN_MODE_BUTTON					PIND
#define PORT_MODEI_BUTTON				PORTD
#define	MODE_BUTTON						6

#define DDR_MCU_FEEDBACK_HANDSHAKE		DDRE
#define PIN_MCU_FEEDBACK_HANDSHAKE		PINE
#define PORT_MCU_FEEDBACK_HANDSHAKE		PORTE
#define	MCU_FEEDBACK_HANDSHAKE			1


/* Digital outputs */

#define DDR_SW_CURRENT					DDRC
#define PORT_SW_CURRENT					PORTC
#define SW_CURRENT						4

#define DDR_SW_VBAT_MEASURE				DDRC
#define PORT_SW_VBAT_MEASURE			PORTC
#define SW_VBAT_MEASURE					5

#define DDR_SW_ESP32					DDRD
#define PORT_SW_ESP32					PORTD
#define SW_ESP32						7

#define DDR_SW_SUPPLY					DDRB
#define PORT_SW_SUPPLY					PORTB
#define SW_SUPPLY						0

#define DDR_SW_ACCEL					DDRB
#define PORT_SW_ACCEL					PORTB
#define SW_ACCEL						1

#define DDR_SW_PICKUP					DDRB
#define PORT_SW_PICKUP					PORTB	
#define SW_PICKUP						2

#define DDR_ACCEL_SENSING_LED			DDRE
#define PORT_ACCEL_SENSING_LED			PORTE
#define ACCEL_SENSING_LED				0


/* SPI Communication */

#define	DDR_MCU_TO_MCU_CS				DDRE			
#define	PIN_MCU_TO_MCU_CS				PINE
#define	PORT_MCU_TO_MCU_CS				PORTE
#define MCU_TO_MCU_CS					2


/*************************************************
 ************* CONSTANTS AND MACROS **************
 *************************************************/

// ------------------ Defines for ADC ------------------------

#define CURRENT_MEASURE_ADC_CHANNEL			ADC_Channel_2
#define VBAT_MEASURE_ADC_CHANNEL			ADC_Channel_3
#define ADC_REF_VOLTAGE						1.1

// ----------- Defines for UART communication -----------------

#define DEBUG_UART_PORT						UART0

// -------------- Defines for time periods ---------------------

#define PSU_SW_WAIT_PERIOD_MS					50	// 50ms 
#define HOROMETER_SECOND_N_STEPS				32	// 32 * 31.25ms = 1 sec
#define CALIBRATION_COUNT_DISPLAY_PERIOD_SEC	20  // 20 seconds
#define BATTERY_MEASURE_PERIOD_SEC				2//900 // 900 seconds = 15 minutes
#define WORKING_COUNT_DISPLAY_PERIOD_SEC		60//3600 // 3600 seconds = 1 hour
#define WEB_PARAMETERS_CHECK_PERIOD_SEC			2

// ----------- Defines for current measurement -----------------

#define CURRENT_SENSE_RESISTOR_OHM				0.02		
#define CURRENT_SENSE_CIRCUIT_GAIN_01			20.0
#define CURRENT_SENSE_CIRCUIT_GAIN_02			(-3.0)
#define CURRENT_RMS_CALC_N_SAMPLES				100

#define CURRENT_SENSE_SPAN						1.0
#define CURRENT_SENSE_OFFSET					0.0

#define OVERCURRENT_UPPER_THRESHOLD				81
#define OVERCURRENT_LOWER_THRESHOLD				79

#define MOTOR_STUCK_CURRENT_UPPER_THRESHOLD		11
#define MOTOR_STUCK_CURRENT_LOWER_THRESHOLD		9

// ----------- Defines for the magnetic pick-up -----------------

#define TIMER1_PRESCALER_VALUE					1
#define F_CPU_HZ								128000
#define FREQ_CORRECTION_FACTOR					0.78125
#define MAG_PICKUP_MAX_FREQ_HZ					60
#define MAG_PICKUP_MIN_FREQ_HZ					5
#define PICKUP_N_SAMPLES						16

//#define MOTOR_STUCK_FREQ_UPPER_THRESHOLD_RPM	121	
//#define MOTOR_STUCK_FREQ_LOWER_THRESHOLD_RPM	119

// ----------- Defines for battery voltage sense -----------------

#define BATTERY_VSENSE_GAIN								5.0
#define BATT_MIN_OUT_V									2.75
#define BATT_MAX_OUT_V									4.2

#define BATT_MED_TO_HIGH_LEVEL_VOLT_THRESHOLD			4.0
#define BATT_HIGH_TO_MED_LEVEL_VOLT_THRESHOLD			3.8
#define BATT_LOW_TO_MED_LEVEL_VOLT_THRESHOLD			3.8
#define BATT_MED_TO_LOW_LEVEL_VOLT_THRESHOLD			3.6	
#define BATT_CRITICAL_TO_LOW_LEVEL_VOLT_THRESHOLD		3.4
#define BATT_LOW_TO_CRITICAL_LEVEL_VOLT_THRESHOLD		3.2	

#define BATTERY_LEVEL_CRITICAL							0
#define BATTERY_LEVEL_LOW								1
#define BATTERY_LEVEL_MED								2
#define BATTERY_LEVEL_HIGH								3

// -------------------- Sequence states ----------------------
#define SEQUENCE_IN_PROCESS								0
#define SEQUENCE_COMPLETE								1

// ---------------------- System modes -----------------------

#define VIBRATION_SENSOR_ONLY_MODE						0
#define VIBRATION_CURRENT_PICKUP_SENSOR_MODE			1
#define VIBRATION_SENSOR_CALIBRATION_MODE				2	

// ----------------- Alarm status flags  -----------------------

#define E1_FLAG		0
#define AL1_FLAG	1
#define E2_FLAG		2
#define AL2_FLAG	3
#define E3_FLAG		4
#define AL3_FLAG	5
#define ALS_FLAG	6
#define ALO_FLAG	7

/* ---------------------- System flags ------------------------ */

#define MAG_PICKUP_TIMEOUT_FLAG			 0
#define MAG_PICKUP_TOO_FAST_FLAG		 1
#define ONE_SECOND_ELAPSED_FLAG			 2
#define BUTTON_READ_FLAG				 3
#define CURRENT_SENSE_FLAG				 4
#define VIBRATION_SENSE_FLAG			 5	
#define WIFI_COMM_EN_FLAG				 6
#define ESP32_COMM_CHECK_FLAG			 7
#define ALARM_01_REACHED_FLAG			 8	
#define ALARM_02_REACHED_FLAG			 9
#define ALARM_03_REACHED_FLAG			 10
#define OVERCURRENT_ALARM_FLAG			 11
#define MOTOR_STUCK_ALARM_FLAG			 12
#define CHANGE_OPERATION_MODE_FLAG		 13	
#define CHANGE_TO_CALIB_MODE_FLAG		 14
#define SHOW_CALIBRATION_SCREEN_FLAG	 15
#define SHOW_MAIN_OR_ALARM_SCREEN_FLAG	 16
#define TOGGLE_SCREEN_INDEX_FLAG		 17				  
#define BATTERY_LEVEL_MEASURE_FLAG		 18
#define ESP32_WEB_PARAMETERS_CHECK_FLAG	 19	
		

#define SERIAL_MSG_FLAG					 31


#endif /* PROJECT_DEFINES_H_ */