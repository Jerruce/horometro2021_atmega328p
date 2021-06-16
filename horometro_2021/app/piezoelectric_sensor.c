/*
 * MT_piezoelectric_sensor.c
 *
 * Created: 21/05/2021 12:07:36
 *  Author: mtorres
 */ 


/* File inclusion */
#include "piezoelectric_sensor.h"


/* Variable definition */

static uint32_t working_time_step_counter = 0;
static uint32_t working_time_hours = 0;
static uint8_t working_time_minutes = 0;
static uint8_t working_time_seconds = 0;

static uint32_t calib_time_step_counter = 0;
static uint32_t calib_time_seconds = 0;


/* Function definition */

void Piezoelectric_Sensor_Initialize(void){
	
	/* Configure acceleration sensor pin as INPUT and enable the internal pull-up resistor if required */
	PIN_ACCEL_SENSOR &= ~(1 << ACCEL_SENSOR);
	PORT_ACCEL_SENSOR |= (1 << ACCEL_SENSOR);
	
	/* Configure external interrupt INT1 to activate with every rising edge */
	EICRA |= (1 << ISC00);
	EICRA |= (1 << ISC01);
	
	/* Clear the INT0 flag */
	EIFR |= (1 << INTF0);	
}


uint8_t Piezoelectric_Sensor_Read(void){
	
	uint8_t val;
	
	cli();
	val = EIFR & (1 << INTF0);
	EIFR |= (1 << INTF0);
	sei();
	
	return val; 	
}


void Working_Time_Count_Update(void){
	
	if(Piezoelectric_Sensor_Read()){
	
		working_time_step_counter++;
		if(working_time_step_counter >= HOROMETER_SECOND_N_STEPS){
			working_time_step_counter = 0;
			working_time_seconds++;
			if(working_time_seconds >= 60){
				working_time_seconds = 0;
				working_time_minutes++;
				if(working_time_minutes >= 60){
					working_time_minutes = 0;
					working_time_hours++;
				}
			}
		}
	}
	
}


void Working_Time_Set(uint32_t hh, uint8_t mm, uint8_t ss){
	working_time_step_counter = 0;
	working_time_hours = hh;
	working_time_minutes = mm;
	working_time_seconds = ss;	
}


void Working_Time_Get(uint32_t *hh, uint8_t *mm, uint8_t *ss){
	*hh = working_time_hours;
	*mm = working_time_minutes;
	*ss = working_time_seconds;
}


void Working_Time_Reset(void){
	working_time_step_counter = 0;
	working_time_hours = 0;
	working_time_minutes = 0;
	working_time_seconds = 0;	
}


void Calib_Time_Count_Update(void){
	
	if(Piezoelectric_Sensor_Read()){
		
		PORT_ACCEL_SENSING_LED |= (1 << ACCEL_SENSING_LED);
		
		calib_time_step_counter++;
		if(calib_time_step_counter >= HOROMETER_SECOND_N_STEPS){
			calib_time_step_counter = 0;
			calib_time_seconds++;
		}
	}else{
		PORT_ACCEL_SENSING_LED &= ~(1 << ACCEL_SENSING_LED);
	}
}


void Calib_Time_Reset(void){
	calib_time_step_counter = 0;
	calib_time_seconds = 0;
}

void Calib_Time_Get(uint32_t *ss){
	*ss = calib_time_seconds;
}




