/*
 * psu_sw_manager.c
 *
 * Created: 14/06/2021 10:51:02
 *  Author: mtorres
 */ 


/* File inclusion */
#include "psu_sw_manager.h"


/* Function definition */


void Enabling_Switches_Initialize(void){
	
	DDR_SW_CURRENT	|= (1 << SW_CURRENT);
	DDR_SW_VBAT_MEASURE |= (1 << SW_VBAT_MEASURE);
	DDR_SW_ESP32 |= (1 << SW_ESP32);
	DDR_SW_SUPPLY |= (1 << SW_SUPPLY);
	DDR_SW_ACCEL |= (1 << SW_ACCEL);
	DDR_SW_PICKUP |= (1 << SW_PICKUP);
	
	All_PSU_Switches_Off();
	
}


void Piezoelectric_Circuit_PSU_On(void){
	PORT_SW_ACCEL |= (1 << SW_ACCEL);
}

void Piezoelectric_Circuit_PSU_Off(void){
	PORT_SW_ACCEL &= ~(1 << SW_ACCEL);
}

void Current_Sense_Circuit_PSU_On(void){
	PORT_SW_CURRENT |= (1 << SW_CURRENT); 
}

void Current_Sense_Circuit_PSU_Off(void){
	PORT_SW_CURRENT &= ~(1 << SW_CURRENT);
}

void Magnetic_Pickup_Circuit_PSU_On(void){
	PORT_SW_PICKUP |= (1 << SW_PICKUP);
}

void Magnetic_Pickup_Circuit_PSU_Off(void){
	PORT_SW_PICKUP &= ~(1 << SW_PICKUP);
}

void Battery_Measure_Circuit_PSU_On(void){
	PORT_SW_VBAT_MEASURE |= (1 << SW_VBAT_MEASURE);
}

void Battery_Measure_Circuit_PSU_Off(void){
	PORT_SW_VBAT_MEASURE &= ~(1 << SW_VBAT_MEASURE);
}

void General_Power_Supply_Circuit_On(void){
	PORT_SW_SUPPLY |= (1 << SW_SUPPLY);
}

void General_Power_Supply_Circuit_Off(void){
	PORT_SW_SUPPLY &= ~(1 << SW_SUPPLY);
}

void ESP32_Microcontroller_PSU_On(void){
	PORT_SW_ESP32 |= (1 << SW_ESP32);
}

void ESP32_Microcontroller_PSU_Off(void){
	PORT_SW_ESP32 &= ~(1 << SW_ESP32);
}

void All_PSU_Switches_Off(void){
	PORT_SW_CURRENT	&= ~(1 << SW_CURRENT);
	PORT_SW_VBAT_MEASURE &= ~(1 << SW_VBAT_MEASURE);
	PORT_SW_ESP32 &= ~(1 << SW_ESP32);
	PORT_SW_SUPPLY &= ~(1 << SW_SUPPLY);
	PORT_SW_ACCEL &= ~(1 << SW_ACCEL);
	PORT_SW_PICKUP &= ~(1 << SW_PICKUP);	
}