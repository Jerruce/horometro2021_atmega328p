/*
 * psu_sw_manager.h
 *
 * Created: 14/06/2021 10:50:47
 *  Author: mtorres
 */ 


#ifndef PSU_SW_MANAGER_H_
#define PSU_SW_MANAGER_H_

/* File inclusion */
#include <avr/io.h>
#include <stdint.h>
#include "project_defines.h"

/* Function declaration */
void Enabling_Switches_Initialize(void);
void Piezoelectric_Circuit_PSU_On(void);
void Piezoelectric_Circuit_PSU_Off(void);
void Current_Sense_Circuit_PSU_On(void);
void Current_Sense_Circuit_PSU_Off(void);
void Magnetic_Pickup_Circuit_PSU_On(void);
void Magnetic_Pickup_Circuit_PSU_Off(void);
void Battery_Measure_Circuit_PSU_On(void);
void Battery_Measure_Circuit_PSU_Off(void);
void General_Power_Supply_Circuit_On(void);
void General_Power_Supply_Circuit_Off(void);
void ESP32_Microcontroller_PSU_On(void);
void ESP32_Microcontroller_PSU_Off(void);
void All_PSU_Switches_Off(void);


#endif /* PSU_SW_MANAGER_H_ */