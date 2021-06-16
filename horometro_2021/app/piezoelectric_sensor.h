/*
 * MT_piezoelectric_sensor.h
 *
 * Created: 21/05/2021 12:07:09
 *  Author: mtorres
 */ 


#ifndef PIEZOELECTRIC_SENSOR_H_
#define PIEZOELECTRIC_SENSOR_H_

/* File inclusion */
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "project_defines.h"

/* Constant definition */
//#define HOROMETER_SECOND_N_STEPS		10
		
/* Function declaration */
void Piezoelectric_Sensor_Initialize(void);
uint8_t Piezoelectric_Sensor_Read(void);
void Working_Time_Count_Update(void);
void Working_Time_Set(uint32_t hh, uint8_t mm, uint8_t ss);
void Working_Time_Get(uint32_t *hh, uint8_t *mm, uint8_t *ss);
void Working_Time_Reset(void);
void Calib_Time_Count_Update(void);
void Calib_Time_Reset(void);
void Calib_Time_Get(uint32_t *ss);

#endif /* MT_PIEZOELECTRIC_SENSOR_H_ */