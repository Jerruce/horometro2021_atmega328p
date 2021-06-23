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
#include "global.h"
		
/* Function declaration */

void Piezoelectric_Sensor_Initialize(void);
uint8_t Piezoelectric_Sensor_Read(void);

void Calib_Time_Count_Update(void);
void Calib_Time_Reset(void);
uint32_t Calib_Time_Get(void);

void Working_Time_Count_Update(void);
void Working_Time_Set(uint32_t hh, uint8_t mm, uint8_t ss);
void Working_Time_Get(uint32_t *hh, uint8_t *mm, uint8_t *ss);
void Working_Time_Reset(void);

void Alarm1_Time_Count_Update(void);
void Alarm1_Time_Set(uint32_t hh, uint8_t mm, uint8_t ss);
void Alarm1_Working_Time_Get(uint32_t *hh, uint8_t *mm, uint8_t *ss);
void Alarm1_Time_Reset(void);
void Alarm1_Setpoint_Set(uint32_t sp);
uint32_t Alarm1_Setpoint_Get(void);

void Alarm2_Time_Count_Update(void);
void Alarm2_Time_Set(uint32_t hh, uint8_t mm, uint8_t ss);
void Alarm2_Working_Time_Get(uint32_t *hh, uint8_t *mm, uint8_t *ss);
void Alarm2_Time_Reset(void);
void Alarm2_Setpoint_Set(uint32_t sp);
uint32_t Alarm2_Setpoint_Get(void);

void Alarm3_Time_Count_Update(void);
void Alarm3_Time_Set(uint32_t hh, uint8_t mm, uint8_t ss);
void Alarm3_Working_Time_Get(uint32_t *hh, uint8_t *mm, uint8_t *ss);
void Alarm3_Time_Reset(void);
void Alarm3_Setpoint_Set(uint32_t sp);
uint32_t Alarm3_Setpoint_Get(void);

#endif /* MT_PIEZOELECTRIC_SENSOR_H_ */