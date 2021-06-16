/*
 * MT_current_sense.h
 *
 * Created: 21/05/2021 18:41:16
 *  Author: mtorres
 */ 


#ifndef CURRENT_SENSE_H_
#define CURRENT_SENSE_H_

/* File inclusion */
#include <avr/io.h>
#include <stdint.h>
#include "project_defines.h"
#include "MT_adc.h"

/* Function declaration */
void Current_Measure(void);
void RMS_Current_Calculate(void);
void Peak_Current_Reset(void);

float Instant_Current_Get(void);
float Peak_Current_Get(void);
float RMS_Current_Get(void);

#endif /* MT_CURRENT_SENSE_H_ */