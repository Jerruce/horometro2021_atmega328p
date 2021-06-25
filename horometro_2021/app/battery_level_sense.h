/*
 * battery_level_sense.h
 *
 * Created: 15/06/2021 14:50:31
 *  Author: mtorres
 */ 


#ifndef BATTERY_LEVEL_SENSE_H_
#define BATTERY_LEVEL_SENSE_H_

/* File inclusion */
#include <avr/io.h>
#include <stdint.h>
#include "project_defines.h"
#include <util/delay.h>
#include "global.h"
#include "MT_adc.h"

/* Function declaration */
void Battery_Level_Measure(void);
uint8_t Battery_Level_Get(void);

#endif /* BATTERY_LEVEL_SENSE_H_ */


