/*
 * low_power_horometer.h
 *
 * Created: 18/05/2021 08:51:32
 *  Author: mtorres
 */ 


#ifndef LOW_POWER_HOROMETER_H_
#define LOW_POWER_HOROMETER_H_

/* File inclusion */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <stdio.h>
#include <stdint.h>
#include "project_defines.h"
#include <util/delay.h>
#include "global.h"
#include "mt_debounce.h"
#include "MT_adc.h"
#include "MT_software_rtc.h"
#include "MT_magnetic_pickup.h"
#include "piezoelectric_sensor.h"
#include "current_sense.h"
#include "battery_level_sense.h"
#include "psu_sw_manager.h"
#include "esp32_comm_interface.h"

#include "system_test.h"

/* Function declaration */
void WDT_Off(void);
void System_Initialize(void);	
void LEDs_Initialize(void);
void Timer0_Initialize(void);
void Timer2_Initialize(void);
void ADC_Initialize(void);
void ADC_Disable(void);
void Analog_Comparator_Disable(void);
void Timer0_Interrupt_Enable(void);
void Timer0_Interrupt_Disable(void);

void System_Mode_Save(void);
void System_Mode_Load(void);

void System_Sequence(void);
void Vibration_Sense_Calibration_Sequence(void);
void Vibration_Sense_Only_Sequence(void);
void Vibration_Sense_Current_Sense_And_Motor_Speed_Sequence(void); 
void Wifi_Connection_Sequence(void); 
void Check_For_Alarm_Events(void);

uint8_t ESP32_Operation_Mode_Update(void);
uint8_t ESP32_Calibration_Screen_Display_Update(void);
uint8_t ESP32_Main_Screen_Display_Update(void);
uint8_t ESP32_Alarm_Screen_Display_Update(void);


#endif /* LOW_POWER_HOROMETER_H_ */