/*
 * battery_level_sense.c
 *
 * Created: 15/06/2021 14:50:13
 *  Author: mtorres
 */ 

/* File inclusion */
#include "battery_level_sense.h"

/* Variable definition */
static uint8_t battery_charge_level = BATTERY_LEVEL_HIGH;


/* Function definition */

void Battery_Level_Measure(void){
	
	uint16_t adc_bin_val;
	float	battery_volt;
	
	/* Select the proper channel */
	ADC_Seleccionar_Canal(VBAT_MEASURE_ADC_CHANNEL);
	/* Start conversion */
	ADCSRA |= (1 << ADSC);
	while(!(ADCSRA & (1 << ADIF))){
		/* Wait until the conversion ends */
	}
	/* Clean the ADC flag */
	ADCSRA |= (1 << ADIF);
	
	/* Measure voltage in the ADC input */
	adc_bin_val = ADC;
	battery_volt = BATTERY_VSENSE_GAIN	* ((adc_bin_val * ADC_REF_VOLTAGE) / 1024.0);
	
	/* Voltage boundaries (according to the battery datasheet */
	if(battery_volt > BATT_MAX_OUT_V){
		battery_volt = BATT_MAX_OUT_V;
	}else if(battery_volt < BATT_MIN_OUT_V){
		battery_volt = BATT_MIN_OUT_V;
	}else{
		//Does nothing
	}	

	/* Determine the battery charge level */
	if(battery_volt >= BATT_MED_TO_HIGH_LEVEL_VOLT_THRESHOLD){
		battery_charge_level = BATTERY_LEVEL_HIGH;
	}else if((battery_volt < BATT_HIGH_TO_MED_LEVEL_VOLT_THRESHOLD) && (battery_volt > BATT_LOW_TO_MED_LEVEL_VOLT_THRESHOLD)){
		battery_charge_level = BATTERY_LEVEL_MED;
	}else if((battery_volt < BATT_MED_TO_LOW_LEVEL_VOLT_THRESHOLD) && (battery_volt > BATT_CRITICAL_TO_LOW_LEVEL_VOLT_THRESHOLD)){
		battery_charge_level = BATTERY_LEVEL_LOW;
	}else if(battery_volt < BATT_LOW_TO_CRITICAL_LEVEL_VOLT_THRESHOLD){
		battery_charge_level = BATTERY_LEVEL_CRITICAL;
	}else{
		// Does nothing
	}
	
}


uint8_t Battery_Level_Get(void){
	return battery_charge_level;
}
